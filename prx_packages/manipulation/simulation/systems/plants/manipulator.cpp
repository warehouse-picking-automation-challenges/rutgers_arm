
/**
 * @file manipulator.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Rahul Shome, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "simulation/systems/plants/manipulator.hpp"

#include "prx/simulation/collision_checking/collision_checker.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"

#include <boost/range/adaptor/map.hpp>
#include <urdf/model.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <sys/param.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include "simulation/prx_chainiksolvervel_pinv_nso.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

#ifdef USE_TRAC_IK
#include <trac_ik/trac_ik.hpp>
#endif

#define IK_ITERATIONS 70
namespace prx
{
    namespace packages
    {
        using namespace util;
        using namespace sim;

        namespace manipulation
        {

            manipulator_t::manipulator_t() : sim::integration_plant_t()
            {
                char* w = std::getenv("PRACSYS_PATH");
                std::string filename(w);
                pracsys_path = filename;
                GRASP_STEP = 1/simulation::simulation_step;

                jacobian_error_code = UNKNOWN_FAILURE;

                ik_steer_failed =0;
                ik_steer_succeeded =0;
                ik_steer_fail_time =0;
                ik_steer_succeed_time =0;
                iter_failures = 0;
                svd_failures = 0;
                rank_failures = 0;
                progress_failures = 0;
                bound_failures = 0;
                singularity_failures = 0;
                num_failures = 0;
            }
            
            manipulator_t::~manipulator_t()
            {
                state_space->free_point(inter_st);
                
                foreach( prx_chainiksolvervel_pinv_nso* solver, chain_solver_map | boost::adaptors::map_values )
                {
                    delete solver;
                }
            }

            void manipulator_t::init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader)
            {
                // sim::kinematic_plant_t::init(reader,template_reader);                

                MAX_IK_STEP = parameters::get_attribute_as<double>("max_ik_step", reader, template_reader, 0.01);
                ik_timeout = parameters::get_attribute_as<double>("ik_timeout", reader, template_reader, 0.005);
                config_t root_config;
                parameters::initialize<config_t > (&root_config,reader,"root_frame",template_reader,"root_frame");
                double qx,qy,qz,qw;
                double x,y,z;
                root_config.get_orientation().get(qx,qy,qz,qw);
                root_config.get_position(x,y,z);              
                root_frame = KDL::Frame(KDL::Rotation::Quaternion(qx,qy,qz,qw),KDL::Vector(x, y, z));
        
                if( parameters::has_attribute( "urdf_file", reader, template_reader ) )
                {
                    urdf_file_name = parameters::get_attribute( "urdf_file", reader, template_reader);
                }
                else
                {
                    PRX_FATAL_S("Manipulator_t needs a urdf file!");
                }
                if ( parameters::has_attribute("input_path", reader, template_reader))
                {
                    std::string temp_path = parameters::get_attribute("input_path", reader, template_reader);
                    input_path = pracsys_path + temp_path;
                }

                urdf_file_name = input_path +"/"+ urdf_file_name;                
                kdl_tree = new KDL::Tree();
                if (!kdl_parser::treeFromFile(urdf_file_name.c_str(), *kdl_tree))
                  PRX_FATAL_S("Failed to construct kdl tree");
                
                KDL::SegmentMap::const_iterator root_iter = kdl_tree->getRootSegment();
                populate_config_names(root_iter);

                foreach(std::string& s, tree_ordered_simple_names) 
                {
                    if(parameters::has_attribute("ignored_dofs/"+s,reader,template_reader))
                    {
                        ignored_dofs.push_back(std::pair<std::string,double>());
                        ignored_dofs.back().first = s;
                        ignored_dofs.back().second = parameters::get_attribute_as<double>("ignored_dofs/"+s, reader, template_reader);
                    }
                    if(parameters::has_attribute("ignored_geoms/"+s,reader,template_reader))
                    {
                        ignored_geoms.push_back(s);
                    }
                }   

                //Alright first let's get a zero vector for every body that has a name
                for(unsigned i=0;i<tree_ordered_simple_names.size();i++)
                {
                    config_offsets.push_back( KDL::Frame() );
                }           
                parameter_reader_t::reader_map_t subsystem_map = parameters::get_map("config_offsets", reader, template_reader);
                foreach(const parameter_reader_t::reader_map_t::value_type key_value, subsystem_map)
                {
                    std::vector<double> offset = key_value.second->get_attribute_as<std::vector<double> >("offset");
                    KDL::Frame tmpVec = KDL::Frame(KDL::Vector(offset[0],offset[1],offset[2]));
                    //Kinda dumb, but let's just search the body
                    for( unsigned i=0; i<tree_ordered_simple_names.size(); ++i )
                    {
                        if( key_value.first == tree_ordered_simple_names[i] )
                        {
                            config_offsets[i] = tmpVec;
                        }
                    }
                }      

                std::vector<unsigned> blank_vec;
                unsigned state_size=0;
                manip_tree = new manip_node_t();
                manip_tree->kdl_node = root_iter;
                populate_state_vectors(root_iter,manip_tree,blank_vec); 
                for(unsigned i=0;i<state_memory.size();i++)
                {
                    control_memory.push_back(new double);
                }

                //read the chain_map entries from input 
                parameter_reader_t::reader_map_t chain_readers = parameters::get_map("chains", reader, template_reader);

                foreach(const parameter_reader_t::reader_map_t::value_type key_value, chain_readers)
                {
                    std::string start_link = key_value.second->get_attribute("start_link");
                    std::string end_link = key_value.second->get_attribute("end_link");
                    std::vector<unsigned>* new_vec = get_state_indices(start_link,end_link);
                    chain_map[start_link+"_"+end_link] = new_vec;
                } 


                //We need to do the population of the state_memory and control_memory before we call create_spaces.
                std::vector<std::string> ee_names;
                if(parameters::has_attribute("end_effectors",reader,template_reader))
                {
                    foreach(const parameter_reader_t* ee_reader, parameters::get_list("end_effectors",reader,template_reader))
                    {
                        ee_names.push_back(ee_reader->get_attribute("name"));
                        config_t config;
                        ee_reader->initialize<config_t > (&config,"local_config");
                        end_effector_local_configs.push_back(config);
                        end_effector_states.push_back(1);
                        end_effector_controls.push_back(0);
                    }

                    for(unsigned i=0; i<ee_names.size(); ++i)
                    {
                        state_memory.push_back(&end_effector_states[i]);
                        control_memory.push_back(&end_effector_controls[i]);
                    }
                } 
                else
                    PRX_FATAL_S("A manipulator needs at least one end effector. Please specify this in the input!");



                grasped_ignored.resize(ee_names.size());

                create_spaces();
                inter_st = state_space->alloc_point();                
                integration_plant_t::init(reader,template_reader); 

                //We have to do the effector_names after the initialization of the plant. 
                //We don't have the pathname before the initialization. 
                foreach(std::string name, ee_names)
                {
                    effector_names.push_back(pathname + "/" + name);
                    end_effector_geometries.push_back(&geometries[effector_names.back()]);
                }

                state_space->zero();
                

                for (auto ee_name : ee_names)
                {
                    config_t ee_config;
                    FK_solver(ee_config, ee_name);
                    PRX_PRINT ("EE CONFIG: " << ee_config << " FOR EE: " << ee_name, PRX_TEXT_MAGENTA);
                    relative_base_to_end_effector_configs[ee_name] = ee_config;
                }


                if( parameters::has_attribute("initial_state", reader, template_reader) ) //   reader->has_element("initial_state") )
                {
                    state_space->set_from_vector(parameters::get_attribute_as< std::vector<double> > ("initial_state", reader, template_reader));
                }
                // config_t test_config;
                // parameters::initialize<config_t > (&test_config,reader,"test_config",template_reader,"test_config");
                // state_t* result_state = state_space->alloc_point();
                // FK_solver(test_config,ee_names[0]);
                // PRX_DEBUG_COLOR("Root Geom: "<<kdl_tree->getRootSegment()->first<<" : " <<  test_config,PRX_TEXT_RED);
                // if(!IK_solver(result_state, result_state, test_config, kdl_tree->getRootSegment()->first, ee_names[0]))
                //     PRX_DEBUG_COLOR("No IK", PRX_TEXT_RED);
                // state_space->copy_from_point(result_state);
                
            }

            void manipulator_t::get_end_effector_configuration_relative_to_base_link(util::config_t& config, const std::string& ee_name)
            {
                if (relative_base_to_end_effector_configs.find(ee_name) != relative_base_to_end_effector_configs.end())
                {
                    config = relative_base_to_end_effector_configs[ee_name];
                }
            }


            void manipulator_t::update_phys_configs(config_list_t& configs, unsigned& index) const
            {
                unsigned name_index=0;
                update_phys_configs(configs,index,manip_tree,root_frame,name_index);
            }

            void manipulator_t::link_collision_info(collision_checker_t* collision_checker)
            {
                collision_infos.resize(tree_ordered_config_names.size());
                for(unsigned i=0;i<collision_infos.size();i++)
                    collision_infos[i].first = NULL;
                int i =0;
                for( unsigned i = 0; i < tree_ordered_config_names.size(); i++ )
                {
                    collision_infos[i].first = collision_checker->get_collision_info(tree_ordered_config_names[i]);
                    PRX_DEBUG_S(collision_infos[i].first<<" "<<tree_ordered_simple_names[i]);
                }
            }

            void manipulator_t::update_collision_info()
            {
                unsigned name_index=0;
                update_collision_info(manip_tree,root_frame,name_index);
                for(unsigned i=0;i<collision_infos.size();i++)
                {
                    if(collision_infos[i].first!=NULL)
                        collision_infos[i].first->update_matrices(collision_infos[i].second);
                }         
            }

            geometry_t* manipulator_t::get_end_effector_geometry(int index)
            {
                return end_effector_geometries[index];
            }

            geometry_t* manipulator_t::get_end_effector_geometry(std::string end_effector_pathname)
            {
                return &geometries[end_effector_pathname];
            }

            void manipulator_t::get_end_effector_geometries( std::vector<geometry_t*>& ee_geoms )
            {
                ee_geoms = end_effector_geometries;
            }

            void manipulator_t::steering_function(const state_t* start, const state_t* goal, plan_t& result_plan)
            {
                //each joint is independent
                double max_time = 0;
                for(unsigned i=0;i<state_space->get_dimension()-end_effector_states.size(); i++)
                {
                    double dist = goal->memory[i] - start->memory[i];
                    double vel = input_control_space->get_bounds()[i]->get_upper_bound();
                    double test_time = fabs(dist)/vel;
                    if(vel <= 0.000000001)
                        test_time = 0;
                    if(test_time>max_time)
                        max_time = test_time;
                }
                double steps = std::ceil(max_time / simulation::simulation_step);
                max_time = steps * simulation::simulation_step;
                result_plan.append_onto_back(max_time);
                std::vector<double> new_control;
                for(unsigned i=0;i<state_space->get_dimension()-end_effector_states.size(); i++)
                {
                    double dist = goal->memory[i] - start->memory[i];
                    new_control.push_back(dist/max_time);
                }
                for(unsigned i=0;i<end_effector_states.size(); ++i)
                {
                    new_control.push_back(0);
                }
#ifndef NDEBUG                
                // AK: We need to make sure that the goal state has the same end_effector state as the start state
                for(unsigned i=state_space->get_dimension()-end_effector_states.size();i<state_space->get_dimension(); i++)
                {
                    if( start->memory[i] != goal->memory[i] )
                    {
                        PRX_PRINT( i<<" start state :: " << state_space->print_point( start, 3 ), PRX_TEXT_BROWN );
                        PRX_PRINT( i<<" goal  state :: " << state_space->print_point( goal, 3 ), PRX_TEXT_BROWN );
                        // PRX_FATAL_S( " Steering error: start grasping state different than the goal one" );
                        PRX_INFO_S( " Steering error: start grasping state different than the goal one" );
                    }
                }
#endif
                input_control_space->set_from_vector(new_control, result_plan.back().control);
            }
            
            std::string manipulator_t::get_end_effector_name(int index) const
            {
                return effector_names[index];
            }

            int manipulator_t::get_end_effector_index(std::string end_effector_pathname)
            {
                for(int i=0; i<effector_names.size(); ++i)
                {
                    if(effector_names[i] == end_effector_pathname)
                        return i;
                }
                return -1;
            }

            const std::vector<std::string>& manipulator_t::get_end_effector_names() const
            {
                return effector_names;
            }

            void manipulator_t::get_end_effector_names(std::vector<std::string>& names)
            {
                names.insert(names.end(), effector_names.begin(), effector_names.end());
            }

            std::vector<std::string> manipulator_t::get_ignored_grasping_bodies(int index)
            {
                return grasped_ignored[index];
            }

            bool manipulator_t::is_end_effector_closed(int index) const
            {
                return (end_effector_states[index] == 2);
            }


            void manipulator_t::FK_solver( config_t& goal_config, std::string link_name)
            {
                //TODO: Find out if we can make this faster by not creating chains: ZL
                KDL::Chain chain1;
                kdl_tree->getChain(kdl_tree->getRootSegment()->first, link_name, chain1);                
                KDL::JntArray q(chain1.getNrOfJoints());               

                std::vector< double > state_var;
                state_space->copy_to_vector( state_var );

                std::vector<unsigned>* indices = chain_map[kdl_tree->getRootSegment()->first+"_"+link_name];

                for(unsigned i = 0; i<indices->size(); ++i)
                {
                    q(i) = state_var[indices->at(i)];
                }

                KDL::ChainFkSolverPos_recursive fk_solver(chain1);
                KDL::Frame F;
                fk_solver.JntToCart(q,F); 
                F = root_frame*F;

                goal_config.set_position(F.p.x(),F.p.y(),F.p.z());
                double qx,qy,qz,qw;
                convert_to_quaternion(F.M,qx,qy,qz,qw);
                goal_config.set_orientation(qx,qy,qz,qw);
                goal_config.normalize_orientation();
            }

            bool manipulator_t::IK_solver( space_point_t* result_state, const space_point_t* start_state, const config_t& goal_config, std::string start_link, std::string end_link)
            {
                state_space->copy_from_point(start_state);

                double qx,qy,qz,qw;
                double x,y,z;
                goal_config.get_orientation().get(qx,qy,qz,qw);
                goal_config.get_position(x,y,z);
                KDL::Chain chain1;
                kdl_tree->getChain(start_link, end_link, chain1);                
                KDL::JntArray q(chain1.getNrOfJoints());
                KDL::JntArray q_out(chain1.getNrOfJoints());
                KDL::JntArray q_min(chain1.getNrOfJoints());
                KDL::JntArray q_max(chain1.getNrOfJoints());                

                std::vector< double > state_var;
                state_space->copy_point_to_vector( start_state, state_var );

                std::vector<unsigned>* indices = chain_map[start_link+"_"+end_link];

                unsigned index;
                for(unsigned i = 0; i<indices->size(); ++i)
                {
                    index = indices->at(i);
                    q(i) = state_var[index];
                    q_min(i) = state_space->get_bounds()[index]->get_lower_bound();
                    q_max(i) = state_space->get_bounds()[index]->get_upper_bound();
                }

            // #ifdef USE_TRAC_IK
                TRAC_IK::TRAC_IK ik_solver(chain1,q_min,q_max,ik_timeout,1e-6);
            // #else
                // KDL::ChainFkSolverPos_recursive fk_solver(chain1);
                // KDL::ChainIkSolverVel_pinv ik_solver_vel(chain1);
                // KDL::ChainIkSolverPos_NR_JL ik_solver(chain1,q_min,q_max,fk_solver,ik_solver_vel,IK_ITERATIONS,1e-6);
            // #endif

                //We subtract the offset of the base to bring the root frame back to 0,0,0 before calling IK.
                KDL::Frame F(KDL::Rotation::Quaternion(qx,qy,qz,qw),KDL::Vector(x, y, z));
                F = root_frame.Inverse()*F;
                
                bool ik_res = (ik_solver.CartToJnt(q,F,q_out)>=0);

                int count=0;
                while(!ik_res && count<0)
                {
                    for(unsigned i=0 ; i< indices->size() ; i++)
                    {
                        q(i) = uniform_random(q_min(i),q_max(i));
                    }
                    count++;
                    ik_res = (ik_solver.CartToJnt(q,F,q_out)>=0);
                }
                
                if(ik_res)
                {
                    std::vector<double> state_vec = state_var;

                    for(int i = 0; i<indices->size(); ++i)
                    {
                        state_vec[indices->at(i)] = q_out(i);
                    }

                    state_space->copy_vector_to_point( state_vec, result_state );
                }
                
                return ik_res;                
            }

            bool manipulator_t::jac_steering( sim::plan_t& result_plan, workspace_trajectory_t& ee_trajectory, util::space_point_t* result_state, const util::space_point_t* start_state, const util::config_t& goal_config, std::string start_link, std::string end_link )
            {
                bool completed = false;
                state_t* cloned_start = state_space->clone_point(start_state);
                KDL::Chain chain1;
                kdl_tree->getChain(start_link, end_link, chain1); 
                
                std::string map_index = start_link+"_"+end_link;
                std::vector<unsigned>* indices = chain_map[map_index];
                
                hash_t<std::string, prx_chainiksolvervel_pinv_nso* >::iterator it = chain_solver_map.find( map_index );
                prx_chainiksolvervel_pinv_nso* solver_pointer;
                ///////////////////////////////////////for target positions
                // IK_solver(result_state,start_state, goal_config, start_link,end_link);
                
                //If we did not find this chain solver
                if( it == chain_solver_map.end() )
                {
                    KDL::JntArray opt_pos(chain1.getNrOfJoints());
                    KDL::JntArray weights(chain1.getNrOfJoints());
                    for(unsigned i = 0; i<chain1.getNrOfJoints(); ++i)
                    {
                        double low,high;
                        state_space->get_bounds()[indices->at(i)]->get_bounds(low,high);
                        opt_pos(i) = cloned_start->at(indices->at(i));//(high+low)/2.0;//result_state->at(indices->at(i));
                        weights(i) = 4.0/(3.0*(high-low)*(high-low));
                    }
                    solver_pointer = new prx_chainiksolvervel_pinv_nso(chain1,opt_pos,weights);
                    solver_pointer->setAlpha(.25);
                    chain_solver_map[ map_index ] = solver_pointer;
                }
                //Otherwise, use that solver
                else
                {
                    solver_pointer = it->second;
                }
                prx_chainiksolvervel_pinv_nso& ik_solver_vel = *solver_pointer;
                
                /////////////////////////////////////////////////
                ///////////////////////////////////////for regular
                // KDL::ChainIkSolverVel_pinv ik_solver_vel(chain1,0.00001,1000);
                // Eigen::MatrixXd weights(chain1.getNrOfJoints(),chain1.getNrOfJoints());
                // for(unsigned i = 0; i<chain1.getNrOfJoints(); ++i)
                // {
                //     weights(i,i) = i+1;
                // }
                // KDL::ChainIkSolverVel_wdls ik_solver_vel(chain1);
                // ik_solver_vel.setWeightJS(weights);
                //////////////////////////////////////////////////
                KDL::JntArray q_in(chain1.getNrOfJoints());
                KDL::JntArray qdot_out(chain1.getNrOfJoints());
                state_space->copy_from_point(cloned_start);
                state_space->copy_to_point(result_state);                    
                config_t start_config;
                KDL::Frame start_frame;
                KDL::Frame end_frame;
                double qx,qy,qz,qw,x,y,z;
                goal_config.get_position(x,y,z);
                goal_config.get_orientation().get(qx,qy,qz,qw);
                end_frame.p = KDL::Vector(x,y,z);
                end_frame.M = KDL::Rotation::Quaternion(qx,qy,qz,qw);



                state_space->copy_from_point(result_state);
                double config_dist = sqrt( (start_config.get_position()[0] - goal_config.get_position()[0] ) * (start_config.get_position()[0] - goal_config.get_position()[0] ) + 
                                            (start_config.get_position()[1] - goal_config.get_position()[1] ) * (start_config.get_position()[1] - goal_config.get_position()[1] ) + 
                                            (start_config.get_position()[2] - goal_config.get_position()[2] ) * (start_config.get_position()[2] - goal_config.get_position()[2] ))
                                    + goal_config.get_orientation().distance(start_config.get_orientation());

                bool not_done = true;
                result_plan.clear();
                int num_iters = 0;
                int max_iters = 700*config_dist;
                int limit_frames = 0;
                bool svd_failure = false;
                bool progress_failure = false;
                bool bound_failed = false;
                bool singularity_failure = false;
                bool rank_failure = false;

                KDL::Twist prev_twist = KDL::Twist::Zero();
                KDL::Twist second_prev_twist = KDL::Twist::Zero();
                KDL::Twist third_prev_twist = KDL::Twist::Zero();
                
                ee_trajectory.clear();
                while(not_done && num_iters<max_iters)
                {
                    num_iters++;
                    FK_solver(start_config, end_link);
                    start_config.get_position(x,y,z);
                    start_config.get_orientation().get(qx,qy,qz,qw);
                    
                    ee_trajectory.copy_onto_back( start_config );
                    
                    start_frame.p = KDL::Vector(x,y,z);
                    start_frame.M = KDL::Rotation::Quaternion(qx,qy,qz,qw);
                    KDL::Twist twist = KDL::diff(start_frame, end_frame,1);
                    KDL::Twist twist2 = KDL::Twist::Zero();
                    
                    //Attempt some automatic detection of a singularity (simple oscillation)
                    singularity_failure = ( KDL::Equal(twist,second_prev_twist,1e-6) && KDL::Equal(prev_twist,third_prev_twist,1e-6) );
                    
                    if(KDL::Equal(twist2, twist, 1e-5))
                    {
                        completed = true;
                        not_done = false;
                        continue;
                    }
                    //DEBUG, let's try only counting my singularities as failures
                    else if( singularity_failure )
                    // else if( KDL::Equal(twist, prev_twist, 1e-6) || singularity_failure )
                    {
                        not_done = false;

                        progress_failure = true;

                        for(unsigned i = 0; i<chain1.getNrOfJoints(); ++i)
                        {
                            double low,high;
                            state_space->get_bounds()[indices->at(i)]->get_bounds(low,high);

                            if(fabs(result_state->at(indices->at(i)) - low) < PRX_ZERO_CHECK )
                            {
                                bound_failed = true;
                            }
                            else if(fabs(result_state->at(indices->at(i)) - high) < PRX_ZERO_CHECK )
                            {
                                bound_failed = true;
                            }
                        } 
                        
                        continue;
                    }
                    third_prev_twist = second_prev_twist;
                    second_prev_twist = prev_twist;
                    prev_twist = twist;
                    unsigned index;
                    for(unsigned i = 0; i<indices->size(); ++i)
                    {
                        index = indices->at(i);
                        q_in(i) = result_state->at(index);
                    }

                    // ===== IK CALL =====
                    // unsigned error_code = ik_solver_vel.CartToJntClamping( q_in, state_space->get_bounds(), twist, qdot_out );
                    // unsigned error_code = ik_solver_vel.CartToJnt_continuous( q_in, state_space->get_bounds(), twist, qdot_out );
                    unsigned error_code = ik_solver_vel.CartToJnt( q_in, twist, qdot_out );
                    if( error_code >= 0 )
                    {
                        //Let's keep track of whether or not H's rank is getting too large
                        if( error_code == SOLVER_ERR_LARGE_H_RANK )
                        {
                            ++limit_frames;
                        }
                        else
                        {
                            limit_frames = 0;
                        }
                        //If we've gone on too long at the limits
                        if( limit_frames == 300 )
                        {
                            //We should abort out due to a rank failure
                            not_done = false;
                            rank_failure = true;
                        }
                        
                        double max_time_scaling=0;
                        // scale the velocities to find the actual velocities
                        for(unsigned i = 0; i<indices->size(); ++i)
                        {
                            index = indices->at(i);
                            double time_scaling = fabs(qdot_out(i))/input_control_space->get_bounds()[index]->get_upper_bound();
                            if(time_scaling>max_time_scaling)
                                max_time_scaling = time_scaling;
                        }
                        max_time_scaling = simulation::simulation_step * std::ceil(max_time_scaling / simulation::simulation_step);
                        // if(max_time_scaling<1)
                        //     max_time_scaling = 1;


                        result_plan.append_onto_back(simulation::simulation_step);
                        input_control_space->zero();
                        for(unsigned i = 0; i<indices->size(); ++i)
                        {
                            index = indices->at(i);
                            result_plan.back().control->at(index) = qdot_out(i)/max_time_scaling;
                            (*(control_memory[index])) = result_plan.back().control->at(index);
                        }
                        //simulate the control_input
                        this->propagate(simulation::simulation_step);
                        state_space->copy_to_point(result_state);
                    }
                    else
                    {
                        not_done = false;
                        svd_failure = true;
                    }
                }
                // PRX_PRINT("Jacobian Time [" << ik_solver_vel.measure_jac()/((double)num_iters) << "]", PRX_TEXT_CYAN);
                // PRX_PRINT("SVD Time [" << ik_solver_vel.measure_svd()/((double)num_iters) << "]", PRX_TEXT_LIGHTGRAY);
                if(!completed)
                {
                    // PRX_PRINT("Failure #[" << num_iters << "]", PRX_TEXT_RED);
                    // Now, we can return partial plans from this process so don't clear.
                    // result_plan.clear();
                    jacobian_error_code = UNKNOWN_FAILURE;
                    
                    num_failures++;
                    if(num_iters==max_iters)
                    {
                        iter_failures++;
                        jacobian_error_code = ITERATION_FAILURE;
                    }
                    if(svd_failure)
                        svd_failures++;
                    if(progress_failure)
                    {
                        progress_failures++;
                        jacobian_error_code = PROGRESS_FAILURE;
                    }
                    if(rank_failure)
                    {
                        rank_failures++;
                        jacobian_error_code = RANK_FAILURE;
                    }
                    if(bound_failed)
                        bound_failures++;
                    if(singularity_failure)
                    {
                        singularity_failures++;
                        jacobian_error_code = SINGULARITY_FAILURE;
                    }
                    if(num_failures%100==0)
                    {
                        // PRX_INFO_S("\nIteration Failures ["<<iter_failures<<"/"<<num_failures<<"]\nSVD Failures ["<<svd_failures<<"/"<<num_failures<<"]\nRank Failures ["<<rank_failures<<"/"<<num_failures<<"]\nProgress Failures ["<<progress_failures<<"/"<<num_failures << "]\n -> Bounds Failures [" << bound_failures << "/" << progress_failures << "]\n -> Singularities [" << singularity_failures << "/" << progress_failures << "]");
                    }
                }
                else
                {
                    // PRX_PRINT("Success! #[" << num_iters << "]", PRX_TEXT_GREEN);
                }
                state_space->free_point(cloned_start);
                return completed;
            }

            unsigned manipulator_t::get_jac_steer_error_type()
            {
                return jacobian_error_code;
            }

            void manipulator_t::get_end_effector_configuration(util::config_t& config, int end_effector_index)
            {
                config = end_effector_local_configs[end_effector_index];
            }

            void manipulator_t::update_derivative(state_t * const result)
            {
                //update state 
                for(unsigned index=0;index<state_space->get_dimension();index++)
                    result->memory[index] = *control_memory[index];
                int i=0;
                for(unsigned index=state_space->get_dimension()- end_effector_controls.size();index<state_space->get_dimension();index++)
                {
                    if(end_effector_controls[i]!=0)
                    {
                        //PRX_DEBUG_COLOR("grasping id: " << index << "   control: " << end_effector_controls[i] << "     state memory: " << *state_memory[index], PRX_TEXT_CYAN);
                        result->memory[index] = round(end_effector_controls[i] - *state_memory[index])*GRASP_STEP;
                    }
                    else
                        result->memory[index] = 0;
                    
                    i++;
                }
            }

            void manipulator_t::update_phys_configs(config_list_t& configs, unsigned& index, 
                                                    manip_node_t* node, const KDL::Frame& parent_frame, 
                                                    unsigned& name_index) const
            {
                //get the segment from the tree
                KDL::Segment seg = GetTreeElementSegment(node->kdl_node->second);
                KDL::Frame this_frame;

                //if this is a controllable joint
                if(node->memory!=NULL)
                {
                    this_frame = parent_frame*seg.pose(*node->memory);
                }
                //if this is a fixed joint, we just need the configuration of the segment
                else
                {
                    this_frame = parent_frame*seg.pose(0.0);
                }

                if(node->physical_geoms)
                {
                    KDL::Frame phys_frame = this_frame*config_offsets[name_index];
                    //make sure there is room in the list
                    augment_config_list(configs,index);
                    //assign the correct name for this config
                    configs[index].first = tree_ordered_config_names[name_index];
                    //set the configuration
                    configs[index].second.set_position(phys_frame.p.x(),phys_frame.p.y(),phys_frame.p.z());
                    double qx,qy,qz,qw;
                    convert_to_quaternion(phys_frame.M,qx,qy,qz,qw);
                    configs[index].second.set_orientation(qx,qy,qz,qw);
                    index++;
                }
                name_index++;

                //recursively call everything
                foreach(manip_node_t* child, node->children )
                {
                    update_phys_configs(configs,index,child,this_frame,name_index);
                }
            }

            void manipulator_t::update_collision_info(manip_node_t* node, KDL::Frame& parent_frame,unsigned& name_index)
            {
                //get the segment from the tree
                KDL::Segment seg = GetTreeElementSegment(node->kdl_node->second);
                KDL::Frame this_frame;
                //if this is a controllable joint
                if(node->memory!=NULL)
                {
                    this_frame = parent_frame*seg.pose(*node->memory);
                }
                //if this is a fixed joint, we just need the configuration of the segment
                else
                {
                    this_frame = parent_frame*seg.pose(0.0);
                }

                if(node->physical_geoms)
                {
                    KDL::Frame phys_frame = this_frame*config_offsets[name_index];
                    //set the configuration
                    collision_infos[name_index].second.set_position(phys_frame.p.x(),phys_frame.p.y(),phys_frame.p.z());
                    double qx,qy,qz,qw;
                    convert_to_quaternion(phys_frame.M,qx,qy,qz,qw);
                    collision_infos[name_index].second.set_orientation(qx,qy,qz,qw);
                }
                name_index++;

                //recursively call everything
                foreach(manip_node_t* child, node->children )
                {
                    update_collision_info(child,this_frame,name_index);
                }
            }

            void manipulator_t::populate_config_names(KDL::SegmentMap::const_iterator iter)
            {
                tree_ordered_config_names.push_back(pathname + "/" + iter->first);
                tree_ordered_simple_names.push_back(iter->first);
                foreach(KDL::SegmentMap::const_iterator child, GetTreeElementChildren(iter->second) )
                {
                    populate_config_names(child);
                }
            }

            void manipulator_t::populate_state_vectors(KDL::SegmentMap::const_iterator iter, manip_node_t* node, std::vector<unsigned> state_indices)
            {
                //if this is a controllable joint
                KDL::Segment seg = GetTreeElementSegment(iter->second);
                if(seg.getJoint().getType()!=KDL::Joint::None)
                {
                    node->memory = new double;
                    bool found_ignored=false;
                    for (std::vector<std::pair<std::string, double> >::iterator i = ignored_dofs.begin(); i != ignored_dofs.end() && !found_ignored; ++i)
                    {
                        if(iter->first==i->first)
                        {
                            PRX_DEBUG_COLOR("IGN: " << iter->first, PRX_TEXT_MAGENTA );
                            found_ignored = true;
                            *node->memory = i->second;
                        }
                    }
                    if(!found_ignored)
                    {
                        PRX_DEBUG_COLOR( iter->first, PRX_TEXT_CYAN );
                        state_memory.push_back(node->memory);
                        state_indices.push_back(state_memory.size()-1);
                        node->state_index = state_memory.size()-1;
                    }
                }

                for (unsigned i=0;i<ignored_geoms.size();i++)
                {
                    if(iter->first==ignored_geoms[i])
                    {
                        node->physical_geoms = false;
                    }
                }
                
                std::vector<unsigned>* new_vec = new std::vector<unsigned>();
                *new_vec = state_indices;
                chain_map[kdl_tree->getRootSegment()->first+"_"+iter->first] = new_vec;

                foreach(KDL::SegmentMap::const_iterator child, GetTreeElementChildren(iter->second) )
                {
                    manip_node_t* new_node = new manip_node_t();
                    new_node->kdl_node = child;
                    new_node->parent = node;
                    node->children.push_back(new_node);
                    populate_state_vectors(child,new_node,state_indices);
                }
            }

            std::vector<unsigned>* manipulator_t::get_state_indices(std::string start_link, std::string end_link)
            {
                std::deque<manip_node_t*> to_explore;
                to_explore.push_back(manip_tree);
                manip_node_t* end_node=NULL;
                while(to_explore.size()!=0)
                {
                    manip_node_t* examined_node = to_explore.front();
                    to_explore.pop_front();
                    if(examined_node->kdl_node->first==end_link)
                    {
                        end_node = examined_node;
                        to_explore.clear();
                    }
                    else
                    {
                        foreach(manip_node_t* child, examined_node->children)
                        {
                            to_explore.push_back(child);
                        }
                    }
                }

                std::vector<unsigned>* indices = new std::vector<unsigned>();
                while(end_node->kdl_node->first!=start_link)
                {
                    if(end_node->memory!=NULL)
                    {
                        indices->push_back(end_node->state_index);
                    }
                    end_node = end_node->parent;
                }
                if(end_node->memory!=NULL)
                {
                    indices->push_back(end_node->state_index);
                }
                std::reverse(indices->begin(),indices->end());
                return indices;
            }

            void manipulator_t::scale_control_bounds(double scaling_factor)
            {
                input_control_space->scale_bounds(scaling_factor);
            }

            void manipulator_t::convert_to_quaternion(KDL::Rotation& rot,double& x,double& y,double& z, double& w) const
            {
                double trace = rot(0,0) + rot(1,1) + rot(2,2) + 1.0;
                //This epsilon potentially causes instability in case of rotation matrices that are not perfectly formed
                double epsilon=1E-3;
                if( trace > epsilon )
                {
                    // PRX_PRINT("trace>epsilon"<<trace, PRX_TEXT_RED);
                    double s = 0.5 / sqrt(trace);
                    w = 0.25 / s;
                    x = ( rot(2,1) - rot(1,2) ) * s;
                    y = ( rot(0,2) - rot(2,0) ) * s;
                    z = ( rot(1,0) - rot(0,1) ) * s;
                }
                else
                {
                    if ( rot(0,0) > rot(1,1) && rot(0,0) > rot(2,2) )
                    {

                        // PRX_PRINT("trace<epsilon", PRX_TEXT_RED);
                        double s = 2.0 * sqrt( 1.0 + rot(0,0) - rot(1,1) - rot(2,2));
                        w = (rot(2,1) - rot(1,2) ) / s;
                        x = 0.25 * s;
                        y = (rot(0,1) + rot(1,0) ) / s;
                        z = (rot(0,2) + rot(2,0) ) / s;
                    } 
                    else if (rot(1,1) > rot(2,2)) 
                    {

                        // PRX_PRINT("trace<epsilon", PRX_TEXT_RED);
                        double s = 2.0 * sqrt( 1.0 + rot(1,1) - rot(0,0) - rot(2,2));
                        w = (rot(0,2) - rot(2,0) ) / s;
                        x = (rot(0,1) + rot(1,0) ) / s;
                        y = 0.25 * s;
                        z = (rot(1,2) + rot(2,1) ) / s;
                    }
                    else 
                    {

                        // PRX_PRINT("trace<epsilon", PRX_TEXT_RED);
                        double s = 2.0 * sqrt( 1.0 + rot(2,2) - rot(0,0) - rot(1,1) );
                        w = (rot(1,0) - rot(0,1) ) / s;
                        x = (rot(0,2) + rot(2,0) ) / s;
                        y = (rot(1,2) + rot(2,1) ) / s;
                        z = 0.25 * s;
                    }
                } 

                double norm = sqrt(w*w+x*x+y*y+z*z);
                w /= norm;
                x /= norm;
                y /= norm;
                z /= norm;

            }

        }
    }
}
