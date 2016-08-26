/**
 * @file jk_tracking_tp_t.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "planning/task_planners/jk_tracking_tp.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "planning/modules/manipulation_validity_checker.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/heuristic_search/object_collision_constraints.hpp"

#include "planning/specifications/simple_pap_specification.hpp"

#include <boost/range/adaptor/map.hpp>
#include <pluginlib/class_list_macros.h>

#include "planning/task_planners/manipulation_tp.hpp"
#include "utilities/definitions/manip_defs.hpp" 
#include "prx/utilities/definitions/sys_clock.hpp"

#include <string>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::jk_tracking_tp_t ,prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;
    using namespace plan::comm;

    namespace packages
    {
        namespace manipulation
        {

            jk_tracking_tp_t::jk_tracking_tp_t()
            {

            }

            jk_tracking_tp_t::~jk_tracking_tp_t()
            {

            }

            void jk_tracking_tp_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                PRX_DEBUG_COLOR("Initializing jk_tracking_tp_t task planner ...", PRX_TEXT_CYAN);
                task_planner_t::init(reader,template_reader);                
                manipulation_task_planner_name = parameters::get_attribute("manipulation_task_planner_name", reader, template_reader);
                full_manipulator_context_name = parameters::get_attribute("full_manipulator_context_name", reader, template_reader);


                foreach( const parameter_reader_t* r, parameters::get_list("paired_manipulation_contexts", reader, template_reader) )
                {
                    std::string ffee = parameters::get_attribute_as<std::string>("ffee", r, template_reader);
                    std::string manip = parameters::get_attribute_as<std::string>("manipulator", r, template_reader);
                    paired_manipulation_contexts.push_back(make_pair(ffee,manip));
                    PRX_DEBUG_COLOR( "Paired ffee context:  " << paired_manipulation_contexts.back().first << " with manip context:  " << paired_manipulation_contexts.back().second, PRX_TEXT_LIGHTGRAY );
                }

                object_target_vec = parameters::get_attribute_as<std::vector<double> >("object_target", reader, template_reader);
                object_name = parameters::get_attribute("object_name", reader, template_reader);
            }

            void jk_tracking_tp_t::link_world_model(world_model_t * const model)
            {
                task_planner_t::link_world_model(model);
                manipulation_model = dynamic_cast<manipulation_world_model_t*>(model);
                if(manipulation_model == NULL)
                    PRX_FATAL_S("The pick and place task planner can work only with manipulation world model!");
            }

            void jk_tracking_tp_t::link_query(query_t* new_query)
            {
                task_planner_t::link_query(new_query);
                in_query = static_cast<motion_planning_query_t*>(new_query);
            }

            void jk_tracking_tp_t::setup()
            {
                manipulation_model->use_context(full_manipulator_context_name);
                manipulation_model->get_objects(objects);
                manip_initial_state = manipulation_model->get_state_space()->alloc_point();
                input_specification->setup(manipulation_model);

                foreach(planner_t* planner, planners | boost::adaptors::map_values)
                {
                    PRX_DEBUG_COLOR("Simple P&P is seting up planner: " << planner->get_name(), PRX_TEXT_CYAN);
                    planner->setup();
                    output_specifications[planner->get_name()]->setup( manipulation_model );
                    planner->link_specification( output_specifications[planner->get_name()] );
                }
                simple_pap_specification_t* pap_spec = dynamic_cast< simple_pap_specification_t* >(input_specification);
                PRX_ASSERT(pap_spec != NULL);
                validity_checker = pap_spec->validity_checker;
                manip_validity_checker = dynamic_cast< manipulation_validity_checker_t* >(pap_spec->validity_checker);
                PRX_ASSERT(manip_validity_checker != NULL);
            }

            bool jk_tracking_tp_t::execute()
            {
                foreach(planner_t* planner, planners | boost::adaptors::map_values)
                {                    
                    PRX_PRINT("Simple P&P executing planner: " << planner->get_name(), PRX_TEXT_GREEN);
                    planner->execute();
                }
                return true;
            }

            bool jk_tracking_tp_t::succeeded() const
            {
                return true;
            }

            const util::statistics_t* jk_tracking_tp_t::get_statistics()
            {
                return NULL;
            }

            void jk_tracking_tp_t::setup_constraints()
            {

                simple_pap_specification_t* pap_spec = dynamic_cast< simple_pap_specification_t* >(input_specification);
                manipulation_query->path_constraints = pap_spec->validity_checker->alloc_constraint();
                valid_constraints = pap_spec->validity_checker->alloc_constraint();
                tmp_constraint = pap_spec->validity_checker->alloc_constraint();
                
                //Need to push in an index for each of the movable bodies in the world?
                object_collision_constraints_t* occ = dynamic_cast< object_collision_constraints_t* >( valid_constraints );
                if( occ != NULL )
                {
                    PRX_PRINT("SUGGESTED TP: Cast to Object Collision Constraints worked!", PRX_TEXT_GREEN);
                    for( unsigned i=0; i<objects.size(); ++i )
                    {
                        occ->constraints.insert( i );
                    }
                }
                else
                {
                    PRX_PRINT("SUGGESTED TP: Cast to Object Collision Constraints DID NOT WORK!", PRX_TEXT_RED);                    
                }
                manipulation_query->set_valid_constraints( valid_constraints );
            }

            bool jk_tracking_tp_t::resolve_ffee_query(trajectory_t& ffee_trajectory)
            {
                valid_constraints = NULL;
                config_t retract_config;
                retract_config.set_position(0.0,0.0,-0.04);
                retract_config.set_orientation(0,0,0,1);
                
                PRX_DEBUG_COLOR("Resolve query from jk_tracking_tp_t ...  context:" << paired_manipulation_contexts.back().second,PRX_TEXT_GREEN);
                
                // Initialize the entire state of the world model
                manipulation_model->use_context(full_manipulator_context_name);
                manipulation_model->get_state_space()->copy_from_point(manip_initial_state);  

                PRX_DEBUG_COLOR("Starting full manip initial state: " <<manipulation_model->get_state_space()->print_point(manip_initial_state), PRX_TEXT_CYAN );


                // Get the manipulator for the non-free-flying end-effector
                manipulation_model->use_context(paired_manipulation_contexts.back().second);
                config_t initial_config;
                manipulation_model->FK(initial_config);
                PRX_DEBUG_COLOR("Initial config: " << initial_config, PRX_TEXT_LIGHTGRAY);
                unsigned current_grasping_state = manipulation_model->get_current_grasping_mode();

                // TODO: Use the correct ffee context based on desired query context

                std::string ffee_context = paired_manipulation_contexts.back().first;

                manipulation_model->use_context(ffee_context);

                movable_body_plant_t* move_object = NULL;
                std::string name, subpath;
                for(unsigned i = 0; i<objects.size() && move_object == NULL; ++i)
                {
                    boost::tie(subpath, name) = reverse_split_path(objects[i]->get_pathname());
                    PRX_DEBUG_COLOR("name : " << name << "  subpath:" << subpath, PRX_TEXT_GREEN);
                    if(name == object_name)
                        move_object = objects[i];
                }

                const space_t* object_space = move_object->get_state_space();

                manipulator = manipulation_model->get_current_manipulation_info()->manipulator;                
                state_t* initial_state = manipulation_model->get_state_space()->alloc_point();

                vector_t set_the_position = initial_config.get_position();
                quaternion_t set_the_orientation = initial_config.get_orientation();
                vector_t euler_angles;
                set_the_orientation.convert_to_euler(euler_angles);
                std::vector<double> set_the_state;
                set_the_state.push_back(set_the_position[0]);
                set_the_state.push_back(set_the_position[1]);
                set_the_state.push_back(set_the_position[2]);
                set_the_state.push_back(euler_angles[0]);
                set_the_state.push_back(euler_angles[1]);
                set_the_state.push_back(euler_angles[2]);
                set_the_state.push_back(current_grasping_state);

                manipulation_model->get_state_space()->set_from_vector(set_the_state, initial_state);

                state_t* initial_object = object_space->alloc_point();
                state_t* target_object = object_space->alloc_point();
                object_space->copy_vector_to_point(object_target_vec, target_object);

                manipulation_query = dynamic_cast< manipulation_query_t* >( output_queries[ manipulation_task_planner_name ] );

                manipulation_query->setup_pick(  ffee_context, false, GRASP_GREEDY, move_object, 1, retract_config, initial_state, initial_object );

                setup_constraints();
                
                //Here is the dummy link
                planners[manipulation_task_planner_name]->link_query(manipulation_query);

                
                // //Object states needed to "resolve_grasp_query"
                // std::vector< sim::state_t* > object_states;
                // object_states.push_back(initial_object);                
                // object_states.push_back(target_object);
                // util::sys_clock_t _clock;
                // _clock.reset();
                //Get the set of grasps from the grasping planner
                 // manipulation_tp_t* manip_tp = dynamic_cast< manipulation_tp_t* >(planners[manipulation_task_planner_name]);
                // //grasp_rrt_tp_t* manip_tp = dynamic_cast< grasp_rrt_tp_t* >(planners[manipulation_task_planner_name]);
                // std::vector< grasp_t > grasps_to_suggest = manip_tp->get_grasps(object_states);
                // PRX_INFO_S("\n\nNUMBER OF GRASPS TO SUGGEST::: "<<grasps_to_suggest.size());

                PRX_DEBUG_COLOR("Starting manip initial FFEE state: " <<manipulation_model->get_state_space()->print_point(initial_state), PRX_TEXT_CYAN );

                ffee_trajectory.link_space(manipulation_model->get_state_space());
                //Loop through the grasps and suggest them one by one

                planners[manipulation_task_planner_name]->resolve_query();

                if (manipulation_query->found_path)
                {
                    PRX_DEBUG_COLOR("FFEE found path!", PRX_TEXT_CYAN);
                    ffee_trajectory.clear();
                    ffee_trajectory.link_space(manipulation_model->get_state_space());
                    manipulation_model->propagate_plan(initial_state, manipulation_query->plan, ffee_trajectory);
                    visualize_trajectory(ffee_trajectory, manipulator->get_end_effector_name(0));
                }

                // //REPORT the final plan
                // manipulation_model->convert_plan(in_query->plan, manipulator->get_control_space(), manipulation_query->plan, manipulation_model->get_control_space());
                // // manipulation_model->convert_plan(in_query->plan, manipulator->get_control_space(), query2->plan, manipulation_model->get_control_space());

                
                manipulation_model->use_context(full_manipulator_context_name);
                simple_pap_specification_t* pap_spec = dynamic_cast< simple_pap_specification_t* >(input_specification);
                if (valid_constraints != NULL)
                {
                    pap_spec->validity_checker->free_constraint(valid_constraints);
                }
                return manipulation_query->found_path;

            }

            void jk_tracking_tp_t::resolve_query()
            {
                PRX_DEBUG_COLOR("Resolving FFEE Query", PRX_TEXT_MAGENTA);
                trajectory_t ffee_trajectory;
                if (!resolve_ffee_query(ffee_trajectory))
                {
                    PRX_ERROR_S ("Could not resolve FFEE query!");
                    return;
                }
                PRX_DEBUG_COLOR("Finished Resolving FFEE Query", PRX_TEXT_CYAN);

                PRX_DEBUG_COLOR("Resolving FFEE Tracking", PRX_TEXT_MAGENTA);
                if(!track_ffee_trajectory(ffee_trajectory))
                {
                    PRX_ERROR_S ("Could not track FFEE trajectory!");
                    return;
                }
                PRX_DEBUG_COLOR("Finished Resolving FFEE Tracking", PRX_TEXT_CYAN);
            }

            bool jk_tracking_tp_t::track_ffee_trajectory(sim::trajectory_t& ffee_trajectory)
            {  
                // Initialize the entire state of the world model
                manipulation_model->use_context(full_manipulator_context_name);
                manipulation_model->get_state_space()->copy_from_point(manip_initial_state);  


                manipulation_model->use_context(paired_manipulation_contexts.back().second);
                manipulator = manipulation_model->get_current_manipulation_info()->manipulator;     
                manipulation_model->get_end_effector_local_config(ee_local_config);
                in_query->link_spaces( manipulator->get_state_space(), manipulator->get_control_space());

                state_t* start_state = manipulation_model->get_state_space()->alloc_point();
                state_t* result_state = manipulation_model->get_state_space()->alloc_point();

                plan_t tmp_plan(manipulation_model->get_control_space()), tmp_plan2(manipulation_model->get_control_space());
                config_t goal_config;

                int failure = 0;
                in_query->plan.clear();
                for(unsigned i = 10; i < ffee_trajectory.size(); i+=20)
                {
                    tmp_plan.clear();
                    euler_state_to_config(ffee_trajectory[i], goal_config);
                    if (attempt_jac_steer(tmp_plan, start_state, result_state, goal_config))
                    {
                        manipulation_model->get_state_space()->copy_point(start_state, result_state);
                        manipulation_model->get_state_space()->copy_from_point(start_state);
                        tmp_plan2 += tmp_plan;
                    }
                    else
                    {
                        PRX_WARN_S ("Jacobian steering failed");
                        ++failure;
                    }
                }
                tmp_plan.clear();
                euler_state_to_config(ffee_trajectory[ffee_trajectory.size()-1], goal_config);
                if (attempt_jac_steer(tmp_plan, start_state, result_state, goal_config))
                {
                    manipulation_model->get_state_space()->copy_point(start_state, result_state);
                    tmp_plan2 += tmp_plan;
                }
                else
                {
                    PRX_WARN_S ("Jacobian steering failed");
                    ++failure;
                }
                //PRX_DEBUG_COLOR("Temp plan: " << tmp_plan2.print(), PRX_TEXT_BLUE);
                manipulation_model->convert_plan(in_query->plan, manipulator->get_control_space(), tmp_plan2, manipulation_model->get_control_space());

                //PRX_DEBUG_COLOR("Converted plan: " << in_query->plan.print(), PRX_TEXT_RED);

                manipulation_model->use_context(full_manipulator_context_name);

                if (failure >= 10)
                    return false;
                else
                    return true;


                //jac_steering( sim::plan_t& result_plan, workspace_trajectory_t& ee_trajectory, sim::state_t* result_state, const util::space_point_t* start_state, const util::config_t& goal_config)
            }

            void jk_tracking_tp_t::euler_state_to_config(const sim::state_t* st, util::config_t& config)
            {
                // Assuming x,y,z,roll,pitch,yaw
                config.set_position(st->memory[0], st->memory[1], st->memory[2]);
                quaternion_t temp_orientation; temp_orientation.set_from_euler(st->memory[3], st->memory[4], st->memory[5]);
                config.set_orientation(temp_orientation);

                config_t blah = ee_local_config;
                blah.relative_to_global(config);
                config = blah;
            }

            bool jk_tracking_tp_t::attempt_jac_steer(plan_t& output_plan, state_t* start_state, state_t* result_state, config_t& goal_config)
            {
                PRX_PRINT("Attempting Jac Steer", PRX_TEXT_BLUE);

                if (!ros::ok())
                    exit(0);

                workspace_trajectory_t ee_traj;
                plan_t new_plan(manipulation_model->get_control_space());

                bool success = manipulation_model->jac_steering( new_plan, ee_traj, result_state, start_state, goal_config);
                unsigned jac_error = manipulation_model->get_jac_steer_error_type();


                if(success)
                {
                    trajectory_t traj(manipulation_model->get_state_space());
                    PRX_PRINT("Jac steering success...", PRX_TEXT_GREEN);
                    manipulation_model->propagate_plan(start_state,new_plan,traj);
                    bool valid_constrained_trajectory = false;
                    tmp_constraint->clear();

                    valid_constrained_trajectory = manip_validity_checker->validate_and_generate_constraints(tmp_constraint, traj, ee_traj);

                    if(valid_constrained_trajectory)
                    {
                        // TODO: Handle constraints here again
                        // PRX_PRINT("Valid IK steering trajectory...", PRX_TEXT_CYAN);
                        // if (manipulation_query->astar_mode == STANDARD_SEARCH || manipulation_query->astar_mode == LAZY_SEARCH )
                        // {
                        //     if (manipulation_query->valid_constraints->has_intersection(tmp_constraint))
                        //     {
                        //         return false;
                        //     }
                        // }
                        // output_constraints->merge(tmp_constraint);

                        output_plan = new_plan;

                        //manipulation_model->convert_plan(output_plan, manipulation_model->get_control_space(), new_plan, manipulation_model->get_control_space());
                        
                        return true;
                    }
                    else
                    {
                        PRX_DEBUG_COLOR("Not a valid constrained trajectory?", PRX_TEXT_LIGHTGRAY);
                    }
                }
                else
                {
                    PRX_DEBUG_COLOR("Jacobian error steering type: " << jac_error, PRX_TEXT_CYAN);
                }
                return false;
            }

            void jk_tracking_tp_t::visualize_trajectory(const trajectory_t& traj, const std::string& vis_geom_name)
            {
                std::vector<geometry_info_t> geoms;
                std::vector<config_t> configs;
                hash_t<std::string, std::vector<double> > map_params;
                std::vector<double> params;
                std::vector<std::string> system_names;
                system_names.push_back(vis_geom_name);

                std::string visualization_solution_name = "jk_tracking_tp";
                                
                for( size_t i = 0; i < traj.size(); i++ )
                {
                    //PRX_DEBUG_COLOR("Traj : " << i << " : " << manipulation_model->get_state_space()->print_point(traj[i]), PRX_TEXT_MAGENTA);

                    //map_params.clear();
                    //((visualization_comm_t*)vis_comm)->compute_configs(traj[i], system_names, map_params);
                    //params.insert(params.end(), map_params[vis_geom_name].begin(), map_params[vis_geom_name].end());
                    //params.back() += 3; //this is so the solution will be above
                    params.push_back(traj[i]->at(0));
                    params.push_back(traj[i]->at(1));
                    params.push_back(traj[i]->at(2));
                }

                std::string name = visualization_solution_name + "/" + vis_geom_name + "/path";//_" + int_to_str(solution_number);
                geoms.push_back(geometry_info_t(vis_geom_name, name, PRX_LINESTRIP, params, "red"));
                configs.push_back(config_t());
                ((visualization_comm_t*)vis_comm)->visualization_geom_map[visualization_solution_name] = geoms;
                ((visualization_comm_t*)vis_comm)->visualization_configs_map[visualization_solution_name] = configs;
                geoms.clear();
                configs.clear();

                ((visualization_comm_t*)vis_comm)->send_geometries();
            }
        }
    }
}

