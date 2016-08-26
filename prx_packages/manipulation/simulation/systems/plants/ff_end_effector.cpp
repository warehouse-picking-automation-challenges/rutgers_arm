/**
 * @file unigripper.cpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2016, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "simulation/systems/plants/ff_end_effector.hpp"

#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/simulation/simulators/simulator.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"

#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/range/adaptor/map.hpp> //adaptors
#include <pluginlib/class_list_macros.h>
#include <urdf/model.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <sys/param.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::ff_end_effector_t, prx::sim::system_t)




namespace prx
{
    using namespace sim;
    using namespace util;
    namespace packages
    {
        namespace manipulation
        {
            ff_end_effector_t::ff_end_effector_t()
            {
            }

            ff_end_effector_t::~ff_end_effector_t()
            {
                state_space->free_point(temp_state1);
                state_space->free_point(temp_state2);
            }

            void ff_end_effector_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                plannable_manipulator = parameters::get_attribute_as<bool>("plannable_manipulator", reader, template_reader, true);

                if (plannable_manipulator)
                {
                    state_memory = {&_x,&_y,&_z,&_roll,&_pitch,&_yaw};
                }
                else
                {
                    state_memory = {&_x,&_y,&_z,&_qx, &_qy, &_qz, &_qw};
                }

                manipulator_t::init(reader, template_reader);   

                KDL::Chain chain1;
                kdl_tree->getChain(kdl_tree->getRootSegment()->first, reverse_split_path(effector_names[0]).second, chain1);          
                KDL::JntArray q(chain1.getNrOfJoints());  

                KDL::ChainFkSolverPos_recursive fk_solver(chain1);
                fk_solver.JntToCart(q,static_transform); 
            }

            void ff_end_effector_t::create_spaces()
            {
                if (plannable_manipulator)
                {
                    state_space = new space_t("THREE_D_BODY|D", state_memory);
                    input_control_space = new space_t("THREE_D_BODY|D", control_memory);
                }
                else
                {
                    state_space = new space_t("SE3|D", state_memory);
                    input_control_space = new space_t("SE3|D", control_memory);
                }

                temp_state1 = state_space->alloc_point();
                temp_state2 = state_space->alloc_point();
            }

            void ff_end_effector_t::update_collision_info()
            {
                if (plannable_manipulator)
                {
                    util::quaternion_t manip_orientation;
                    manip_orientation.set_from_euler(_roll, _pitch, _yaw);
                    double qx, qy, qz, qw;
                    manip_orientation.get(qx, qy, qz, qw);
                    root_frame = KDL::Frame(KDL::Rotation::Quaternion(qx,qy,qz,qw),KDL::Vector(_x, _y, _z));
                    manipulator_t::update_collision_info();    
                }
                else
                {
                    root_frame = KDL::Frame(KDL::Rotation::Quaternion(_qx,_qy,_qz,_qw),KDL::Vector(_x, _y, _z));
                    manipulator_t::update_collision_info();
                }
            }

            void ff_end_effector_t::update_phys_configs(config_list_t& configs, unsigned& index) const
            {
                if (plannable_manipulator)
                {
                    util::quaternion_t manip_orientation;
                    manip_orientation.set_from_euler(_roll, _pitch, _yaw);
                    double qx, qy, qz, qw;
                    manip_orientation.get(qx, qy, qz, qw);
                    root_frame = KDL::Frame(KDL::Rotation::Quaternion(qx,qy,qz,qw),KDL::Vector(_x, _y, _z));
                    manipulator_t::update_phys_configs(configs,index);  
                }
                else
                {
                    root_frame = KDL::Frame(KDL::Rotation::Quaternion(_qx,_qy,_qz,_qw),KDL::Vector(_x, _y, _z));
                    manipulator_t::update_phys_configs(configs, index);
                }
            }

            bool ff_end_effector_t::IK_solver( space_point_t* result_state, const space_point_t* start_state, const config_t& goal_config, std::string start_link, std::string end_link)
            {      
                if (plannable_manipulator)
                {
                    //PRX_PRINT ("Calling IK SOLVER!", PRX_TEXT_GREEN);
                    //PRX_DEBUG_COLOR("Input config: " << goal_config, PRX_TEXT_MAGENTA);
                    //ignoring the names of the start and end link
                    KDL::Frame end_frame;
                    double qx,qy,qz,qw,x,y,z;
                    goal_config.get_position(x,y,z);
                    goal_config.get_orientation().get(qx,qy,qz,qw);
                    end_frame.p = KDL::Vector(x,y,z);
                    end_frame.M = KDL::Rotation::Quaternion(qx,qy,qz,qw); 
                    end_frame = end_frame*static_transform.Inverse();
                    state_space->copy_point(result_state,start_state);

                    result_state->at(0) = end_frame.p.x();
                    result_state->at(1) = end_frame.p.y();
                    result_state->at(2) = end_frame.p.z();
                    convert_to_quaternion(end_frame.M,qx,qy,qz,qw);
                    quaternion_t convert_to_euler(qx,qy,qz,qw);

                    //PRX_DEBUG_COLOR("Quaternion: " << convert_to_euler, PRX_TEXT_BLUE);

                    vector_t angles(3);
                    convert_to_euler.convert_to_euler(angles);

                    result_state->at(3) = angles[0];
                    result_state->at(4) = angles[1];
                    result_state->at(5) = angles[2];

                    //PRX_DEBUG_COLOR("ANGLES: " << angles[0] << ","<< angles[1] << ","<< angles[2], PRX_TEXT_MAGENTA);
                    return true;
                }
                else
                {
                    return manipulator_t::IK_solver(result_state, start_state, goal_config, start_link, end_link);
                }

            }

            void ff_end_effector_t::FK_solver( config_t& goal_config, std::string link_name)
            {
                if (plannable_manipulator)
                {
                    util::quaternion_t manip_orientation;
                    manip_orientation.set_from_euler(_roll, _pitch, _yaw);
                    double qx, qy, qz, qw;
                    manip_orientation.get(qx, qy, qz, qw);
                    root_frame = KDL::Frame(KDL::Rotation::Quaternion(qx,qy,qz,qw),KDL::Vector(_x, _y, _z));
                    manipulator_t::FK_solver( goal_config, link_name);
                }
                else
                {
                    root_frame = KDL::Frame(KDL::Rotation::Quaternion(_qx,_qy,_qz,_qw),KDL::Vector(_x, _y, _z));
                    manipulator_t::FK_solver(goal_config, link_name);
                }
            }

            bool ff_end_effector_t::jac_steering( sim::plan_t& result_plan, workspace_trajectory_t& ee_trajectory, util::space_point_t* result_state, const util::space_point_t* start_state, const util::config_t& goal_config, std::string start_link, std::string end_link)
            {
                if (plannable_manipulator)
                {
                    PRX_PRINT ("Calling IK steering!", PRX_TEXT_GREEN);
                    this->IK_solver(result_state,start_state,goal_config,start_link,end_link);

                    PRX_DEBUG_COLOR("Going from start: " << state_space->print_point(start_state,3) << " to: " << state_space->print_point(result_state,3), PRX_TEXT_GREEN);
                    steering_function(start_state, result_state, result_plan);

                    PRX_DEBUG_COLOR("Computing EE Trajectory for plan of size: " << result_plan.size(), PRX_TEXT_BLUE);
                    compute_ee_trajectory(ee_trajectory, start_state, result_plan);

                    return true;
                }
                else
                {
                    return manipulator_t::jac_steering(result_plan, ee_trajectory, result_state, start_state, goal_config, start_link, end_link);
                }
            }


            void ff_end_effector_t::steering_function(const state_t* start, const state_t* goal, plan_t& result_plan)
            {
                // if (plannable_manipulator)
                {
                    // PRX_PRINT( "start state :: " << state_space->print_point( start, 3 ), PRX_TEXT_BROWN );
                    // PRX_PRINT( "goal  state :: " << state_space->print_point( goal, 3 ), PRX_TEXT_BROWN );
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

                    if (max_time > PRX_ZERO_CHECK)
                    {
                        result_plan.append_onto_back(max_time);
                        std::vector<double> new_control;
                        PRX_ASSERT( max_time > PRX_ZERO_CHECK);
                        for(unsigned i=0;i<state_space->get_dimension()-end_effector_states.size(); i++)
                        {
                            double dist = goal->memory[i] - start->memory[i];
                            new_control.push_back(dist/max_time);
                        }
                        for(unsigned i=0;i<end_effector_states.size(); ++i)
                        {
                            new_control.push_back(0);
                        }

                        input_control_space->set_from_vector(new_control, result_plan.back().control);
                    }
                }
                // else
                // {
                //     manipulator_t::steering_function(start, goal, result_plan);
                // }
            }

            void ff_end_effector_t::state_to_config( util::config_t& result_config, const sim::state_t* state)
            {
                util::quaternion_t manip_orientation;
                manip_orientation.set_from_euler(state->memory[3], state->memory[4],state->memory[5]);
                result_config.set_position(state->memory[0], state->memory[1],state->memory[2]);
                result_config.set_orientation(manip_orientation);
            }

            void ff_end_effector_t::compute_ee_trajectory(workspace_trajectory_t& workspace_trajectory, const state_t* start_state, const plan_t& plan)
            {

                config_t temp_config;

                // Save the initial state
                state_space->copy_to_point(temp_state1);

                // Copy the start_state
                state_space->copy_from_point(start_state);
                state_space->copy_to_point(temp_state2);
                workspace_trajectory.clear();

                state_to_config(temp_config, temp_state2);
                workspace_trajectory.copy_onto_back(temp_config);

                foreach(plan_step_t step, plan)
                {
                    int steps = (int)((step.duration / simulation::simulation_step) + .1);
                    int i = 0;
                    PRX_PRINT ("Computed steps: " << steps, PRX_TEXT_MAGENTA);
                    if( steps > 0 )
                    {
                        for( i = 0; i < steps; i++ )
                        {
                            input_control_space->copy_from_point(step.control);
                            propagate(simulation::simulation_step);
                            state_space->copy_to_point(temp_state2);
                            state_to_config(temp_config, temp_state2);
                            workspace_trajectory.copy_onto_back(temp_config);
                        }
                    }
                }


                state_space->copy_from_point(temp_state1);
            }

        }
    }
}
