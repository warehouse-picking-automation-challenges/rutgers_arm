/**
 * @file apc_controller.cpp 
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "simulation/apc_controller.hpp"
#include "prx_simulation/state_msg.h"


#include <boost/range/adaptor/map.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::apc::apc_controller_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace packages
    {
        using namespace manipulation;
        namespace apc
        {
            apc_controller_t::apc_controller_t()
            {
                has_goal = false;
                send_to_robot = false;
            }

            apc_controller_t::~apc_controller_t()
            {

            }

            void apc_controller_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
            {
                controller_t::init(reader, template_reader);

                // Set up communication with paired planning node
                execute_plan_server = new TrajectoryServer(n,"prx/execute_plan_action",false);
                // execute_plan_server->registerGoalCallback( boost::bind(&apc_controller_t::receive_goal, this, _1) );
                // execute_plan_server->registerCancelCallback( boost::bind(&apc_controller_t::cancel_goal, this, _1) );
         
                foreach(system_ptr_t plant, subsystems | boost::adaptors::map_values)
                {
                    //Create and initialize information for all the end effectors. 
                    if(dynamic_cast<manipulator_t*>(plant.get()) != NULL)
                    {
                        //Create end effector infos for each end effector.
                        manipulator = dynamic_cast<manipulator_t*>(plant.get());
                    }

                }
                child_state_space = manipulator->get_state_space();

                // create contingency plan
                contingency_plan.link_control_space(output_control_space);
                plan.link_control_space(output_control_space);
                robot_plan.link_control_space(output_control_space);
                manipulator->append_contingency(contingency_plan, 0.0);

                // state_publisher = n.advertise<prx_simulation::state_msg>("prx/manipulator_state", 1);
                execute_plan_server->start();
                
            }

            void apc_controller_t::convert_and_copy_to_robot_plan(const std::vector<prx_simulation::control_msg>& received_plan)
            {
                PRX_PRINT("has_goal: "<<has_goal, PRX_TEXT_GREEN);
                control_t* control = output_control_space->alloc_point();
                robot_plan.clear();
                foreach(const control_msg& control_msg, received_plan)
                {
                    for( unsigned int i = 0; i < control_msg.control.size(); ++i ){
                        control->at(i) = control_msg.control[i];
                    }
                    robot_plan.copy_onto_back(control, control_msg.duration);
                }
                output_control_space->free_point(control);
            }

            void apc_controller_t::set_robot_plan()
            {
                if(has_goal)
                {
                    PRX_INFO_S("Rejecting plan!!!");
                }
                else
                {
                    PRX_INFO_S("Accepting plan...");
                    plan = robot_plan;
                    has_goal = true;
                }
            }

            void apc_controller_t::set_completed()
            {
                has_goal = false;
            }

            void apc_controller_t::get_robot_plans(std::vector<trajectory_t*>& robot_trajs, std::vector<plan_t*>& robot_plans, std::vector<bool>& grasp_plan)
            {
                if(has_goal)
                {

                    // state_t* local_state = child_state_space->alloc_point();
                    robot_trajs.push_back(new trajectory_t(child_state_space));
                    robot_plans.push_back(new plan_t(output_control_space));
                    grasp_plan.push_back(false);
                    // robot_trajs.back()->copy_onto_back(local_state);
                    bool last_was_grasp = false;
                    foreach(plan_step_t step, robot_plan)
                    {
                        int steps = (int)((step.duration / simulation::simulation_step) + .1);
                        output_control_space->copy_from_point(step.control);
                        bool new_grasp = (output_control_space->at(16)>=1 || output_control_space->at(17)>=1 || output_control_space->at(18)>=1 || output_control_space->at(19)>=1);
                        if(last_was_grasp || new_grasp)
                        {
                            //end old traj, start new one
                            robot_trajs.push_back(new trajectory_t(child_state_space));
                            robot_plans.push_back(new plan_t(output_control_space));
                            grasp_plan.push_back(new_grasp);
                            // robot_trajs.back()->copy_onto_back(local_state);
                            if(last_was_grasp)
                                last_was_grasp = false;
                            last_was_grasp = new_grasp;

                        }
                        robot_plans.back()->copy_onto_back(step.control,step.duration);
                    }
                }
            }

            void apc_controller_t::clear_robot_plan()
            {
                robot_plan.clear();
            }

            util::space_t* apc_controller_t::get_output_control_space()
            {
                return output_control_space;
            }

           

            void apc_controller_t::cancel_goal()
            {
                //TODO
                has_goal = false;
                robot_plan.clear();
                plan.clear();

            }
            void apc_controller_t::propagate(const double simulation_step)
            {
                controller_t::propagate(simulation_step);
            }

            void apc_controller_t::compute_control()
            {
                if(!real_robot)
                {
                    control_t* new_control = plan.get_next_control(simulation::simulation_step);
                    if(plan.length()>0)
                    {
                        // PRX_STATUS("Plan: "<<plan.length(),PRX_TEXT_GREEN);
                        // PRX_STATUS("control: "<<new_control->at(16), PRX_TEXT_GREEN);
                        if(new_control->at(16)>1.0)
                        {
                            //early grasp detection
                            if(new_control->at(16)==3)
                            {
                                PRX_PRINT("@CONTROLLER: Early grasp detected!",PRX_TEXT_RED);
                                PRX_PRINT("@CONTROLLER: Ignoring early grasp for simulation!",PRX_TEXT_GREEN);
                                new_control->at(16)=0;
                            }
                            else
                            {
                                PRX_PRINT("GRASPING!!",PRX_TEXT_RED);
                            }
                        }
                    }
                    if( new_control == NULL )
                    {
                        new_control = contingency_plan.get_control_at(0);
                        if(has_goal)
                        {
                            // active_goal.setSucceeded();
                            has_goal = false;
                        }
                    }

                    output_control_space->copy_from_point(new_control);
                    manipulator->compute_control();
                }
            }

            void apc_controller_t::send_unigripper_grasping_command(bool gripper_on)
            {
                has_goal = true;
                PRX_PRINT("@CONTROLLER:  SENDING UNIGRIPPER GRASPING COMMAND",PRX_TEXT_CYAN);
                control_t* unigripper_grasping_control = output_control_space->alloc_point();
                if (gripper_on)
                {
                    unigripper_grasping_control->at(18) = 2;
                }
                else
                {
                    unigripper_grasping_control->at(18) = 1;
                }
                output_control_space->copy_from_point(unigripper_grasping_control);
                manipulator->compute_control();
            }

            void apc_controller_t::send_zero_control()
            {
                // has_goal = true;
                // PRX_PRINT("@CONTROLLER: SENDING ZERO CONTROL",PRX_TEXT_RED);
                control_t* zero_control = output_control_space->alloc_point();
                output_control_space->copy_from_point(zero_control);
                manipulator->compute_control();
            }
        }
    }
}
