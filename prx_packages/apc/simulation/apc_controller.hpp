/**
 * @file apc_controller.hpp 
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
#pragma once

#ifndef PRX_APC_CONTROLLER
#define PRX_APC_CONTROLLER

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/plan.hpp"
#include "prx/simulation/systems/controllers/controller.hpp"
#include "prx/simulation/state.hpp"
#include "prx/simulation/control.hpp"

#include "simulation/systems/plants/manipulator.hpp"
#include "simulation/systems/plants/movable_body_plant.hpp"

#include <prx_simulation/send_trajectoryAction.h>
#include <actionlib/server/action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "prx_simulation/object_msg.h"

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryResult.h>

typedef actionlib::ActionServer<prx_simulation::send_trajectoryAction> TrajectoryServer;

namespace prx
{
    namespace packages
    {
        namespace apc
        {
            class apc_controller_t : public sim::controller_t
            {

              public:
                apc_controller_t();

                virtual ~apc_controller_t();

                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

                virtual void propagate(const double simulation_step = 0);

                virtual void compute_control();

                bool real_robot;

                void convert_and_copy_to_robot_plan(const std::vector<prx_simulation::control_msg>& received_plan);
                void get_robot_plans(std::vector<sim::trajectory_t*>& robot_trajs, std::vector<sim::plan_t*>& robot_plans, std::vector<bool>& grasp_plan);
                void set_robot_plan();
                void clear_robot_plan();
                void set_completed();
                void cancel_goal();
                void send_unigripper_grasping_command(bool gripper_on);
                void send_zero_control();
                util::space_t* get_output_control_space();
              protected:

                ros::NodeHandle n;
                sim::plan_t plan;
                sim::plan_t contingency_plan;
                const util::space_t* child_state_space;
                manipulation::manipulator_t* manipulator;
                util::config_t new_config;

                TrajectoryServer* execute_plan_server;
                ros::Publisher state_publisher;

                bool has_goal;
                TrajectoryServer::GoalHandle active_goal;

                control_msgs::FollowJointTrajectoryGoal robot_command;
                control_msgs::FollowJointTrajectoryGoal unigripper_command;
                actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* robot_ac;
                actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* unigripper_ac;
                sim::plan_t robot_plan;
                bool send_to_robot;
                

            };
        }
    }
}

#endif