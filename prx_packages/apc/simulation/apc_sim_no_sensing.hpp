/**
 * @file manual_application.hpp
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
#ifndef PRX_APC_SIM_NO_SENSING
#define PRX_APC_SIM_NO_SENSING

#pragma once 

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/plan.hpp"
// #include "prx/simulation/systems/controllers/controller.hpp"
#include "simulation/apc_controller.hpp"
#include "prx/simulation/state.hpp"
#include "prx/simulation/control.hpp"

#include "simulation/systems/plants/manipulator.hpp"
#include "simulation/systems/plants/movable_body_plant.hpp"

#include <prx_simulation/send_trajectoryAction.h>
#include <actionlib/server/action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include "prx_simulation/object_msg.h"

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryResult.h>

#include "prx/simulation/applications/application.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include <prx_planning/apc_queryAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <simulation/json/read_file.h>




typedef actionlib::ActionServer<prx_simulation::send_trajectoryAction> TrajectoryServer;
typedef actionlib::SimpleActionClient<prx_simulation::send_trajectoryAction> send_traj_client;
typedef actionlib::SimpleActionClient<prx_planning::apc_queryAction> planning_app_query_client;

namespace prx
{
    namespace packages
    {
        namespace apc
        {
            class apc_sim_no_sensing_t : public sim::application_t
            {
              public:
                apc_sim_no_sensing_t();
                virtual ~apc_sim_no_sensing_t();

                enum automaton_states
                {
                   START, SHELF_DETECTION, TARGET_SELECTION, MOVE_AND_SENSE, POSE_ESTIMATION, GRASP_PLANNING, BLOCKING_ITEM_SELECTION, EXECUTE_REACHING, EVALUATE_GRASP,
                   ADJUST_EE, EXECUTE_GRASP, DISENGAGE_EE, PLAN_FOR_BLOCKING_ITEM, PLAN_FOR_TARGET_ITEM, EXECUTE_PLACING, PLAN_FOR_RETRACTION, STOP_ROBOT, TURN_OFF_SENSING,
                   MOVE_TO_HOME, END, PLAN_BIN_TRAJECTORIES, PLAN_MOVE_TARGET_OUTSIDE_BIN
                };

              /** @copydoc empty_application_t::init(const util::parameter_reader_t * const) */
                virtual void init(const util::parameter_reader_t * const reader);
                virtual void frame(const ros::TimerEvent& event);
                virtual void set_selected_path(const std::string& path);

            protected:
                double plan_duration;
                double execution_time;
                bool executing_trajectory;
                bool currently_in_home_position;
                automaton_states automaton_state;
                bool displayed_state;
                planning_app_query_client* planning_app_query;
                send_traj_client* controller_query;
                apc_controller_t* controller;
                util::space_t* output_control_space;

                bool real_robot;
                bool has_goal;
                util::hash_t<std::string,unsigned> name_index_map;
                ros::Subscriber real_robot_state;
                sim::state_t* robot_state;
                double duration;
                void real_robot_state_callback(const sensor_msgs::JointState& stateMsg);
                ros::Publisher state_publisher;

                control_msgs::FollowJointTrajectoryGoal robot_command;
                control_msgs::FollowJointTrajectoryGoal unigripper_command;
                actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* robot_ac;
                actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* unigripper_ac;
                double create_robot_message(std::vector<sim::trajectory_t*>& robot_trajs, std::vector<sim::plan_t*>& robot_plans, std::vector<bool>& grasp_plan, int index);
                void send_robot_message(double duration);
                void create_and_send_robot_command();
                bool start_servoing();

                ros::Publisher object_publisher;

                util::hash_t<std::string, prx::sim::system_ptr_t > subsystems;
                const util::space_t* child_state_space;
                manipulation::manipulator_t* manipulator;
                std::vector<manipulation::movable_body_plant_t*> movable_bodies;
                util::hash_t<std::string, prx::sim::system_ptr_t> obstacles_hash;
                std::vector<sim::obstacle_t*> obstacles;
                sim::obstacle_t* shelf;
                manipulation::movable_body_plant_t* moved;
                util::config_t new_config;
                prx_simulation::object_msg objectMsg;
                std::vector<std::string> bin_names;

                ros::NodeHandle n;
                int counter;
                bool Init_Finished;


                std::string getStringFromEnum(automaton_states a_s);
                void plan_to_bin(prx_planning::apc_queryGoal command);
                void place_object_callback(const prx_simulation::object_msg& objectMsg);
                void estimate_shelf_position();
                bool estimate_objects_pose();
                void update_simulation();
                void reset_object_pose();
                void check_object_collision(manipulation::movable_body_plant_t* plant);

                WorkOrder work_order;
                BinContents bin_contents;
                int current_item_index;
                int number_of_orders;
                int current_bin_name;
                bool Got_Item;
                std::string CurrentTarget;
                std::string CurrentBin;
                std::string CurrentArm;
                prx_planning::apc_queryResult current_result;
                void publish_state();
                bool visualize_plans;
                int nr_execution_failures;
                int nr_grasping_failures;
                bool rearrangement_flag;


                // For pose evalution
                double l2_norm(std::vector<double> const& u);
                std::vector<double> target_obj_pose;
                double POSE_CAHNGE_THRESHOLD = 0.001;
                
                void clear_plan();
                std::vector<double> object_pose;
                std::vector<double> object_new_pose;
                double target_index;
                double z_axis_offset;

          };
      }
  }
}
#endif