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
#ifndef PRX_APC_SIM_APPLICATION
#define PRX_APC_SIM_APPLICATION

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
#include <std_msgs/Bool.h>
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

#include <semaphore.h>



typedef actionlib::ActionServer<prx_simulation::send_trajectoryAction> TrajectoryServer;
typedef actionlib::SimpleActionClient<prx_simulation::send_trajectoryAction> send_traj_client;
typedef actionlib::SimpleActionClient<prx_planning::apc_queryAction> planning_app_query_client;

namespace prx
{
    namespace packages
    {
        namespace apc
        {
            class apc_sim_application_t : public sim::application_t
            {
              public:
              	apc_sim_application_t();
              	virtual ~apc_sim_application_t();

              	enum automaton_states
	            {
	               START, SHELF_DETECTION, TARGET_SELECTION, MOVE_AND_SENSE, MOVE_AND_SENSE_TWO, MOVE_AND_SENSE_THREE, POSE_ESTIMATION, GRASP_PLANNING, BLOCKING_ITEM_SELECTION, EXECUTE_REACHING, EVALUATE_GRASP,
	               ADJUST_EE, EXECUTE_GRASP, DISENGAGE_EE, PLAN_FOR_BLOCKING_ITEM, PLAN_FOR_TARGET_ITEM, EXECUTE_PLACING, PLAN_FOR_RETRACTION, STOP_ROBOT, TURN_OFF_SENSING,
	               MOVE_TO_HOME, END, PLAN_BIN_TRAJECTORIES, RETRY_GRASP, EXECUTE_REGRASP, LIFT, EXECUTE_LIFT
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
                bool no_sensing;
                bool shelf_detection_done;
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

                std::vector<std::string> object_priority_list = {
                    "expo_dry_erase_board_eraser",
                    "soft_white_lightbulb",
                    "kleenex_tissue_box",
                    "jane_eyre_dvd",
                    "laugh_out_loud_joke_book",
                    "i_am_a_bunny_book",
                    "crayola_24_ct",
                    "staples_index_cards",
                    "up_glucose_bottle",
                    "command_hooks",
                    "ticonderoga_12_pencils",
                    "scotch_bubble_mailer",
                    "elmers_washable_no_run_school_glue",
		    "folgers_classic_roast_coffee",
                    "woods_extension_cord",
                    "kleenex_paper_towels",
                    "hanes_tube_socks",
                    "dove_beauty_bar",
                    "peva_shower_curtain_liner",
                    "safety_first_outlet_plugs", 
                    "rawlings_baseball",
                    "scotch_duct_tape",
                    "dasani_water_bottle",
                    "dr_browns_bottle_brush",
                    "kyjen_squeakin_eggs_plush_puppies",
                    "oral_b_toothbrush_green",
                    "oral_b_toothbrush_red",
                    "barkely_hide_bones",
                    "easter_turtle_sippy_cup",
                    "clorox_utility_brush",
                    "platinum_pets_dog_bowl",
                    "creativity_chenille_stems",
                    "cloud_b_plush_bear",
                    "cool_shot_glue_sticks",
                    "rolodex_jumbo_pencil_cup",
                    "cherokee_easy_tee_shirt",
                    "womens_knit_gloves",
                    "fitness_gear_3lb_dumbbell",
                    "fiskars_scissors_red"
                };


                ros::NodeHandle n;
                int counter;
                bool Init_Finished;

                // Output JSON File functions
                void update_json_file();
                void output_json_file();

                // -----
        	    std::string getStringFromEnum(automaton_states a_s);
                void plan_to_bin(prx_planning::apc_queryGoal command);
	        	void place_object_callback(const prx_simulation::object_msg& objectMsg);
	        	void estimate_shelf_position();
                void estimate_objects_pose();
                bool update_objects_pose();

	        	void update_simulation();
	        	void reset_object_pose();
                void publish_state();
                WorkOrder work_order;
                BinContents bin_contents;
                int current_item_index;
                int number_of_orders;
                int current_bin_name;
                bool Got_Item;
                std::string CurrentTarget;
                std::string CurrentBin;
                std::string lastBin;
                std::string CurrentArm;
                prx_planning::apc_queryResult current_result;
                prx_planning::apc_queryResult regrasp_result;
                prx_planning::apc_queryResult lift_result;
                int nr_execution_failures;
                int nr_grasping_failures;
                bool rearrangement_flag;
                bool move_to_next_bin = false;

                // For pose evalution
                double l2_norm(std::vector<double> const& u);
                std::vector<double> target_obj_pose;

                // For objects in collision
                double offset_threshold;

                void clear_plan();
                void get_current_target_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
                void unigripper_callback(const std_msgs::Bool& Msg);
                ros::Subscriber get_current_target_obj_pose;
                ros::Subscriber grasp_success_sub;
                std::vector<double> current_target_obj_pose = {10,10,10,0,0,0,1};
                std::vector<std::string> arms = {"right","right","right","right","right","right","right","right","right","right","right","right"};
                // "left","left","left","left","left","left","left","left","left","left","left","left",
                bool pose_change = false;
                bool grasp_success = true;
                bool early_grasp = false;
                bool actual_grasp = false;
                bool good_grasp = true;
                std::vector<double> home_position={0.00000,1.57000,0.00000,0.00000,-1.70000,0.00000,0.00000,0.00000,0.00000,1.57000,0.00000,0.00000,-1.70000,0.00000,0.00000,0.00000,1.00000,1.00000,1.00000,1.00000};
                sim::control_t* unigripper_control_on;
                double target_index;
                int sensing_counter = 0;

                bool check_object_collision(manipulation::movable_body_plant_t* plant, unsigned depth);
                double z_axis_offset = 0.0;
                double left_offset = 0.0;
                double right_offset = 0.0;
                std::vector<double> object_pose;

                util::hash_t<std::string, std::vector< std::string > > object_to_prioritized_end_effector_context;
                util::hash_t<std::string, std::vector< double > > object_poses;
                std::string json_file;
                std::string output_json_path;
                std::vector<std::string> tote_contents;

                sem_t semaphore;
                void block()
                {
                      sem_wait(&semaphore);
                }     
                void unblock()
                {
                      sem_post(&semaphore);
                }
          };
      }
  }
}
#endif
