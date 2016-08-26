/**
 * @file apc_planning_application.hpp 
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

#include "prx/planning/applications/planning_application.hpp"
#include "prx/planning/queries/query.hpp"
#include "planning/apc_task_query.hpp"
#include "planning/manipulation_world_model.hpp"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <prx_simulation/send_trajectoryAction.h>
#include <prx_planning/apc_queryAction.h>
#include "prx_simulation/state_msg.h"

#include "prx_simulation/object_msg.h"
#include "simulation/systems/plants/movable_body_plant.hpp" 
#include "prx/simulation/systems/obstacle.hpp"

#include "prx/planning/modules/validity_checkers/validity_checker.hpp"

#ifndef PRX_APC_PLANNING_APPLICATION_HPP
#define	PRX_APC_PLANNING_APPLICATION_HPP

namespace prx
{
    namespace plan
    {
        class validity_checker_t;
    }
    namespace packages
    {
        namespace apc
        {

            typedef actionlib::SimpleActionClient<prx_simulation::send_trajectoryAction> send_traj_client;
            typedef actionlib::SimpleActionServer<prx_planning::apc_queryAction> query_server;

            class apc_planning_application_t : public plan::planning_application_t
            {

              public:
                apc_planning_application_t();
                virtual ~apc_planning_application_t();

                virtual void init(const util::parameter_reader_t* reader);

                virtual void execute();

              protected:

                void resolve_query(const prx_planning::apc_queryGoalConstPtr& req);

                void convert_to_plan_msg(sim::plan_t& in_plan, prx_simulation::send_trajectoryGoal& goal);
                // void send_command(prx_simulation::send_trajectoryGoal& goal,double timeout);
                void send_command(prx_simulation::send_trajectoryGoal& goal,double timeout);
                void get_state_callback(const prx_simulation::state_msg& stateMsg);

                void execute_plan_from_file(std::string file_name);

                void initialize_objects_pose();
                void update_object_callback(const prx_simulation::object_msg& objectMsg);
                manipulation::movable_body_plant_t* update_manipulated_object(const prx_planning::apc_queryGoalConstPtr& req);
                  

                std::ifstream saved_plan_file;
                sim::plan_t saved_plan;
                int plan_counter;

                std::vector<manipulation::movable_body_plant_t* > objects;
                util::hash_t<std::string, prx::sim::system_ptr_t> obstacles_hash;
                std::vector<sim::obstacle_t*> obstacles;
                sim::obstacle_t* shelf;
                manipulation::movable_body_plant_t* moved;

                ros::NodeHandle n;
                std::string full_manipulator_context_name;
                std::string current_planning_context_name;
                apc_task_query_t* output_query;
                manipulation::manipulation_world_model_t* manipulation_model;
                send_traj_client* execute_client;
                query_server* q_server;
                ros::Subscriber state_subscriber;
                sim::state_t* current_manipulator_state;
                sim::state_t* propagated_manipulator_state;
                ros::Subscriber object_subscriber;
                int total_resolves;
                bool record_plans;
                std::string cur_hand;
                std::string prev_hand;
                int stage_counter ;
                std::string cur_bin;

                unsigned shelf_update_counter;

                plan::validity_checker_t* validity_checker;
                
                util::hash_t<std::string, std::vector< std::string > > object_to_prioritized_end_effector_context;
                double planning_budget_time;
                util::sys_clock_t planning_clock;

                unsigned current_end_effector_index = 0;
                std::vector<double> home_position={0.00000,1.57000,0.00000,0.00000,-1.70000,0.00000,0.00000,0.00000,0.00000,1.57000,0.00000,0.00000,-1.70000,0.00000,0.00000,0.00000,1.00000,1.00000,1.00000,1.00000};
               
            };
        }

    }
}

#endif

