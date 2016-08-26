/**
 * @file apc_task_query.hpp
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

#ifndef PRX_APC_TASK_QUERY_HPP
#define	PRX_APC_TASK_QUERY_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/queries/query.hpp"
#include "prx/simulation/plan.hpp"

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
        class space_t;
    }

    namespace packages
    {
        namespace manipulation
        {
            class movable_body_plant_t;
        }
        namespace apc
        {


            /**
             * @anchor apc_task_query_t
             */
            class apc_task_query_t : public plan::query_t
            {

              public:

                enum apc_stage
                {
                    MOVE, MOVE_AND_DETECT, PERFORM_GRASP, MOVE_TO_ORDER_BIN, TEST_BIN_ROADMAP, EXECUTE_SAVED_TRAJ, ADJUST_EE, MOVE_TO_OTHER_BIN, MOVE_OUTSIDE_BIN, THREE_STAGE_TRAJECTORY_SECOND, THREE_STAGE_TRAJECTORY_THIRD, MOVE_AND_DETECT_TOTE, REMOVE_FROM_TOTE, RETRY_GRASP, LIFT, PLACE_INSIDE_BIN
                };

                apc_task_query_t();
                apc_task_query_t(util::space_t* control_space);

                virtual ~apc_task_query_t();
                virtual void setup(std::string in_hand, manipulation::movable_body_plant_t* requested_object, apc_stage in_stage, char which_bin, sim::state_t* goal);
                virtual void setup(std::string in_hand, manipulation::movable_body_plant_t* requested_object, apc_stage in_stage, char which_bin, sim::state_t* goal, std::vector<double> final_obj);

                virtual void clear();

                std::string hand;
                manipulation::movable_body_plant_t* object;
                apc_stage stage;
                char bin;
                sim::state_t* goal_state;
                std::vector<double> final_obj_state;

                sim::plan_t move_plan;
                sim::plan_t first_mapping_plan;
                sim::plan_t second_mapping_plan;
                sim::plan_t third_mapping_plan;
                sim::plan_t move_gripper_to_bin;
                sim::plan_t approach_object;
                sim::plan_t retrieve_object;
                sim::plan_t move_to_order_bin;

                bool found_solution;

            };
        }
    }
}

#endif

