/**
 * @file apc_task_planner.hpp
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

#pragma once

#ifndef PRX_APC_TP_HPP
#define	PRX_APC_TP_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/spaces/space.hpp"

#include "prx/simulation/plan.hpp"
#include "prx/simulation/trajectory.hpp"

#include "prx/planning/task_planners/task_planner.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"

#include "planning/apc_task_query.hpp"

#include "planning/manipulation_world_model.hpp"
#include "planning/queries/manipulation_query.hpp"
#include "planning/task_planners/manipulation_tp.hpp"
#include "utilities/definitions/manip_defs.hpp"

// #include "prx/simulation/state.hpp"

#include <boost/progress.hpp>

namespace prx
{
    namespace packages
    {
        namespace apc
        {
            
            struct data_ik
            {
                double plan_length;
                double interpolation_length;
                bool collision_free;
                double error;
                double ee_distance;
                bool success;
                double time_elapsed;
                double interpolation_time;
                double collision_time;
                
                data_ik()
                {
                    plan_length = 0;
                    interpolation_length = 0;
                    interpolation_time = 0;
                    success = false;
                    error = 0;
                    ee_distance = 0;
                    collision_free = false;
                    time_elapsed = 0;
                    collision_time = 0;
                }
            };
            
            class examination_profile_t
            {
            public:
                std::vector<double> focus;
                util::quaternion_t base_viewpoint;
                std::vector<util::quaternion_t> offsets;
                double distance;

            };

            /**
             * A task planner for performing loops of the Amazon Picking Challenge
             *
             * @authors Zakary Littlefield
             */
            class apc_task_planner_t : public plan::task_planner_t
            {

              public:

                apc_task_planner_t();
                virtual ~apc_task_planner_t();

                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                /** @copydoc motion_planner_t::link_world_model(world_model_t* const) */
                void link_world_model(plan::world_model_t * const model);

                /** @copydoc planner_t::link_query(query_t*) */
                virtual void link_query(plan::query_t* new_query);

                /** @copydoc motion_planner_t::setup() */
                virtual void setup();

                /** @copydoc motion_planner_t::execute() */
                virtual bool execute();

                /** @copydoc motion_planner_t::succeeded() const */
                virtual bool succeeded() const;

                /** @copydoc motion_planner_t::get_statistics() */
                virtual const util::statistics_t* get_statistics();

                virtual void resolve_query();

              protected:

                bool move_and_detect();
                bool move_and_detect_second_stage();
                bool move_and_detect_third_stage();
                bool perform_grasp();
                bool move_to_order_bin();
                bool move();
                bool adjust_ee();
                bool move_to_other_bin();
                bool move_outside_bin();
                bool move_left_hand(sim::state_t* initial_state_left, sim::state_t* left_goal);
                bool move_right_hand(sim::state_t* initial_state_right, sim::state_t* right_goal);
                bool move_and_detect_tote();
                bool remove_from_tote();
                bool place_inside_bin();
                bool retry_grasp();

                manipulation::manipulation_world_model_t* manipulation_model;
                manipulation::manipulator_t* manipulator;
                sim::state_t* manip_initial_state;
                const util::space_t* full_manipulator_state_space;
                const util::space_t* full_manipulator_control_space;

                /** @brief The input manipulation query */
                manipulation::manipulation_query_t* left_manipulation_query;
                manipulation::manipulation_query_t* right_manipulation_query;
                manipulation::manipulation_query_t* manipulation_query;
                manipulation::manipulation_tp_t* manip_planner;

                apc_task_query_t* in_query;

                std::string full_manipulator_context_name;
                std::string left_context_name;
                std::string right_context_name;
                std::string left_full_context_name;
                std::string right_full_context_name;
                std::string left_camera_context_name;
                std::string left_camera_jac_context_name;
                std::string right_camera_context_name;
                std::string right_camera_jac_context_name;
                std::string manipulation_task_planner_name;

                util::hash_t<char,examination_profile_t*> camera_positions;
                std::vector<double> left_arm_order_bin;
                std::vector<double> right_arm_order_bin;
                std::vector<double> left_arm_home;
                std::vector<double> right_arm_home;
                bool bin_trajectories;
                bool smooth_paths;
                bool jk_prm_star;

                float total_resolve_time, total_move_time;
                int total_tries;
                int total_failures;
                int failedmanualretr1;
                int failedmanualretr2;
                int transferfailed;
                int movefailed;
                float first_stage_offset;

            };
        }
    }
}


#endif
