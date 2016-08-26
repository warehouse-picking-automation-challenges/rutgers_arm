/**
 * @file rearrangement_path_planner_t.hpp
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

#ifndef PRX_REARRANGEMENT_PATH_PLANNER_HPP
#define PRX_REARRANGEMENT_PATH_PLANNER_HPP

#include "planning/task_planners/rearrangement_primitive.hpp"

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/task_planners/task_planner.hpp"
#include "prx/simulation/plan.hpp"

#include <list>
#include <sstream>

namespace prx
{
    namespace util
    {
        class bounds_t;
        class multiple_goal_states_t;
        class statistics_t;
    }

    namespace plan
    {
        class motion_planning_query_t;
    }

    namespace packages
    {

        namespace labeled_rearrangement_manipulation
        {
            using namespace baxter;
            using namespace manipulation;
            
            class rearrangement_path_statistics_t;

            /**
             * The task planner for the forward rearrangement RSC algorithm. Computes the path for rearranging labeled 
             * objects.
             * 
             * @autors Athanasios Krontiris
             */
            class rearrangement_path_planner_t : public rearrangement_primitive_t
            {

              public:

                rearrangement_path_planner_t();
                virtual ~rearrangement_path_planner_t();

                /** @copydoc task_planner_t::init(const util::parameter_reader_t*, const util::parameter_reader_t*) */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);
                
                virtual void setup();
                
              protected:
                /** @copydoc rearrangement_primitive_t::solve() */
                virtual bool solve(); 
                
                
                /**
                 * The final step where we are combining all the parts to one plan. 
                 * 
                 * @param plan The final plan that will move the objects.
                 * @param parts The parts that will build the plan.
                 * @param a_curr The current arrangement of the objects.
                 * 
                 * @return True if the rearrangement_path_planner manage to combine the path. False, otherwise.
                 */
                virtual bool combine_path(sim::plan_t& plan, std::deque<path_part_t>& parts, std::vector<unsigned> a_curr);

                virtual bool combine_path_with_basic_smoothing(sim::plan_t& plan, std::deque<path_part_t>& parts, std::vector<unsigned> a_curr);

                virtual bool combine_path_with_IK(sim::plan_t& plan, std::deque<path_part_t>& parts, std::vector<unsigned> a_curr);

                virtual bool move_with_IK(sim::plan_t& plan, const util::config_t& start_config, const util::config_t& end_config, const sim::state_t* start_state, const sim::state_t* goal_state, bool set_grasping);

                virtual bool move_with_local_planner(sim::plan_t& plan, const sim::state_t* start, const sim::state_t* goal);

                virtual bool basic_smoothing(sim::plan_t& plan, const sim::state_t* start, const sim::state_t* goal);

                virtual bool smooth_forward(sim::plan_t& plan, const sim::state_t* start, sim::trajectory_t& path);
                
                virtual bool smooth_backward(sim::plan_t& plan, const sim::state_t* goal, sim::trajectory_t& path);

                /**
                 * This function checks if there are two points that can come closer. It is used from \c bring_parts_together.
                 *
                 * @param parts The deque with the parts;
                 * @param id The id of the part that we are looking to move
                 * @param start_index The index inside the list to start looking. Its the positions after the element with the id.
                 *
                 * @return Returns a pair with the first element will be True if we have to move a point else False. The second
                 *         element is from where we will move the point to the start_index position. 
                 */
                virtual std::pair<bool, unsigned> swap(std::deque<path_part_t>& parts, unsigned id, unsigned start_index);
                
                virtual std::pair<bool, unsigned> swap_backward(std::deque<path_part_t>& parts, unsigned id, unsigned start_index);
                
                virtual bool bring_parts_together(std::deque<path_part_t>& parts);
                
                virtual bool remove_same_objects(std::deque<path_part_t>& parts);
                
                virtual bool validate_full_path();

                virtual std::string print_parts(const std::deque<path_part_t> parts);

                //For checking collisions between the manipulator, active objects and static objects.
                std::string pc_name_manip_collision_checking;
                
                /** @brief If we will apply smoothing or just path planning*/
                bool apply_smoothing;
                /** @brief If we will add sensing we have to stop the path at the retraction point*/
                bool sensing;
                /** @brief If the time is over*/
                bool time_ends;
                /** @brief tmp list with the parts */
                std::deque<path_part_t> tmp_parts;

                sim::plan_t tmp_plan;
                sim::plan_t smoothed_plan;
                sim::trajectory_t tmp_path;
                bool store_trajectory;
                bool is_motoman;
                bool IK_cheating;
            };
        }
    }
}


#endif	
