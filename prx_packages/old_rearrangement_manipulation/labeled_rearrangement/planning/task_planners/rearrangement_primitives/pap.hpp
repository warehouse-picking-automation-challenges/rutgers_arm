/**
 * @file pap_t.hpp
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

#ifndef PRX_PAP_HPP
#define PRX_PAP_HPP

#include "planning/task_planners/rearrangement_primitive.hpp"

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/task_planners/task_planner.hpp"
#include "prx/simulation/plan.hpp"


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

            /**
             * The task planner for the pick and place primitive. Computes the path for rearranging labeled 
             * objects.
             * 
             * @autors Athanasios Krontiris
             */
            class pap_t : public rearrangement_primitive_t
            {

              public:

                pap_t();
                virtual ~pap_t();

              protected:
                /** @copydoc rearrangement_primitive_t::solve() */
                virtual bool solve();

                /**
                 * This algorithm will execute the mRS but with a single target. Each time the algortihm will check 
                 * multiple grasping states but will select the first one that will work. 
                 * It is faster but with worst path quality. 
                 * 
                 * @param object_id The Id of the object that we will move.
                 * @param from_states The states that we will move from towards going to grasp the object.
                 * @param pose The pose that we want to move the object.
                 * @param future All the objects that have not been moved yet. 
                 * @param a_curr The current arrangement of the objects.
                 * @param successful_index This variable it is not useful for this algorithm. 
                 * 
                 * @return True if the path is feasible, else false. 
                 */                 
                virtual bool pick_and_place(std::deque<path_part_t>& sequence, const std::vector<unsigned>& a_curr, const std::vector<unsigned>& target_arrangement);

                virtual bool pick_and_place2(std::deque<path_part_t>& sequence, const std::vector<unsigned>& a_curr, const std::vector<unsigned>& target_arrangement);

                virtual bool get_path(path_part_t& part, sim::state_t* safe_state, pose_t* start_pose, pose_t* end_pose);

                virtual int single_step(const std::vector<unsigned>& a_curr, const std::vector<unsigned>& target_arrangement);

                virtual int detect_released_state(pose_t* pose, sim::state_t* state);

                virtual int detect_grasped_state(pose_t* pose, sim::state_t* state);

                virtual void reset_constraints(const std::vector<unsigned>& arrangement);
                
            };
        }
    }
}


#endif	
