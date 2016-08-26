/**
 * @file mrs_t.hpp
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

#ifndef PRX_MRS_HPP
#define PRX_MRS_HPP

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
             * The task planner for the forward rearrangement RSC algorithm. Computes the path for rearranging labeled 
             * objects.
             * 
             * @autors Athanasios Krontiris
             */
            class mrs_t : public rearrangement_primitive_t
            {

              public:

                mrs_t();
                virtual ~mrs_t();

                /** @copydoc motion_planner_t::init(const util::parameter_reader_t*, const util::parameter_reader_t*) */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);


              protected:
                /** @copydoc rearrangement_primitive_t::solve() */
                virtual bool solve();
                
                /**
                 * A helping function to avoid typing the same lines over and over again. 
                 * 
                 * @param arrangement The assignement with the current poses that will be the current constraints.
                 */
                virtual void reset_constraints(const std::vector<unsigned>& arrangement);

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
                virtual bool mrs(std::deque<path_part_t>& sequence, unsigned object_id, const std::vector<sim::state_t*>& from_states, pose_t& pose, std::deque<unsigned> future, std::vector<unsigned> a_curr, const std::vector<unsigned>& target_arrangement);

                /**
                 * Checks if the pose that the manipulator will move the object is not blocked by other objects.
                 *  
                 * @param object_id The id of the object that we are going to move.
                 * @param pose The pose that the object will be moved.
                 * @param arrangement The current arrangement;
                 * 
                 * @return True if the pose is clear, False otherwise.
                 */
                virtual bool clear_to_move(unsigned object_id, const pose_t& pose, const std::vector<unsigned>& arrangement);

                /**
                 * Compute a path from a initial state to a state that will grasp an object from a pose (transit phase). 
                 * Then computes and appends the path that will transfer the object to its target pose (transfer phase). 
                 * 
                 * @param plan The union plan of Transit and Transfer phase.
                 * @param from_states The set of states that the algorithm will check as initial states. In this version there 
                 *                    is only one state.
                 * @param start_pose The pose from where the manipulator will pick the object.
                 * @param end_pose The pose that the manipulator has to take the object. 
                 * 
                 * @return A pair with the two indices that work during the computation of the union path. The first index is the 
                 * one for the poses from where we got the object, while the second one will be the index of the point we left the object. 
                 */
                virtual std::pair<int, int> get_union_path(sim::plan_t& plan, std::set<unsigned>& full_constraints, const std::vector<sim::state_t*>& from_states, pose_t* start_pose, pose_t* end_pose);
              
                bool multiple_targets;       
                int max_depth;
                int curr_depth;
                std::deque<path_part_t> curr_order;
                std::deque<path_part_t> best_order;
                
            };
        }
    }
}


#endif	
