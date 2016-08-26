/**
 * @file nmrs.hpp
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

#ifndef PRX_NMRS_HPP
#define PRX_NMRS_HPP

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
            class nmrs_t : public rearrangement_primitive_t
            {

              public:

                nmrs_t();
                virtual ~nmrs_t();

                /** @copydoc motion_planner_t::init(const util::parameter_reader_t*, const util::parameter_reader_t*) */
                //virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

              protected:
                
                /** @copydoc rearrangement_primitive_t::init(const util::parameter_reader_t* , const util::parameter_reader_t* ) */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                /** @copydoc rearrangement_primitive_t::solve() */
                virtual bool solve();

                virtual bool nmrs(std::deque<path_part_t>& sequence, unsigned object_id, sim::state_t* from_state, std::deque<unsigned> future, std::vector<unsigned> a_curr, const std::vector<unsigned>& target_arrangement);
                
                /**
                 * A helping function to avoid typing the same lines over and over again. 
                 * 
                 * @param arrangement The assignement with the current poses that will be the current constraints.
                 */
                virtual void fix_constraints(unsigned object_id, const std::vector<unsigned>& arrangement, const std::deque<unsigned>& future, const std::set<unsigned>& constraints);
               
                virtual sim::state_t* clear_blocker(std::deque<path_part_t>& sequence, unsigned object_id, sim::state_t* from_state, const std::deque<unsigned>& future, std::vector<unsigned>& a_curr, const std::vector<unsigned>& target_arrangement, const std::set<unsigned>& constraints);

                /**
                 * 
                 * @param plan
                 * @param constraints
                 * @param full_constraints
                 * @param ordered_constraints
                 * @param from_states
                 * @param start_pose
                 * @param end_pose
                 * 
                 * @return The index of the grasp state that has worked and the union path will end.
                 */
                virtual std::pair<int, int> get_union_path(std::set<unsigned>& constraints, std::set<unsigned>& full_constraints, std::set<unsigned>& transfer_full_constraints, std::vector<unsigned>& ordered_constraints, sim::state_t* from_state, pose_t* start_pose, pose_t* end_pose);
                virtual std::pair<int, int> get_union_path2(std::set<unsigned>& constraints, std::set<unsigned>& full_constraints, std::set<unsigned>& transfer_full_constraints, std::vector<unsigned>& ordered_constraints, sim::state_t* from_state, pose_t* start_pose, pose_t* end_pose);

                /**
                 * 
                 * @return Return the state where the manipulator will end after executing the plan. 
                 */
                bool intermediate_pose(path_part_t& part, std::set<unsigned>& path_constraints, std::set<unsigned>& transfer_full_constraints, std::vector<unsigned>& ordered_constraints, unsigned object_id, sim::state_t* from_state, const std::deque<unsigned>& future, const std::vector<unsigned>& a_curr, const std::vector<unsigned>& target_arrangement, const std::set<unsigned>& constraints);

                virtual unsigned detect_blocker(unsigned object_id, const std::vector<unsigned>& a_curr, const std::vector<unsigned>& constraints);

                /**
                 * Given the poses_set and a ungrasped state, it will return the corresponding grasped point. 
                 * 
                 * @param poses The poses that we will search through.
                 * @param state The state that we need to much with the grasped state.
                 * 
                 * @return The first integer is the index of the pose inside the vector of poses, while the second one is the index of 
                 * the state inside the pose_t struct.
                 */
                virtual std::pair<int, int> detect_pose(const std::vector<pose_t*>& poses, sim::state_t * state);

                virtual void get_valid_intermediate_poses(std::vector<pose_t*>& free_poses, const std::vector<unsigned>& a_curr, const std::deque<unsigned>& future, const std::set<unsigned>& constraints);

                /**
                 * The start and the goal for the queries has to be in place. This function will call the shortest path in order to compute the 
                 * correct max length for the current step. 
                 */
                void fix_transit_query();
                void fix_transfer_query();

                double length_multiplier;
            };
        }
    }
}


#endif	

