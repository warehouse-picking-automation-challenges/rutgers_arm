/**
 * @file fmrs_t.hpp
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

#ifndef PRX_FMRS_HPP
#define PRX_FMRS_HPP

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
            class fmrs_t : public rearrangement_primitive_t
            {

              public:

                fmrs_t();
                virtual ~fmrs_t();

                /** @copydoc motion_planner_t::init(const util::parameter_reader_t*, const util::parameter_reader_t*) */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                virtual void setup();

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
                virtual bool fmrs(std::deque<path_part_t>& sequence, const std::vector<unsigned>& a_curr, const std::vector<unsigned>& target_arrangement);

                virtual bool partial_fmrs(std::deque<path_part_t>& sequence, util::directed_graph_t constraint_graph, const std::vector<unsigned>& a_curr, const std::vector<unsigned>& target_arrangement);

                virtual bool get_path(path_part_t& part, sim::state_t* safe_state, pose_t* start_pose, pose_t* end_pose);

                virtual bool get_test_path(path_part_t& part, sim::state_t* safe_state, pose_t* start_pose, pose_t* end_pose);

                virtual bool get_full_path(path_part_t& part, sim::state_t* safe_state, pose_t* start_pose, pose_t* end_pose);

                virtual bool construct_constraint_graph(util::directed_graph_t& graph, const std::vector<unsigned>& object_ids, const std::vector<unsigned>& a_curr, const std::vector<unsigned>& target_arrangement);

                virtual bool get_super_order(std::vector<util::directed_vertex_index_t>& order, util::directed_graph_t& graph);

                virtual int detect_released_state(pose_t* pose, sim::state_t* state);

                virtual int detect_grasped_state(pose_t* pose, sim::state_t* state);


                bool exact_method;
                bool multi_start;

                //==========================//
                //         mRS Code         //
                //==========================//
                virtual void reset_constraints(const std::vector<unsigned>& arrangement);
                virtual bool mrs(std::deque<path_part_t>& sequence, unsigned object_id, const std::vector<sim::state_t*>& from_states, pose_t& pose, std::deque<unsigned> future, std::vector<unsigned> a_curr, const std::vector<unsigned>& target_arrangement);
                virtual bool clear_to_move(unsigned object_id, const pose_t& pose, const std::vector<unsigned>& arrangement);
                virtual std::pair<int, int> get_union_path(sim::plan_t& plan, const std::vector<sim::state_t*>& from_states, pose_t* start_pose, pose_t* end_pose);
              
                std::vector<sim::state_t*> from_safe_state;
                
                int max_depth;
                int curr_depth;
                std::deque<path_part_t> curr_order;
                std::deque<path_part_t> best_order;
                
            };
        }
    }
}


#endif	
