/**
 * @file rearrangement_query.hpp
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

#ifndef PRX_REARRANGEMENT_QUERY_HPP
#define	PRX_REARRANGEMENT_QUERY_HPP

#include "planning/modules/path_part.hpp"

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/state.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"

#include <deque>
namespace prx
{
    namespace util
    {
        class parameter_reader_t;
    }

    namespace plan
    {
        class stopping_criteria_t;
    }

    namespace packages
    {
        namespace labeled_rearrangement_manipulation
        {

            /**
             * @anchor rearrangement_query_t
             *
             * This class represents a problem instance to be solved by a rearrangement task planner in the label rearrangement problems.
             * It is a self-contained class which holds the starts/goals poses for the objects.
             * It stores all the plans and the constraints of the results.
             *
             * @brief <b> General query for manipulator task planners in the rearrangement problem. </b>
             *
             * @author Athanasios Krontiris
             */
            class rearrangement_query_t : public plan::motion_planning_query_t
            {

              public:

                rearrangement_query_t();
                virtual ~rearrangement_query_t();

                /** @copydoc motion_planning_query_t::clear() */
                virtual void clear();
                
                virtual bool found_solution();
                
                /** @brief Safe state*/
                sim::state_t* start_state;
                /** @brief The ids correspond to the initial and target poses.*/
                std::vector<unsigned> ids;
                /** @brief The indices of he initial poses*/
                std::vector<unsigned> initial_poses_ids;
                /** @brief The indices of he target poses*/
                std::vector<unsigned> target_poses_ids;
                /** @brief The sequence of the final path*/
                std::deque<path_part_t> path_sequence;
                /** @brief The partial arrangement of the objects*/
                std::vector<unsigned> partial_solution;
                /** @brief If we can accept partial solutions or if we need the full solution*/
                bool accept_partial_solutions;                
                /** @brief If the algorithm found a solution*/
                bool got_solution;
                /** @brief If the algorithm found a partial solution*/
                bool got_partial_solution;                
                /** @bries The time that the algorithm has in order to give a solution*/
                double time_limit;
            };
        }
    }
}

#endif

