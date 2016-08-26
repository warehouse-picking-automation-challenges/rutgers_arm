/**
 * @file mcr_test.hpp
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

#ifndef PRX_MCR_TEST_HPP
#define PRX_MCR_TEST_HPP

#include "planning/task_planners/rearrangement_primitive.hpp"
#include "planning/statistics/mcr_test_statistics.hpp"

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

            /**
             * The task planner for the forward rearrangement RSC algorithm. Computes the path for rearranging labeled 
             * objects.
             * 
             * @autors Athanasios Krontiris
             */
            class mcr_test_t : public rearrangement_primitive_t
            {

              public:

                mcr_test_t();
                virtual ~mcr_test_t();

                /** @copydoc motion_planner_t::init(const util::parameter_reader_t*, const util::parameter_reader_t*) */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                virtual const util::statistics_t* get_statistics();

              protected:
                /** @copydoc rearrangement_primitive_t::solve() */
                virtual bool solve();

                virtual bool get_path(unsigned id, double* computation_time, double* path_length, double* path_cost, double* num_constraints, bool extract_constraints = false);
                
                virtual bool get_transit_path(unsigned id, double* computation_time, double* path_length, double* path_cost, double* num_constraints, bool extract_constraints);
                
                virtual bool get_full_path(unsigned id, double* computation_time, double* path_length, double* num_constraints, bool extract_constraints = false);

                void reset_constraints(const std::vector<unsigned>& arrangement);

                int detect_released_state(pose_t* pose, sim::state_t* state);

                int detect_grasped_state(pose_t* pose, sim::state_t* state);

                std::vector<double> multipliers;
                mcr_test_statistics_t* mcr_stats;
                double transit_short_len;
                double transfer_short_len;
                unsigned index_from, index_to;
                bool transit_mode;
            };
        }
    }
}


#endif	
