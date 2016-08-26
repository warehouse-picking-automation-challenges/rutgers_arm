/**
 * @file mcr.hpp
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

#ifndef PRX_MCR_HPP
#define PRX_MCR_HPP

#include "planning/task_planners/rearrangement_search_algorithm.hpp"
#include "planning/statistics/mcr_test_statistics.hpp"
#include "prx/utilities/statistics/statistics.hpp"

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

            class rearrangement_search_query_t;           
            class rearrangement_search_specification_t;
            class rearrangement_primitive_t;
            class rearrangement_primitive_specification_t;
            class rearrangement_query_t;
            class rearrangement_path_planner_t;

            /**
             * 
             * 
             * @autors Athanasios Krontiris
             */
            class mcr_t : public rearrangement_search_algorithm_t
            {

              public:
                mcr_t();
                virtual ~mcr_t();

                /** @copydoc task_planner_t::setup() */
                virtual void setup();

                /** @copydoc task_planner_t::resolve_query() */
                virtual void resolve_query();
                
                /** @copydoc task_planner_t::get_statistics() */
                const util::statistics_t* get_statistics();
                
                const mcr_test_statistics_t* mcr_stats;
            };
        }

    }

}
#endif	
