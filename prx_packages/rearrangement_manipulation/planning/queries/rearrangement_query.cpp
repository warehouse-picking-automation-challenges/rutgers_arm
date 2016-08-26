/**
 * @file manipulation_query.cpp
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

#include "planning/queries/rearrangement_query.hpp"

#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::rearrangement_manipulation::rearrangement_query_t, prx::plan::query_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace rearrangement_manipulation
        {

            rearrangement_query_t::rearrangement_query_t()
            {
                got_solution = false;
                got_partial_solution = false;
            }

            rearrangement_query_t::~rearrangement_query_t()
            {
                clear();
            }

            void rearrangement_query_t::clear()
            {
                motion_planning_query_t::clear();
                path_sequence.clear();
                partial_solution.clear();
                got_solution = false;
                got_partial_solution = false;
                time_limit = 0;
            }

            bool rearrangement_query_t::found_solution()
            {
                return got_solution || (got_partial_solution && accept_partial_solutions);
            }
        }
    }
}