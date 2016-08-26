/**
 * @file motion_query.cpp
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

#include "prx/planning/queries/motion_query.hpp"

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::plan::motion_query_t, prx::plan::query_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace plan
    {

        motion_query_t::motion_query_t()
        {
            state_space = NULL;
            control_space = NULL;
        }

        motion_query_t::~motion_query_t()
        {
        }

        void motion_query_t::clear()
        {
            plan.clear();
            path.clear();
        }

        void motion_query_t::link_spaces(const space_t* state_space, const space_t* control_space)
        {
            this->state_space = state_space;
            this->control_space = control_space;         
            plan.link_control_space(control_space);
            path.link_space(state_space);
        }
        
        util::space_point_t* motion_query_t::get_solution_final_state()
        {
            if( path.size() != 0 )
            {
                return path.back();
            }
            return NULL;
        }

    }
}
