/**
 * @file multi_shot_query.cpp
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

#include "prx/planning/task_planners/multi_shot/multi_shot_query.hpp"

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS(prx::plan::multi_shot_query_t, prx::plan::query_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    void multi_shot_query_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
    {
        motion_query_t::init( reader, template_reader );

        //Need to get the goals to test        
        foreach(const parameter_reader_t* reader, parameters::get_list("goal_states", reader, template_reader))
        {
            goal_vectors.push_back(reader->get_attribute_as< std::vector<double> >("state"));
        }
        
    }
}

