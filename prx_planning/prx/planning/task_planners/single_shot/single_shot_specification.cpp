/**
 * @file single_shot_specification.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "prx/planning/task_planners/single_shot/single_shot_specification.hpp"

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS(prx::plan::single_shot_specification_t, prx::plan::specification_t)

namespace prx
{
    using namespace util;
    namespace plan
    {
        single_shot_specification_t::single_shot_specification_t()
        {
        }

        single_shot_specification_t::~single_shot_specification_t()
        {
        }
    }
}
