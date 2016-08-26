/**
 * @file manhattan_distance.cpp 
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
#include "utilities/distance_functions/poses_distance.hpp"
#include "prx/utilities/spaces/space.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::packages::distance_functions::poses_distance_t, prx::util::distance_function_t)

namespace prx
{
    using namespace util;
    namespace packages
    {       
        namespace distance_functions
        {
            double poses_distance_t::distance(const space_point_t* p1, const space_point_t* p2)
            {
                double* s1 = const_cast<double*>(&p1->memory[0]);
                double* s2 = const_cast<double*>(&p2->memory[0]);
                
                unsigned dim = p1->memory.size();
                double val = 0;
                for(unsigned i=0;i<dim;i+=2)
                {
                    val += sqrt((s1[i]-s2[i])*(s1[i]-s2[i]) + (s1[i+1]-s2[i+1])*(s1[i+1]-s2[i+1]));
                }
                return val;
            }
        }
    }
}
