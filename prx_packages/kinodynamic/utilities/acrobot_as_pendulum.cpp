/**
 * @file acrobot_as_pendulum.hpp 
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
#include "utilities/acrobot_as_pendulum.hpp"
#include "prx/utilities/spaces/space.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::packages::distance_functions::acrobot_as_pendulum_t, prx::util::distance_function_t)

namespace prx
{
    using namespace util;
    namespace packages
    {       
        namespace distance_functions
        {
            double acrobot_as_pendulum_t::distance(const space_point_t* p1, const space_point_t* p2)
            {
                double* s1 = const_cast<double*>(&p1->memory[0]);
                double* s2 = const_cast<double*>(&p2->memory[0]);
                double ddot = fabs(s1[2] - s2[2]);
                double ddot2 = fabs(s1[3] - s2[3]);
                return fabs(sin(s2[0]) + sin(s2[0] + s2[1]) - (sin(s1[0]) + sin(s1[0] + s1[1])))
                        + fabs(cos(s2[0]) + cos(s2[0] + s2[1]) - (cos(s1[0]) + cos(s1[0] + s1[1])))
                        + .8 * ddot2
                        + .2 * ddot;
            }
        }
    }
}
