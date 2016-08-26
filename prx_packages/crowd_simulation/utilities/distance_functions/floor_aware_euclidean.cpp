/**
 * @file floor_aware_euclidean.hpp
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
#include "utilities/distance_functions/floor_aware_euclidean.hpp"
#include "prx/utilities/spaces/space.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::packages::crowd::floor_aware_euclidean_t, prx::util::distance_function_t)

namespace prx
{
    using namespace util;

    namespace packages
    {
        namespace crowd
        {
            void floor_aware_euclidean_t::link_space(const space_t* space)
            {
                ref_space = space;
                this->dist = boost::bind(&distance_function_t::distance, this, _1, _2);
                PRX_ASSERT( space->get_dimension() == 3 );
            }

            double floor_aware_euclidean_t::distance(const space_point_t* s1, const space_point_t* s2)
            {
                const double& x1 = s1->memory[0];
                const double& x2 = s2->memory[0];
                const double& y1 = s1->memory[1];
                const double& y2 = s2->memory[1];

                double difx = (x1 - x2);
                double dify = (y1 - y2);
                double euclid_dist = sqrt( difx*difx + dify*dify );
                return fabs( s1->memory[2] - s2->memory[2] ) < 1.5 ? euclid_dist : 9999;
            }
        }
    }
}
