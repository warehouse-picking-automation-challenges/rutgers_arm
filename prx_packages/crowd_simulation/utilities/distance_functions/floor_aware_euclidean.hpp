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
#pragma once

#ifndef PRX_FLOOR_AWARE_EUCLIDEAN_HPP
#define	PRX_FLOOR_AWARE_EUCLIDEAN_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/distance_functions/distance_function.hpp"

namespace prx
{
    namespace packages
    {
        namespace crowd
        {
            class floor_aware_euclidean_t : public util::distance_function_t
            {
            public:
                floor_aware_euclidean_t(){}
                ~floor_aware_euclidean_t(){}

                virtual void link_space(const util::space_t* space);

                virtual double distance(const util::space_point_t* s1, const util::space_point_t* s2);

            };
        }
    }
}

#endif
