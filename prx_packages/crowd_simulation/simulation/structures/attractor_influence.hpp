/**
 * @file attractor_influence.hpp
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

#ifndef PRX_ATTRACTOR_INFLUENCE_HPP
#define PRX_ATTRACTOR_INFLUENCE_HPP

#include "prx/utilities/math/configurations/vector.hpp"

#include <vector>

namespace prx
{

    namespace packages
    {
        namespace crowd
        {
            class attractor_t;
            
            class attractor_influence_t
            {

              public:                
                attractor_influence_t( int type, int frame, double influence );
                ~attractor_influence_t();

                int type;
                int frame;
                double influence;
            };

            bool operator<(const attractor_influence_t &s1, const attractor_influence_t &s2);
        }
    }
}

#endif

