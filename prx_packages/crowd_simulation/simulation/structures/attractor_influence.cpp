/**
 * @file attractor_influence.cpp
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

#include "simulation/structures/attractor_influence.hpp"

namespace prx
{
    using namespace util;

    namespace packages
    {
        namespace crowd
        {

            attractor_influence_t::attractor_influence_t( int type, int frame, double influence)
            {
                this->type = type;
                this->frame = frame;
                this->influence = influence;
            }

            attractor_influence_t::~attractor_influence_t()
            {
            }

            bool operator<(const attractor_influence_t &s1, const attractor_influence_t &s2)
            {
                if(s1.frame < s2.frame)
                    return true;
                return false;
            }
        }
    }
}

