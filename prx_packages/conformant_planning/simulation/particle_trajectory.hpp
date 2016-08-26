/**
 * @file particle_trajectory.hpp 
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#pragma once


#ifndef PRACSYS_PARTICLE_TRAJECTORY_HPP
#define PRACSYS_PARTICLE_TRAJECTORY_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/trajectory.hpp"

namespace prx 
{ 
    namespace packages
    {
        namespace conformant
        {
            class particle_trajectory_t : public sim::trajectory_t
            {
                public:
                    particle_trajectory_t();
                    virtual ~particle_trajectory_t() 
                    {
                        
                    }
                    std::vector<bool> collisions;

            };
        }
    }
}

#endif