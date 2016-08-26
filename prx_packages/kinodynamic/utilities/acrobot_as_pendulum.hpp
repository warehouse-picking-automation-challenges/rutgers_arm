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
#pragma once

#ifndef PRX_ACROBOT_AS_PENDULUM_HPP
#define PRX_ACROBOT_AS_PENDULUM_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/distance_functions/distance_function.hpp"

namespace prx 
{ 
    namespace packages 
    {
        namespace distance_functions
        {
            /**
            * A class containing a wrapper around the space distance function.
            * @brief <b> A class containing a wrapper around the space distance function. </b>
            * @author Zakary Littlefield 
            */
            class acrobot_as_pendulum_t : public util::distance_function_t
            {
                public:
                    acrobot_as_pendulum_t(){}
                    ~acrobot_as_pendulum_t(){}

                    virtual double distance(const util::space_point_t* s1, const util::space_point_t* s2);
            };
        }
    } 
}

#endif 