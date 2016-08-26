/**
 * @file multi_shot_specification.hpp
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

#pragma once

#ifndef PRX_MULTI_SHOT_SPECIFICATION_HPP
#define	PRX_MULTI_SHOT_SPECIFICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/problem_specifications/specification.hpp"

namespace prx
{
    namespace plan
    {

        class multi_shot_specification_t : public specification_t
        {

          public:

            multi_shot_specification_t();

            virtual ~multi_shot_specification_t();

        };

    }
}

#endif
