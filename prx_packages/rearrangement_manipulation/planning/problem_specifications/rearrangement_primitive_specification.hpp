/**
 * @file rearrangement_primitive_specification.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#pragma once

#ifndef PRX_REARRANGEMENT_PRIMITIVE_SPECIFICATION_HPP
#define PRX_REARRANGEMENT_PRIMITIVE_SPECIFICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/problem_specifications/specification.hpp"

#include "planning/modules/pose.hpp"

namespace prx
{

    namespace packages
    {
        namespace rearrangement_manipulation
        {

            /**
             * @anchor rearrangement_primitive_specification_t
             *
             * 
             * @Author Athanasios Krontiris
             */
            class rearrangement_primitive_specification_t : public plan::specification_t
            {

              public:

                rearrangement_primitive_specification_t();

                virtual ~rearrangement_primitive_specification_t();
                                
                unsigned k_objects;
                bool gather_statistics;
                std::string transit_graph_file;
                std::string transfer_graph_file;
                std::string poses_file;
                std::vector<double> safe_position;
            };
        }
    }
}

#endif
