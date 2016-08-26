/**
 * @file simple_place_specification.hpp
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

#ifndef PRX_SIMPLE_PLACE_SPECIFICATION_HPP
#define PRX_SIMPLE_PLACE_SPECIFICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/problem_specifications/specification.hpp"

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
    }
    
    namespace packages
    {
        namespace manipulation
        {
            /**
             * @anchor simple_place_specification_t
             *
             * This class represents a problem for the simple pick and place task planner.
             *
             * @brief <b> Specification for simple_place_tp. </b>
             *
             * @author Athanasios Krontiris
             */
            class simple_place_specification_t : public plan::specification_t
            {

              public:
                simple_place_specification_t();
                virtual ~simple_place_specification_t();

                /**
                 * @brief Initialize the specification from input.
                 *
                 * @param reader The reader for the parameters.
                 * @param template_reader A template reader for reading template parameters 
                 */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);
                
                plan::validity_checker_t* validity_checker;
              protected:
                
            };
        }
    }
}

#endif

