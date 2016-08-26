/**
 * @file simple_pap_specification.hpp
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

#ifndef APC_DRIVER_STRESS_SPECIFICATION_HPP
#define APC_DRIVER_STRESS_SPECIFICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
// #include "prx/planning/problem_specifications/specification.hpp"
#include "planning/specifications/manipulation_specification.hpp"

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
    }
    namespace plan
    {
        class sampler_t;
    }
    namespace packages
    {
        namespace apc
        {
            /**
             * @anchor apc_driver_stress_specification
             *
             * This class represents a problem for the apc driver stress task planner
             *
             * @brief <b> Specification for simple_pap_tp. </b>
             *
             * @author Athanasios Krontiris
             */
            class apc_driver_stress_specification_t : public prx::packages::manipulation::manipulation_specification_t
            {

              public:
                apc_driver_stress_specification_t();
                virtual ~apc_driver_stress_specification_t();

                /**
                 * @brief Initialize the specification from input.
                 *
                 * @param reader The reader for the parameters.
                 * @param template_reader A template reader for reading template parameters 
                 */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);
                /** @brief Random sampling module. */
                // void apc_driver_stress_specification_t::setup(plan::world_model_t * const model);
                plan::sampler_t* sampler;
              protected:
                
            };
        }
    }
}

#endif

