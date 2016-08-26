/**
 * @file manipulation_specification.hpp
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

#ifndef PRX_MANIPULATION_SPECIFICATION_HPP
#define PRX_MANIPULATION_SPECIFICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/problem_specifications/specification.hpp"

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
    }
    
    namespace plan
    {
        class validity_checker_t;
    }
        
    namespace packages
    {
        namespace manipulation
        {

            /**
             * @anchor manipulation_specification_t
             *
             * This class represents a problem instance to be solved by a motion planner.
             * It is a self-contained class which holds the start/goal pair as well as has
             * storage for answering trajectories and plans.
             *
             * @brief <b> General query for motion planners. </b>
             *
             * @author Athanasios Krontiris
             */
            class manipulation_specification_t : public plan::specification_t
            {

              public:
                manipulation_specification_t();
                virtual ~manipulation_specification_t();

                /**
                 * @brief Initialize the planing query from input parameters.
                 *
                 * Initialize the planing query from input parameters.
                 * 
                 * @param reader The reader for the parameters.
                 * @param template_reader A template reader for reading template parameters 
                 */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                /**
                 * @brief Clear the plan and trajectory of this query.
                 *
                 * Clears the plan and the trajectory of the planning query in order to 
                 * reuse the planning query with the same start and stopping criterion. 
                 */
                virtual void clear();

                /**
                 * Prepare the planning specification to be linked to the children
                 * 
                 * @brief Prepare the planning specification to be linked to the children
                 */
                virtual void setup(plan::world_model_t * const model);

                /** @brief State validity checker. */
                plan::validity_checker_t* validity_checker;

              protected:
                
            };
        }
    }
}

#endif

