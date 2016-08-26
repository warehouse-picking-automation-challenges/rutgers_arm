/**
 * @file motion_planning_specification.hpp
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

#ifndef PRX_POSE_BASED_MOTION_PLANNING_SPECIFICATION_HPP
#define PRX_POSE_BASED_MOTION_PLANNING_SPECIFICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"

namespace prx
{
    namespace plan
    {
        class world_model_t;
    }
    
    namespace packages
    {
        namespace manipulation
        {
            class manipulation_world_model_t;
            
            /**
             * @anchor pose_based_mp_specification_t
             *
             * @brief <b> Motion Planning specification for doing pose-based queries. </b>
             *
             * @author Andrew Dobson
             */
            class pose_based_mp_specification_t : public plan::motion_planning_specification_t
            {

              public:
                pose_based_mp_specification_t();
                virtual ~pose_based_mp_specification_t();

                /**
                 * Prepare the planning specification to be linked to the children
                 * 
                 * @brief Prepare the planning specification to be linked to the children
                 */
                virtual void setup( plan::world_model_t * const model );
            
                manipulation_world_model_t* get_manipulation_model();
            
              protected:
                manipulation_world_model_t* manip_model;
            };
        }
    }
}

#endif

