/**
 * @file end_effector_distance.hpp 
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

#ifndef PRX_END_EFFECTOR_DISTANCE_HPP
#define PRX_END_EFFECTOR_DISTANCE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/distance_functions/distance_function.hpp"

#include <boost/function.hpp>
#include <pluginlib/class_loader.h>

namespace prx
{
    namespace util
    {
        class space_point_t;
    }
    
    namespace packages
    {
        namespace manipulation
        {
            class manipulation_world_model_t;
            
            /**
             * A distance function which uses the end-effector positions to compute
             * workspace distances.
             * @brief <b> End-effector workspace distance. </b>
             * @author Andrew Dobson
             */
            class end_effector_distance_t : public util::distance_function_t
            {
              public:
                end_effector_distance_t();
                virtual ~end_effector_distance_t();
                
                void link_manipulation_model( manipulation_world_model_t* input_manip_model );

                virtual double distance(const util::space_point_t* s1, const util::space_point_t* s2);

              protected:
                manipulation_world_model_t* manip_model;
                
                const util::space_t* arm_space;
            };
        }
    }
}

#endif 
