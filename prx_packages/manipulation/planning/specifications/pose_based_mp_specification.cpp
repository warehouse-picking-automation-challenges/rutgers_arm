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

#include "planning/specifications/pose_based_mp_specification.hpp"

#include "prx/planning/world_model.hpp"
#include "planning/manipulation_world_model.hpp"

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::pose_based_mp_specification_t, prx::plan::specification_t)

namespace prx
{
    using namespace plan;
    
    namespace packages
    {
        namespace manipulation
        {
            pose_based_mp_specification_t::pose_based_mp_specification_t()
            {
                
            }
            
            pose_based_mp_specification_t::~pose_based_mp_specification_t()
            {
                
            }
            
            void pose_based_mp_specification_t::setup(world_model_t * const model)
            {
                manip_model = dynamic_cast< manipulation_world_model_t* >( model );
                motion_planning_specification_t::setup( model );
            }
            
            manipulation_world_model_t* pose_based_mp_specification_t::get_manipulation_model()
            {
                return manip_model;
            }
            
        }
    }
}


