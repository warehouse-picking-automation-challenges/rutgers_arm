/**
 * @file end_effector_distance.cpp 
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

#include "prx/utilities/spaces/space.hpp"

#include "planning/distance_functions/end_effector_distance.hpp"
#include "planning/manipulation_world_model.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::end_effector_distance_t, prx::util::distance_function_t);

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;
    
    namespace packages
    {
        namespace manipulation
        {
            end_effector_distance_t::end_effector_distance_t()
            {
                manip_model = NULL;
                arm_space = NULL;
            }
            
            end_effector_distance_t::~end_effector_distance_t()
            {
                
            }
            
            void end_effector_distance_t::link_manipulation_model( manipulation_world_model_t* input_manip_model )
            {
                manip_model = input_manip_model;
                PRX_ASSERT( manip_model != NULL );
                arm_space = manip_model->get_state_space();
            }

            double end_effector_distance_t::distance(const space_point_t* p1, const space_point_t* p2)
            {
                config_t config_one;
                config_t config_two;
                
                //Get the EE position for the first state
                arm_space->copy_from_point( p1 );
                manip_model->FK( config_one );

                //Get the EE position for the second state
                arm_space->copy_from_point( p2 );
                manip_model->FK( config_two );
                
                //This is set up to replicate how distances are done in the space
                const vector_t& pos_one = config_one.get_position();
                const vector_t& pos_two = config_two.get_position();

                const quaternion_t& quat_one = config_one.get_orientation();
                const quaternion_t& quat_two = config_two.get_orientation();
                
                double distance = 0;
                distance += (pos_one[0] - pos_two[0]) * (pos_one[0] - pos_two[0]);
                distance += (pos_one[1] - pos_two[1]) * (pos_one[1] - pos_two[1]);
                distance += (pos_one[2] - pos_two[2]) * (pos_one[2] - pos_two[2]);
                //Debug: try scaling down orientation's effect a lot
                distance += 0.2 * quat_one.distance( quat_two );

                return std::sqrt( distance );
            }
            
        }
    }
}
