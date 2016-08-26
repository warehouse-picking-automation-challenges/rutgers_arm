/**
 * @file position_from_rigid_body_mapping.cpp 
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

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "utilities/position_from_rigid_body_mapping.hpp"

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::packages::mapping_functions::position_from_rigid_body_mapping_t, prx::util::mapping_function_t)

namespace prx
{
    namespace packages
    {        
        namespace mapping_functions
        {
            void position_from_rigid_body_mapping_t::embed() const
            {
                unsigned start_index = preimage_interval.first;
                unsigned image_start_index = image_interval.first;
                get_image_index(image_start_index) = get_preimage_index(start_index);
                get_image_index(image_start_index + 1) = get_preimage_index(start_index + 1);
                get_image_index(image_start_index + 2) = get_preimage_index(start_index + 2);
            }

            void position_from_rigid_body_mapping_t::invert() const
            {
                unsigned start_index = preimage_interval.first;
                unsigned image_start_index = image_interval.first;
                get_preimage_index(start_index) = get_image_index(image_start_index);
                get_preimage_index(start_index + 1) = get_image_index(image_start_index + 1);
                get_preimage_index(start_index + 2) = get_image_index(image_start_index + 2);
            }

            void position_from_rigid_body_mapping_t::init_spaces()
            {
                //subspace and preimage should be set
                if( subspace == NULL || preimage_space == NULL )
                    PRX_FATAL_S("Position from rigid body mapping doesn't have subspace or preimage_space");
                unsigned start_index = preimage_interval.first;
                //make the image space have the correct bounds information
                double low = -15;
                double high = 15;
                // preimage_space->get_bounds()[start_index]->get_bounds(low, high);
                subspace->get_bounds()[0]->set_bounds(low, high);

                // preimage_space->get_bounds()[start_index + 1]->get_bounds(low, high);
                subspace->get_bounds()[1]->set_bounds(low, high);

                // preimage_space->get_bounds()[start_index + 2]->get_bounds(low, low);
                subspace->get_bounds()[2]->set_bounds(low, high);

            }
        }
    }
}