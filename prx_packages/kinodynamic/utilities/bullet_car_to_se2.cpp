/**
 * @file bullet_car_to_se2.cpp 
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

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "utilities/bullet_car_to_se2.hpp"
#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::packages::mapping_functions::bullet_car_to_se2_t, prx::util::mapping_function_t)

namespace prx
{
    namespace packages
    {        
        namespace mapping_functions
        {
            void bullet_car_to_se2_t::embed() const
            {
                unsigned start_index = preimage_interval.first;
                unsigned image_start_index = image_interval.first;
                get_image_index(image_start_index) = get_preimage_index(start_index);
                get_image_index(image_start_index + 1) = get_preimage_index(start_index + 1);
                get_image_index(image_start_index + 2) = get_preimage_index(start_index + 2);
                q.set(get_preimage_index(start_index + 3), get_preimage_index(start_index + 4), get_preimage_index(start_index + 5), get_preimage_index(start_index + 6));
                q.convert_to_euler(v);
                get_image_index(image_start_index + 3) = v[2];
            }

            void bullet_car_to_se2_t::invert() const
            {
                unsigned start_index = preimage_interval.first;
                unsigned image_start_index = image_interval.first;
                get_preimage_index(start_index) = get_image_index(image_start_index);
                get_preimage_index(start_index + 1) = get_image_index(image_start_index + 1);
                get_preimage_index(start_index + 2) = get_image_index(image_start_index + 2);
                v[0] = v[1] = 0;
                v[2] = 1;
                q.set(v, get_image_index(image_start_index + 3));
                get_preimage_index(start_index + 3) = q[0];
                get_preimage_index(start_index + 4) = q[1];
                get_preimage_index(start_index + 5) = q[2];
                get_preimage_index(start_index + 6) = q[3];
            }

            void bullet_car_to_se2_t::init_spaces()
            {
                //subspace and preimage should be set
                if( subspace == NULL || preimage_space == NULL )
                    PRX_FATAL_S("Bullet car to SE2 doesn't have subspace or preimage_space");
                unsigned start_index = preimage_interval.first;
                //make the image space have the correct bounds information
                double low, high;
                preimage_space->get_bounds()[start_index]->get_bounds(low, high);
                subspace->get_bounds()[0]->set_bounds(-200, 200);

                preimage_space->get_bounds()[start_index + 1]->get_bounds(low, high);
                subspace->get_bounds()[1]->set_bounds(-200, 200);

                preimage_space->get_bounds()[start_index + 2]->get_bounds(low, high);
                subspace->get_bounds()[2]->set_bounds(-1, 10);

                subspace->get_bounds()[3]->set_bounds(-PRX_PI, PRX_PI);

            }
        }
    }
}