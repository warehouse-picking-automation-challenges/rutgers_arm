/**
 * @file quad_task.cpp 
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
#include "utilities/quad_task.hpp"
#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::packages::mapping_functions::quad_task_t, prx::util::mapping_function_t)

namespace prx
{
    namespace packages
    {
        namespace mapping_functions
        {
            void quad_task_t::embed() const
            {
                unsigned start_index = preimage_interval.first;
                unsigned image_start_index = image_interval.first;
                get_image_index(image_start_index) = get_preimage_index(start_index);
                get_image_index(image_start_index+1) = get_preimage_index(start_index+1);
                get_image_index(image_start_index+2) = get_preimage_index(start_index+2);
                quat.set_from_euler(get_preimage_index(start_index+3),
                                    get_preimage_index(start_index+4),
                                    get_preimage_index(start_index+5));
                get_image_index(image_start_index+3) = quat.get_x();
                get_image_index(image_start_index+4) = quat.get_y();
                get_image_index(image_start_index+5) = quat.get_z();
                get_image_index(image_start_index+6) = quat.get_w();

            }

            void quad_task_t::invert() const
            {
                unsigned start_index = preimage_interval.first;
                unsigned image_start_index = image_interval.first;
                quat.set(get_image_index(image_start_index+3),
                        get_image_index(image_start_index+4),
                        get_image_index(image_start_index+5),
                        get_image_index(image_start_index+6));
                quat.convert_to_euler(v);
                get_preimage_index(start_index) = get_image_index(image_start_index);
                get_preimage_index(start_index+1) = get_image_index(image_start_index+1);
                get_preimage_index(start_index+2) = get_image_index(image_start_index+2);
                get_preimage_index(start_index+3) = v[0];
                get_preimage_index(start_index+4) = v[1];
                get_preimage_index(start_index+5) = v[2];
            }

            void quad_task_t::init_spaces()
            {
                //subspace and preimage should be set
                if(subspace==NULL || preimage_space==NULL)
                    PRX_FATAL_S("Quad task space mapping doesn't have subspace or preimage_space");
                unsigned start_index = preimage_interval.first;
                //make the image space have the correct bounds information
                double low,high;
                preimage_space->get_bounds()[start_index]->get_bounds(low,high);
                subspace->get_bounds()[0]->set_bounds(low,high);
                preimage_space->get_bounds()[start_index+1]->get_bounds(low,high);
                subspace->get_bounds()[1]->set_bounds(low,high);
                preimage_space->get_bounds()[start_index+2]->get_bounds(low,high);
                subspace->get_bounds()[2]->set_bounds(low,high);


            }
        }
    }
}