/**
 * @file subset_mapping.cpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/spaces/mapping_functions/subset_mapping.hpp"

#include <pluginlib/class_list_macros.h> 

namespace prx
{
    namespace util
    {        

        void subset_mapping_t::embed() const
        {
            for(unsigned i=0;i<mapped_indices.size();i++)
            {
                get_image_index(i) = get_preimage_index(mapped_indices[i]);
            }
        }

        void subset_mapping_t::invert() const
        {
            for(unsigned i=0;i<mapped_indices.size();i++)
            {
                get_preimage_index(mapped_indices[i]) = get_image_index(i);
            }
        }

        void subset_mapping_t::init_spaces()
        {            
            for(unsigned i=0;i<range;i++)
            {
                memory.push_back(preimage_space->addresses[mapped_indices[i]]);
            }
            subspace = new space_t(output_space_name,memory);
            image_space = subspace;

            std::vector<bounds_t*> bounds = preimage_space->get_bounds();
            std::vector<double*> scales = preimage_space->get_scales();
            std::vector<double> inscales;
            std::vector<bounds_t*> inbounds; 
            for(unsigned i=0;i<range;i++)
            {
                inscales.push_back(*scales[mapped_indices[i]]);
                inbounds.push_back(bounds[mapped_indices[i]]);
            }
            subspace->set_bounds(inbounds);
            subspace->set_scales(inscales);
        }


    }
}