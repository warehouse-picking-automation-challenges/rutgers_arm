/**
 * @file grasp_descriptor.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2016, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zacharias Psarakis, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#pragma once

#ifndef PRX_GRASP_DESCRIPTOR_HPP
#define PRX_GRASP_DESCRIPTOR_HPP

#include "prx/utilities/math/3d_geometry/trimesh.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/math/configurations/vector.hpp"
#include "prx/utilities/math/configurations/config.hpp"

namespace prx 
{ 
    namespace packages 
    {
        namespace manipulation
        {
            
            struct grasp_surface_t
            {
                util::trimesh_t mesh;
                util::trimesh_t sample_mesh;
                double total_area;
                std::vector< double > face_areas;
                util::vector_t normal;
                std::vector< double > triangles;
                std::vector< double > sample_triangles;
                unsigned index;
            };

            struct grasp_volume_t
            {
                //These are just going to be cylinders
                double height;
                double radius;
                double volume;
                util::config_t relative_config;
                unsigned index;
            };

            class grasp_descriptor_t
            {
              public:
                grasp_descriptor_t();
                ~grasp_descriptor_t();
                
                void init( const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader );
                
                void sample_volume( unsigned index, util::vector_t& sampled_point ) const;
                void sample_surface( unsigned index, util::vector_t& sampled_point ) const;
            
                const std::vector< grasp_volume_t* >& get_volumes( std::string end_effector = "" ) const;
                const std::vector< grasp_surface_t* >& get_surfaces( std::string end_effector = "" ) const;
            
              protected:
            
                std::vector< grasp_volume_t* > volumes;
                std::vector< grasp_surface_t* > surfaces;

                std::vector< grasp_surface_t* > empty_surfaces;
                std::vector< grasp_volume_t* > empty_volumes;

                double surface_scaling_factor;
                
                util::hash_t< std::string, std::vector< grasp_volume_t* > > volume_map;
                util::hash_t< std::string, std::vector< grasp_surface_t* > > surface_map;
            };

        }
    }
}

#endif //PRX_GRASP_DESCRIPTOR_HPP

