/**
 * @file restricted_normals_sampler.cpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2016, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */


#include "prx/planning/modules/samplers/restricted_normals_sampler.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/math/configurations/bounds.hpp"

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::plan::restricted_normals_sampler_t, prx::plan::sampler_t)

namespace prx
{
    using namespace util;
    namespace plan
    {        


        void restricted_normals_sampler_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
        {
            default_axis.resize(3); rotated_axis.resize(3); converted_euler.resize(3);

            default_axis = parameters::get_attribute_as<vector_t>("default_axis", reader, template_reader);

        }

        void restricted_normals_sampler_t::sample(const space_t* space, space_point_t* point)
        {
            do
            {
                random_quaternion.random();
                rotated_axis = random_quaternion.qv_rotation(default_axis);
            }
            while (rotated_axis[0] < 0 || rotated_axis[2] > 0);


            space->uniform_sample(point);

            unsigned dimension = space->get_dimension();

            random_quaternion.convert_to_euler(converted_euler);
            point->memory[dimension-3] = converted_euler[0];
            point->memory[dimension-2] = converted_euler[1];
            point->memory[dimension-1] = converted_euler[2];

            // PRX_FATAL_S("Point: " << space->print_point(point,3) << ", Axis: " << rotated_axis);
        }

        void restricted_normals_sampler_t::sample_near(const space_t* space, space_point_t* near_point, std::vector<bounds_t*>& bounds, space_point_t* point)
        {
            do
            {
                random_quaternion.random();
                rotated_axis = random_quaternion.qv_rotation(default_axis);
            }
            while (rotated_axis[0] < 0 || rotated_axis[2] > 0);


            space->uniform_sample_near(point, near_point, bounds);

            unsigned dimension = space->get_dimension();

            random_quaternion.convert_to_euler(converted_euler);
            point->memory[dimension-3] = converted_euler[0];
            point->memory[dimension-2] = converted_euler[1];
            point->memory[dimension-1] = converted_euler[2];

        }

    }
}