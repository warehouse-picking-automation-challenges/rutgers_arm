/**
 * @file goal_to_direction.cpp
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

#include "prx/simulation/systems/controllers/converters/goal_to_direction.hpp"
#include <boost/assign/list_of.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::goal_to_direction_t, prx::sim::system_t)

namespace prx
{
    using namespace util;

    namespace sim
    {

        const unsigned int goal_to_direction_t::X = 0;
        const unsigned int goal_to_direction_t::Y = 1;
        

        const unsigned int goal_to_direction_t::V = 0;
        const unsigned int goal_to_direction_t::THETA = 1;
        
        goal_to_direction_t::goal_to_direction_t() 
        {
            control_memory = { &_Cx , &_Cy };
            input_control_space = new space_t( "XY", control_memory );
            converted_control = NULL;
        }

        goal_to_direction_t::~goal_to_direction_t() 
        {
            if (converted_control)
            {
                output_control_space->free_point(converted_control);
            }
        }

        void goal_to_direction_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            controller_t::init(reader, template_reader);
            max_vel = output_control_space->get_bounds().front()->get_upper_bound();
            converted_control = output_control_space->alloc_point();

        }

        void goal_to_direction_t::verify() const
        {
            controller_t::verify();
            PRX_ASSERT_MSG(output_control_space->get_space_name() == "Vector", "Must have a vector output control space for convert");
        }

        void goal_to_direction_t::compute_control()
        {
            double difx = (*input_control_space)[X] - (*state_space)[X];
            double dify = (*input_control_space)[Y] - (*state_space)[Y];

            double magn = sqrt(difx * difx + dify * dify);

            if( magn == 0 )
            {
                (*converted_control)[V] = 0;
                (*converted_control)[THETA] = 0;
            }
            else
            {
                difx = (difx * max_vel) / magn;
                dify = (dify * max_vel) / magn;
                (*converted_control)[V] = sqrt(difx * difx + dify * dify);;
                (*converted_control)[THETA] = atan2(dify, difx);
            }
            PRX_DEBUG_COLOR("Converted control: " <<(*input_control_space)[X] << ", " << (*input_control_space)[Y] << " to: " << output_control_space->print_point(converted_control), PRX_TEXT_RED);
            output_control_space->copy_from_point(converted_control);
            controller_t::compute_control();
        }


    }
}
