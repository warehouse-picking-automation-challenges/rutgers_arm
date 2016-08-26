/**
 * @file minimal_avoidance_controller.cpp 
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "prx/simulation/systems/controllers/minimal_avoidance_controller.hpp"
#include "prx/simulation/sensing/infos/2d_proximity_sensing_info.hpp"
//#include "simulation/controllers/VO_controller.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::minimal_avoidance_controller_t, prx::sim::system_t)

namespace prx
{
    using namespace util;

    namespace sim
    {

       minimal_avoidance_controller_t::minimal_avoidance_controller_t()
        {
           prox_info = NULL;
        }

        minimal_avoidance_controller_t::~minimal_avoidance_controller_t() 
        { 
        }

        void minimal_avoidance_controller_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            simple_controller_t::init(reader,template_reader);
            obstacle_distance = parameters::get_attribute_as<double>("obstacle_distance", reader, template_reader);
            plant_distance = parameters::get_attribute_as<double>("plant_distance", reader, template_reader);
            for(unsigned i=0; i < this->sensing_info.size() && prox_info == NULL; ++i)
            {
                prox_info = dynamic_cast<twoD_proximity_sensing_info_t*>(sensing_info[i]);
            }
            
            PRX_ASSERT_MSG(prox_info != NULL, "Minimal avoidance controller needs a proximity sensing info");
            
            contingency_plan.link_control_space(output_control_space);
            subsystems.begin()->second->append_contingency(contingency_plan, 0.0);
            input_control_space = output_control_space;

        }

        void minimal_avoidance_controller_t::compute_control()
        {
            /** Check proximity info */
            if (prox_info->get_closest_obstacle_distance() < obstacle_distance || prox_info->get_closest_plant_distance() < plant_distance)
            {
                
                output_control_space->copy_from_point(contingency_plan.get_control_at(0));
                PRX_WARN_S ("Executing safety control: " << output_control_space->print_memory());
            }

            subsystems.begin()->second->compute_control();
        }
    }
}
