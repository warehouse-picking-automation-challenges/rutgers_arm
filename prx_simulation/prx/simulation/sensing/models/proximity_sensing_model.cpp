/**
 * @file proximity_sensing_model.cpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Kimmel, Andrew Dobson, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/simulation/sensing/models/proximity_sensing_model.hpp"
#include "prx/simulation/sensing/sensors/config_sensor.hpp"
#include "prx/simulation/sensing/sensors/geometry_sensor.hpp"
#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::proximity_sensing_model_t, prx::sim::sensing_model_t);
namespace prx
{
    using namespace util;
    
    namespace sim
    {
        
        proximity_sensing_model_t::proximity_sensing_model_t() 
        {
            conf_sensor = NULL;
            geom_sensor = NULL;
        }

        proximity_sensing_model_t::~proximity_sensing_model_t() 
        {

        }
        
        void proximity_sensing_model_t::init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader)
        {
            sensing_model_t::init(reader, template_reader);
            foreach(sensor_t* sensor, all_sensors)
            {
                if (dynamic_cast<config_sensor_t*>(sensor) != NULL)
                    conf_sensor = dynamic_cast<config_sensor_t*>(sensor);
                else if (dynamic_cast<geometry_sensor_t*>(sensor) != NULL)
                    geom_sensor = dynamic_cast<geometry_sensor_t*>(sensor);
            }

            /** Make sure all sensors have been set, and that a valid system path is provided */
            PRX_ASSERT_MSG(conf_sensor != NULL, "Proximity sensing model must have valid config sensor");
            PRX_ASSERT_MSG(geom_sensor != NULL, "Proximity sensing model must have valid geom sensor");
                
            
        }

        void proximity_sensing_model_t::update_sensed_state ( const state_t* new_state)
        {
            sensing_model_t::update_sensed_state (new_state);
            
        }
        
        void proximity_sensing_model_t::sense()
        {
            sensing_model_t::sense();
        }

   

    }
    
}
