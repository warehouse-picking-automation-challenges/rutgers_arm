/**
 * @file sensor.cpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/simulation/sensing/sensor.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

namespace prx
{
    using namespace util;
    
    namespace sim
    {

        pluginlib::ClassLoader<sensor_t> sensor_t::loader("prx_simulation", "prx::sim::sensor_t");
        
        sensor_t::sensor_t()
        {
            //Say the last time it sensed was the beginning of the simulation.
            last_sense_time.fromSec( 0.0 );
        }

        sensor_t::~sensor_t()
        {
        }
        
        void sensor_t::init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader)
        {
            source = parameters::get_attribute_as<std::string>("source", reader, template_reader, "no_source");
            sensor_delay = parameters::get_attribute_as<double>("sensor_delay", reader, template_reader, -1);
            periodic_sensing = parameters::get_attribute_as< bool >("periodic_sensing", reader, template_reader, true);
        }
        
        bool sensor_t::fires()
        {
            if(!periodic_sensing)
            {
                return false;
            }
            //Okay, so just need the logic here which appropriately handles things?
            ros::Duration duration_since_update = simulation::simulation_time - last_sense_time;

            if( sensor_delay <= duration_since_update.toSec() )
            {
                last_sense_time = simulation::simulation_time;
                return true;
            }
            return false;
        }
        
        pluginlib::ClassLoader<sensor_t>& sensor_t::get_loader()
        {
            return loader;
        }
        
        std::string sensor_t::get_source()
        {
            return source;
        }
        
        void sensor_t::reset_delay()
        {
            last_sense_time = simulation::simulation_time;
        }
        
        void sensor_t::set_periodic_sensing(bool flag)
        {
            periodic_sensing = flag;
        }

        double sensor_t::get_sensor_delay()
        {
            return sensor_delay;
        }
        
        
        
    }
}

