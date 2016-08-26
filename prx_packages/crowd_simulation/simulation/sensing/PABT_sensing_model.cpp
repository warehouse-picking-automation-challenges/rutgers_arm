/**
 * @file PABT_sensing_model.cpp
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

#include "simulation/sensing/PABT_sensing_model.hpp"

#include "simulation/sensing/navigation_graph_sensor.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/range/adaptor/map.hpp>

#define _USE_MATH_DEFINES
#include <math.h>

#include <sstream>
#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/assign/list_of.hpp>

PLUGINLIB_EXPORT_CLASS( prx::packages::crowd::PABT_sensing_model_t, prx::sim::sensing_model_t )

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {
        namespace crowd
        {

            PABT_sensing_model_t::PABT_sensing_model_t()
            {
                graph_sensor = NULL;
            }

            PABT_sensing_model_t::~PABT_sensing_model_t()
            {
            }

            void PABT_sensing_model_t::init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader)
            {
                sensing_model_t::init(reader,template_reader);
                foreach(sensor_t* sensor, all_sensors)
                {
                    if (dynamic_cast<navigation_graph_sensor_t*>(sensor) != NULL)
                        graph_sensor = dynamic_cast<navigation_graph_sensor_t*>(sensor);
                }
                PRX_ASSERT(graph_sensor != NULL);
            }

            void PABT_sensing_model_t::initialize_sensors(sim::simulator_t* sim)
            {
                sensing_model_t::initialize_sensors(sim);
            }

            void PABT_sensing_model_t::update_sensed_state( const sim::state_t* new_state)
            {
                sensing_model_t::update_sensed_state(new_state);
            }

            void PABT_sensing_model_t::sense()
            {
                sensing_model_t::sense();
            }
            
        }
    }
}


