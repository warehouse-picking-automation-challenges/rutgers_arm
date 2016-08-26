/**
 * @file neighbor_sensing_info.cpp
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

#include "prx/utilities/parameters/parameter_reader.hpp"

#include "simulation/sensing/neighbor_sensing_info.hpp"
#include "simulation/sensing/navigation_graph_sensor.hpp"

#include "prx/simulation/systems/plants/plant.hpp"

#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::crowd::neighbor_sensing_info_t, prx::sim::sensing_info_t);

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {

        namespace crowd
        {
            double neighbor_sensing_info_t::update_time = 0;
            unsigned neighbor_sensing_info_t::update_calls = 0;

            neighbor_sensing_info_t::neighbor_sensing_info_t()
            {
                nav_sensor = NULL;
            }

            neighbor_sensing_info_t::~neighbor_sensing_info_t()
            {
            }

            void neighbor_sensing_info_t::init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader)
            {
                sensing_info_t::init(reader, template_reader);
            }

            void neighbor_sensing_info_t::set_sensors(const std::vector<sensor_t*>& sensors)
            {
                sensing_info_t::set_sensors(sensors);

                /** Find the relevant sensors */
                foreach(sensor_t* sensor, sensors)
                {
                    nav_sensor = dynamic_cast< navigation_graph_sensor_t* >(sensor);
                }

                /** Make sure all sensors have been set, and that a valid system path is provided */
                PRX_ASSERT(nav_sensor != NULL);
            }

            bool neighbor_sensing_info_t::updates()
            {
                ros::Duration duration_since_update = simulation::simulation_time - last_update_time;

                if( update_delay <= duration_since_update.toSec() || duration_since_update.toSec() == 0)
                {
                    reset_delay();
                    return true;
                }
                return false;
            }

            void neighbor_sensing_info_t::update_info()
            {
                nav_sensor->get_neighbors( agent_index, neighborhood );
            }

            void neighbor_sensing_info_t::get_info()
            {
                PRX_WARN_S ("Get info does not do anything here!");
            }
            
            unsigned neighbor_sensing_info_t::get_max_num_neighbors()
            {
                return nav_sensor->get_max_num_neighbors();
            }
        }
    }
}

