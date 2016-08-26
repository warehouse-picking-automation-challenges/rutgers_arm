/**
 * @file VO_sensing_model.cpp
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

#include "prx/utilities/definitions/string_manip.hpp"

#include "simulation/sensing/VO_sensing_model.hpp"
#include "simulation/sensing/VO_sensing_info.hpp"

#include "prx/simulation/simulators/simulator.hpp"
#include "prx/simulation/system_graph.hpp"
#include "prx/simulation/systems/system.hpp"
#include "prx/simulation/systems/plants/plant.hpp"
#include "prx/simulation/systems/obstacle.hpp"
#include "prx/simulation/sensing/sensors/velocity_sensor.hpp"
#include "prx/simulation/sensing/sensors/config_sensor.hpp"
#include "prx/simulation/sensing/sensors/geometry_sensor.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/range/adaptor/map.hpp>

#define _USE_MATH_DEFINES
#include <math.h>

#include <sstream>
#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/assign/list_of.hpp>

PLUGINLIB_EXPORT_CLASS( prx::packages::crowd::VO_sensing_model_t, prx::sim::sensing_model_t)

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {
        namespace crowd
        {

            VO_sensing_model_t::VO_sensing_model_t()
            {
                vel_sensor = NULL;
            }

            VO_sensing_model_t::~VO_sensing_model_t()
            {
            }

            void VO_sensing_model_t::init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader)
            {
                proximity_sensing_model_t::init(reader,template_reader);
                foreach(sensor_t* sensor, all_sensors)
                {
                    if (dynamic_cast<velocity_sensor_t*>(sensor) != NULL)
                        vel_sensor = dynamic_cast<velocity_sensor_t*>(sensor);
                }

                /** Make sure all sensors have been set, and that a valid system path is provided */
                PRX_ASSERT(conf_sensor != NULL);
                PRX_ASSERT(vel_sensor != NULL);
                PRX_ASSERT(geom_sensor != NULL);
            }

            void VO_sensing_model_t::initialize_sensors(sim::simulator_t* sim)
            {
                proximity_sensing_model_t::initialize_sensors(sim);
                create_conf_to_sys_path_map(sim);
                simulator = sim;
            }

            void VO_sensing_model_t::update_sensed_state( const sim::state_t* new_state)
            {
                proximity_sensing_model_t::update_sensed_state(new_state);
            }

            void VO_sensing_model_t::sense()
            {
                proximity_sensing_model_t::sense();
            }

            void VO_sensing_model_t::set_sensing_info( const std::vector<sensing_info_t*>& sensing_info)
            {
                proximity_sensing_model_t::set_sensing_info(sensing_info);
                if( !(vo_plants_set && other_plants_set) )
                {
                    PRX_WARN_S("Plants have not been set for the sensing model, cannot complete initialization.");
                    return;
                }
                foreach(sensing_info_t* info, all_sensing_info)
                {
                    VO_sensing_info_t* vo_info = dynamic_cast<VO_sensing_info_t*>(info);
                    if (vo_info != NULL)
                    {
                        // PRX_DEBUG_COLOR ("Setting config to sys path map for : " << vo_info->get_system_path(), PRX_TEXT_BLUE);
                        vo_info->set_conf_to_sys_path_map(confpath_to_syspath_map);
                        vo_info->set_sensed_VO_plants(vo_plants_map);
                        vo_info->set_sensed_other_plants(other_plants_map);
                        vo_info->set_all_plants( all_plants );
                    }
                }
            }

            void VO_sensing_model_t::set_all_plants( const std::vector< plant_t* >& plants )
            {
                all_plants = plants;
            }

            void VO_sensing_model_t::set_sensed_VO_plants(hash_t<std::string, plant_t*>& plants)
            {
                vo_plants_map = plants;
                // foreach(std::string name, vo_plants_map | boost::adaptors::map_keys)
                // {
                //     PRX_DEBUG_COLOR("Name of VO plant in map: " << name, PRX_TEXT_LIGHTGRAY);
                // }
                vo_plants_set = true;
            }

            void VO_sensing_model_t::set_sensed_other_plants(hash_t<std::string, plant_t*>& plants)
            {
                other_plants_map = plants;
                // foreach(std::string name, other_plants_map | boost::adaptors::map_keys)
                // {
                //     PRX_DEBUG_COLOR("Name of other plant in map: " << name, PRX_TEXT_LIGHTGRAY);
                // }
                other_plants_set = true;
            }

            void VO_sensing_model_t::create_conf_to_sys_path_map(sim::simulator_t* sim)
            {

                system_graph_t graph;
                sim->update_system_graph(graph);
                std::vector<plant_t*> plants;
                graph.get_plants(plants);
                foreach(plant_t* plant, plants)
                {
                    std::string path = plant->get_pathname();
                    foreach(std::string name, *(plant->get_geometries_names()))
                    {
                        confpath_to_syspath_map[name] = path;
                    }
                }

                foreach(system_ptr_t sys, sim->get_obstacles() | boost::adaptors::map_values)
                {
                    obstacle_t* obst = dynamic_cast<obstacle_t*>(sys.get());
                    std::string path = obst->get_pathname();
                    foreach(std::string name, *(obst->get_geometries_names()))
                    {
                        confpath_to_syspath_map[name] = path;
                    }

                }
            }
        }
    }
}


