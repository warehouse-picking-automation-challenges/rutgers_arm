/**
 * @file VO_sensing_info.cpp
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

#include "simulation/sensing/VO_sensing_info.hpp"

#include "prx/simulation/sensing/sensors/velocity_sensor.hpp"
#include "prx/simulation/sensing/sensors/config_sensor.hpp"
#include "prx/simulation/sensing/sensors/geometry_sensor.hpp"
#include "prx/simulation/systems/plants/plant.hpp"

#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>
#include <boost/range/adaptor/map.hpp>

 PLUGINLIB_EXPORT_CLASS(prx::packages::crowd::VO_sensing_info_t, prx::sim::sensing_info_t);

 namespace prx
 {
    using namespace util;
    using namespace sim;

    namespace packages
    {

        namespace crowd
        {
            double VO_sensing_info_t::update_time = 0;
            unsigned VO_sensing_info_t::update_calls = 0;

            VO_sensing_info_t::VO_sensing_info_t()
            {
                static_configs_retrieved = false;
                vel_sensor = NULL;
            }

            VO_sensing_info_t::~VO_sensing_info_t()
            {
            }

            void VO_sensing_info_t::init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader)
            {
                twoD_proximity_sensing_info_t::init(reader, template_reader);

                /** 3*max_num_neighbors since we could have max neighbors for
                 each of the following cases: obstacles, vo plants, and non-vo plants*/
                for(unsigned i = 0; i<max_num_neighbors; i++)
                {
                    all_neighbors.push_back(new neighbor_t());
                }
            }

            void VO_sensing_info_t::set_follow_controller( void* controller )
            {
                path_controller = (path_follow_controller_t*)controller;
            }

            void VO_sensing_info_t::set_sensors(const std::vector<sensor_t*>& sensors)
            {
                twoD_proximity_sensing_info_t::set_sensors(sensors);

                /** Find the relevant sensors */
                foreach(sensor_t* sensor, sensors)
                {
                    if (dynamic_cast<velocity_sensor_t*>(sensor) != NULL)
                        vel_sensor = dynamic_cast<velocity_sensor_t*>(sensor);
                }

                /** Make sure all sensors have been set, and that a valid system path is provided */
                PRX_ASSERT(vel_sensor != NULL);
            }

            void VO_sensing_info_t::update_info()
            {
                stats_clock.reset();
                current_neighbors.clear(); current_neighbors.resize(0);
                if( !path_controller->is_active() )
                {
                    return;
                }
                
                //Temp variables
                double distance = 0.0;

                //Clear out the proximity list
                prox_list.clear(); prox_list.resize(0);

                //And ask the config sensor where the plants are.
                const util::config_list_t& plant_configs = conf_sensor->get_plant_configs();

                //Alright, let's update the agent state with the new config information
                agent_state->memory[0] = plant_configs[system_index].second.get_position()[0];
                agent_state->memory[1] = plant_configs[system_index].second.get_position()[1];
                agent_state->memory[2] = plant_configs[system_index].second.get_position()[2];

                ///FIRST, WE NEED ELEVATOR SHAFTS
                prox_elem_t* near_elem = NULL;
                const nav_node_t* nav_node = path_controller->get_nearest_node();
                
                if( nav_node != NULL )
                {
                    elevator_t* e = nav_node->near_elevator;
                    if( e != NULL )
                    {
                        //If the elevator is not open at this height
                        if( !e->is_open() || fabs( e->current_height() - nav_node->point->memory[2] ) > 0.8 )
                        {
                            //Then we add it to the proximity list
                            prox_list.push_back( prox_elem_t( e->name, e->configuration, 0, e->shaft_geometry ) );
                        }
                    }
                }

                /// SECOND, NEAREST OBSTACLES
                if( nav_node != NULL )
                {
                    near_elem = nav_node->obstacle_info;
                    for( unsigned i=0; i<near_elem->distances.size() && prox_list.size() < max_num_neighbors/3.0; ++i )
                    {
                        prox_list.push_back( prox_elem_t( near_elem->config_names[i], near_elem->configs[i], near_elem->distances[i], near_elem->geometries[i] ) );
                    }
                }
                
                proximity_list_t agent_list;

                /// THEN, NEIGHBORING AGENTS
                //list of geoms for the plant
                geometry_t* input_geom;
                //For each of the plants
                for (unsigned i=0; i < plant_geoms.size(); i++)
                {
                    //If it is not our system
                    if ( i != system_index && all_plants[i]->is_active() )
                    {
                        //Find out how far away it is
                        transform_config = plant_configs[system_index].second;
                        transform_config -= plant_configs[i].second;
                        
                        //We should only care if they are on the same floor.
                        double z_diff = fabs( transform_config.get_position()[2] );
                        if( z_diff < 0.5 )
                        {
                            double x_dist = transform_config.get_position()[0];
                            double y_dist = transform_config.get_position()[1];

                            //Assume we're on the same plane if the z difference is small
                            transform_config.set_position( x_dist, y_dist, 0 );

                            double sys_rad = system_geometry->get_bounding_radius();
                            double o_rad = plant_geoms[ i ].get_bounding_radius();
                            distance = ( transform_config.get_position().norm() - ( sys_rad + o_rad ) );
                            //And if it is close by
                            if( distance < sensing_radius )
                            {
                                input_geom = &(plant_geoms[i]);
                                //Insert it into the prox list
                                if (agent_list.size() < 2.0*max_num_neighbors/3.0)
                                {
                                    agent_list.push_back(prox_elem_t(plant_configs[i].first, plant_configs[i].second, distance, input_geom));
                                }
                                else
                                {
                                    std::vector<prox_elem_t>::iterator it = std::max_element(agent_list.begin(), agent_list.end());
                                    prox_elem_t temp(plant_configs[i].first, plant_configs[i].second, distance, input_geom );
                                    if (temp < *it)
                                    {
                                        *it = temp;
                                    }
                                }
                            }
                        }
                    }
                }
                
                //Then, put the agents into the full prox list
                for( unsigned i=0; i<agent_list.size(); ++i )
                {
                    prox_list.push_back( agent_list[i] );
                }

                vo_neighbors.clear(); vo_neighbors.resize(0);
                obstacle_neighbors.clear(); obstacle_neighbors.resize(0);
                other_neighbors.clear(); other_neighbors.resize(0);
                const hash_t<std::string, vector_t>& velocities = vel_sensor->get_velocities();
                for (unsigned i = 0; i < prox_list.size(); i ++)
                {
                    std::string converted_path = confpath_to_syspath_map[prox_list[i].config_names[0]];
                    all_neighbors[i]->set_name(converted_path);
                    all_neighbors[i]->set_neighbor_center(prox_list[i].configs[0].get_position().at(0), prox_list[i].configs[0].get_position().at(1));
                    all_neighbors[i]->set_neighbor_geotype(prox_list[i].geometries[0]->get_type());
                    all_neighbors[i]->set_neighbor_radius(prox_list[i].geometries[0]->get_bounding_radius());

                    // The detected prox_list[i] is a vo plant
                    if (vo_plants_map.find(converted_path) != vo_plants_map.end())
                    {
                        all_neighbors[i]->set_obstacle_marker(false);
                        all_neighbors[i]->set_reciprocity(true);
                        all_neighbors[i]->set_valid(true);
                        all_neighbors[i]->set_vo_index(i);
                        all_neighbors[i]->set_neighbor_velocity(velocities[prox_list[i].config_names[0]].at(0), velocities[prox_list[i].config_names[0]].at(1));
                        vo_neighbors.push_back(all_neighbors[i]);
                    }
                    // The detected prox_list[i] is a non-vo plant
                    else if (other_plants_map.find(converted_path) != other_plants_map.end())
                    {
                        all_neighbors[i]->set_obstacle_marker(false);
                        all_neighbors[i]->set_reciprocity(false);
                        all_neighbors[i]->set_valid(true);
                        all_neighbors[i]->set_vo_index(i);
                        all_neighbors[i]->set_neighbor_velocity(velocities[prox_list[i].config_names[0]].at(0), velocities[prox_list[i].config_names[0]].at(1));
                        other_neighbors.push_back(all_neighbors[i]);

                    }
                    // The detected neighbor is an obstacle then
                    else
                    {
                        all_neighbors[i]->set_name(prox_list[i].config_names[0]);
                        all_neighbors[i]->set_obstacle_marker(true);
                        all_neighbors[i]->set_reciprocity(false);
                        all_neighbors[i]->set_valid(true);
                        all_neighbors[i]->set_vo_index(i);
                        all_neighbors[i]->set_neighbor_velocity(0,0);
                        obstacle_neighbors.push_back(all_neighbors[i]);
                    }

                    current_neighbors.push_back(all_neighbors[i]);
                }
                
                update_time += stats_clock.measure();
                ++update_calls;
            }

            void VO_sensing_info_t::get_info()
            {
                PRX_WARN_S ("Get info does not do anything here!");
            }

            void VO_sensing_info_t::set_system_path(const std::string& path)
            {
                system_path = path;
            }

            void VO_sensing_info_t::set_all_plants( const std::vector< sim::plant_t* >& plants )
            {
                all_plants = plants;
            }

            void VO_sensing_info_t::set_conf_to_sys_path_map(util::hash_t<std::string, std::string>& conf_sys_map)
            {
                confpath_to_syspath_map = conf_sys_map;
            }

            void VO_sensing_info_t::set_sensed_VO_plants(hash_t<std::string,plant_t*>& plants)
            {
                vo_plants_map = plants;
            }

            void VO_sensing_info_t::set_sensed_other_plants(hash_t<std::string,plant_t*>& plants)
            {
                other_plants_map = plants;
            }

            std::vector< neighbor_t*>& VO_sensing_info_t::get_current_neighbors()
            {
                return current_neighbors;
            }

            std::string VO_sensing_info_t::get_system_path()
            {
                return system_path;
            }
        }
    }
}

