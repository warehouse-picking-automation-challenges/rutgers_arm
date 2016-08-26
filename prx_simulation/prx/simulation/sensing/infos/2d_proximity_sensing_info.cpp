/**
 * @file twoD_proximity_sensing_info.cpp
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

#include "prx/simulation/sensing/infos/2d_proximity_sensing_info.hpp"
#include "prx/simulation/sensing/sensors/config_sensor.hpp"
#include "prx/simulation/sensing/sensors/geometry_sensor.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/math/2d_geometry/line.hpp"
#include "prx/utilities/graph/abstract_node.hpp"

#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>
#include <boost/range/adaptor/map.hpp>
#include <boost/assign/list_of.hpp>

PLUGINLIB_EXPORT_CLASS(prx::sim::twoD_proximity_sensing_info_t, prx::sim::sensing_info_t);

namespace prx 
{
    using namespace util;
    using namespace sim;

    namespace sim
    {

        twoD_proximity_sensing_info_t::twoD_proximity_sensing_info_t()
        {
            conf_sensor = NULL;
            geom_sensor = NULL;
            system_geometry = NULL;
            static_configs_retrieved = false;

            plant_geoms.resize(0);
            obstacle_geoms.resize(0);

            //Let's create the navigation space (Hack for now)
            navigation_space = new space_t( "XYZ", {&_Nx,&_Ny,&_Nz});
            agent_state = navigation_space->alloc_point();
        }

        twoD_proximity_sensing_info_t::~twoD_proximity_sensing_info_t()
        {
            if (system_geometry != NULL)
                delete system_geometry;

            for( unsigned i=0; i<pq_nodes.size(); ++i )
            {
                navigation_space->free_point( pq_nodes[i].point );
            }

            navigation_space->free_point( agent_state );
            delete navigation_space;
        }

        void twoD_proximity_sensing_info_t::init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader)
        {
            sensing_info_t::init(reader, template_reader);
            system_path = parameters::get_attribute_as<std::string>("system_path", reader, template_reader, "");
            // PRX_DEBUG_COLOR("Set system path as: " << system_path, PRX_TEXT_MAGENTA);
            max_num_neighbors = parameters::get_attribute_as<unsigned>("max_num_neighbors", reader, template_reader, 5);
            sensing_radius = parameters::get_attribute_as<double>("sensing_radius", reader, template_reader, 2.0);
            //duplicated so that it can sense obstacles within a different radius
            obstacle_sensing_radius = parameters::get_attribute_as<double>("obstacle_sensing_radius", reader, template_reader, 2.0);

            if( parameters::has_attribute("distance_metric", reader, template_reader) )
            {
                metric = parameters::initialize_from_loader< distance_metric_t >("prx_utilities", reader, "distance_metric", template_reader, "distance_metric");
            }
            else
            {
                PRX_FATAL_S("Missing distance_metric attribute in 2D Proximity sensing info!");
            }

            metric->link_space( navigation_space );
        }

        void twoD_proximity_sensing_info_t::set_sensors(const std::vector<sensor_t*>& sensors)
        {
            /** Find the relevant sensors */
            foreach(sensor_t* sensor, sensors)
            {
                if (dynamic_cast<config_sensor_t*>(sensor) != NULL)
                    conf_sensor = dynamic_cast<config_sensor_t*>(sensor);
                else if (dynamic_cast<geometry_sensor_t*>(sensor) != NULL)
                    geom_sensor = dynamic_cast<geometry_sensor_t*>(sensor);
            }

            /** Make sure all sensors have been set, and that a valid system path is provided */
            PRX_ASSERT(conf_sensor != NULL);
            PRX_ASSERT(geom_sensor != NULL);
            PRX_ASSERT(!system_path.empty());

            //If we have never gotten the configuration of the static obstacles
            if( !static_configs_retrieved && conf_sensor != NULL )
            {
                // PRX_PRINT("Retrieving Static configurations!", PRX_TEXT_GREEN);
                //Report that we have the static information for the geometries
                static_configs_retrieved = true;

                //We need to get all the plant configs
                const util::config_list_t& plant_configs = conf_sensor->get_plant_configs();

                //Get the obstacle configs and the plant configs
                conf_sensor->update_data();
                obstacle_configs = conf_sensor->get_obstacle_configs();

                //Then, get the actual geometry information from the simulation
                const util::geom_map_t& geom_map = geom_sensor->get_sensed_geometries();

                //Get all of the plant geometries
                foreach(std::string name, plant_configs | boost::adaptors::map_keys)
                {
                    // PRX_PRINT("Got plant geom: " << name << "  of type: " << geom_map[name].get_type(), PRX_TEXT_CYAN );
                    plant_geoms.push_back( geom_map[name] );
                }
                //Then, get all of the obstacle geometries
                foreach(std::string name, obstacle_configs | boost::adaptors::map_keys)
                {
                    // PRX_PRINT("Got obstacle geom: " << name << "  of type: " << geom_map[name].get_type(), PRX_TEXT_LIGHTGRAY );
                    obstacle_geoms.push_back( geom_map[name] );
                }

                //Then we can compute all of the relevant points for the metric
                // compute_static_obstacle_vertices();

                //Also, we should at this point keep track of our system's configuraiton
                if( !find_system_configuration_index( plant_configs ) )
                {
                    PRX_FATAL_S("Unable to find system [" << system_path << "] configuration.");
                }
            }
        }

        void twoD_proximity_sensing_info_t::update_info()
        {
            //Temp variables
            double distance = 0.0;

            //Clear out the proximity list
            prox_list.clear();

            //And ask the config sensor where the plants are.
            const util::config_list_t& plant_configs = conf_sensor->get_plant_configs();

            //Alright, let's update the agent state with the new config information
            agent_state->memory[0] = plant_configs[system_index].second.get_position()[0];
            agent_state->memory[1] = plant_configs[system_index].second.get_position()[1];
            agent_state->memory[2] = plant_configs[system_index].second.get_position()[2];

            /// NEIGHBORING AGENTS
            //list of geoms for the plant
            geometry_t* input_geom;
            //For each of the plants
            for (unsigned i=0; i < plant_geoms.size(); i++)
            {
                //If it is not our system
                if ( i != system_index )
                {
                    //Find out how far away it is
                    transform_config = plant_configs[system_index].second;
                    transform_config -= plant_configs[i].second;
                    double sys_rad = system_geometry->get_bounding_radius();
                    double o_rad = plant_geoms[ i ].get_bounding_radius();
                    distance = ( transform_config.get_position().norm() - ( sys_rad + o_rad ) );
                    //And if it is close by
                    if( distance < sensing_radius )
                    {
                        input_geom = &(plant_geoms[i]);
                        //Insert it into the prox list
                        if (prox_list.size() < max_num_neighbors)
                        {
                            prox_list.push_back(prox_elem_t(plant_configs[i].first, plant_configs[i].second, distance, input_geom));
                        }
                        else
                        {
                            std::vector<prox_elem_t>::iterator it = std::max_element(prox_list.begin(), prox_list.end());
                            prox_elem_t temp(plant_configs[i].first, plant_configs[i].second, distance, input_geom );
                            if (temp < *it)
                            {
                                *it = temp;
                            }
                        }
                    }
                }
            }

            //Then, we will do a query of the metric to see what obstacles are nearby
            std::vector< const abstract_node_t* > obstacle_points;
            obstacle_points = metric->radius_query( agent_state, obstacle_sensing_radius + system_geometry->get_bounding_radius() );

            //Now that we have all the points, we must get the unique obstacles they represent
            sensed_obstacle_indices.clear();
            sensed_obstacle_distances.clear();
            for( unsigned i=0; i<obstacle_points.size(); ++i )
            {
                const proximity_query_node_t* prox_node = dynamic_cast< const proximity_query_node_t* >( obstacle_points[i] );

                //If we haven't seen this before
                if( std::find( sensed_obstacle_indices.begin(), sensed_obstacle_indices.end(), prox_node->obstacle_index ) == sensed_obstacle_indices.end() )
                {
                    //Push it in
                    sensed_obstacle_indices.push_back( prox_node->obstacle_index );
                    //Also record the distance
                    sensed_obstacle_distances.push_back( navigation_space->distance( agent_state, prox_node->point ) );
                }
            }

            //Then, we see what obstacles we ended up with
            geometry_t* geom;
            for( unsigned i=0; i<sensed_obstacle_indices.size(); ++i )
            {
                geom = &obstacle_geoms[sensed_obstacle_indices[i]];
                //And put all of them in there
                if (prox_list.size() < max_num_neighbors)
                {
                    prox_list.push_back(prox_elem_t(obstacle_configs[sensed_obstacle_indices[i]].first, obstacle_configs[sensed_obstacle_indices[i]].second, distance, geom ));
                }
                else
                {
                    std::vector<prox_elem_t>::iterator it = std::max_element(prox_list.begin(), prox_list.end());
                    prox_elem_t temp(obstacle_configs[sensed_obstacle_indices[i]].first, obstacle_configs[sensed_obstacle_indices[i]].second, distance, geom );
                    if (temp < *it)
                    {
                        *it = temp;
                    }
                }
            }
        }

        void twoD_proximity_sensing_info_t::compute_static_obstacle_vertices()
        {
            std::vector< proximity_query_node_t > object_verts;

            for( unsigned i=0; i<obstacle_configs.size(); ++i )
            {
                //Get the geometry we are computing vertices
                geometry_t& geo = obstacle_geoms[ i ];

                //Then, we only really care about boxes for now
                if( geo.get_type() == PRX_BOX )
                {
                    //We need to process the vertices of the mesh
                    const std::vector<vector_t>& vertices = geo.get_trimesh()->get_vertices();

                    //Then, clear out whatever notion we might have about what its vertices are
                    object_verts.clear();

                    //Then, we will assume things are axis aligned (z-axis) to find "middle points" to use for queries
                    for( unsigned s=0; s<vertices.size(); ++s )
                    {
                        const vector_t& first = vertices[s];
                        for( unsigned t=s+1; t<vertices.size(); ++t )
                        {
                            const vector_t& second = vertices[t];

                            //If the x and y values for the points are nearly the same, they must be above each other
                            if( fabs(first[0] - second[0]) < PRX_DISTANCE_CHECK && fabs(first[1] - second[1]) < PRX_DISTANCE_CHECK )
                            {
                                //Let's get another priority query node
                                pq_nodes.resize( pq_nodes.size()+1 );

                                //Fill up the node's information
                                pq_nodes.back().obstacle_index = i;
                                pq_nodes.back().point = navigation_space->alloc_point();

                                //Now, we need to transform the point so it represent the obstacle's pose.
                                transform_config.set_position( first );
                                transform_config.set_orientation( 0, 0, 0, 1 );
                                transform_config.relative_to_global( obstacle_configs[i].second );

                                //Then, put in the transformed points
                                pq_nodes.back().point->memory[0] = transform_config.get_position()[0];
                                pq_nodes.back().point->memory[1] = transform_config.get_position()[1];
                                pq_nodes.back().point->memory[2] = (first[2] + second[2])/2.0 + obstacle_configs[i].second.get_position()[2];

                                object_verts.resize( object_verts.size() + 1 );
                                object_verts.back() = pq_nodes.back();
                            }
                        }
                    }

                    //Need the centroid
                    std::vector< double > centroid(2);
                    centroid[0] = centroid[1] = 0;
                    for( unsigned t=0; t<object_verts.size(); ++t )
                    {
                        centroid[0] += object_verts[t].point->memory[0];
                        centroid[1] += object_verts[t].point->memory[1];
                    }
                    centroid[0] /= ((double)object_verts.size());
                    centroid[1] /= ((double)object_verts.size());

                    //Then, compute their angle relative to the centroid.
                    for( unsigned t=0; t<object_verts.size(); ++t )
                    {
                        object_verts[t].sort_value = atan2( object_verts[t].point->memory[1] - centroid[1], object_verts[t].point->memory[0] - centroid[0] );
                    }

                    //Then sort
                    std::sort( object_verts.begin(), object_verts.end() );

                    //Okay, now we have the vertices in order, need to put in intermediate points now
                    for( unsigned k=0; k<object_verts.size(); ++k )
                    {
                        unsigned l = (k+1)%(object_verts.size());

                        //Figure out how many interpolation points we want
                        double distance = navigation_space->distance( object_verts[k].point, object_verts[l].point );
                        double steps = std::ceil( obstacle_sensing_radius / distance );

                        //Then, for each interpolation point
                        for(double t=1; t<=steps; ++t)
                        {
                            //Get another node
                            pq_nodes.resize( pq_nodes.size()+1 );

                            //Fill up the node's information
                            pq_nodes.back().obstacle_index = i;
                            pq_nodes.back().point = navigation_space->alloc_point();

                            navigation_space->interpolate( object_verts[k].point, object_verts[l].point, t/(steps+1.0), pq_nodes.back().point );
                        }
                    }
                }
            }

            //Alright, now that that is over, put the points into the metric (sadly, one at a time)
            for( unsigned i=0; i<pq_nodes.size(); ++i )
            {
                const prx::util::proximity_query_node_t* node = &(pq_nodes[i]);
                metric->add_point( node );
            }

            PRX_PRINT("Added points to the metric: " << metric->get_nr_points(), PRX_TEXT_RED );

        }

        bool twoD_proximity_sensing_info_t::find_system_configuration_index( const util::config_list_t& plant_configs )
        {
            bool found = false;
            for (unsigned i=0; i < plant_configs.size() && !found; i++)
            {
                if(plant_configs[i].first == system_path)
                {
                    found = true;
                    system_index = i;
                    system_config = plant_configs[i].second;
                    system_geometry = new geometry_t( plant_geoms[i] );

                    // PRX_DEBUG_COLOR("Found system geometry of type: " << system_geometry->get_type(), PRX_TEXT_BLUE);

                    // foreach(std::string name, geoms | boost::adaptors::map_keys)
                    // {
                    //     if (name == system_path)
                    //     {
                    //         system_geometry = new geometry_t(geoms[name]);
                    //         break;
                    //     }
                    // }
                    // found = true;
                }
            }
            return found;
        }

        proximity_list_t twoD_proximity_sensing_info_t::get_proximity_list()
        {
            return prox_list;
        }

        double twoD_proximity_sensing_info_t::get_closest_plant_distance()
        {
            return closest_plant_distance;
        }

        double twoD_proximity_sensing_info_t::get_closest_obstacle_distance()
        {
            return closest_obstacle_distance;
        }

        void twoD_proximity_sensing_info_t::get_info()
        {
            PRX_WARN_S ("Get info not implemented in base class!");
        }

        void twoD_proximity_sensing_info_t::set_system_path(const std::string& path)
        {
            system_path = path;
        }

        unsigned twoD_proximity_sensing_info_t::get_max_num_neighbors()
        {
            return max_num_neighbors;
        }

        double twoD_proximity_sensing_info_t::get_sensing_radius()
        {
            return sensing_radius;
        }

        //duplicated so that it can sense obstacles at a different radius
        double twoD_proximity_sensing_info_t::get_obstacle_sensing_radius()
        {
            return obstacle_sensing_radius;
        }

    }
}

