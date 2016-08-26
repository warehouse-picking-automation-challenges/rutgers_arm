/**
 * @file navigation_graph_sensor.hpp 
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
#pragma once

#ifndef PRX_NAVIGATION_GRAPH_SENSOR_HPP
#define PRX_NAVIGATION_GRAPH_SENSOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp" 
#include "prx/utilities/definitions/sys_clock.hpp"

#include "prx/simulation/sensing/sensor.hpp"
#include "prx_simulation/pabt_node_msg.h"
#include "prx_simulation/pabt_agent_msg.h"

#include "simulation/structures/agent_data.hpp"



namespace prx
{
    namespace util
    {
        class parameter_reader_t;
    }
    namespace packages
    {
        namespace crowd
        {
            class nav_node_t;
            class neighbor_t;
            
            /**
             * @author Andrew Dobson
             */
            class navigation_graph_sensor_t : public sim::sensor_t
            {
            public:

                navigation_graph_sensor_t();
                virtual ~navigation_graph_sensor_t();
                
                /**
                 * Initializes from the given parameters.
                 * 
                 * @brief Initializes from the given parameters.
                 * 
                 * @param reader A \ref util::parameter_reader_t with a dictionary of parameters for this system.
                 * @param template_reader A template \ref util::parameter_reader_t with a dictionary of parameters.
                 * Default value for the template reader is NULL. The priority goes to primary reader \c reader, if something
                 * is not specified in the \c reader then it will be read from the \c template_reader 
                 */
                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

                void initialize_sensor(sim::simulator_t* sim);
                void link_navigation_primitives( util::undirected_graph_t* input_graph, util::distance_metric_t* input_metric, std::vector< agent_data_t >& input_agents );
                
                virtual void update_data(); 

                virtual bool fires();

                void get_near_nodes( unsigned index, std::pair< const nav_node_t*, const nav_node_t* >& node_data );

                void update_node_information( unsigned index, const nav_node_t* triangle, const nav_node_t* near );
                const nav_node_t* find_triangle( const nav_node_t* start, const std::vector< double >& current_state );
                // const nav_node_t* find_near_triangle( const nav_node_t* start, const std::vector< double >& current_state );

                void get_neighbors( unsigned index, std::vector< neighbor_t* >& neighborhood );

                void update_neighbor_information( unsigned index );
                void update_neighborhood( unsigned index );
                
                unsigned get_max_num_neighbors();

                void set_num_of_triangles(int num_of_triangles);
                void generate_node_msg(int node_id, prx_simulation::pabt_node_msg& node_msg);
                void update_node_info(int node_id, const prx_simulation::pabt_node_msg& node_msg);

                static double update_time;
                static unsigned update_calls;

            protected:
                // @brief A pointer to the navigation graph
                util::undirected_graph_t* graph_pointer;
                util::distance_metric_t* metric;

                std::vector< const nav_node_t* > agent_nodes;
                std::vector< const nav_node_t* > waypoint_nodes;
                std::vector< agent_data_t* > all_agents;
                std::vector< sim::plant_t* > all_plants;
                std::vector< neighbor_t > all_neighbors;
                std::vector< std::vector< neighbor_t* > > all_neighborhoods;

                std::vector<neighbor_t*> extra_neighbors;
                int active_extra_neighbors;
                std::vector< std::vector < neighbor_t *> > extra_neighbors_per_triangle;

                std::vector< double > prior_position;

                std::vector< double > point_vector_memory;
                util::space_point_t* query_point;
                util::space_point_t* neighbor_point;
                
                double agent_radius;
                double sensing_radius;
                unsigned max_neighbors;

                util::sys_clock_t stats_clock;

            };
        }

    }
}

#endif

