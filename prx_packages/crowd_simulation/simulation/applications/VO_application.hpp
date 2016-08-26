/**
 * @file vo_application.hpp
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
#pragma once

#ifndef PRX_VO_APPLICATION_HPP
#define PRX_VO_APPLICATION_HPP

#include "simulation/structures/world_structure.hpp"
#include "simulation/structures/ramp.hpp"
#include "simulation/structures/agent_data.hpp"
#include "simulation/structures/attractor_influence.hpp"
#include "simulation/structures/OD_info.hpp"
#include "simulation/controllers/VO_structure/VO.hpp"

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"
#include "prx/utilities/heuristic_search/astar_search.hpp"

#include "prx/simulation/applications/empty_application.hpp"
#include "simulation/structures/queue_manager.hpp"

#include "prx_simulation/pabt_node_msg.h"

#include <ros/callback_queue.h>


namespace prx
{
    namespace util 
    {
        class distance_metric_t;
    }

    namespace sim
    {
        class plant_t;
    }

    namespace packages
    {
        namespace crowd
        {
            class path_follow_controller_t;
            class behavior_controller_t;
            class PABT_sensing_model_t;

            class VO_application_t : public sim::empty_application_t
            {
              public:
                VO_application_t();
                virtual ~VO_application_t();

                void info_broadcasting(const ros::TimerEvent& event);              

                virtual void init(const util::parameter_reader_t * const reader);

                virtual bool running();

                virtual void frame(const ros::TimerEvent& event);                

              protected:
                virtual void handle_key();

                void set_systems_to_sensing();

                void gather_systems();
                void load_nav_graph(const util::parameter_reader_t * const reader);
                void clean_up_components( const util::parameter_reader_t * const reader );
                void save_nav_graph_annotations( std::ofstream& out );
                void load_nav_graph_annotations( const std::string& filename );
                void load_semantics(const util::parameter_reader_t * const reader);
                void load_semantics_from_file(const std::string& filename);
                void find_nearby_triangles();
                void compute_static_obstacle_vertices();
                void collect_annotations();
                void propagate_region_distances( region_t* region, int search_id, bool hindered = false );
                double check_transition( const nav_node_t* source, const nav_node_t* destination, bool hindered = false );
                void collect_info_for_vertex(util::undirected_vertex_index_t v, int search_id);
                void load_agent_paths(const util::parameter_reader_t * const reader);
                void start_sim_with_agents();
                void open_file_output();
                void find_structures();
                void visualize_nav_graph();
                void visualize_back_pointers( region_t* region, bool hindered = false );
                void visualize_triangles( );
                void visualize_segments( );
                void visualize_ramp_triangles( );
                void visualize_wall_data( unsigned index );
                void visualize_queue_points( );
                void visualize_elevator_associations(  );

                void register_ramp( const std::vector< double >& points, double height, double base, bool is_escalator, bool goes_up );
                bool check_for_agent_spawn();
                void check_for_departures();
                bool can_spawn();
                bool spawn_agent(const util::space_point_t* spawn_position);
                // void spawn_in_other_node();
                
                // Check if any region is ready for depature (For every 1000 frames)
                void check_regions_for_depatures();
                    

                double find_closest_door_for_region(const nav_node_t* node, region_t* region);
                int detect_door(region_t* region, util::undirected_vertex_index_t v);

                void update_sensing_model(const prx_simulation::pabt_node_msg& msg);

                bool ready_to_go();

                void get_longest_line(std::pair< std::vector< double > , std::vector< double > >& lines, const std::vector< util::vector_t >& points, double z);

                const util::space_point_t* get_adjacent_triangle(util::undirected_vertex_index_t v);                
                ////////////
                /* QUEUES */
                ////////////
                /////////////*  START */////////////

                // Function to initialize the queue manager
                void init_queue_manager_from_file(const std::string filename);

                // Get Region from the region name read from the csv file using the world structure
                region_t* get_region_from_region_name(const std::string &region_name);
            
                // Set current queue State from queue point
                void set_queue_state_from_point(points_t point);
                
                // Get the navigation node closest to the given point
                const nav_node_t* get_nav_node_from_point(points_t point);

                // Assign a queue for the region using the region properties
                queue_t* get_queue_from_region_properites(const std::vector<std::string> &region_properties, int queue_id);

                // Returns the triangle node for the given nav node
                const nav_node_t* find_queue_triangle_slot( const nav_node_t* start, const std::vector< double >& current_state );
            
                // Row length in csv
                const static size_t region_properties_row_length = 11;

                const static size_t region_name_for_queue = 9;

                // A pointer to the queue_manager
                queue_managers_t* queue_manager;

                // Start Node for the Queue
                //const nav_node_t* start_nav_node;
                
                //The global path for all the files
                std::string folder_path;
                int triangle_counter;

                region_t* q_region;
                util::space_point_t* queue_query_point; 
                std::vector< double > current_queue_state;
                queue_t* queue;

                segment_struct_t* segment_structure;
                // Walls Dictionary
                util::hash_t<int , segment_struct_t*> wall_segment_structures_map;
                
                // Helper Nav Nodes
                const nav_node_t* nav_node;
                const nav_node_t*  cur_nav_node;
                const nav_node_t* neighbor;
                /////////////*  END */////////////

                //A collection of relevant systems
                std::vector< sim::plant_t* > plants;
                std::vector< collision_avoidance_controller_t* > vo_controllers;
                std::vector< path_follow_controller_t* > path_follower_controllers;
                std::vector< behavior_controller_t* > behavior_controllers;
                
                //A pool which holds a structure with the plants, behavior controllers, etc.
                std::vector< agent_data_t > all_agents;
                //An actual queue of agents which we can grab available agents from
                std::deque< agent_data_t* > agent_pool;
                //The size of all the agents that this node has available.
                int all_agents_size;

                std::vector<OD_info_t> agent_paths;
                unsigned agent_path_index;
                unsigned nr_paths;
                
                double hindered_escalator_cost;
                double hindered_stair_cost;
                
                std::vector< double > state_vec;

                std::vector< util::geometry_info_t > debug_vis_geoms;
                std::vector< util::config_t > debug_vis_configs;
                std::vector< std::string > debug_colors;

                //Navigation graph stuffs
                util::undirected_graph_t navigation_graph;
                util::space_t* navigation_space;
                double _Nx;
                double _Ny;
                double _Nz;

                util::astar_search_t graph_search;
                util::distance_metric_t* metric;
                util::distance_metric_t* obstacle_metric;

                std::vector< std::ofstream* > file_streams;
                std::ofstream moe_file_stream;
                std::ofstream simulation_file_stream;
                int frame_rate_output;

                world_structure_t world_structure;
                std::vector< ramp_t* > ramps;

                std::vector< neighbor_t > obstacle_neighbors;

                util::sys_clock_t frame_clock;                
                unsigned frame_id;
                unsigned end_frame_id;
                
                //Attractor and Origin type indices
                util::hash_t<std::string, unsigned> agent_index;
                util::hash_t<std::string, unsigned> origin_index;
                util::hash_t<std::string, unsigned> attractor_index;
                
                //Reverse maps for the above Hash
                std::vector< std::string > agent_types;

                unsigned nr_agent_types;
                unsigned nr_origin_types;
                unsigned nr_attractor_types;
                
                unsigned minimum_obstacle_neighbors;

                //The vector with all the changes in influence 
                std::vector< attractor_influence_t > attractors_influence_in_time;
                
                // std::vector< std::vector< std::pair<double,double> > > agent_attractor_desires;
                // std::vector< std::pair<double,double> > agent_speed_distribution;

                // Map of all of the plants which have a VO for sensing
                util::hash_t<std::string, sim::plant_t*> vo_plants;
                // Everybody else for sensing
                util::hash_t<std::string, sim::plant_t*> other_plants;
                // Hash table for minkowski vertex lookup
                util::hash_t<std::string, minkowski_body_t> minkowski_vertices;
                // The original vertices for the obstacles 
                util::hash_t<std::string,  std::pair< std::vector< double > , std::vector< double > > > object_original_vertices;

                /** @brief This is only for Visualization purposes? */
                util::hash_t<std::string, collision_avoidance_controller_t*> plant_to_VO;
                /** @brief The currently selected VO controller: also only for visualization */
                collision_avoidance_controller_t* selected;

                /** @brief Our sensing model */
                PABT_sensing_model_t* vo_sensing;
                navigation_graph_sensor_t* nav_sensor;
                util::config_t transform_config;
                std::vector< sim::prox_elem_t* > prox_elements;
                util::geom_map_t geom_map;
                util::config_list_t obstacle_configs;
                std::vector< util::proximity_query_node_t* > pq_nodes;

                double agent_radius;
                double graph_offset;

                bool display_graph;
                bool display_triangles;
                bool display_segments;
                bool multifloor_sim;
                bool loading_annotations;

                bool initialized;

                double MSEC_2_FRAMES;
                double SEC_2_FRAMES;

                double frame_time;
                double spawn_time;
                double propagate_time;
                double publish_state_time;
                double communication_time;
                double update_node_time;
                double saturated_time;
                unsigned saturated_frames;

                double update_node_id;

                bool update_once; 

                //////////////
                /* PARALLEL */
                //////////////
                ros::NodeHandle node;
                int node_id;
                int num_of_nodes;
                int num_of_responses;
                double time_before_synch;
                std::vector<int> spawned_agents_per_node;
                ros::Publisher agent_states_pub;

                
                std::vector<ros::Subscriber> node_subs;
                prx_simulation::pabt_node_msg node_msg;

                ////////////////
                /* EVACUATION */
                ////////////////    
                std::vector<region_t*> evacuation_points;
                int evacuation_frame;
                bool evacuation_plan;
        

                ///////////
                /* STATS */
                ///////////
                bool export_data;
                std::ofstream stats_stream;
                std::ofstream waiting_time_stream;
                std::ofstream comm_time_stream;
                int msg_index;
                int num_spins;
                int num_of_agents_in_total;
                double total_computation_time;
                double total_simulation_time;
            };
        }
    }
}

#endif

