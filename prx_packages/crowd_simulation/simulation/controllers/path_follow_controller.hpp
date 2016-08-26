/**
 * @file path_follow_controller.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#pragma once

#ifndef PRX_PATH_FOLLOW_CONTROLLER_HPP
#define PRX_PATH_FOLLOW_CONTROLLER_HPP

#include "simulation/graph/nav_node.hpp"

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"

#include "prx/simulation/systems/controllers/simple_controller.hpp"

#define QUEUE_EPSILON  5
#define RE_COMPUTE_DISTANCE  20
#define Z_DISTANCE_CHECK  2e-1

#define MAX_FRAME_COUNTER 3000
#define MIN_STEP_CHECK 0.1
#define LOST_LIMIT  1e3

namespace prx
{
    namespace util
    {
        class astar_search_t;
        class distance_metric_t;
    }

    namespace packages
    {
        namespace crowd
        {
            class agent_data_t;
            class node_sensing_info_t;
            class collision_avoidance_controller_t;

            enum agent_mode_t { PRX_AGENT_MODE_NORMAL,  PRX_AGENT_MODE_GOING_TOWARDS_REGION, PRX_AGENT_MODE_GOING_TOWARDS_QUEUE, PRX_AGENT_MODE_GOING_TOWARDS_RESERVED_SLOT, PRX_AGENT_MODE_GOING_TOWARDS_QUEUE_POINT, PRX_AGENT_MODE_IN_QUEUE, PRX_AGENT_MODE_IN_OTHER};

            /**
             * @brief <b>A controller which intelligently follows an input path and
             * guides a system between them in a predefined pattern.</b>
             *
             * @author Andrew Dobson
             */
            class path_follow_controller_t : public sim::simple_controller_t
            {

              public:
                /**
                 * Initializes internal variables and sets up the controller's state space with the time parameter.
                 *
                 * @brief Initializes internal variables and sets up the controller's state space with the time parameter.
                 */
                path_follow_controller_t();

                /** @copydoc system_t::~system_t() */
                virtual ~path_follow_controller_t();

                /**
                 * Reads the list of path_follows from input and creates the sequence of plans
                 * which will be used to control the child system.
                 *
                 * Also does standard controller_t::init()
                 *
                 * @brief Initializes the path_follow_controller_t
                 * @param reader Used to initialize the controller
                 * @param template_reader Used to initialize the controller from template files
                 */
                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

                /**
                 * Calls \c propagate to its single subsystem.
                 *
                 * @brief Calls \c propagate to its single subsystem.
                 */
                virtual void propagate(const double simulation_step = 0);

                /**
                 * Calls \c compute_control to its single subsystem.
                 *
                 * @brief \c Calls compute_control to its single subsystem.
                 */
                virtual void compute_control();

                /**
                 * Gets the proximity element corresponding to the nearest obstacle to the current waypoint
                 */
                const nav_node_t* get_nearest_node() const;

                /**
                 * Links in the primitives needed to perform the A* search on the high-level grid
                 */
                void link_metric( util::distance_metric_t* input_metric );
                void link_search_primitives(util::undirected_graph_t* input_graph, util::astar_search_t* input_astar, util::distance_metric_t* input_metric );

                /**
                 * @brief Checks if the agents has arrived in the target point or if the agent never had a target.
                 * @details Checks if the agents has arrived in the target point. The behavior controller needs to know if the 
                 * agent has arrived at its target point in order to compute the next step. The \c path_folow_controller has access
                 * to the agent so it can check if the agent is at its target point. 
                 * @return True if the agent needs a new target, otherwise false.
                 */
                bool need_new_target();

                /**
                 * @brief Forces the agent to change his target.
                 * @details If the agent is lost for a long period and just standing somewhere, the behavior controller will force the agent to change his 
                 * target. 
                 */
                void force_new_target();

                /**
                 * @brief Checks if the agent is at the same position for the last \c MAX_FRAME_COUNTER frames.
                 * @details  Checks if the agent is at the same position for the last \c MAX_FRAME_COUNTER frames. If that is true then  the behavior 
                 * controller will reset the target for that agent. 
                 * @return True if the agent is stack to a location. Otherwise, false.
                 */
                bool is_the_agent_lost();

                void reset_lost_counter();

                /**
                 * @brief Helping function for the behavior controller in order to realize when is time to check for queues.
                 * @details Helping function for the behavior controller in order to realize when is time to check for queues.
                 * The behavior controller will check this function and if its time will get information for queuing. 
                 * 
                 * @return True if the agent is close to the region and should check for queues. False, otherwise. 
                 */
                // Tells whether the agent is nearby
                bool is_near_by();
                
                bool need_to_check_for_queue();

                bool need_to_reserve_queue_point();

                bool need_to_go_to_queue();
                
                bool at_goal();

                void set_id( unsigned input_id );

                unsigned get_id();

                // Fills the currrent position of the agent in the vector
                void get_current_position(std::vector<double>& current_position);

                /**
                 * @brief Updates nearest nav_node information for the path-follower.
                 */
                void update_node_information( const nav_node_t* triangle, const nav_node_t* near );
                
                /**
                 *
                 */
                void go_to_region( region_t* goal_region, bool hindered = false );

                void go_to_point( const vector<double>& point);

                // Set the agents orientation inside the queue
                void set_queue_orientation(double queue_orientation);

                /**
                 * @brief Finds a path for the agent to go to the given destination. 
                 * @details Finds a path for the agent to go to the given destination. This function will
                 * setup the agent to follow the computed path. 
                 * 
                 * @param destination The destination that we want the agent to go.
                 * @return [description]
                 */
                bool go_to_destination( const nav_node_t* new_destination );

                void leave_queue();

                void set_bias( double input_bias );
                
                void set_agent_mode_to_normal();

                void set_agent_mode(agent_mode_t mode);

                agent_mode_t get_agent_mode();

                // To check whether the agent mode is normal
                bool is_agent_mode_normal();                

                // To check whether the agent is in queue
                bool is_agent_in_queue();
                // To remove agent from the environment and push back to la-la land
                //bool remove_agent_from_environment();

                // Give the current position(triangle node) of the agent
                const nav_node_t* get_current_triangle_node();                

                static double cc_time;
                static unsigned cc_calls;
                static double pf_time;
                static unsigned pf_calls;

              protected:

                agent_mode_t agent_mode;

                // Gets the region point
                std::vector<double> point_to_go;

                void find_next_point();

                void setup_waypoints( region_t* goal_region );                
                
                void setup_waypoints_towards_queue(region_t* goal_region);

                void clear_queue_data();
                // Does A star from triangle_node to the given node (destination)
                bool generate_path_towards_queue(const nav_node_t* destination);

                void compute_segment_target( const nav_node_t* segment_node, util::space_point_t* test_state, util::space_point_t* segment_point );

                void go_to_closest_triangle();

                /**
                 * Create the subsystem of the path follower.  In our case, we must do some pre-initialization shenanigans.
                 */
                virtual sim::system_ptr_t create_subsystem(const std::string& path, const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                void visualize_marker() const;                
                
                const nav_node_t* destination; // The destination that the agent goes.

                //The three distinct nodes a path-follower needs to consider
                const nav_node_t* triangle_node; //Agent's current position
                const nav_node_t* waypoint_node; //Helping variable 
                util::space_point_t* target_point;
                util::space_point_t* next_point;
                
                util::space_point_t* point_a;
                util::space_point_t* point_b;
                
                bool near_goal;
                unsigned frame_count;

                double distance_to_target;
                double father_to_ggfather_dist;
                double max_ratio;

                node_sensing_info_t* node_info;

                //Primitives needed for search
                util::undirected_graph_t* nav_graph;
                util::distance_metric_t* metric;
                util::astar_search_t* graph_search;
                std::deque< util::undirected_vertex_index_t > path_vertices_to_queue;
                std::deque< util::undirected_vertex_index_t > path_vertices_to_queue_slot;
                
                int path_vertices_to_queue_counter;             
                
                //Child system information.
                collision_avoidance_controller_t* vo_controller;
                const util::space_t* child_state_space;
                util::space_point_t* current_state;
                util::space_point_t* prev_state;

                region_t* goal_region;

                double _Cx;
                double _Cy;
                double _Cz;

                double side_bias;

                bool check_queue;
                bool has_target;
                bool hindered;

                bool called;

                int lost_counter;
            };
        }
    }
}


#endif
