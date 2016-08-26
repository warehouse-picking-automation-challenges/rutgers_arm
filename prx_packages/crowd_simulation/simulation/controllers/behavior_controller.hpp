/**
 * @file behavior_controller.hpp
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

#ifndef BEHAVIOR_CONTROLLER_HPP
#define BEHAVIOR_CONTROLLER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"
#include "prx/simulation/systems/controllers/simple_controller.hpp"
#include "simulation/controllers/path_follow_controller.hpp"

 #include "simulation/graph/nav_node.hpp"

#include <boost/any.hpp>

namespace prx
{
    namespace util
    {
        class space_t;
        class space_point_t;
        class astar_search_t;
        class distance_metric_t;
    }

    namespace packages
    {
        namespace crowd
        {
            // class path_follow_controller_t;
            class nav_node_t;
            class origin_t;
            class OD_info_t;
            class queue_t;

            /**
             * @brief <b>Controller which performs high-level behavior processing</b>
             *
             * @author Athanasios Krontiris
             */
            class behavior_controller_t : public sim::simple_controller_t
            {
              public:

                behavior_controller_t();
                virtual ~behavior_controller_t();

                void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

//                void setup(const std::vector<double>& desires, double desire_to_go, double avg_speed, int duration_frames, origin_t* origin, double MSEC_2_FRAMES);

                void setup(const OD_info_t& info, double MSEC_2_FRAMES);

                /**
                 * Calls \c propagate to its single subsystem.
                 *
                 * @brief Calls \c propagate to its single subsystem.
                 */
                void propagate(const double simulation_step = 0);

                /**
                 * Calls \c compute_control to its single subsystem.
                 *
                 * @brief \c Calls compute_control to its single subsystem.
                 */
                void compute_control();

                /**
                 * Links in the primitives needed to perform the A* search on the high-level grid
                 */
                void link_search_primitives( util::undirected_graph_t* input_graph, util::astar_search_t* input_astar, util::distance_metric_t* input_metric );

                bool is_agent_leaving() const;

                void set_time_for_departure();

                int get_frames_to_leave() const;

                unsigned get_id();

                void set_goal( const util::space_point_t* new_goal );

                void set_goal( const std::vector<double>& new_goal );

                void in_queue(queue_t* queue_arg, int queue_position );

                void move_in_queue(int queue_position, const std::vector<double>& new_goal);

                void leave_queue();

                int get_queue_id();

                double get_queue_orientation();

                double get_lookup_x();

                double get_lookup_y();

                // Called for the first agent in the queue
                void set_region_lookup();
                
                // Called for the remanining agents in the queue
                void set_lookup(double lookup_x, double lookup_y);

                void evacuate(const std::vector<region_t*>& evacuation_points);

                static double cc_time;
                static unsigned cc_calls;
                static double tot_time;

              protected:
                double lookup_x;

                double lookup_y;

                bool checked_queue;

                bool time_for_departure;

                double queue_orientation;
                
                path_follow_controller_t* follower;
                
                origin_t* goal_origin;
                region_t* curr_region;
                attractor_t* tmp_attractor;

                util::space_point_t* goal_point;

                //Primitives needed for behavior                 
                std::vector<double> desires; //The different desires for the agent. Given as input.
                double desire_to_go; //The desire of the agent to leave.
                double avg_speed;
                int frames_to_leave; //how many frames the agent has in the environment before he/she leaves. 
                bool is_leaving; //True if the agent is leaving.
                bool is_evacuating; //If the agent has to evacuate the building. 
                double gamma; //A positive value <1 that is used to compute the influence values.
                int duration; //how many frames the agent will stay in an attractor location.
                int try_attractors; //How many attractors we want to try before the agent decides where to go.
                bool hindered; //Whether or not this agent is either disabled or carrying luggage                

                //Primitives needed for search
                util::undirected_graph_t* nav_graph;
                util::astar_search_t* graph_search;
                util::distance_metric_t* metric;                

                region_t* get_next_region_to_go(const nav_node_t* curr_node, double best_value);

                double leaving_influence(double dist);

                double attractor_influence(region_t* region, double dist, double origin_dist, double duration);

                double capacity_influence(region_t* region);

                double msec2frames;
                //The critical point that the agents start thinking for leaving. Frames
                double spare_time_thres;

                ////////////
                // QUEUES //
                ////////////                
                //The queue the agent is waiting.
                queue_t* queue;
                //The position of the agent inside the queue. 
                int index_in_queue;
                //If the agent waits in the queue or not.
                bool waits_in_queue;
                //Helping variable to maintain the position of the queue point that the agent has to go. 
                std::vector<double> queue_points;

                std::vector<double> attractor_point;

                // Gets the current position of the agent
                std::vector<double> current_position;
                
            };
        }
    }
}


#endif

