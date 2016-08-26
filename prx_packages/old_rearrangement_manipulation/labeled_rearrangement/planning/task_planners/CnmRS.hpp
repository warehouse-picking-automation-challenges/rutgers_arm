///**
// * @file cnmrs_t.hpp
// *
// * @copyright Software License Agreement (BSD License)
// * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick 
// * All Rights Reserved.
// * For a full description see the file named LICENSE.
// *
// * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
// *
// * Email: pracsys@googlegroups.com
// */
//
//#pragma once
//
//#ifndef PRX_CNMRS_HPP
//#define PRX_CNMRS_HPP
//
//
//#include "planning/task_planners/nmRS.hpp"
//#include "planning/graphs/crs_graph.hpp"
//
//#include "prx/utilities/definitions/defs.hpp"
//
//#include <list>
//#include <sstream>
//
//namespace prx
//{
//    namespace util
//    {
//        class bounds_t;
//        class multiple_goal_states_t;
//        class statistics_t;
//    }
//
//    namespace plan
//    {
//        class motion_planning_query_t;
//    }
//
//    namespace packages
//    {
//
//        namespace labeled_rearrangement_manipulation
//        {
//            using namespace baxter;
//            using namespace manipulation;
//
//            /**
//             * The task planner for the forward rearrangement RSC algorithm. Computes the path for rearranging labeled 
//             * objects.
//             * 
//             * @autors Athanasios Krontiris
//             */
//            class cnmrs_t : public nmrs_t
//            {
//
//              public:
//
//                cnmrs_t();
//                virtual ~cnmrs_t();
//
//                /** @copydoc motion_planner_t::setup() */
//                virtual void setup();
//
//                /** @copydoc motion_planner_t::resolve_query() */
//                virtual void resolve_query();
//                
//                const util::statistics_t* get_statistics();
//
//              protected:
//                bool connect_nodes(const crs_node_t* v, const crs_node_t* u);
//
//                /**
//                 * 
//                 * @param arrangement The successful arrangement that we are able to use. This vector
//                 * needs to be initialized with k_object size. 
//                 * 
//                 * @return True if we found a valid arrangement, otherwise False. 
//                 */
//                bool sample_arrangement(std::vector<unsigned>& arrangement);
//                bool valid_pose(const sim::state_t* new_pose);
//                bool node_exists(const std::vector<unsigned>& arrangement);
//
//                /**
//                 * @brief Updates the k value for the k-prm*.
//                 * 
//                 * @param nr_nodes The number of the nodes in the graph.
//                 */
//                virtual void update_k(unsigned nr_nodes);
//
//                crs_node_t* start_node;
//                crs_node_t* target_node;
//
//                std::deque<unsigned> objects;
//                std::deque<unsigned> init_objects;
//                std::vector<unsigned> rand_arrangement;
//                std::vector<sim::state_t*> initial_states;
//                std::vector<sim::state_t*> graph_poses;
//                unsigned arrangement_size;
//                /** @brief The number of nearest neighbors to attempt connections with. */
//                unsigned int k_near;
//
//                util::space_t* graph_space;
//                sim::state_t* graph_point;
//                std::vector<double*> graph_space_memory;
//                std::vector<double> graph_vec;
//
//                int num_cc;
//                int num_connections_tries;
//                int num_successful_connections;
//
//                double average_node_connection_time;
//                double max_connection_time;
//                double min_connection_time;
//
//                double average_node_connection_time_success;
//                double max_connection_time_success;
//                double min_connection_time_success;
//
//                double average_node_connection_time_failed;
//                double max_connection_time_failed;
//                double min_connection_time_failed;
//
//                double everything_time;
//
//                std::stringstream connection_times;
//
//            };
//        }
//    }
//}
//
//
//#endif	
