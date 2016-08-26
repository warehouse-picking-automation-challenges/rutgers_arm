// /**
//  * @file rearrangement_search_algorithm_t.hpp
//  *
//  * @copyright Software License Agreement (BSD License)
//  * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick 
//  * All Rights Reserved.
//  * For a full description see the file named LICENSE.
//  *
//  * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
//  *
//  * Email: pracsys@googlegroups.com
//  */

// #pragma once

// #ifndef PRX_REARRANGEMENT_SEARCH_ALGORITHM_HPP
// #define PRX_REARRANGEMENT_SEARCH_ALGORITHM_HPP

// #include "prx/utilities/definitions/defs.hpp"
// #include "prx/utilities/definitions/sys_clock.hpp"
// #include "prx/planning/task_planners/task_planner.hpp"

// #include "simulation/plants/manipulator.hpp"
// #include "simulation/plants/movable_body_plant.hpp"
// #include "planning/graphs/rearrangement_graph.hpp"
// #include "planning/statistics/rearrangement_search_statistics.hpp"
// #include "planning/modules/pose.hpp"

// #include <list>
// #include <sstream>

// namespace prx
// {
//     namespace util
//     {
//         class bounds_t;
//         class multiple_goal_states_t;
//         class statistics_t;
//     }

//     namespace plan
//     {
//         class motion_planning_query_t;
//     }

//     namespace packages
//     {
//         namespace rearrangement_manipulation
//         {
//             using namespace baxter;
//             using namespace manipulation;

//             class rearrangement_search_query_t;
//             class rearrangement_search_specification_t;
//             class rearrangement_primitive_t;
//             class rearrangement_primitive_specification_t;
//             class rearrangement_query_t;
//             class rearrangement_path_planner_t;

//             /**             
//              * 
//              * @autors Athanasios Krontiris
//              */
//             class rearrangement_search_algorithm_t : public plan::task_planner_t
//             {

//               public:
//                 rearrangement_search_algorithm_t();
//                 virtual ~rearrangement_search_algorithm_t();

//                 /** @copydoc task_planner_t::init(const util::parameter_reader_t*, const util::parameter_reader_t*) */
//                 virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

//                 /** @copydoc task_planner_t::execute() */
//                 virtual bool execute();
//                 /** @copydoc task_planner_t::succeeded() const */
//                 virtual bool succeeded() const;

//                 /** @copydoc task_planner_t::setup() */
//                 virtual void setup();

//                 /** @copydoc task_planner_t::resolve_query() */
//                 virtual void resolve_query();

//                 const util::statistics_t* get_statistics();

//                 /**
//                  * @copydoc planner_t::link_specification(specification_t*)
//                  */
//                 virtual void link_specification(plan::specification_t* new_spec);

//                 /**
//                  * @copydoc motion_planner_t::link_query()
//                  */
//                 virtual void link_query(plan::query_t* new_query);

//               protected:

//                 int detect_pose(sim::state_t * pose);
//                 util::undirected_vertex_index_t add_node(std::vector<unsigned>& arrangement);
//                 util::undirected_vertex_index_t add_node(std::vector<unsigned>& arrangement, sim::state_t* graph_point);
//                 bool try_connect(std::vector<unsigned>& arrangement);

//                 /**
//                  * @param arrangement The successful arrangement that we are able to use. This vector
//                  * needs to be initialized with k_object size. 
//                  * 
//                  * @return True if we found a valid arrangement, otherwise False. 
//                  */
//                 bool sample_arrangement(std::vector<unsigned>& arrangement);
//                 /**
//                  * Will generate a graph_point with XY points for all the objects. The state that is given as argument 
//                  * has already memory. The function will just fill the point with the correct numbers.
//                  * 
//                  * @param point The state point for the graph point. Has already memory and the point belongs to the rearrangement search algorithm.
//                  * @param arrangement The arrangement of poses that we want the point to be created from. 
//                  */
//                 void generate_graph_point(sim::state_t* point, const std::vector<unsigned>& arrangement);

//                 *
//                  * Checks if a pose is valid in an arrangement.
//                  * @param index The index of the new pose that we want to check for validity. 
//                  * @param arrangement The current arrangement with the current number of poses. 
//                  * @return True if the pose is not overlapping with other poses in the arrangement, otherwise False.
                 
//                 bool valid_pose(unsigned index, const std::vector<unsigned>& arrangement);

//                 /**
//                  * Checks if the point with the current arrangement already exists in the graph.
//                  * @param point The new point that we want to add.
//                  * 
//                  * @return True if the point already in the graph. False, otherwise.
//                  */
//                 util::undirected_vertex_index_t node_exists(const std::vector<unsigned>& arrangement);


//                 /**
//                  * @brief Updates the k value for the k-prm*.
//                  * 
//                  * @param nr_nodes The number of the nodes in the graph.
//                  */
//                 virtual void update_k(unsigned nr_nodes);

//                 void create_spaces();

//                 //===============================================//
//                 // Graph space with XY points of all the objects //
//                 //===============================================//
//                 util::space_t* graph_space;
//                 sim::state_t* graph_point;
//                 std::vector<double*> graph_space_memory;
//                 std::vector<double> graph_vec;

//                 //==========================================================//
//                 // Setup spaces and space_points for manipulator and object //
//                 //==========================================================//
//                 std::string pc_name_manipulator_only;
//                 /** @brief The state space over the manipulator that the manipulation task planner is using. */
//                 const util::space_t* manip_state_space;
//                 /** @brief The control space over the manipulator that the manipulation task planner is using. */
//                 const util::space_t* manip_control_space;
//                 /** @brief The state space of an object*/
//                 const util::space_t* object_state_space;
//                 /** @brief Safe state for the manipulator*/
//                 sim::state_t* safe_state;
//                 /** @brief Safe control for the manipulator */
//                 sim::control_t* safe_control;
//                 /** @brief State for the object*/
//                 sim::state_t* object_state;

//                 //================================//
//                 // Communication from application //
//                 //================================//
//                 rearrangement_search_query_t* in_query;
//                 rearrangement_search_specification_t* in_specs;

//                 //==============================//
//                 // Communication with primitive //
//                 //==============================//
//                 rearrangement_primitive_t* primitive;
//                 rearrangement_primitive_specification_t* primitive_specs;
//                 rearrangement_query_t* primitive_query;

//                 //=================================//
//                 // Communication with Path planner //
//                 //=================================//
//                 rearrangement_path_planner_t* path_planner;

//                 //=======//
//                 // Graph //
//                 //=======//
//                 util::undirected_graph_t super_graph;
//                 util::undirected_vertex_index_t start_v;
//                 util::undirected_vertex_index_t target_v;

//                 std::vector<pose_t> poses_set;
//                 unsigned poses_set_length;

//                 std::vector<unsigned> ids;
//                 std::vector<unsigned> initial_arrangement;
//                 std::vector<unsigned> target_arrangement;
//                 std::vector<bool> pose_checked;

//                 /** @brief The number of nearest neighbors to attempt connections with. */
//                 unsigned int k_near;

//                 //================//
//                 // For Statistics //
//                 //================//
//                 util::sys_clock_t statistics_clock;
//                 rearrangement_search_statistics_t* stats;
//                 std::string statistics_file;
//                 bool print_all;

//                 //======================//
//                 // For De/Serialization //
//                 //======================//
//                 std::string prx_output_dir;
//                 std::string prx_input_dir;


//                 std::string print(const std::vector<unsigned>& arrangement);
//             };
//         }
//     }
// }
// #endif	
