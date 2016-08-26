// /**
//  * @file rrt_grasping_planner.hpp
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

// #ifndef PRX_RRT_GRASPING_PLANNER_HPP
// #define	PRX_RRT_GRASPING_PLANNER_HPP

// #include "planning/task_planners/grasping_planner.hpp"
// #include "planning/goals/half_space_goal.hpp"
// #include "planning/modules/planner_info.hpp"

// namespace prx
// {
//     namespace packages
//     {
//         namespace manipulation
//         {
//             /**
//              * Manipulation task planner. Computes the path for moving an object from an
//              * initial to a target position.
//              *
//              * @authors Rahul Shome
//              */
//             class rrt_grasping_planner_t : public grasping_planner_t
//             {

//               public:

//                 rrt_grasping_planner_t();
//                 virtual ~rrt_grasping_planner_t(); 

//                 virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);
//                 virtual void link_query(plan::query_t* new_query);
//                 virtual void update_plane_specs(std::vector<double> new_shelf_plane, bool new_half_space_side);
//                 virtual void link_world_model(plan::world_model_t * const model);
//                 // virtual void link_parent_planner_info_map(util::hash_t<std::string, planner_info_t*> in_planner_info_map);

//               protected:

//                 // virtual bool evaluate_the_relative_config(const grasp_t* grasp);
//                 virtual bool compute_rrt_solution(sim::plan_t& plan, sim::state_t* satisfied_goal, planner_info_t* current_planner, sim::state_t* start, const util::space_t* start_space, std::vector<sim::state_t* >& roadmap_neighbors);
//                 virtual void reverse_plan_from_path(sim::plan_t& reverse_plan, sim::trajectory_t& path);
//                 virtual std::vector<sim::state_t*> get_nearest_points_on_roadmap(sim::state_t* grasping_state, const util::space_t* grasping_space, util::config_t rel_grasp);
//                 virtual double get_config_distance(util::config_t& c1, util::config_t& c2);
//                 virtual void update_object_with_relative_grasp(util::config_t ee_config, util::config_t relative_grasp);

//                 std::vector<double> shelf_plane;
//                 bool half_space_side;
//                 half_space_goal_t* hs_goal; 


//                 /** @brief A map from planning context to manipulation contexts (planner, query and spaces)*/
//                 util::hash_t<std::string, planner_info_t*> planner_info_map;
//                 /** @brief The current active context (planner, query and spaces)*/
//                 planner_info_t* active_planner;
//                 manipulation_context_info_t* current_manipulation_context_info;

//                 util::hash_t<std::string, planner_info_t*> parent_planner_info_map;
//                 planner_info_t* parent_planner;

//             };
//         }
//     }
// }


// #endif
