// /**
//  * @file pap_trajectory_generator_tp.hpp
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

// #ifndef PRX_POSE_AND_GRASP_TRAJECTORY_GENERATOR_TP_HPP
// #define	PRX_POSE_AND_GRASP_TRAJECTORY_GENERATOR_TP_HPP

// #include "prx/utilities/definitions/defs.hpp"
// #include "prx/utilities/parameters/parameter_reader.hpp"
// #include "prx/utilities/definitions/sys_clock.hpp"
// #include "prx/utilities/boost/hash.hpp"
// #include "prx/utilities/math/configurations/config.hpp"
// #include "prx/utilities/spaces/space.hpp"

// #include "prx/simulation/plan.hpp"
// #include "prx/simulation/trajectory.hpp"

// #include "prx/planning/task_planners/task_planner.hpp"
// #include "prx/planning/motion_planners/motion_planner.hpp"
// #include "prx/planning/queries/query.hpp"
// #include "prx/planning/problem_specifications/specification.hpp"
// #include "prx/planning/queries/motion_planning_query.hpp"

// #include "planning/manipulation_world_model.hpp"
// #include "planning/queries/manipulation_query.hpp"


// namespace prx
// {
//     namespace packages
//     {
//         namespace manipulation
//         {
            
//             /**
//              * A task planner for generating pick-and-place trajectories.
//              * The input of this tp comes from the pag_evaluation_tp.
//              *
//              * @authors Andrew Kimmel
//              */
//             class pap_trajectory_generator_tp_t : public plan::task_planner_t
//             {

//               public:

//                 pap_trajectory_generator_tp_t();
//                 virtual ~pap_trajectory_generator_tp_t();

//                 virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

//                 /** @copydoc motion_planner_t::link_world_model(world_model_t* const) */
//                 void link_world_model(plan::world_model_t * const model);

//                 /** @copydoc planner_t::link_query(query_t*) */
//                 virtual void link_query(plan::query_t* new_query);

//                 /** @copydoc motion_planner_t::setup() */
//                 virtual void setup();

//                 /** @copydoc motion_planner_t::execute() */
//                 virtual bool execute();

//                 /** @copydoc motion_planner_t::succeeded() const */
//                 virtual bool succeeded() const;

//                 /** @copydoc motion_planner_t::get_statistics() */
//                 virtual const util::statistics_t* get_statistics();

//                 virtual void resolve_query();

//               protected:
                  
//                   virtual void save_plan_to_file(std::string arm, unsigned object_index, sim::plan_t* plan);
//                   virtual void save_valid_poses_to_file(std::string arm, const std::vector< util::config_t >& valid_grasps, const std::vector< std::vector<double> >& valid_poses);

//                 manipulation_world_model_t* manipulation_model;
//                 manipulator_t* manipulator;
//                 sim::state_t* manip_initial_state;

//                 /** @brief The input manipulation query */
//                 manipulation_query_t* manipulation_query;

//                 plan::motion_planning_query_t* in_query;
                
//                 bool deserialize_plans;
//                 std::vector< std::vector<double> > potential_left_valid_poses, potential_right_valid_poses;
//                 std::vector< std::vector<double> > left_valid_poses, right_valid_poses;
//                 std::vector< util::config_t > left_grasps, right_grasps;
//                 std::string left_pose_file, right_pose_file;

//                 std::string full_manipulator_context_name;
//                 std::string left_context_name, right_context_name;
//                 const util::space_t* left_state_space, *right_state_space;
//                 const util::space_t* left_control_space, *right_control_space;
//                 const util::space_t* full_state_space, *full_control_space;
//                 std::string manipulation_task_planner_name;
//                 std::vector<double> left_target_vec, right_target_vec;

//             };
//         }
//     }
// }


// #endif
