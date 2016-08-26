// /**
//  * @file grasp_evaluation_tp.hpp
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

// #ifndef PRX_GRASP_EVALUATION_TP_HPP
// #define	PRX_GRASP_EVALUATION_TP_HPP

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
// #include "prx/planning/modules/validity_checkers/validity_checker.hpp"
// #include "prx/planning/modules/local_planners/local_planner.hpp"

// #include "planning/manipulation_world_model.hpp"
// #include "planning/queries/manipulation_query.hpp"
// #include "planning/queries/grasping_query.hpp"
// #include "planning/task_planners/grasping_planner.hpp"


// namespace prx
// {
//     namespace packages
//     {
//         namespace manipulation
//         {

//             /**
//              * Evaluate a set of grasps and determine success and timing data.
//              */
//             class grasp_evaluation_tp_t : public plan::task_planner_t
//             {

//               public:

//                 grasp_evaluation_tp_t();
//                 virtual ~grasp_evaluation_tp_t();

//                 virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

//                 /**
//                  * @copydoc motion_planner_t::reset()
//                  */
//                 virtual void reset();

//                 /**
//                  * @copydoc motion_planner_t::link_world_model()
//                  */
//                 virtual void link_world_model(plan::world_model_t * const model);

//                 /**
//                  * @copydoc motion_planner_t::get_statistics()
//                  */
//                 virtual const util::statistics_t* get_statistics();

//                 /**
//                  * @copydoc planner_t::link_specification(specification_t*)
//                  */
//                 virtual void link_specification(plan::specification_t* new_spec);

//                 /**
//                  * @copydoc motion_planner_t::link_query()
//                  */
//                 virtual void link_query(plan::query_t* new_query);

//                 /**
//                  * @copydoc motion_planner_t::setup()
//                  *
//                  * Will occupy memory for the random_open_point and the new_control, after
//                  * planning_query has been linked.
//                  */
//                 virtual void setup();

//                 /**
//                  * @copydoc motion_planner_t::execute()
//                  */
//                 virtual bool execute();


//                 /**
//                  * @copydoc motion_planner_t::succeeded() const
//                  */
//                 virtual bool succeeded() const;

//                 /**
//                  * @copydoc motion_planner_t::resolve_query()
//                  *
//                  * At the end of the resolve_query the algorithm will remove the vertices
//                  * for the start and the goal for this specific query from the graph.
//                  */
//                 virtual void resolve_query();

//               protected:

//                 std::string left_context_name;
//                 std::string right_context_name;

//                 manipulation_world_model_t* manipulation_model;
//                 manipulation_context_info_t* current_manipulation_context_info;

//                 /** @brief The grasping planner.*/
//                 grasping_planner_t* grasping_planner;
//                 /** @brief The grasping query that the manipulation task planner will use to communicate with the grasping planner.*/
//                 grasping_query_t* grasping_query;

//                 std::string resolve_grasp_query(movable_body_plant_t* object, int grasping_index, sim::state_t* object_state, util::config_t& retraction_config);
//             };
//         }
//     }
// }


// #endif