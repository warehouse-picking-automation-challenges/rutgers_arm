/**
 * @file planner_info.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#pragma once

#ifndef PRX_PLANNER_INFO_HPP
#define	PRX_PLANNER_INFO_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/heuristic_search/constraints.hpp"

#include "prx/planning/planner.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/queries/query.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"

#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/modules/heuristic_search/constraints_astar_search.hpp"

#include "planning/manipulation_world_model.hpp"
#include "planning/queries/manipulation_query.hpp"
#include "planning/modules/manipulation_validity_checker.hpp"

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
            /**
             *
             *  Helper class that sets up a motion planner to compute manipulation type plans
             * @authors Andrew Dobson, Andrew Kimmel, Rahul Shome
             */
            class planner_info_t
            {
              public:

                planner_info_t(std::string input_construction_context_name, std::string input_planning_context_name, plan::planner_t* input_planner, plan::motion_planning_specification_t* input_specification, plan::query_t* input_query);

                virtual ~planner_info_t();

                void setup(manipulation_world_model_t* model);

                void setup_search(plan::search_mode_t input_heuristic_search_mode, const util::constraints_t* input_valid_constraints, std::vector< std::string >& input_constraint_names );

                // Helper function that calls the complete compute solution for when you only have a single goal state
                bool compute_solution(sim::plan_t& plan, util::constraints_t* path_constraints, sim::state_t* start_state,  sim::state_t* input_goal_state, bool ik_steer, bool set_constraints = false);
                
                // The actual compute solution
                bool compute_solution(sim::plan_t& plan, util::constraints_t* path_constraints, sim::state_t* start_state, const std::vector< sim::state_t* >& input_goal_states, bool ik_steer, bool set_constraints = false);

                // Helper function that calls the complete compute solution for when you only have a single goal state
                bool compute_solution(sim::plan_t& plan, util::constraints_t* path_constraints, std::vector<sim::state_t*> start_states,  sim::state_t* input_goal_state, bool ik_steer, bool set_constraints = false);
                
                // The actual compute solution
                bool compute_solution(sim::plan_t& plan, util::constraints_t* path_constraints, std::vector<sim::state_t*> start_states, const std::vector< sim::state_t* >& input_goal_states, bool ik_steer, bool set_constraints = false);

                // The version of compute solution for link placement
                bool compute_solution(sim::plan_t& plan, util::constraints_t* path_constraints, sim::state_t* start_state, sim::state_t* reached_state, const util::config_t& target_config, std::string end_effector, std::string input_context, bool set_constraints = false);

                std::string construction_context_name;
                plan::motion_planner_t* planner;
                std::string planning_context_name;
                plan::motion_planning_specification_t* specification;
                plan::motion_planning_query_t* query;

            protected:

                manipulation_world_model_t* manip_model;

                //The spaces for the motion planner. 
                const util::space_t* state_space;
                const util::space_t* control_space;
                sim::state_t* start_point;
                std::string end_effector;

                plan::search_mode_t heuristic_search_mode;
                const util::constraints_t* valid_constraints;
                std::vector< std::string > constraint_names;                
                plan::validity_checker_t* checker;

                util::constraints_t* tmp_constraints;

                manipulation_validity_checker_t* manip_validity_checker;
            };
        }
    }
}


#endif
