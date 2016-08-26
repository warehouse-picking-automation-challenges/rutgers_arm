/**
 * @file motion_planning_query.hpp
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

#ifndef PRX_PLANNING_QUERY_HPP
#define	PRX_PLANNING_QUERY_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/plan.hpp"
#include "prx/simulation/state.hpp"
#include "prx/simulation/trajectory.hpp"
#include "prx/planning/queries/motion_query.hpp"
#include "prx/planning/modules/heuristic_search/constraints_astar_search.hpp"
#include "prx/utilities/heuristic_search/constraints.hpp"

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
        class goal_t;
    }

    namespace plan
    {

        class stopping_criteria_t;

        /**
         * @anchor motion_planning_query_t
         *
         * This class represents a problem instance to be solved by a motion planner.
         * It is a self-contained class which holds the start/goal pair as well as has
         * storage for answering trajectories and plans.
         *
         * @brief <b> General query for motion planners. </b>
         *
         * @author Athanasios Krontiris
         */
        class motion_planning_query_t : public motion_query_t
        {
          public:

            
            motion_planning_query_t();
            virtual ~motion_planning_query_t();

            /**
             * @brief Initialize the planing query from input parameters.
             *
             * @param reader The reader for the parameters.
             * @param template_reader A template reader for reading template parameters 
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

            //TODO One more setup required or should we make constraints part of the other setups?
            /**
             * @brief Setups a new motion planning query.
             * @details Setups a new  motion planning query. This covers only the basic case where the query has only a single 
             * start and a singe goal.  Additionally, search modes and active constraints are to be considered during the current motion
             * planning query.
             * 
             * @param start_state The start state.
             * @param goal_state The goal state.
             * @param search_mode The search mode corresponds to ways the underlying search is invoked and constraints are dealt with.
             * @param active_constraints Set of active constrains that will be considered during the current motion planning query 
             */
            virtual void setup(sim::state_t* input_start_state, sim::state_t* input_goal_state, search_mode_t input_search_mode, const util::constraints_t* input_active_constraints, util::constraints_t* input_path_constraints,  bool should_update_constraints = false);

            /**
             * @brief Setups a new motion planning query (special case of not passing in a goal state).
             * @details Setups a new motion planning query. This is a special case where we know that we are separately doing a setup
             * for the internal goal_t.  It does not take a goal state so as to not overwrite information in the goal_t class instance.
             * 
             * @param start_state The start state.
             * @param search_mode The search mode corresponds to ways the underlying search is invoked and constraints are dealt with.
             * @param active_constraints Set of active constrains that will be considered during the current motion planning query 
             */
            virtual void setup(sim::state_t* input_start_state, search_mode_t input_search_mode, const util::constraints_t* input_active_constraints, util::constraints_t* input_path_constraints,  bool should_update_constraints = false);

            //TODO One more setup required or should we make constraints part of the other setups?
            /**
             * @brief Setups a new motion planning query.
             * @details Setups a new  motion planning query. This covers only the basic case where the query has only a single 
             * start and a singe goal.  Additionally, search modes is to be considered during the current motion
             * planning query.
             * 
             * @param start_state The start state.
             * @param goal_state The goal state.
             * @param search_mode The search mode corresponds to ways the underlying search is invoked and constraints are dealt with.
             */
            virtual void setup(sim::state_t* input_start_state, sim::state_t* input_goal_state, search_mode_t input_search_mode, bool should_update_constraints = false);


            /**
             * @brief Setups a new motion planning query.
             * @details Setups a new  motion planning query. This covers only the basic case where the query has only a single 
             * start and a singe goal. 
             * 
             * @param start_state The start state.
             * @param goal_state The goal state. 
             */
            virtual void setup(sim::state_t* input_start_state, sim::state_t* input_goal_state);

            virtual void setup(std::vector<sim::state_t*>& input_start_states, sim::state_t* input_goal_state, search_mode_t input_search_mode, const util::constraints_t* input_active_constraints, util::constraints_t* input_path_constraints,  bool should_update_constraints = false);

            virtual void setup(std::vector<sim::state_t*>& input_start_states, search_mode_t input_search_mode, const util::constraints_t* input_active_constraints, util::constraints_t* input_path_constraints,  bool should_update_constraints = false);

            virtual void setup(std::vector<sim::state_t*>& input_start_states, sim::state_t* input_goal_state, search_mode_t input_search_mode, bool should_update_constraints = false);

            virtual void setup(std::vector<sim::state_t*>& input_start_states, sim::state_t* input_goal_state);

            virtual void update_start_states(std::vector<sim::state_t*>& new_start_states);
            
            virtual void update_start_states(sim::state_t* new_start_state);

            /**
             * @brief Clear the plan and trajectory of this query.
             *
             * Clears the plan and the trajectory of the planning query in order to 
             * reuse the planning query with the same start and stopping criterion. 
             */
            virtual void clear();

            /**
             * @brief Returns the starting state for the query.
             *
             * @return The start state.
             */
            virtual sim::state_t* get_start_state() const;
            
            virtual std::vector<sim::state_t*> get_start_states() const;

            /**
             * @brief Returns the goal state for the query.
             *
             * @return The goal state.
             */
            virtual util::goal_t* get_goal() const;

            /**
             * @brief Links the state and control spaces that the planning query is working over.
             *
             * @param state_space The state space.
             * @param control_space The control space.
             */
            virtual void link_spaces(const util::space_t* input_state_space, const util::space_t* input_control_space);
            
            /**
             * 
             * @brief Copies the start state to the query.
             * 
             * The motion_planning_query_t has an internal state point that will copy
             * the given start state. It does not own the given state. The user has to
             * delete the given state later. 
             * 
             * @param start The state that will be coppied to the query.
             */
            virtual void copy_start(const sim::state_t* start);

            /**
             * @brief Set a group of start states.
             *
             * @param s_vec A vector containing start states to query from.
             */
            virtual void set_start_from_vector(const std::vector<double>& s_vec);

            /**
             * @brief Set the query goal state.
             *
             * The motion_planning_query_t will own this goal.Planning_query_t will
             * delete the goal at the end of the execution. 
             *
             * @param goal The util::goal_t that we want our query to end.
             */
            virtual void set_goal(util::goal_t* input_goal);

            /** @brief If we found a solution or not */
            bool found_solution;
            double solution_cost;

            // Search mode corresponds to ways the underlying search is invoked and constraints are dealt with.
            search_mode_t search_mode;

            // Set of active constrains that will be considered during the current motion planning query
            const util::constraints_t* active_constraints;

            // The path constraints for the solution
            util::constraints_t* path_constraints;

            /** @brief Indicates if we should update constraints on the planning structure. */
            bool update_constraints;


            /** @brief Indicates if we should restart the astar during seting up search */
            bool restart_astar_search;

            /** @brief Indicates how many times we should try the lazy search in the astar */
            int lazy_astar_iterations;

            /** @brief The start state belonging to the solution path in the query. */
            sim::state_t* satisfied_start_state;

            /** @brief The goal state belonging to the solution path in the query. */
            sim::state_t* satisfied_goal_state;
          protected:
            /** @brief Vector containing all possible start states in double form */ //TODO: Shouldn't this information be in sim::state_t* form?
            std::vector<double> start_vec;
            /** @brief Start state of the query. */
            sim::state_t* start_state;            
            /** @brief Multiple start states of the query. */
            std::vector<sim::state_t*> start_states;
            /** @brief Goal state of the query. */
            util::goal_t* goal;
        };
    }
}

#endif

