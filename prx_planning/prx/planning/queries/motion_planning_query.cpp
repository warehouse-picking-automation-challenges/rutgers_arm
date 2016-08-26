/**
 * @file motion_planning_query.cpp
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

#include "prx/planning/queries/motion_planning_query.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"

//TODO: Null Constraints
#include "prx/utilities/heuristic_search/null_constraints.hpp"

#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS( prx::plan::motion_planning_query_t, prx::plan::query_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace plan
    {

        motion_planning_query_t::motion_planning_query_t()
        {
            state_space = NULL;
            start_state = NULL;
            goal = NULL;
            solution_cost = 0;
            found_solution = false;
            active_constraints = NULL;
            update_constraints = false;
            restart_astar_search = true;
            path_constraints = NULL;
            satisfied_start_state = NULL;
            satisfied_goal_state = NULL;
            lazy_astar_iterations = PRX_INFINITY;
        }

        motion_planning_query_t::~motion_planning_query_t()
        {
            if( state_space == NULL )
                PRX_FATAL_S("Planning query does not have a linked state space!");

            clear();
            start_vec.clear();
            if( start_state != NULL )
                state_space->free_point(start_state);
            if( goal != NULL )
            {
                delete goal;
            }

        }

        void motion_planning_query_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            query_t::init(reader,template_reader);

            if( parameters::has_attribute("start_state", reader, template_reader) )
            {
                start_vec = parameters::get_attribute_as< std::vector<double> >("start_state", reader, template_reader);
            }
            else
            {
                PRX_WARN_S("Missing start state attribute from planning query.");
            }

            if( parameters::has_attribute("goal", reader, template_reader) )
            {
                std::string type = parameters::get_attribute_as<std::string > ("goal/type", reader, template_reader);
                goal = parameters::initialize_from_loader<goal_t>("prx_utilities",reader,"goal",template_reader,"goal");
            }
            else
            {
                PRX_WARN_S("Missing goal attribute from planning query.");
            }
            

            //TODO Search Mode replaces the existing modes
            if( parameters::has_attribute("search_mode", reader, template_reader) )
            {
                std::string mode_str = parameters::get_attribute_as<std::string > ("search_mode", reader, template_reader);
                if(mode_str == "standard_search")
                {
                    search_mode = STANDARD_SEARCH;
                }
                else if(mode_str == "lazy_search")
                {
                    search_mode = LAZY_SEARCH;
                }
                else if(mode_str == "trusted_mcr")
                {
                    search_mode = TRUSTED_MCR;
                }
                else if(mode_str == "untrusted_mcr")
                {
                    search_mode = UNTRUSTED_MCR;
                }
                else
                {
                    PRX_WARN_S("Invalid search mode set through input for motion planning query.");
                    search_mode = STANDARD_SEARCH;
                }
            }
            else
            {
                PRX_WARN_S("Missing search mode in input for motion planning query.");
            }

            restart_astar_search = parameters::get_attribute_as< bool >("restart_astar_search", reader, template_reader, true);
            lazy_astar_iterations = parameters::get_attribute_as< int >("lazy_astar_iterations", reader, template_reader, PRX_INFINITY);

            
        }

        void motion_planning_query_t::setup(state_t* input_start_state, state_t* input_goal_state, search_mode_t input_search_mode, const constraints_t* input_active_constraints, constraints_t* input_path_constraints,  bool should_update_constraints)
        {
            PRX_ASSERT(goal != NULL);
            copy_start(input_start_state);
            goal->copy_goal_state(input_goal_state);
            search_mode = input_search_mode;
            active_constraints =  input_active_constraints;
            path_constraints = input_path_constraints;
            update_constraints = should_update_constraints;
            clear();
        }

        void motion_planning_query_t::setup(state_t* input_start_state, search_mode_t input_search_mode, const constraints_t* input_active_constraints, constraints_t* input_path_constraints,  bool should_update_constraints)
        {
            PRX_ASSERT(goal != NULL);
            copy_start(input_start_state);
            search_mode = input_search_mode;
            active_constraints =  input_active_constraints;
            path_constraints = input_path_constraints;
            update_constraints = should_update_constraints;
            clear();
        }

        void motion_planning_query_t::setup(state_t* input_start_state, state_t* input_goal_state, search_mode_t input_search_mode, bool should_update_constraints)
        {
            setup(input_start_state, input_goal_state, input_search_mode, active_constraints, path_constraints, should_update_constraints);
        }

        void motion_planning_query_t::setup(state_t* input_start_state, state_t* input_goal_state)
        {
            setup(input_start_state, input_goal_state, search_mode, active_constraints, path_constraints, false);
        }





        void motion_planning_query_t::setup(std::vector<state_t*>& input_start_states, state_t* input_goal_state, search_mode_t input_search_mode, const constraints_t* input_active_constraints, constraints_t* input_path_constraints,  bool should_update_constraints)
        {
            setup(input_start_states[0], input_goal_state, input_search_mode, input_active_constraints, input_path_constraints, should_update_constraints);
            update_start_states(input_start_states);
        }

        void motion_planning_query_t::setup(std::vector<state_t*>& input_start_states, search_mode_t input_search_mode, const constraints_t* input_active_constraints, constraints_t* input_path_constraints,  bool should_update_constraints)
        {
            setup(input_start_states[0], input_search_mode, input_active_constraints, input_path_constraints,  should_update_constraints);
            update_start_states(input_start_states);
        }

        void motion_planning_query_t::setup(std::vector<state_t*>& input_start_states, state_t* input_goal_state, search_mode_t input_search_mode, bool should_update_constraints)
        {
            setup(input_start_states, input_goal_state, input_search_mode, active_constraints, path_constraints, should_update_constraints);
        }

        void motion_planning_query_t::setup(std::vector<state_t*>& input_start_states, state_t* input_goal_state)
        {
            setup(input_start_states, input_goal_state, search_mode, active_constraints, path_constraints, false);
        }

        void motion_planning_query_t::clear()
        {
            // PRX_WARN_S("clearing motion_planning_query_t");
            if( state_space == NULL )
                PRX_FATAL_S("Planning query does not have a linked state space!");
            
            plan.clear();
            path.clear();
            found_solution = false;
            // TODO: Do the active constraints need to be cleared here?

            // TODO: Do the path constraints need to be cleared here?
        }

        state_t* motion_planning_query_t::get_start_state() const
        {
            return start_state;
        }

        std::vector<state_t*> motion_planning_query_t::get_start_states() const
        {
            return start_states;
        }

        goal_t* motion_planning_query_t::get_goal() const
        {
            return goal;
        }

        void motion_planning_query_t::link_spaces(const space_t* input_state_space, const space_t* input_control_space)
        {
            if(state_space != NULL && state_space->get_space_name() != state_space->get_space_name())
                PRX_FATAL_S("You cannot assign different space in the motion_planning_query_t!");
            state_space = input_state_space;
            control_space = input_control_space;
            start_state = state_space->alloc_point();
            if( !start_vec.empty() )
                state_space->copy_vector_to_point(start_vec, start_state);
            if(goal != NULL)
                goal->link_space(state_space);            
            plan.link_control_space(control_space);
            path.link_space(state_space);
        }
        
        void motion_planning_query_t::copy_start(const state_t* start)
        {
            PRX_ASSERT(state_space != NULL);
            state_space->copy_point(start_state,start);
            update_start_states(start_state);
        }

        void motion_planning_query_t::set_start_from_vector(const std::vector<double>& s_vec)
        {
            start_vec.clear();
            for( unsigned i = 0; i < s_vec.size(); i++ )
            {
                start_vec.push_back(s_vec[i]);
            }
            PRX_ASSERT(state_space != NULL);
            state_space->set_from_vector(s_vec,start_state);

        }

        void motion_planning_query_t::set_goal(goal_t* input_goal)
        {
            PRX_ASSERT(input_goal != NULL);
            if( goal != NULL )
            {
                //        PRX_INFO_S("Deleting the old goal state in the planning query in order to replace it with a new");
                delete goal;
            }
            goal = input_goal;
        }

        void motion_planning_query_t::update_start_states(std::vector<state_t*>& new_start_states)
        {
            foreach(state_t* start_st, start_states)
            {
                state_space->free_point(start_st);
            }
            start_states.clear();
            foreach(state_t* start_st, new_start_states)
            {
                start_states.push_back(state_space->clone_point(start_st));
            }
        }
        void motion_planning_query_t::update_start_states(state_t* new_start_state)
        {
            std::vector<state_t*> new_start_states;
            new_start_states.push_back(new_start_state);
            update_start_states(new_start_states);
        }
    }
}
