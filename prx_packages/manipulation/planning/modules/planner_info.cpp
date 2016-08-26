/**
 * @file planner_info.cpp
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

#include "planning/modules/planner_info.hpp"
#include "planning/queries/pose_based_mp_query.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/goals/multiple_goal_states.hpp"

#include "simulation/workspace_trajectory.hpp"

#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"

#include <boost/assign/list_of.hpp>
#include <boost/range/adaptor/map.hpp>
#include <pluginlib/class_list_macros.h>

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace manipulation
        {

            planner_info_t::planner_info_t(std::string input_construction_context_name, std::string input_planning_context_name, planner_t* input_planner, motion_planning_specification_t* input_specification, query_t* input_query)
            {
                construction_context_name = input_construction_context_name;
                planning_context_name = input_planning_context_name;

                planner = dynamic_cast<motion_planner_t*>(input_planner);
                if(planner == NULL)
                    PRX_FATAL_S("Manipulation task planner can only work with motion planners!");
                query = dynamic_cast<motion_planning_query_t*>(input_query);
                if(query == NULL)
                    PRX_FATAL_S("Manipulation task planner van have only motion planning queries as output queries.");
                specification = input_specification;
                heuristic_search_mode = STANDARD_SEARCH;

                tmp_constraints = NULL;
                checker = specification->validity_checker;
                manip_validity_checker = dynamic_cast< manipulation_validity_checker_t* >( checker );
                state_space = NULL;
                control_space = NULL;
                start_point = NULL;
            }

            planner_info_t::~planner_info_t()
            {
                state_space->free_point(start_point);

                if (tmp_constraints != NULL)
                {
                    if (checker != NULL)
                    {
                        checker->free_constraint(tmp_constraints);
                    }
                    else
                    {
                        PRX_ERROR_S ("Invalid state, validity checker wrongfully deleted.");
                    }
                }
            }

            void planner_info_t::setup(manipulation_world_model_t* input_manip_model)
            {                    

                // Get the manipulation model and switch to the correct context
                manip_model = input_manip_model;
                std::string old_context = manip_model->get_current_context();
                PRX_PRINT("\n\nSetting up planner info, storing old context: " << old_context, PRX_TEXT_BLUE);
                manip_model->use_context(planning_context_name);

                state_space = manip_model->get_state_space();
                control_space = manip_model->get_control_space();
                start_point = state_space->alloc_point();
                specification->link_spaces(state_space, control_space);
                specification->setup(manip_model);
                query->link_spaces(state_space, control_space);   
                planner->link_specification(specification);
                specification->get_stopping_criterion()->link_motion_planner(planner);

                

                // Switch back to the previous context
                manip_model->use_context(old_context);

            }

            void planner_info_t::setup_search(search_mode_t input_heuristic_search_mode, const constraints_t* input_valid_constraints, std::vector< std::string >& input_constraint_names)
            {
                heuristic_search_mode = input_heuristic_search_mode;
                if (specification->astar != NULL)
                {
                    specification->astar->setup_astar_search( input_heuristic_search_mode, input_valid_constraints );
                }
                else
                {
                    PRX_WARN_S ("Warning: Setup search called with no A*");
                }

                if (tmp_constraints != NULL)
                {
                    if (checker != NULL)
                    {
                        checker->free_constraint(tmp_constraints);
                    }
                    else
                    {
                        PRX_FATAL_S ("Invalid state, validity checker wrongfully deleted.");
                    }
                }
                valid_constraints = input_valid_constraints;
                tmp_constraints = checker->alloc_constraint();
                constraint_names = input_constraint_names;
            }

            bool planner_info_t::compute_solution(plan_t& plan, constraints_t* path_constraints, std::vector<state_t*> start_states,  state_t* input_goal_state, bool ik_steer, bool set_constraints)
            {
                std::vector<state_t*> goal_states = boost::assign::list_of(input_goal_state);
                return compute_solution(plan, path_constraints, start_states,  goal_states, ik_steer, set_constraints);
            }


            bool planner_info_t::compute_solution(plan_t& plan, constraints_t* path_constraints, std::vector<state_t*> start_states, const std::vector< sim::state_t* >& input_goal_states, bool ik_steer, bool set_constraints)
            {
                PRX_PRINT("===================== BEGIN COMPUTE SOLUTION (MULTI START) =============================", PRX_TEXT_GREEN);

                multiple_goal_states_t* goal = dynamic_cast< multiple_goal_states_t* >( query->get_goal() );


                //First, let's set up the goal states we will use
                std::vector< state_t* > actual_goal_states;
                
                if (goal != NULL)
                {
                    foreach( state_t* original_state, input_goal_states )
                    {
                        manip_model->get_state_space()->copy_from_point(original_state);
                        actual_goal_states.push_back( state_space->alloc_point() );
                        
                        PRX_PRINT("Goal: " << state_space->print_point( actual_goal_states.back(), 5 ), PRX_TEXT_GREEN );
                    }
                }

                std::string old_context = manip_model->get_current_context();
                manip_model->use_context(planning_context_name);

                query->setup(start_states, heuristic_search_mode, valid_constraints, path_constraints, set_constraints);
                if (goal != NULL)
                {
                    goal->clear();
                    goal->append_multiple_goal_states( actual_goal_states, actual_goal_states.size() );
                }

                planner->link_query(query);

                planner->resolve_query();

                foreach( state_t* st, actual_goal_states )
                {
                    state_space->free_point( st );
                }

                manip_model->use_context(old_context);

                if(query->found_solution)
                {
                    manip_model->convert_plan(plan, manip_model->get_current_manipulation_info()->full_arm_control_space, query->plan, control_space);
                    PRX_PRINT("======================= GOOD COMPUTE SOLUTION ===============================", PRX_TEXT_CYAN);
                    
                    return true;
                }
                PRX_PRINT("======================== BAD COMPUTE SOLUTION ============================", PRX_TEXT_RED);
                return false;
            }





            bool planner_info_t::compute_solution(sim::plan_t& plan, util::constraints_t* path_constraints, sim::state_t* start_state,  sim::state_t* input_goal_state, bool ik_steer, bool set_constraints)
            {
                std::vector<state_t*> goal_states = {input_goal_state};
                return compute_solution(plan, path_constraints, start_state,  goal_states, ik_steer, set_constraints);
            }

            bool planner_info_t::compute_solution(plan_t& plan, constraints_t* path_constraints, state_t* start_state, const std::vector< state_t* >& input_goal_states, bool ik_steer, bool set_constraints)
            {

                PRX_PRINT("===================== BEGIN COMPUTE SOLUTION (SINGLE START) =============================", PRX_TEXT_GREEN);

                // if( !manip_model->valid_state( start_state ) )
                // {
                //     PRX_ERROR_S("YOU GAVE A START STATE WHICH WAS IN COLLISION!  FIX IT!!!!");
                    
                //     foreach( collision_pair_t p, manip_model->get_colliding_bodies()->get_body_pairs() )
                //     {
                //         PRX_PRINT("[" << p.first << "] - [" << p.second << "]", PRX_TEXT_LIGHTGRAY);
                //     }
                    
                //     PRX_ERROR_S( manip_model->get_state_space()->print_point( start_state, 5 ) );
                //     PRX_FATAL_S( manip_model->get_full_state_space()->print_memory( 5 ) );
                // }
                multiple_goal_states_t* goal = dynamic_cast< multiple_goal_states_t* >( query->get_goal() );

                workspace_trajectory_t ee_traj;

                if(ik_steer)
                {
                    for( unsigned i=0; i<input_goal_states.size(); ++i )
                    {
                        state_t* goal_state = input_goal_states[i];

                        // PRX_PRINT("Trying IK steering first...",PRX_TEXT_BLUE);
                        config_t goal_config;
                        manip_model->get_state_space()->copy_from_point(goal_state);
                        manip_model->FK(goal_config);
                        state_t* result_state = manip_model->get_state_space()->alloc_point();
                        plan_t new_plan(manip_model->get_control_space());

                        PRX_PRINT("IK steering...",PRX_TEXT_GREEN);
                        bool success = manip_model->jac_steering( new_plan, ee_traj, result_state, start_state, goal_state, goal_config);
                        manip_model->get_state_space()->free_point(result_state);
                        trajectory_t traj(manip_model->get_state_space());
                        if(success)
                        {
                            specification->local_planner->propagate(start_state,new_plan,traj);
                            bool valid_constrained_trajectory = false;
                            tmp_constraints->clear();
                            
                            if( manip_validity_checker != NULL )
                            {
                                valid_constrained_trajectory = manip_validity_checker->validate_and_generate_constraints(tmp_constraints, traj, ee_traj);
                            }
                            else
                            {
                                valid_constrained_trajectory = checker->validate_and_generate_constraints(tmp_constraints, traj);
                            }

                            if(valid_constrained_trajectory)
                            {
                                PRX_PRINT("Valid IK steering trajectory...",PRX_TEXT_CYAN);
                                if (heuristic_search_mode == STANDARD_SEARCH)
                                {
                                    if (valid_constraints->has_intersection(tmp_constraints))
                                        continue;
                                }
                                path_constraints->merge(tmp_constraints);
                                manip_model->convert_plan(plan, manip_model->get_current_manipulation_info()->full_arm_control_space, new_plan, manip_model->get_control_space());
                                //plan+=new_plan;
                                if (goal != NULL)
                                {
                                    goal->clear();
                                }
                                return true;
                            }
                        }
                    }

                    // PRX_PRINT("Failed IK steering. Continuing to query roadmap...",PRX_TEXT_BROWN);
                }

                //First, let's set up the goal states we will use
                std::vector< state_t* > actual_goal_states;
                
                if (goal != NULL)
                {
                    foreach( state_t* original_state, input_goal_states )
                    {
                        manip_model->get_state_space()->copy_from_point(original_state);
                        actual_goal_states.push_back( state_space->alloc_point() );
                    }
                }

                //Then get the appropriate start state, making sure to move the object back to the hand
                manip_model->get_state_space()->copy_from_point(start_state);
                state_space->copy_to_point(start_point);

                std::string old_context = manip_model->get_current_context();
                manip_model->use_context(planning_context_name);

                query->setup(start_point, heuristic_search_mode, valid_constraints, path_constraints, set_constraints);
                if (goal != NULL)
                {
                    goal->clear();
                    goal->append_multiple_goal_states( actual_goal_states, actual_goal_states.size() );
                }

                planner->link_query(query);
                planner->resolve_query();

                foreach( state_t* st, actual_goal_states )
                {
                    state_space->free_point( st );
                }

                manip_model->use_context(old_context);

                if(query->found_solution)
                {
                    manip_model->convert_plan(plan, manip_model->get_current_manipulation_info()->full_arm_control_space, query->plan, control_space);
                    PRX_PRINT("======================= GOOD COMPUTE SOLUTION (" << query->plan.length() << ") ===============================", PRX_TEXT_CYAN);
                    
                    return true;
                }
                PRX_PRINT("======================== BAD COMPUTE SOLUTION ============================", PRX_TEXT_RED);
                return false;
            }
            
            bool planner_info_t::compute_solution( plan_t& plan, constraints_t* path_constraints, state_t* start_state, state_t* reached_state, const config_t& target_config, std::string end_effector, std::string input_context, bool set_constraints)
            {
                PRX_PRINT("===================== BEGIN COMPUTE SOLUTION (TO CONFIG) =============================", PRX_TEXT_GREEN);
                PRX_PRINT("Setting up with end-effector: " << end_effector, PRX_TEXT_MAGENTA);
                PRX_PRINT("Current Context: "<< manip_model->get_current_context(),PRX_TEXT_MAGENTA);
                //Then get the appropriate start state, making sure to move the object back to the hand
                manip_model->get_state_space()->copy_from_point(start_state);
                state_space->copy_to_point(start_point);

                std::string old_context = manip_model->get_current_context();
                manip_model->use_context( input_context ); //What planning context will this be though...?

                //We need to make sure we have specifically a pose-based motion planning query
                pose_based_mp_query_t* pose_query = dynamic_cast< pose_based_mp_query_t* >( query );
                pose_query->setup( start_point, heuristic_search_mode, target_config, end_effector, valid_constraints, path_constraints, set_constraints);

                planner->link_query(query);
                planner->resolve_query();
                manip_model->use_context(old_context);
                
                reached_state = pose_query->satisfied_goal_state;

                if(query->found_solution)
                {
                    manip_model->convert_plan(plan, manip_model->get_current_manipulation_info()->full_arm_control_space, query->plan, control_space);
                    PRX_PRINT("======================= GOOD COMPUTE SOLUTION ===============================", PRX_TEXT_CYAN);
                    
                    return true;
                }
                PRX_PRINT("======================== BAD COMPUTE SOLUTION ============================", PRX_TEXT_RED);
                return false;                
            }

        }
    }
}
