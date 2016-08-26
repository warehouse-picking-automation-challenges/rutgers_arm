 /**
 * @file gta_planner.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Andrew Kimmel, Andrew Dobson, Athanasios Krontiris, Zakary Littlefield,  Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#pragma once
#ifndef PRX_REPLANNING_GTA_HPP
#define	PRX_REPLANNING_GTA_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "planning/task_planners/deconfliction/replanning_gta_query.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/task_planners/task_planner.hpp"
#include "planning/task_planners/deconfliction/action.hpp"
// Motion planners used for compute action set
#include "../../../../homotopies/planning/motion_planners/subhrajit/h_graph_planner.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"

#include <boost/numeric/ublas/matrix.hpp>
#include <queue>


namespace prx
{
    namespace util
    {
        class linear_distance_metric_t;
    }

    namespace plan
    {
        class timed_criterion_t;
    }
    
    namespace packages
    {
        namespace gta
        {
            /**
             * @class replanning_gta_planner_t
             * 
             * A simple task planner that simply will run a motion planner.
             * 
             */
            class replanning_gta_planner_t : public plan::task_planner_t
            {    
            protected:
                                
                /** From previous version */
                replanning_gta_query_t* gta_query;
                plan::motion_planning_query_t* my_query;
                sim::state_t* my_query_start;
                plan::stopping_criteria_t* my_stopping_criteria;
                plan::criterion_t* my_goal_criterion;

                plan::motion_planner_t* my_motion_planner;
                packages::homotopies::h_graph_planner_t* h_graph_action_generator;
                
                sim::state_t *temp_state, *predicted_state;
                util::linear_distance_metric_t* goal_metric;
                double reconnection_radius;
                double planning_duration;
                double window, delta;
                
                double velocity, disk_radius, horizon_factor;
                
                util::sys_clock_t comp_timer;
                unsigned iteration_counter;
                bool once;
                
                std::vector<std::string> color_map;
                bool visualize_actions;
                
                
                /** BEGIN REFACTOR */
                unsigned num_greedy_actions; // g
                std::vector<sim::trajectory_t> k_greedy_paths;
                std::vector<sim::plan_t> k_greedy_plans;
                sim::trajectory_t greedy_strategy_path;
                sim::plan_t greedy_strategy_plan; // red
                
                bool use_safety; // s
                sim::trajectory_t safety_strategy_path;
                sim::plan_t safety_strategy_plan; // green
                
                bool consistent; // c
                sim::trajectory_t consistent_strategy_path;
                sim::plan_t consistent_strategy_plan; // blue
                
                bool deterministic;
                double eta;
                double interaction_factor;
                
                action_t* selected_strategy;
                sim::state_t* last_valid_state;
                unsigned selected_greedy_index;
                
                unsigned total_strategies;
                std::vector<action_t> strategy_vector;
                std::vector<double> weight_vector;
                
                /** Used to determine waiting*/
                bool waiting;
                double angle_threshold;
                double sensing_radius;
                double greedy_ratio;
                util::vector_t a, b;
                util::vector_t prev_selected_vector, current_selected_vector;
                action_t previous_selected_action;
                sim::trajectory_t previous_selected_path;
                sim::plan_t previous_selected_plan; // blue
                
                // Functions
                bool strategy_selection();
                
                // protected
                void generate_consistent(unsigned start_index, const std::vector<sim::state_t*>& neighbor_states);
                void generate_greedy(unsigned start_index, const std::vector<sim::state_t*>& neighbor_states);
                void generate_safe(unsigned start_index, const std::vector<sim::state_t*>& neighbor_states);
                
                action_t* polynomial_weights();
                
                bool pre_waiting_check();
                bool post_waiting_check();
                
                bool compute_predicted_state(); // saves this predicted state onto predicted_state
                double compute_interaction_ratio(sim::trajectory_t* executed_path, const std::vector<sim::state_t*>& neighbor_states);
                bool reconnect_action (action_t* to_connect, sim::state_t* connect_state);


            public:
                
                replanning_gta_planner_t();
                ~replanning_gta_planner_t();

                virtual void init(const util::parameter_reader_t* reader,const util::parameter_reader_t* template_reader=NULL);

                virtual void setup();

                virtual bool execute();

                virtual const util::statistics_t* get_statistics();

                virtual bool succeeded() const;

                virtual void link_query(plan::query_t* in_query);

                virtual void resolve_query();

                void set_planning_duration(double new_duration);
                
            protected:
                virtual void resize_and_allocate();
                virtual void update_vis_info() const;
//
//                // The different algorithms
//                /** ALG 2 */
//                bool standard_rrvo_execute();
//                
//                /** ALG 4 and 5 */
//                bool the_one_method_to_rule_them_all();
//                
//                
//                bool pre_waiting_condition();
//                bool check_waiting_condition();
//                
//                unsigned total_actions;
//                
//                std::vector<sim::trajectory_t> greedy_trajectory_memory;
//                std::vector<sim::plan_t> greedy_plans_memory;
//                unsigned k_greedy_actions; unsigned num_greedy_actions;
//                void compute_best_scaled_greedy_strategies(unsigned start_action_index, int k_actions, const std::vector<sim::state_t*>& neighbor_states);
//                
//                std::vector<sim::trajectory_t> safety_trajectory_memory;
//                std::vector<sim::plan_t> safety_plans_memory;
//                unsigned num_safety_actions;
//                void compute_saftey_strategies(unsigned start_action_index, int k_actions, const std::vector<sim::state_t*>& neighbor_states);
//                void compute_previous_else_safety_strategy(unsigned start_action_index,  const std::vector<sim::state_t*>& neighbor_states, bool always_safe = false);
//                bool use_min_conflict, use_k_best;
//                
//                double compute_interaction_ratio(sim::trajectory_t* executed_path, const std::vector<sim::state_t*>& neighbor_states);
//                                
//                
//                // GTA FUNCTIONS
//                virtual void compute_action_set(bool execute_planner);
//                virtual double compute_action_cost(action_t* my_action);
//                virtual void reduce_action_set(int new_action_limit, std::vector<sim::trajectory_t>& trajectories,  std::vector<sim::plan_t>& plans, bool seed = true);
//                        // Replanning parameters
//                double planning_duration, duration_scaling;
//
//                    // TODO functions
//                virtual void reconnect_to_actions();
//                virtual void reconnect_action(action_t* to_connect_action, sim::state_t* connect_state);
//
//
//                    // Might not be needed!
//            //    virtual double compute_penalty(action_t* my_action, action_t* neighbor_action);
//
//
//                virtual void resize_and_allocate();
//                
//
//                // --------------------------------- 
//
//                // GTA VARIABLES
//                    // The gta query
//                
//                std::vector<plan::criterion_t*> stats_criteria;
//
//                    // Collision penalty radii
//                double my_radius, neighbor_radius;
//
//
//                    // Important matrices to the standard GTA algorithm
//                std::vector< double > regret_vector;
//                    //limits
//                int action_limit, prm_max_iterations;
//                    // counters
//                int num_neighbors;
//                    // action sets
//                std::vector<action_t> my_action_set;
//                std::vector<int> my_action_index;
//                std::vector<int> my_plan_index;
//                std::vector<sim::plan_t> my_previous_plan_memory;
//
//                    // my motion planning queries
//                
//                
//                
//
//                    // Goal metric
//                
//                    // action set generators (as motion planners)
//                
//                    // Visualization flags (which actions do we wish to see?)
//                bool visualize_actions, compare_actions, visualize_neighbors;
//                    // The index of the minmax action
//                int minmax_index;
//
//                  // Visualization colors
//                
//                  // Flags
//                bool online_planning, once;
//                
//                  // My action generators
//                
//                
//                
//                double reconnection_radius;
//                int window;
//
//                    // Learning vector
//                std::vector< double > learning_vector;
//                , *temp_state, *my_query_start;
//
//                plan::timed_criterion_t* t_criterion;
//                int straight_up_counter;
//                int prev_minmax;
//
//                int max_exploratory_actions;
//                std::vector<sim::trajectory_t> exploratory_actions_memory;
//                std::vector<sim::plan_t> exploratory_plans_memory;
//                
//                std::vector<sim::trajectory_t> replaced_actions_memory;
//                std::vector<sim::plan_t> replaced_plans_memory;
//
//                sim::plan_t dummy_plan;
//                sim::trajectory_t dummy_trajectory1, dummy_trajectory2;
//
//                double velocity, delta;
//                double eta;
//
//                double disk_radius;
//
//                // Magical method variables
//                std::vector<homotopies::complex_t> h_signatures;
//                std::vector<homotopies::complex_t> previous_h_signatures;
//
//                
//                std::vector<int> index_list;
//                double horizon_factor;
//
//                bool agent_waits;
//                sim::plan_t wait_plan;
//                sim::trajectory_t wait_path;
//                double alpha;
//                
//                // bool guards
//                bool the_one_method;
//                
//                double interaction_factor;
//                
//                
//                

                
                
                
                
                
 

            };
        }
    }
}

#endif	// PRX_REPLANNING_GTA_HPP

