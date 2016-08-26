/**
 * @file manipulation_tp.hpp
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

#ifndef PRX_MANIPULATION_TP_HPP
#define	PRX_MANIPULATION_TP_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/spaces/space.hpp"

#include "prx/simulation/plan.hpp"
#include "prx/simulation/trajectory.hpp"

#include "prx/planning/task_planners/task_planner.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/queries/query.hpp"
#include "prx/planning/problem_specifications/specification.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/modules/heuristic_search/constraints_astar_search.hpp"

#include "utilities/definitions/manip_defs.hpp"

#include "planning/distance_functions/end_effector_distance.hpp"
#include "planning/manipulation_world_model.hpp"
#include "planning/modules/planner_info.hpp"
#include "planning/queries/manipulation_query.hpp"
#include "planning/queries/grasping_query.hpp"
#include "planning/task_planners/grasping_planner.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "planning/modules/manipulation_validity_checker.hpp"

namespace prx
{
    namespace util
    {
        class multiple_goal_states_t;
    }

    namespace packages
    {
        namespace manipulation
        {
            class half_space_goal_t;
            class manipulation_specification_t;
            class grasping_specification_t;            
            /**
             * Manipulation task planner. Computes the path for moving an object from an
             * initial to a target position.
             *
             * @authors Andrew Dobson, Andrew Kimmel, Rahul Shome
             */
            class manipulation_tp_t : public plan::task_planner_t
            {

              public:

                manipulation_tp_t();
                virtual ~manipulation_tp_t();

                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                /**
                 * @copydoc motion_planner_t::reset()
                 */
                virtual void reset();

                /**
                 * @copydoc motion_planner_t::link_world_model()
                 */
                virtual void link_world_model(plan::world_model_t * const model);

                /**
                 * @copydoc motion_planner_t::get_statistics()
                 */
                virtual const util::statistics_t* get_statistics();

                /**
                 * @copydoc planner_t::link_specification(specification_t*)
                 */
                virtual void link_specification(plan::specification_t* new_spec);

                /**
                 * @copydoc motion_planner_t::link_query()
                 */
                virtual void link_query(plan::query_t* new_query);

                /**
                 * @copydoc motion_planner_t::setup()
                 *
                 * Will occupy memory for the random_open_point and the new_control, after
                 * planning_query has been linked.
                 */
                virtual void setup();

                /**
                 * @copydoc motion_planner_t::execute()
                 */
                virtual bool execute();


                /**
                 * @copydoc motion_planner_t::succeeded() const
                 */
                virtual bool succeeded() const;

                /**
                 * @copydoc motion_planner_t::resolve_query()
                 *
                 * At the end of the resolve_query the algorithm will remove the vertices
                 * for the start and the goal for this specific query from the graph.
                 */
                virtual void resolve_query();

                virtual void resolve_task();

                virtual bool serialize();
                virtual bool deserialize();

                std::vector< grasp_t > get_grasps( std::vector<sim::state_t* >& input_states );
                
                int get_nr_grasps(std::string manipulation_context_name, movable_body_plant_t* requested_object)
                {
                    return grasping_planner->nr_grasps(manipulation_context_name, requested_object);
                }


                double _grasp_success_time, _grasp_failure_time, _mp_success_time, _mp_failure_time;
                int _grasp_success_count, _grasp_failure_count, _mp_success_count, _mp_failure_count;

              protected:

                /** @brief When doing a pick with the FEASIBLE mode, you may want to evaluate the grasps in a random order. */
                double random_feasible_grasp;

                bool IK_steer_movements;

                manipulation_world_model_t* manipulation_model;
                manipulation_context_info_t* current_manipulation_context_info;

                manipulation_specification_t* manip_specification;

                /** @brief A map from planning context to manipulation contexts (planner, query and spaces)*/
                util::hash_t<std::string, planner_info_t*> planner_info_map;
                /** @brief A map from planning context to manipulation contexts (planner, query and spaces)*/
                util::hash_t<std::string, planner_info_t*> connection_info_map;
                /** @brief The current active context (planner, query and spaces)*/
                planner_info_t* active_planner;

                /** @brief The input manipulation query */
                manipulation_query_t* manipulation_query;

                /** @brief The grasping planner.*/
                grasping_planner_t* grasping_planner;
                /** @brief The grasping query that the manipulation task planner will use to communicate with the grasping planner.*/
                grasping_query_t* grasping_query;
                /** @brief The grasping planner's specification */
                grasping_specification_t* grasping_specification;
                /** @brief The grasping query for the feasibility tests. */
                grasping_query_t* feasible_query;
                
                /** @brief The names for all of the constraints */
                std::vector< std::string > constraint_names;

                //Helping variable used for the pick and place.
                util::constraints_t* tmp_constraint;

                /** @brief A distance function which operates in the workspace of the end-effector */
                std::vector< end_effector_distance_t* > ee_distances;

                /**
                 * @brief If we want to serialize all the roadmaps after we finish building them. 
                 * If want to serialize the roadmaps the motion planners have to read the output file
                 * from their input. Otherwise the serialization will failed. (AD: What?)
                 */
                // bool serialize_flag;
                
                /** @brief Whether to use the End-Effector distance in the planners. */
                bool use_ee_dist_in_planners;

                /** @brief Defines a plane-based goal **/
                half_space_goal_t* shelf_plane_goal;
                std::vector<double> shelf_plane;
                bool half_space_side;

                /** @brief Whether the grasp fails if the connection phase does not succeed */
                bool force_connection_plans;

                /** @brief Determines whether the order of grasps is randomized **/
                bool randomized_grasp_order;

                /**
                 * @brief It computes all the information we need for the object at the object_intial_state.
                 * @details It computes all the information we need for the object at the object_states. If the states
                 * are more than one then the grasping planner will check only the grasps that are working for all the
                 * different states. 
                 * 
                 * @param object_initial_state [description]
                 * @param grasping_index [description]
                 * @param default_open_mode [description]
                 * @return [description]
                 */
                virtual void restore_grasping_mode( double previous_grasping_mode, sim::state_t* start_state);
                virtual bool resolve_grasp_query(std::vector<sim::state_t*>& object_states,  grasping_query_type_t grasping_query_mode, int default_open_mode, grasp_t* suggested_grasp = NULL);
                virtual bool resolve_connection( sim::plan_t& output_plan, sim::trajectory_t& output_path, util::constraints_t* output_constraints, sim::state_t* output_state, const std::vector< sim::state_t* >& goal_states, planner_info_t* input_connection_planner, sim::state_t* grasp_start_state, sim::state_t* connection_start_state);

                
                virtual bool attempt_IK_steer(sim::plan_t& output_plan, util::constraints_t* output_constraints, sim::state_t* start_state, sim::state_t* goal_state);
                virtual void generate_goal(std::vector<sim::state_t*>& candidate_goals, sim::state_t* the_start_state, sim::state_t* object_state, grasp_data_t* grasp_data);

                virtual bool compute_connection_plan(sim::state_t* start_state, sim::state_t* object_state, grasp_data_t* grasp_data, unsigned state_index );

                virtual bool compute_motion_plan(sim::plan_t& the_plan, util::constraints_t* the_constraints, const std::vector<sim::state_t*>& object_states, const std::vector<grasp_data_t*>& the_datas, sim::state_t* the_start, sim::state_t* the_goal);

                
                //DEBUG CHUPLES
                void test_roadmap_queries();
                

                /**
                 * @brief Convenience function to force the mode to be trusted
                 */
                plan::search_mode_t force_untrusted_mode( plan::search_mode_t input_mode );

                /**
                 * @brief Will execute the pick action. 
                 * @details Will execute the pick action. The function will return the plan that the manipulator has 
                 * to execute and the constraints, if there are constraints, that this plan will have. The returned plan
                 * and constraints will append to the existing ones. This function will not clean the data already in the 
                 * plan and the constraints.
                 * 
                 * @param plan The plan that this function will return for the manipulator to execute. 
                 * @param path_constraints The possible constraints that this action will have. 
                 * @param manipulator_final_state The final state that the manipulator will be after the execution of the pick action.
                 * @param manipulator_initial_state The initial state for the manipulator
                 * @param objest_picking_state The state from where the manipulator will pick up the object.
                 * @param path_quality The quality of the path, that can be either optima, feasible or suggested.
                 * @param suggested_relative_config If the mode of the quality is suggested then the user has to specify and the suggested
                 * relative configuration that the user would like to test. 
                 * 
                 * @return True if a solution for the pick action is found, otherwise false. 
                 */
                virtual bool execute_move(sim::plan_t& plan, util::constraints_t* path_constraints, sim::state_t* manipulator_initial_state, sim::state_t* manipulator_target_state);
                virtual bool execute_move_to_config(sim::plan_t& plan, util::constraints_t* path_constraints, sim::state_t* manipulator_initial_state, const util::config_t& manipulator_target_pose);
                virtual bool execute_pick(sim::plan_t& plan, util::constraints_t* path_constraints, grasp_data_t* data, sim::state_t* actual_start_state);
                virtual bool execute_place(sim::plan_t& plan, util::constraints_t* path_constraints, grasp_data_t* data, sim::state_t* actual_start_state);
                virtual bool execute_pick_and_place(sim::plan_t& plan, util::constraints_t* path_constraints, grasp_data_t* pick_data, grasp_data_t* place_data);
                virtual bool execute_pick_and_move(sim::plan_t& plan, util::constraints_t* path_constraints, grasp_data_t* pick_data, sim::state_t* manipulator_initial_state, sim::state_t* manipulator_target_state);
                virtual bool execute_pick_via_config_and_move(sim::plan_t& plan, util::constraints_t* path_constraints, grasp_data_t* pick_data, sim::state_t* manipulator_initial_state, sim::state_t* manipulator_target_state);
                
                virtual void engage_grasp_after_pick(sim::plan_t& plan, grasp_data_t* data );

                virtual void smooth_path(sim::plan_t& input_plan, sim::state_t* start_state, bool object_moves, int max_offset = 5);
                
                /**
                 * @copydoc planner_t::update_vis_info() const
                 */
                virtual void update_vis_info() const;

              private:
                sim::plan_t engage_grasp_plan;

                unsigned num_candidate_connections;

                util::sys_clock_t _grasp_clock;
                util::sys_clock_t _mp_clock;

                bool skip_connectivity_check;
                bool apc_retract;

                manipulation_validity_checker_t* manip_validity_checker;
            };
        }
    }
}


#endif
