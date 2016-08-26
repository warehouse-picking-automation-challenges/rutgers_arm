/**
 * @file manipulation_query.hpp
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

#ifndef PRX_MANIPULATION_QUERY_HPP
#define	PRX_MANIPULATION_QUERY_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/heuristic_search/constraints.hpp"
#include "prx/planning/modules/heuristic_search/constraints_astar_search.hpp"
#include "prx/simulation/state.hpp"
#include "prx/simulation/plan.hpp"
#include "prx/simulation/trajectory.hpp"
#include "prx/planning/queries/motion_query.hpp"

#include "utilities/definitions/manip_defs.hpp"
#include "planning/modules/grasp.hpp"

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
    }

    namespace packages
    {
        namespace manipulation
        {

        	class movable_body_plant_t;

            /**
             * @anchor manipulation_query_t
             *
             * This class represents a problem that a manipulation task planner will have to solve.
             * It is a self-contained class which holds the manipulator that we want to use, the index of the 
             * end effector that we want to use, the initial state of the manipulator and the start/goal states 
             * for the object that we want to manipulate. This query is able to request a full pick and place action
             * or just a transit plan to the initial state of the object or a transfer path between the two positions.
             * 
             * It will return if found a path the plan that will solve the problem that the user has requested. Also, will
             * return the final state of the manipulator. 
             * 
             * @brief <b> General query for manipulation task planners. </b>
             *
             * @author Andrew Dobson, Andrew Kimmel, Rahul Shome
             */
            class manipulation_query_t : public plan::motion_query_t
            {

              public:

                manipulation_query_t();

                manipulation_query_t(std::string manipulation_context_name, task_type_t mode, bool input_retraction_flag, grasp_evaluation_type_t path_quality, movable_body_plant_t* object_id, int open_end_effector_mode, util::config_t& retract_config, sim::state_t* manipulator_initial_state, sim::state_t* manipulator_target_state, sim::state_t* object_initial_state, sim::state_t* object_target_state, grasp_t* suggested_grasp = NULL, bool set_constraints = false);

                virtual ~manipulation_query_t();

                void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                virtual void link_spaces(const util::space_t* state_space, const util::space_t* control_space);
                
                virtual void setup_pick( std::string input_manipulation_context_name, bool input_retraction_flag, grasp_evaluation_type_t input_evaluation_mode, movable_body_plant_t* input_object, int open_end_effector_mode, util::config_t& input_retract_config, sim::state_t* input_manipulator_initial_state, sim::state_t* input_object_initial_state, grasp_t* input_suggested_grasp = NULL, bool set_constraints = false);
               
                //TODO: We should be smarter about place and handling suggested grasps...
                virtual void setup_place( std::string input_manipulation_context_name, bool input_retraction_flag, movable_body_plant_t* input_object, int open_end_effector_mode, util::config_t& input_retract_config, sim::state_t* input_manipulator_initial_state, sim::state_t* input_object_target_state, grasp_t* input_suggested_grasp, bool set_constraints = false );

                virtual void setup_move(std::string manipulation_context_name, sim::state_t* input_manipulator_initial_state, sim::state_t* input_manipulator_target_state, bool set_constraints = false);

                virtual void setup_move_to_config( std::string manipulation_context_name, sim::state_t* input_manipulator_initial_state, const util::config_t& input_manipulator_target_pose, bool set_constraints = false);

                virtual void setup_pick_and_place( std::string input_manipulation_context_name,  bool input_retraction_flag, grasp_evaluation_type_t input_evaluation_mode, movable_body_plant_t* input_object, int open_end_effector_mode, util::config_t& input_retract_config, sim::state_t* input_manipulator_initial_state, sim::state_t* input_object_initial_state, sim::state_t* input_object_target_state, grasp_t* input_suggested_grasp = NULL, bool set_constraints = false);

                virtual void setup_pick_and_move( std::string input_manipulation_context_name, grasp_evaluation_type_t input_evaluation_mode, movable_body_plant_t* input_object, int open_end_effector_mode, util::config_t& input_retract_config, sim::state_t* input_manipulator_initial_state, sim::state_t* input_manipulator_target_state, sim::state_t* input_object_initial_state, grasp_t* input_suggested_grasp = NULL, bool set_constraints = false);

                virtual void setup_pick_via_config_and_move( std::string input_manipulation_context_name, grasp_evaluation_type_t input_evaluation_mode, movable_body_plant_t* input_object, int open_end_effector_mode, util::config_t& input_retract_config, sim::state_t* input_manipulator_initial_state, sim::state_t* input_manipulator_target_state, sim::state_t* input_object_initial_state, grasp_t* input_suggested_grasp = NULL, bool set_constraints = false);

                virtual void set_search_mode(plan::search_mode_t astar_mode);
                
                virtual void set_valid_constraints( const util::constraints_t* valid_constraints );

                /** 
                 * @copydoc motion_planning_query_t::clear() 
                 * 
                 * Its also clears the plans that the query has stored.
                 */
                virtual void clear();

                /** @brief The planning context that we are going to plan in. */
                std::string manipulation_context_name;

                /** 
                 * @brief Declares the mode for the manipulation planning. 
                 *
                 *  Requests:
                 *   - Pick: From a open grasp state going to grasp a specific object at a pose.
                 *   - Place: From a grasping state we are going to place and retract the manipulator to a target position.
                 *   - Move: Find a path to move the manipulator for an initial state to a target state. The end effector can be either open or closed. 
                 *   - Transfer: Find a path to move an object from one place to another. Requires a valid grasp to have been performed. 
                 *   - Pick & Place: Find a path from an open grasp state to go pick up a specific object, move it o a location and return the hand to its initial state. 
                 */
                task_type_t task_mode;
                bool manipulator_retracts;
                
                /**
                 * - Feasible: The grasping planner will return the first path that it will work 
                 *             and it will pick,place or pick and place and object to specific states.
                 *             
                 * - Optimal: The grasping planner will use all the possible combinations to grasp
                 *            the object at the specific state and it will return the shortest or 
                 *            the one with the least amount of constraints, given the astar mode. 
                 *                             
                 */
                grasp_evaluation_type_t grasp_evaluation;

                /** @brief the initial state of the manipulator in the planning context.*/
                sim::state_t* manipulator_initial_state;
                /** @brief the target state of the manipulator in the planning context.*/
                sim::state_t* manipulator_target_state;
                
                util::config_t target_pose;
                
                /** @brief The initial state for the object that we need to grasp.*/
                sim::state_t* object_initial_state;
                /** @brief The target state for the object to be placed*/
                sim::state_t* object_target_state;

                /** @brief The object that the manipulator will control. .*/
                movable_body_plant_t* object;

                /** @brief The requested relative configuration for determining how to approach the object.*/
                util::config_t retraction_config;

                /** @brief The suggested relative grasp that we want the grasp planner to test. */
                grasp_t* suggested_grasp;

                /**
                 * @brief The default open mode that we want to test for our end effector. 
                 * The manipulator will start from this mode before calling the grasping planner.
                 */
                int default_open_mode;

                /**
                 * @brief Whether queries to the motion planner should be preceded by a call to IK steering.
                 * If IK steering returns a path, no roadmap search is needed. In general, this should be
                 * used for what the user knows to be short paths in workspace for the end effector, as IK
                 * steering can be computationally heavy.
                 */
                bool ik_steer_paths;

                /**
                 * @brief Declares the search mode for the heuristic search.
                 * 
                 * Requests:
                 *   - Standard: This is the standard A* search where we trust the edges on the roadmap.
                 *   - Lazy: If we do not trust the edges, run a lazy version of A*.
                 *   - Trusted MCR: If we are doing MCR problems, and trust the roadmap, run in this mode.
                 *   - Untrusted MCR: Otherwise, untrusted MCR problems are solved with this mode.
                 */
                plan::search_mode_t astar_mode;

                /** @brief The constraints that the minimum conflict astar has to take into accound. */
                const util::constraints_t* valid_constraints;

                /** @brief Indicates if we should update constraints on the planning structure. */
                bool update_constraints;

                bool smooth_paths;


                //------------------------//
                //--  Return Variables  --//
                //------------------------//
                /** @brief If the manipulation task planner found a path to our problem. */
                bool found_path;
                /** @brief Stores final state of the manipulator. Where the manipulator end up being after completing the task.*/
                sim::state_t* manipulator_final_state;
                /** @brief The relative grasp that worked for the specific task. */
                grasp_t* relative_grasp;
                /** @brief The constraints that the heuristic search will return. */
                util::constraints_t* path_constraints;

            protected:
                virtual void setup(std::string manipulation_context_name, task_type_t mode, bool input_retraction_flag, grasp_evaluation_type_t path_quality, movable_body_plant_t* object_id, int open_end_effector_mode, util::config_t& retract_config, sim::state_t* manipulator_initial_state, sim::state_t* manipulator_target_state, sim::state_t* object_initial_state, sim::state_t* object_target_state, grasp_t* suggested_grasp = NULL, bool set_constraints = false);

            };
        }
    }
}

#endif

