/**
 * @file grasping_query.hpp
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

#ifndef PRX_GRASPING_QUERY_HPP
#define PRX_GRASPING_QUERY_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/config.hpp"
 
#include "prx/simulation/state.hpp"
#include "prx/simulation/plan.hpp"
#include "prx/simulation/trajectory.hpp"
 
#include "prx/planning/queries/query.hpp"

//TODO: NOT DO THIS
#include "prx/planning/modules/heuristic_search/constraints_astar_search.hpp"

#include "utilities/definitions/manip_defs.hpp"

#include "planning/modules/grasp_data.hpp"

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
             * @anchor grasping_query_t
             *
             * This class is the query that the manipulation task planner will create for the grasping planner. 
             * It operates over a single object and the specific manipulator and end effector that the world model has active. 
             * It will return from the grasping planner all the information is needed for grasping the specific object at a given 
             * location. 
             * 
             * @brief <b> General query for the grasping planner. </b>
             *
             * @author Andrew Dobson, Andrew Kimmel, Rahul Shome
             */
            class grasping_query_t : public plan::query_t
            {
              public: 

                grasping_query_t();

                grasping_query_t(const util::space_t* input_state_space, const util::space_t* input_control_space, grasping_query_type_t input_grasping_mode, plan::search_mode_t input_search_mode, task_type_t input_task_mode, bool input_retraction_flag, movable_body_plant_t* input_object, std::vector<sim::state_t*>& object_initial_state, util::config_t& input_config, grasp_t* input_grasp, const util::constraints_t* input_active_constraints);
                
                virtual ~grasping_query_t();

                /**
                 * @brief Setup a new query for the grasping planner. 
                 * 
                 * @details Setup a new query for the grasping planner. This setup will delete the old plans 
                 * and paths and will initialize new ones with the give space. If the spaces are the same 
                 * then nothing will happen, but you could also had used the other setup function, provided by this class. 
                 * 
                 * @param state_space The state space for the trajectory.
                 * @param control_space The control space for the plan.
                 * @param object The object that we need to grasp.
                 * @param grasping_id The grasping id that we need to test.
                 * @param object_initial_state The initial state of the object.
                 * @param retract_config Relative retracted configuration for the end effector.
                 */
                virtual void setup(const util::space_t* input_state_space, const util::space_t* input_control_space, grasping_query_type_t input_grasping_mode, plan::search_mode_t input_search_mode, task_type_t input_task_mode, bool input_retraction_flag, movable_body_plant_t* input_object, std::vector<sim::state_t*>& object_initial_state, util::config_t& input_config, grasp_t* input_grasp, const util::constraints_t* input_active_constraints);

                /** 
                 * @copydoc motion_planning_query_t::clear() 
                 * 
                 * Its also clears the plans that the query has stored.
                 */
                virtual void clear();

                virtual void set_data(unsigned state_index, grasp_data_t* data);

                virtual void remove_last_data_from(unsigned state_index);

                virtual std::string print(unsigned prec = 8);

                /** 
                 * @brief Declares the mode for the grasping planner. 
                 *
                 */
                grasping_query_type_t grasping_mode;
                bool reuse_grasping_collision_checks;

                /**
                 * @brief The manipulation path mode which tells us what to fill in
                 */
                task_type_t task_mode;
                bool manipulator_retracts;

                /** @brief The object that the manipulator will control. */
                movable_body_plant_t* object;
                
                /**
                 *  @brief The different states that we want to check for grasping. 
                 *  If there are multiple states, only the common grasping indices will be tested.
                 */
                std::vector< sim::state_t* > object_states;

                /** @brief The requested relative configuration for determining how to approach the object.*/
                util::config_t retraction_config;
                
                /** @brief The A* mode for the search */
                plan::search_mode_t astar_mode;
                /** @brief The active constraints to check against */
                const util::constraints_t* active_constraints;

                /** 
                 * @brief The relative grasping configuration that we want to test if it will work. 
                 * Also this variable will be used as a returning value for the grasp that worked.
                 */
                grasp_t* suggested_grasp;

                //------------------------//
                //--  Return Variables  --//
                //------------------------//
                /** @brief If the manipulation task planner found a path to our problem. */
                bool found_grasp;

                /** For each object state it will return a set of vectors with grasped data for each relative configuration that worked.*/
                std::vector< std::vector< grasp_data_t* > > grasp_data;

                std::string reason_for_failure;

                const util::space_t* state_space;
                const util::space_t* control_space;
                
            };
        }
    }
}

#endif

