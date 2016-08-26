/**
 * @file grasp_evaluator.hpp
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

#ifndef PRX_GRASP_EVALUATOR_HPP
#define	PRX_GRASP_EVALUATOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/heuristic_search/constraints.hpp"

#include "prx/simulation/plan.hpp"
#include "prx/simulation/trajectory.hpp"

#include "prx/planning/planner.hpp"
#include "planning/specifications/grasping_specification.hpp"
#include "prx/planning/queries/query.hpp"

#include "planning/manipulation_world_model.hpp"
#include "planning/modules/grasp.hpp"
#include "planning/queries/grasping_query.hpp"
#include "prx/planning/task_planners/task_planner.hpp"

#include "planning/modules/planner_info.hpp"


namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
            /**
             * Grasp evaluation module to expose a common interface for suggesting, approaching and evaluating grasps.
             *
             * @authors Rahul Shome, Andrew Kimmel
             */
            class grasp_evaluator_t
            {

              public:

                grasp_evaluator_t();
                virtual ~grasp_evaluator_t();

                static pluginlib::ClassLoader<grasp_evaluator_t>& get_loader();

                /**
                 * @copydoc planner_t::init()
                 */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                /**
                 * @copydoc planner_t::reset()
                 */
                virtual void reset();

                /**
                 * @copydoc planner_t::link_world_model()
                 */
                virtual void link_world_model(plan::world_model_t * const model);

                /**
                 * @brief Function to link a vector of grasps to the evaluator
                 * @param input_grasps: vector of grasp_t from the grasping databases
                 */
                virtual void link_grasping_database(std::vector<grasp_t>&  input_grasps);

                /**
                 * @brief Query the metric for states closest to the object pose
                 * @param closest_states The output vector of states from the metric that have been reasoned to be a valid neighborhood of arm states around the object pose
                 * @param ee_metric The metric of arm states which have an associated end effector distance function
                 * @param input_object_pose The SE3 state of the object to grasp 
                 */
                virtual void states_closest_to_grasp(std::vector<sim::state_t* >& closest_states, util::distance_metric_t* ee_metric, const sim::state_t* input_object_pose, int num_closest_points = 10);


                /**
                 * @brief Evaluate the state of the arm and the object pose to check for a valid and stable grasp
                 * @param input_arm_state The input state of the arm
                 * @param input_object_pose The SE3 state of the object to grasp 
                 */
                virtual double grasp_evaluation(const sim::state_t* input_arm_state, const sim::state_t* input_object_pose);

                /**
                 * @brief Approach the object to a valid suggested grasp from the input arm state
                 * @param output_plan The output plan of approaching the object from the input arm state
                 * @param output_state The output state that corresponds to the end of the output plan, which should grasp the object, if the steering worked
                 * @param arm_state The state of the arm from where to try the approach
                 * @param input_object_pose The SE3 state of the object to grasp 
                 */
                virtual bool grasp_extension(sim::plan_t& output_plan, sim::state_t* output_state, const sim::state_t* arm_state, const sim::state_t* input_object_pose);
                /**
                 * @brief It will return the configuration of the last link along the active chain per the manipulation context.
                 * @details It will return the configuration of the last link along the active chain per the manipulation context.
                 * 
                 * @param link The result configuration for the last link along the chain. 
                 */
                virtual void FK(sim::state_t* arm_state, util::config_t& link, bool restore_original_state = false);

                /**
                 * @brief This function calls IK for the specific manipulation context.
                 * 
                 * @details This function calls IK for the specific manipulation context.
                 * 
                 * @param result_state The resulting state of the IK solution. 
                 * @param start_state The initial state of the manipulator. From this state we can retrieve the starting configuration of the link 
                 * that we are interesting in solving IK for and the mode of the manipulator if it is grasping or not.  
                 * @param goal_config The goal configuration that we need to achieve.
                 * @param validate Only return successful IK if the state is collision-free. 
                 * 
                 * @return True if the IK_solver found a solution, otherwise false.
                 */
                //virtual bool IK( sim::state_t* result_state, const util::space_point_t* start_state, const util::config_t& goal_config, bool validate = false);

                
              protected:
                
                /**
                 * @brief Convert the input state to an output config, if the state is an SE3 point
                 * @param state The input state of the object
                 * @param config The SE3 config of the object 
                 */
                virtual void state_to_config(util::config_t& config, const sim::state_t* state);
                // Manipulation world model
                manipulation_world_model_t* manipulation_model;
                // The threshold measure for deciding which grasping states are valid and stable grasps
                double grasp_quality_threshold;                
                // The vector of grasp_t linked into the evaluator
                std::vector<grasp_t>  grasps;                
                /** @brief The pluginlib loader which is returned by the get_loader() class. */
                static pluginlib::ClassLoader<grasp_evaluator_t> loader;

            };
        }
    }
}


#endif
