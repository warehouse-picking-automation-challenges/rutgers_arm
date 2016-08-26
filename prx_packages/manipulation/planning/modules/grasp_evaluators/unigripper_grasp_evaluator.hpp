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

#ifndef PRX_UNIGRIPPER_GRASP_EVALUATOR_HPP
#define PRX_UNIGRIPPER_GRASP_EVALUATOR_HPP


#include "planning/modules/grasp_evaluator.hpp"


namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
            /**
             * Grasp evaluation for the unigripper (end effectors with a desired grasping surface)
             *
             * @authors Andrew Kimmel
             */
            class unigripper_grasp_evaluator_t : public grasp_evaluator_t
            {

              public:

                unigripper_grasp_evaluator_t();
                virtual ~unigripper_grasp_evaluator_t();

                /**
                 * @copydoc planner_t::init()
                 */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                /**
                 * @copydoc planner_t::reset()
                 */
                virtual void reset();

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
                virtual double grasp_evaluation(const sim::state_t* input_arm_state, const movable_body_plant_t* target_object);

                /**
                 * @brief Approach the object to a valid suggested grasp from the input arm state
                 * @param output_plan The output plan of approaching the object from the input arm state
                 * @param output_state The output state that corresponds to the end of the output plan, which should grasp the object, if the steering worked
                 * @param arm_state The state of the arm from where to try the approach
                 * @param input_object_pose The SE3 state of the object to grasp 
                 */
                virtual bool grasp_extension(sim::plan_t& output_plan, sim::state_t* output_state, const sim::state_t* arm_state, const sim::state_t* input_object_pose);

    
              protected:

                std::vector<util::vector_t> restricted_normals;
                util::vector_t default_ee_normal;            

            };
        }
    }
}


#endif
