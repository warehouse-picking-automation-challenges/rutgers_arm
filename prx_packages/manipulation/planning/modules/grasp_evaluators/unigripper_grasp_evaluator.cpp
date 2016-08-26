/**
 * @file unigripper_grasp_evaluator.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2016, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "planning/modules/grasp_evaluators/unigripper_grasp_evaluator.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "utilities/distance_metrics/task_space_node.hpp"

#include "planning/manipulation_world_model.hpp"

#include <boost/range/adaptor/map.hpp>
#include <boost/filesystem.hpp>
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"

#include "prx/utilities/definitions/random.hpp"
#include <pluginlib/class_list_macros.h>


#include "utilities/distance_metrics/motion_planning_task_space_node.hpp"
#include "utilities/distance_metrics/task_space_metric.hpp"


PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::unigripper_grasp_evaluator_t, prx::packages::manipulation::grasp_evaluator_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace manipulation
        {
        
            unigripper_grasp_evaluator_t::unigripper_grasp_evaluator_t()
            {
            }

            unigripper_grasp_evaluator_t::~unigripper_grasp_evaluator_t()
            {
            }

            void unigripper_grasp_evaluator_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                default_ee_normal.resize(3);
                default_ee_normal = parameters::get_attribute_as<vector_t>("default_ee_normal", reader, template_reader);

                if(parameters::has_attribute("restricted_normals", reader, NULL))
                {
                    parameter_reader_t::reader_map_t planner_map = reader->get_map("restricted_normals");

                    foreach(const parameter_reader_t::reader_map_t::value_type key_value, planner_map)
                    {
                        vector_t new_normal =  parameters::get_attribute_as<vector_t>("restricted_normals/"+key_value.first,reader,NULL);
                        restricted_normals.push_back(new_normal);
                    }
                }
            }

            void unigripper_grasp_evaluator_t::reset()
            {

            }

            void unigripper_grasp_evaluator_t::states_closest_to_grasp(std::vector<sim::state_t* >& closest_states, util::distance_metric_t* ee_metric, const sim::state_t* input_object_pose, int num_closest_points)
            {
                // TODO

                // task_space_metric_t* ts_metric = dynamic_cast< task_space_metric_t* > (ee_metric);
                // closest_states.clear();

                // if(ts_metric==NULL)
                // {
                //     PRX_WARN_S("The evaluator needs to reason over the task space and get linked a task space metric.");
                //     return;
                // }

                // ts_metric->use_workspace_metric = true;
                
                // foreach(const abstract_node_t* node, ee_metric->multi_query(input_object_pose, num_closest_points))
                // {
                //     const task_space_node_t* ts_node = dynamic_cast<const task_space_node_t*>(node);
                //     closest_states.push_back(ts_node->other_metric_node->point);
                // }

                // ts_metric->use_workspace_metric = false;
                
            }

            double unigripper_grasp_evaluator_t::grasp_evaluation(const sim::state_t* input_arm_state, const movable_body_plant_t* target_object)
            {
                //Default behavior



                return 1.0;
            }

            bool unigripper_grasp_evaluator_t::grasp_extension(sim::plan_t& output_plan, sim::state_t* output_state, const sim::state_t* arm_state, const sim::state_t* input_object_pose)
            {

                // TODO

                // //Select grasp from the database
                // int grasp_number = uniform_int_random(0,grasps.size()-1);
                // grasp_t grasp = grasps[grasp_number];



                // //Find grasping config
                // config_t ee_local_config, grasping_config, object_config;

                // manipulation_model->get_end_effector_local_config(ee_local_config);
                // grasping_config = ee_local_config;
                // grasping_config.relative_to_global( grasp.relative_config );
                // state_to_config(object_config, input_object_pose);
                // grasping_config.relative_to_global( object_config );
                // output_plan.clear();
                // //JK steering to grasping config
                // workspace_trajectory_t ee_trajectory;
                // bool reached_grasp = manipulation_model->jac_steering(output_plan, ee_trajectory, output_state, arm_state, grasping_config);

                // if (!reached_grasp)
                // {
                //     //If the approach was incomplete
                //     PRX_PRINT("JK Steering to grasp <"<<grasp_number<<"> failed", PRX_TEXT_GREEN);
                //     return false;
                // }
                // else
                // {
                //     //If the evaluation returns a quality measure exceeding the threshold
                //     PRX_PRINT("JK Steering to grasp <"<<grasp_number<<"> succeeded", PRX_TEXT_GREEN);
                //     return (grasp_evaluation(output_state, input_object_pose) > grasp_quality_threshold);
                // }

            }


        }
    }
}
