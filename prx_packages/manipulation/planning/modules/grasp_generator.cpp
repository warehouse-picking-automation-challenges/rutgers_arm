/**
 * @file grasp_generator.cpp
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

#include "planning/modules/grasp_generator.hpp"

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


namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace manipulation
        {
            pluginlib::ClassLoader<grasp_generator_t> grasp_generator_t::loader("prx_planning", "prx::packages::manipulation::grasp_generator_t");

            pluginlib::ClassLoader<grasp_generator_t>& grasp_generator_t::get_loader()
            {
                return loader;
            }
                        
            grasp_generator_t::grasp_generator_t()
            {
                manipulation_model = NULL;
            }

            grasp_generator_t::~grasp_generator_t()
            {
            }

            void grasp_generator_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                PRX_DEBUG_COLOR("Grasp generator init", PRX_TEXT_LIGHTGRAY);

                ffee_context_name = parameters::get_attribute("ffee_context_name", reader, template_reader, "");
                save_grasps_to_file = parameters::get_attribute_as<bool>("save_grasps_to_file", reader, template_reader, false);
                end_effector_name = parameters::get_attribute_as<std::string>("end_effector_name", reader, template_reader, "");
            }

            void grasp_generator_t::link_world_model(world_model_t * const model)
            {
                PRX_DEBUG_COLOR("Grasp generator link world model", PRX_TEXT_LIGHTGRAY);
                manipulation_model = dynamic_cast<manipulation_world_model_t*>(model);
                PRX_ASSERT(manipulation_model != NULL);
            }

            void grasp_generator_t::state_to_config(config_t& config, const state_t* state)
            {
                PRX_ASSERT(state->memory.size() == 7);
                config.set_position(state->memory[0], state->memory[1], state->memory[2]);
                config.set_orientation(state->memory[3], state->memory[4], state->memory[5], state->memory[6]);
            }

            void grasp_generator_t::config_to_state(const config_t& config, state_t* st)
            {
                PRX_ASSERT(st->memory.size() == 7);
                vector_t pos, euler;
                pos = config.get_position();
                st->memory[0] = pos[0];
                st->memory[1] = pos[1];
                st->memory[2] = pos[2];

                quaternion_t orientation;
                orientation = config.get_orientation();
                st->memory[3] = orientation[0];
                st->memory[4] = orientation[1];
                st->memory[5] = orientation[2];
                st->memory[6] = orientation[3];
            }

            void grasp_generator_t::config_to_euler_state(const config_t& config, sim::state_t* st)
            {
                vector_t pos, euler;
                pos = config.get_position();
                st->memory[0] = pos[0];
                st->memory[1] = pos[1];
                st->memory[2] = pos[2];

                quaternion_t orientation;
                orientation = config.get_orientation();
                orientation.convert_to_euler(euler);
                st->memory[3] = euler[0];
                st->memory[4] = euler[1];
                st->memory[5] = euler[2];

                PRX_DEBUG_COLOR("Position: " << pos, PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR("Orientation: " << orientation, PRX_TEXT_LIGHTGRAY);
            }


            bool grasp_generator_t::get_grasps(std::vector<grasp_t>& returned_grasps, const std::string& object_type)
            {
                if (grasps.find(object_type) != grasps.end())
                {
                    returned_grasps = grasps[object_type];
                    return true;
                }
                else
                {
                    return false;
                }
            }

            void grasp_generator_t::append_to_stat_string(const std::string& input)
            {
                statistics += input;
            }

            std::string& grasp_generator_t::get_stat_string()
            {
                return statistics;
            }

            // void grasp_generator_t::states_closest_to_grasp(std::vector<sim::state_t* >& closest_states, util::distance_metric_t* ee_metric, const sim::state_t* input_object_pose, int num_closest_points)
            // {
            //     task_space_metric_t* ts_metric = dynamic_cast< task_space_metric_t* > (ee_metric);
            //     closest_states.clear();

            //     if(ts_metric==NULL)
            //     {
            //         PRX_WARN_S("The evaluator needs to reason over the task space and get linked a task space metric.");
            //         return;
            //     }

            //     ts_metric->use_workspace_metric = true;
                
            //     foreach(const abstract_node_t* node, ee_metric->multi_query(input_object_pose, num_closest_points))
            //     {
            //         const task_space_node_t* ts_node = dynamic_cast<const task_space_node_t*>(node);
            //         closest_states.push_back(ts_node->other_metric_node->point);
            //     }

            //     ts_metric->use_workspace_metric = false;
                
            // }

            // double grasp_generator_t::grasp_evaluation(const sim::state_t* input_arm_state, const sim::state_t* input_object_pose)
            // {
            //     //Default behavior
            //     return 1.0;
            // }

            // bool grasp_generator_t::grasp_extension(sim::plan_t& output_plan, sim::state_t* output_state, const sim::state_t* arm_state, const sim::state_t* input_object_pose)
            // {
            //     //Select grasp from the database
            //     int grasp_number = uniform_int_random(0,grasps.size()-1);
            //     grasp_t grasp = grasps[grasp_number];



            //     //Find grasping config
            //     config_t ee_local_config, grasping_config, object_config;

            //     manipulation_model->get_end_effector_local_config(ee_local_config);
            //     grasping_config = ee_local_config;
            //     grasping_config.relative_to_global( grasp.relative_config );
            //     state_to_config(object_config, input_object_pose);
            //     grasping_config.relative_to_global( object_config );
            //     output_plan.clear();
            //     //JK steering to grasping config
            //     workspace_trajectory_t ee_trajectory;
            //     bool reached_grasp = manipulation_model->jac_steering(output_plan, ee_trajectory, output_state, arm_state, grasping_config);

            //     if (!reached_grasp)
            //     {
            //         //If the approach was incomplete
            //         PRX_PRINT("JK Steering to grasp <"<<grasp_number<<"> failed", PRX_TEXT_GREEN);
            //         return false;
            //     }
            //     else
            //     {
            //         //If the evaluation returns a quality measure exceeding the threshold
            //         PRX_PRINT("JK Steering to grasp <"<<grasp_number<<"> succeeded", PRX_TEXT_GREEN);
            //         return (grasp_evaluation(output_state, input_object_pose) > grasp_quality_threshold);
            //     }

            // }

            // void grasp_generator_t::FK(state_t* arm_state, util::config_t& link, bool restore_original_state)
            // {

            //     state_t* original_state ;
            //     if (restore_original_state)
            //     {
            //         original_state = manipulation_model->get_state_space()->alloc_point();
            //     }
                
            //     manipulation_model->get_current_manipulation_info()->arm_state_space->copy_from_point(arm_state);
            //     manipulation_model->FK(link);

            //     if (restore_original_state)
            //     {
            //         manipulation_model->get_state_space()->copy_from_point(original_state);
            //         manipulation_model->get_state_space()->free_point(original_state);
            //     }
            // }

        }
    }
}
