/**
 * @file manipulation_query.cpp
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

#include "planning/queries/manipulation_query.hpp"
#include "simulation/systems/plants/movable_body_plant.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/goals/goal.hpp"


#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::manipulation_query_t, prx::plan::query_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace manipulation
        {

            manipulation_query_t::manipulation_query_t()
            {
                //cannot call clear here because of the motion_planning_query_t::clear() that first needs the spaces.
                manipulation_context_name = "";
                manipulator_initial_state = NULL;
                manipulator_target_state = NULL;
                object_initial_state = NULL;
                object_target_state = NULL;
                manipulator_final_state = NULL;
                object = NULL;                                
                default_open_mode = -1;
                found_path = false;
                ik_steer_paths = false;
                retraction_config.zero();
                suggested_grasp = NULL;
                task_mode = TASK_MOVE;
                manipulator_retracts = false;
                grasp_evaluation = GRASP_GREEDY;                
                astar_mode = LAZY_SEARCH;
                relative_grasp = NULL;
                path_constraints = NULL;
                valid_constraints = NULL;
                update_constraints = false;
                smooth_paths = false;
            }

            manipulation_query_t::manipulation_query_t(std::string input_manipulation_context_name, task_type_t input_task_mode, bool input_retraction_flag, grasp_evaluation_type_t input_evaluation_mode, movable_body_plant_t* input_object, int open_end_effector_mode, util::config_t& input_retract_config, state_t* input_manipulator_initial_state, state_t* input_manipulator_target_state, state_t* input_object_initial_state, state_t* input_object_target_state, grasp_t* input_suggested_grasp, bool set_constraints)
            {
            	setup(input_manipulation_context_name, input_task_mode, input_retraction_flag, input_evaluation_mode, input_object, open_end_effector_mode, input_retract_config, input_manipulator_initial_state, input_manipulator_target_state, input_object_initial_state, input_object_target_state, input_suggested_grasp, set_constraints);
            }

            manipulation_query_t::~manipulation_query_t()
            {
                state_space->free_point( manipulator_final_state );
                clear();
            }
            
            void manipulation_query_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                smooth_paths = parameters::get_attribute_as<bool>("smooth_paths", reader, template_reader, false);
                
                if( parameters::has_attribute("search_mode", reader, template_reader) )
                {
                    std::string mode_str = parameters::get_attribute_as<std::string > ("search_mode", reader, template_reader);
                    if(mode_str == "standard_search")
                    {
                        PRX_PRINT("Search mode set to STANDARD_SEARCH", PRX_TEXT_BROWN);
                        astar_mode = STANDARD_SEARCH;
                    }
                    else if(mode_str == "lazy_search")
                    {
                        PRX_PRINT("Search mode set to LAZY_SEARCH", PRX_TEXT_BROWN);
                        astar_mode = LAZY_SEARCH;
                    }
                    else if(mode_str == "trusted_mcr")
                    {
                        PRX_PRINT("Search mode set to TRUSTED_MCR", PRX_TEXT_BROWN);
                        astar_mode = TRUSTED_MCR;
                    }
                    else if(mode_str == "untrusted_mcr")
                    {
                        PRX_PRINT("Search mode set to UNTRUSTED_MCR", PRX_TEXT_BROWN);
                        astar_mode = UNTRUSTED_MCR;
                    }
                    else
                    {
                        PRX_WARN_S("Invalid search mode set through input for motion planning query.");
                        astar_mode = LAZY_SEARCH;
                    }
                }
                else
                {
                    PRX_WARN_S("Missing search mode in input for motion planning query.");
                }

            }

            void manipulation_query_t::link_spaces(const util::space_t* state_space, const util::space_t* control_space)
            {
                motion_query_t::link_spaces( state_space, control_space );
                manipulator_final_state = state_space->alloc_point();
            }

            void manipulation_query_t::setup_pick( std::string input_manipulation_context_name,  bool input_retraction_flag, grasp_evaluation_type_t input_evaluation_mode, movable_body_plant_t* input_object, int open_end_effector_mode, config_t& input_retract_config, state_t* input_manipulator_initial_state, state_t* input_object_initial_state, grasp_t* input_suggested_grasp, bool set_constraints)
            {
                setup(input_manipulation_context_name, TASK_PICK, input_retraction_flag, input_evaluation_mode, input_object, open_end_effector_mode, input_retract_config,
                        input_manipulator_initial_state, NULL, input_object_initial_state, NULL, input_suggested_grasp, set_constraints);
            }

            void manipulation_query_t::setup_place( std::string input_manipulation_context_name,  bool input_retraction_flag, movable_body_plant_t* input_object, int open_end_effector_mode, config_t& input_retract_config, state_t* input_manipulator_initial_state, state_t* input_object_target_state, grasp_t* input_suggested_grasp, bool set_constraints )
            {
                setup(input_manipulation_context_name, TASK_PLACE, input_retraction_flag, GRASP_SUGGESTED, input_object, open_end_effector_mode, input_retract_config,
                        input_manipulator_initial_state, NULL, NULL, input_object_target_state, input_suggested_grasp, set_constraints);
            }

            void manipulation_query_t::setup_move(std::string input_manipulation_context_name, state_t* input_manipulator_initial_state, state_t* input_manipulator_target_state, bool set_constraints)
            {
                config_t dummy_config;
                setup(input_manipulation_context_name, TASK_MOVE, false, GRASP_GREEDY, NULL, -1, dummy_config,
                        input_manipulator_initial_state, input_manipulator_target_state, NULL, NULL, NULL, set_constraints);
            }

            void manipulation_query_t::setup_move_to_config( std::string input_manipulation_context_name, sim::state_t* input_manipulator_initial_state, const config_t& input_manipulator_target_pose, bool set_constraints)
            {
                config_t dummy_config;
                target_pose = input_manipulator_target_pose;
                setup( input_manipulation_context_name, TASK_MOVE_TO_CONFIG, false, GRASP_GREEDY, NULL, -1, dummy_config,
                        input_manipulator_initial_state, NULL, NULL, NULL, NULL, set_constraints );
            }

            void manipulation_query_t::setup_pick_and_place( std::string input_manipulation_context_name, bool input_retraction_flag, grasp_evaluation_type_t input_evaluation_mode, movable_body_plant_t* input_object, int open_end_effector_mode, config_t& input_retract_config, state_t* input_manipulator_initial_state, state_t* input_object_initial_state, state_t* input_object_target_state, grasp_t* input_suggested_grasp, bool set_constraints)
            {
                setup(input_manipulation_context_name, TASK_PICK_AND_PLACE, input_retraction_flag, input_evaluation_mode, input_object, open_end_effector_mode, input_retract_config,
                        input_manipulator_initial_state, NULL, input_object_initial_state, input_object_target_state, input_suggested_grasp, set_constraints);
            }

            void manipulation_query_t::setup_pick_and_move( std::string input_manipulation_context_name, grasp_evaluation_type_t input_evaluation_mode, movable_body_plant_t* input_object, int open_end_effector_mode, config_t& input_retract_config, state_t* input_manipulator_initial_state, state_t* input_manipulator_target_state, state_t* input_object_initial_state, grasp_t* input_suggested_grasp, bool set_constraints)
            {
                setup(input_manipulation_context_name, TASK_PICK_AND_MOVE, false, input_evaluation_mode, input_object, open_end_effector_mode, input_retract_config,
                        input_manipulator_initial_state, input_manipulator_target_state, input_object_initial_state, NULL, input_suggested_grasp, set_constraints);
            }

            void manipulation_query_t::setup_pick_via_config_and_move( std::string input_manipulation_context_name, grasp_evaluation_type_t input_evaluation_mode, movable_body_plant_t* input_object, int open_end_effector_mode, config_t& input_retract_config, state_t* input_manipulator_initial_state, state_t* input_manipulator_target_state, state_t* input_object_initial_state, grasp_t* input_suggested_grasp, bool set_constraints)
            {
                setup(input_manipulation_context_name, TASK_PICK_VIA_CONFIG_AND_MOVE, false, input_evaluation_mode, input_object, open_end_effector_mode, input_retract_config,
                        input_manipulator_initial_state, input_manipulator_target_state, input_object_initial_state, NULL, input_suggested_grasp, set_constraints);
            }

            void manipulation_query_t::setup(std::string input_manipulation_context_name, task_type_t input_task_mode, bool input_retraction_flag, grasp_evaluation_type_t input_evaluation_mode, movable_body_plant_t* input_object, int open_end_effector_mode, util::config_t& input_retract_config, state_t* input_manipulator_initial_state, state_t* input_manipulator_target_state, state_t* input_object_initial_state, state_t* input_object_target_state, grasp_t* input_suggested_grasp, bool set_constraints)
            {
                manipulation_context_name = input_manipulation_context_name;
                task_mode = input_task_mode;
                object = input_object;
                grasp_evaluation = input_evaluation_mode;
                suggested_grasp = input_suggested_grasp;
                default_open_mode = open_end_effector_mode;
                manipulator_initial_state = input_manipulator_initial_state;
                manipulator_target_state = input_manipulator_target_state;
                object_initial_state = input_object_initial_state;
                object_target_state = input_object_target_state;
                retraction_config = input_retract_config;
                update_constraints = set_constraints;

                manipulator_retracts = input_retraction_flag;

                relative_grasp = NULL;
                found_path = false;
                ik_steer_paths = false;
                
                // If we are in a suggested grasp mode and have not been provided a grasp
                if(grasp_evaluation == GRASP_SUGGESTED && suggested_grasp == NULL)
                {
                    // If we are in a mode that cares about grasping
                    if( task_mode != TASK_MOVE && task_mode != TASK_PLACE )
                    {
                        PRX_FATAL_S("Creating a suggested configuration query but the suggested relative grasp config is NULL for path mode: " << task_mode);
                    }
                }
            }


            void manipulation_query_t::set_search_mode(search_mode_t search_mode)
            {
                astar_mode = search_mode;
            }

            void manipulation_query_t::set_valid_constraints( const constraints_t* input_valid_constraints )
            {
                valid_constraints = input_valid_constraints;
            }

            void manipulation_query_t::clear()
            {
                manipulation_context_name = "";
                manipulator_initial_state = NULL;
                manipulator_target_state = NULL;
                object_initial_state = NULL;
                object_target_state = NULL;
                manipulator_final_state = NULL;
                object = NULL;                                
                default_open_mode = -1;
                found_path = false;
                ik_steer_paths = false;
                retraction_config.zero();
                suggested_grasp = NULL;
                relative_grasp = NULL;
                motion_query_t::clear();
            }
        }
    }
}
