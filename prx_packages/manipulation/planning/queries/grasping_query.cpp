/**
 * @file grasping_query.cpp
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

#include "planning/queries/grasping_query.hpp"
#include "simulation/systems/plants/movable_body_plant.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/goals/goal.hpp"


#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::grasping_query_t, prx::plan::query_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace manipulation
        {

            grasping_query_t::grasping_query_t()
            {
            	clear();
            }

            grasping_query_t::grasping_query_t(const util::space_t* input_state_space, const util::space_t* input_control_space, grasping_query_type_t input_grasping_mode, search_mode_t input_search_mode, task_type_t input_task_mode, bool input_retraction_flag, movable_body_plant_t* input_object, std::vector<sim::state_t*>& object_initial_state, util::config_t& input_config, grasp_t* input_grasp, const constraints_t* input_active_constraints)
            {
            	setup(input_state_space, input_control_space, input_grasping_mode, input_search_mode, input_task_mode, input_retraction_flag, input_object, object_initial_state, input_config, input_grasp, input_active_constraints);
            }

            grasping_query_t::~grasping_query_t()
            {
                clear();
            }

            void grasping_query_t::setup(const util::space_t* input_state_space, const util::space_t* input_control_space, grasping_query_type_t input_grasping_mode, search_mode_t input_search_mode, task_type_t input_task_mode, bool input_retraction_flag, movable_body_plant_t* input_object, std::vector<sim::state_t*>& object_initial_state, util::config_t& input_config, grasp_t* input_grasp, const constraints_t* input_active_constraints)
            {
                state_space = input_state_space;
                control_space = input_control_space;

                object = input_object;

                grasping_mode = input_grasping_mode;
                astar_mode = input_search_mode;
                task_mode = input_task_mode;

                active_constraints = input_active_constraints;

                object_states = object_initial_state;
                grasp_data.resize( object_states.size() );
                suggested_grasp = input_grasp;
                retraction_config = input_config;

                manipulator_retracts = input_retraction_flag;
                
                found_grasp = false;
                reason_for_failure = "";

            }

            void grasping_query_t::clear()
            {
                state_space = NULL;
                control_space = NULL;
                object = NULL;
                object_states.clear();
                for(unsigned i = 0; i < grasp_data.size(); ++i)
                {
                    for(unsigned j = 0; j< grasp_data[i].size(); ++j)
                    {
                        grasp_data[i][j]->clear();
                        delete grasp_data[i][j];
                    }
                    grasp_data[i].clear();
                }
                grasp_data.clear();
                suggested_grasp = NULL;
                found_grasp = false;
                reason_for_failure = "";
                retraction_config.zero();
            }

            void grasping_query_t::set_data(unsigned state_index, grasp_data_t* data)
            {
                grasp_data[state_index].push_back(data);
            }

            void grasping_query_t::remove_last_data_from(unsigned state_index)
            {
                grasp_data[state_index].back()->clear();
                delete grasp_data[state_index].back();
                grasp_data[state_index].pop_back();
            }

            std::string grasping_query_t::print(unsigned prec)
            {
                std::stringstream out(std::stringstream::out);

                return out.str();
            }
        }
    }
}
