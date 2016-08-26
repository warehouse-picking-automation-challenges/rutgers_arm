/**
 * @file apc_task_query.cpp
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

#include "prx/utilities/parameters/parameter_reader.hpp"

#include "simulation/systems/plants/movable_body_plant.hpp"

#include "planning/apc_task_query.hpp"

// #include <pluginlib/class_list_macros.h> 
// #include <ros/ros.h>

// PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::apc_task_query_t, prx::plan::query_t)

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {
        using namespace manipulation;
        namespace apc
        {

            apc_task_query_t::apc_task_query_t()
            {
                clear();
                
            }

            apc_task_query_t::~apc_task_query_t()
            {
                clear();
            }

            apc_task_query_t::apc_task_query_t(space_t* control_space)
            {
                move_plan.link_control_space(control_space);
                first_mapping_plan.link_control_space(control_space);
                second_mapping_plan.link_control_space(control_space);
                third_mapping_plan.link_control_space(control_space);
                move_gripper_to_bin.link_control_space(control_space);
                approach_object.link_control_space(control_space);
                retrieve_object.link_control_space(control_space);
                move_to_order_bin.link_control_space(control_space);
                clear();
            }

            void apc_task_query_t::setup(std::string in_hand, movable_body_plant_t* requested_object, apc_stage in_stage, char which_bin, sim::state_t* goal)
            {
                hand = in_hand;
                object = requested_object;
                stage = in_stage;
                bin = which_bin;
                found_solution = false;
                goal_state = goal;            
            }

            void apc_task_query_t::setup(std::string in_hand, movable_body_plant_t* requested_object, apc_stage in_stage, char which_bin, sim::state_t* goal, std::vector<double> final_obj)
            {
                hand = in_hand;
                object = requested_object;
                stage = in_stage;
                bin = which_bin;
                found_solution = false;
                goal_state = goal;
                final_obj_state = final_obj;                
            }

            void apc_task_query_t::clear()
            {            	
                hand = "";
                object = NULL;
                goal_state = NULL;
                stage = MOVE;
                bin = ' ';
                first_mapping_plan.clear();
                second_mapping_plan.clear();
                third_mapping_plan.clear();
                move_gripper_to_bin.clear();
                approach_object.clear();
                retrieve_object.clear();
                move_to_order_bin.clear();
                found_solution = false;
            }
        }
    }
}
