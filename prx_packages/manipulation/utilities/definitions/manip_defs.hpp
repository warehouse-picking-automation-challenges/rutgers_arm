/**
 * @file manip_defs.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2016, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_MANIPULATION_DEFS_HPP
#define PRX_MANIPULATION_DEFS_HPP


#include <yaml-cpp/yaml.h>
#include "prx/utilities/math/configurations/config.hpp"

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
            /**
             * - Greedy: The grasping planner will return the first path that it will work 
             *             for pick,place or pick and place the object at specific object states.
             *             
             * - Exhaustive: The grasping planner will use all the possible grasp from the data base to grasp
             *            the object at the specific state and it will return the shortest or 
             *            the one with the least amount of constraints, given the astar mode.
             *            
             * - Suggested: The grasping planner will use just the suggested relative configuration             
             *              in order to detect if it can be used for all the given states of the object.
             */
            enum grasp_evaluation_type_t
            {
                GRASP_GREEDY, GRASP_EXHAUSTIVE, GRASP_SUGGESTED
            };

            /**
             *  - Configurations: 
             *
             *  - States:
             *
             *  - Plans:
             */
            enum grasping_query_type_t
            {
                GRASPING_CONFIGURATIONS, GRASPING_PLANS
            };

            /**
             *   - Pick: From a open grasp state going to grasp a specific object at a pose.
             *   - Place: From a grasping state we are going to place and retract the manipulator to a target position.
             *   - Move: Find a path to move the manipulator for an initial state to a target state. The end effector can be either open or closed. 
             *   - Transfer: Find a path to move an object from one place to another. Requires a valid grasp to have been performed. 
             *   - Pick & Place: Find a path from an open grasp state to go pick up a specific object, move it to a location and return the hand to its initial state. 
             *   - Pick & Move: Find a path from an open grasp state to go pick up a specific object, retract the hand while holding the object
             *   - Move to Config: Similar to pick or place, but moves a specific robot link to a target configuration
             */
            enum task_type_t
            {
                TASK_MOVE, TASK_PICK, TASK_PLACE, TASK_PICK_AND_PLACE, TASK_PICK_AND_MOVE, TASK_MOVE_TO_CONFIG, TASK_PICK_VIA_CONFIG_AND_MOVE
            };
            

                
        }
    }
}

#endif //PRX_MANIPULATION_DEFS_HPP

