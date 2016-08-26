
/**
 * @file floating_hand_mappings.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */


#include "utilities/mapping_functions/floating_hand_mappings.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::floating_hand_full_state_mapping_t, prx::util::mapping_function_t)
PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::floating_hand_full_control_mapping_t, prx::util::mapping_function_t)
PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::floating_hand_state_mapping_t, prx::util::mapping_function_t)
PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::floating_hand_control_mapping_t, prx::util::mapping_function_t)
PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::floating_hand_gripper_state_mapping_t, prx::util::mapping_function_t)
PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::floating_hand_gripper_control_mapping_t, prx::util::mapping_function_t)