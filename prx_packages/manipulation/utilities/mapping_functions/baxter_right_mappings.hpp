
/**
 * @file baxter_right_mappings.hpp
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

#pragma once

#ifndef PRX_BAXTER_RIGHT_MAPPINGS_HPP
#define PRX_BAXTER_RIGHT_MAPPINGS_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/mapping_functions/subset_mapping.hpp"

#include <boost/assign/list_of.hpp>

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {

        	//Right Arm + gripper
        	class baxter_right_full_arm_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            baxter_right_full_arm_state_mapping_t() 
		            {
		                domain = 16;
		                range = 8;
		                mapped_indices = {7,8,9,10,11,12,13,15};
		                output_space_name = "BaxterRightFullArm";
		                mapping_name = "baxter_right_full_arm_state_mapping";
		            }
		            virtual ~baxter_right_full_arm_state_mapping_t() {}
        	};

        	//Right Arm 
        	class baxter_right_arm_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            baxter_right_arm_state_mapping_t() 
		            {
		                domain = 16;
		                range = 7;
		                mapped_indices = {7,8,9,10,11,12,13};
		                output_space_name = "BaxterRightArm";
		                mapping_name = "baxter_right_arm_state_mapping";
		            }
		            virtual ~baxter_right_arm_state_mapping_t() {}
        	};

        	//Right Gripper
        	class baxter_right_gripper_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            baxter_right_gripper_state_mapping_t() 
		            {
		                domain = 16;
		                range = 1;
		                mapped_indices = {15};
		                output_space_name = "Hand";
		                mapping_name = "baxter_right_gripper_state_mapping";
		            }
		            virtual ~baxter_right_gripper_state_mapping_t() {}
        	};


        	///////////////////////////////////////////
        	// Same mappings for the control subsets //
        	///////////////////////////////////////////


        	class baxter_right_full_arm_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            baxter_right_full_arm_control_mapping_t() 
		            {
		                domain = 16;
		                range = 8;
		                mapped_indices = {7,8,9,10,11,12,13,15};
		                output_space_name = "BaxterRightFullArm";
		                mapping_name = "baxter_right_full_arm_control_mapping";
		            }
		            virtual ~baxter_right_full_arm_control_mapping_t() {}
        	};

        	class baxter_right_arm_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            baxter_right_arm_control_mapping_t() 
		            {
		                domain = 16;
		                range = 7;
		                mapped_indices = {7,8,9,10,11,12,13};
		                output_space_name = "BaxterRightArm";
		                mapping_name = "baxter_right_arm_control_mapping";
		            }
		            virtual ~baxter_right_arm_control_mapping_t() {}
        	};

        	class baxter_right_gripper_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            baxter_right_gripper_control_mapping_t() 
		            {
		                domain = 16;
		                range = 1;
		                mapped_indices = {15};
		                output_space_name = "Hand";
		                mapping_name = "baxter_right_gripper_control_mapping";
		            }
		            virtual ~baxter_right_gripper_control_mapping_t() {}
        	};
        }
    }
}

#endif