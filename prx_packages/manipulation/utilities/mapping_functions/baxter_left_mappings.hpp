
/**
 * @file baxter_left_mappings.hpp
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

#ifndef PRX_BAXTER_LEFT_MAPPINGS_HPP
#define PRX_BAXTER_LEFT_MAPPINGS_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/mapping_functions/subset_mapping.hpp"

#include <boost/assign/list_of.hpp>

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {

        	//Left Arm + gripper
        	class baxter_left_full_arm_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            baxter_left_full_arm_state_mapping_t() 
		            {
		                domain = 16;
		                range = 8;
		                mapped_indices = {0,1,2,3,4,5,6,14};
		                output_space_name = "BaxterLeftFullArm";
		                mapping_name = "baxter_left_full_arm_state_mapping";
		            }
		            virtual ~baxter_left_full_arm_state_mapping_t() {}
        	};

        	//Left Arm 
        	class baxter_left_arm_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            baxter_left_arm_state_mapping_t() 
		            {
		                domain = 16;
		                range = 7;
		                mapped_indices = {0,1,2,3,4,5,6};
		                output_space_name = "BaxterLeftArm";
		                mapping_name = "baxter_left_arm_state_mapping";
		            }
		            virtual ~baxter_left_arm_state_mapping_t() {}
        	};

        	//Left Gripper
        	class baxter_left_gripper_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            baxter_left_gripper_state_mapping_t() 
		            {
		                domain = 16;
		                range = 1;
		                mapped_indices = {14};
		                output_space_name = "Hand";
		                mapping_name = "baxter_left_gripper_state_mapping";
		            }
		            virtual ~baxter_left_gripper_state_mapping_t() {}
        	};


        	///////////////////////////////////////////
        	// Same mappings for the control subsets //
        	///////////////////////////////////////////


        	class baxter_left_full_arm_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            baxter_left_full_arm_control_mapping_t() 
		            {
		                domain = 16;
		                range = 8;
		                mapped_indices = {0,1,2,3,4,5,6,14};
		                output_space_name = "BaxterLeftFullArm";
		                mapping_name = "baxter_left_full_arm_control_mapping";
		            }
		            virtual ~baxter_left_full_arm_control_mapping_t() {}
        	};

        	class baxter_left_arm_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            baxter_left_arm_control_mapping_t() 
		            {
		                domain = 16;
		                range = 7;
		                mapped_indices = {0,1,2,3,4,5,6};
		                output_space_name = "BaxterLeftArm";
		                mapping_name = "baxter_left_arm_control_mapping";
		            }
		            virtual ~baxter_left_arm_control_mapping_t() {}
        	};

        	class baxter_left_gripper_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            baxter_left_gripper_control_mapping_t() 
		            {
		                domain = 16;
		                range = 1;
		                mapped_indices = {14};
		                output_space_name = "Hand";
		                mapping_name = "baxter_left_gripper_control_mapping";
		            }
		            virtual ~baxter_left_gripper_control_mapping_t() {}
        	};
        }
    }
}

#endif