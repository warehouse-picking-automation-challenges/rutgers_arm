
/**
 * @file motoman_left_mappings.hpp
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

#ifndef PRX_MOTOMAN_LEFT_MAPPINGS_HPP
#define PRX_MOTOMAN_LEFT_MAPPINGS_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/mapping_functions/subset_mapping.hpp"

#include <boost/assign/list_of.hpp>

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {

        	///Left Arm + torso + gripper
        	class motoman_left_full_with_torso_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_left_full_with_torso_state_mapping_t() 
		            {
		                domain = 18;
		                range = 10;
		                mapped_indices = {0,1,2,3,4,5,6,7,8,16};
		                output_space_name = "MotomanLeftFull";
		                mapping_name = "motoman_left_full_with_torso_state_mapping";
		            }
		            virtual ~motoman_left_full_with_torso_state_mapping_t() {}
        	};

        	///Left Arm + torso
        	class motoman_left_torso_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_left_torso_state_mapping_t() 
		            {
		                domain = 18;
		                range = 9;
		                mapped_indices = {0,1,2,3,4,5,6,7,8};
		                output_space_name = "MotomanLeft";
		                mapping_name = "motoman_left_torso_state_mapping";
		            }
		            virtual ~motoman_left_torso_state_mapping_t() {}
        	};

        	//Left Arm + gripper
        	class motoman_left_full_arm_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_left_full_arm_state_mapping_t() 
		            {
		                domain = 18;
		                range = 9;
		                mapped_indices = {1,2,3,4,5,6,7,8,16};
		                output_space_name = "MotomanLeftFullArm";
		                mapping_name = "motoman_left_full_arm_state_mapping";
		            }
		            virtual ~motoman_left_full_arm_state_mapping_t() {}
        	};

        	//Left Arm 
        	class motoman_left_arm_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_left_arm_state_mapping_t() 
		            {
		                domain = 18;
		                range = 8;
		                mapped_indices = {1,2,3,4,5,6,7,8};
		                output_space_name = "MotomanLeftArm";
		                mapping_name = "motoman_left_arm_state_mapping";
		            }
		            virtual ~motoman_left_arm_state_mapping_t() {}
        	};

        	//Left Gripper
        	class motoman_left_gripper_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_left_gripper_state_mapping_t() 
		            {
		                domain = 18;
		                range = 1;
		                mapped_indices = {16};
		                output_space_name = "Hand";
		                mapping_name = "motoman_left_gripper_state_mapping";
		            }
		            virtual ~motoman_left_gripper_state_mapping_t() {}
        	};


        	///////////////////////////////////////////
        	// Same mappings for the control subsets //
        	///////////////////////////////////////////

        	///Left Arm + torso + gripper
        	class motoman_left_full_with_torso_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_left_full_with_torso_control_mapping_t() 
		            {
		                domain = 18;
		                range = 10;
		                mapped_indices = {0,1,2,3,4,5,6,7,8,16};
		                output_space_name = "MotomanLeftFull";
		                mapping_name = "motoman_left_full_with_torso_control_mapping";
		            }
		            virtual ~motoman_left_full_with_torso_control_mapping_t() {}
        	};

        	class motoman_left_torso_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_left_torso_control_mapping_t() 
		            {
		                domain = 18;
		                range = 9;
		                mapped_indices = {0,1,2,3,4,5,6,7,8};
		                output_space_name = "MotomanLeft";
		                mapping_name = "motoman_left_torso_control_mapping";
		            }
		            virtual ~motoman_left_torso_control_mapping_t() {}
        	};

        	class motoman_left_full_arm_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_left_full_arm_control_mapping_t() 
		            {
		                domain = 18;
		                range = 9;
		                mapped_indices = {1,2,3,4,5,6,7,8,16};
		                output_space_name = "MotomanLeftFullArm";
		                mapping_name = "motoman_left_full_arm_control_mapping";
		            }
		            virtual ~motoman_left_full_arm_control_mapping_t() {}
        	};

        	class motoman_left_arm_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_left_arm_control_mapping_t() 
		            {
		                domain = 18;
		                range = 8;
		                mapped_indices = {1,2,3,4,5,6,7,8};
		                output_space_name = "MotomanLeftArm";
		                mapping_name = "motoman_left_arm_control_mapping";
		            }
		            virtual ~motoman_left_arm_control_mapping_t() {}
        	};

        	class motoman_left_gripper_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_left_gripper_control_mapping_t() 
		            {
		                domain = 18;
		                range = 1;
		                mapped_indices = {16};
		                output_space_name = "Hand";
		                mapping_name = "motoman_left_gripper_control_mapping";
		            }
		            virtual ~motoman_left_gripper_control_mapping_t() {}
        	};
        }
    }
}

#endif