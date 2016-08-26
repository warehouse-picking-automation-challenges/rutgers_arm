
/**
 * @file motoman_right_mappings.hpp
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

#ifndef PRX_MOTOMAN_RIGHT_MAPPINGS_HPP
#define PRX_MOTOMAN_RIGHT_MAPPINGS_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/mapping_functions/subset_mapping.hpp"

#include <boost/assign/list_of.hpp>

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {

        	///Right Arm + torso + gripper
        	class motoman_right_full_with_torso_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_right_full_with_torso_state_mapping_t() 
		            {
		                domain = 18;
		                range = 9;
		                mapped_indices = {0,9,10,11,12,13,14,15,17};
		                output_space_name = "MotomanRightFull";
		                mapping_name = "motoman_right_full_with_torso_state_mapping";
		            }
		            virtual ~motoman_right_full_with_torso_state_mapping_t() {}
        	};

        	///Right Arm + torso
        	class motoman_right_torso_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_right_torso_state_mapping_t() 
		            {
		                domain = 18;
		                range = 8;
		                mapped_indices = {0,9,10,11,12,13,14,15};
		                output_space_name = "MotomanRight";
		                mapping_name = "motoman_right_torso_state_mapping";
		            }
		            virtual ~motoman_right_torso_state_mapping_t() {}
        	};

        	//Right Arm + gripper
        	class motoman_right_full_arm_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_right_full_arm_state_mapping_t() 
		            {
		                domain = 18;
		                range = 8;
		                mapped_indices = {9,10,11,12,13,14,15,17};
		                output_space_name = "MotomanRightFullArm";
		                mapping_name = "motoman_right_full_arm_state_mapping";
		            }
		            virtual ~motoman_right_full_arm_state_mapping_t() {}
        	};

        	//Right Arm 
        	class motoman_right_arm_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_right_arm_state_mapping_t() 
		            {
		                domain = 18;
		                range = 7;
		                mapped_indices = {9,10,11,12,13,14,15};
		                output_space_name = "MotomanRightArm";
		                mapping_name = "motoman_right_arm_state_mapping";
		            }
		            virtual ~motoman_right_arm_state_mapping_t() {}
        	};

        	//Right Gripper
        	class motoman_right_gripper_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_right_gripper_state_mapping_t() 
		            {
		                domain = 18;
		                range = 1;
		                mapped_indices = {17};
		                output_space_name = "Hand";
		                mapping_name = "motoman_right_gripper_state_mapping";
		            }
		            virtual ~motoman_right_gripper_state_mapping_t() {}
        	};


        	///////////////////////////////////////////
        	// Same mappings for the control subsets //
        	///////////////////////////////////////////

        	///Right Arm + torso + gripper
        	class motoman_right_full_with_torso_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_right_full_with_torso_control_mapping_t() 
		            {
		                domain = 18;
		                range = 9;
		                mapped_indices = {0,9,10,11,12,13,14,15,17};
		                output_space_name = "MotomanRightFull";
		                mapping_name = "motoman_right_full_with_torso_control_mapping";
		            }
		            virtual ~motoman_right_full_with_torso_control_mapping_t() {}
        	};

        	class motoman_right_torso_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_right_torso_control_mapping_t() 
		            {
		                domain = 18;
		                range = 8;
		                mapped_indices = {0,9,10,11,12,13,14,15};
		                output_space_name = "MotomanRight";
		                mapping_name = "motoman_right_torso_control_mapping";
		            }
		            virtual ~motoman_right_torso_control_mapping_t() {}
        	};

        	class motoman_right_full_arm_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_right_full_arm_control_mapping_t() 
		            {
		                domain = 18;
		                range = 8;
		                mapped_indices = {9,10,11,12,13,14,15,17};
		                output_space_name = "MotomanRightFullArm";
		                mapping_name = "motoman_right_full_arm_control_mapping";
		            }
		            virtual ~motoman_right_full_arm_control_mapping_t() {}
        	};

        	class motoman_right_arm_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_right_arm_control_mapping_t() 
		            {
		                domain = 18;
		                range = 7;
		                mapped_indices = {9,10,11,12,13,14,15};
		                output_space_name = "MotomanRightArm";
		                mapping_name = "motoman_right_arm_control_mapping";
		            }
		            virtual ~motoman_right_arm_control_mapping_t() {}
        	};

        	class motoman_right_gripper_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_right_gripper_control_mapping_t() 
		            {
		                domain = 18;
		                range = 1;
		                mapped_indices = {17};
		                output_space_name = "Hand";
		                mapping_name = "motoman_right_gripper_control_mapping";
		            }
		            virtual ~motoman_right_gripper_control_mapping_t() {}
        	};

        	///////////////////////////////////////////////////////////////////////////////////
        	////////////////////////////Parallel Unigripper On Right Hand//////////////////////
        	///////////////////////////////////////////////////////////////////////////////////

        	///Right Arm + torso + gripper
        	class motoman_pu_right_full_with_torso_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_pu_right_full_with_torso_state_mapping_t() 
		            {
		                domain = 20;
		                range = 11;
		                mapped_indices = {0,9,10,11,12,13,14,15,17,18,19};
		                output_space_name = "MotomanRightFull_Parallel_Unigripper";
		                mapping_name = "motoman_pu_right_full_with_torso_state_mapping";
		            }
		            virtual ~motoman_pu_right_full_with_torso_state_mapping_t() {}
        	};

        	///Right Arm + torso
        	class motoman_pu_right_torso_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_pu_right_torso_state_mapping_t() 
		            {
		                domain = 20;
		                range = 8;
		                mapped_indices = {0,9,10,11,12,13,14,15};
		                output_space_name = "MotomanRight_Parallel_Unigripper";
		                mapping_name = "motoman_pu_right_torso_state_mapping";
		            }
		            virtual ~motoman_pu_right_torso_state_mapping_t() {}
        	};

        	//Right Arm + gripper
        	class motoman_pu_right_full_arm_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_pu_right_full_arm_state_mapping_t() 
		            {
		                domain = 20;
		                range = 10;
		                mapped_indices = {9,10,11,12,13,14,15,17,18,19};
		                output_space_name = "MotomanRightFullArm_Parallel_Unigripper";
		                mapping_name = "motoman_pu_right_full_arm_state_mapping";
		            }
		            virtual ~motoman_pu_right_full_arm_state_mapping_t() {}
        	};

        	//Right Arm 
        	class motoman_pu_right_arm_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_pu_right_arm_state_mapping_t() 
		            {
		                domain = 20;
		                range = 7;
		                mapped_indices = {9,10,11,12,13,14,15};
		                output_space_name = "MotomanRightArm_Parallel_Unigripper";
		                mapping_name = "motoman_pu_right_arm_state_mapping";
		            }
		            virtual ~motoman_pu_right_arm_state_mapping_t() {}
        	};

        	//Right Gripper
        	class motoman_pu_right_gripper_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_pu_right_gripper_state_mapping_t() 
		            {
		                domain = 20;
		                range = 3;
		                mapped_indices = {17,18,19};
		                output_space_name = "Parallel_Unigripper";
		                mapping_name = "motoman_pu_right_gripper_state_mapping";
		            }
		            virtual ~motoman_pu_right_gripper_state_mapping_t() {}
        	};
        	//Right Front Vacuum
        	class motoman_pu_right_front_vacuum_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_pu_right_front_vacuum_state_mapping_t() 
		            {
		                domain = 20;
		                range = 1;
		                mapped_indices = {17};
		                output_space_name = "Hand";
		                mapping_name = "motoman_pu_right_front_vacuum_state_mapping";
		            }
		            virtual ~motoman_pu_right_front_vacuum_state_mapping_t() {}
        	};
        	//Right Upper Vacuum
        	class motoman_pu_right_upper_vacuum_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_pu_right_upper_vacuum_state_mapping_t() 
		            {
		                domain = 20;
		                range = 1;
		                mapped_indices = {18};
		                output_space_name = "Hand";
		                mapping_name = "motoman_pu_right_upper_vacuum_state_mapping";
		            }
		            virtual ~motoman_pu_right_upper_vacuum_state_mapping_t() {}
        	};
        	//Right Parallel Gripper
        	class motoman_pu_right_parallel_gripper_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_pu_right_parallel_gripper_state_mapping_t() 
		            {
		                domain = 20;
		                range = 1;
		                mapped_indices = {19};
		                output_space_name = "Hand";
		                mapping_name = "motoman_pu_right_parallel_gripper_state_mapping";
		            }
		            virtual ~motoman_pu_right_parallel_gripper_state_mapping_t() {}
        	};


        	///////////////////////////////////////////
        	// Same mappings for the control subsets //
        	///////////////////////////////////////////

        	///Right Arm + torso + gripper
        	class motoman_pu_right_full_with_torso_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_pu_right_full_with_torso_control_mapping_t() 
		            {
		                domain = 20;
		                range = 11;
		                mapped_indices = {0,9,10,11,12,13,14,15,17,18,19};
		                output_space_name = "MotomanRightFull_Parallel_Unigripper";
		                mapping_name = "motoman_pu_right_full_with_torso_control_mapping";
		            }
		            virtual ~motoman_pu_right_full_with_torso_control_mapping_t() {}
        	};

        	class motoman_pu_right_torso_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_pu_right_torso_control_mapping_t() 
		            {
		                domain = 20;
		                range = 8;
		                mapped_indices = {0,9,10,11,12,13,14,15};
		                output_space_name = "MotomanRight_Parallel_Unigripper";
		                mapping_name = "motoman_pu_right_torso_control_mapping";
		            }
		            virtual ~motoman_pu_right_torso_control_mapping_t() {}
        	};

        	class motoman_pu_right_full_arm_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_pu_right_full_arm_control_mapping_t() 
		            {
		                domain = 20;
		                range = 10;
		                mapped_indices = {9,10,11,12,13,14,15,17,18,19};
		                output_space_name = "MotomanRightFullArm_Parallel_Unigripper";
		                mapping_name = "motoman_pu_right_full_arm_control_mapping";
		            }
		            virtual ~motoman_pu_right_full_arm_control_mapping_t() {}
        	};

        	class motoman_pu_right_arm_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_pu_right_arm_control_mapping_t() 
		            {
		                domain = 20;
		                range = 7;
		                mapped_indices = {9,10,11,12,13,14,15};
		                output_space_name = "MotomanRightArm_Parallel_Unigripper";
		                mapping_name = "motoman_pu_right_arm_control_mapping";
		            }
		            virtual ~motoman_pu_right_arm_control_mapping_t() {}
        	};

        	//Right Gripper
        	class motoman_pu_right_gripper_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_pu_right_gripper_control_mapping_t() 
		            {
		                domain = 20;
		                range = 3;
		                mapped_indices = {17,18,19};
		                output_space_name = "Parallel_Unigripper";
		                mapping_name = "motoman_pu_right_gripper_control_mapping";
		            }
		            virtual ~motoman_pu_right_gripper_control_mapping_t() {}
        	};
        	//Right Front Vacuum
        	class motoman_pu_right_front_vacuum_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_pu_right_front_vacuum_control_mapping_t() 
		            {
		                domain = 20;
		                range = 1;
		                mapped_indices = {17};
		                output_space_name = "Hand";
		                mapping_name = "motoman_pu_right_front_vacuum_control_mapping";
		            }
		            virtual ~motoman_pu_right_front_vacuum_control_mapping_t() {}
        	};
        	//Right Upper Vacuum
        	class motoman_pu_right_upper_vacuum_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_pu_right_upper_vacuum_control_mapping_t() 
		            {
		                domain = 20;
		                range = 1;
		                mapped_indices = {18};
		                output_space_name = "Hand";
		                mapping_name = "motoman_pu_right_upper_vacuum_control_mapping";
		            }
		            virtual ~motoman_pu_right_upper_vacuum_control_mapping_t() {}
        	};
        	//Right Parallel Gripper
        	class motoman_pu_right_parallel_gripper_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            motoman_pu_right_parallel_gripper_control_mapping_t() 
		            {
		                domain = 20;
		                range = 1;
		                mapped_indices = {19};
		                output_space_name = "Hand";
		                mapping_name = "motoman_pu_right_parallel_gripper_control_mapping";
		            }
		            virtual ~motoman_pu_right_parallel_gripper_control_mapping_t() {}
        	};
        }
    }
}

#endif