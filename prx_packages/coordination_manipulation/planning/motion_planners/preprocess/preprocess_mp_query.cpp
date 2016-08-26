/**
 * @file preprocess_mp_query.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "planning/motion_planners/preprocess/preprocess_mp_query.hpp"
#include "prx/utilities/goals/goal_state.hpp"
#include <boost/assign/list_of.hpp>


#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::coordination_manipulation::preprocess_mp_query_t, prx::plan::query_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace coordination_manipulation
        {
            preprocess_mp_query_t::preprocess_mp_query_t()
            {
                left_trajectory = NULL;
                right_trajectory = NULL;
                
                left_cup_safe_state = NULL;
                right_cup_safe_state = NULL;
                left_cup_space = NULL;
                right_cup_space = NULL;
                left_armcup_space = NULL;
                right_armcup_space = NULL;       
                _x = 0;
                _y = 0;
                
                left_state_space = left_control_space = NULL;
                right_state_space = right_control_space = NULL;
                
                state_memory = {&_x,&_y};
                preprocess_space = new space_t("XY", state_memory);
                
                std::vector<double> min = {0.0,0.0};
                std::vector<double> max = {1.0,1.0};
                bounds.push_back(new bounds_t());
                bounds.push_back(new bounds_t());
                bounds::set_bounds(bounds, min, max);
                preprocess_space->set_bounds(bounds); 
                
                preprocess_goal = preprocess_space->alloc_point();
                
                approximate_collision_check = false;

            }

            preprocess_mp_query_t::~preprocess_mp_query_t()
            {
                preprocess_space->free_point(preprocess_goal);
                if (!approximate_collision_check)
                {
                    left_cup_space->free_point(left_cup_safe_state);
                    right_cup_space->free_point(right_cup_safe_state);
                    left_armcup_space->free_point(left_armcup_state);
                    right_armcup_space->free_point(right_armcup_state);
                }
                delete preprocess_space;
            }
            
            void preprocess_mp_query_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                motion_planning_query_t::init(reader,template_reader);
                
                goal_state_t* goal_st = dynamic_cast<goal_state_t*>(goal);
                goal_st->link_space(preprocess_space);
                goal_st->copy_goal_state(preprocess_goal);
            }
            
            void preprocess_mp_query_t::link_left_arm(const space_t* left_arm_state_space, const space_t* left_arm_control_space)
            {
                left_state_space = left_arm_state_space;
                left_control_space = left_arm_control_space;                

            }
            
            void preprocess_mp_query_t::link_right_arm(const space_t* right_arm_state_space, const space_t* right_arm_control_space)
            {
                right_state_space = right_arm_state_space;
                right_control_space = right_arm_control_space;

            }
            
            void preprocess_mp_query_t::link_imaginary_cups(const space_t* left_space, const space_t* right_space)
            {
                left_cup_space = left_space;
                right_cup_space = right_space;
                
                left_cup_safe_state =  left_cup_space->alloc_point();
                right_cup_safe_state = right_cup_space->alloc_point();
                
                PRX_DEBUG_COLOR("LEFT CUP SAFE STATE: " << left_cup_space->print_point(left_cup_safe_state), PRX_TEXT_BLUE);
                PRX_DEBUG_COLOR("RIGHT CUP SAFE STATE: " << right_cup_space->print_point(right_cup_safe_state), PRX_TEXT_RED);
                
            }
            
            void preprocess_mp_query_t::link_armcups(const util::space_t* left_armcup, const util::space_t* right_armcup)
            {
                left_armcup_space = left_armcup;
                right_armcup_space = right_armcup;
                left_armcup_state = left_armcup_space->alloc_point();
                right_armcup_state = right_armcup_space->alloc_point();
            }
                            
            void preprocess_mp_query_t::link_arm_trajectories(sim::trajectory_t* left_arm_trajectory, sim::trajectory_t* right_arm_trajectory)
            {
                left_trajectory = left_arm_trajectory;
                right_trajectory = right_arm_trajectory;
                
                PRX_DEBUG_COLOR("Linked right arm trajectory: " << right_trajectory->size(), PRX_TEXT_RED);
                PRX_DEBUG_COLOR("Linked left arm trajectory: " << left_trajectory->size(), PRX_TEXT_BLUE);
            }

            void preprocess_mp_query_t::define_coordination_problem(unsigned rob_index, unsigned lob_index, unsigned rs_index, unsigned ls_index, 
                                                   std::string left_object_type, std::string right_object_type, std::string experiment_type)
            {
                right_object_index = rob_index;
                left_object_index = lob_index;
                right_start_index = rs_index;
                left_start_index = ls_index;
                right_object = right_object_type;
                left_object = left_object_type;
                experiment = experiment_type;
            }
            
            void preprocess_mp_query_t::setup()
            {
                PRX_ASSERT(left_control_space != NULL);
                PRX_ASSERT(right_control_space != NULL);
                PRX_ASSERT(left_state_space != NULL);
                PRX_ASSERT(right_state_space != NULL);
                if (!approximate_collision_check)
                {
                    PRX_ASSERT(left_cup_safe_state != NULL);
                    PRX_ASSERT(right_cup_safe_state != NULL);
                    PRX_ASSERT(left_cup_space != NULL);
                    PRX_ASSERT(right_cup_space != NULL);
                    PRX_ASSERT(left_armcup_space != NULL);
                    PRX_ASSERT(right_armcup_space != NULL);
                }
                PRX_ASSERT(left_trajectory != NULL);
                PRX_ASSERT(right_trajectory != NULL);
                
                
                double left_max_steps = left_trajectory->size()-1;
                double right_max_steps = right_trajectory->size()-1;
                
                std::vector<double> min = {0.0,0.0};
                std::vector<double> max = {right_max_steps,left_max_steps};
                bounds::set_bounds(bounds, min, max);
                preprocess_space->set_bounds(bounds);
                
                preprocess_goal->at(0) = right_max_steps;
                preprocess_goal->at(1) = left_max_steps;
                
                goal->copy_goal_state(preprocess_goal);
 
            }

        }

    }
}
