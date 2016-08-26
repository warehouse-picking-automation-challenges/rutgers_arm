/**
 * @file coordination_prm_query.cpp
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

#include "planning/motion_planners/coordination_prm_query.hpp"
#include "prx/utilities/goals/goal_state.hpp"
#include <boost/assign/list_of.hpp>


#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::coordination_manipulation::coordination_prm_query_t, prx::plan::query_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace coordination_manipulation
        {
            coordination_prm_query_t::coordination_prm_query_t()
            {
                left_plan = NULL;
                right_plan = NULL;
                
                left_cup_safe_state = NULL;
                right_cup_safe_state = NULL;
                left_cup_space = NULL;
                right_cup_space = NULL;
                left_armcup_space = NULL;
                right_armcup_space = NULL;

                max_velocity_bias = false;
                incremental_assignment = false;
                left_finished = false;
                right_finished = false;
                
                _x = 0;
                _y = 0;
                
                left_state_space = left_control_space = NULL;
                right_state_space = right_control_space = NULL;
                
                state_memory = {&_x,&_y};
                coordination_state_space = new space_t("DD", state_memory);
                
                std::vector<double> min = {0.0,0.0};
                std::vector<double> max = {1.0,1.0};
                bounds.push_back(new bounds_t());
                bounds.push_back(new bounds_t());
                bounds::set_bounds(bounds, min, max);
                coordination_state_space->set_bounds(bounds);
                
                this->start_state = coordination_state_space->alloc_point();
                coordination_goal_state = coordination_state_space->alloc_point();
                ideal_goal_state = coordination_state_space->alloc_point();  

            }

            coordination_prm_query_t::~coordination_prm_query_t()
            {
                coordination_state_space->free_point(coordination_goal_state);
                coordination_state_space->free_point(ideal_goal_state);
                left_cup_space->free_point(left_cup_safe_state);
                right_cup_space->free_point(right_cup_safe_state);
                delete coordination_state_space;
            }
            
            void coordination_prm_query_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                motion_planning_query_t::init(reader,template_reader);
                
                goal_state_t* goal_st = dynamic_cast<goal_state_t*>(goal);
                goal_st->link_space(coordination_state_space);
                goal_st->copy_goal_state(coordination_goal_state);
            }
            
            void coordination_prm_query_t::link_left_arm(const space_t* left_arm_state_space, const space_t* left_arm_control_space, plan_t* left_arm_plan)
            {
                left_state_space = left_arm_state_space;
                left_control_space = left_arm_control_space;
                left_plan = left_arm_plan;

                partial_plan.link_control_space(left_control_space);
                partial_plan.link_state_space(left_state_space);
                
                PRX_DEBUG_COLOR("Linked left arm plan: " << left_plan->size(), PRX_TEXT_BLUE);
            }
            
            void coordination_prm_query_t::link_right_arm(const space_t* right_arm_state_space, const space_t* right_arm_control_space, plan_t* right_arm_plan)
            {
                right_state_space = right_arm_state_space;
                right_control_space = right_arm_control_space;
                right_plan = right_arm_plan;
                
                PRX_DEBUG_COLOR("Linked right arm plan: " << right_plan->size(), PRX_TEXT_RED);
            }
            
            void coordination_prm_query_t::link_imaginary_cups(const space_t* left_space, const space_t* right_space)
            {
                left_cup_space = left_space;
                right_cup_space = right_space;
                
                left_cup_safe_state =  left_cup_space->alloc_point();
                right_cup_safe_state = right_cup_space->alloc_point();
                
                PRX_DEBUG_COLOR("LEFT CUP SAFE STATE: " << left_cup_space->print_point(left_cup_safe_state), PRX_TEXT_BLUE);
                PRX_DEBUG_COLOR("RIGHT CUP SAFE STATE: " << right_cup_space->print_point(right_cup_safe_state), PRX_TEXT_RED);
                
            }
            
            void coordination_prm_query_t::link_armcups(const util::space_t* left_armcup, const util::space_t* right_armcup)
            {
                left_armcup_space = left_armcup;
                right_armcup_space = right_armcup;
                
                left_armcup_path.link_space(left_armcup_space);
                right_armcup_path.link_space(right_armcup_space);
            }

            
            void coordination_prm_query_t::setup()
            {
                PRX_ASSERT(left_control_space != NULL);
                PRX_ASSERT(right_control_space != NULL);
                PRX_ASSERT(left_state_space != NULL);
                PRX_ASSERT(right_state_space != NULL);
                PRX_ASSERT(left_cup_safe_state != NULL);
                PRX_ASSERT(right_cup_safe_state != NULL);
                PRX_ASSERT(left_cup_space != NULL);
                PRX_ASSERT(right_cup_space != NULL);
                PRX_ASSERT(left_armcup_space != NULL);
                PRX_ASSERT(right_armcup_space != NULL);
                
                
                double left_max_steps = left_plan->size()-1;
                double right_max_steps = right_plan->size()-1;
                
                std::vector<double> min = {0.0,0.0};
                std::vector<double> max = {right_max_steps,left_max_steps};
                bounds::set_bounds(bounds, min, max);
                coordination_state_space->set_bounds(bounds);
                
                coordination_goal_state->at(0) = right_max_steps;
                coordination_goal_state->at(1) = left_max_steps;
                
                goal->copy_goal_state(coordination_goal_state);
                
                ideal_goal_state->at(0) = ideal_goal_state->at(1) = left_max_steps > right_max_steps ? right_max_steps : left_max_steps;
                
                left_finished = false;
                right_finished = false;
                
                partial_plan.clear();
            }

        }

    }
}
