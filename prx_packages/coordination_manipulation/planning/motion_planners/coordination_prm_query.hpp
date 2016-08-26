/**
 * @file coordination_prm_query.hpp
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
#pragma once

#ifndef PRX_COORDINATION_PRM_QUERY_HPP
#define	PRX_COORDINATION_PRM_QUERY_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
        class goal_t;
    }
    
    namespace plan
    {
        class stopping_criteria_t;
    }

    namespace packages
    {
        
        namespace coordination_manipulation
        {

            /**
             * @author Andrew Kimmel
             */
            class coordination_prm_query_t : public plan::motion_planning_query_t
            {
              public:

                coordination_prm_query_t();
                virtual ~coordination_prm_query_t();
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);
                virtual void link_left_arm(const util::space_t* left_arm_state_space, const util::space_t* left_arm_control_space, sim::plan_t* left_arm_plan);
                virtual void link_right_arm(const util::space_t* right_arm_state_space, const util::space_t* right_arm_control_space, sim::plan_t* right_arm_plan);
                virtual void link_imaginary_cups(const util::space_t* left_space, const util::space_t* right_space);
                virtual void link_armcups(const util::space_t* left_armcup, const util::space_t* right_armcup);
                virtual void setup();
                
                util::space_t* coordination_state_space;
                
                sim::state_t* left_cup_safe_state;
                sim::state_t* right_cup_safe_state;
                const util::space_t* left_cup_space;
                const util::space_t* right_cup_space;

                bool left_finished, right_finished;
                sim::plan_t partial_plan;
                
                
                const util::space_t* left_armcup_space;
                sim::trajectory_t left_armcup_path;
                const util::space_t* right_armcup_space;
                sim::trajectory_t right_armcup_path;
                
                const util::space_t* left_state_space;
                const util::space_t* left_control_space;
                sim::plan_t* left_plan;
                
                const util::space_t* right_state_space;
                const util::space_t* right_control_space;
                sim::plan_t* right_plan;
                
                sim::state_t* coordination_goal_state;
                sim::state_t* ideal_goal_state;
                
                bool max_velocity_bias;
                bool incremental_assignment;
                
            protected:
                double _x, _y;
                std::vector<double*> state_memory;
                std::vector<util::bounds_t*> bounds;


            };
        }
    }
}

#endif

