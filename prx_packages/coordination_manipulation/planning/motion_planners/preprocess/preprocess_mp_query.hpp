/**
 * @file preprocess_mp_query.hpp
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

#ifndef PRX_PREPROCESS_MP_QUERY_HPP
#define	PRX_PREPROCESS_MP_QUERY_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "../../../../manipulation/simulation/systems/plants/manipulator.hpp"

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
            class preprocess_mp_query_t : public plan::motion_planning_query_t
            {
              public:

                preprocess_mp_query_t();
                virtual ~preprocess_mp_query_t();
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);
                virtual void link_left_arm(const util::space_t* left_arm_state_space, const util::space_t* left_arm_control_space);
                virtual void link_right_arm(const util::space_t* right_arm_state_space, const util::space_t* right_arm_control_space);
                virtual void link_arm_trajectories(sim::trajectory_t* left_arm_trajectory, sim::trajectory_t* right_arm_trajectory);
                virtual void link_imaginary_cups(const util::space_t* left_space, const util::space_t* right_space);
                virtual void link_armcups(const util::space_t* left_armcup, const util::space_t* right_armcup);
                
                virtual void define_coordination_problem(unsigned rob_index, unsigned lob_index, unsigned rs_index, unsigned ls_index, 
                                                   std::string left_object_type, std::string right_object_type, std::string experiment_type);
                
                virtual void setup();
                                
                sim::state_t* left_cup_safe_state;
                sim::state_t* right_cup_safe_state;
                const util::space_t* left_cup_space;
                const util::space_t* right_cup_space;

                const util::space_t* left_armcup_space;
                sim::state_t* left_armcup_state;
                const util::space_t* right_armcup_space;
                sim::state_t* right_armcup_state;
                
                const util::space_t* left_state_space;
                const util::space_t* left_control_space;
                sim::trajectory_t* left_trajectory;
                
                const util::space_t* right_state_space;
                const util::space_t* right_control_space;
                sim::trajectory_t* right_trajectory;
                
                sim::state_t* preprocess_goal;
                util::space_t* preprocess_space;
                
                unsigned right_object_index, left_object_index;
                unsigned right_start_index, left_start_index;
                std::string right_object, left_object;
                std::string experiment;
                
                bool approximate_collision_check;
                
            protected:
                double _x, _y;
                std::vector<double*> state_memory;
                std::vector<util::bounds_t*> bounds;


            };
        }
    }
}

#endif

