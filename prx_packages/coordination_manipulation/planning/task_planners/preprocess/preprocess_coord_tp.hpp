/**
 * @file preprocess_coord_tp.hpp
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

#ifndef PRX_PREPROCESS_COORD_TP_HPP
#define	PRX_PREPROCESS_COORD_TP_HPP

#include "prx/utilities/definitions/sys_clock.hpp"

#include "prx/planning/task_planners/task_planner.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"

#include "planning/motion_planners/coordination_prm_query.hpp"
#include "planning/motion_planners/coordination_prm.hpp"

#include "planning/motion_planners/preprocess/preprocess_mp_query.hpp"
#include "planning/motion_planners/preprocess/preprocess_coord_mp.hpp"

#include "../../../../manipulation/simulation/systems/plants/manipulator.hpp"
#include "../../../../manipulation/planning/manipulation_world_model.hpp"

namespace prx
{
    namespace util
    {
        class linear_distance_metric_t;
    }

    namespace plan
    {
        class motion_planner_t;
    }

    namespace packages
    {
        namespace manipulation
        {
            class movable_body_plant_t;
            class manipulation_specification_t;
        }
        namespace coordination_manipulation
        {
            using namespace manipulation;
            
            struct coordination_constraint_t
            {
                int plan_index;
                unsigned control_index;
            };

            /**
             * Coordination task planner for multiple manipulators
             *
             * @authors Andrew Kimmel
             */
            class preprocess_coordination_tp_t : public plan::task_planner_t
            {
            public:

                
                preprocess_coordination_tp_t();
                ~preprocess_coordination_tp_t();

                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                virtual void setup();
                virtual void link_world_model( plan::world_model_t* const model );
                virtual const util::statistics_t* get_statistics();
                virtual void link_specification( plan::specification_t* new_spec );
                virtual void link_query( plan::query_t* new_query );
                virtual bool execute();
                virtual void resolve_query();
                virtual void reset();
                virtual bool succeeded() const;

            protected:
                
                std::string trajectories_directory;
                
                // Helper Methods
                virtual void find_plants();
                virtual void store_start_state();
                virtual void init_preprocess_mp();
                
                preprocess_mp_query_t* preprocess_query;
                preprocess_coordination_mp_t* preprocess_mp;
                
                sim::state_t* left_cup_safe_state;
                sim::state_t* right_cup_safe_state;
                const util::space_t* left_cup_space;
                const util::space_t* right_cup_space;
                
                const util::space_t* left_armcup_space;
                const util::space_t* right_armcup_space;
          
                std::string algorithm, experiment;
                util::sys_clock_t computation_timer;

                manipulator_t* manip;
                manipulation_world_model_t* manipulation_model;
                std::string full_manipulator_context_name,left_context_name, right_context_name;
                
                std::vector<std::string> object_types;
                bool check_all_trajectories, approximate_collisions;
                std::vector<int> num_trajectories_per_object_type;
                
                sim::trajectory_t current_left_trajectory, current_right_trajectory;
                
                sim::plan_t left_arm_full_plan;
                const util::space_t* left_arm_control_space;
                const util::space_t* left_arm_state_space;
                util::space_point_t* left_arm_safe_state;
                
                sim::plan_t right_arm_full_plan;
                const util::space_t* right_arm_control_space;
                const util::space_t* right_arm_state_space;
                util::space_point_t* right_arm_safe_state;
                
                util::space_t* full_control_space;
                util::space_t* full_state_space;

                util::space_point_t* global_start_state; //The global start state that is read in from input
                
                unsigned start_index;
                unsigned end_index;


            };
        }
    }
}

#endif	// PRX_PREPROCESS_COORD_TP_HPP

