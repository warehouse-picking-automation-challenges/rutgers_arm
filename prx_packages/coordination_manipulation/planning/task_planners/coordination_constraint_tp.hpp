/**
 * @file coordination_constraint_tp.hpp
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

#ifndef PRX_COORDINATION_CONSTRAINT_TP_HPP
#define PRX_COORDINATION_CONSTRAINT_TP_HPP

#include "planning/motion_planners/coordination_graph/coordination_constraint.hpp"
#include "planning/motion_planners/coordination_graph/coordination_constraint_astar.hpp"
#include "planning/motion_planners/coordination_graph/multi_constraint_astar.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/task_planners/task_planner.hpp"

#include "../../../manipulation/simulation/systems/plants/manipulator.hpp"
#include "../../../manipulation/simulation/systems/plants/movable_body_plant.hpp"
#include "../../../manipulation/planning/manipulation_world_model.hpp"

namespace prx
{

    namespace packages
    {

        namespace coordination_manipulation
        {
            using namespace manipulation;

            /**
             * Coordination task planner for multiple manipulators
             *
             * @authors Andrew Kimmel
             */
            class coordination_constraint_tp_t : public plan::task_planner_t
            {
            public:
                
                enum scheduling_type_t
                {
                    MANUAL_SCHEDULE, RANDOM_SCHEDULE, INCREMENTAL_SCHEDULE, MINCONF_SCHEDULE
                };

                enum coordination_type_t
                {

                    NONE_COORDINATION, PRM_COORDINATION, PRM_MAX_VEL_COORDINATION, GRID_COORDINATION, GRID_MAX_VEL_COORDINATION
                };

                enum selection_type_t
                {

                    NONE_SELECTION, SHORTEST_SELECTION, LONGEST_SELECTION, MINCONF_SELECTION, RANDOM_SELECTION
                };
                
                enum minimization_type_t
                {
                    NONE_MINIMIZATION, JOINT_MINIMIZATION, MASTER_ONLY_MINIMIZATION
                };
                
                enum
                {
                    BAXTER_LEFT_ARM = 0,
                    BAXTER_RIGHT_ARM = 1
                };
                
                coordination_constraint_tp_t();
                ~coordination_constraint_tp_t();

                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                virtual void setup();
                virtual void link_query( plan::query_t* new_query );
                void link_world_model(plan::world_model_t* const model );
                virtual bool execute();
                virtual void resolve_query();

            protected:
                
                /** Multiple experiments code */
                virtual const util::statistics_t* get_statistics();
                virtual bool succeeded() const;
                void save_statistics();
                void write_statistics();
                
                void get_new_tasks();
                void get_old_tasks();
                
                std::vector<std::string> original_left_objects, original_right_objects;
                std::vector<unsigned> original_left_tasks, original_right_tasks;
                unsigned number_of_tasks;
                
                std::vector<double>solution_time;
                std::vector<double>computation_time;
                double min_solution_time, max_solution_time;
                double max_computation_time, min_computation_time;
                
                unsigned number_of_experiments, total_new_experiments;
                unsigned num_left_tasks_to_assign, num_right_tasks_to_assign;
                unsigned current_experiment;
                
                
                /** Coordination constraint stuff */
                bool use_preprocessed_constraints;
                std::string constraints_directory;
                coordination_constraints_t* precomputed_constraints;
                coordination_constraint_astar_t* constraint_aware_astar;
                multi_constraint_astar_t* multi_constraint_astar;
                
                bool naive, batch, incremental;
                coordination_constraint_astar_t::coordination_heuristic_type_t bias_alg;
                plan::motion_planning_query_t* mp_query;
                
                /** High-level algorithm */
                virtual void assign_tasks();
                virtual void coordinate_paths();
                virtual void construct_full_individual_plans();
                virtual void construct_full_joined_plan();
                
                /** task assignment algorithms */
                virtual void random_assignment();
                virtual void min_conf_assignment();
                
                /** Incremental Assignment Functions */
                virtual void incremental_assignment();
                bool get_master_and_task_assignment(unsigned& master_arm, unsigned& slave_arm, unsigned& goal_right_step, unsigned& goal_left_step, unsigned& rob_index, unsigned& lob_index, std::string& right_type, std::string& left_type);
                void solve_slave_task_assignment(unsigned slave_arm, unsigned start_right_step, unsigned start_left_step, unsigned& goal_right_step, unsigned& goal_left_step, unsigned& rob_index, unsigned& lob_index, std::string& right_type, std::string& left_type, std::deque<unsigned>& right_arm_solution_vertices, std::deque<unsigned>& left_arm_solution_vertices);
                void remove_task(unsigned task, unsigned arm);
                
                /** Previously from Naive coord manip tp **/
                
                // Helper Methods
                virtual void find_plants();
                virtual void store_start_state();
                void print_full_state();
                void print_tasks();
                
                
                /** Incremental Assignment Functions */
                sim::plan_t fully_coordinated_plan;
                void add_task_to_query(unsigned task, unsigned arm);
                
                std::string results_file;
                
                scheduling_type_t scheduling_type;
                coordination_type_t coordination_type;
                selection_type_t selection_type;
                minimization_type_t minimization_type;
                
//                coordination_prm_query_t* prm_query;
//                coordination_prm_t* coordination_prm;
                
                std::string algorithm, experiment;
                util::sys_clock_t computation_timer;

                std::string plans_directory;
                util::hash_t<std::string, std::vector<sim::plan_t> > left_arm_plans;
                util::hash_t<std::string, std::vector<sim::plan_t> > right_arm_plans;

                manipulator_t* manip;
                manipulation_world_model_t* manipulation_model;
                std::string full_manipulator_context_name,left_context_name, right_context_name;
                
                std::vector<std::string> object_types;
                std::vector<int> num_plans_per_object_type;
                
                sim::plan_t left_arm_full_plan;
                sim::control_t* left_zero_control;
                const util::space_t* left_arm_control_space;
                const util::space_t* left_arm_state_space;
                util::space_point_t* left_arm_safe_state;
                
                sim::plan_t right_arm_full_plan;
                sim::control_t* right_zero_control;
                const util::space_t* right_arm_control_space;
                const util::space_t* right_arm_state_space;
                util::space_point_t* right_arm_safe_state;
                
                sim::plan_t all_arms_plan;
                util::space_t* full_control_space;
                util::space_t* full_state_space;
                util::space_t* both_arm_state_space;
                util::space_t* both_arm_control_space;
                
                
                std::vector<unsigned> left_arm_tasks;
                std::vector<unsigned> right_arm_tasks;
                std::vector<unsigned> completed_left_arm_tasks;
                std::vector<unsigned> completed_right_arm_tasks;
                
                std::vector<std::string> left_arm_objects;
                std::vector<std::string> right_arm_objects;
                std::vector<std::string> completed_left_arm_objects;
                std::vector<std::string> completed_right_arm_objects;
                

                std::vector< movable_body_plant_t* > objects; //List of the objects in the scene

                util::space_t* object_space; //Space of the object
                util::space_point_t* object_start; //Start state of the object.
                
                util::space_point_t* global_start_state; //The global start state that is read in from input                

            };
        }
    }
}

#endif //PRX_COORDINATION_CONSTRAINT_TP_HPP
