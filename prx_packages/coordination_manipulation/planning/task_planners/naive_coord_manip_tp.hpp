///**
// * @file coordination_manip_tp.hpp
// *
// * @copyright Software License Agreement (BSD License)
// * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick
// * All Rights Reserved.
// * For a full description see the file named LICENSE.
// *
// * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
// *
// * Email: pracsys@googlegroups.com
// */
//
//#pragma once
//
//#ifndef PRX_COORDINATION_MANIPULATION_HPP
//#define PRX_COORDINATION_MANIPULATION_HPP
//
//#include "prx/utilities/definitions/sys_clock.hpp"
//
//#include "prx/planning/task_planners/task_planner.hpp"
//#include "prx/planning/queries/motion_planning_query.hpp"
//#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
//#include "prx/planning/world_model.hpp"
//
//#include "planning/motion_planners/coordination_prm_query.hpp"
//#include "planning/motion_planners/coordination_prm.hpp"
//
//#include "../../../baxter/simulation/plants/manipulator.hpp"
//
//namespace prx
//{
//    namespace util
//    {
//        class linear_distance_metric_t;
//    }
//
//    namespace plan
//    {
//        class motion_planner_t;
//    }
//
//    namespace packages
//    {
//        namespace manipulation
//        {
//            class manip_sampler_t;
//            class movable_body_plant_t;
//            class manipulation_specification_t;
//        }
//
//        namespace coordination_manipulation
//        {
//            using namespace manipulation;
//
//            /**
//             * Coordination task planner for multiple manipulators
//             *
//             * @authors Andrew Kimmel
//             */
//            class naive_coord_manip_tp_t : public plan::task_planner_t
//            {
//            public:
//                
//                enum scheduling_type_t
//                {
//                    MANUAL_SCHEDULE, RANDOM_SCHEDULE, INCREMENTAL_SCHEDULE, MINCONF_SCHEDULE
//                };
//
//                enum coordination_type_t
//                {
//
//                    NONE_COORDINATION, PRM_COORDINATION, PRM_MAX_VEL_COORDINATION, GRID_COORDINATION, GRID_MAX_VEL_COORDINATION
//                };
//
//                enum selection_type_t
//                {
//
//                    NONE_SELECTION, SHORTEST_SELECTION, LONGEST_SELECTION, MINCONF_SELECTION, RANDOM_SELECTION
//                };
//                
//                enum minimization_type_t
//                {
//                    NONE_MINIMIZATION, JOINT_MINIMIZATION, MASTER_ONLY_MINIMIZATION
//                };
//                
//                enum
//                {
//                    BAXTER_LEFT_ARM = 0,
//                    BAXTER_RIGHT_ARM = 1
//                };
//                
//                naive_coord_manip_tp_t();
//                ~naive_coord_manip_tp_t();
//
//                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);
//
//                virtual void setup();
//                virtual void link_world_model( plan::world_model_t* const model );
//                virtual const util::statistics_t* get_statistics();
//                virtual void link_specification( plan::specification_t* new_spec );
//                virtual void link_query( plan::query_t* new_query );
//                virtual bool execute();
//                virtual void resolve_query();
//                virtual void reset();
//                virtual bool succeeded() const;
//                virtual void set_param( const std::string& path, const std::string& parameter_name, const boost::any& value );
//
//            protected:
//                // Helper Methods
//                virtual void find_plants();
//                virtual void store_start_state();
//                void print_full_state();
//                void print_tasks();
//                
//                void init_coordination_prm();
//                
//                /** High-level algorithm */
//                virtual void assign_tasks();
//                virtual void coordinate_paths();
//                virtual void construct_full_individual_plans();
//                virtual void construct_full_joined_plan();
//                
//                /** task assignment algorithms */
//                virtual void random_assignment();
//                virtual void min_conf_assignment();
//                
//                /** Incremental Assignment Functions */
//                sim::plan_t incremental_coordinated_plan;
//                sim::plan_t fully_coordinated_plan;
//                sim::plan_t incremental_partial_plan;
//                virtual void incremental_assignment();
//                bool get_master_and_task_assignment(unsigned& master_task, unsigned& master_arm, unsigned& slave_arm, unsigned& remaining_tasks);
//                void solve_slave_task_assignment(unsigned slave_arm, sim::plan_t& best_coordinated_plan, sim::plan_t& best_partial_plan, unsigned& next_task);
//                void add_task_to_query(unsigned task, unsigned arm);
//                void remove_task(unsigned task, unsigned arm);
//                
//                std::string results_file;
//                
//                
//                scheduling_type_t scheduling_type;
//                coordination_type_t coordination_type;
//                selection_type_t selection_type;
//                minimization_type_t minimization_type;
//                
//                coordination_prm_query_t* prm_query;
//                coordination_prm_t* coordination_prm;
//                
//                std::string algorithm, experiment;
//                util::sys_clock_t computation_timer;
//
//                std::string plans_directory;
//                unsigned num_left_plans, num_right_plans;
//                std::vector<sim::plan_t> left_arm_plans;
//                std::vector<sim::plan_t> right_arm_plans;
//                
//                sim::plan_t left_arm_full_plan;
//                util::space_t* left_arm_control_space;
//                util::space_t* left_arm_state_space;
//                util::space_point_t* left_arm_safe_state;
//                
//                sim::plan_t right_arm_full_plan;
//                util::space_t* right_arm_control_space;
//                util::space_t* right_arm_state_space;
//                util::space_point_t* right_arm_safe_state;
//                
//                sim::plan_t all_arms_plan;
//                util::space_t* full_control_space;
//                util::space_t* full_state_space;
//                util::space_t* both_arm_state_space;
//                util::space_t* both_arm_control_space;
//                
//                
//                std::vector<unsigned> left_arm_tasks;
//                std::vector<unsigned> right_arm_tasks;
//                std::vector<unsigned> completed_left_arm_tasks;
//                std::vector<unsigned> completed_right_arm_tasks;
//                
//                baxter_arm_t* left_arm, *right_arm;
//
//                std::vector< movable_body_plant_t* > objects; //List of the objects in the scene
//
//                util::space_t* object_space; //Space of the object
//                util::space_point_t* object_start; //Start state of the object.
//                
//                util::space_point_t* global_start_state; //The global start state that is read in from input
//
//
//            };
//        }
//    }
//}
//
//#endif //PRX_COORDINATION_MANIPULATION_HPP
