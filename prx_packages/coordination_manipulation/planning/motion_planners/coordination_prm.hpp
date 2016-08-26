///**
// * @file coordination_prm_query.hpp
// * 
// * @copyright Software License Agreement (BSD License)
// * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick  
// * All Rights Reserved.
// * For a full description see the file named LICENSE.
// * 
// * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
// * 
// * Email: pracsys@googlegroups.com
// */
//#pragma once
//
//#ifndef PRX_CPRM_HPP
//#define	PRX_CPRM_HPP
//
//#include "prx/utilities/definitions/sys_clock.hpp"
//#include "prx/utilities/definitions/defs.hpp"
//#include "prx/utilities/graph/undirected_graph.hpp"
//#include "prx/planning/motion_planners/prm_star/prm_star.hpp"
//#include "planning/motion_planners/coordination_astar.hpp"
//#include "planning/motion_planners/coordination_prm_query.hpp"
//
//namespace prx
//{
//    namespace packages
//    {
//        namespace coordination_manipulation
//        {
//            /**
//             * PRM star for generating coordination roadmaps for two manipulators
//             * 
//             * @author Andrew Kimmel
//             * 
//             */
//            class coordination_prm_t : public plan::prm_star_t
//            {
//
//              public:
//
//                coordination_prm_t();
//                virtual ~coordination_prm_t();
//
//                /** @copydoc motion_planner_t::init(const util::parameter_reader_t*) */
//                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);
//                
//                virtual void link_specification(plan::specification_t* new_spec);
//                
//                virtual void link_query(plan::query_t* new_query);
//                
//                virtual void setup();
//                
//                virtual void resolve_query();
//                
//                virtual void reset();
//
//              protected:
//
//                virtual void link_node_to_neighbors(util::undirected_vertex_index_t v, const std::vector< const util::abstract_node_t* >& neighbors);
//                                
//                virtual void valid_random_sample();
//                
//                virtual bool varying_velocity_steer(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& plan, sim::trajectory_t& traj, double& edge_dist, bool generate_trajectory = true);
//                virtual void get_arm_start_and_goal(unsigned start_point, unsigned goal_point, sim::plan_t* arm_plan, sim::state_t* start, 
//                                                   sim::state_t* goal, sim::plan_t& seed_plan );
//                virtual bool get_tuned_plan(const util::space_t* arm_state_space,const util::space_t* arm_control_space, double total_steps, 
//                                            sim::plan_t& untuned_plan, sim::plan_t& tuned_plan);
//                virtual void get_armcup_path(baxter::baxter_arm_t* arm, sim::state_t* arm_start_state,  sim::plan_t& arm_plan, const sim::state_t* cup_safe_state, const util::space_t* arm_state_space, 
//                                             const util::space_t* cup_space, const util::space_t* armcup_space,  sim::trajectory_t& armcup_path);
//                virtual void combine_tuned_plans(const sim::plan_t& left_plan, const sim::plan_t& right_plan, sim::plan_t& combined_plan);
//                virtual void combine_armcup_paths(sim::trajectory_t& left_armcup_path, sim::trajectory_t& right_armcup_path, 
//                                                 const util::space_t* left_armcup_space, const util::space_t* right_armcup_space, sim::trajectory_t& combined_path); 
//                
//                virtual void extract_astar_path(const std::deque<util::undirected_vertex_index_t>& path_vertices, sim::plan_t& full_plan, sim::plan_t& partial_plan, coordination_astar_t::coordination_heuristic_type_t& arm_bias);
//                
//                coordination_prm_query_t* prm_query;
//                
//                const util::space_t* full_arm_state_space, *full_arm_control_space;
//                sim::plan_t left_tuned_plan, left_untuned_plan;
//                sim::plan_t right_tuned_plan, right_untuned_plan;
//                sim::state_t* right_start, *right_goal, *left_start, *left_goal;
//                
//                
//                /** Coordination astar variables */
//                coordination_astar_t* coordination_astar;
//                sim::plan_t keep_plan;
//                sim::plan_t left_full_plan, left_partial_plan;
//                sim::plan_t right_full_plan, right_partial_plan;
//                double goal_left_arm, goal_right_arm;
//                double bias_left_percentage, bias_right_percentage;
//                std::vector<double> dice_rolls;
//                
//                double max_step;
//
//            };
//        }
//
//    }
//}
//
//#endif	// PRX_CPRM_HPP
//
