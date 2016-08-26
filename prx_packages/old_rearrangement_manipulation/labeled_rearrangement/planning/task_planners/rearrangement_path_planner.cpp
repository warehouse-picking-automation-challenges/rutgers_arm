/**
 * @file rearrangement_path_planner.cpp
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

#include "planning/task_planners/rearrangement_path_planner.hpp"

#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/goals/multiple_goal_states.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/simulation/plan.hpp"

 #include "simulation/manipulator_simulator.hpp"

#include "planning/modules/obstacle_aware_astar.hpp"
#include "rearrangement_primitive.hpp"
#include "rearrangement_path_planner.hpp"

#include <vector>
#include <pluginlib/class_list_macros.h>
#include <boost/range/adaptor/map.hpp>
#include <set>

#include <list>
#include <bits/stl_deque.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::labeled_rearrangement_manipulation::rearrangement_path_planner_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        using namespace manipulation;
        using namespace rearrangement_manipulation;

        namespace labeled_rearrangement_manipulation
        {

            rearrangement_path_planner_t::rearrangement_path_planner_t()
            {
                time_ends = false;
            }

            rearrangement_path_planner_t::~rearrangement_path_planner_t() { }

            void rearrangement_path_planner_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
            {
                rearrangement_primitive_t::init(reader, template_reader);
                apply_smoothing = parameters::get_attribute_as<bool>("apply_smoothing", reader, template_reader, false);
                sensing = parameters::get_attribute_as<bool>("sensing", reader, template_reader, false);
                is_motoman = parameters::get_attribute_as<bool>("is_motoman", reader, template_reader, false);
                store_trajectory = parameters::get_attribute_as<bool>("store_trajectory", reader, template_reader, false);
                IK_cheating = parameters::get_attribute_as<bool>("IK_cheating", reader, template_reader, false);
            }

            void rearrangement_path_planner_t::setup()
            {
                rearrangement_primitive_t::setup();
                context_flags active_flags(true, false);

                util::hash_t<std::string, context_flags> mappings;
                mappings[_manipulator->get_pathname()] = active_flags;
                mappings[_manipulator->get_pathname()].plannable = true;
                model->create_new_planning_context(pc_name_manip_collision_checking, mappings, active_flags); 

                tmp_path.link_space(manip_state_space);
                tmp_plan.link_control_space(manip_control_space);                
                smoothed_plan.link_control_space(manip_control_space); 
            }

            bool rearrangement_path_planner_t::solve()
            {

                foreach(path_part_t p, in_query->path_sequence)
                {
                    PRX_DEBUG_COLOR(p.print(), PRX_TEXT_GREEN);
                }

                if( apply_smoothing && in_query->path_sequence.size() > 1 )
                {
                    bool changed = true;
                    
                    // in_query->path_sequence.clear();
                    // in_query->path_sequence.resize(5);

                    // in_query->path_sequence[0].reset(0,3,7,0,0);
                    // in_query->path_sequence[1].reset(1,4,6,0,0);
                    // in_query->path_sequence[2].reset(2,5,2,0,0);
                    // in_query->path_sequence[3].reset(1,6,1,0,0);
                    // in_query->path_sequence[4].reset(0,7,0,0,0);

                    // in_query->path_sequence.clear();
                    // in_query->path_sequence.resize(8);

                    // in_query->path_sequence[0].reset(0,3,7,0,0);
                    // in_query->path_sequence[1].reset(1,4,6,0,0);
                    // in_query->path_sequence[2].reset(2,5,2,0,0);
                    // in_query->path_sequence[2].constraints.insert(0);
                    // in_query->path_sequence[2].constraints.insert(1);
                    // in_query->path_sequence[3].reset(1,6,0,0,0);                    
                    // in_query->path_sequence[4].reset(1,0,4,0,0);
                    // in_query->path_sequence[5].reset(0,7,6,0,0);
                    // in_query->path_sequence[6].reset(1,4,1,0,0);
                    // in_query->path_sequence[6].constraints.insert(0);
                    // in_query->path_sequence[7].reset(0,6,0,0,0);
                    
                    while( changed )
                    {
                        PRX_DEBUG_COLOR("Going to change " << print_parts(in_query->path_sequence), PRX_TEXT_GREEN);
                        changed = bring_parts_together(in_query->path_sequence);
                        PRX_DEBUG_COLOR("Changed " << print_parts(in_query->path_sequence), PRX_TEXT_BLUE);
                        changed = remove_same_objects(in_query->path_sequence) || changed;
                        // PRX_DEBUG_COLOR("---------------------------------------------", PRX_TEXT_GREEN);
                        // PRX_DEBUG_COLOR("After Remove  " << print_parts(in_query->path_sequence), PRX_TEXT_BROWN);
                        // PRX_ASSERT(false);
                    }

                    // in_query->path_sequence.clear();
                    // in_query->path_sequence.insert(in_query->path_sequence.begin(), parts.begin(), parts.end());
                }

                PRX_DEBUG_COLOR("After smoothing: " << print_parts(in_query->path_sequence), PRX_TEXT_BROWN);

                
                if(IK_cheating)
                    combine_path_with_IK(in_query->plan, in_query->path_sequence, in_query->initial_poses_ids);
                else
                    combine_path(in_query->plan, in_query->path_sequence, in_query->initial_poses_ids);
                stats->path_length = in_query->plan.length();

                if(store_trajectory)
                {
                    std::string traj_file = prx_output_dir + "trajectory.txt";
                    std::ofstream fout(traj_file.c_str());
                    trajectory_t traj;
                    model->use_context(pc_name_manipulator_only);
                    traj.link_space(manip_state_space);
                    local_planner->link_state_space(manip_state_space);
                    local_planner->propagate(safe_state, in_query->plan, traj );
                    traj.save_to_stream(fout);
                    fout.close();
                }
#ifndef NDEBUG
                //                if( !validate_full_path() )
                //                    PRX_DEBUG_COLOR("The path is not correct!", PRX_TEXT_RED);

#endif
                return true;
            }

            bool rearrangement_path_planner_t::combine_path(plan_t& plan, std::deque<path_part_t>& parts, std::vector<unsigned> a_curr)
            {
                PRX_DEBUG_COLOR("", PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR("--------------------------", PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR("---- COMBINE PATH " << parts.size() << " ----", PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("--------------------------", PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR("", PRX_TEXT_GREEN);

                transit_constraints.clear();
                transit_constraints.insert(a_curr.begin(), a_curr.end());
                transfer_constraints = transit_constraints;

                transit_astar->set_minimum_conflict(false);
                transit_astar->set_shortest_path_flag(true);
                transfer_astar->set_minimum_conflict(false);
                transfer_astar->set_shortest_path_flag(true);

                state_t* start_state = safe_state;
                control_t* ctrl = manip_control_space->alloc_point();
                for( unsigned i = 0; i < parts.size(); ++i )
                {
                    PRX_DEBUG_COLOR(i << ") part: " << parts[i].print(), PRX_TEXT_GREEN);
                    if( parts[i].pose_from != parts[i].pose_to )
                    {
                        transit_query->clear();
                        transit_query->copy_start(start_state);
                        if(sensing)
                            transit_query_goal->copy_goal_state(poses_set[parts[i].pose_from].retracted_set[parts[i].index_from]);
                        else
                            transit_query_goal->copy_goal_state(poses_set[parts[i].pose_from].ungrasped_set[parts[i].index_from]);
                        manip_tp->link_query(transit_query);
                        manip_tp->resolve_query();
                        if( !transit_query->found_path )
                        {
                            plan.clear();
                            manip_control_space->free_point(ctrl);
                            PRX_DEBUG_COLOR("Approach to pose: " << parts[i].pose_from << " failed", PRX_TEXT_RED);                            
                            return false;
                        }
                        // PRX_DEBUG_COLOR("The transit plan: " << transit_query->plans[0]->print(), PRX_TEXT_GREEN);
                        plan += (*transit_query->plans[0]);
                        if(is_motoman)
                        {
                            manip_control_space->zero(ctrl);
                            ctrl->memory.back() = 1;
                            plan.copy_onto_back(ctrl, 2*simulation::simulation_step);
                        }
                        if(sensing)
                            plan.copy_onto_back(transit_query->plans[0]->back().control,5);


                        transfer_constraints.erase(parts[i].pose_from);
                        transit_constraints.erase(parts[i].pose_from);
                        PRX_ASSERT(transfer_constraints.count(parts[i].pose_to) == 0);
                        transfer_query->clear();
                        if(sensing)
                            transfer_query->copy_start(poses_set[parts[i].pose_from].grasped_retracted_set[parts[i].index_from]);
                        else
                            transfer_query->copy_start(poses_set[parts[i].pose_from].grasped_set[parts[i].index_from]);
                        transfer_query_goal->copy_goal_state(poses_set[parts[i].pose_to].grasped_set[parts[i].index_to]);
                        manip_tp->link_query(transfer_query);
                        manip_tp->resolve_query();
                        if( !transfer_query->found_path )
                        {
                            PRX_DEBUG_COLOR("Transfer to pose: " << parts[i].pose_to << " failed", PRX_TEXT_RED);
                            plan.clear();
                            manip_control_space->free_point(ctrl);
                            return false;
                        }
                        // PRX_DEBUG_COLOR("The Transfer plan: " << transfer_query->plans[0]->print(), PRX_TEXT_GREEN);
                        plan += (*transfer_query->plans[0]);
                        if(is_motoman)
                        {
                            manip_control_space->zero(ctrl);                            
                            plan.copy_onto_back(ctrl, 2*simulation::simulation_step);
                        }
                        transfer_constraints.insert(parts[i].pose_to);
                        transit_constraints.insert(parts[i].pose_to);

                        start_state = poses_set[parts[i].pose_to].ungrasped_set[parts[i].index_to];
                    }
                }

                transit_query->clear();
                transit_query->copy_start(start_state);
                transit_query_goal->copy_goal_state(safe_state);
                manip_tp->link_query(transit_query);
                manip_tp->resolve_query();
                if( !transit_query->found_path )
                {
                    plan.clear();
                    manip_control_space->free_point(ctrl);
                    PRX_DEBUG_COLOR("Back to safe failed", PRX_TEXT_RED);
                    return false;
                }
                plan += (*transit_query->plans[0]);

                if(is_motoman)
                {
                    manip_control_space->zero(ctrl);
                    plan.copy_onto_back(ctrl, simulation::simulation_step);
                }
                manip_control_space->free_point(ctrl);
                return true;
            }

            // bool rearrangement_path_planner_t::combine_path_with_IK(plan_t& plan, std::deque<path_part_t>& parts, std::vector<unsigned> a_curr)
            // {
            //     PRX_DEBUG_COLOR("", PRX_TEXT_GREEN);
            //     PRX_DEBUG_COLOR("--------------------------", PRX_TEXT_GREEN);
            //     PRX_DEBUG_COLOR("---- COMBINE PATH " << parts.size() << " ----", PRX_TEXT_CYAN);
            //     PRX_DEBUG_COLOR("--------------------------", PRX_TEXT_GREEN);
            //     PRX_DEBUG_COLOR("", PRX_TEXT_GREEN);

            //     transit_constraints.clear();
            //     transit_constraints.insert(a_curr.begin(), a_curr.end());
            //     transfer_constraints = transit_constraints;

            //     transit_astar->set_minimum_conflict(false);
            //     transit_astar->set_shortest_path_flag(true);
            //     transfer_astar->set_minimum_conflict(false);
            //     transfer_astar->set_shortest_path_flag(true);

            //     state_t* start_state = safe_state;
            //     state_t* next_retracted_state;
            //     state_t* next_manip_retracted_state = manip_state_space->alloc_point();
            //     state_t* this_manip_retracted_state = manip_state_space->alloc_point();
            //     state_t* next_grapsed_retracted_state;
            //     control_t* ctrl = manip_control_space->alloc_point();
            //     config_t tmp_start_config;
            //     config_t tmp_end_config;
            //     plan_t tmp_plan;
            //     tmp_plan.link_control_space(manip_control_space);
            //     tmp_path.link_space(manip_state_space);
            //     bool from_start = true;
            //     unsigned number_transit_IKs = 0;
            //     unsigned number_transfer_IKs = 0;
            //     for( unsigned i = 0; i < parts.size(); ++i )
            //     {
            //         PRX_DEBUG_COLOR(i << ") part: " << parts[i].print(), PRX_TEXT_GREEN);
            //         if( parts[i].pose_from != parts[i].pose_to )
            //         {
            //             //it is the retracted position. 
            //             next_retracted_state = poses_set[parts[i].pose_from].retracted_set[parts[i].index_from];
            //             manip_state_space->copy_from_point(next_retracted_state);
            //             _manipulator->get_end_effector_configuration(tmp_end_config);
            //             manip_state_space->copy_from_point(start_state);
            //             _manipulator->get_end_effector_configuration(tmp_start_config);

            //             transit_query->clear();
            //             tmp_plan.clear();
            //             if(!from_start && move_with_IK(tmp_plan, tmp_start_config, tmp_end_config, start_state, next_retracted_state,false))
            //             {
            //                 number_transit_IKs++;
            //                 plan += tmp_plan;                            
            //                 transit_query->copy_start(next_retracted_state);
            //                 PRX_DEBUG_COLOR("IK for transit was ok!", PRX_TEXT_GREEN);
            //             }
            //             else
            //             {
            //                 PRX_DEBUG_COLOR("No Ik  for transit so we are doing local_planner",PRX_TEXT_CYAN);
            //                 transit_query->copy_start(start_state);
            //                 from_start = false;
            //             }

            //             transit_query_goal->copy_goal_state(poses_set[parts[i].pose_from].ungrasped_set[parts[i].index_from]);
            //             manip_tp->link_query(transit_query);
            //             manip_tp->resolve_query();
            //             if( !transit_query->found_path )
            //             {
            //                 plan.clear();
            //                 manip_state_space->free_point(next_manip_retracted_state);
            //                 manip_state_space->free_point(this_manip_retracted_state);
            //                 manip_control_space->free_point(ctrl);
            //                 PRX_DEBUG_COLOR("Approach to pose: " << parts[i].pose_from << " failed", PRX_TEXT_RED);                            
            //                 return false;
            //             }
            //             // PRX_DEBUG_COLOR("The transit plan: " << transit_query->plans[0]->print(), PRX_TEXT_GREEN);
            //             plan += (*transit_query->plans[0]);

            //             if(is_motoman)
            //             {
            //                 manip_control_space->zero(ctrl);
            //                 ctrl->memory.back() = 1;
            //                 plan.copy_onto_back(ctrl, 2*simulation::simulation_step);
            //             }


            //             transfer_constraints.erase(parts[i].pose_from);
            //             transit_constraints.erase(parts[i].pose_from);
            //             PRX_ASSERT(transfer_constraints.count(parts[i].pose_to) == 0);

            //             next_grapsed_retracted_state = poses_set[parts[i].pose_to].grasped_retracted_set[parts[i].index_to];
            //             mo_space->copy_from_point(next_grapsed_retracted_state);     
            //             manip_state_space->copy_to_point(next_manip_retracted_state);
            //             _manipulator->get_end_effector_configuration(tmp_end_config);
            //             mo_space->copy_from_point(poses_set[parts[i].pose_from].grasped_retracted_set[parts[i].index_from]);
            //             _manipulator->get_end_effector_configuration(tmp_start_config);
            //             manip_state_space->copy_to_point(this_manip_retracted_state);
                        
            //             transfer_query->clear();
            //             tmp_plan.clear();
            //             if( move_with_IK(tmp_plan, tmp_start_config, tmp_end_config, this_manip_retracted_state, next_manip_retracted_state,true))
            //             {
            //                 number_transfer_IKs++;
            //                 PRX_DEBUG_COLOR("Yes Ik for transfer part.",PRX_TEXT_GREEN);
            //                 //Have to raise the hand.
            //                 transfer_query->copy_start(poses_set[parts[i].pose_from].grasped_set[parts[i].index_from]); 
            //                 transfer_query_goal->copy_goal_state(poses_set[parts[i].pose_from].grasped_retracted_set[parts[i].index_from]);
            //                 manip_tp->link_query(transfer_query);
            //                 manip_tp->resolve_query();
            //                 if( !transfer_query->found_path )
            //                 {
            //                     PRX_DEBUG_COLOR("Transfer from pose: " << parts[i].pose_from << " failed", PRX_TEXT_RED);
            //                     plan.clear();
            //                     manip_state_space->free_point(next_manip_retracted_state);
            //                     manip_state_space->free_point(this_manip_retracted_state);
            //                     manip_control_space->free_point(ctrl);
            //                     return false;
            //                 }
            //                 plan += (*transfer_query->plans[0]);
            //                 plan += tmp_plan;
            //                 transfer_query->clear();
            //                 transfer_query->copy_start(poses_set[parts[i].pose_to].grasped_retracted_set[parts[i].index_to]); 
            //             }
            //             else
            //             {
            //                 PRX_DEBUG_COLOR("No Ik  for transfer so we are doing local_planner",PRX_TEXT_CYAN);
            //                 transfer_query->copy_start(poses_set[parts[i].pose_from].grasped_set[parts[i].index_from]);
            //             }

            //             transfer_query_goal->copy_goal_state(poses_set[parts[i].pose_to].grasped_set[parts[i].index_to]);
            //             manip_tp->link_query(transfer_query);
            //             manip_tp->resolve_query();
            //             if( !transfer_query->found_path )
            //             {
            //                 PRX_DEBUG_COLOR("Transfer to pose: " << parts[i].pose_to << " failed", PRX_TEXT_RED);
            //                 plan.clear();
            //                 manip_state_space->free_point(next_manip_retracted_state);
            //                 manip_state_space->free_point(this_manip_retracted_state);
            //                 manip_control_space->free_point(ctrl);
            //                 return false;
            //             }
            //             // PRX_DEBUG_COLOR("The Transfer plan: " << transfer_query->plans[0]->print(), PRX_TEXT_GREEN);
            //             plan += (*transfer_query->plans[0]);

            //             if(is_motoman)
            //             {
            //                 manip_control_space->zero(ctrl);                            
            //                 plan.copy_onto_back(ctrl, 2*simulation::simulation_step);
            //             }
            //             transfer_constraints.insert(parts[i].pose_to);
            //             transit_constraints.insert(parts[i].pose_to);

            //             //Rise the hand after the place
            //             PRX_DEBUG_COLOR("Rise the hand after placing the object.",PRX_TEXT_BLUE);
            //             PRX_DEBUG_COLOR("starT: " << manip_state_space->print_point(poses_set[parts[i].pose_to].ungrasped_set[parts[i].index_to],5),PRX_TEXT_GREEN);
            //             PRX_DEBUG_COLOR("end  : " << manip_state_space->print_point(poses_set[parts[i].pose_to].retracted_set[parts[i].index_to],5),PRX_TEXT_CYAN);
            //             manip_state_space->copy_from_point(poses_set[parts[i].pose_to].ungrasped_set[parts[i].index_to]);
            //             transit_query->clear();
            //             transit_query->copy_start(poses_set[parts[i].pose_to].ungrasped_set[parts[i].index_to]);
            //             transit_query_goal->copy_goal_state(poses_set[parts[i].pose_to].retracted_set[parts[i].index_to]);
            //             manip_tp->link_query(transit_query);
            //             manip_tp->resolve_query();
            //             if( !transit_query->found_path )
            //             {
            //                 plan.clear();
            //                 manip_state_space->free_point(next_manip_retracted_state);
            //                 manip_state_space->free_point(this_manip_retracted_state);
            //                 manip_control_space->free_point(ctrl);
            //                 PRX_DEBUG_COLOR("Approach to pose: " << parts[i].pose_from << " failed", PRX_TEXT_RED);                            
            //                 return false;
            //             }
            //             // PRX_DEBUG_COLOR("The transit plan: " << transit_query->plans[0]->print(), PRX_TEXT_GREEN);
            //             plan += (*transit_query->plans[0]);

            //             start_state = poses_set[parts[i].pose_to].retracted_set[parts[i].index_to];
            //         }
            //     }

            //     PRX_DEBUG_COLOR("Back to base!!!!!!", PRX_TEXT_LIGHTGRAY);
            //     transit_query->clear();
            //     transit_query->copy_start(start_state);
            //     transit_query_goal->copy_goal_state(safe_state);
            //     manip_tp->link_query(transit_query);
            //     manip_tp->resolve_query();
            //     if( !transit_query->found_path )
            //     {
            //         plan.clear();
            //         manip_state_space->free_point(next_manip_retracted_state);
            //         manip_state_space->free_point(this_manip_retracted_state);
            //         manip_control_space->free_point(ctrl);
            //         PRX_DEBUG_COLOR("Back to safe failed", PRX_TEXT_RED);
            //         return false;
            //     }
            //     plan += (*transit_query->plans[0]);

            //     if(is_motoman)
            //     {
            //         manip_control_space->zero(ctrl);
            //         plan.copy_onto_back(ctrl, simulation::simulation_step);
            //     }
            //     manip_state_space->free_point(next_manip_retracted_state);
            //     manip_state_space->free_point(this_manip_retracted_state);
            //     manip_control_space->free_point(ctrl);

            //     PRX_DEBUG_COLOR("NoTransit IK: " << number_transit_IKs << "   NoTranfer IK: " << number_transfer_IKs,PRX_TEXT_LIGHTGRAY);
            //     PRX_ASSERT(false);
            //     return true;
            // }

            bool rearrangement_path_planner_t::combine_path_with_basic_smoothing(plan_t& plan, std::deque<path_part_t>& parts, std::vector<unsigned> a_curr)
            {
                PRX_DEBUG_COLOR("", PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR("--------------------------", PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR("---- COMBINE PATH " << parts.size() << " ----", PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("--------------------------", PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR("", PRX_TEXT_GREEN);

                transit_constraints.clear();
                transit_constraints.insert(a_curr.begin(), a_curr.end());
                transfer_constraints = transit_constraints;

                transit_astar->set_minimum_conflict(false);
                transit_astar->set_shortest_path_flag(true);
                transfer_astar->set_minimum_conflict(false);
                transfer_astar->set_shortest_path_flag(true);

                state_t* start_state = safe_state;
                state_t* next_retracted_state;
                state_t* next_manip_retracted_state = manip_state_space->alloc_point();
                state_t* this_manip_retracted_state = manip_state_space->alloc_point();
                state_t* next_grapsed_retracted_state;
                control_t* ctrl = manip_control_space->alloc_point();
                config_t tmp_start_config;
                config_t tmp_end_config;                
                bool from_start = true;
                unsigned number_transit_IKs = 0;
                unsigned number_transfer_IKs = 0;
                for( unsigned i = 0; i < parts.size(); ++i )
                {
                    PRX_DEBUG_COLOR(i << ") part: " << parts[i].print(), PRX_TEXT_GREEN);
                    if( parts[i].pose_from != parts[i].pose_to )
                    {
                        //it is the retracted position. 
                        next_retracted_state = poses_set[parts[i].pose_from].retracted_set[parts[i].index_from];

                        transit_query->clear();
                        transit_query_goal->copy_goal_state(next_retracted_state);                         
                        manip_tp->link_query(transit_query);
                        manip_tp->resolve_query();
                        if( !transit_query->found_path )
                        {
                            plan.clear();
                            manip_state_space->free_point(next_manip_retracted_state);
                            manip_state_space->free_point(this_manip_retracted_state);
                            manip_control_space->free_point(ctrl);
                            PRX_DEBUG_COLOR("Approach to pose: " << parts[i].pose_from << " failed", PRX_TEXT_RED);                            
                            return false;
                        }
                        // PRX_DEBUG_COLOR("The transit plan: " << transit_query->plans[0]->print(), PRX_TEXT_GREEN);
                        plan += (*transit_query->plans[0]);

                        transit_query_goal->copy_goal_state(poses_set[parts[i].pose_from].ungrasped_set[parts[i].index_from]);


                        if(move_with_IK(tmp_plan, tmp_start_config, tmp_end_config, start_state, next_retracted_state,false) || move_with_local_planner(tmp_plan, start_state, next_retracted_state))
                        {
                            number_transit_IKs++;
                            plan += tmp_plan;                            
                            transit_query->copy_start(next_retracted_state);
                            PRX_DEBUG_COLOR("IK for transit was ok!", PRX_TEXT_GREEN);
                        }
                        else
                        {
                            PRX_DEBUG_COLOR("No Ik  for transit so we are doing local_planner",PRX_TEXT_CYAN);
                            transit_query->copy_start(start_state);
                            from_start = false;
                        }

                        transit_query_goal->copy_goal_state(poses_set[parts[i].pose_from].ungrasped_set[parts[i].index_from]);
                        manip_tp->link_query(transit_query);
                        manip_tp->resolve_query();
                        if( !transit_query->found_path )
                        {
                            plan.clear();
                            manip_state_space->free_point(next_manip_retracted_state);
                            manip_state_space->free_point(this_manip_retracted_state);
                            manip_control_space->free_point(ctrl);
                            PRX_DEBUG_COLOR("Approach to pose: " << parts[i].pose_from << " failed", PRX_TEXT_RED);                            
                            return false;
                        }
                        // PRX_DEBUG_COLOR("The transit plan: " << transit_query->plans[0]->print(), PRX_TEXT_GREEN);
                        plan += (*transit_query->plans[0]);

                        if(is_motoman)
                        {
                            manip_control_space->zero(ctrl);
                            ctrl->memory.back() = 1;
                            plan.copy_onto_back(ctrl, 2*simulation::simulation_step);
                        }


                        transfer_constraints.erase(parts[i].pose_from);
                        transit_constraints.erase(parts[i].pose_from);
                        PRX_ASSERT(transfer_constraints.count(parts[i].pose_to) == 0);

                        next_grapsed_retracted_state = poses_set[parts[i].pose_to].grasped_retracted_set[parts[i].index_to];
                        mo_space->copy_from_point(next_grapsed_retracted_state);     
                        manip_state_space->copy_to_point(next_manip_retracted_state);
                        _manipulator->get_end_effector_configuration(tmp_end_config);
                        mo_space->copy_from_point(poses_set[parts[i].pose_from].grasped_retracted_set[parts[i].index_from]);
                        _manipulator->get_end_effector_configuration(tmp_start_config);
                        manip_state_space->copy_to_point(this_manip_retracted_state);
                        
                        transfer_query->clear();
                        tmp_plan.clear();
                        if( move_with_IK(tmp_plan, tmp_start_config, tmp_end_config, this_manip_retracted_state, next_manip_retracted_state,true) || move_with_local_planner(tmp_plan,this_manip_retracted_state,next_manip_retracted_state))
                        {
                            number_transfer_IKs++;
                            PRX_DEBUG_COLOR("Yes Ik for transfer part.",PRX_TEXT_GREEN);
                            //Have to raise the hand.
                            transfer_query->copy_start(poses_set[parts[i].pose_from].grasped_set[parts[i].index_from]); 
                            transfer_query_goal->copy_goal_state(poses_set[parts[i].pose_from].grasped_retracted_set[parts[i].index_from]);
                            manip_tp->link_query(transfer_query);
                            manip_tp->resolve_query();
                            if( !transfer_query->found_path )
                            {
                                PRX_DEBUG_COLOR("Transfer from pose: " << parts[i].pose_from << " failed", PRX_TEXT_RED);
                                plan.clear();
                                manip_state_space->free_point(next_manip_retracted_state);
                                manip_state_space->free_point(this_manip_retracted_state);
                                manip_control_space->free_point(ctrl);
                                return false;
                            }
                            plan += (*transfer_query->plans[0]);
                            plan += tmp_plan;
                            transfer_query->clear();
                            transfer_query->copy_start(poses_set[parts[i].pose_to].grasped_retracted_set[parts[i].index_to]); 
                        }
                        else
                        {
                            PRX_DEBUG_COLOR("No Ik  for transfer so we are doing local_planner",PRX_TEXT_CYAN);
                            transfer_query->copy_start(poses_set[parts[i].pose_from].grasped_set[parts[i].index_from]);
                        }

                        transfer_query_goal->copy_goal_state(poses_set[parts[i].pose_to].grasped_set[parts[i].index_to]);
                        manip_tp->link_query(transfer_query);
                        manip_tp->resolve_query();
                        if( !transfer_query->found_path )
                        {
                            PRX_DEBUG_COLOR("Transfer to pose: " << parts[i].pose_to << " failed", PRX_TEXT_RED);
                            plan.clear();
                            manip_state_space->free_point(next_manip_retracted_state);
                            manip_state_space->free_point(this_manip_retracted_state);
                            manip_control_space->free_point(ctrl);
                            return false;
                        }
                        // PRX_DEBUG_COLOR("The Transfer plan: " << transfer_query->plans[0]->print(), PRX_TEXT_GREEN);
                        plan += (*transfer_query->plans[0]);

                        if(is_motoman)
                        {
                            manip_control_space->zero(ctrl);                            
                            plan.copy_onto_back(ctrl, 2*simulation::simulation_step);
                        }
                        transfer_constraints.insert(parts[i].pose_to);
                        transit_constraints.insert(parts[i].pose_to);

                        //Rise the hand after the place
                        PRX_DEBUG_COLOR("Rise the hand after placing the object.",PRX_TEXT_BLUE);
                        PRX_DEBUG_COLOR("starT: " << manip_state_space->print_point(poses_set[parts[i].pose_to].ungrasped_set[parts[i].index_to],5),PRX_TEXT_GREEN);
                        PRX_DEBUG_COLOR("end  : " << manip_state_space->print_point(poses_set[parts[i].pose_to].retracted_set[parts[i].index_to],5),PRX_TEXT_CYAN);
                        manip_state_space->copy_from_point(poses_set[parts[i].pose_to].ungrasped_set[parts[i].index_to]);
                        transit_query->clear();
                        transit_query->copy_start(poses_set[parts[i].pose_to].ungrasped_set[parts[i].index_to]);
                        transit_query_goal->copy_goal_state(poses_set[parts[i].pose_to].retracted_set[parts[i].index_to]);
                        manip_tp->link_query(transit_query);
                        manip_tp->resolve_query();
                        if( !transit_query->found_path )
                        {
                            plan.clear();
                            manip_state_space->free_point(next_manip_retracted_state);
                            manip_state_space->free_point(this_manip_retracted_state);
                            manip_control_space->free_point(ctrl);
                            PRX_DEBUG_COLOR("Approach to pose: " << parts[i].pose_from << " failed", PRX_TEXT_RED);                            
                            return false;
                        }
                        // PRX_DEBUG_COLOR("The transit plan: " << transit_query->plans[0]->print(), PRX_TEXT_GREEN);
                        plan += (*transit_query->plans[0]);

                        start_state = poses_set[parts[i].pose_to].retracted_set[parts[i].index_to];
                    }
                }

                PRX_DEBUG_COLOR("Back to base!!!!!!", PRX_TEXT_LIGHTGRAY);
                transit_query->clear();
                transit_query->copy_start(start_state);
                transit_query_goal->copy_goal_state(safe_state);
                manip_tp->link_query(transit_query);
                manip_tp->resolve_query();
                if( !transit_query->found_path )
                {
                    plan.clear();
                    manip_state_space->free_point(next_manip_retracted_state);
                    manip_state_space->free_point(this_manip_retracted_state);
                    manip_control_space->free_point(ctrl);
                    PRX_DEBUG_COLOR("Back to safe failed", PRX_TEXT_RED);
                    return false;
                }
                plan += (*transit_query->plans[0]);

                if(is_motoman)
                {
                    manip_control_space->zero(ctrl);
                    plan.copy_onto_back(ctrl, simulation::simulation_step);
                }
                manip_state_space->free_point(next_manip_retracted_state);
                manip_state_space->free_point(this_manip_retracted_state);
                manip_control_space->free_point(ctrl);

                PRX_DEBUG_COLOR("NoTransit IK: " << number_transit_IKs << "   NoTranfer IK: " << number_transfer_IKs,PRX_TEXT_LIGHTGRAY);
                PRX_ASSERT(false);
                return true;
            }

            //Tries to connect the retracted points with IKs or local planners.
            bool rearrangement_path_planner_t::combine_path_with_IK(plan_t& plan, std::deque<path_part_t>& parts, std::vector<unsigned> a_curr)
            {
                PRX_DEBUG_COLOR("", PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR("--------------------------", PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR("---- COMBINE PATH " << parts.size() << " ----", PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("--------------------------", PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR("", PRX_TEXT_GREEN);

                transit_constraints.clear();
                transit_constraints.insert(a_curr.begin(), a_curr.end());
                transfer_constraints = transit_constraints;

                transit_astar->set_minimum_conflict(false);
                transit_astar->set_shortest_path_flag(true);
                transfer_astar->set_minimum_conflict(false);
                transfer_astar->set_shortest_path_flag(true);

                state_t* start_state = safe_state;
                state_t* next_retracted_state;
                state_t* next_manip_retracted_state = manip_state_space->alloc_point();
                state_t* this_manip_retracted_state = manip_state_space->alloc_point();
                state_t* next_grapsed_retracted_state;
                control_t* ctrl = manip_control_space->alloc_point();
                config_t tmp_start_config;
                config_t tmp_end_config;                
                bool from_start = true;
                unsigned number_transit_IKs = 0;
                unsigned number_transfer_IKs = 0;
                for( unsigned i = 0; i < parts.size(); ++i )
                {
                    PRX_DEBUG_COLOR(i << ") part: " << parts[i].print(), PRX_TEXT_GREEN);
                    if( parts[i].pose_from != parts[i].pose_to )
                    {
                        //it is the retracted position. 
                        next_retracted_state = poses_set[parts[i].pose_from].retracted_set[parts[i].index_from];
                        manip_state_space->copy_from_point(next_retracted_state);
                        _manipulator->get_end_effector_configuration(tmp_end_config);
                        manip_state_space->copy_from_point(start_state);
                        _manipulator->get_end_effector_configuration(tmp_start_config);

                        transit_query->clear();
                        tmp_plan.clear();
                        if(move_with_IK(tmp_plan, tmp_start_config, tmp_end_config, start_state, next_retracted_state,false) || move_with_local_planner(tmp_plan, start_state, next_retracted_state))
                        {
                            number_transit_IKs++;
                            plan += tmp_plan;                            
                            transit_query->copy_start(next_retracted_state);
                            PRX_DEBUG_COLOR("IK for transit was ok!", PRX_TEXT_GREEN);
                        }
                        else
                        {
                            PRX_DEBUG_COLOR("No Ik  for transit so we are doing local_planner",PRX_TEXT_CYAN);
                            transit_query->copy_start(start_state);
                            from_start = false;
                        }

                        transit_query_goal->copy_goal_state(poses_set[parts[i].pose_from].ungrasped_set[parts[i].index_from]);
                        manip_tp->link_query(transit_query);
                        manip_tp->resolve_query();
                        if( !transit_query->found_path )
                        {
                            plan.clear();
                            manip_state_space->free_point(next_manip_retracted_state);
                            manip_state_space->free_point(this_manip_retracted_state);
                            manip_control_space->free_point(ctrl);
                            PRX_DEBUG_COLOR("Approach to pose: " << parts[i].pose_from << " failed", PRX_TEXT_RED);                            
                            return false;
                        }
                        // PRX_DEBUG_COLOR("The transit plan: " << transit_query->plans[0]->print(), PRX_TEXT_GREEN);
                        plan += (*transit_query->plans[0]);

                        if(is_motoman)
                        {
                            manip_control_space->zero(ctrl);
                            ctrl->memory.back() = 1;
                            plan.copy_onto_back(ctrl, 2*simulation::simulation_step);
                        }


                        transfer_constraints.erase(parts[i].pose_from);
                        transit_constraints.erase(parts[i].pose_from);
                        PRX_ASSERT(transfer_constraints.count(parts[i].pose_to) == 0);

                        next_grapsed_retracted_state = poses_set[parts[i].pose_to].grasped_retracted_set[parts[i].index_to];
                        mo_space->copy_from_point(next_grapsed_retracted_state);     
                        manip_state_space->copy_to_point(next_manip_retracted_state);
                        _manipulator->get_end_effector_configuration(tmp_end_config);
                        mo_space->copy_from_point(poses_set[parts[i].pose_from].grasped_retracted_set[parts[i].index_from]);
                        _manipulator->get_end_effector_configuration(tmp_start_config);
                        manip_state_space->copy_to_point(this_manip_retracted_state);
                        
                        transfer_query->clear();
                        tmp_plan.clear();
                        if( move_with_IK(tmp_plan, tmp_start_config, tmp_end_config, this_manip_retracted_state, next_manip_retracted_state,true) || move_with_local_planner(tmp_plan,this_manip_retracted_state,next_manip_retracted_state))
                        {
                            number_transfer_IKs++;
                            PRX_DEBUG_COLOR("Yes Ik for transfer part.",PRX_TEXT_GREEN);
                            //Have to raise the hand.
                            transfer_query->copy_start(poses_set[parts[i].pose_from].grasped_set[parts[i].index_from]); 
                            transfer_query_goal->copy_goal_state(poses_set[parts[i].pose_from].grasped_retracted_set[parts[i].index_from]);
                            manip_tp->link_query(transfer_query);
                            manip_tp->resolve_query();
                            if( !transfer_query->found_path )
                            {
                                PRX_DEBUG_COLOR("Transfer from pose: " << parts[i].pose_from << " failed", PRX_TEXT_RED);
                                plan.clear();
                                manip_state_space->free_point(next_manip_retracted_state);
                                manip_state_space->free_point(this_manip_retracted_state);
                                manip_control_space->free_point(ctrl);
                                return false;
                            }
                            plan += (*transfer_query->plans[0]);
                            plan += tmp_plan;
                            transfer_query->clear();
                            transfer_query->copy_start(poses_set[parts[i].pose_to].grasped_retracted_set[parts[i].index_to]); 
                        }
                        else
                        {
                            PRX_DEBUG_COLOR("No Ik  for transfer so we are doing local_planner",PRX_TEXT_CYAN);
                            transfer_query->copy_start(poses_set[parts[i].pose_from].grasped_set[parts[i].index_from]);
                        }

                        transfer_query_goal->copy_goal_state(poses_set[parts[i].pose_to].grasped_set[parts[i].index_to]);
                        manip_tp->link_query(transfer_query);
                        manip_tp->resolve_query();
                        if( !transfer_query->found_path )
                        {
                            PRX_DEBUG_COLOR("Transfer to pose: " << parts[i].pose_to << " failed", PRX_TEXT_RED);
                            plan.clear();
                            manip_state_space->free_point(next_manip_retracted_state);
                            manip_state_space->free_point(this_manip_retracted_state);
                            manip_control_space->free_point(ctrl);
                            return false;
                        }
                        // PRX_DEBUG_COLOR("The Transfer plan: " << transfer_query->plans[0]->print(), PRX_TEXT_GREEN);
                        plan += (*transfer_query->plans[0]);

                        if(is_motoman)
                        {
                            manip_control_space->zero(ctrl);                            
                            plan.copy_onto_back(ctrl, 2*simulation::simulation_step);
                        }
                        transfer_constraints.insert(parts[i].pose_to);
                        transit_constraints.insert(parts[i].pose_to);

                        //Rise the hand after the place
                        PRX_DEBUG_COLOR("Rise the hand after placing the object.",PRX_TEXT_BLUE);
                        PRX_DEBUG_COLOR("starT: " << manip_state_space->print_point(poses_set[parts[i].pose_to].ungrasped_set[parts[i].index_to],5),PRX_TEXT_GREEN);
                        PRX_DEBUG_COLOR("end  : " << manip_state_space->print_point(poses_set[parts[i].pose_to].retracted_set[parts[i].index_to],5),PRX_TEXT_CYAN);
                        manip_state_space->copy_from_point(poses_set[parts[i].pose_to].ungrasped_set[parts[i].index_to]);
                        transit_query->clear();
                        transit_query->copy_start(poses_set[parts[i].pose_to].ungrasped_set[parts[i].index_to]);
                        transit_query_goal->copy_goal_state(poses_set[parts[i].pose_to].retracted_set[parts[i].index_to]);
                        manip_tp->link_query(transit_query);
                        manip_tp->resolve_query();
                        if( !transit_query->found_path )
                        {
                            plan.clear();
                            manip_state_space->free_point(next_manip_retracted_state);
                            manip_state_space->free_point(this_manip_retracted_state);
                            manip_control_space->free_point(ctrl);
                            PRX_DEBUG_COLOR("Approach to pose: " << parts[i].pose_from << " failed", PRX_TEXT_RED);                            
                            return false;
                        }
                        // PRX_DEBUG_COLOR("The transit plan: " << transit_query->plans[0]->print(), PRX_TEXT_GREEN);
                        plan += (*transit_query->plans[0]);

                        start_state = poses_set[parts[i].pose_to].retracted_set[parts[i].index_to];
                    }
                }

                PRX_DEBUG_COLOR("Back to base!!!!!!", PRX_TEXT_LIGHTGRAY);
                transit_query->clear();
                transit_query->copy_start(start_state);
                transit_query_goal->copy_goal_state(safe_state);
                manip_tp->link_query(transit_query);
                manip_tp->resolve_query();
                if( !transit_query->found_path )
                {
                    plan.clear();
                    manip_state_space->free_point(next_manip_retracted_state);
                    manip_state_space->free_point(this_manip_retracted_state);
                    manip_control_space->free_point(ctrl);
                    PRX_DEBUG_COLOR("Back to safe failed", PRX_TEXT_RED);
                    return false;
                }
                plan += (*transit_query->plans[0]);

                if(is_motoman)
                {
                    manip_control_space->zero(ctrl);
                    plan.copy_onto_back(ctrl, simulation::simulation_step);
                }
                manip_state_space->free_point(next_manip_retracted_state);
                manip_state_space->free_point(this_manip_retracted_state);
                manip_control_space->free_point(ctrl);

                PRX_DEBUG_COLOR("NoTransit IK: " << number_transit_IKs << "   NoTranfer IK: " << number_transfer_IKs,PRX_TEXT_LIGHTGRAY);
                PRX_ASSERT(false);
                return true;
            }

            bool rearrangement_path_planner_t::move_with_IK(plan_t& plan, const config_t& start_config, const config_t& end_config, const state_t* start_state, const state_t* goal_state, bool set_grasping)
            {
                std::string old_context = model->get_current_context();
                if(_manipulator->IK_steering(start_config, end_config, goal_state, plan, set_grasping) )
                {
                    model->use_context(pc_name_manip_collision_checking);
                    tmp_path.clear();
                    local_planner->propagate(start_state, plan, tmp_path);

                    if( tmp_path.size() != 0 && validity_checker->is_valid(tmp_path) )
                    {
                        model->use_context(old_context);
                        return true;
                    }
                }
                plan.clear();
                model->use_context(old_context);
                return false;
            }

            bool rearrangement_path_planner_t::move_with_local_planner(plan_t& plan, const state_t* start, const state_t* goal)
            {
                std::string old_context = model->get_current_context();
                model->use_context(pc_name_manip_collision_checking);
                manip_state_space->copy_from_point(start);                                
                ((manipulation_simulator_t*)model->get_simulator())->compute_relative_config();
                tmp_path.clear();
                local_planner->steer(start, goal, plan, tmp_path);
                if( tmp_path.size() != 0 && validity_checker->is_valid(tmp_path) )
                {
                    model->use_context(old_context);
                    return true;
                }
                plan.clear();
                model->use_context(old_context);
                return false;
            }

            bool rearrangement_path_planner_t::basic_smoothing(plan_t& plan, const state_t* start, const state_t* goal)
            {
                bool changed_once = false;
                bool changes = true;                
                while(changes)
                {
                    tmp_path.clear();
                    local_planner->steer(start, goal, plan, tmp_path);
                    changes = smooth_forward(plan,start,tmp_path);
                    if(changes)
                    {
                        tmp_path.clear();
                        local_planner->steer(start, goal, plan, tmp_path);
                    }
                    changes = changes || smooth_backward(plan,start,tmp_path);       
                    changed_once = changed_once || changes;             
                }
                return changed_once;
            }

            bool rearrangement_path_planner_t::smooth_forward(plan_t& plan, const state_t* start, trajectory_t& path)
            {
                smoothed_plan.clear();
                for(int i = tmp_path.size()-1; i>=1; --i)
                {
                    if(move_with_local_planner(smoothed_plan,start,tmp_path[i]))
                    {
                        double t = i * simulation::simulation_step;
                        int index = plan.get_index_at(t);
                        double t_plan = 0;
                        for(int j = 0; j<index-1; ++j)
                            t_plan += plan[j].duration;
                        PRX_ASSERT(t_plan != 0);
                        PRX_ASSERT(plan[index].duration > t - t_plan);
                        smoothed_plan.copy_onto_back(plan[index].control,plan[index].duration - (t-t_plan));
                        for(int j=index+1; j < plan.size(); ++j)
                        {
                            smoothed_plan.copy_onto_back(plan[j].control, plan[j].duration);
                        }
                        plan.clear();
                        plan = smoothed_plan;
                        return true;
                    }
                }
                return false;
            }

            bool rearrangement_path_planner_t::smooth_backward(plan_t& plan, const state_t* goal, trajectory_t& path)
            {
                smoothed_plan.clear();
                for(int i = 0; i<tmp_path.size()-2; ++i)
                {
                    if(move_with_local_planner(smoothed_plan,tmp_path[i],goal))
                    {
                        double t = i * simulation::simulation_step;
                        plan.trim(t);
                        plan += smoothed_plan;
                        return true;
                    }
                }
                return false;
            }

            // bool rearrangement_path_planner_t::smooth_with_IK(plan_t& plan, state_t* start, bool set_grasping)
            // {
            //     bool changes = true;
            //     plan_t tmp_plan;
            //     tmp_plan.link_control_space(manip_control_space);
            //     trajectory_t tmp_path;
            //     tmp_path.link_space(manip_state_space);
            //     local_planner->propagate(start, plan, tmp_path);
            //     config_t tmp_start_config, tmp_end_config;
            //     while(changes)
            //     {
            //         changes = false;
            //         for(unsigned i = 0; i< path.size()-1; ++i)
            //         {
            //             for(int j = path.size()-1; j>i; --j)
            //             {
            //                 manip_state_space->copy_from_point(path[j]);
            //                 _manipulator->get_end_effector_configuration(tmp_end_config);
            //                 manip_state_space->copy_from_point(path[i]);
            //                 _manipulator->get_end_effector_configuration(tmp_start_config);
            //                 if(move_with_IK(tmp_plan, tmp_start_config, tmp_end_config, path[i], path[j], set_grasping))
            //                 {
                                
            //                 }

            //             }
            //         }
            //         tmp_plan.clear();
            //         changes = move_with_IK()
            

            // }

            std::pair<bool, unsigned> rearrangement_path_planner_t::swap(std::deque<path_part_t>& parts, unsigned id, unsigned start_index)
            {
                // PRX_DEBUG_COLOR("swap will start from : " << start_index << " / " << parts.size(), PRX_TEXT_LIGHTGRAY);
                for( unsigned i = start_index; i < parts.size(); ++i)
                {
                    if(parts[i].id == id)
                    {
                        // PRX_DEBUG_COLOR("Got same id at place : " << i , PRX_TEXT_GREEN);
                        for(unsigned j = start_index; j < i; ++j)
                        {
                            if(parts[i].is_constrained_by(parts[j].pose_from) || parts[j].is_constrained_by(parts[i].pose_to) )
                            {
                                // PRX_DEBUG_COLOR("The part at place  " << j << "  is blocking us",PRX_TEXT_RED);
                                return std::make_pair(false,0);
                            }
                        }
                        // PRX_DEBUG_COLOR("Swapping from place " << i, PRX_TEXT_GREEN);
                        return std::make_pair(true,i);
                    }
                }
                // PRX_DEBUG_COLOR("No same id", PRX_TEXT_CYAN);
                return std::make_pair(false,0);
            }

            std::pair<bool, unsigned> rearrangement_path_planner_t::swap_backward(std::deque<path_part_t>& parts, unsigned id, unsigned start_index)
            {
                // PRX_DEBUG_COLOR("swap will start from : " << start_index << " / " << parts.size(), PRX_TEXT_LIGHTGRAY);
                for( int i = start_index; i >= 0; --i)
                {
                    if(parts[i].id == id)
                    {
                        // PRX_DEBUG_COLOR("Got same id at place : " << i , PRX_TEXT_GREEN);
                        for(int j = start_index; j > i; --j)
                        {
                            if(parts[i].is_constrained_by(parts[j].pose_to) || parts[j].is_constrained_by(parts[i].pose_from) )
                            {
                                // PRX_DEBUG_COLOR("The part at place  " << j << "  is blocking us",PRX_TEXT_RED);
                                return std::make_pair(false,0);
                            }
                        }
                        // PRX_DEBUG_COLOR("Swapping from place " << i, PRX_TEXT_GREEN);
                        return std::make_pair(true,i);
                    }
                }
                // PRX_DEBUG_COLOR("No same id", PRX_TEXT_CYAN);
                return std::make_pair(false,0);
            }

            bool rearrangement_path_planner_t::bring_parts_together(std::deque<path_part_t>& parts)
            {
                bool changed = false;
                bool swap_flag = false;
                unsigned index = 0;
                for(unsigned i = 0; i < parts.size()-1; ++i)
                {
                    unsigned id = parts[i].id;
                    // PRX_DEBUG_COLOR("new id to check " << id, PRX_TEXT_GREEN);
                    if(parts[i+1].id != id)
                    {
                        boost::tie(swap_flag, index) = swap(parts,id,i+1);
                        if(swap_flag)
                        {
                            path_part_t new_part(parts[index]);
                            // PRX_DEBUG_COLOR("The new part is : " << new_part.print(), PRX_TEXT_BLUE);
                            parts.erase(parts.begin() + index);
                            parts.insert(parts.begin() + i + 1, new_part);
                            // PRX_DEBUG_COLOR("Delete from index: " << index << "  to the index " << i+1, PRX_TEXT_BLUE);
                            changed = true;
                        }
                    }
                }
                // PRX_DEBUG_COLOR("AFTER FORWARD: " << print_parts(parts), PRX_TEXT_BROWN);

                //***********//
                // BACKWARDS //
                //***********//
                for(unsigned c = 0; c < parts.size()-1; ++c)
                {
                    unsigned i = parts.size() - 1 - c;
                    unsigned id = parts[i].id;
                    // PRX_DEBUG_COLOR("new id to check " << id << "  at index:" << i, PRX_TEXT_GREEN);
                    if(parts[i-1].id != id)
                    {
                        boost::tie(swap_flag, index) = swap_backward(parts,id,i-1);
                        if(swap_flag)
                        {
                            path_part_t new_part(parts[index]);
                            // PRX_DEBUG_COLOR("The new part is : " << new_part.print(), PRX_TEXT_BLUE);                            
                            parts.insert(parts.begin() + i, new_part);
                            parts.erase(parts.begin() + index);
                            // PRX_DEBUG_COLOR("Delete from index: " << index << "  to the index " << i, PRX_TEXT_BLUE);
                            // PRX_DEBUG_COLOR("Steps: " << print_parts(parts), PRX_TEXT_CYAN);
                            changed = true;
                        }
                    }
                }
                return changed;
            }

            bool rearrangement_path_planner_t::remove_same_objects(std::deque<path_part_t>& parts)
            {
                bool changed = false;
                for(unsigned i = 0; i < parts.size()-1; ++i)
                {
                    unsigned id = parts[i].id;
                    unsigned j = i + 1;
                    bool found_same = false;
                    while(parts[j].id == id)
                    {
                        found_same = true;
                        j++;
                    }

                    if(found_same)
                    {
                        j--;
                        parts[i].merge_to_end(parts[j]);
                        parts.erase(parts.begin()+i+1, parts.begin()+j+1);
                        if(parts[i].pose_from == parts[i].pose_to)
                        {
                            parts.erase(parts.begin()+i);
                            i--;
                        }
                        changed = true;
                    }
                }
                return changed;
            }

            bool rearrangement_path_planner_t::validate_full_path()
            {
                trajectory_t full_path;

                model->set_propagate_response(true);
                manip_state_space->copy_from_point(safe_state);
                all_object_space->copy_from_point(real_initial_state);
                model->use_context("full_space");
                const space_t* full_space = model->get_full_state_space();
                full_path.link_space(full_space);
                state_t* curr_state = manip_state_space->clone_point(safe_state);
                state_t* new_state = full_space->alloc_point();
                model->use_context(pc_name_all_objects);

                foreach(plan_step_t step, in_query->plan)
                {
                    //                    PRX_DEBUG_COLOR(manip_control_space->print_point(step.control, 5) << "   t:" << step.duration, PRX_TEXT_CYAN);
                    model->propagate_once(curr_state, step.control, step.duration, curr_state);
                    full_space->copy_to_point(new_state);
                    full_path.copy_onto_back(new_state);
                    //                    PRX_DEBUG_COLOR(full_space->print_point(new_state, 5), PRX_TEXT_GREEN);
                }
                full_space->free_point(new_state);

                model->use_context("full_space");
                bool valid = validity_checker->is_valid(full_path);
                model->set_propagate_response(false);
                return valid;
            }

            std::string rearrangement_path_planner_t::print_parts(const std::deque<path_part_t> parts)
            {
                std::stringstream output(std::stringstream::out);
                
                foreach(path_part_t p, parts)
                {
                    output << std::endl << p.print();
                }                
                
                return output.str();
            }
        }
    }
}
