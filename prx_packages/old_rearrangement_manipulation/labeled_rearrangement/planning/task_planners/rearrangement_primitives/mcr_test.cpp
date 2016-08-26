/**
 * @file mcr_test.cpp
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

#include "planning/task_planners/rearrangement_primitives/mcr_test.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/goals/multiple_goal_states.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/simulation/plan.hpp"

#include "../../../../rearrangement_manipulation/planning/modules/obstacle_aware_astar.hpp"

#include <vector>
#include <pluginlib/class_list_macros.h>
#include <boost/range/adaptor/map.hpp>
#include <set>

PLUGINLIB_EXPORT_CLASS(prx::packages::labeled_rearrangement_manipulation::mcr_test_t, prx::plan::planner_t)

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

            mcr_test_t::mcr_test_t() { }

            mcr_test_t::~mcr_test_t() { }

            void mcr_test_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
            {
                rearrangement_primitive_t::init(reader, template_reader);
                if( parameters::has_attribute("multipliers", reader, template_reader) )
                {
                    multipliers = parameters::get_attribute_as<std::vector<double> >("multipliers", reader, template_reader);
                }
                else
                {
                    multipliers.push_back(1.5);
                    multipliers.push_back(1.8);
                    multipliers.push_back(2);

                }
                index_from = parameters::get_attribute_as<unsigned>("index_from", reader, template_reader, 0);
                index_to = parameters::get_attribute_as<unsigned>("index_to", reader, template_reader, 0);
                transit_mode = parameters::get_attribute_as<bool>("transit_mode", reader, template_reader, false);
                PRX_DEBUG_COLOR(index_from << "   to: " << index_to << "   mode:" << transit_mode,PRX_TEXT_GREEN);
                PRX_ASSERT(false);
                mcr_stats = new mcr_test_statistics_t();
            }

            const statistics_t* mcr_test_t::get_statistics()
            {
                return mcr_stats;
            }

            void mcr_test_t::reset_constraints(const std::vector<unsigned>& arrangement)
            {
                transit_constraints.clear();
                transit_constraints.insert(arrangement.begin(), arrangement.end());
                transfer_constraints.clear();
                transfer_constraints.insert(arrangement.begin(), arrangement.end());
            }

            bool mcr_test_t::solve()
            {
                reset_constraints(in_query->initial_poses_ids);
                double computation_time;
                double path_length;
                double path_cost;
                double constraints;
                double short_time, shortest_path_length, shortest_cost, short_constraints;

                mcr_stats->objects_no = k_objects;
                mcr_stats->add_stats("shortest_path");

                mcr_stats->add_stats("exact");
                for( unsigned i = 0; i < multipliers.size(); ++i )
                {
                    mcr_stats->add_stats("exact" + boost::lexical_cast<std::string > (multipliers[i]));
                }

                mcr_stats->add_stats("greedy");
                //                for( unsigned i = 0; i < multipliers.size(); ++i )
                //                {
                //                    mcr_stats->add_stats("greedy" + boost::lexical_cast<std::string > (multipliers[i]));
                //                }


                for( unsigned i = 0; i < k_objects; ++i )
                {
                    //For the shortest path
                    transit_query->q_collision_type = motion_planning_query_t::PRX_LAZY_COLLISIONS;
                    transfer_query->q_collision_type = motion_planning_query_t::PRX_LAZY_COLLISIONS;
                    transit_astar->set_shortest_path_flag(false);
                    transfer_astar->set_shortest_path_flag(false);
                    transit_astar->set_minimum_conflict(true);
                    transfer_astar->set_minimum_conflict(true);
                    transit_astar->set_max_length(PRX_INFINITY);
                    transfer_astar->set_max_length(PRX_INFINITY);

                    //                    mcr_stats->clear();
                    //                    shortest_path = 0;
                    transit_short_len = 0;
                    transfer_short_len = 0;
                    PRX_DEBUG_COLOR("================================================================", PRX_TEXT_GREEN);
                    PRX_DEBUG_COLOR("======================== OBJECT " << i << " ========================", PRX_TEXT_CYAN);
                    PRX_DEBUG_COLOR("================================================================", PRX_TEXT_GREEN);
                    PRX_DEBUG_COLOR("================================================================", PRX_TEXT_BROWN);
                    PRX_DEBUG_COLOR("======================== Shortest  Path ========================", PRX_TEXT_BROWN);
                    PRX_DEBUG_COLOR("================================================================", PRX_TEXT_BROWN);
                    int index = 0;
                    shortest_path_length = 0;
                    short_time = 0;
                    short_constraints = 0;
                    if( get_path(i, &short_time, &shortest_path_length, &shortest_cost, &short_constraints, true) )
                    {
                        //mcr versions                        
                        transit_query->q_collision_type = motion_planning_query_t::PRX_ACTIVE_COLLISIONS_REUSE_EDGES;
                        transfer_query->q_collision_type = motion_planning_query_t::PRX_ACTIVE_COLLISIONS_REUSE_EDGES;

                        //exact
                        transit_astar->set_exact_flag(true);
                        transfer_astar->set_exact_flag(true);

                        PRX_DEBUG_COLOR("================================================================", PRX_TEXT_BROWN);
                        PRX_DEBUG_COLOR("===========================  Exact   ===========================", PRX_TEXT_BROWN);
                        PRX_DEBUG_COLOR("================================================================", PRX_TEXT_BROWN);
                        transit_astar->set_max_length(PRX_INFINITY);
                        transfer_astar->set_max_length(PRX_INFINITY);
                        if( get_path(i, &computation_time, &path_length, &path_cost, &constraints) )
                        {
                            if( constraints == 0 )
                                continue;
                            mcr_stats->add_at(index, short_time, shortest_path_length, shortest_cost, short_constraints, 0);
                            mcr_stats->num_of_moved_objects++;
                            index++;
                            mcr_stats->add_at(index, computation_time, path_length / shortest_path_length, path_cost / shortest_cost, constraints, 0);
                        }
                        else
                        {
                            PRX_DEBUG_COLOR("Exact did not find solution!", PRX_TEXT_RED);
                            continue;
                            //                            mcr_stats->add_at(index, 0, 0, 0, 1);
                        }
                        index++;


                        for( unsigned m = 0; m < multipliers.size(); ++m )
                        {
                            PRX_DEBUG_COLOR("================================================================", PRX_TEXT_BROWN);
                            PRX_DEBUG_COLOR("===========================  Exact " << multipliers[m] << " ===========================", PRX_TEXT_BROWN);
                            PRX_DEBUG_COLOR("================================================================", PRX_TEXT_BROWN);
                            transit_astar->set_max_length(multipliers[m] * transit_short_len);
                            transfer_astar->set_max_length(multipliers[m] * transfer_short_len);
                            if( get_path(i, &computation_time, &path_length, &path_cost, &constraints) )
                                mcr_stats->add_at(index, computation_time, path_length / shortest_path_length, path_cost / shortest_cost, constraints, 0);
                            else
                                mcr_stats->add_at(index, 0, 0, 0, 0, 1);
                            index++;
                            //PRX_ASSERT(false);
                        }

                        //greedy
                        transit_astar->set_exact_flag(false);
                        transfer_astar->set_exact_flag(false);

                        PRX_DEBUG_COLOR("================================================================", PRX_TEXT_BROWN);
                        PRX_DEBUG_COLOR("===========================  Greedy   ==========================", PRX_TEXT_BROWN);
                        PRX_DEBUG_COLOR("================================================================", PRX_TEXT_BROWN);
                        transit_astar->set_max_length(PRX_INFINITY);
                        transfer_astar->set_max_length(PRX_INFINITY);
                        if( get_path(i, &computation_time, &path_length, &path_cost, &constraints) )
                            mcr_stats->add_at(index, computation_time, path_length / shortest_path_length, path_cost / shortest_cost, constraints, 0);
                        else
                            mcr_stats->add_at(index, 0, 0, 0, 0, 1);
                        index++;

                        //                        for( unsigned m = 0; m < multipliers.size(); ++m )
                        //                        {
                        //                            PRX_DEBUG_COLOR("================================================================", PRX_TEXT_BROWN);
                        //                            PRX_DEBUG_COLOR("===========================  Greedy " << multipliers[m] << " ==========================", PRX_TEXT_BROWN);
                        //                            PRX_DEBUG_COLOR("================================================================", PRX_TEXT_BROWN);
                        //                            transit_astar->set_max_length(multipliers[m] * transit_short_len);
                        //                            transfer_astar->set_max_length(multipliers[m] * transfer_short_len);
                        //                            if( get_path(i, &computation_time, &path_length, &constraints) )
                        //                                mcr_stats->add_at(index, computation_time, path_length / shortest_path_length, constraints, 0);
                        //                            else
                        //                                mcr_stats->add_at(index, 0, 0, 0, 1);
                        //                            index++;
                        //                        }
                        PRX_DEBUG_COLOR("STATS for object (" << i << "): \n " << mcr_stats->get_statistics(), PRX_TEXT_GREEN);
                    }
                }
                mcr_stats->average_results(mcr_stats->num_of_moved_objects);
                return true;
            }

            bool mcr_test_t::get_path(unsigned id, double* computation_time, double* path_length, double* path_cost, double* num_constraints, bool extract_constraints)
            {
                if( in_query->initial_poses_ids[id] != in_query->target_poses_ids[id] )
                {
                    if( transit_mode )
                        return get_transit_path(id, computation_time, path_length, path_cost, num_constraints, extract_constraints);

                    std::set<unsigned> path_constraints;
                    statistics_clock.reset();

                    transfer_constraints.erase(in_query->initial_poses_ids[id]);
                    transfer_query->clear();
                    transfer_query->copy_start(poses_set[in_query->initial_poses_ids[id]].grasped_set[index_from]);
                    transfer_query_goal->copy_goal_state(poses_set[in_query->target_poses_ids[id]].grasped_set[index_to]);

                    //                    foreach(state_t* st, poses_set[in_query->target_poses_ids[id]].grasped_set)
                    //                    {
                    //                        transfer_query_goal->add_goal_state(st);
                    //                    }
                    manip_tp->link_query(transfer_query);
                    manip_tp->resolve_query();
                    if( transfer_query->found_path )
                    {
                        if( extract_constraints )
                        {
                            transfer_astar->extract_path_constraints(path_constraints);
                            transfer_short_len = transfer_astar->get_path_cost();
                        }
                        else
                        {
                            path_constraints.insert(transfer_query->constraints[0].begin(), transfer_query->constraints[0].end());
                        }
                        //*path_length = transfer_query->plans[0]->size();
                        *path_length = transfer_query->plans[0]->length();
                        *path_cost = transfer_query->solutions_costs[0];
                        *num_constraints = path_constraints.size();
                        *computation_time = statistics_clock.measure();
                        transfer_constraints.insert(in_query->initial_poses_ids[id]);
                        return true;
                    }
                }
                transfer_constraints.insert(in_query->initial_poses_ids[id]);
                return false;
            }

            bool mcr_test_t::get_transit_path(unsigned id, double* computation_time, double* path_length, double* path_cost, double* num_constraints, bool extract_constraints)
            {

                std::set<unsigned> path_constraints;
                statistics_clock.reset();

                transit_query->clear();
                transit_query->copy_start(safe_state);
                transit_query_goal->copy_multiple_goal_states(poses_set[in_query->initial_poses_ids[id]].ungrasped_set);

                // foreach(state_t* st, poses_set[in_query->initial_poses_ids[id]].ungrasped_set)
                // {
                //     transit_query_goal->add_goal_state(st);
                // }
                manip_tp->link_query(transit_query);
                manip_tp->resolve_query();
                if( transit_query->found_path )
                {
                    if( extract_constraints )
                    {
                        transit_astar->extract_path_constraints(path_constraints);
                        transit_short_len = transit_astar->get_path_cost();
                    }
                    else
                    {
                        path_constraints.insert(transit_query->constraints[0].begin(), transit_query->constraints[0].end());
                    }
                    //*path_length = transit_query->plans[0]->size();
                    *path_length = transit_query->plans[0]->length();
                    *path_cost = transit_query->solutions_costs[0] / transit_short_len;
                    *num_constraints = path_constraints.size();
                    *computation_time = statistics_clock.measure();
                    return true;
                }

                return false;
            }

            bool mcr_test_t::get_full_path(unsigned id, double* computation_time, double* path_length, double* num_constraints, bool extract_constraints)
            {
                std::set<unsigned> path_constraints;
                int index = 0;
                statistics_clock.reset();
                if( in_query->initial_poses_ids[id] != in_query->target_poses_ids[id] )
                {
                    transit_query->clear();
                    transit_query->copy_start(in_query->start_state);
                    transit_query_goal->copy_multiple_goal_states(poses_set[in_query->initial_poses_ids[id]].ungrasped_set);

                    // foreach(state_t* st, poses_set[in_query->initial_poses_ids[id]].ungrasped_set)
                    // {
                    //     transit_query_goal->add_goal_state(st);
                    // }

                    manip_tp->link_query(transit_query);
                    manip_tp->resolve_query();
                    if( transit_query->found_path )
                    {
                        *path_length += transit_query->plans[0]->size();
                        if( extract_constraints )
                        {
                            transit_astar->extract_path_constraints(path_constraints);
                            transit_short_len = transit_astar->get_path_cost();

                        }
                        else
                        {
                            path_constraints.insert(transit_query->constraints[0].begin(), transit_query->constraints[0].end());
                        }
                        transfer_query->clear();
                        index = detect_released_state(&poses_set[in_query->initial_poses_ids[id]], transit_query->found_goal_state);
                        PRX_ASSERT(index != -1);
                        transfer_query->copy_start(poses_set[in_query->initial_poses_ids[id]].grasped_set[index]);
                        transfer_query_goal->copy_multiple_goal_states(poses_set[in_query->target_poses_ids[id]].grasped_set);

                        // foreach(state_t* st, poses_set[in_query->target_poses_ids[id]].grasped_set)
                        // {
                        //     transfer_query_goal->add_goal_state(st);
                        // }
                        manip_tp->link_query(transfer_query);
                        manip_tp->resolve_query();
                        if( transfer_query->found_path )
                        {
                            *path_length += transfer_query->plans[0]->size();
                            if( extract_constraints )
                            {
                                transfer_astar->extract_path_constraints(path_constraints);
                                transfer_short_len = transfer_astar->get_path_cost();
                            }
                            else
                            {
                                path_constraints.insert(transfer_query->constraints[0].begin(), transfer_query->constraints[0].end());
                            }
                            transit_query->clear();
                            index = detect_grasped_state(&poses_set[in_query->target_poses_ids[id]], transfer_query->found_goal_state);
                            transit_query->copy_start(poses_set[in_query->target_poses_ids[id]].ungrasped_set[index]);
                            transit_query_goal->copy_goal_state(in_query->start_state);
                            manip_tp->link_query(transit_query);
                            manip_tp->resolve_query();
                            if( transit_query->found_path )
                            {
                                *path_length += transit_query->plans[0]->size();
                                if( extract_constraints )
                                {
                                    transit_astar->extract_path_constraints(path_constraints);
                                    transit_short_len += transit_astar->get_path_cost();
                                    transit_short_len /= 2;
                                }
                                else
                                {
                                    path_constraints.insert(transit_query->constraints[0].begin(), transit_query->constraints[0].end());
                                }
                                *num_constraints += path_constraints.size();
                                *computation_time += statistics_clock.measure();
                                return true;
                            }
                        }
                    }
                }
                return false;
            }

            int mcr_test_t::detect_released_state(pose_t* pose, state_t * state)
            {
                for( unsigned i = 0; i < pose->ungrasped_set.size(); ++i )
                {
                    if( manip_state_space->equal_points(pose->ungrasped_set[i], state, PRX_DISTANCE_CHECK) )
                        return i;
                }
                return -1;
            }

            int mcr_test_t::detect_grasped_state(pose_t* pose, state_t * state)
            {
                for( unsigned i = 0; i < pose->grasped_set.size(); ++i )
                {
                    if( mo_space->equal_points(pose->grasped_set[i], state, PRX_DISTANCE_CHECK) )
                        return i;
                }
                return -1;
            }
        }
    }
}
