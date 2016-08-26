/**
 * @file mrs.cpp
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

#include "planning/task_planners/rearrangement_primitives/mRS.hpp"


#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/goals/multiple_goal_states.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/simulation/plan.hpp"

#include "../../../../rearrangement_manipulation/planning/modules/obstacle_aware_astar.hpp"

#include <vector>
#include <pluginlib/class_list_macros.h>
#include <boost/range/adaptor/map.hpp>
#include <set>

PLUGINLIB_EXPORT_CLASS(prx::packages::labeled_rearrangement_manipulation::mrs_t, prx::plan::planner_t)

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

            mrs_t::mrs_t() { }

            mrs_t::~mrs_t() { }

            void mrs_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
            {
                rearrangement_primitive_t::init(reader, template_reader);
                multiple_targets = parameters::get_attribute_as<bool>("multiple_targets", reader, template_reader, false);
            }

            void mrs_t::reset_constraints(const std::vector<unsigned>& arrangement)
            {
                transit_constraints.clear();
                transit_constraints.insert(arrangement.begin(), arrangement.end());
                transfer_constraints.clear();
                transfer_constraints.insert(arrangement.begin(), arrangement.end());
            }

            bool mrs_t::solve()
            {
                curr_order.resize(k_objects);
                max_depth = -1;
                curr_depth = -1;

                std::deque<unsigned> future;
                for( unsigned i = 0; i < k_objects; ++i )
                    future.push_back(i);

                std::vector<state_t*> from_states;
                from_states.push_back(safe_state);

                for( unsigned i = 0; i < k_objects && !time_ends; ++i )
                {
                    std::vector<unsigned> a_curr = in_query->initial_poses_ids;
                    unsigned id = future[0];
                    //                                        PRX_DEBUG_COLOR("-------------------------------------------", PRX_TEXT_CYAN);
                    //                                        PRX_DEBUG_COLOR("--- STARTS With : " << id << "   POSE: " << a_curr[i] << " ---", PRX_TEXT_GREEN);
                    //                                        PRX_DEBUG_COLOR("-------------------------------------------", PRX_TEXT_CYAN);
                    future.pop_front();
                    if( mrs(in_query->path_sequence, id, from_states, poses_set[in_query->target_poses_ids[id]], future, a_curr, in_query->target_poses_ids) )
                    {
                        return true;
                    }
                    PRX_ASSERT(curr_depth == -1);
                    future.push_back(id);
                }

                in_query->got_partial_solution = false;
                if( in_query->accept_partial_solutions && max_depth != -1 )
                {

                    PRX_DEBUG_COLOR("------------------------------", PRX_TEXT_BROWN);
                    PRX_DEBUG_COLOR("------ PARTIAL SOLUTION " << max_depth << "----", PRX_TEXT_RED);
                    PRX_DEBUG_COLOR("------------------------------", PRX_TEXT_BROWN);
                    in_query->got_partial_solution = true;
                    in_query->partial_solution = in_query->initial_poses_ids;
                    for( int i = 0; i <= max_depth; ++i )
                    {
                        in_query->path_sequence.push_back(best_order[i]);
                        in_query->partial_solution[best_order[i].id] = best_order[i].pose_to;
                        PRX_DEBUG_COLOR("order (" << i << "/" << max_depth << "):" << best_order[i].print(), PRX_TEXT_GREEN);
                    }
                }
                return false;
            }

            bool mrs_t::mrs(std::deque<path_part_t>& sequence, unsigned object_id, const std::vector<sim::state_t*>& from_states, pose_t& pose, std::deque<unsigned> future, std::vector<unsigned> a_curr, const std::vector<unsigned>& target_arrangement)
            {                
                PRX_ASSERT(pose.pose_id == target_arrangement[object_id]);
                if( statistics_clock.measure() > in_query->time_limit )
                {
                    time_ends = true;
                    return false;
                } 
                curr_depth++;

                bool will_stay = pose.pose_id == a_curr[object_id];

                if( will_stay || clear_to_move(object_id, pose, a_curr) )
                {
                    std::vector<state_t*> new_states;
                    plan_t union_plan;
                    std::set<unsigned> union_full_constraints;
                    int index_from = -1;
                    int index_to = -1;

                    unsigned pose_from = a_curr[object_id];
                    PRX_DEBUG_COLOR("-------------------------------------------------------------------------------------", PRX_TEXT_BROWN);
                    PRX_DEBUG_COLOR("  GOING TO MOVE object: " << object_id << "  from pose: " << pose_from << "   to pose: " << target_arrangement[object_id] << "  Acurr : " << print(a_curr) << "  future: " << print(future), PRX_TEXT_MAGENTA);
                    PRX_DEBUG_COLOR("-------------------------------------------------------------------------------------", PRX_TEXT_BROWN);
                    if( will_stay )
                    {
                        new_states.push_back(from_states[0]);
                    }
                    else
                    {
                        reset_constraints(a_curr);
                        transfer_constraints.erase(a_curr[object_id]);
                        boost::tie(index_from, index_to) = get_union_path(union_plan, union_full_constraints, from_states, &poses_set[a_curr[object_id]], &pose);
                        if( index_to != -1 )
                        {
                            a_curr[object_id] = pose.pose_id;
                            new_states.push_back(pose.ungrasped_set[index_to]);
                        }

                    }

                    if( will_stay || index_to != -1 )
                    {
                        if( in_query->accept_partial_solutions )
                        {
                            curr_order[curr_depth].reset(object_id, pose_from, target_arrangement[object_id], index_from, index_to);
                            curr_order[curr_depth].constraints = union_full_constraints;
                            if( curr_depth > max_depth )
                            {
                                PRX_DEBUG_COLOR("---------------------------------------------------------------", PRX_TEXT_BROWN);
                                PRX_DEBUG_COLOR("- UPDATE BEST ORDER " << curr_depth << "/" << max_depth << "   object:" << object_id << ") " << pose_from << "->" << target_arrangement[object_id] << " -", PRX_TEXT_BLUE);
                                PRX_DEBUG_COLOR("---------------------------------------------------------------", PRX_TEXT_BROWN);
                                best_order = curr_order;
                                max_depth = curr_depth;
                                //                                PRX_ASSERT(false);  
                            }
                        }
                        if( future.size() == 0 )
                        {
                            transit_constraints.clear();
                            transit_constraints.insert(a_curr.begin(), a_curr.end());

                            transit_query->clear();
                            transit_query->copy_start(new_states[0]);
                            transit_query_goal->copy_goal_state(safe_state);
                            manip_tp->link_query(transit_query);
                            manip_tp->resolve_query();
                            if( transit_query->found_path )
                            {
                                //                                PRX_DEBUG_COLOR("------------------", PRX_TEXT_BROWN);
                                //                                PRX_DEBUG_COLOR("------ DONE ------", PRX_TEXT_RED);
                                //                                PRX_DEBUG_COLOR("------------------", PRX_TEXT_BROWN);
                                //                                PRX_DEBUG_COLOR("", PRX_TEXT_BROWN);
                                if( !will_stay )
                                {
                                    stats->path_length += union_plan.size();
                                    stats->path_length += transit_query->plans[0]->size();
                                    path_part_t p(object_id, pose_from, target_arrangement[object_id], index_from, index_to);
                                    p.constraints = union_full_constraints;
                                    sequence.push_front(p);
                                }

                                //                            PRX_DEBUG_COLOR("From states size:" << from_states.size() << "   Pu_size:" << union_plan.size(), PRX_TEXT_CYAN);
                                //                            PRX_DEBUG_COLOR(object_id << ") From pose: (" << a_curr[object_id] << ") " << object_state_space->print_point(poses_set[a_curr[object_id]].state, 6), PRX_TEXT_GREEN);
                                //                            PRX_DEBUG_COLOR(object_id << ") To   pose: (" << target_arrangement[object_id] << ") " << object_state_space->print_point(poses_set[target_arrangement[object_id]].state, 6), PRX_TEXT_BROWN);
                                //                            PRX_DEBUG_COLOR("----------------------------------------------------------------", PRX_TEXT_BLUE);
                                return true;
                            }
                        }

                        unsigned size = future.size();
                        //                        PRX_DEBUG_COLOR("future size: " << size, PRX_TEXT_BROWN);
                        for( unsigned i = 0; i < size && !time_ends; ++i )
                        {
                            unsigned id = future[0];
                            future.pop_front();
                            PRX_ASSERT(id != object_id);
                            //                            PRX_DEBUG_COLOR("Id to move: " << id << "   future size: " << size, PRX_TEXT_BROWN);
                            if( mrs(sequence, id, new_states, poses_set[target_arrangement[id]], future, a_curr, target_arrangement) )
                            {
                                if( !will_stay )
                                {
                                    //                                    union_plan += final_plan;
                                    //                                    final_plan = union_plan;
                                    stats->path_length += union_plan.size();
                                    path_part_t p(object_id, pose_from, target_arrangement[object_id], index_from, index_to);
                                    p.constraints = union_full_constraints;
                                    sequence.push_front(p);
                                }

                                //                            PRX_DEBUG_COLOR(object_id << ") From pose: (" << a_curr[object_id] << ") " << object_state_space->print_point(poses_set[a_curr[object_id]].state, 6), PRX_TEXT_GREEN);
                                //                            PRX_DEBUG_COLOR(object_id << ") To   pose: (" << target_arrangement[object_id] << ") " << object_state_space->print_point(poses_set[target_arrangement[object_id]].state, 6), PRX_TEXT_BROWN);
                                //                            PRX_DEBUG_COLOR("----------------------------------------------------------------", PRX_TEXT_BLUE);

                                return true;
                            }
                            future.push_back(id);
                        }
                    }
                }
                curr_depth--;
                return false;
            }

            bool mrs_t::clear_to_move(unsigned object_id, const pose_t& pose, const std::vector<unsigned>& arrangement)
            {
                //We have already checked that the object that we are going to move to the pose \c pose is not on this pose.
                //This check will find out if another object is occupying the pose that we want to move the object. 
                if( std::find(arrangement.begin(), arrangement.end(), pose.pose_id) != arrangement.end() )
                    return false;

                if( pose.constraints.size() != 0 )
                    for( unsigned i = 0; i < arrangement.size(); ++i )
                        if( i != object_id )
                            if( pose.constraints.count(arrangement[i]) > 0 )
                                return false;

                return true;
            }

            std::pair<int, int> mrs_t::get_union_path(sim::plan_t& plan, std::set<unsigned>& full_constraints, const std::vector<sim::state_t*>& from_states, pose_t* start_pose, pose_t* end_pose)
            {
                unsigned f_size = from_states.size();
                unsigned s_size = start_pose->ungrasped_set.size();
                unsigned e_size = end_pose->ungrasped_set.size();

                for( unsigned f = 0; f < f_size; ++f )
                {
                    for( unsigned s = 0; s < s_size; ++s )
                    {
                        //                        PRX_DEBUG_COLOR("f: " << f + 1 << "/" << f_size << "  s:" << s + 1 << "/" << s_size, PRX_TEXT_CYAN);
                        transit_query->clear();
                        transit_query->copy_start(from_states[f]);
                        transit_query_goal->copy_goal_state(start_pose->ungrasped_set[s]);
                        manip_tp->link_query(transit_query);
                        manip_tp->resolve_query();
                        PRX_ASSERT(transit_query->plans.size() == 1);
                        if( transit_query->found_path )
                        {
                            PRX_ASSERT(transit_query->found_path == (transit_query->plans[0]->size() != 0));
                            //                            PRX_DEBUG_COLOR("-------------------------------------------", PRX_TEXT_BLUE);
                            //                            PRX_DEBUG_COLOR("\tFound TRANSIT: " << transit_query->plans[0]->size(), PRX_TEXT_LIGHTGRAY);
                            //                            PRX_DEBUG_COLOR("-------------------------------------------", PRX_TEXT_BLUE);

                            for( unsigned e = 0; e < e_size; ++e )
                            {
                                transfer_query->clear();
                                transfer_query->copy_start(start_pose->grasped_set[s]);
                                transfer_query_goal->copy_goal_state(end_pose->grasped_set[e]);
                                manip_tp->link_query(transfer_query);
                                manip_tp->resolve_query();
                                if( transfer_query->found_path )
                                {
                                    PRX_ASSERT(transfer_query->found_path == (transfer_query->plans[0]->size() != 0));
                                    //                                    PRX_DEBUG_COLOR("-------------------------------------------", PRX_TEXT_BLUE);
                                    //                                    PRX_DEBUG_COLOR("\tFound TRANSFER: " << transfer_query->plans[0]->size(), PRX_TEXT_LIGHTGRAY);
                                    //                                    PRX_DEBUG_COLOR("-------------------------------------------", PRX_TEXT_BLUE);
                                    PRX_ASSERT(transit_query->plans[0]->size() != 0);
                                    PRX_ASSERT(transfer_query->plans[0]->size() != 0);
                                    plan = (*transit_query->plans[0]);
                                    plan += (*transfer_query->plans[0]);
                                    full_constraints = transit_query->full_constraints[0];
                                    full_constraints.insert(transfer_query->full_constraints[0].begin(), transfer_query->full_constraints[0].end());
                                    PRX_DEBUG_COLOR("Got union path!", PRX_TEXT_GREEN);
                                    return std::make_pair(s, e);
                                }
                            }
                        }
                    }
                }
                return std::make_pair(-1, -1);
            }
        }
    }
}