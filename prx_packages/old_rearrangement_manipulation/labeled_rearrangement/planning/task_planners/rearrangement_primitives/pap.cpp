/**
 * @file fmrs.cpp
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

#include "planning/task_planners/rearrangement_primitives/pap.hpp"
#include "prx/utilities/definitions/random.hpp"


#include "planning/graphs/constraints_graph.hpp"

#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/goals/multiple_goal_states.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/graph/directed_graph.hpp"
#include "prx/utilities/graph/directed_node.hpp"
#include "prx/utilities/graph/directed_edge.hpp"
#include "prx/simulation/plan.hpp"

#include "../../../../rearrangement_manipulation/planning/modules/obstacle_aware_astar.hpp"

#include <vector>
#include <pluginlib/class_list_macros.h>
#include <boost/range/adaptor/map.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/graph/strong_components.hpp>
#include <set>


PLUGINLIB_EXPORT_CLASS(prx::packages::labeled_rearrangement_manipulation::pap_t, prx::plan::planner_t)

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
            pap_t::pap_t() { }

            pap_t::~pap_t() { }

            bool pap_t::solve()
            {
                return pick_and_place(in_query->path_sequence, in_query->initial_poses_ids, in_query->target_poses_ids);
            }          

            bool pap_t::pick_and_place(std::deque<path_part_t>& sequence, const std::vector<unsigned>& a_curr, const std::vector<unsigned>& target_arrangement)
            {
                if( statistics_clock.measure() > in_query->time_limit )
                {
                    time_ends = true;
                    return false;
                }

                int index = single_step(a_curr,target_arrangement);
                if(index != -1)
                {
                    path_part_t part;
                    PRX_DEBUG_COLOR("Connection between : " << print(a_curr) << " -> " << print(target_arrangement), PRX_TEXT_LIGHTGRAY );
                    reset_constraints(a_curr);

                    transfer_obstacles.erase(a_curr[index]);
                    if(get_path(part, safe_state, &poses_set[a_curr[index]], &poses_set[target_arrangement[index]]))
                    {
                        part.init(index, a_curr[index], target_arrangement[index]);
                        sequence.push_back(part);
    #ifndef NDEBUG
                            PRX_DEBUG_COLOR("", PRX_TEXT_RED);
                            PRX_DEBUG_COLOR("========================== PATH ======================================", PRX_TEXT_RED);
                            PRX_DEBUG_COLOR("====   Found path : " << part.print() << "  ====", PRX_TEXT_LIGHTGRAY);
                            PRX_DEBUG_COLOR("======================================================================", PRX_TEXT_RED);
                            PRX_DEBUG_COLOR("", PRX_TEXT_RED);
    #endif                        
                        return true;
                    }    
                }     
                return false;           

                
//                 path_part_t part;
//                 in_query->got_partial_solution = false;
//                 reset_constraints(a_curr);

//                 int index = single_step(a_curr,target_arrangement);
//                 //This means that we have single different pose at the a_curr[index] position.
//                 if(index != -1)
//                 {
//                     transfer_obstacles.erase(a_curr[index]);                    
//                     if(get_path(part, safe_state, &poses_set[a_curr[index]], &poses_set[target_arrangement[index]]))
//                     {
//                         part.init(index, a_curr[index], target_arrangement[index]);
//                         sequence.push_back(part);
// #ifndef NDEBUG
//                             PRX_DEBUG_COLOR("", PRX_TEXT_RED);
//                             PRX_DEBUG_COLOR("========================== PATH ======================================", PRX_TEXT_RED);
//                             PRX_DEBUG_COLOR("====   Found path : " << part.print() << "  ====", PRX_TEXT_LIGHTGRAY);
//                             PRX_DEBUG_COLOR("======================================================================", PRX_TEXT_RED);
//                             PRX_DEBUG_COLOR("", PRX_TEXT_RED);
// #endif                        
//                         return true;
//                     }                    
//                 }
//                 else if( in_query->accept_partial_solutions)
//                 {
//                     for(unsigned i = 0; i < k_objects; ++i)
//                     {
//                         if( statistics_clock.measure() > in_query->time_limit )
//                         {
//                             time_ends = true;
//                             return false;
//                         }
                
//                         int new_index = uniform_int_random(0,k_objects-1);
//                         transfer_obstacles.erase(a_curr[new_index]);
//                         if(a_curr[new_index] != target_arrangement[new_index] && get_path(part, safe_state, &poses_set[a_curr[new_index]], &poses_set[target_arrangement[new_index]]))
//                         {
//                             part.init(new_index, a_curr[new_index], target_arrangement[new_index]);
//                             sequence.push_back(part);
//                             in_query->got_partial_solution = true;
//                             in_query->partial_solution = a_curr;
//                             in_query->partial_solution[new_index] = target_arrangement[new_index];
// #ifndef NDEBUG
//                             PRX_DEBUG_COLOR("", PRX_TEXT_RED);
//                             PRX_DEBUG_COLOR("========================== PATH ======================================", PRX_TEXT_RED);
//                             PRX_DEBUG_COLOR("====   Found Partial path : " << part.print() << "  ====", PRX_TEXT_LIGHTGRAY);
//                             PRX_DEBUG_COLOR("======================================================================", PRX_TEXT_RED);
//                             PRX_DEBUG_COLOR("", PRX_TEXT_RED);
// #endif
//                             //Return false otherwise the rearrangement_primitive will declare that got a solution. 
//                             return false;
//                         }
//                         transfer_obstacles.insert(a_curr[new_index]);
//                     }
//                 }
//                 return false;
            }

            bool pap_t::pick_and_place2(std::deque<path_part_t>& sequence, const std::vector<unsigned>& a_curr, const std::vector<unsigned>& target_arrangement)
            {
                if( statistics_clock.measure() > in_query->time_limit )
                {
                    time_ends = true;
                    return false;
                }
                PRX_DEBUG_COLOR("Connection between : " << print(a_curr) << " -> " << print(target_arrangement), PRX_TEXT_LIGHTGRAY );
                path_part_t part;
                in_query->got_partial_solution = false;
                reset_constraints(a_curr);

                int index = single_step(a_curr,target_arrangement);
                //This means that we have single different pose at the a_curr[index] position.
                if(index != -1)
                {                    
                	transfer_obstacles.erase(a_curr[index]);                    
                    if(get_path(part, safe_state, &poses_set[a_curr[index]], &poses_set[target_arrangement[index]]))
                    {
                        part.init(index, a_curr[index], target_arrangement[index]);
                        sequence.push_back(part);
#ifndef NDEBUG
                            PRX_DEBUG_COLOR("", PRX_TEXT_RED);
                            PRX_DEBUG_COLOR("========================== PATH ======================================", PRX_TEXT_RED);
                            PRX_DEBUG_COLOR("====   Found path : " << part.print() << "  ====", PRX_TEXT_LIGHTGRAY);
                            PRX_DEBUG_COLOR("======================================================================", PRX_TEXT_RED);
                            PRX_DEBUG_COLOR("", PRX_TEXT_RED);
#endif                        
                        return true;
                    }                    
                }
                else if( in_query->accept_partial_solutions)
                {
                    for(unsigned i = 0; i < k_objects; ++i)
                    {
                        if( statistics_clock.measure() > in_query->time_limit )
                        {
                            time_ends = true;
                            return false;
                        }
                
                        int new_index = uniform_int_random(0,k_objects-1);
                        transfer_obstacles.erase(a_curr[new_index]);
                        if(a_curr[new_index] != target_arrangement[new_index] && get_path(part, safe_state, &poses_set[a_curr[new_index]], &poses_set[target_arrangement[new_index]]))
                        {
                            part.init(new_index, a_curr[new_index], target_arrangement[new_index]);
                            sequence.push_back(part);
                            in_query->got_partial_solution = true;
                            in_query->partial_solution = a_curr;
                            in_query->partial_solution[new_index] = target_arrangement[new_index];
#ifndef NDEBUG
                            PRX_DEBUG_COLOR("", PRX_TEXT_RED);
                            PRX_DEBUG_COLOR("========================== PATH ======================================", PRX_TEXT_RED);
                            PRX_DEBUG_COLOR("====   Found Partial path : " << part.print() << "  ====", PRX_TEXT_LIGHTGRAY);
                            PRX_DEBUG_COLOR("======================================================================", PRX_TEXT_RED);
                            PRX_DEBUG_COLOR("", PRX_TEXT_RED);
#endif
                            //Return false otherwise the rearrangement_primitive will declare that got a solution. 
                            return false;
                        }
                        transfer_obstacles.insert(a_curr[new_index]);
                    }
                }
                return false;
            }

            int pap_t::single_step(const std::vector<unsigned>& a_curr, const std::vector<unsigned>& target_arrangement)
            {
                int index = -1;
                for(int i = 0; i < k_objects; ++i)
                {
                    PRX_DEBUG_COLOR("Single step : " << i << "/" << k_objects, PRX_TEXT_LIGHTGRAY);
                    if(a_curr[i] != target_arrangement[i] && std::find(target_arrangement.begin(), target_arrangement.end(), a_curr[i]) == target_arrangement.end())
                    {                        
                        if(index != -1)
                            return -1;
                        index = i;                        
                    }
                }
                //PRX_ASSERT(index != -1);
                return index;
            }

            bool pap_t::get_path(path_part_t& part, sim::state_t* safe_state, pose_t* start_pose, pose_t* end_pose)
            {
            	std::set<unsigned> constraints;
                transit_query->clear();
                transit_query->copy_start(safe_state);                
                transit_query_goal->copy_multiple_goal_states(start_pose->ungrasped_set);
                manip_tp->link_query(transit_query);
                manip_tp->resolve_query();
                if( transit_query->found_path )
                {
                	constraints = transit_query->full_constraints[0];
                	int from_pose_index = detect_released_state(start_pose,transit_query->found_goal_state);
                    transfer_query->clear();    
                    transfer_query->copy_start(start_pose->grasped_set[from_pose_index]);                    
                    transfer_query_goal->copy_multiple_goal_states(end_pose->grasped_set);                    
                    manip_tp->link_query(transfer_query);
                    manip_tp->resolve_query();

                    if( transfer_query->found_path )
                    {
                    	int to_pose_index = detect_grasped_state(end_pose, transfer_query->found_goal_state);

                    	transit_obstacles.erase(from_pose_index);
                        transit_obstacles.insert(to_pose_index);

                        transit_query->clear();
                        transit_query->copy_start(end_pose->ungrasped_set[to_pose_index]);
                        transit_query_goal->copy_goal_state(safe_state);
                        manip_tp->link_query(transit_query);
                        manip_tp->resolve_query();
                        if( transit_query->found_path )
                        {
                        	constraints.insert(transfer_query->full_constraints[0].begin(), transfer_query->full_constraints[0].end());
                        	constraints.insert(transit_query->full_constraints[0].begin(), transit_query->full_constraints[0].end());                            
                        	part.update(constraints, from_pose_index, to_pose_index );
                            part.has_plan = true;
                            return true;
                        }
                        transit_obstacles.erase(to_pose_index);
                        transit_obstacles.insert(from_pose_index);
                    }
                }
                return false;
            }

            int pap_t::detect_released_state(pose_t* pose, state_t* state)
            {
                for( unsigned i = 0; i < pose->ungrasped_set.size(); ++i )
                {
                    if( manip_state_space->equal_points(pose->ungrasped_set[i], state, PRX_DISTANCE_CHECK) )
                        return i;
                }
                return -1;
            }

            int pap_t::detect_grasped_state(pose_t* pose, state_t* state)
            {
                for( unsigned i = 0; i < pose->grasped_set.size(); ++i )
                {
                    if( mo_space->equal_points(pose->grasped_set[i], state, PRX_DISTANCE_CHECK) )
                        return i;
                }
                return -1;
            }

            void pap_t::reset_constraints(const std::vector<unsigned>& arrangement)
            {
                transit_obstacles.clear();
                transit_obstacles.insert(arrangement.begin(), arrangement.end());
                transfer_obstacles.clear();
                transfer_obstacles.insert(arrangement.begin(), arrangement.end());
            }
        }
    }
}