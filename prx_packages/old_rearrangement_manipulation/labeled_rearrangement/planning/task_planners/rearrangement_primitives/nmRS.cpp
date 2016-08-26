/**
 * @file nmrs.cpp
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

#include "planning/task_planners/rearrangement_primitives/nmRS.hpp"


#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/goals/multiple_goal_states.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/simulation/plan.hpp"

#include "../../../../rearrangement_manipulation/planning/modules/obstacle_aware_astar.hpp"

#include <vector>
#include <pluginlib/class_list_macros.h>
#include <boost/range/adaptor/map.hpp>
#include <set>

PLUGINLIB_EXPORT_CLASS(prx::packages::labeled_rearrangement_manipulation::nmrs_t, prx::plan::planner_t)

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

            nmrs_t::nmrs_t() { }

            nmrs_t::~nmrs_t() { }

            void nmrs_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                rearrangement_primitive_t::init(reader,template_reader);
                length_multiplier = parameters::get_attribute_as<double>("length_multiplier", reader, template_reader, 1.5);
            }
            
            bool nmrs_t::solve()
            {
                std::deque<unsigned> future;
                for( unsigned i = 0; i < k_objects; ++i )
                    future.push_back(i);

                in_query->got_partial_solution = false;                
                for( unsigned i = 0; i < k_objects && !time_ends; ++i )
                {
                    std::vector<unsigned> a_curr = in_query->initial_poses_ids;
                    unsigned id = future[0];
                    //                                        PRX_DEBUG_COLOR("-------------------------------------------", PRX_TEXT_CYAN);
                    //                                        PRX_DEBUG_COLOR("--- STARTS With : " << id << "   POSE: " << a_curr[i] << " ---", PRX_TEXT_GREEN);
                    //                                        PRX_DEBUG_COLOR("-------------------------------------------", PRX_TEXT_CYAN);
                    future.pop_front();
                    if( nmrs(in_query->path_sequence, id, safe_state, future, a_curr, in_query->target_poses_ids) )
                    {
                        std::string path_file = prx_output_dir + "paths/nmRS.txt";
                        std::ofstream foutpath(path_file.c_str());
                        in_query->plan.save_to_stream(foutpath, 8);
                        foutpath.close();
                        return true;
                    }
                    future.push_back(id);
                }
                return false;
            }

            bool nmrs_t::nmrs(std::deque<path_part_t>& sequence, unsigned object_id, state_t* from_state, std::deque<unsigned> future, std::vector<unsigned> a_curr, const std::vector<unsigned>& target_arrangement)
            {
                if( statistics_clock.measure() > in_query->time_limit )
                {
                    time_ends = true;
                    return false;
                }

                bool will_stay = (a_curr[object_id] == target_arrangement[object_id]);

                state_t* new_state;
                std::set<unsigned> union_constraints;
                std::set<unsigned> union_full_constraints;
                std::set<unsigned> transfer_full_constraints;
                std::vector<unsigned> ordered_constraints;
                int index_from = -1;
                int index_to = -1;

                unsigned pose_from = a_curr[object_id];
                PRX_DEBUG_COLOR("====================================================================================", PRX_TEXT_BROWN);
                PRX_DEBUG_COLOR("  GOING TO MOVE object: " << object_id << "  from pose: " << pose_from << "   to pose: " << target_arrangement[object_id] << "  Acurr : " << print(a_curr) << "  future: " << print(future), PRX_TEXT_MAGENTA);
                PRX_DEBUG_COLOR("====================================================================================", PRX_TEXT_BROWN);
                if( will_stay )
                {
                    new_state = from_state;
                }
                else
                {
                    //We are going to move a new object. We care only for the objects in their final position. 
                    std::set<unsigned> constraints;
                    fix_constraints(object_id, a_curr, future, constraints);
                    transfer_constraints.erase(pose_from);
                    boost::tie(index_from, index_to) = get_union_path(union_constraints, union_full_constraints, transfer_full_constraints, ordered_constraints, from_state, &poses_set[pose_from], &poses_set[target_arrangement[object_id]]);
                    PRX_DEBUG_COLOR("union_full_constraints: " << print(union_full_constraints), PRX_TEXT_CYAN);
                    PRX_DEBUG_COLOR("transfer_constraints: " << print(transfer_full_constraints), PRX_TEXT_CYAN);
                    if( index_from == -1 )
                        return false;
                    if( union_constraints.size() == 0 )
                    {
                        a_curr[object_id] = target_arrangement[object_id];
                        new_state = poses_set[target_arrangement[object_id]].ungrasped_set[index_to];
                    }
                }

                if( will_stay || union_constraints.size() == 0 )
                {
                    if( future.size() == 0 )
                    {
                        transit_obstacles.clear();
                        transit_constraints.clear();
                        transit_avoid.clear();
                        transit_obstacles.insert(a_curr.begin(), a_curr.end());

                        transit_query->clear();
                        transit_query->copy_start(new_state);
                        transit_query_goal->copy_goal_state(safe_state);
                        transit_astar->set_max_length(PRX_INFINITY);
                        manip_tp->link_query(transit_query);
                        manip_tp->resolve_query();
                        if( transit_query->found_path )
                        {
                            PRX_DEBUG_COLOR("------------------", PRX_TEXT_BROWN);
                            PRX_DEBUG_COLOR("------ DONE ------", PRX_TEXT_RED);
                            PRX_DEBUG_COLOR("------------------", PRX_TEXT_BROWN);
                            PRX_DEBUG_COLOR("", PRX_TEXT_BROWN);
                            //PRX_ASSERT(transit_query->plans[0]->size() != 0);

                            if( !will_stay )
                            {
                                path_part_t p(object_id, pose_from, target_arrangement[object_id], index_from, index_to);
                                p.constraints = union_full_constraints;
                                sequence.push_front(p);
                            }
                            return true;
                        }
                    }
                    unsigned size = future.size();
                    for( unsigned i = 0; i < size && !time_ends; ++i )
                    {
                        unsigned id = future[0];
                        future.pop_front();
                        PRX_ASSERT(id != object_id);
#ifndef NDEBUG
                        unsigned old_object_pose = a_curr[id]; //TODO: this is only for debugging remove this line
                        unsigned to_go_object_pose = target_arrangement[id];
#endif
                        if( nmrs(sequence, id, new_state, future, a_curr, target_arrangement) )
                        {
                            if( !will_stay )
                            {
                                path_part_t p(object_id, pose_from, target_arrangement[object_id], index_from, index_to);
                                p.constraints = union_full_constraints;
                                sequence.push_front(p);                                
                            }

                            PRX_ASSERT(old_object_pose == a_curr[id]);
                            PRX_DEBUG_COLOR("MOVED: (" << id << ") " << old_object_pose << "->" << to_go_object_pose << "     a_curr:" << print(a_curr), PRX_TEXT_BLUE);
                            return true;
                        }
                        future.push_back(id);
                    }
                }
                else
                {
                    PRX_ASSERT(union_constraints.size() != 0);
                    unsigned blocker_id = detect_blocker(object_id, a_curr, ordered_constraints);
                    PRX_DEBUG_COLOR("The blocker is: " << blocker_id << "   future:" << print(future), PRX_TEXT_MAGENTA);
                    PRX_ASSERT(blocker_id != object_id);
                    if( std::find(future.begin(), future.end(), blocker_id) != future.end() )
                    {
                        std::deque<unsigned> new_future = future;
                        new_future.erase(std::find(new_future.begin(), new_future.end(), blocker_id));
                        std::deque<path_part_t> blockers_plan;
                        PRX_DEBUG_COLOR("union_full_constraints for the blocker: " << print(union_full_constraints), PRX_TEXT_CYAN);
                        // if( (new_state = clear_blocker(blockers_plan, blocker_id, from_state, new_future, a_curr, target_arrangement, transfer_full_constraints)) != NULL )
                        if( (new_state = clear_blocker(blockers_plan, blocker_id, from_state, new_future, a_curr, target_arrangement, union_full_constraints)) != NULL )
                        {
#ifndef NDEBUG
                            unsigned old_object_pose = a_curr[object_id]; //TODO: this is only for debugging remove this line
                            unsigned to_go_object_pose = target_arrangement[object_id];
#endif
                            if( nmrs(sequence, object_id, new_state, future, a_curr, target_arrangement) )
                            {
                                sequence.insert(sequence.begin(), blockers_plan.begin(), blockers_plan.end());
                                PRX_DEBUG_COLOR("MOVED: (" << object_id << ") " << old_object_pose << "->" << to_go_object_pose << "     a_curr:" << print(a_curr), PRX_TEXT_CYAN);
                                return true;
                            }
                        }
                    }
                    if( time_ends )
                        return false;

                }
                return false;
            }

            void nmrs_t::fix_constraints(unsigned object_id, const std::vector<unsigned>& arrangement, const std::deque<unsigned>& future, const std::set<unsigned>& constraints)
            {
                transit_obstacles.clear();
                transit_constraints.clear();
                transit_avoid.clear();
                transfer_obstacles.clear();
                transfer_constraints.clear();
                transfer_avoid.clear();

                //First fix the solid constraints and the positions that we can go through but as a constraint. 
                for( unsigned i = 0; i < arrangement.size(); ++i )
                {
                    if( i == object_id )
                    {
                        transit_obstacles.insert(arrangement[i]);
                    }
                    else if( std::find(future.begin(), future.end(), i) == future.end() )
                    {
                        //If the object is not in the future list means that the object has already be moved and we have to respect its current pose.
                        transit_obstacles.insert(arrangement[i]);
                        transfer_obstacles.insert(arrangement[i]);
                    }
                    else
                    {
                        transit_constraints.insert(arrangement[i]);
                        transfer_constraints.insert(arrangement[i]);
                    }
                }


                //Checks which poses are neither occupied of objects nor constrained by another's object path.
                //These poses are the free poses and we are trying to avoid as much as we can to violate many of them.
                std::vector<pose_t*> free_poses;
                get_valid_intermediate_poses(free_poses, arrangement, future, constraints);

                foreach(pose_t* p, free_poses)
                {
                    transit_avoid.insert(p->pose_id);
                    transfer_avoid.insert(p->pose_id);
                }
                PRX_DEBUG_COLOR("\ntransit_obstacles:" << print(transit_obstacles) << "\ntransit_constraints:" << print(transit_constraints) << "\ntransit_avoid:" << print(transit_avoid), PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("\ntransfer_obstacles:" << print(transfer_obstacles) << "\ntransfer_constraints:" << print(transfer_constraints) << "\ntransfer_avoid:" << print(transfer_avoid), PRX_TEXT_CYAN);
            }

            state_t* nmrs_t::clear_blocker(std::deque<path_part_t>& sequence, unsigned object_id, state_t* from_state, const std::deque<unsigned>& future, std::vector<unsigned>& a_curr, const std::vector<unsigned>& target_arrangement, const std::set<unsigned>& constraints)
            {
                std::set<unsigned> path_constraints;
                std::set<unsigned> transfer_full_constraints;
                std::vector<unsigned> ordered_constraints;
                path_part_t new_part;
                PRX_DEBUG_COLOR("====================================================================================", PRX_TEXT_MAGENTA);
                PRX_DEBUG_COLOR("  GOING TO MOVE blocker: " << object_id << "  from pose: " << a_curr[object_id] << "  Acurr : " << print(a_curr) << "  future: " << print(future), PRX_TEXT_RED);
                PRX_DEBUG_COLOR("====================================================================================", PRX_TEXT_MAGENTA);
                if( !time_ends && intermediate_pose(new_part, path_constraints, transfer_full_constraints, ordered_constraints, object_id, from_state, future, a_curr, target_arrangement, constraints) )
                {
                    if( path_constraints.size() == 0 )
                    {
                        a_curr[object_id] = (unsigned)new_part.pose_to;
                        // new_part.constraints = transfer_full_constraints;
                        new_part.constraints = path_constraints;
                        sequence.push_front(new_part);
                        return poses_set[new_part.pose_to].ungrasped_set[new_part.index_to];
                    }
                    else
                    {
                        unsigned blocker_id = detect_blocker(object_id, a_curr, ordered_constraints);
                        PRX_DEBUG_COLOR("The new blocker is: " << blocker_id << "   future:" << print(future), PRX_TEXT_MAGENTA);
                        PRX_ASSERT(blocker_id != object_id);
                        if( std::find(future.begin(), future.end(), blocker_id) != future.end() )
                        {
                            std::deque<unsigned> new_future = future;
                            new_future.erase(std::find(new_future.begin(), new_future.end(), blocker_id));
                            state_t* next_state = NULL;
                            std::set<unsigned> union_constraints = constraints;
                            union_constraints.insert(transfer_full_constraints.begin(), transfer_full_constraints.end());                            
                            std::deque<path_part_t> blockers_plan;
                            if( (next_state = clear_blocker(blockers_plan, blocker_id, from_state, new_future, a_curr, target_arrangement, union_constraints)) != NULL )
                            {
                                if( (next_state = clear_blocker(sequence, object_id, next_state, future, a_curr, target_arrangement, constraints)) != NULL )
                                {
                                    sequence.insert(sequence.begin(), blockers_plan.begin(), blockers_plan.end());
                                    return next_state;
                                }
                            }
                        }
                    }

                }
                PRX_DEBUG_COLOR("Cound not find intermediate pose for blocker : " << object_id, PRX_TEXT_RED);
                return NULL;
            }        

            std::pair<int, int> nmrs_t::get_union_path(std::set<unsigned>& constraints, std::set<unsigned>& full_constraints, std::set<unsigned>& transfer_full_constraints, std::vector<unsigned>& ordered_constraints, state_t* from_state, pose_t* start_pose, pose_t* end_pose)
            {
                unsigned s_size = start_pose->ungrasped_set.size();
                unsigned e_size = end_pose->ungrasped_set.size();

                transit_query->clear();
                transit_query->copy_start(from_state);                
                transit_query_goal->copy_multiple_goal_states(start_pose->ungrasped_set);
                PRX_DEBUG_COLOR("=========================== A* union_path transit  ===========================", PRX_TEXT_BROWN);                
                manip_tp->link_query(transit_query);
                manip_tp->resolve_query();
                if( transit_query->found_path )
                {

                    transfer_query->clear();    
                    int from_pose_index;                
                    for( from_pose_index = 0; from_pose_index < start_pose->ungrasped_set.size(); ++from_pose_index )
                    {
                        if( manip_state_space->equal_points(start_pose->ungrasped_set[from_pose_index], transit_query->found_goal_state, PRX_DISTANCE_CHECK) )
                        {
                            transfer_query->copy_start(start_pose->grasped_set[from_pose_index]);
                            break;
                        }
                    }
                    
                    transfer_query_goal->copy_multiple_goal_states(end_pose->grasped_set);
                    manip_tp->link_query(transfer_query);
                    PRX_DEBUG_COLOR("=========================== A* union_path transfer  ===========================", PRX_TEXT_BROWN);
                    manip_tp->resolve_query();
                    if(transfer_query->found_path)
                    {                        
                        PRX_ASSERT(transfer_query->found_path == (transfer_query->plans[0]->size() != 0));
                        //                                    PRX_DEBUG_COLOR("-------------------------------------------", PRX_TEXT_BLUE);
                        //                                    PRX_DEBUG_COLOR("\tFound TRANSFER: " << transfer_query->plans[0]->size(), PRX_TEXT_LIGHTGRAY);
                        //                                    PRX_DEBUG_COLOR("-------------------------------------------", PRX_TEXT_BLUE);
                        PRX_ASSERT(transit_query->plans[0]->size() != 0);
                        PRX_ASSERT(transfer_query->plans[0]->size() != 0);

                        int to_pose_index;                
                        for( to_pose_index = 0; to_pose_index < end_pose->grasped_set.size(); ++to_pose_index )
                        {
                            if( mo_space->equal_points(end_pose->grasped_set[to_pose_index], transfer_query->found_goal_state, PRX_DISTANCE_CHECK) )
                                break;
                        }

                        constraints = transit_query->constraints[0];
                        constraints.insert(transfer_query->constraints[0].begin(), transfer_query->constraints[0].end());
                        full_constraints = transit_query->full_constraints[0];
                        full_constraints.insert(transfer_query->full_constraints[0].begin(), transfer_query->full_constraints[0].end());
                        transfer_full_constraints = transfer_query->full_constraints[0];
                        ordered_constraints.insert(ordered_constraints.end(), transit_query->constraints_in_order[0].begin(), transit_query->constraints_in_order[0].end());

                        foreach(unsigned c, transfer_query->constraints_in_order[0])
                        {
                            if( std::find(ordered_constraints.begin(), ordered_constraints.end(), c) == ordered_constraints.end() )
                            {
                                ordered_constraints.push_back(c);
                            }
                        }
                        return std::make_pair(from_pose_index, to_pose_index);
                    }
                }
                return std::make_pair(-1,-1);
            }


            std::pair<int, int> nmrs_t::get_union_path2(std::set<unsigned>& constraints, std::set<unsigned>& full_constraints, std::set<unsigned>& transfer_full_constraints, std::vector<unsigned>& ordered_constraints, state_t* from_state, pose_t* start_pose, pose_t* end_pose)
            {
                unsigned s_size = start_pose->ungrasped_set.size();
                unsigned e_size = end_pose->ungrasped_set.size();


                for( unsigned s = 0; s < s_size; ++s )
                {
                    //PRX_DEBUG_COLOR("Start the transit shortest path." , PRX_TEXT_BROWN);
                    transit_query->clear();
                    transit_query->copy_start(from_state);
                    transit_query_goal->copy_goal_state(start_pose->ungrasped_set[s]);
                    // fix_transit_query();                    

                    // PRX_ASSERT(transit_query_goal->get_goal_points().size() != 0);
                    // //PRX_DEBUG_COLOR("f: " << f + 1 << "/" << f_size << "  s:" << s + 1 << "/" << s_size, PRX_TEXT_CYAN);                    
                    // transit_query->restart();                    
                    PRX_ASSERT(transit_query_goal->size() != 0);
                    // transit_query->copy_start(from_state);
                    // transit_query_goal->copy_goal_state(start_pose->ungrasped_set[s]);
                    manip_tp->link_query(transit_query);
                    PRX_DEBUG_COLOR("=========================== A* union_path transit  ===========================", PRX_TEXT_BROWN);
                    manip_tp->resolve_query();
                    PRX_ASSERT(transit_query->plans.size() == 1);
                    if( transit_query->found_path )
                    {
                        PRX_ASSERT(transit_query->found_path && transit_query->plans[0]->size() != 0);
                        PRX_ASSERT(transit_query->found_path == (transit_query->plans[0]->size() != 0));
                        //                            PRX_DEBUG_COLOR("-------------------------------------------", PRX_TEXT_BLUE);
                        //                            PRX_DEBUG_COLOR("\tFound TRANSIT: " << transit_query->plans[0]->size(), PRX_TEXT_LIGHTGRAY);
                        //                            PRX_DEBUG_COLOR("-------------------------------------------", PRX_TEXT_BLUE);

                        for( unsigned e = 0; e < e_size; ++e )
                        {
                            transfer_query->clear();
                            transfer_query->copy_start(start_pose->grasped_set[s]);
                            transfer_query_goal->copy_goal_state(end_pose->grasped_set[e]);
                            // fix_transfer_query();

                            // transfer_query->restart();
                            manip_tp->link_query(transfer_query);
                            PRX_DEBUG_COLOR("=========================== A* union_path transfer  ===========================", PRX_TEXT_BROWN);
                            manip_tp->resolve_query();

                            if( transfer_query->found_path )
                            {
                                PRX_ASSERT(transfer_query->found_path == (transfer_query->plans[0]->size() != 0));
                                //                                    PRX_DEBUG_COLOR("-------------------------------------------", PRX_TEXT_BLUE);
                                //                                    PRX_DEBUG_COLOR("\tFound TRANSFER: " << transfer_query->plans[0]->size(), PRX_TEXT_LIGHTGRAY);
                                //                                    PRX_DEBUG_COLOR("-------------------------------------------", PRX_TEXT_BLUE);
                                PRX_ASSERT(transit_query->plans[0]->size() != 0);
                                PRX_ASSERT(transfer_query->plans[0]->size() != 0);

                                constraints = transit_query->constraints[0];
                                constraints.insert(transfer_query->constraints[0].begin(), transfer_query->constraints[0].end());
                                full_constraints = transit_query->full_constraints[0];
                                full_constraints.insert(transfer_query->full_constraints[0].begin(), transfer_query->full_constraints[0].end());
                                transfer_full_constraints = transfer_query->full_constraints[0];
                                ordered_constraints.insert(ordered_constraints.end(), transit_query->constraints_in_order[0].begin(), transit_query->constraints_in_order[0].end());

                                foreach(unsigned c, transfer_query->constraints_in_order[0])
                                {
                                    if( std::find(ordered_constraints.begin(), ordered_constraints.end(), c) == ordered_constraints.end() )
                                    {
                                        ordered_constraints.push_back(c);
                                    }
                                }
                                return std::make_pair(s, e);
                            }
                            // else
                            // {
                            //     PRX_DEBUG_COLOR("Start the transfer shortest path." , PRX_TEXT_BROWN);
                            //     transfer_query->clear();                                
                            //     transfer_query->copy_start(start_pose->grasped_set[s]);
                            //     transfer_query_goal->copy_goal_state(end_pose->grasped_set[e]);
                            //     transfer_query->q_collision_type = motion_planning_query_t::PRX_LAZY_COLLISIONS;
                            //     transfer_astar->set_shortest_path_flag(true);
                            //     transfer_astar->set_minimum_conflict(false);
                            //     transfer_astar->set_max_length(PRX_INFINITY);
                            //     manip_tp->link_query(transfer_query);
                            //     manip_tp->resolve_query();
                            //     if( transfer_query->found_path )
                            //     {
                            //         PRX_DEBUG_COLOR("\tFound SHORTEST TRANSFER: " << transfer_query->plans[0]->size() << "    |C|:" << transfer_query->constraints[0].size() << "     |C_f|: " << transfer_query->full_constraints[0].size() << "  C_f:" << print(transfer_query->full_constraints[0]), PRX_TEXT_LIGHTGRAY);
                            //     }
                            //     else
                            //     {
                            //         PRX_DEBUG_COLOR("NO SHORTEST path!!", PRX_TEXT_MAGENTA);
                            //     }
                            //     transfer_query->q_collision_type = motion_planning_query_t::PRX_ACTIVE_COLLISIONS_REUSE_EDGES;
                            //     transfer_astar->set_shortest_path_flag(false);
                            //     transfer_astar->set_minimum_conflict(true);
                            //     transfer_astar->set_max_length(PRX_INFINITY);
                            // }
                        }
                    }
                    // else
                    // {
                    //     PRX_DEBUG_COLOR("Start the transit shortest path." , PRX_TEXT_BROWN);
                    //     transit_query->clear();
                    //     transit_query->copy_start(from_state);
                    //     transit_query_goal->copy_goal_state(start_pose->ungrasped_set[s]);
                    //     transit_query->q_collision_type = motion_planning_query_t::PRX_LAZY_COLLISIONS;
                    //     transit_astar->set_shortest_path_flag(true);
                    //     transit_astar->set_minimum_conflict(false);
                    //     transit_astar->set_max_length(PRX_INFINITY);
                    //     manip_tp->link_query(transit_query);
                    //     manip_tp->resolve_query();
                    //     if( transit_query->found_path )
                    //     {
                    //         PRX_DEBUG_COLOR("\tFound SHORTEST TRANSIT: " << transit_query->plans[0]->size() << "    |C|:" << transit_query->constraints[0].size() << "     |C_f|: " << transit_query->full_constraints[0].size() << "  C_f:" << print(transfer_query->full_constraints[0]), PRX_TEXT_LIGHTGRAY);
                    //     }
                    //     else
                    //     {
                    //         PRX_DEBUG_COLOR("NO SHORTEST path!!", PRX_TEXT_MAGENTA);
                    //     }
                    //     transit_query->q_collision_type = motion_planning_query_t::PRX_ACTIVE_COLLISIONS_REUSE_EDGES;
                    //     transit_astar->set_shortest_path_flag(false);
                    //     transit_astar->set_minimum_conflict(true);
                    //     transit_astar->set_max_length(PRX_INFINITY);
                    // }
                }

                return std::make_pair(-1, -1);
            }



            bool nmrs_t::intermediate_pose(path_part_t& part, std::set<unsigned>& path_constraints, std::set<unsigned>& transfer_full_constraints, std::vector<unsigned>& ordered_constraints, unsigned object_id, state_t* start_state, const std::deque<unsigned>& future, const std::vector<unsigned>& a_curr, const std::vector<unsigned>& target_arrangement, const std::set<unsigned>& constraints)
            {
                fix_constraints(object_id, a_curr, future, constraints);
                std::vector<pose_t*> free_poses;
                get_valid_intermediate_poses(free_poses, a_curr, future, constraints);

                if( free_poses.size() > 0 )
                {
                    transit_query->clear();
                    transit_query->copy_start(start_state);
                    transit_query_goal->copy_multiple_goal_states(poses_set[a_curr[object_id]].ungrasped_set);

                    // foreach(state_t* st, poses_set[a_curr[object_id]].ungrasped_set)
                    // {
                    //     transit_query_goal->add_goal_state(st);
                    // }
                    // fix_transit_query();

                    // transit_query->restart();
                    PRX_DEBUG_COLOR("=========================== A* intermediate_pose transit  ===========================", PRX_TEXT_BROWN);
                    manip_tp->link_query(transit_query);
                    manip_tp->resolve_query();
                    if( transit_query->found_path )
                    {
#ifndef NDEBUG
                        PRX_ASSERT(transit_query->found_path && transit_query->plans[0]->size() != 0);
                        PRX_DEBUG_COLOR("------------------------------------------------------", PRX_TEXT_BLUE);
                        PRX_DEBUG_COLOR("\tFound TRANSIT: size |" << transit_query->plans[0]->size() << "|", PRX_TEXT_LIGHTGRAY);
                        PRX_DEBUG_COLOR(" Constraints      : " << print(transit_query->constraints[0]), PRX_TEXT_LIGHTGRAY);
                        PRX_DEBUG_COLOR(" Full Constraints : " << print(transit_query->full_constraints[0]), PRX_TEXT_LIGHTGRAY);
                        PRX_DEBUG_COLOR(" Order Constraints: " << print(transit_query->constraints_in_order[0]), PRX_TEXT_LIGHTGRAY);
                        PRX_DEBUG_COLOR("------------------------------------------------------", PRX_TEXT_BLUE);
                        bool check = false;

                        foreach(state_t* st, poses_set[a_curr[object_id]].ungrasped_set)
                        {
                            if( manip_state_space->equal_points(st, transit_query->found_goal_state, PRX_DISTANCE_CHECK) )
                                check = true;
                        }
                        PRX_ASSERT(check);
#endif
                        transfer_query->clear();
                        unsigned start_index;
                        for( start_index = 0; start_index < poses_set[a_curr[object_id]].ungrasped_set.size(); ++start_index )
                        {
                            if( manip_state_space->equal_points(poses_set[a_curr[object_id]].ungrasped_set[start_index], transit_query->found_goal_state, PRX_DISTANCE_CHECK) )
                            {
                                transfer_query->copy_start(poses_set[a_curr[object_id]].grasped_set[start_index]);
                                break;
                            }
                        }

                        transfer_query_goal->clear();
                        foreach(pose_t* pose, free_poses)
                        {
                            PRX_DEBUG_COLOR("-- Pose:" << pose->pose_id << "  will add " << pose->grasped_set.size() << " new points", PRX_TEXT_GREEN);
                            transfer_query_goal->append_multiple_goal_states(pose->grasped_set);
                        }
                        // fix_transfer_query();

                        // transfer_query->restart();
                        manip_tp->link_query(transfer_query);    
                        PRX_DEBUG_COLOR("=========================== A* intermediate_pose transfer  ===========================", PRX_TEXT_BROWN);                    
                        manip_tp->resolve_query();
                        if( transfer_query->found_path )
                        {
                            int index_p, index_e;
                            boost::tie(index_p, index_e) = detect_pose(free_poses, transfer_query->found_goal_state);
                            PRX_ASSERT(index_p != -1 && index_e != -1);
                            index_p = free_poses[index_p]->pose_id;
                            PRX_DEBUG_COLOR("pose that worked: " << index_p << "   grasping index: " << index_e, PRX_TEXT_GREEN);

                            PRX_DEBUG_COLOR("-------------------------------------------", PRX_TEXT_BLUE);
                            PRX_DEBUG_COLOR("\tFound Intermediate TRANSFER size: " << transfer_query->plans[0]->size() << "   to pose: " << index_p, PRX_TEXT_LIGHTGRAY);
                            PRX_DEBUG_COLOR(" Constraints     : " << print(transfer_query->constraints[0]), PRX_TEXT_LIGHTGRAY);
                            PRX_DEBUG_COLOR(" Full Constraints: " << print(transfer_query->full_constraints[0]), PRX_TEXT_LIGHTGRAY);
                            PRX_DEBUG_COLOR(" transfer_query_goal size:" << transfer_query_goal->size(), PRX_TEXT_LIGHTGRAY);
                            PRX_DEBUG_COLOR("-------------------------------------------", PRX_TEXT_BLUE);

                            PRX_ASSERT(transit_query->plans[0]->size() != 0);
                            PRX_ASSERT(transfer_query->plans[0]->size() != 0);

                            path_constraints = transfer_query->constraints[0];
                            path_constraints.erase(start_index);
                            path_constraints.insert(transit_query->constraints[0].begin(), transit_query->constraints[0].end());
                            if( std::find(a_curr.begin(), a_curr.end(), index_p) == a_curr.end() )
                                path_constraints.erase((unsigned)index_p);

//                            full_constraints = transit_query->full_constraints[0];
//                            full_constraints.insert(transfer_query->full_constraints[0].begin(), transfer_query->full_constraints[0].end());
                            transfer_full_constraints = transit_query->full_constraints[0];
                            transfer_full_constraints.insert(transfer_query->full_constraints[0].begin(), transfer_query->full_constraints[0].end());
                            // transfer_full_constraints = transfer_query->full_constraints[0];
                            ordered_constraints = transit_query->constraints_in_order[0];
                            ordered_constraints.insert(ordered_constraints.end(), transfer_query->constraints_in_order[0].begin(), transfer_query->constraints_in_order[0].end());
                            part.reset(object_id, a_curr[object_id], index_p, start_index, index_e);
                            return true;

                        }
                    }
                }
                return false;
            }

            std::pair<int, int> nmrs_t::detect_pose(const std::vector<pose_t*>& poses, state_t * state)
            {
                for( unsigned p = 0; p < poses.size(); ++p )
                {
                    for( unsigned i = 0; i < poses[p]->grasped_set.size(); ++i )
                    {
                        if( mo_space->equal_points(poses[p]->grasped_set[i], state, PRX_DISTANCE_CHECK) )
                        {
                            return std::make_pair(p, i);
                        }
                    }
                }
                return std::make_pair(-1, -1);
            }

            unsigned nmrs_t::detect_blocker(unsigned object_id, const std::vector<unsigned>& a_curr, const std::vector<unsigned>& constraints)
            {
                PRX_DEBUG_COLOR("Acurr: " << print(a_curr), PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR("constraints: " << print(constraints), PRX_TEXT_CYAN);
                //Checks for the first blocker in the ordered list of constraints. We are certain that there is blocker at this point. 
                for( unsigned i = 0; i < a_curr.size(); ++i )
                    if( a_curr[i] == constraints[0] )
                        return i;
                PRX_FATAL_S("There should be a blocker!");
                return 0;
            }

            void nmrs_t::get_valid_intermediate_poses(std::vector<pose_t*>& free_poses, const std::vector<unsigned>& a_curr, const std::deque<unsigned>& future, const std::set<unsigned>& constraints)
            {
                std::set<unsigned> full_constraints;
                full_constraints.insert(a_curr.begin(), a_curr.end());
                std::vector<unsigned> common;

                //The objects that are still in the future list can be used as intermediate poses. 
                for( unsigned i = 0; i < future.size(); ++i )
                {
                    full_constraints.erase(a_curr[future[i]]);
                }
                full_constraints.insert(constraints.begin(), constraints.end());

#ifndef NDEBUG
                PRX_DEBUG_COLOR("--------------------------------------------------------------------------------", PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR("   Get Valid Intermediate_poses: acurr: " << print(a_curr) << "   future:" << print(future), PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR("   Constraints     : " << print(constraints), PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR("   Full Constraints: " << print(full_constraints), PRX_TEXT_GREEN);
#endif


                for( unsigned i = 0; i < poses_set_length; ++i )
                {
                    if( full_constraints.count(i) == 0 )
                    {
                        //Some poses are overlapping with other poses. The constraints of these poses have also to be free. 
                        common.clear();
                        std::set_intersection(poses_set[i].constraints.begin(), poses_set[i].constraints.end(), full_constraints.begin(), full_constraints.end(), std::back_inserter(common));
                        if( common.size() == 0 )
                            free_poses.push_back(&poses_set[i]);
                    }
                }
#ifndef NDEBUG
                PRX_DEBUG_COLOR("   The free poses: " << print(free_poses), PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("--------------------------------------------------------------------------------", PRX_TEXT_GREEN);
#endif
            }

            void nmrs_t::fix_transit_query()
            {
                transit_query->q_collision_type = motion_planning_query_t::PRX_LAZY_COLLISIONS;
                transit_astar->set_shortest_path_flag(true);
                transit_astar->set_minimum_conflict(false);
                transit_astar->set_max_length(PRX_INFINITY);
                manip_tp->link_query(transit_query);
                PRX_DEBUG_COLOR("=========================== A* short transit  ===========================", PRX_TEXT_BROWN);
                manip_tp->resolve_query();
//                if( transit_query->found_path )
//                {
//                    PRX_DEBUG_COLOR("\tFound SHORTEST TRANSIT: " << transit_query->plans[0]->size() << "    |C|:" << transit_query->constraints[0].size() << "     |C_f|: " << transit_query->full_constraints[0].size() << "  C_f:" << print(transfer_query->full_constraints[0]), PRX_TEXT_LIGHTGRAY);
//                }
//                else
//                {
//                    PRX_DEBUG_COLOR("NO SHORTEST path!!", PRX_TEXT_MAGENTA);
//                }
                transit_query->q_collision_type = motion_planning_query_t::PRX_ACTIVE_COLLISIONS_REUSE_EDGES;
                transit_astar->set_shortest_path_flag(false);
                transit_astar->set_minimum_conflict(true);
                transit_astar->set_max_length(length_multiplier * transit_astar->get_path_cost());
            }

            void nmrs_t::fix_transfer_query()
            {
                transfer_query->q_collision_type = motion_planning_query_t::PRX_LAZY_COLLISIONS;
                transfer_astar->set_shortest_path_flag(true);
                transfer_astar->set_minimum_conflict(false);
                transfer_astar->set_max_length(PRX_INFINITY);
                manip_tp->link_query(transfer_query);
                PRX_DEBUG_COLOR("=========================== A* short transfer  ===========================", PRX_TEXT_BROWN);
                manip_tp->resolve_query();
//                if( transfer_query->found_path )
//               {
//                    PRX_DEBUG_COLOR("\tFound SHORTEST TRANSFER: " << transfer_query->plans[0]->size() << "    |C|:" << transfer_query->constraints[0].size() << "     |C_f|: " << transfer_query->full_constraints[0].size() << "  C_f:" << print(transfer_query->full_constraints[0]), PRX_TEXT_LIGHTGRAY);
//                }
//                else
//                {
//                    PRX_DEBUG_COLOR("NO SHORTEST path!!", PRX_TEXT_MAGENTA);
//                }
                transfer_query->q_collision_type = motion_planning_query_t::PRX_ACTIVE_COLLISIONS_REUSE_EDGES;
                transfer_astar->set_shortest_path_flag(false);
                transfer_astar->set_minimum_conflict(true);
                transfer_astar->set_max_length(length_multiplier * transfer_astar->get_path_cost());
            }
        }
    }
}
