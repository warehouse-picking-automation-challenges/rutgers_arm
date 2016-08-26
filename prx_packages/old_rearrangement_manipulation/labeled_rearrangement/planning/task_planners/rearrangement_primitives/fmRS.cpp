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

#include "planning/task_planners/rearrangement_primitives/fmRS.hpp"
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


PLUGINLIB_EXPORT_CLASS(prx::packages::labeled_rearrangement_manipulation::fmrs_t, prx::plan::planner_t)

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

            fmrs_t::fmrs_t() { }

            fmrs_t::~fmrs_t() { }

            void fmrs_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
            {
                rearrangement_primitive_t::init(reader, template_reader);
                exact_method = parameters::get_attribute_as<bool>("exact_method", reader, template_reader, false);
                multi_start = parameters::get_attribute_as<bool>("multi_start", reader, template_reader, true);
            }

            void fmrs_t::setup()
            {
                rearrangement_primitive_t::setup();
                from_safe_state.push_back(safe_state);


            }

            bool fmrs_t::solve()
            {
                return fmrs(in_query->path_sequence, in_query->initial_poses_ids, in_query->target_poses_ids);
            }

            bool fmrs_t::fmrs(std::deque<path_part_t>& sequence, const std::vector<unsigned>& a_curr, const std::vector<unsigned>& target_arrangement)
            {

                if( statistics_clock.measure() > in_query->time_limit )
                {
                    time_ends = true;
                    return false;
                }

                transit_obstacles.clear();
                transfer_obstacles.clear();
                transit_constraints.clear();
                transit_constraints.insert(a_curr.begin(), a_curr.end());
                transit_constraints.insert(target_arrangement.begin(), target_arrangement.end());
                transfer_constraints = transit_constraints;

                util::directed_graph_t constraint_graph;
                try
                {
                    std::vector<unsigned> ids(k_objects);
                    for( unsigned i = 0; i < k_objects; ++i )
                        ids[i] = i;
                    if( construct_constraint_graph(constraint_graph, ids, a_curr, target_arrangement) )
                    {
                        PRX_DEBUG_COLOR("Constraint_graph : |V|" << boost::num_vertices(constraint_graph.graph) << "   |E|:" << boost::num_edges(constraint_graph.graph), PRX_TEXT_LIGHTGRAY);
                        std::vector<directed_vertex_index_t> order;

                        boost::topological_sort(constraint_graph.graph, std::back_inserter(order));

                        foreach(directed_vertex_index_t v, order)
                        {
                            constraints_node_t* node = constraint_graph.get_vertex_as<constraints_node_t > (v);
                            PRX_DEBUG_COLOR("object: " << node->node_id << "  has plan:" << node->has_plan(), PRX_TEXT_GREEN);
                            if( node->has_plan() )
                                sequence.push_back(node->part);
                        }
                        return true;
                    }
                }
                catch( boost::not_a_dag e )
                {
                    PRX_DEBUG_COLOR("Graph is not a DAG!", PRX_TEXT_RED);
                }
                if( in_query->accept_partial_solutions )
                    partial_fmrs(sequence, constraint_graph, a_curr, target_arrangement);
                return false;
            }

            bool fmrs_t::partial_fmrs(std::deque<path_part_t>& sequence, directed_graph_t constraint_graph, const std::vector<unsigned>& a_curr, const std::vector<unsigned>& target_arrangement)
            {
                in_query->got_partial_solution = false;

                int num = boost::strong_components(constraint_graph.graph, constraint_graph.components);
                if( num == 1 )
                    return false;
                directed_graph_t super_constraints_graph;
                std::vector<super_constraints_node_t*> nodes;
                std::vector<unsigned> curr_pos = a_curr;
                for( int i = 0; i < num; ++i )
                {
                    directed_vertex_index_t v = super_constraints_graph.add_vertex<super_constraints_node_t>();
                    super_constraints_graph.get_vertex_as<super_constraints_node_t>(v)->node_id = i;
                    nodes.push_back(super_constraints_graph.get_vertex_as<super_constraints_node_t>(v));
                }

                foreach(directed_vertex_index_t v, boost::vertices(constraint_graph.graph))
                {
                    nodes[constraint_graph.components[v]]->add_to_cycle(constraint_graph.get_vertex_as<constraints_node_t>(v));
                    PRX_DEBUG_COLOR("The vertex : " << constraint_graph[v]->node_id << "   is in the component: " << constraint_graph.components[v], PRX_TEXT_MAGENTA);
                }

#ifndef NDEBUG

                foreach(directed_vertex_index_t v, boost::vertices(super_constraints_graph.graph))
                {
                    PRX_DEBUG_COLOR(super_constraints_graph.get_vertex_as<super_constraints_node_t>(v)->print(), PRX_TEXT_GREEN);
                }
#endif

                std::vector<directed_vertex_index_t> order;
                get_super_order(order, super_constraints_graph);
                PRX_DEBUG_COLOR("Got the super order size of: " << order.size(), PRX_TEXT_BROWN);
                path_part_t part;

                foreach(directed_vertex_index_t v, order)
                {
                    super_constraints_node_t* node = super_constraints_graph.get_vertex_as<super_constraints_node_t>(v);
                    PRX_DEBUG_COLOR("Try to move node: " << node->node_id << "   is it cycle:" << node->is_cycle_node, PRX_TEXT_BLUE);
                    if( node->is_cycle_node )
                    {

                        //                        transit_constraints.clear();
                        //                        transfer_constraints.clear();
                        //                        transit_obstacles.clear();
                        //                        transit_obstacles.insert(curr_pos.begin(), curr_pos.end());
                        //
                        //                        foreach(unsigned t, node->starts)
                        //                        {
                        //                            PRX_ASSERT(transit_obstacles.count(t) > 0);
                        //                            transit_obstacles.erase(t);
                        //                        }
                        //                        transfer_obstacles = transit_obstacles;
                        //
                        //                        curr_order.resize(node->ids.size());
                        //                        max_depth = -1;
                        //                        curr_depth = -1;
                        //
                        //                        std::deque<unsigned> future;
                        //                        for( unsigned i = 0; i < k_objects; ++i )
                        //                        {
                        //                            future.push_back(i);
                        //                        }
                        //
                        //                        std::vector<unsigned> curr_targets = curr_pos;
                        //                        for( unsigned i = 0; i < node->ids.size(); ++i )
                        //                        {
                        //                            curr_targets[node->ids[i]] = node->targets[i];
                        //                        }
                        //
                        //                        transit_astar->set_minimum_conflict(false);
                        //                        transit_astar->set_shortest_path_flag(true);
                        //                        transfer_astar->set_minimum_conflict(false);
                        //                        transfer_astar->set_shortest_path_flag(true);
                        //                        bool found_solution = false;
                        //                        for( unsigned i = 0; i < k_objects && !time_ends; ++i )
                        //                        {
                        //                            unsigned id = future[0];
                        //                            //                                        PRX_DEBUG_COLOR("-------------------------------------------", PRX_TEXT_CYAN);
                        //                            //                                        PRX_DEBUG_COLOR("--- STARTS With : " << id << "   POSE: " << a_curr[i] << " ---", PRX_TEXT_GREEN);
                        //                            //                                        PRX_DEBUG_COLOR("-------------------------------------------", PRX_TEXT_CYAN);
                        //                            future.pop_front();
                        //                            if( mrs(sequence, id, from_safe_state, poses_set[curr_targets[id]], future, curr_pos, curr_targets) )
                        //                            {
                        //                                found_solution = true;
                        //                                for( unsigned i = 0; i < node->ids.size(); ++i )
                        //                                {
                        //                                    curr_pos[node->ids[i]] = node->targets[i];
                        //                                }
                        //                                in_query->got_partial_solution = true;
                        //                                break;
                        //                            }
                        //                            PRX_ASSERT(curr_depth == -1);
                        //                            future.push_back(id);
                        //                        }
                        //
                        //                        if( !found_solution && max_depth != -1 )
                        //                        {
                        //
                        //                            PRX_DEBUG_COLOR("------------------------------", PRX_TEXT_BROWN);
                        //                            PRX_DEBUG_COLOR("------ PARTIAL mRS SOLUTION " << max_depth << "----", PRX_TEXT_RED);
                        //                            PRX_DEBUG_COLOR("------------------------------", PRX_TEXT_BROWN);
                        //                            in_query->got_partial_solution = true;
                        //                            for( int i = 0; i <= max_depth; ++i )
                        //                            {
                        //                                sequence.push_back(best_order[i]);
                        //                                curr_pos[best_order[i].id] = best_order[i].pose_to;
                        //                                PRX_DEBUG_COLOR("order (" << i << "/" << max_depth << "):" << best_order[i].print(), PRX_TEXT_GREEN);
                        //                            }
                        //                        }
                        //
                        //                        transit_astar->set_minimum_conflict(true);
                        //                        transit_astar->set_shortest_path_flag(false);
                        //                        transfer_astar->set_minimum_conflict(true);
                        //                        transfer_astar->set_shortest_path_flag(false);

                        transit_constraints.clear();
                        transfer_constraints.clear();
                        transit_obstacles.clear();
                        transit_obstacles.insert(curr_pos.begin(), curr_pos.end());

                        transit_constraints.insert(node->starts.begin(), node->starts.end());
                        transit_constraints.insert(node->targets.begin(), node->targets.end());

                        foreach(unsigned t, node->starts)
                        {
                            PRX_ASSERT(transit_obstacles.count(t) > 0);
                            transit_obstacles.erase(t);
                        }
                        transfer_constraints = transit_constraints;
                        transfer_obstacles = transit_obstacles;



                        directed_graph_t cycle_graph;
                        try
                        {
                            if( construct_constraint_graph(cycle_graph, node->ids, curr_pos, target_arrangement) )
                            {
                                std::vector<directed_vertex_index_t> order;

                                boost::topological_sort(cycle_graph.graph, std::back_inserter(order));

                                foreach(directed_vertex_index_t v, order)
                                {
                                    constraints_node_t* c_node = cycle_graph.get_vertex_as<constraints_node_t > (v);
                                    PRX_DEBUG_COLOR("PARTIAL object: " << c_node->node_id << "  has_plan:" << c_node->has_plan() << "   is_constrained:" << c_node->is_constrained(curr_pos), PRX_TEXT_GREEN);
                                    PRX_DEBUG_COLOR("curr pos : " << print(curr_pos), PRX_TEXT_GREEN);
                                    if( c_node->has_plan() && !c_node->is_constrained(curr_pos) )
                                    {
                                        sequence.push_back(c_node->part);
                                        curr_pos[c_node->node_id] = target_arrangement[c_node->node_id];
                                        in_query->got_partial_solution = true;
                                    }
                                }
                            }
                        }
                        catch( boost::not_a_dag e )
                        {
                            PRX_DEBUG_COLOR("The Cycle Graph is not a DAG!", PRX_TEXT_RED);
                        }

                    }
                    else
                    {
                        transit_constraints.clear();
                        transfer_constraints.clear();
                        transit_obstacles.clear();
                        transit_obstacles.insert(curr_pos.begin(), curr_pos.end());
                        transfer_obstacles = transit_obstacles;
                        transfer_obstacles.erase(curr_pos[node->ids[0]]);
                        part.clear();
                        PRX_ASSERT(target_arrangement[node->ids[0]] == node->targets[0]);
                        PRX_ASSERT(curr_pos[node->ids[0]] == node->starts[0]);
                        part.init(node->ids[0], node->starts[0], node->targets[0]);
                        PRX_DEBUG_COLOR("The node " << node->node_id << " is not a cycle going to: " << node->starts[0] << " -> " << node->targets[0], PRX_TEXT_CYAN);
                        if( get_path(part, safe_state, &poses_set[node->starts[0]], &poses_set[node->targets[0]]) )
                        {

                            curr_pos[node->ids[0]] = target_arrangement[node->ids[0]];
                            sequence.push_back(part);
                            in_query->got_partial_solution = true;
                        }



                    }
                    //                    PRX_DEBUG_COLOR("object: " << constraint_graph.get_vertex_as<directed_node_t > (v)->node_id, PRX_TEXT_GREEN);
                    //                    int index = constraint_graph.get_vertex_as<directed_node_t > (v)->node_id;
                    //                    if( plan_parts[index].plan.size() > 0 )
                    //                        sequence.push_back(plan_parts[index]);
                }

                if( in_query->got_partial_solution )
                {
                    PRX_DEBUG_COLOR("------------------------------", PRX_TEXT_BROWN);
                    PRX_DEBUG_COLOR("------ PARTIAL SOLUTION : " << print(curr_pos) << " ----", PRX_TEXT_RED);
                    PRX_DEBUG_COLOR("------------------------------", PRX_TEXT_BROWN);
                    in_query->partial_solution = curr_pos;
                }
                return false;
            }

            bool fmrs_t::get_path(path_part_t& part, sim::state_t* safe_state, pose_t* start_pose, pose_t* end_pose)
            {
                transfer_extra_start_states.clear();
                transfer_query->clear();
                transfer_query->copy_start(start_pose->grasped_set[0]);

                for( unsigned i = 1; i < start_pose->grasped_set.size(); ++i )
                {
                    transfer_extra_start_states.push_back(start_pose->grasped_set[i]);
                }

                transfer_query_goal->copy_multiple_goal_states(end_pose->grasped_set);
                // foreach(state_t* st, end_pose->grasped_set)
                // {
                //     transfer_query_goal->add_goal_state(st);
                // }
                manip_tp->link_query(transfer_query);
                manip_tp->resolve_query();
                if( transfer_query->found_path )
                {
                    part.update(transfer_query->constraints[0], detect_grasped_state(start_pose, transfer_query->found_start_state), detect_grasped_state(end_pose, transfer_query->found_goal_state));

                    transit_query->clear();
                    transit_query->copy_start(safe_state);
                    transit_query_goal->copy_goal_state(start_pose->ungrasped_set[part.index_from]);
                    manip_tp->link_query(transit_query);
                    manip_tp->resolve_query();

                    if( transit_query->found_path )
                    {
                        part.constraints.insert(transit_query->constraints[0].begin(), transit_query->constraints[0].end());

                        //                        PRX_ASSERT(transit_constraints.count(part.pose_from) > 0);
                        //                        PRX_ASSERT(transit_constraints.count(part.pose_to) == 0);
                        transit_constraints.erase(part.pose_from);
                        transit_constraints.insert(part.pose_to);

                        transit_query->clear();
                        transit_query->copy_start(end_pose->ungrasped_set[part.index_to]);
                        transit_query_goal->copy_goal_state(safe_state);
                        manip_tp->link_query(transit_query);
                        manip_tp->resolve_query();

                        if( transit_query->found_path )
                        {
                            part.constraints.insert(transit_query->constraints[0].begin(), transit_query->constraints[0].end());
                            part.has_plan = true;

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
                }
                return false;
            }

            bool fmrs_t::construct_constraint_graph(directed_graph_t& graph, const std::vector<unsigned>& objects_ids, const std::vector<unsigned>& a_curr, const std::vector<unsigned>& target_arrangement)
            {

                foreach(unsigned i, objects_ids)
                {
                    directed_vertex_index_t new_vertex = graph.add_vertex<constraints_node_t > ();
                    constraints_node_t* node = graph.get_vertex_as<constraints_node_t > (new_vertex);
                    node->index = new_vertex;
                    node->node_id = i;
                    node->part.init(i, a_curr[i], target_arrangement[i]);
                    PRX_DEBUG_COLOR("============================================================", PRX_TEXT_BROWN);
                    PRX_DEBUG_COLOR("  GOING TO MOVE object: " << i << "  from pose: " << a_curr[i] << "   to pose: " << target_arrangement[i], PRX_TEXT_LIGHTGRAY);
                    PRX_DEBUG_COLOR("============================================================", PRX_TEXT_BROWN);
                    if( a_curr[i] != target_arrangement[i] )
                    {
                        transfer_constraints.erase(target_arrangement[i]);
                        transfer_constraints.erase(a_curr[i]);
                        transit_constraints.erase(target_arrangement[i]);
                        get_path(node->part, safe_state, &poses_set[a_curr[i]], &poses_set[target_arrangement[i]]);

                        if( std::find(a_curr.begin(), a_curr.end(), target_arrangement[i]) != a_curr.end() )
                        {
//                            PRX_ASSERT(node->part.constraints.count(target_arrangement[i]) == 0);
                            node->part.constraints.insert(target_arrangement[i]);
                        }

                        transfer_constraints.insert(target_arrangement[i]);
                        transfer_constraints.insert(a_curr[i]);
                        transit_constraints.insert(target_arrangement[i]);
                        transit_constraints.insert(a_curr[i]);
                    }
                }

                foreach(directed_vertex_index_t v, boost::vertices(graph.graph))
                {
                    constraints_node_t* node = graph.get_vertex_as<constraints_node_t > (v);
                    PRX_DEBUG_COLOR("object " << node->node_id << "  with constraints: " << print(node->part.constraints), PRX_TEXT_GREEN);

                    foreach(directed_vertex_index_t u, boost::vertices(graph.graph))
                    {
                        if( v != u )
                        {
                            constraints_node_t* u_node = graph.get_vertex_as<constraints_node_t > (u);
                            if( node->part.constraints.count(u_node->part.pose_from) > 0 && !boost::edge(v, u, graph.graph).second )
                            {
                                PRX_DEBUG_COLOR("ADD EDGE : " << node->node_id << " -> " << u_node->node_id, PRX_TEXT_CYAN);
                                graph.add_edge<directed_edge_t > (v, u);
                            }
                            if( node->part.constraints.count(u_node->part.pose_to) > 0 && !boost::edge(u, v, graph.graph).second )
                            {
                                PRX_DEBUG_COLOR("ADD EDGE : " << u_node->node_id << " -> " << node->node_id, PRX_TEXT_CYAN);
                                graph.add_edge<directed_edge_t > (u, v);
                            }

                            if( !in_query->accept_partial_solutions && boost::edge(v, u, graph.graph).second && boost::edge(u, v, graph.graph).second )
                            {
                                PRX_DEBUG_COLOR("This problem DOES NOT have a solution!", PRX_TEXT_RED);
                                return false;
                            }
                        }
                    }
                }
                return true;
            }

            bool fmrs_t::get_super_order(std::vector<directed_vertex_index_t>& order, directed_graph_t& graph)
            {

                foreach(directed_vertex_index_t v, boost::vertices(graph.graph))
                {
                    super_constraints_node_t* node = graph.get_vertex_as<super_constraints_node_t > (v);

                    foreach(directed_vertex_index_t u, boost::vertices(graph.graph))
                    {
                        if( v != u )
                        {
                            super_constraints_node_t* u_node = graph.get_vertex_as<super_constraints_node_t > (u);
                            if( node->interact_with_starts(u_node) )
                            {
                                PRX_DEBUG_COLOR("ADD EDGE : " << node->node_id << " -> " << u_node->node_id, PRX_TEXT_CYAN);
                                graph.add_edge<directed_edge_t > (v, u);
                            }
                            if( node->interact_with_targets(u_node) )
                            {
                                PRX_DEBUG_COLOR("ADD EDGE : " << u_node->node_id << " -> " << node->node_id, PRX_TEXT_CYAN);
                                graph.add_edge<directed_edge_t > (u, v);
                            }

                            if( boost::edge(v, u, graph.graph).second && boost::edge(u, v, graph.graph).second )
                            {
                                PRX_DEBUG_COLOR("This problem DOES NOT have a solution!", PRX_TEXT_RED);
                                PRX_FATAL_S("This should not happen in the super graph!");
                                return false;
                            }
                        }
                    }
                }

                try
                {
                    boost::topological_sort(graph.graph, std::back_inserter(order));
                }
                catch( boost::not_a_dag e )
                {
                    PRX_FATAL_S("The super graph should be a DAG!");
                    return false;
                }
                return true;
            }

            int fmrs_t::detect_released_state(pose_t* pose, state_t* state)
            {
                for( unsigned i = 0; i < pose->ungrasped_set.size(); ++i )
                {
                    if( manip_state_space->equal_points(pose->ungrasped_set[i], state, PRX_DISTANCE_CHECK) )
                        return i;
                }
                return -1;
            }

            int fmrs_t::detect_grasped_state(pose_t* pose, state_t* state)
            {
                for( unsigned i = 0; i < pose->grasped_set.size(); ++i )
                {
                    if( mo_space->equal_points(pose->grasped_set[i], state, PRX_DISTANCE_CHECK) )
                        return i;
                }
                return -1;
            }

            bool fmrs_t::get_test_path(path_part_t& part, sim::state_t* safe_state, pose_t* start_pose, pose_t* end_pose)
            {

                if( exact_method && !multi_start )
                    return get_full_path(part, safe_state, start_pose, end_pose);

                transfer_extra_start_states.clear();
                transfer_query->clear();
                transfer_query->copy_start(start_pose->grasped_set[0]);

                if( multi_start )
                {
                    for( unsigned i = 1; i < start_pose->grasped_set.size(); ++i )
                    {
                        transfer_extra_start_states.push_back(start_pose->grasped_set[i]);
                    }
                }

                transfer_query_goal->copy_multiple_goal_states(end_pose->grasped_set);
                // foreach(state_t* st, end_pose->grasped_set)
                // {
                //     transfer_query_goal->add_goal_state(st);
                // }
                manip_tp->link_query(transfer_query);
                manip_tp->resolve_query();
                PRX_ASSERT(transfer_query->found_path);
                if( multi_start )
                {
                    part.update(transfer_query->constraints[0], detect_grasped_state(start_pose, transfer_query->found_start_state), detect_grasped_state(end_pose, transfer_query->found_goal_state));
                }
                else
                {
                    part.update(transfer_query->constraints[0], 0, detect_grasped_state(end_pose, transfer_query->found_goal_state));

                    for( unsigned i = 1; i < start_pose->grasped_set.size(); ++i )
                    {
                        transfer_query->clear();
                        transfer_query->copy_start(start_pose->grasped_set[i]);

                        transfer_query_goal->copy_multiple_goal_states(end_pose->grasped_set);
                        // foreach(state_t* st, end_pose->grasped_set)
                        // {
                        //     transfer_query_goal->add_goal_state(st);
                        // }
                        manip_tp->link_query(transfer_query);
                        manip_tp->resolve_query();
                        PRX_ASSERT(transfer_query->found_path);
                        if( part.constraints.size() > transfer_query->constraints[0].size() )
                        {
                            part.update(transfer_query->constraints[0], i, detect_grasped_state(end_pose, transfer_query->found_goal_state));
                        }
                    }
                }

                if( exact_method )
                {
                    transit_query->clear();
                    transit_query->copy_start(safe_state);
                    transit_query_goal->copy_goal_state(start_pose->ungrasped_set[part.index_from]);
                    manip_tp->link_query(transit_query);
                    manip_tp->resolve_query();

                    PRX_ASSERT(transit_query->found_path);
                    part.constraints.insert(transit_query->constraints[0].begin(), transit_query->constraints[0].end());

                    PRX_ASSERT(transit_constraints.count(part.pose_from) > 0);
                    PRX_ASSERT(transit_constraints.count(part.pose_to) == 0);
                    transit_constraints.erase(part.pose_from);
                    transit_constraints.insert(part.pose_to);

                    transit_query->clear();
                    transit_query->copy_start(end_pose->ungrasped_set[part.index_to]);
                    transit_query_goal->copy_goal_state(safe_state);
                    manip_tp->link_query(transit_query);
                    manip_tp->resolve_query();

                    PRX_ASSERT(transit_query->found_path);
                    part.constraints.insert(transit_query->constraints[0].begin(), transit_query->constraints[0].end());
                }

#ifndef NDEBUG
                PRX_DEBUG_COLOR("", PRX_TEXT_RED);
                PRX_DEBUG_COLOR("========================== PATH ======================================", PRX_TEXT_RED);
                PRX_DEBUG_COLOR("====   Found path : " << part.print() << "  ====", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("===========================================================================", PRX_TEXT_RED);
                PRX_DEBUG_COLOR("", PRX_TEXT_RED);
#endif
                return true;
            }

            bool fmrs_t::get_full_path(path_part_t& part, sim::state_t* safe_state, pose_t* start_pose, pose_t* end_pose)
            {

                transit_query->clear();
                transit_query->copy_start(safe_state);
                transit_query_goal->copy_multiple_goal_states(start_pose->ungrasped_set);
                // foreach(state_t* st, start_pose->ungrasped_set)
                // {
                //     transit_query_goal->add_goal_state(st);
                // }
                manip_tp->link_query(transit_query);
                manip_tp->resolve_query();

                PRX_ASSERT(transit_query->found_path);
                part.index_from = detect_released_state(start_pose, transit_query->found_goal_state);
                PRX_ASSERT(part.index_from != -1);


                transfer_query->clear();
                transfer_query->copy_start(start_pose->grasped_set[part.index_from]);
                transfer_query_goal->copy_multiple_goal_states(end_pose->grasped_set);
                // foreach(state_t* st, end_pose->grasped_set)
                // {
                //     transfer_query_goal->add_goal_state(st);
                // }
                manip_tp->link_query(transfer_query);
                manip_tp->resolve_query();

                PRX_ASSERT(transfer_query->found_path);
                part.constraints = transit_query->constraints[0];

                PRX_ASSERT(transit_constraints.count(start_pose->pose_id) > 0);
                PRX_ASSERT(transit_constraints.count(end_pose->pose_id) == 0);
                transit_constraints.erase(start_pose->pose_id);
                transit_constraints.insert(end_pose->pose_id);

                part.index_to = detect_grasped_state(end_pose, transfer_query->found_goal_state);
                transit_query->clear();
                transit_query->copy_start(end_pose->ungrasped_set[part.index_to]);
                transit_query_goal->copy_goal_state(safe_state);
                manip_tp->link_query(transit_query);
                manip_tp->resolve_query();
                PRX_ASSERT(transit_query->found_path);

                part.constraints.insert(transfer_query->constraints[0].begin(), transfer_query->constraints[0].end());
                part.constraints.insert(transit_query->constraints[0].begin(), transit_query->constraints[0].end());

#ifndef NDEBUG
                PRX_DEBUG_COLOR("", PRX_TEXT_RED);
                PRX_DEBUG_COLOR("========================== FULL PATH ======================================", PRX_TEXT_RED);
                PRX_DEBUG_COLOR("====   Found path : " << part.print() << "  ====", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("===========================================================================", PRX_TEXT_RED);
                PRX_DEBUG_COLOR("", PRX_TEXT_RED);
#endif 


                return true;
            }




            //==========================//
            //         mRS Code         //
            //==========================//

            void fmrs_t::reset_constraints(const std::vector<unsigned>& arrangement)
            {
                transit_constraints.clear();
                transit_constraints.insert(arrangement.begin(), arrangement.end());
                transit_constraints.insert(transit_obstacles.begin(), transit_obstacles.end());
                transfer_constraints.clear();
                transfer_constraints.insert(arrangement.begin(), arrangement.end());
                transfer_constraints.insert(transfer_obstacles.begin(), transfer_obstacles.end());
            }

            bool fmrs_t::mrs(std::deque<path_part_t>& sequence, unsigned object_id, const std::vector<sim::state_t*>& from_states, pose_t& pose, std::deque<unsigned> future, std::vector<unsigned> a_curr, const std::vector<unsigned>& target_arrangement)
            {
                curr_depth++;
                PRX_ASSERT(pose.pose_id == target_arrangement[object_id]);
                if( statistics_clock.measure() > in_query->time_limit )
                {
                    curr_depth--;
                    time_ends = true;
                    return false;
                }

                bool will_stay = pose.pose_id == a_curr[object_id];

                if( will_stay || clear_to_move(object_id, pose, a_curr) )
                {
                    std::vector<state_t*> new_states;
                    plan_t union_plan;
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
                        boost::tie(index_from, index_to) = get_union_path(union_plan, from_states, &poses_set[a_curr[object_id]], &pose);
                        if( index_to != -1 )
                        {
                            a_curr[object_id] = pose.pose_id;
                            new_states.push_back(pose.ungrasped_set[index_to]);
                        }

                    }

                    if( will_stay || index_to != -1 )
                    {
                        curr_order[curr_depth].reset(object_id, pose_from, target_arrangement[object_id], index_from, index_to);
                        if( curr_depth > max_depth )
                        {
                            PRX_DEBUG_COLOR("---------------------------------------------------------------", PRX_TEXT_BROWN);
                            PRX_DEBUG_COLOR("- UPDATE BEST ORDER " << curr_depth << "/" << max_depth << "   object:" << object_id << ") " << pose_from << "->" << target_arrangement[object_id] << " -", PRX_TEXT_BLUE);
                            PRX_DEBUG_COLOR("---------------------------------------------------------------", PRX_TEXT_BROWN);
                            best_order = curr_order;
                            max_depth = curr_depth;
                            //                                PRX_ASSERT(false);  
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
                                    sequence.push_front(path_part_t(object_id, pose_from, target_arrangement[object_id], index_from, index_to));
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
                                    sequence.push_front(path_part_t(object_id, pose_from, target_arrangement[object_id], index_from, index_to));
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

            bool fmrs_t::clear_to_move(unsigned object_id, const pose_t& pose, const std::vector<unsigned>& arrangement)
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

            std::pair<int, int> fmrs_t::get_union_path(sim::plan_t& plan, const std::vector<sim::state_t*>& from_states, pose_t* start_pose, pose_t* end_pose)
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