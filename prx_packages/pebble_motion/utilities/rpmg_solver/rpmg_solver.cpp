/**
 * @file rpmg_solver.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2012, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Athanasios Krontiris, Andrew Dobson, Andrew Kimmel, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "utilities/rpmg_solver/rpmg_solver.hpp"
#include "utilities/pebble_spanning_tree.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

#include <boost/tuple/tuple.hpp> // boost::tie

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::packages::pebble_motion::rpmg_solver_t, prx::packages::pebble_motion::pebble_solver_t)

namespace prx
{
    using namespace util;

    namespace packages
    {
        namespace pebble_motion
        {

            rpmg_solver_t::rpmg_solver_t() { }

            rpmg_solver_t::~rpmg_solver_t()
            {
                delete pmt_visitor;
                delete pmt_heuristic;
            }

            bool rpmg_solver_t::swap(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign, int pebble1, int pebble2, std::deque<util::undirected_vertex_index_t>& path, int path_start, int path_end)
            {

                swap_pebble1 = pebble1;
                swap_pebble2 = pebble2;
                //                resolve_pos = -1;
                //                if(id_order[swap_pebble1] < id_order[swap_pebble2])
                //                    resolve_pos = solution.size();

//                if(id_order[swap_pebble1] < id_order[swap_pebble2])
//                    need_resolve = true;
//                else                
                    need_resolve = false;
                
                resolve_pos = solution.size();
                
                bool swapped = pmt_solver_t::swap(solution, graph, assign, pebble1, pebble2, path, path_start, path_end);

                //If resolve is needed
                //                if(resolve_pos != -1)
                if( need_resolve )
                {

                    solution.reverse_path(assign, resolve_pos, solution.size() - 7, pebble1, pebble2);

                    //TODO :  remove these lines.
                    std::vector<pebble_step_t>* sp = new std::vector<pebble_step_t > ();
                    solution.compress(sp, s_assignment);
                    if( !is_valid_solution(graph, sp, s_assignment) )
                        PRX_LOG_ERROR("DONE");
                    //TODO :  remove these lines.
                }

                return swapped;
            }

            bool rpmg_solver_t::clear_vertex(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign, int pebble_id)
            {
                if( pebble_id == -1 )
                    return true;

                bool path_exists = false;
                int blocker_id = -1;
                int i_step = 1;
                undirected_vertex_index_t v_step;

                undirected_vertex_index_t v_start = assign.get_position(pebble_id);
                undirected_vertex_index_t v_goal = t_assignment.get_position(pebble_id);

                //                boost::tie(path_exists, path_lengths[pebble_id]) = path_to_target(agent_paths[pebble_id], graph, assign, v_start, v_goal, 0);
                boost::tie(path_exists, path_lengths[pebble_id]) = find_path(agent_paths[pebble_id], graph, assign, v_start, v_goal, 0);

                if( path_exists && path_lengths[pebble_id] > 1 )
                {
                    for( i_step = 1; i_step < path_lengths[pebble_id]; ++i_step )
                    {

                        v_step = agent_paths[pebble_id][i_step];
                        blocker_id = assign.get_robot(v_step);
                        if( blocker_id != -1 )
                        {
                            //                            --i_step;
                            PRX_DEBUG_COLOR("Has robot (" << blocker_id << ")... to target " << print_point(graph, v_step), PRX_TEXT_RED);
                            //                            v_step = agent_paths[pebble_id][i_step];
                            break;
                        }

                        if( graph->get_vertex_as<pmt_graph_node_t > (v_step)->avoid_id != avoid_global_id )
                            break;
                    }
                    PRX_ASSERT(i_step != path_lengths[pebble_id]);

                    graph->get_vertex_as<pmt_graph_node_t > (v_start)->obstacle_id = obstacle_global_id;

                    if( blocker_id == -1 || clear_vertex(solution, graph, assign, blocker_id) )
                    {
                        graph->get_vertex_as<pmt_graph_node_t > (v_start)->obstacle_id = 0;
                        PRX_DEBUG_COLOR("Push to target : " << pebble_id << "  moved to : " << print_point(graph, v_step), PRX_TEXT_BLUE);
                        solution.push_back(pebble_id, agent_paths[pebble_id], 0, i_step);
                        PRX_DEBUG_COLOR("i_Step: " << i_step << "   from_i_step: " << print_point(graph, agent_paths[pebble_id][i_step]), PRX_TEXT_MAGENTA);
                        assign.change_position(pebble_id, v_step);
                        return true;
                    }
                }

                boost::tie(path_exists, path_lengths[pebble_id]) = path_to_empty(agent_paths[pebble_id], graph, assign, v_start);

                if( path_exists && path_lengths[pebble_id] > 1 )
                {
                    for( i_step = 1; i_step < path_lengths[pebble_id]; ++i_step )
                    {
                        v_step = agent_paths[pebble_id][i_step];
                        blocker_id = assign.get_robot(v_step);
                        if( assign.has_robot_on(v_step) )
                        {
                            //                            --i_step; 
                            PRX_DEBUG_COLOR("Has robot... to empty", PRX_TEXT_MAGENTA);
                            //                            v_step = agent_paths[pebble_id][i_step];
                            break;
                        }

                        if( graph->get_vertex_as<pmt_graph_node_t > (v_step)->avoid_id != avoid_global_id )
                            break;

                    }
                    PRX_ASSERT(i_step != path_lengths[pebble_id]);
                    //                    blocker_id = assign.get_robot(v_step);
                    graph->get_vertex_as<pmt_graph_node_t > (v_start)->obstacle_id = obstacle_global_id;

                    if( blocker_id == -1 || clear_vertex(solution, graph, assign, blocker_id) )
                    {
                        PRX_DEBUG_COLOR("the order id : " << id_order[swap_pebble1] << "    pushed id : " << id_order[pebble_id], PRX_TEXT_LIGHTGRAY);
                        //                        if(resolve_pos == -1 && id_order[swap_pebble1] < id_order[pebble_id])
                        //                            resolve_pos = solution.size();
                        if( !need_resolve && id_order[swap_pebble1] < id_order[pebble_id] )
                            need_resolve = true;

                        graph->get_vertex_as<pmt_graph_node_t > (v_start)->obstacle_id = 0;
                        PRX_DEBUG_COLOR("Push to empty : " << pebble_id << "  moved to : " << print_point(graph, v_step), PRX_TEXT_CYAN);
                        solution.push_back(pebble_id, agent_paths[pebble_id], 0, i_step);
                        PRX_DEBUG_COLOR("i_Step: " << i_step << "   from_i_step: " << print_point(graph, agent_paths[pebble_id][i_step]), PRX_TEXT_MAGENTA);
                        assign.change_position(pebble_id, v_step);
                        return true;
                    }
                }
                return false;
            }

            void rpmg_solver_t::compute_the_order(undirected_graph_t* graph, pebble_assignment_t& assignment)
            {
                static_heap.restart();
                pebble_spanning_tree_t spanning_tree;
                spanning_tree.compute_spanning_tree(graph);
                spanning_tree.get_leaves(static_heap);
                undirected_vertex_index_t v_tree, v_par_tree;
                undirected_vertex_index_t v;
                int index = 0;

                while( !static_heap.empty() )
                {
                    //                    v_tree = static_heap.pop_front();
                    v_tree = static_heap.pop_random();
                    v_par_tree = spanning_tree.get_parent(v_tree);
                    v = spanning_tree.get_graph_index(v_tree);

                    if( assignment.has_robot_on(v) )
                    {
                        int pebble_id = assignment.get_robot(v);
                        order[index] = pebble_id;
                        id_order[pebble_id] = k - index;
                        ++index;
                    }

                    spanning_tree.remove_vertex(v_tree);

                    if( v_par_tree != v_tree && !spanning_tree.has_children(v_par_tree) )
                        if( !static_heap.has(v_par_tree) )
                            static_heap.push_back(v_par_tree);
                }
            }
        }
    }
}

