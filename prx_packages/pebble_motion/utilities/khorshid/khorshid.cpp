/**
 * @file khorshid.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "utilities/khorshid/khorshid.hpp"

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::packages::pebble_motion::khorshid_t, prx::packages::pebble_motion::pebble_solver_t)


namespace prx
{
    using namespace util;

    namespace packages
    {
        namespace pebble_motion
        {

            khorshid_t::khorshid_t() { }

            khorshid_t::~khorshid_t() { }

            bool khorshid_t::find_solution(std::vector<pebble_step_t>* solution, undirected_graph_t* graph)
            {
                PRX_DEBUG_COLOR("Going to find solution with khorshid solver...", PRX_TEXT_CYAN);
                initial_graph = graph;
                pmt_heuristic->set_graph(graph);
                PRX_DEBUG_COLOR("Set the graph and heuristic", PRX_TEXT_CYAN);
                int pebble_id;
                tmp_path.resize(number_of_vertices);
                tmp_empty_path.resize(number_of_vertices);
                tmp_heap.resize(number_of_vertices);
                push_order.resize(k);
                order.resize(k);


                v_u_path.resize(number_of_vertices);
                v_w_path.resize(number_of_vertices);
                u_w_path.resize(number_of_vertices);

                PRX_DEBUG_COLOR("Resize everything", PRX_TEXT_CYAN);
                update_info(graph, t_assignment);
                compute_the_order(graph, t_assignment);

                s_new = s_assignment;
                PRX_WARN_S("------------    strt    ------------");
                print_assignment(graph, s_assignment);
                PRX_WARN_S("------------    s_new   ------------");
                print_assignment(graph, s_new);
                PRX_WARN_S("-----------     targ    ------------");
                print_assignment(graph, t_assignment);
                PRX_WARN_S("------------------------------------");
                int iter;
                try
                {
                    while( s_new != t_assignment )
                    {
                        for( iter = 0; iter < k; ++iter )
                        {
                            pebble_id = order[iter];
                            PRX_ERROR_S("-------------------  PEBBLE " << pebble_id << "  --------------------");
                            if( !move_to_goal(solution_path, graph, s_new, pebble_id, s_new.get_position(pebble_id), t_assignment.get_position(pebble_id)) )
                            {
                                return false;
                            }
                        }
                    }
                }
                catch( no_solution_t e )
                {
                    return false;
                }

                PRX_DEBUG_COLOR("------------    strt    ------------", PRX_TEXT_BROWN);
                print_assignment(graph, s_assignment);
                PRX_DEBUG_COLOR("------------    s_new    ------------", PRX_TEXT_BROWN);
                print_assignment(graph, s_new);
                PRX_DEBUG_COLOR("-----------     targ    ------------", PRX_TEXT_BROWN);
                print_assignment(graph, t_assignment);
                PRX_DEBUG_COLOR("------------------------------------", PRX_TEXT_BROWN);

                solution_path.compress(solution, s_assignment);
                return true;
            }

            bool khorshid_t::move_to_goal(pebble_solution_path_t& solution, undirected_graph_t* graph, pebble_assignment_t& assign, int pebble_id, undirected_vertex_index_t v_start, undirected_vertex_index_t v_goal)
            {
                if( v_start == v_goal )
                    return true;

                bool path_exists;
                int blocker_id;

                ++avoid_global_id;
                ++obstacle_global_id;
                agent_paths_index = -1;

                PRX_DEBUG_COLOR("------------    assign in move to goal    ------------", PRX_TEXT_MAGENTA);
                print_assignment(graph, assign);

                boost::tie(path_exists, tmp_path_length) = find_path(tmp_path, graph, assign, v_start, v_goal, 0, false, false);

                if( !path_exists )
                    PRX_LOG_ERROR("Path between start and goal of the pebble %d does not exist.", pebble_id);

                PRX_DEBUG_S("Going to move pebble " << pebble_id << "  from :" << print_point(graph, v_start) << " -> " << print_point(graph, v_goal) << "  p_len : " << tmp_path_length);

                int tmp_begin = 0;
                PRX_ASSERT(v_start == tmp_path[tmp_begin]);
                graph->get_vertex_as<pmt_graph_node_t > (v_start)->obstacle_id = obstacle_global_id;
                for( int p = 1; p < tmp_path_length; ++p )
                {
                    blocker_id = assign.get_robot(tmp_path[p]);
                    if( blocker_id != -1 )
                    {
                        //swap will clean all the previous obstacles.
                        PRX_DEBUG_S("------------- Before SWAP -------------");
                        print_assignment(graph, assign);
                        if( !swap(solution, graph, assign, pebble_id, blocker_id, tmp_path, tmp_begin, p) )
                        {
                            PRX_ERROR_S("SWAP between : " << pebble_id << " - " << blocker_id << "   FAILED");
                            return false;
                        }
                        PRX_DEBUG_S("------------- AFTER SWAP -------------");
                        print_assignment(graph, assign);
                        tmp_begin = p;
                        graph->get_vertex_as<pmt_graph_node_t > (tmp_path[tmp_begin])->obstacle_id = obstacle_global_id;
                    }
                }
                if( tmp_begin + 1 <= tmp_path_length - 1 )
                {
                    solution.push_back(pebble_id, tmp_path, tmp_begin, tmp_path_length - 1);
                    assign.change_position(pebble_id, tmp_path[tmp_path_length - 1]);
                }

                return true;
            }

            bool khorshid_t::swap(pebble_solution_path_t& solution, undirected_graph_t* graph, pebble_assignment_t& assign, int pebble1, int pebble2, std::deque<undirected_vertex_index_t>& path, int path_start, int path_end)
            {

                undirected_vertex_index_t v = assign.get_position(pebble1);
                undirected_vertex_index_t u = assign.get_position(pebble2);
                PRX_DEBUG_COLOR("SWAP : " << pebble1 << " - " << pebble2 << "   v:" << print_point(graph, v) << " - " << print_point(graph, u) << "  pstart:" << path_start << "   pend:" << path_end, PRX_TEXT_CYAN);
                int path_length = path_end - path_start + 1;
                PRX_ASSERT(v == path[path_start]);
                PRX_ASSERT(u == path[path_end]);
                pebble_assignment_t keep_assign = assign;
                int revert_index = solution.size();
                int reverse_end;
                ++visited_global_id;
                ++obstacle_global_id;
                ++avoid_global_id;

                //If the vertex u is already on a branch.
                w_node = graph->get_vertex_as<pmt_graph_node_t > (u);
                w_node->visited_id = visited_global_id;
                if( w_node->is_branch )
                {
                    PRX_DEBUG_COLOR("for Criterion 2 w on u : " << print_point(graph, u), PRX_TEXT_MAGENTA);
                    w = u;
                    r = NULL;
                    q = NULL;
                    neigh = path[path_end - 1];
                    if( path_length > 2 )
                    {
                        PRX_DEBUG_S("Will bring pebble " << pebble1 << "  from pos:" << print_point(graph, v) << " -> " << print_point(graph, neigh));
                        solution.push_back(pebble1, path, path_start, path_end - 1);
                        assign.change_position(pebble1, neigh);
                    }

                    if( check_branch(solution, graph, assign) )
                    {
                        PRX_DEBUG_COLOR("CRITERION 2 : w " << print_point(graph, u) << " on u worked ", PRX_TEXT_BROWN);
                        reverse_end = solution.size() - 1;
                        if( std::find(tmp_path.begin(), tmp_path.begin() + tmp_path_length, r) == tmp_path.begin() + tmp_path_length )
                        {
                            step(solution, assign, pebble2, r);
                            two_steps(solution, assign, pebble1, w, q);
                        }
                        else
                        {
                            step(solution, assign, pebble2, q);
                            two_steps(solution, assign, pebble1, w, r);
                        }
                        two_steps(solution, assign, pebble2, w, neigh);
                        step(solution, assign, pebble1, w);
                        solution.reverse_path(assign, revert_index, reverse_end, pebble1, pebble2);
                        return true;
                    }

                    solution.revert(revert_index);
                    assign = keep_assign;
                }

                ++obstacle_global_id;

                print_assignment(graph, assign);
                //For branches between the two nodes.
                for( int i = path_end - 1; i > path_start; --i )
                {
                    w_node = graph->get_vertex_as<pmt_graph_node_t > (path[i]);
                    w_node->visited_id = visited_global_id;
                    if( w_node->is_branch )
                    {
                        w = path[i];
                        PRX_DEBUG_COLOR("for Criterion 3 : w between " << print_point(graph, w), PRX_TEXT_MAGENTA);
                        q = NULL;

                        PRX_DEBUG_S("Will move pebble " << pebble1 << "  from pos:" << print_point(graph, assign.get_position(pebble1)) << " -> " << print_point(graph, path[i]));
                        //pebble1 will go on the branch vertex.

                        solution.push_back(pebble1, path, path_start, i);
                        assign.change_position(pebble1, path[i]);


                        PRX_DEBUG_S("Will move pebble " << pebble2 << "  from pos:" << print_point(graph, assign.get_position(pebble2)) << " -> " << print_point(graph, path[i + 1]));
                        //pebble2 will go on the vertex next to w from its side.
                        if( path_end > i + 1 )
                        {
                            solution.push_back(pebble2, path, path_end, i + 1);
                            assign.change_position(pebble2, path[i + 1]);
                        }

                        neigh = path[i + 1];
                        r = path[i - 1];

                        if( check_branch(solution, graph, assign) )
                        {
                            PRX_WARN_S("CRITERION 3 : w " << print_point(graph, w) << " between worked ");
                            reverse_end = solution.size() - 1;
                            step(solution, assign, pebble1, q);
                            two_steps(solution, assign, pebble2, w, r);
                            two_steps(solution, assign, pebble1, w, neigh);
                            step(solution, assign, pebble2, w);
                            solution.reverse_path(assign, revert_index, reverse_end, pebble1, pebble2);
                            return true;
                        }
                        solution.revert(revert_index);
                        assign = keep_assign;
                    }
                }

                ++obstacle_global_id;

                print_assignment(graph, assign);
                //If v is branch
                w_node = graph->get_vertex_as<pmt_graph_node_t > (v);
                w_node->visited_id = visited_global_id;
                if( w_node->is_branch )
                {
                    PRX_DEBUG_COLOR("for Criterion 1  w on v : " << print_point(graph, v), PRX_TEXT_MAGENTA);
                    w = v;
                    r = NULL;
                    q = NULL;
                    if( path_length > 2 )
                    {
                        PRX_DEBUG_S("Will move pebble " << pebble2 << "  from pos:" << print_point(graph, assign.get_position(pebble2)) << " -> " << print_point(graph, path[1]));
                        solution.push_back(pebble2, path, path_end, path_start + 1);
                        assign.change_position(pebble2, path[path_start + 1]);
                    }
                    neigh = path[path_start + 1];
                    if( check_branch(solution, graph, assign) )
                    {
                        PRX_WARN_S("CRITERION 1 : w " << print_point(graph, v) << " on v worked ");
                        PRX_DEBUG_COLOR("w: " << print_point(graph, w) << "  neigh:" << print_point(graph, neigh) << "  r:" << print_point(graph, r) << "  q:" << print_point(graph, q), PRX_TEXT_BLUE);
                        print_assignment(graph, assign);
                        reverse_end = solution.size() - 1;
                        PRX_DEBUG_COLOR("reverse start : " << revert_index << "   reverse_end : " << reverse_end, PRX_TEXT_BLUE);
                        step(solution, assign, pebble1, q);
                        two_steps(solution, assign, pebble2, w, r);
                        two_steps(solution, assign, pebble1, w, neigh);
                        step(solution, assign, pebble2, w);
                        print_assignment(graph, assign);
                        PRX_DEBUG_S("to reverse");
                        solution.reverse_path(assign, revert_index, reverse_end, pebble1, pebble2);
                        print_assignment(graph, assign);
                        PRX_ERROR_S("after reverse");
                        return true;
                    }

                    solution.revert(revert_index);
                    assign = keep_assign;
                }

                neigh = path[path_end - 1];
                if( path_length > 2 )
                {
                    PRX_DEBUG_S("Will move pebble " << pebble1 << "  from pos:" << print_point(graph, assign.get_position(pebble1)) << " -> " << print_point(graph, neigh));
                    solution.push_back(pebble1, path, path_start, path_end - 1);
                    assign.change_position(pebble1, neigh);
                }

                neigh_node = graph->get_vertex_as<pmt_graph_node_t > (neigh);
                r = NULL;
                q = NULL;
                if( check_visible_branches(solution, graph, assign, u, false) )
                    return true;
                solution.revert(revert_index);
                assign = keep_assign;


                ++obstacle_global_id;
                neigh = path[path_start + 1];
                if( path_length > 2 )
                {
                    PRX_DEBUG_S("Will move pebble " << pebble2 << "  from pos:" << print_point(graph, assign.get_position(pebble2)) << " -> " << print_point(graph, neigh));
                    solution.push_back(pebble2, path, path_end, path_start + 1);
                    assign.change_position(pebble2, neigh);
                }

                neigh_node = graph->get_vertex_as<pmt_graph_node_t > (neigh);
                r = NULL;
                q = NULL;
                if( check_visible_branches(solution, graph, assign, v, true) )
                    return true;
                solution.revert(revert_index);
                assign = keep_assign;

                //PRX_ERROR_S("SWAP between : " << pebble1 << " - " << pebble2 << " FAILED!!!");
                //PRX_INFO_S(solution.print(graph, state_space));
                //    print_assignment(graph, assign);
                PRX_ERROR_S("There is no solution");
                throw no_solution_t();
                return false;
            }

            bool khorshid_t::check_visible_branches(pebble_solution_path_t& solution, undirected_graph_t* graph, pebble_assignment_t& assign, undirected_vertex_index_t v_start, bool caller_on_w)
            {
                ++bfs_global_id;
                ++avoid_global_id;

                graph->predecessors[v_start] = v_start;
                node = graph->get_vertex_as<pmt_graph_node_t > (v_start);
                static_heap.restart();
                static_heap.push_back(v_start);
                bool is_branch_ok = true;

                node->bfs_id = bfs_global_id;
                int revert_index = solution.size();
                int reverse_end;
                pebble_assignment_t keep_assign = assign;
                undirected_vertex_index_t v_pebble2_start = neigh;
                int pebble1 = assign.get_robot(v_start);
                int pebble2 = assign.get_robot(neigh);

                unsigned int curr_obstacle_id = obstacle_global_id;
                while( !static_heap.empty() )
                {
                    undirected_vertex_index_t v = static_heap.pop_front();

                    foreach(undirected_vertex_index_t adj, boost::adjacent_vertices(v, graph->graph))
                    {
                        w = adj;

                        w_node = graph->get_vertex_as<pmt_graph_node_t > (adj);
                        if( w_node->bfs_id != bfs_global_id )
                        {
                            if( w_node->visited_id != visited_global_id && w_node->obstacle_id != curr_obstacle_id )
                            {
                                graph->predecessors[adj] = v;
                                if( w_node->is_branch )
                                {
                                    //                        if( caller_on_w )
                                    //                            //PRX_DEBUG_S("for CRITERION 4 : w " << print_point(graph, w));
                                    //                        else
                                    //                            //PRX_DEBUG_S("for CRITERION 5 : w " << print_point(graph, w));
                                    //                        //PRX_DEBUG_S(static_heap.size() << " )  w : " << print_point(graph, w) << "   from v_start : " << print_point(graph, v_start) << "   is from v side: " << caller_on_w);
                                    v_w_path_length = 0;
                                    for( undirected_vertex_index_t p = adj; p != v_start; p = graph->predecessors[p] )
                                    {
                                        v_w_path[v_w_path_length] = p;
                                        ++v_w_path_length;
                                    }
                                    v_w_path[v_w_path_length] = v_start;
                                    ++v_w_path_length;

                                    PRX_ASSERT(w == v_w_path[0]);

                                    ++obstacle_global_id;

                                    graph->get_vertex_as<pmt_graph_node_t > (v_pebble2_start)->obstacle_id = obstacle_global_id;

                                    undirected_vertex_index_t v_prev = v_pebble2_start;
                                    for( int i = v_w_path_length - 2; i >= 0; --i )
                                    {
                                        graph->get_vertex_as<pmt_graph_node_t > (v_w_path[i + 1])->obstacle_id = obstacle_global_id;

                                        if( assign.has_robot_on(v_w_path[i]) )
                                        {
                                            //                                int pidd = assign.get_robot(v_w_path[i]);
                                            //PRX_DEBUG_S("going to push to empty pebble " << pidd << " from: " << print_point(graph, v_w_path[i]));
                                            //  
                                            if( !clear_vertex(solution, graph, assign, assign.get_robot(v_w_path[i])) )
                                                //if( !clear_to_target(solution, graph, assign, assign.get_robot(v_w_path[i])) )
                                            {
                                                is_branch_ok = false;
                                                break;
                                            }
                                            //PRX_DEBUG_S("Pebble " << pidd << " is now at: " << print_point(graph, assign.get_position(pidd)));
                                        }

                                        step(solution, assign, pebble1, v_w_path[i]);
                                        step(solution, assign, pebble2, v_w_path[i + 1]);
                                        graph->get_vertex_as<pmt_graph_node_t > (v_prev)->obstacle_id = 0;
                                        v_prev = v_w_path[i + 1];
                                    }

                                    PRX_DEBUG_COLOR("Done with the moving now checks if the branch is ok", PRX_TEXT_MAGENTA);
                                    if( is_branch_ok )
                                    {
                                        neigh = v_w_path[1];
                                        PRX_ASSERT(w == v_w_path[0]);
                                        //PRX_DEBUG_S("pebble " << pebble1 << "  is now " << print_point(graph, assign.get_position(pebble1)) << "   and pebble2 " << pebble2 << "  at point: " << print_point(graph, assign.get_position(pebble2)));
                                        if( check_branch(solution, graph, assign) )
                                        {
                                            reverse_end = solution.size() - 1;
                                            if( caller_on_w )
                                            {
                                                //PRX_WARN_S("CRITERION 4 : w " << print_point(graph, w) << "  worked");
                                                step(solution, assign, pebble1, r);
                                                two_steps(solution, assign, pebble2, w, q);
                                                two_steps(solution, assign, pebble1, w, neigh);
                                                step(solution, assign, pebble2, w);
                                            }
                                            else
                                            {
                                                
                                                //PRX_WARN_S("CRITERION 5 : w " << print_point(graph, w) << "  worked");
                                                if( std::find(tmp_path.begin(), tmp_path.begin() + tmp_path_length, r) == tmp_path.begin() + tmp_path_length )
                                                {
                                                    step(solution, assign, pebble1, r);
                                                    two_steps(solution, assign, pebble2, w, q);
                                                }
                                                else
                                                {
                                                    step(solution, assign, pebble1, q);
                                                    two_steps(solution, assign, pebble2, w, r);
                                                }
                                                two_steps(solution, assign, pebble1, w, neigh);
                                                step(solution, assign, pebble2, w);
                                            }
                                            solution.reverse_path(assign, revert_index, reverse_end, pebble1, pebble2);
                                            return true;
                                        }
                                        PRX_DEBUG_COLOR(" OH nooo branch " << print_point(graph, w) << "  Failed!", PRX_TEXT_MAGENTA);
                                    }

                                    w_node->visited_id = visited_global_id;
                                    solution.revert(revert_index);
                                    assign = keep_assign;
                                }
                                else
                                    static_heap.push_back(adj);
                            }

                            w_node->bfs_id = bfs_global_id;
                        }
                    }
                }
                w = NULL;
                w_node = NULL;
                return false;

            }
        }
    }
}
