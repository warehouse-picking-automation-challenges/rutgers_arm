/**
 * @file pmt_solver.cpp
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

#include "utilities/pmt_solver/pmt_solver.hpp"
#include "utilities/pebble_spanning_tree.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"

#include <boost/tuple/tuple.hpp> // boost::tie

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::packages::pebble_motion::pmt_solver_t, prx::packages::pebble_motion::pebble_solver_t)

namespace prx
{
    using namespace util;

    namespace packages
    {
        namespace pebble_motion
        {

            pmt_solver_t::pmt_solver_t() { }

            pmt_solver_t::~pmt_solver_t()
            {
                delete pmt_visitor;
                delete pmt_heuristic;
            }

            void pmt_solver_t::setup(undirected_graph_t* graph, pebble_assignment_t& start_assign, pebble_assignment_t& target_assign, int num_robots, const space_t* space, distance_metric_t* distance_metric)
            {
                tmp_path_length = 0;
                on_path_branch_global_id = 0;
                v_u_path_length = 0;
                v_w_path_length = 0;
                u_w_path_length = 0;
                visited_global_id = 1;
                avoid_global_id = 1;
                obstacle_global_id = 1;
                on_path_branch_global_id = 1;
                not_solved = true;

                pebble_solver_t::setup(graph, start_assign, target_assign, num_robots, space, distance_metric);
                pmt_heuristic = new pmt_default_distance_heuristic_t(distance_metric);
                pmt_visitor = new pmt_astar_visitor_t();

                initial_graph = graph;
                pmt_heuristic->set_graph(graph);

                tmp_path.resize(number_of_vertices);
                tmp_path_length = 0;
                tmp_empty_path.resize(number_of_vertices);
                tmp_heap.resize(number_of_vertices);
                push_order.resize(k);
                order.resize(k);
                called.resize(k);
                called_size = 0;
                priority_cycle_detected = false;

                path_lengths.resize(k);
                agent_paths.resize(k);
                for( int i = 0; i < k; ++i )
                    agent_paths[i].resize(number_of_vertices);

                priority_path_lengths.resize(k);
                agent_priority_paths.resize(k);
                for( int i = 0; i < k; ++i )
                    agent_priority_paths[i].resize(number_of_vertices);

                v_u_path.resize(number_of_vertices);
                v_w_path.resize(number_of_vertices);
                u_w_path.resize(number_of_vertices);
            }

            undirected_vertex_index_t pmt_solver_t::add_new_vertex(undirected_graph_t* graph)
            {
                //                undirected_vertex_index_t v = graph->add_vertex<pmt_graph_node_t > ();
                //                graph->get_vertex_as<pmt_graph_node_t>(v)->id = node_id;
                return graph->add_vertex<pmt_graph_node_t > ();
            }

            void pmt_solver_t::deserialize(util::undirected_graph_t* graph, std::string graphfile, const space_t* space)
            {
                PRX_ERROR_S("deserialize of PMT");
                std::ifstream fin;
                graph->deserialize<pmt_graph_node_t, pebble_solver_edge_t > (graphfile, fin, space);
                fin.close();
            }

            bool pmt_solver_t::reduce(std::vector<pebble_step_t>* solution, undirected_graph_t* graph, pebble_assignment_t& s_new, pebble_assignment_t& s_assign, pebble_assignment_t& t_assign)
            {
                return true;
            }

            void pmt_solver_t::update_info(undirected_graph_t* graph, pebble_assignment_t& t_assign)
            {

                foreach(undirected_vertex_index_t v, boost::vertices(graph->graph))
                {
                    graph->get_vertex_as<pmt_graph_node_t > (v)->is_branch = (boost::degree(v, graph->graph) > 2);
                }
            }

            bool pmt_solver_t::find_solution(std::vector<pebble_step_t>* solution, undirected_graph_t* graph)
            {
                int pebble_id;

                update_info(graph, t_assignment);
                compute_the_order(graph, t_assignment);

                s_new = s_assignment;
                //PRX_WARN_S("------------    strt    ------------");
                print_assignment(graph, s_assignment);
                //PRX_WARN_S("------------    s_new   ------------");
                print_assignment(graph, s_new);
                //PRX_WARN_S("-----------     targ    ------------");
                print_assignment(graph, t_assignment);
                //PRX_WARN_S("------------------------------------");
                int iter;
                try
                {
                    while( s_new != t_assignment )
                    {
                        PRX_DEBUG_COLOR("-------------------    START ALL THE PEBBLES  --------------------", PRX_TEXT_RED);
                        print_assignment(graph, s_new, PRX_TEXT_RED);
                        priority_cycle_detected = false;
                        for( iter = 0; iter < k; ++iter )
                        {
                            pebble_id = order[iter];
                            PRX_ERROR_S("-------------------  PEBBLE " << pebble_id << "  --------------------");
                            if( !move_to_goal(solution_path, graph, s_new, pebble_id, s_new.get_position(pebble_id), t_assignment.get_position(pebble_id)) )
                            {
                                called_size = 0;
                                break;
                            }
                            //               PRX_LOG_ERROR("Pebble %d couldn't reach its goal!");
                        }

                        //        //PRX_ERROR_S("-----------------------------------------------------------");
                        //        //PRX_ERROR_S("-----------------------------------------------------------");
                        //        //PRX_ERROR_S("-------------------   DONE FIRST ITER  --------------------");
                        //        //PRX_ERROR_S("-----------------------------------------------------------");
                        //        //PRX_ERROR_S("-----------------------------------------------------------");
                        //        
                        //        for( iter = k-1; iter >=0; --iter )
                        //        {
                        //            pebble_id = order[iter];
                        //            //PRX_ERROR_S("-------------------  PEBBLE " << pebble_id << "  --------------------");
                        //            while( !move_to_goal(solution_path, graph, s_new, pebble_id, s_new.get_position(pebble_id), t_assignment.get_position(pebble_id)) )
                        //            {
                        //            }
                        //            //                PRX_LOG_ERROR("Pebble %d couldn't reach its goal!");
                        //        }
                        //        
                        //        //PRX_WARN_S("-----------------------------------------------------------");
                        //        //PRX_WARN_S("-----------------------------------------------------------");
                        //        //PRX_WARN_S("-------------------   DONE SEDOND ITER  -------------------");
                        //        //PRX_WARN_S("-----------------------------------------------------------");
                        //        //PRX_WARN_S("-----------------------------------------------------------");
                    }
                }
                catch( no_solution_t e )
                {
                    solution_path.compress(solution, s_assignment);
                    return false;
                }

                //    //PRX_WARN_S("------------    strt    ------------");
                //    print_assignment(graph, s_assignment);
                //    //PRX_WARN_S("------------    s_new    ------------");
                //    print_assignment(graph, s_new);
                //    //PRX_WARN_S("-----------     targ    ------------");
                //    print_assignment(graph, t_assignment);
                //    //PRX_WARN_S("------------------------------------");

                //PRX_DEBUG_S("solution path : " << solution_path.print(graph, state_space));

                solution_path.compress(solution, s_assignment);
                return true;
            }

            bool pmt_solver_t::move_to_goal(pebble_solution_path_t& solution, undirected_graph_t* graph, pebble_assignment_t& assign, int pebble_id, undirected_vertex_index_t v_start, undirected_vertex_index_t v_goal)
            {
                if( v_start == v_goal )
                    return true;

                bool path_exists;
                int blocker_id;
                undirected_vertex_index_t v_blocker;
                undirected_vertex_index_t v_blocker_end;

                ++avoid_global_id;
                ++obstacle_global_id;
                agent_paths_index = -1;

                PRX_ASSERT(called_size >= 0 && called_size < k);
                called[called_size] = pebble_id;
                ++called_size;

                PRX_DEBUG_COLOR("------------    assign in move to goal    ------------", PRX_TEXT_CYAN);
                print_assignment(graph, assign, PRX_TEXT_CYAN);

                //Checks first for a path that will not move the pebble that are on their goals.
                boost::tie(path_exists, priority_path_lengths[pebble_id]) = find_a_path(agent_priority_paths[pebble_id], graph, assign, v_start, v_goal, 0, false, false);

                int path_length = priority_path_lengths[pebble_id];

                if( !path_exists )
                    PRX_LOG_ERROR("Path between start and goal of the pebble %d does not exist.", pebble_id);

                PRX_DEBUG_COLOR("Going to move pebble " << pebble_id << "  from :" << print_point(graph, v_start) << " -> " << print_point(graph, v_goal) << "  p_len : " << path_length, PRX_TEXT_GREEN);

                int tmp_begin = 0;
                //                graph->get_vertex_as<pmt_graph_node_t > (agent_priority_paths[pebble_id][tmp_begin])->obstacle_id = obstacle_global_id;
                int pos;
                boost::tie(blocker_id, pos) = find_blocker_on_path(agent_priority_paths[pebble_id], path_length, assign);

                if( blocker_id == -1 )
                {
                    solution.push_back(pebble_id, agent_priority_paths[pebble_id], 0, path_length - 1);
                    assign.change_position(pebble_id, agent_priority_paths[pebble_id][path_length - 1]);
                }
                else
                {
                    //                    if( need_swap(graph, agent_priority_paths[pebble_id], tmp_begin, path_length, agent_priority_paths[pebble_id][pos], t_assignment.get_position(blocker_id)) || std::find(called.begin(), called.begin() + called_size, blocker_id) != called.begin() + called_size )
                    if( need_swap(graph, agent_priority_paths[pebble_id], tmp_begin, path_length, agent_priority_paths[pebble_id][pos], t_assignment.get_position(blocker_id)) )
                    {
                        PRX_DEBUG_COLOR("Pebble " << pebble_id << " and Pebble " << blocker_id << " need swap", PRX_TEXT_MAGENTA);
                        for( int i = tmp_begin; i <= pos; ++i )
                            PRX_DEBUG_COLOR("path[" << i << "]: " << print_point(graph, agent_priority_paths[pebble_id][i]), PRX_TEXT_BROWN);
                        if( swap(solution, graph, assign, pebble_id, blocker_id, agent_priority_paths[pebble_id], tmp_begin, pos) )
                            return false;

                        std::string msg = "SWAP between " + int_to_str(pebble_id) + " and " + int_to_str(blocker_id) + " FAILED!";
                        throw no_solution_t(msg);
                    }
                    else
                    {
                        v_blocker = agent_priority_paths[pebble_id][pos];
                        v_blocker_end = t_assignment.get_position(blocker_id);

                        priority_cycle_detected = std::find(called.begin(), called.begin() + called_size, blocker_id) != called.begin() + called_size;
                        if( !priority_cycle_detected && move_to_goal(solution, graph, assign, blocker_id, v_blocker, v_blocker_end) )
                        {
                            --called_size;
                            return move_to_goal(solution, graph, assign, pebble_id, v_start, v_goal);
                        }
                        else
                        {
                            if( priority_cycle_detected )
                                PRX_DEBUG_COLOR("Cycle is detected with pebble : " << pebble_id << " - blocker: " << blocker_id, PRX_TEXT_BROWN);

                            if( !assign.has_robot_on(agent_priority_paths[pebble_id][1]) )
                                step(solution, assign, pebble_id, agent_priority_paths[pebble_id][1]);

                            return false;
                        }
                    }
                    return false;

                }
                --called_size;
                priority_cycle_detected = false;
                return true;
            }

            bool pmt_solver_t::swap(pebble_solution_path_t& solution, undirected_graph_t* graph, pebble_assignment_t& assign, int pebble1, int pebble2, std::deque<undirected_vertex_index_t>& path, int path_start, int path_end)
            {

                undirected_vertex_index_t v = assign.get_position(pebble1);
                undirected_vertex_index_t u = assign.get_position(pebble2);
                PRX_DEBUG_COLOR("SWAP : " << pebble1 << " - " << pebble2 << "   v:" << print_point(graph, v) << " - " << print_point(graph, u) << "  pstart:" << path_start << "   pend:" << path_end, PRX_TEXT_CYAN);
                for( int i = path_start; i <= path_end; ++i )
                    PRX_DEBUG_COLOR("path[" << i << "]: " << print_point(graph, path[i]), PRX_TEXT_BLUE);
                int path_length = path_end - path_start + 1;
                PRX_ASSERT(v == path[path_start]);
                PRX_ASSERT(u == path[path_end]);
                pebble_assignment_t keep_assign = assign;
                int revert_index = solution.size();
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
                        //                        if( std::find(tmp_path.begin(), tmp_path.begin() + tmp_path_length, r) == tmp_path.begin() + tmp_path_length )
                        if( std::find(path.begin() + path_start, path.begin() + path_end + 1, r) == path.begin() + path_end + 1 )
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
                        return true;
                    }

                    solution.revert(revert_index);
                    assign = keep_assign;
                    for( int i = path_start; i <= path_end; ++i )
                        PRX_DEBUG_COLOR("path[" << i << "]: " << print_point(graph, path[i]), PRX_TEXT_MAGENTA);
                }

                ++obstacle_global_id;

                //For branches between the two nodes.
                for( int i = path_start; i <= path_end; ++i )
                    PRX_DEBUG_COLOR("path[" << i << "]: " << print_point(graph, path[i]), PRX_TEXT_GREEN);
                for( int i = path_end - 1; i > path_start; --i )
                {
                    PRX_DEBUG_COLOR("i: " << i << "/" << path_end << "  is the node: " << print_point(graph, path[i]), PRX_TEXT_GREEN);
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
                            step(solution, assign, pebble1, q);
                            two_steps(solution, assign, pebble2, w, r);
                            two_steps(solution, assign, pebble1, w, neigh);
                            step(solution, assign, pebble2, w);
                            return true;
                        }
                        solution.revert(revert_index);
                        assign = keep_assign;
                    }
                }

                ++obstacle_global_id;

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
                        step(solution, assign, pebble1, q);
                        two_steps(solution, assign, pebble2, w, r);
                        two_steps(solution, assign, pebble1, w, neigh);
                        step(solution, assign, pebble2, w);
                        return true;
                    }

                    solution.revert(revert_index);
                    assign = keep_assign;
                }

                //For criteria 4 and 5 away branch vertices.
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

            bool pmt_solver_t::need_swap(undirected_graph_t* graph, std::deque<undirected_vertex_index_t>& path, int path_start, int path_length, undirected_vertex_index_t initial_p2, undirected_vertex_index_t final_p2)
            {
                if( std::find(path.begin() + path_start, path.begin() + path_length, final_p2) != (path.begin() + path_length) )
                {
                    //PRX_INFO_S("need swap because P2 " << print_point(graph, final_p2) << "  belongs on the path of P1: " << print_point(graph, path[path_start]) << " - " << print_point(graph, path[path_length - 1]));
                    return true;
                }

                if( is_on_path(graph, initial_p2, final_p2, path[path_start]) )
                {
                    //PRX_INFO_S("need swap because P1 " << print_point(graph, path[path_start]) << "  belongs on the path of P2 : " << print_point(graph, tmp_vector[0]) << " - " << print_point(graph, tmp_vector[tmp_vector_index - 1]));
                    return true;
                }

                return false;

            }

            bool pmt_solver_t::check_branch(pebble_solution_path_t& solution, undirected_graph_t* graph, pebble_assignment_t& assign)
            {
                PRX_DEBUG_COLOR("Check branch : " << print_point(graph, w), PRX_TEXT_GREEN);
                int pebble1 = assign.get_robot(w);
                int pebble2 = assign.get_robot(neigh);

                neigh_node = graph->get_vertex_as<pmt_graph_node_t > (neigh);
                int empty_vertices = 0;
                if( r != NULL )
                {
                    empty_vertices = 1;
                    r_node = graph->get_vertex_as<pmt_graph_node_t > (r);
                }

                if( clear_branch(solution, graph, assign, empty_vertices) )
                {
                    PRX_DEBUG_COLOR("FREE BRANCH : " << print_point(graph, w) << "  neigh : " << print_point(graph, neigh) << "  r : " << print_point(graph, r) << " q : " << print_point(graph, q), PRX_TEXT_CYAN);
                    //PRX_DEBUG_S("w : " << print_point(graph, w));
                    //PRX_DEBUG_S("neigh : " << print_point(graph, neigh));
                    //PRX_DEBUG_S("r : " << print_point(graph, r));
                    //PRX_DEBUG_S("q : " << print_point(graph, q));
                    //PRX_DEBUG_S("------------- DONE checking branch " << print_point(graph, w) << " ----------------");
                    return true;
                }
                else
                {
                    //                    PRX_ASSERT(empty_vertices == 1);
                    if( empty_vertices != 0 )
                    {
                        //from the clear_branch the w and neigh will be obstacles.
                        //and r will be in in the avoid list.
                        //Because r is only avoidable pebble2 could go through it. We have to avoid that
                        //when we are pushing pebble2 away from its position. 
                        r_node->obstacle_id = obstacle_global_id;
                        //            if( /push_to_empty(solution, graph, assign, neigh) )
                        if( clear_vertex(solution, graph, assign, assign.get_robot(neigh)) )
                            //                        if( clear_to_target(solution, graph, assign, assign.get_robot(neigh)) )
                        {
                            //neigh and the new position of pebble2 has to be obstacles.    
                            PRX_DEBUG_COLOR("Pebble2 : " << pebble2 << "  is now on : " << print_point(graph, assign.get_position(pebble2)), PRX_TEXT_GREEN);
                            node = graph->get_vertex_as<pmt_graph_node_t > (assign.get_position(pebble2));
                            node->obstacle_id = obstacle_global_id;
                            neigh_node->obstacle_id = obstacle_global_id;

                            ++avoid_global_id;
                            r_node->avoid_id = avoid_global_id;
                            r_node->obstacle_id = 0;
                            w_node->avoid_id = avoid_global_id;
                            w_node->obstacle_id = 0;

                            step(solution, assign, pebble1, neigh);
                            PRX_DEBUG_COLOR("Pebble1 : " << pebble1 << "  is now on : " << print_point(graph, assign.get_position(pebble1)), PRX_TEXT_GREEN);
                            //Get a q that its neither avoidable nor obstacle
                            q = get_valid_adjacent_vertex(graph, w);
                            if( q != NULL )
                            {
                                int blocker_id = assign.get_robot(q);
                                PRX_DEBUG_COLOR("The blocker is : " << blocker_id, PRX_TEXT_BROWN);
                                PRX_ASSERT(blocker_id != -1);

                                two_steps(solution, assign, blocker_id, w, r);
                                PRX_DEBUG_COLOR("w : " << print_point(graph, w) << "   r: " << print_point(graph, r), PRX_TEXT_GREEN);

                                step(solution, assign, pebble1, w);
                                step(solution, assign, pebble2, neigh);

                                PRX_DEBUG_COLOR("Pebble1 : " << pebble1 << "  is now on : " << print_point(graph, assign.get_position(pebble1)), PRX_TEXT_CYAN);
                                PRX_DEBUG_COLOR("Pebble2 : " << pebble2 << "  is now on : " << print_point(graph, assign.get_position(pebble2)), PRX_TEXT_CYAN);
                                //w, q and r, such as neigh from before, are now obstacles.
                                //the previous position of the pebble2 is now open as obstacles and 
                                //be selected. 
                                graph->get_vertex_as<pmt_graph_node_t > (q)->obstacle_id = obstacle_global_id;
                                node->obstacle_id = 0;
                                node->avoid_id = 0;
                                w_node->obstacle_id = obstacle_global_id;
                                r_node->obstacle_id = obstacle_global_id;
                                r_node->avoid_id = 0;

                                //                    if( push_to_empty(solution, graph, assign, r) )
                                if( clear_vertex(solution, graph, assign, assign.get_robot(r)) )
                                    //                                if( clear_to_target(solution, graph, assign, assign.get_robot(r)) )
                                {
                                    //PRX_DEBUG_S("FREE BRANCH 2: ");
                                    //PRX_DEBUG_S("w : " << print_point(graph, w));
                                    //PRX_DEBUG_S("neigh : " << print_point(graph, neigh));
                                    //PRX_DEBUG_S("r : " << print_point(graph, r));
                                    //PRX_DEBUG_S("q : " << print_point(graph, q));
                                    //PRX_DEBUG_S("------------- DONE checking branch " << print_point(graph, w) << " ----------------");
                                    return true;
                                }
                            }

                        }
                    }
                }

                return false;
            }

            void pmt_solver_t::check_visible_branches(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign, util::undirected_vertex_index_t v_start, util::undirected_vertex_index_t u_start, std::deque<util::undirected_vertex_index_t>& uv_path, int path_start, int path_end)
            {
                ++bfs_global_id;
                ++avoid_global_id;
                bool is_v_first = true;

                graph->predecessors[v_start] = v_start;
                node = graph->get_vertex_as<pmt_graph_node_t > (v_start);
                node->bfs_id = bfs_global_id;

                graph->predecessors[u_start] = u_start;
                node = graph->get_vertex_as<pmt_graph_node_t > (u_start);
                node->bfs_id = bfs_global_id;

                static_heap.restart();
                static_heap.push_back(v_start);
                static_heap.push_back(u_start);
                bool is_branch_ok = true;

                int revert_index = solution.size();
                pebble_assignment_t keep_assign = assign;
                undirected_vertex_index_t v_pebble2_start = u_start;
                int pebble1 = assign.get_robot(v_start);
                int pebble2 = assign.get_robot(u_start);

                unsigned int curr_obstacle_id = obstacle_global_id;

//                while( !static_heap.empty() )
//                {
//                    undirected_vertex_index_t v = static_heap.pop_front();
//
//                    foreach(undirected_vertex_index_t adj, boost::adjacent_vertices(v, graph->graph))
//                    {
//                        w = adj;
//
//                        w_node = graph->get_vertex_as<pmt_graph_node_t > (adj);
//                        if( w_node->bfs_id != bfs_global_id )
//                        {
//                            if( w_node->visited_id != visited_global_id && w_node->obstacle_id != curr_obstacle_id )
//                            {
//                                graph->predecessors[adj] = v;
//                                if( w_node->is_branch )
//                                {
//                                    v_w_path_length = 0;
//                                    for( undirected_vertex_index_t p = adj; p != v_start || p != u_start; p = graph->predecessors[p] )
//                                    {
//                                        v_w_path[v_w_path_length] = p;
//                                        ++v_w_path_length;
//                                    }
//                                    if( p == v_start )
//                                    {
//                                        v_w_path[v_w_path_length] = v_start;
//                                        is_v_first = true;
//
//                                        neigh = uv_path[path_start + 1];
//                                        if( path_length > 2 )
//                                        {
//                                            PRX_DEBUG_S("Will move pebble " << pebble2 << "  from pos:" << print_point(graph, assign.get_position(pebble2)) << " -> " << print_point(graph, neigh));
//                                            solution.push_back(pebble2, uv_path, path_end, path_start + 1);
//                                            assign.change_position(pebble2, neigh);
//                                        }
//
//
//                                    }
//                                    else
//                                    {
//                                        v_w_path[v_w_path_length] = u_start;
//                                        is_v_first = false;
//                                    }
//                                    ++v_w_path_length;
//
//                                    PRX_ASSERT(w == v_w_path[0]);
//
//                                    ++obstacle_global_id;
//
//                                    graph->get_vertex_as<pmt_graph_node_t > (v_pebble2_start)->obstacle_id = obstacle_global_id;
//
//                                    undirected_vertex_index_t v_prev = v_pebble2_start;
//                                    for( int i = v_w_path_length - 2; i >= 0; --i )
//                                    {
//                                        graph->get_vertex_as<pmt_graph_node_t > (v_w_path[i + 1])->obstacle_id = obstacle_global_id;
//
//                                        if( assign.has_robot_on(v_w_path[i]) )
//                                        {
//                                            //                                int pidd = assign.get_robot(v_w_path[i]);
//                                            //PRX_DEBUG_S("going to push to empty pebble " << pidd << " from: " << print_point(graph, v_w_path[i]));
//                                            //                                if( !push_to_empty(solution, graph, assign, v_w_path[i]) )
//                                            if( !clear_vertex(solution, graph, assign, assign.get_robot(v_w_path[i])) )
//                                                //                                            if( !clear_to_target(solution, graph, assign, assign.get_robot(v_w_path[i])) )
//                                            {
//                                                is_branch_ok = false;
//                                                break;
//                                            }
//                                            //PRX_DEBUG_S("Pebble " << pidd << " is now at: " << print_point(graph, assign.get_position(pidd)));
//                                        }
//
//                                        step(solution, assign, pebble1, v_w_path[i]);
//                                        step(solution, assign, pebble2, v_w_path[i + 1]);
//                                        graph->get_vertex_as<pmt_graph_node_t > (v_prev)->obstacle_id = 0;
//                                        v_prev = v_w_path[i + 1];
//                                    }
//
//                                    PRX_DEBUG_COLOR("Done with the moving now checks if the branch is ok", PRX_TEXT_MAGENTA);
//                                    PRX_DEBUG_COLOR("pebble : " << pebble1 << "  is at " << print_point(graph, assign.get_position(pebble1)) << "  pebble2:" << pebble2 << " is at:" << print_point(graph, assign.get_position(pebble2)), PRX_TEXT_MAGENTA);
//                                }
//                                else
//                                    static_heap.push_back(adj);
//                            }
//                            w_node->bfs_id = bfs_global_id;
//                        }
//                    }
//                }
//                w = NULL;
//                w_node = NULL;
//                return false;
            }

            bool pmt_solver_t::check_visible_branches(pebble_solution_path_t& solution, undirected_graph_t* graph, pebble_assignment_t& assign, undirected_vertex_index_t v_start, bool caller_on_w)
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
                                            //                                if( !push_to_empty(solution, graph, assign, v_w_path[i]) )
                                            if( !clear_vertex(solution, graph, assign, assign.get_robot(v_w_path[i])) )
                                                //                                            if( !clear_to_target(solution, graph, assign, assign.get_robot(v_w_path[i])) )
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
                                    PRX_DEBUG_COLOR("pebble : " << pebble1 << "  is at " << print_point(graph, assign.get_position(pebble1)) << "  pebble2:" << pebble2 << " is at:" << print_point(graph, assign.get_position(pebble2)), PRX_TEXT_MAGENTA);
                                    if( is_branch_ok )
                                    {
                                        neigh = v_w_path[1];
                                        PRX_ASSERT(w == v_w_path[0]);
                                        //PRX_DEBUG_S("pebble " << pebble1 << "  is now " << print_point(graph, assign.get_position(pebble1)) << "   and pebble2 " << pebble2 << "  at point: " << print_point(graph, assign.get_position(pebble2)));
                                        if( check_branch(solution, graph, assign) )
                                        {
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

            bool pmt_solver_t::clear_branch(pebble_solution_path_t& solution, undirected_graph_t* graph, pebble_assignment_t& assign, int& empty_vertices)
            {
                ++obstacle_global_id;

                foreach(undirected_vertex_index_t adj, boost::adjacent_vertices(w, graph->graph))
                {
                    if( adj != neigh && adj != r )
                    {
                        if( !assign.has_robot_on(adj) )
                        {
                            if( r == NULL )
                            {
                                empty_vertices = 1;
                                r = adj;
                                r_node = graph->get_vertex_as<pmt_graph_node_t > (r);

                            }
                            else
                            {
                                q = adj;
                                empty_vertices = 2;
                                return true;
                            }
                        }
                    }
                }

                ++avoid_global_id;

                neigh_node->obstacle_id = obstacle_global_id;
                w_node->obstacle_id = obstacle_global_id;
                if( r != NULL )
                    r_node->avoid_id = avoid_global_id;

                //If we reach here the adj vertex will definitely be a occupied vertex. The only un-occupied
                //vertex will be the r. If there were more than 2 free vertices the algorithm will have stopped 
                //earlier. Thats why we are checking immediately if adj can be cleaned by the robot.

                foreach(undirected_vertex_index_t adj, boost::adjacent_vertices(w, graph->graph))
                {
                    if( adj != neigh && adj != r )
                    {
                        //                        if( clear_to_target(solution, graph, assign, assign.get_robot(adj)) )
                        if( clear_vertex(solution, graph, assign, assign.get_robot(adj)) )
                        {
                            PRX_DEBUG_COLOR("Clear vertex " << print_point(graph, adj), PRX_TEXT_GREEN);
                            if( r == NULL )
                            {
                                empty_vertices = 1;
                                r = adj;
                                r_node = graph->get_vertex_as<pmt_graph_node_t > (r);
                                r_node->avoid_id = avoid_global_id;
                            }
                            else
                            {
                                q = adj;
                                empty_vertices = 2;
                                return true;
                            }
                        }

                    }
                }

                return false;

            }

            bool pmt_solver_t::clear_vertex(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign, int pebble_id)
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
                        if( blocker_id != -1 )
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

            std::pair<bool, int> pmt_solver_t::path_to_empty(std::deque<undirected_vertex_index_t>& path, const undirected_graph_t* graph, pebble_assignment_t& assign, undirected_vertex_index_t v_start)
            {
                ++bfs_global_id;
                undirected_vertex_index_t v;
                int path_len = 0;

                static_heap.restart();
                static_heap.push_back(v_start);
                graph->get_vertex_as<pmt_graph_node_t > (v_start)->bfs_id = bfs_global_id;
                graph->predecessors[v_start] = v_start;

                PRX_DEBUG_COLOR("Wants to push to empty pebble " << assign.get_robot(v_start) << "  from the position : " << print_point(graph, v_start), PRX_TEXT_GREEN);
                while( !static_heap.empty() )
                {
                    v = static_heap.pop_front();

                    foreach(undirected_vertex_index_t adj, boost::adjacent_vertices(v, graph->graph))
                    {
                        node = graph->get_vertex_as<pmt_graph_node_t > (adj);
                        if( node->bfs_id != bfs_global_id )
                        {
                            if( node->obstacle_id != obstacle_global_id )
                            {
                                graph->predecessors[adj] = v;

                                if( node->avoid_id != avoid_global_id && !assign.has_robot_on(adj) )
                                {
                                    PRX_DEBUG_COLOR("found free position " << print_point(graph, adj), PRX_TEXT_GREEN);
                                    tmp_empty_path_length = 0;

                                    while( adj != v_start )
                                    {
                                        tmp_empty_path[tmp_empty_path_length] = adj;
                                        adj = graph->predecessors[adj];
                                        ++tmp_empty_path_length;
                                    }
                                    PRX_ASSERT(adj == v_start);
                                    path[path_len] = v_start;
                                    //                        PRX_DEBUG_COLOR("tmp_vectror_index: " << tmp_empty_path_length, PRX_TEXT_BLUE);
                                    for( int i = tmp_empty_path_length - 1; i >= 0; --i )
                                    {
                                        //                            PRX_DEBUG_COLOR("in tmp vector[" << i << "]:" << print_point(graph,tmp_empty_path[i]),PRX_TEXT_GREEN);
                                        ++path_len;
                                        path[path_len] = tmp_empty_path[i];
                                        //                            PRX_DEBUG_COLOR(path_len << " - " << i << ") path to empty : " << print_point(graph, path[path_len]), PRX_TEXT_CYAN);
                                    }
                                    ++path_len;
                                    return std::make_pair(true, path_len);
                                }
                                static_heap.push_back(adj);
                                graph->predecessors[adj] = v;
                            }
                            node->bfs_id = bfs_global_id;
                        }
                    }
                }
                return std::make_pair(false, path_len);
            }

            std::pair<bool, int> pmt_solver_t::find_a_path(std::deque<undirected_vertex_index_t>& path, undirected_graph_t* graph, pebble_assignment_t& assign, undirected_vertex_index_t v_start, undirected_vertex_index_t v_target, int path_length, bool avoid_path, bool block_path)
            {
                pmt_heuristic->set(v_start, &assign);
                pmt_visitor->set_goal(v_start);
                try
                {
                    astar_search<
                            undirected_graph_type,
                            undirected_graph_t,
                            undirected_vertex_index_t,
                            pmt_default_distance_heuristic_t,
                            pmt_astar_visitor_t
                            > (graph, v_target, pmt_heuristic, pmt_visitor);
                }
                catch( prx_found_goal_t f_goal )
                {
                    for( undirected_vertex_index_t v = v_start; v != v_target; v = graph->predecessors[v] )
                    {
                        node = graph->get_vertex_as<pmt_graph_node_t > (v);
                        if( block_path )
                            node->obstacle_id = obstacle_global_id;
                        if( avoid_path )
                            node->avoid_id = avoid_global_id;
                        path[path_length] = v;
                        ++path_length;
                    }
                    node = graph->get_vertex_as<pmt_graph_node_t > (v_target);
                    if( block_path )
                        node->obstacle_id = obstacle_global_id;
                    if( avoid_path )
                        node->avoid_id = avoid_global_id;
                    path[path_length] = v_target;
                    ++path_length;
                    return std::make_pair(true, path_length);
                }

                return std::make_pair(false, path_length);
            }

            std::pair<bool, int> pmt_solver_t::find_path(std::deque<undirected_vertex_index_t>& path, undirected_graph_t* graph, pebble_assignment_t& assign, undirected_vertex_index_t v_start, undirected_vertex_index_t v_target, int path_length, bool avoid_path, bool block_path)//, bool check_pebble_obstacles)
            {
                bool check_pebble_obstacles = false;

                node = graph->get_vertex_as<pmt_graph_node_t > (v_target);
                if( node->obstacle_id != obstacle_global_id && node->avoid_id != avoid_global_id )
                {
                    PRX_DEBUG_COLOR("Target " << print_point(graph, v_target) << "   for pebble " << assign.get_robot(v_start) << "   is not obstacle", PRX_TEXT_GREEN);
                    ++bfs_global_id;
                    undirected_vertex_index_t v;
                    bool free_from_pebble;

                    static_heap.restart();
                    static_heap.push_back(v_target);
                    node->bfs_id = bfs_global_id;

                    graph->predecessors[v_target] = v_target;
                    while( !static_heap.empty() )
                    {
                        v = static_heap.pop_front();

                        foreach(undirected_vertex_index_t adj, boost::adjacent_vertices(v, graph->graph))
                        {
                            node = graph->get_vertex_as<pmt_graph_node_t > (adj);

                            //            //PRX_INFO_S("point : " << print_point(graph,adj) << "   start:" << print_point(graph,v_start;));
                            //            //PRX_INFO_S("bfs:" << bfs_global_id << "  obstacle_id:" << obstacle_global_id << "  visited:" << visited_global_id << "  avoid:" << avoid_global_id);
                            //            //PRX_DEBUG_S(node->print());
                            if( node->bfs_id != bfs_global_id )
                            {
                                if( node->obstacle_id != obstacle_global_id )
                                {
                                    free_from_pebble = true;
                                    if( check_pebble_obstacles )
                                    {
                                        int blocker_id = assign.get_robot(adj);
                                        if( blocker_id != -1 && adj == t_assignment.get_position(blocker_id) )
                                            free_from_pebble = false;
                                    }

                                    if( free_from_pebble )
                                    {
                                        graph->predecessors[adj] = v;
                                        if( adj == v_start )
                                        {
                                            for( v = v_start; v != v_target; v = graph->predecessors[v] )
                                            {
                                                node = graph->get_vertex_as<pmt_graph_node_t > (v);
                                                if( block_path )
                                                    node->obstacle_id = obstacle_global_id;
                                                if( avoid_path )
                                                    node->avoid_id = avoid_global_id;
                                                path[path_length] = v;
                                                ++path_length;
                                            }
                                            node = graph->get_vertex_as<pmt_graph_node_t > (v);
                                            if( block_path )
                                                node->obstacle_id = obstacle_global_id;
                                            if( avoid_path )
                                                node->avoid_id = avoid_global_id;
                                            path[path_length] = v_target;
                                            ++path_length;
                                            return std::make_pair(true, path_length);
                                        }

                                        static_heap.push_back(adj);
                                    }
                                }
                                node->bfs_id = bfs_global_id;
                            }
                        }
                    }
                }

                return std::make_pair(false, path_length);
            }

            bool pmt_solver_t::is_on_path(undirected_graph_t* graph, undirected_vertex_index_t v_start, undirected_vertex_index_t v_target, undirected_vertex_index_t v_search)
            {
                if( v_start == v_search || v_target == v_search )
                    return true;

                ++bfs_global_id;
                undirected_vertex_index_t v;

                static_heap.restart();
                static_heap.push_back(v_target);
                graph->get_vertex_as<pmt_graph_node_t > (v_target)->bfs_id = bfs_global_id;

                graph->predecessors[v_target] = v_target;
                while( !static_heap.empty() )
                {
                    v = static_heap.pop_front();

                    foreach(undirected_vertex_index_t adj, boost::adjacent_vertices(v, graph->graph))
                    {
                        node = graph->get_vertex_as<pmt_graph_node_t > (adj);
                        if( node->bfs_id != bfs_global_id )
                        {
                            //                if( node->obstacle_id != obstacle_global_id )
                            //                {
                            graph->predecessors[adj] = v;
                            if( adj == v_start )
                            {
                                for( v = v_start; v != v_target; v = graph->predecessors[v] )
                                {
                                    if( v == v_search )
                                        return true;
                                }
                                return false;
                            }
                            static_heap.push_back(adj);
                            //                }
                        }
                        node->bfs_id = bfs_global_id;
                    }
                }
                return false;
            }

            void pmt_solver_t::compute_the_order(undirected_graph_t* graph, pebble_assignment_t& assignment)
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
                        order[index] = assignment.get_robot(v);
                        ++index;
                    }

                    spanning_tree.remove_vertex(v_tree);

                    if( v_par_tree != v_tree && !spanning_tree.has_children(v_par_tree) )
                        if( !static_heap.has(v_par_tree) )
                            static_heap.push_back(v_par_tree);
                }
            }

            std::pair<int, int> pmt_solver_t::find_blocker_on_path(const std::deque<undirected_vertex_index_t>& path, int path_length, pebble_assignment_t& assign)
            {
                int blocker_id = -1;
                for( int p = 1; p < path_length; ++p )
                {
                    blocker_id = assign.get_robot(path[p]);
                    if( blocker_id != -1 )
                    {
                        return std::make_pair(blocker_id, p);
                    }
                }

                return std::make_pair(blocker_id, 0);
            }

            undirected_vertex_index_t pmt_solver_t::get_valid_adjacent_vertex(undirected_graph_t* graph, undirected_vertex_index_t v)
            {

                foreach(undirected_vertex_index_t adj, boost::adjacent_vertices(v, graph->graph))
                {
                    if( graph->get_vertex_as<pmt_graph_node_t > (adj)->avoid_id != avoid_global_id && graph->get_vertex_as<pmt_graph_node_t > (adj)->obstacle_id != obstacle_global_id )
                        return adj;
                }
                return NULL;
            }

            void pmt_solver_t::step(pebble_solution_path_t& solution, pebble_assignment_t& assign, int pebble_id, undirected_vertex_index_t v_step)
            {
                PRX_DEBUG_COLOR("pebble " << pebble_id << "  from point: " << print_point(initial_graph, assign.get_position(pebble_id)) << " (" << assign.get_position(pebble_id) << " ) -> " << print_point(initial_graph, v_step) << " (" << v_step << " )", PRX_TEXT_LIGHTGRAY);
                undirected_vertex_index_t v_curr = assign.get_position(pebble_id);
                if( v_curr != v_step )
                {
                    solution.push_back(pebble_id, v_curr, v_step);
                    assign.change_position(pebble_id, v_step);
                    PRX_ASSERT(v_step == assign.get_position(pebble_id));
                }
            }

            void pmt_solver_t::two_steps(pebble_solution_path_t& solution, pebble_assignment_t& assign, int pebble_id, undirected_vertex_index_t v_step1, undirected_vertex_index_t v_step2)
            {
                PRX_DEBUG_COLOR("pebble " << pebble_id << "  from point: " << print_point(initial_graph, assign.get_position(pebble_id)) << " -> " << print_point(initial_graph, v_step1), PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("pebble " << pebble_id << "  from point: " << print_point(initial_graph, v_step1) << " -> " << print_point(initial_graph, v_step2), PRX_TEXT_LIGHTGRAY);
                undirected_vertex_index_t v_curr = assign.get_position(pebble_id);
                if( v_curr != v_step2 )
                {
                    solution.push_back(pebble_id, v_curr, v_step1);
                    solution.push_back(pebble_id, v_step1, v_step2);
                    assign.change_position(pebble_id, v_step2);
                    PRX_ASSERT(v_step2 == assign.get_position(pebble_id));
                }
            }
        }
    }
}