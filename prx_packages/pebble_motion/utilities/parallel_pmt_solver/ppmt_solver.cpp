/**
 * @file ppmt_solver.cpp
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

#include "utilities/parallel_pmt_solver/ppmt_solver.hpp"
#include "utilities/pebble_spanning_tree.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"

#include <boost/tuple/tuple.hpp> // boost::tie

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::packages::pebble_motion::ppmt_solver_t, prx::packages::pebble_motion::pebble_solver_t)

namespace prx
{
    using namespace util;

    namespace packages
    {
        namespace pebble_motion
        {

            ppmt_solver_t::ppmt_solver_t() { }

            ppmt_solver_t::~ppmt_solver_t()
            {
                delete pmt_visitor;
                delete pmt_heuristic;
            }

            void ppmt_solver_t::setup(undirected_graph_t* graph, pebble_assignment_t& start_assign, pebble_assignment_t& target_assign, int num_robots, const space_t* space, distance_metric_t* distance_metric)
            {
                pmt_solver_t::setup(graph, start_assign, target_assign, num_robots, space, distance_metric);
                used_global_id = 0;
                busy_global_id = 0;
                agents_list.resize(k);

                for( int i = 0; i < k; ++i )
                    agents_list[i].setup(i, number_of_vertices, start_assign.get_position(i), target_assign.get_position(i));

                to_evacuate.resize(k);
                evacuate_len = 0;
            }

            undirected_vertex_index_t ppmt_solver_t::add_new_vertex(undirected_graph_t* graph)
            {
                return graph->add_vertex<ppmt_graph_node_t > ();
            }

            void ppmt_solver_t::get_parallel_steps(std::vector<int>& steps)
            {
                steps = par_steps;
            }

            bool ppmt_solver_t::find_solution(std::vector<pebble_step_t>* solution, undirected_graph_t* graph)
            {
                int pebble_id;
                moving_action_t act;

                update_info(graph, t_assignment);
                compute_the_order(graph, t_assignment);

                s_new = s_assignment;
                PRX_DEBUG_COLOR("------------    strt    ------------", PRX_TEXT_BROWN);
                print_assignment(graph, s_assignment);
                PRX_DEBUG_COLOR("------------    s_new   ------------", PRX_TEXT_BROWN);
                print_assignment(graph, s_new);
                PRX_DEBUG_COLOR("-----------     targ    ------------", PRX_TEXT_BROWN);
                print_assignment(graph, t_assignment);
                PRX_DEBUG_COLOR("------------------------------------", PRX_TEXT_BROWN);

                int iter;

                while( s_new != t_assignment )
                {
                    PRX_DEBUG_COLOR("-------------------    START ALL THE PEBBLES PPMT --------------------", PRX_TEXT_RED);
                    print_assignment(graph, s_new, PRX_TEXT_RED);
                    busy_global_id++;
                    used_global_id++;

                    for( iter = 0; iter < k; ++iter )
                    {
                        pebble_id = order[iter];
                        ppmt_agent_t* agent = &agents_list[pebble_id];
                        if( agent->is_swapping() && !agent->is_busy(busy_global_id) )
                        {
                            PRX_DEBUG_COLOR("--------------- SWAP OP: PEBBLE " << pebble_id << "  --------------------", PRX_TEXT_MAGENTA);
                            swap(solution_path, graph, s_new, agent);
                        }
                    }

                    for( iter = 0; iter < k; ++iter )
                    {
                        pebble_id = order[iter];
                        ppmt_agent_t* agent = &agents_list[pebble_id];
                        if( agent->is_free(busy_global_id) )
                        {
                            PRX_DEBUG_COLOR("-------------------  PEBBLE " << pebble_id << "  --------------------", PRX_TEXT_MAGENTA);
                            priority_cycle_detected = false;
                            called_size = 0;

                            act = move_agent(solution_path, graph, s_new, agent);
                            if( act == MA_SWAP_FAILED )
                            {
                                if( solution_path.size() > 0 )
                                    par_steps.push_back(solution_path.size() - 1);
                                solution_path.convert(solution, s_assignment);
                                return false;
                            }
                        }
                    }
                    PRX_DEBUG_COLOR("solution size: " << solution_path.size(), PRX_TEXT_RED);
                    if( solution_path.size() > 0 )
                        par_steps.push_back(solution_path.size() - 1);
                }

                solution_path.convert(solution, s_assignment);
                return true;
            }

            ppmt_solver_t::moving_action_t ppmt_solver_t::move_agent(pebble_solution_path_t& solution, undirected_graph_t* graph, pebble_assignment_t& assign, ppmt_agent_t* agent)
            {

                undirected_vertex_index_t v_start = agent->v_curr;
                if( agent->on_goal(v_start) )
                    return MA_WAIT;

                int blocker_id;
                ppmt_agent_t* blocker;
                bool path_exists;

                ++avoid_global_id;
                ++obstacle_global_id;

                PRX_ASSERT(called_size >= 0 && called_size < k);
                agent->called = true;
//                called[called_size] = agent->id;
                ++called_size;

                PRX_DEBUG_COLOR("------------    assign in move to goal    ------------", PRX_TEXT_CYAN);
                print_assignment(graph, assign, PRX_TEXT_CYAN);

                if( !agent->has_path() )
                {
                    PRX_DEBUG_COLOR("Compute new path to " << print_point(graph, agent->get_goal()), PRX_TEXT_GREEN);
                    agent->clear_path();
                    boost::tie(path_exists, agent->path_length) = find_sort_path(agent->tmp_path, graph, assign, v_start, agent->get_goal());

                    if( !path_exists )
                        PRX_LOG_ERROR("Path between start and goal of the pebble %d does not exist.", agent->id);
                }

                PRX_DEBUG_COLOR("Going to move pebble " << agent->id << "  from :" << print_point(graph, v_start) << " -> " << print_point(graph, agent->get_goal()) << "  length : " << agent->path_length - agent->path_start, PRX_TEXT_GREEN);

                //                boost::tie(blocker_id, pos) = find_blocker_on_path(agent, assign);

                undirected_vertex_index_t v_step = agent->get_next_step();
                blocker_id = assign.get_robot(v_step);

                if( blocker_id != -1 )
                {
                    blocker = &agents_list[blocker_id];
                    PRX_DEBUG_COLOR("Found blocker: " << blocker_id << "  at the position:" << print_point(graph, v_step), PRX_TEXT_BROWN);
                }

                moving_action_t action = MA_WAIT;
                if( blocker_id == -1 || !blocker->is_free(busy_global_id) )
                {
                    if( can_agent_move(graph, assign, agent) )
                        action = step_agent(solution, graph, assign, agent);
                }
                else // blocker is free and can do anything with this agent.
                {
                    if( need_swap(graph, agent, blocker, v_step) )
                    {
                        if( !prepare_swap(solution, graph, assign, agent, blocker) )
                        {
                            PRX_ERROR_S("SWAP between " + int_to_str(agent->id) + " and " + int_to_str(blocker_id) + " FAILED!");
                            return MA_SWAP_FAILED;
                        }
                        action = MA_SWAP;
                    }
                    else
                    {
                        if( blocker->called ) //std::find(called.begin(), called.begin() + called_size, blocker_id) != called.begin() + called_size )
                        {
                            PRX_DEBUG_COLOR("Cycle is detected with pebble : " << agent->id << " - blocker: " << blocker_id, PRX_TEXT_BROWN);
                            step(solution, assign, agent->id, v_step);
                            priority_cycle_detected = true;
                            return MA_CYCLE;
                        }

                        blocker->clear_path();
                        moving_action_t act = move_agent(solution, graph, assign, blocker);
                        if( act == MA_MOVE )
                        {
                            action = move_agent(solution, graph, assign, agent);
                        }
                        else if( act == MA_CYCLE )
                        {
                            step(solution, assign, agent->id, v_step);
                            return MA_CYCLE;
                        }
                        else if( act == MA_SWAP_FAILED )
                        {
                            return MA_SWAP_FAILED;
                        }
                        else
                        {
                            //If a swap happens later in the chain the agent will try to move after the swap.
                            action = step_agent(solution, graph, assign, agent);
                        }
                    }
                }

                agent->called = false;
                --called_size;
                PRX_DEBUG_COLOR("Agent " << agent->id << " did the action : " << action, PRX_TEXT_BLUE);
                return action;
            }

            bool ppmt_solver_t::need_swap(undirected_graph_t* graph, ppmt_agent_t* agent, ppmt_agent_t* blocker, undirected_vertex_index_t blocker_pos)
            {
                undirected_vertex_index_t blocker_ends;
                if( blocker->has_path() )
                {
                    if( std::find(blocker->tmp_path.begin() + blocker->path_start, blocker->tmp_path.begin() + blocker->path_length, agent->get_current_step()) != (blocker->tmp_path.begin() + blocker->path_length) )
                        return true;
                    blocker_ends = blocker->get_last_step();
                }
                else
                {
                    blocker_ends = blocker->goal;
                    if( is_on_path(graph, blocker_pos, blocker_ends, agent->get_current_step()) )
                        return true;
                }

                if( std::find(agent->tmp_path.begin() + agent->path_start, agent->tmp_path.begin() + agent->path_length, blocker_ends) != (agent->tmp_path.begin() + agent->path_length) )
                {
                    //PRX_INFO_S("need swap because P2 " << print_point(graph, final_p2) << "  belongs on the path of P1: " << print_point(graph, path[path_start]) << " - " << print_point(graph, path[path_length - 1]));
                    return true;
                }
                return false;
            }

            bool ppmt_solver_t::prepare_swap(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign, ppmt_agent_t* agent, ppmt_agent_t* blocker)
            {
                undirected_vertex_index_t v = assign.get_position(agent->id);
                undirected_vertex_index_t u = assign.get_position(blocker->id);
                undirected_vertex_index_t v_tmp;
                PRX_DEBUG_COLOR("SWAP : " << agent->id << " - " << blocker->id << "   v:" << print_point(graph, agent->v_curr) << " - " << print_point(graph, blocker->v_curr), PRX_TEXT_CYAN);
                pebble_assignment_t keep_assign = assign;
                ++visited_global_id;
                ++obstacle_global_id;
                ++avoid_global_id;

                agent->swapping_agent = blocker;
                blocker->swapping_agent = agent;

                //If the vertex u is already on a branch.
                w_node = graph->get_vertex_as<pmt_graph_node_t > (u);

                if( w_node->is_branch )
                {
                    PRX_DEBUG_COLOR("for Criterion w ON U : " << print_point(graph, u), PRX_TEXT_GREEN);
                    w = u;
                    r = NULL;
                    q = NULL;
                    neigh = v;
                    evacuate_len = 0;
                    if( check_branch(solution, graph, assign) )
                    {
                        PRX_DEBUG_COLOR("CRITERION ON U: w " << print_point(graph, u) << " ON U worked", PRX_TEXT_BROWN);
                        agent->start_swap(SW_SWAPPING, w, neigh, r, q);
                        blocker->start_swap(SW_SWAPPING, w, neigh, r, q);

                        graph->get_vertex_as<ppmt_graph_node_t > (w)->lock(true);
                        graph->get_vertex_as<ppmt_graph_node_t > (r)->lock(true);
                        graph->get_vertex_as<ppmt_graph_node_t > (q)->lock(true);
                        graph->get_vertex_as<ppmt_graph_node_t > (neigh)->lock(true);

                        blocker->add_new_step(w);
                        blocker->add_new_step(r);
                        blocker->add_new_step(w);
                        blocker->add_new_step(neigh);
                        agent->add_new_step(neigh);
                        agent->add_new_step(w);
                        agent->add_new_step(q);
                        agent->add_new_step(w);

                        to_evacuate[evacuate_len++] = blocker;
                        to_evacuate[evacuate_len++] = agent;
                        for( int i = 0; i < evacuate_len; ++i )
                        {
                            PRX_DEBUG_COLOR("pos:" << i << "  has agent: " << to_evacuate[i]->id, PRX_TEXT_BROWN);
                            swap(solution, graph, assign, to_evacuate[i]);
                        }
                        return true;
                    }
                    assign = keep_assign;
                }

                ++obstacle_global_id;

                //If v is branch
                w_node = graph->get_vertex_as<pmt_graph_node_t > (v);
                if( w_node->is_branch )
                {
                    PRX_DEBUG_COLOR("for Criterion w ON V : " << print_point(graph, v), PRX_TEXT_GREEN);
                    w = v;
                    r = NULL;
                    q = NULL;
                    neigh = u;
                    evacuate_len = 0;
                    if( check_branch(solution, graph, assign) )
                    {
                        PRX_DEBUG_COLOR("CRITERION ON V : w " << print_point(graph, v) << " ON V worked ", PRX_TEXT_BROWN);
                        agent->start_swap(SW_SWAPPING, w, neigh, r, q);
                        blocker->start_swap(SW_SWAPPING, w, neigh, r, q);

                        graph->get_vertex_as<ppmt_graph_node_t > (w)->lock(true);
                        graph->get_vertex_as<ppmt_graph_node_t > (r)->lock(true);
                        graph->get_vertex_as<ppmt_graph_node_t > (q)->lock(true);
                        graph->get_vertex_as<ppmt_graph_node_t > (neigh)->lock(true);

                        agent->add_new_step(w);
                        agent->add_new_step(r);
                        agent->add_new_step(w);
                        agent->add_new_step(neigh);
                        blocker->add_new_step(neigh);
                        blocker->add_new_step(w);
                        blocker->add_new_step(q);
                        blocker->add_new_step(w);

                        to_evacuate[evacuate_len++] = blocker;
                        to_evacuate[evacuate_len++] = agent;
                        for( int i = 0; i < evacuate_len; ++i )
                        {
                            PRX_DEBUG_COLOR("pos:" << i << "  has agent: " << to_evacuate[i]->id, PRX_TEXT_BROWN);
                            swap(solution, graph, assign, to_evacuate[i]);
                        }
                        return true;
                    }
                    assign = keep_assign;
                }

                //
                //                neigh = agent->tmp_path[path_end - 1];
                //                if( path_length > 2 )
                //                {
                //                    PRX_DEBUG_S("Will move pebble " << agent->id << "  from pos:" << print_point(graph, assign.get_position(agent->id)) << " -> " << print_point(graph, neigh));
                //                    solution.push_back(agent->id, agent->tmp_path, agent->path_start, path_end - 1);
                //                    assign.change_position(agent->id, neigh);
                //                }
                //
                //                neigh_node = graph->get_vertex_as<pmt_graph_node_t > (neigh);
                //                r = NULL;
                //                q = NULL;
                //                if( check_visible_branches(solution, graph, assign, u, false) )
                //                    return true;
                //                solution.revert(revert_index);
                //                assign = keep_assign;
                //
                //
                //                ++obstacle_global_id;
                //                neigh = agent->tmp_path[agent->path_start + 1];
                //                if( path_length > 2 )
                //                {
                //                    PRX_DEBUG_S("Will move pebble " << blocker->id << "  from pos:" << print_point(graph, assign.get_position(blocker->id)) << " -> " << print_point(graph, neigh));
                //                    solution.push_back(blocker->id, agent->tmp_path, path_end, agent->path_start + 1);
                //                    assign.change_position(blocker->id, neigh);
                //                }
                //
                //                neigh_node = graph->get_vertex_as<pmt_graph_node_t > (neigh);
                //                r = NULL;
                //                q = NULL;
                //                if( check_visible_branches(solution, graph, assign, v, true) )
                //                    return true;
                //                solution.revert(revert_index);
                //                assign = keep_assign;

                PRX_ERROR_S("There is no solution!  SWAP between : " << agent->id << " - " << blocker->id << " FAILED!!!");
                return false;
            }

            ppmt_solver_t::moving_action_t ppmt_solver_t::swap(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign, ppmt_agent_t* agent)
            {
                if( agent->is_busy(busy_global_id) )
                    return MA_WAIT;

                PRX_DEBUG_COLOR("agent      " << agent->id << "   in swap : " << agent->status << "  steps: " << agent->path_start << "/" << agent->path_length, PRX_TEXT_GREEN);
                if( agent->swapping_agent )
                    PRX_DEBUG_COLOR("agent pair " << agent->swapping_agent->id << "   in swap :" << agent->swapping_agent->status << "  steps: " << agent->swapping_agent->path_start << "/" << agent->swapping_agent->path_length << "   busy:" << agent->swapping_agent->is_busy(busy_global_id), PRX_TEXT_CYAN);

                //                if( agent->status == SW_READY )
                //                {
                //                    if( agent->swapping_agent->status == SW_READY && !agent->swapping_agent->is_busy(busy_global_id) )
                //                    {
                //                        PRX_DEBUG_COLOR("START SWAPPING", PRX_TEXT_RED);
                //                        agent->clear_swap_path();
                //                        agent->swapping_agent->clear_swap_path();
                //                        if( assign.get_position(agent->id) == agent->w )
                //                        {
                //                            step(solution, assign, agent->id, agent->r);
                //                            step(solution, assign, agent->swapping_agent->id, agent->w);
                //                            agent->add_swap_step(agent->r);
                //                            agent->add_swap_step(agent->w);
                //                            agent->add_swap_step(agent->neigh);
                //                            agent->swapping_agent->add_swap_step(agent->w);
                //                            agent->swapping_agent->add_swap_step(agent->q);
                //                            agent->swapping_agent->add_swap_step(agent->w);
                //                        }
                //                        else
                //                        {
                //                            step(solution, assign, agent->id, agent->w);
                //                            step(solution, assign, agent->swapping_agent->id, agent->r);
                //                            agent->add_swap_step(agent->w);
                //                            agent->add_swap_step(agent->q);
                //                            agent->add_swap_step(agent->w);
                //                            agent->swapping_agent->add_swap_step(agent->r);
                //                            agent->swapping_agent->add_swap_step(agent->w);
                //                            agent->swapping_agent->add_swap_step(agent->neigh);
                //                        }
                //                        agent->busy = busy_global_id;
                //                        agent->swapping_agent->busy = busy_global_id;
                //                        agent->status = SW_SWAPPING;
                //                        agent->swapping_agent->status = SW_SWAPPING;
                //
                //                        return MA_MOVE;
                //                    }
                //                    graph->get_vertex_as<ppmt_graph_node_t > (assign.get_position(agent->id))->used_id = used_global_id;
                //                    return MA_WAIT;
                //                }

                undirected_vertex_index_t v;
                undirected_vertex_index_t v_curr = agent->v_curr;

                //                if( agent->status == SW_MOVING )
                //                {
                //                    if( blocker_id != -1 )
                //                    {
                //                        PRX_DEBUG_COLOR("Agent: " << agent->id << ") Blocker : " << blocker_id << "  is blocking my SW_MOVING progress", PRX_TEXT_RED);
                //                    }
                //                    graph->get_vertex_as<ppmt_graph_node_t > (v_prev)->used_id = used_global_id - 1;
                //                    graph->get_vertex_as<ppmt_graph_node_t > (v)->used_id = used_global_id;
                //                    if( agent->done_swap_action() )
                //                        agent->status = SW_READY;
                //
                //                    return MA_MOVE;
                //
                //                }
                //                else 
                if( agent->status == SW_SWAPPING )
                {
                    v = agent->get_next_step();
                    step(solution, assign, agent->id, v);

                    if( !agent->has_path() )
                    {
                        PRX_DEBUG_COLOR("DONE WITH SWAPPING", PRX_TEXT_RED);
                        graph->get_vertex_as<ppmt_graph_node_t > (agent->w)->lock(false);
                        graph->get_vertex_as<ppmt_graph_node_t > (agent->r)->lock(false);
                        graph->get_vertex_as<ppmt_graph_node_t > (agent->q)->lock(false);
                        graph->get_vertex_as<ppmt_graph_node_t > (agent->neigh)->lock(false);
                        agent->done_swap();
                    }

                    return MA_MOVE;
                }
                else if( agent->status == SW_EVACUATE )
                {
                    v = agent->get_next_step();
                    step(solution, assign, agent->id, v);
                    agent->done_evacuate();
                    return MA_MOVE;
                }

                return MA_WAIT;
            }

            bool ppmt_solver_t::check_branch(pebble_solution_path_t& solution, undirected_graph_t* graph, pebble_assignment_t& assign)
            {

                PRX_DEBUG_COLOR("Check branch : " << print_point(graph, w), PRX_TEXT_GREEN);
                int pebble1 = assign.get_robot(w);
                int pebble2 = assign.get_robot(neigh);

                neigh_node = graph->get_vertex_as<pmt_graph_node_t > (neigh);
                int empty_vertices = 0; // [0, 1 , 2] if we have empty vertices except w and neigh. 
                if( r != NULL )
                {
                    empty_vertices = 1;
                    r_node = graph->get_vertex_as<pmt_graph_node_t > (r);
                }

                if( clear_branch(solution, graph, assign, empty_vertices) )
                {
                    PRX_DEBUG_COLOR("FREE BRANCH : " << print_point(graph, w) << "  neigh : " << print_point(graph, neigh) << "  r : " << print_point(graph, r) << " q : " << print_point(graph, q), PRX_TEXT_BROWN);
                    return true;
                }
                else
                {
                    //                    //If the clear_branch could not clear 2 vertices, the two pebbles have to move and 
                    //                    //open the way to the pebble that is blocking the branch vertex. 
                    //
                    //                    if( empty_vertices != 0 )
                    //                    {
                    //                        //from the clear_branch the w and neigh will be obstacles.
                    //                        //and r will be in in the avoid list.
                    //                        //Because r is only avoidable pebble2 could go through it. We have to avoid that
                    //                        //when we are pushing pebble2 away from its position. 
                    //                        r_node->obstacle_id = obstacle_global_id;
                    //                        //            if( /push_to_empty(solution, graph, assign, neigh) )
                    //                        if( evacuate_vertex(solution, graph, assign, assign.get_robot(neigh)) )
                    //                        {
                    //                            //neigh and the new position of pebble2 has to be obstacles.    
                    //                            PRX_DEBUG_COLOR("Pebble2 : " << pebble2 << "  is now on : " << print_point(graph, assign.get_position(pebble2)), PRX_TEXT_GREEN);
                    //                            node = graph->get_vertex_as<pmt_graph_node_t > (assign.get_position(pebble2));
                    //                            node->obstacle_id = obstacle_global_id;
                    //                            neigh_node->obstacle_id = obstacle_global_id;
                    //
                    //                            ++avoid_global_id;
                    //                            r_node->avoid_id = avoid_global_id;
                    //                            r_node->obstacle_id = 0;
                    //                            w_node->avoid_id = avoid_global_id;
                    //                            w_node->obstacle_id = 0;
                    //
                    //                            step(solution, assign, pebble1, neigh);
                    //                            PRX_DEBUG_COLOR("Pebble1 : " << pebble1 << "  is now on : " << print_point(graph, assign.get_position(pebble1)), PRX_TEXT_GREEN);
                    //                            //Get a q that its neither avoidable nor obstacle
                    //                            q = get_valid_adjacent_vertex(graph, w);
                    //                            if( q != NULL )
                    //                            {
                    //                                int blocker_id = assign.get_robot(q);
                    //                                PRX_DEBUG_COLOR("The blocker is : " << blocker_id, PRX_TEXT_BROWN);
                    //                                PRX_ASSERT(blocker_id != -1);
                    //
                    //                                two_steps(solution, assign, blocker_id, w, r);
                    //                                PRX_DEBUG_COLOR("w : " << print_point(graph, w) << "   r: " << print_point(graph, r), PRX_TEXT_GREEN);
                    //
                    //                                step(solution, assign, pebble1, w);
                    //                                step(solution, assign, pebble2, neigh);
                    //
                    //                                PRX_DEBUG_COLOR("Pebble1 : " << pebble1 << "  is now on : " << print_point(graph, assign.get_position(pebble1)), PRX_TEXT_CYAN);
                    //                                PRX_DEBUG_COLOR("Pebble2 : " << pebble2 << "  is now on : " << print_point(graph, assign.get_position(pebble2)), PRX_TEXT_CYAN);
                    //                                //w, q and r, such as neigh from before, are now obstacles.
                    //                                //the previous position of the pebble2 is now open as obstacles and 
                    //                                //be selected. 
                    //                                graph->get_vertex_as<pmt_graph_node_t > (q)->obstacle_id = obstacle_global_id;
                    //                                node->obstacle_id = 0;
                    //                                node->avoid_id = 0;
                    //                                w_node->obstacle_id = obstacle_global_id;
                    //                                r_node->obstacle_id = obstacle_global_id;
                    //                                r_node->avoid_id = 0;
                    //
                    //                                //                    if( push_to_empty(solution, graph, assign, r) )
                    //                                if( evacuate_vertex(solution, graph, assign, assign.get_robot(r)) )
                    //                                    //                                if( clear_to_target(solution, graph, assign, assign.get_robot(r)) )
                    //                                {
                    //                                    //PRX_DEBUG_S("FREE BRANCH 2: ");
                    //                                    //PRX_DEBUG_S("w : " << print_point(graph, w));
                    //                                    //PRX_DEBUG_S("neigh : " << print_point(graph, neigh));
                    //                                    //PRX_DEBUG_S("r : " << print_point(graph, r));
                    //                                    //PRX_DEBUG_S("q : " << print_point(graph, q));
                    //                                    //PRX_DEBUG_S("------------- DONE checking branch " << print_point(graph, w) << " ----------------");
                    //                                    return true;
                    //                                }
                    //                            }
                    //
                    //                        }
                    //                    }
                }

                for( int i = 0; i < evacuate_len; ++i )
                {
                    graph->get_vertex_as<ppmt_graph_node_t > (to_evacuate[i]->get_next_step())->used_id = used_global_id-1;
                    to_evacuate[i]->done_swap();
                }
                return false;
            }

            bool ppmt_solver_t::check_visible_branches(pebble_solution_path_t& solution, undirected_graph_t* graph, pebble_assignment_t& assign, undirected_vertex_index_t v_start, bool caller_on_w)
            {
                //                ++bfs_global_id;
                //                ++avoid_global_id;
                //
                //                graph->predecessors[v_start] = v_start;
                //                node = graph->get_vertex_as<pmt_graph_node_t > (v_start);
                //                static_heap.restart();
                //                static_heap.push_back(v_start);
                //                bool is_branch_ok = true;
                //
                //                node->bfs_id = bfs_global_id;
                //                int revert_index = solution.size();
                //                pebble_assignment_t keep_assign = assign;
                //                undirected_vertex_index_t v_pebble2_start = neigh;
                //                int pebble1 = assign.get_robot(v_start);
                //                int pebble2 = assign.get_robot(neigh);
                //
                //                unsigned int curr_obstacle_id = obstacle_global_id;
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
                //                                    //                        if( caller_on_w )
                //                                    //                            //PRX_DEBUG_S("for CRITERION 4 : w " << print_point(graph, w));
                //                                    //                        else
                //                                    //                            //PRX_DEBUG_S("for CRITERION 5 : w " << print_point(graph, w));
                //                                    //                        //PRX_DEBUG_S(static_heap.size() << " )  w : " << print_point(graph, w) << "   from v_start : " << print_point(graph, v_start) << "   is from v side: " << caller_on_w);
                //                                    v_w_path_length = 0;
                //                                    for( undirected_vertex_index_t p = adj; p != v_start; p = graph->predecessors[p] )
                //                                    {
                //                                        v_w_path[v_w_path_length] = p;
                //                                        ++v_w_path_length;
                //                                    }
                //                                    v_w_path[v_w_path_length] = v_start;
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
                //                                            if( !evacuate_vertex(solution, graph, assign, assign.get_robot(v_w_path[i])) )
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
                //                                    if( is_branch_ok )
                //                                    {
                //                                        neigh = v_w_path[1];
                //                                        PRX_ASSERT(w == v_w_path[0]);
                //                                        //PRX_DEBUG_S("pebble " << pebble1 << "  is now " << print_point(graph, assign.get_position(pebble1)) << "   and pebble2 " << pebble2 << "  at point: " << print_point(graph, assign.get_position(pebble2)));
                //                                        if( check_branch(solution, graph, assign) )
                //                                        {
                //                                            if( caller_on_w )
                //                                            {
                //                                                //PRX_WARN_S("CRITERION 4 : w " << print_point(graph, w) << "  worked");
                //                                                step(solution, assign, pebble1, r);
                //                                                two_steps(solution, assign, pebble2, w, q);
                //                                                two_steps(solution, assign, pebble1, w, neigh);
                //                                                step(solution, assign, pebble2, w);
                //                                            }
                //                                            else
                //                                            {
                //                                                //PRX_WARN_S("CRITERION 5 : w " << print_point(graph, w) << "  worked");
                //                                                if( std::find(tmp_path.begin(), tmp_path.begin() + tmp_path_length, r) == tmp_path.begin() + tmp_path_length )
                //                                                {
                //                                                    step(solution, assign, pebble1, r);
                //                                                    two_steps(solution, assign, pebble2, w, q);
                //                                                }
                //                                                else
                //                                                {
                //                                                    step(solution, assign, pebble1, q);
                //                                                    two_steps(solution, assign, pebble2, w, r);
                //                                                }
                //                                                two_steps(solution, assign, pebble1, w, neigh);
                //                                                step(solution, assign, pebble2, w);
                //                                            }
                //                                            return true;
                //                                        }
                //                                        PRX_DEBUG_COLOR(" OH nooo branch " << print_point(graph, w) << "  Failed!", PRX_TEXT_MAGENTA);
                //                                    }
                //
                //                                    w_node->visited_id = visited_global_id;
                //                                    solution.revert(revert_index);
                //                                    assign = keep_assign;
                //                                }
                //                                else
                //                                    static_heap.push_back(adj);
                //                            }
                //
                //                            w_node->bfs_id = bfs_global_id;
                //                        }
                //                    }
                //                }
                //                w = NULL;
                //                w_node = NULL;
                return false;

            }

            bool ppmt_solver_t::clear_branch(pebble_solution_path_t& solution, undirected_graph_t* graph, pebble_assignment_t& assign, int& empty_vertices)
            {
                ++obstacle_global_id;

                foreach(undirected_vertex_index_t adj, boost::adjacent_vertices(w, graph->graph))
                {
                    if( adj != neigh && adj != r )
                    {
                        if( !graph->get_vertex_as<ppmt_graph_node_t > (adj)->is_vertex_blocked(used_global_id,obstacle_global_id) && !assign.has_robot_on(adj) )
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

                w_node->obstacle_id = obstacle_global_id;
                neigh_node->obstacle_id = obstacle_global_id;
                if( r != NULL )
                    r_node->avoid_id = avoid_global_id;

                //If we reach here the adj vertex will definitely be a occupied vertex. The only un-occupied
                //vertex will be the r. If there were more than 2 free vertices the algorithm will have stopped 
                //earlier. Thats why we are checking immediately if adj can be cleaned by the robot.

                ppmt_graph_node_t* node;
                foreach(undirected_vertex_index_t adj, boost::adjacent_vertices(w, graph->graph))
                {
                    if( adj != neigh && adj != r )
                    {
                        node = graph->get_vertex_as<ppmt_graph_node_t > (adj);
                        if( !node->is_vertex_blocked(used_global_id,obstacle_global_id) && evacuate_vertex(graph, assign, assign.get_robot(adj)) == MA_MOVE )
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

            std::pair<int, int> ppmt_solver_t::find_blocker_on_path(ppmt_agent_t* agent, pebble_assignment_t& assign)
            {
                int blocker_id = -1;
                for( int p = agent->path_start + 1; p < agent->path_length; ++p )
                {
                    blocker_id = assign.get_robot(agent->tmp_path[p]);
                    if( blocker_id != -1 )
                    {
                        return std::make_pair(blocker_id, p);
                    }
                }

                return std::make_pair(blocker_id, 0);
            }

            std::pair<bool, int> ppmt_solver_t::find_sort_path(std::deque<util::undirected_vertex_index_t>& path, util::undirected_graph_t* graph, pebble_assignment_t& assign, util::undirected_vertex_index_t v_start, util::undirected_vertex_index_t v_target, int path_length, bool avoid_pebbles, bool avoid_used_v)
            {
                pmt_heuristic->set(v_start, &assign);
                pmt_visitor->set_goal(v_start);
                ppmt_graph_node_t* node;
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
                        node = graph->get_vertex_as<ppmt_graph_node_t > (v);
                        path[path_length] = v;
                        ++path_length;
                        if( (avoid_pebbles && assign.has_robot_on(v)) || (avoid_used_v && node->is_vertex_blocked(used_global_id,obstacle_global_id)) )
                            return std::make_pair(false, path_length);
                    }
                    node = graph->get_vertex_as<ppmt_graph_node_t > (v_target);
                    path[path_length] = v_target;
                    ++path_length;
                    if( (avoid_pebbles && assign.has_robot_on(v_target)) || (avoid_used_v && node->is_vertex_blocked(used_global_id,obstacle_global_id)) )
                        return std::make_pair(false, path_length);
                    return std::make_pair(true, path_length);
                }

                return std::make_pair(false, path_length);
            }

            ppmt_solver_t::moving_action_t ppmt_solver_t::evacuate_vertex(util::undirected_graph_t* graph, pebble_assignment_t& assign, int pebble_id)
            {
                if( pebble_id == -1 )
                    return MA_MOVE;

                moving_action_t ret = MA_WAIT;
                ppmt_agent_t* agent = &agents_list[pebble_id];

                if( !agent->is_free(busy_global_id) )
                    return MA_WAIT;

                int blocker_id = -1;
                ppmt_agent_t* blocker;
                undirected_vertex_index_t v;

                PRX_ASSERT(agent->v_curr == assign.get_position(pebble_id));

                if( agent->has_path() )
                {
                    v = agent->get_next_step();
                    blocker_id = assign.get_robot(v);


                    //TODO: This will not work on the graphs. If this vertex will be the r vertex on a graph
                    //      it can be connected with the q vertex. If the blocker is moving towards the q vertex
                    //      before the q vertex is identified it will be a problem. 
                    if( !graph->get_vertex_as<ppmt_graph_node_t > (v)->is_vertex_blocked(used_global_id,obstacle_global_id) )
                    {
                        if( evacuate_vertex(graph, assign, blocker_id) == MA_MOVE )
                        {
                            PRX_DEBUG_COLOR("Push to target : " << pebble_id << "  moved to : " << print_point(graph, v), PRX_TEXT_BLUE);
                            agent->start_evacuate();
                            graph->get_vertex_as<ppmt_graph_node_t > (v)->used_id = used_global_id;
                            to_evacuate[evacuate_len++] = agent;
                            return MA_MOVE;
                        }
                    }
                }

                moving_action_t action = evacuate_to_empty(graph, assign, agent);

                if( action == MA_MOVE || action == MA_WAIT )
                    return action;
                return ret;
            }

            ppmt_solver_t::moving_action_t ppmt_solver_t::evacuate_to_empty(util::undirected_graph_t* graph, pebble_assignment_t& assign, ppmt_agent_t* agent)
            {
                moving_action_t ret = MA_WAIT;
                ppmt_graph_node_t* node;

                foreach(undirected_vertex_index_t adj, boost::adjacent_vertices(agent->v_curr, graph->graph))
                {
                    node = graph->get_vertex_as<ppmt_graph_node_t > (adj);
                    if( node->is_vertex_blocked(used_global_id,obstacle_global_id) )
                    {
                        ret = MA_WAIT;
                    }
                    else
                    {
                        int blocker_id = assign.get_robot(adj);

                        //If blocker_id == -1 the evacuate_vertex will return MA_MOVE either way. So we don't need to check
                        //the case where blocker_id == -1.
                        moving_action_t action = evacuate_vertex(graph, assign, blocker_id);
                        if( action == MA_MOVE )
                        {
                            PRX_DEBUG_COLOR("Push to empty : " << agent->id << "  moved to : " << print_point(graph, adj), PRX_TEXT_CYAN);
                            agent->start_evacuate(adj);
                            graph->get_vertex_as<ppmt_graph_node_t > (adj)->used_id = used_global_id;
                            to_evacuate[evacuate_len++] = agent;
                            return MA_MOVE;
                        }
                        else if( action == MA_WAIT )
                        {
                            ret = action;
                        }

                    }
                }
                return ret;
            }

            ppmt_solver_t::moving_action_t ppmt_solver_t::step_agent(pebble_solution_path_t& solution, undirected_graph_t* graph, pebble_assignment_t& assign, ppmt_agent_t* agent)
            {
                undirected_vertex_index_t v_curr = agent->get_current_step();
                undirected_vertex_index_t v_step = agent->get_next_step();
                PRX_DEBUG_COLOR("Agent : << " << agent->id << "  will try to move TO: " << print_point(graph, v_step), PRX_TEXT_CYAN);

                if( can_agent_move(graph, assign, agent) )
                {
                    step(solution, assign, agent->id, v_step);
                    return MA_MOVE;
                }
                PRX_DEBUG_COLOR("Agent : " << agent->id << "  will wait at : " << print_point(graph, agent->v_curr), PRX_TEXT_CYAN);
                return MA_WAIT;
            }

            void ppmt_solver_t::step(pebble_solution_path_t& solution, pebble_assignment_t& assign, int pebble_id, undirected_vertex_index_t v_step)
            {
                //                PRX_DEBUG_COLOR("priority_cycle_detected: " << priority_cycle_detected, PRX_TEXT_GREEN);
                //                if( priority_cycle_detected )
                //                {
                //                    PRX_DEBUG_COLOR("pebble " << pebble_id << /*"  from point: " << print_point(initial_graph, assign.get_position(pebble_id)) <<*/ " -> " << print_point(initial_graph, v_step), PRX_TEXT_LIGHTGRAY);
                //                }
                //                else
                //                {
                //                    PRX_DEBUG_COLOR("pebble " << pebble_id << "  from point: " << print_point(initial_graph, assign.get_position(pebble_id)) << " -> " << print_point(initial_graph, v_step), PRX_TEXT_LIGHTGRAY);
                //                }

                PRX_DEBUG_COLOR("pebble " << pebble_id << " -> " << print_point(initial_graph, v_step), PRX_TEXT_LIGHTGRAY);
                undirected_vertex_index_t v_curr = assign.get_position(pebble_id);
                if( v_curr != v_step )
                {
                    solution.push_back(pebble_id, v_curr, v_step);
                    assign.change_position(pebble_id, v_step);
                    agents_list[pebble_id].move();
                    agents_list[pebble_id].busy = busy_global_id;
                    PRX_ASSERT(v_step == agents_list[pebble_id].v_curr);
                    PRX_ASSERT(v_step == assign.get_position(pebble_id));
                }
            }

            bool ppmt_solver_t::can_agent_move(util::undirected_graph_t* graph, pebble_assignment_t& assign, ppmt_agent_t* agent, ppmt_agent_t* first_agent)
            {
                if( agent != first_agent )
                {
                    //////////////////////////////////////////////////////////////////////////////////////////
                    PRX_DEBUG_COLOR("can agent move : " << agent->id, PRX_TEXT_CYAN);
                    if( first_agent != NULL )
                        PRX_DEBUG_COLOR("With First agent  : " << first_agent->id, PRX_TEXT_CYAN);
                    //////////////////////////////////////////////////////////////////////////////////////////

                    if( first_agent == NULL )
                        first_agent = agent;

                    undirected_vertex_index_t v;
                    if( agent->is_busy(busy_global_id) ) //This will happen if a push is called back after a swap.  
                        return false;

                    if( agent->has_path() )
                        v = agent->get_next_step();
                    else
                        return false;

                    if( graph->get_vertex_as<ppmt_graph_node_t > (v)->is_vertex_blocked(used_global_id,obstacle_global_id) )
                        return false;

                    int blocker_id = assign.get_robot(v);
                    if( blocker_id != -1 )
                    {
                        return can_agent_move(graph, assign, &agents_list[blocker_id], first_agent);
                    }
                }
                return true;
            }
        }
    }
}

