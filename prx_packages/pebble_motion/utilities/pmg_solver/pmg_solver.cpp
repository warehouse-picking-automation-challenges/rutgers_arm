/**
 * @file pmg_solver.cpp
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

#include "utilities/pmg_solver/pmg_solver.hpp"
#include "utilities/pebble_spanning_tree.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"
#include "utilities/kornhauser/kornhauser_graph.hpp"
#include "utilities/kornhauser/kornhauser_tester.hpp"

#include <boost/tuple/tuple.hpp> // boost::tie

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
 
PLUGINLIB_EXPORT_CLASS( prx::packages::pebble_motion::pmg_solver_t, prx::packages::pebble_motion::pebble_solver_t)

namespace prx
{
    using namespace util;

    namespace packages
    {
        namespace pebble_motion
        {

            pmg_solver_t::pmg_solver_t()
            {
//                pmt_solver_t::pmt_solver_t();                
            }

            pmg_solver_t::~pmg_solver_t() { }

            bool pmg_solver_t::find_solution(std::vector<pebble_step_t>* solution, undirected_graph_t* graph)
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
                        PRX_ERROR_S("-------------------    START ALL THE PEBBLES  --------------------");
                        print_assignment(graph, s_new);
                        for( iter = 0; iter < k; ++iter )
                        {
                            pebble_id = order[iter];
                            PRX_ERROR_S("-------------------  PEBBLE " << pebble_id << "  --------------------");
                            if( !move_to_goal(solution_path, graph, s_new, pebble_id, s_new.get_position(pebble_id), t_assignment.get_position(pebble_id)) )
                            {
                                break;
                            }
                            //                PRX_LOG_ERROR("Pebble %d couldn't reach its goal!");
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

            bool pmg_solver_t::move_to_goal(pebble_solution_path_t& solution, undirected_graph_t* graph, pebble_assignment_t& assign, int pebble_id, undirected_vertex_index_t v_start, undirected_vertex_index_t v_goal)
            {
                if( v_start == v_goal )
                    return true;

                bool path_exists;
                int blocker_id;

                ++avoid_global_id;
                ++obstacle_global_id;
                agent_paths_index = -1;

                PRX_DEBUG_COLOR("------------    assign in move to goal    ------------", PRX_TEXT_MAGENTA);
                print_assignment(graph, assign, PRX_TEXT_MAGENTA);

                //Checks first for a path that will not move the pebble that are on their goals.
                boost::tie(path_exists, tmp_path_length) = find_path(tmp_path, graph, assign, v_start, v_goal, 0, true, false);

                if( !path_exists )
                    PRX_LOG_ERROR("Path between start and goal of the pebble %d does not exist.", pebble_id);

                PRX_DEBUG_S("Going to move pebble " << pebble_id << "  from :" << print_point(graph, v_start) << " -> " << print_point(graph, v_goal) << "  p_len : " << tmp_path_length);

                int tmp_begin = 0;
                graph->get_vertex_as<pmt_graph_node_t > (tmp_path[tmp_begin])->obstacle_id = obstacle_global_id;
                for( int p = 1; p < tmp_path_length; ++p )
                {
                    blocker_id = assign.get_robot(tmp_path[p]);
                    if( blocker_id != -1 )
                    {
                        if( need_swap(graph, tmp_path, tmp_begin, tmp_path_length, tmp_path[p], t_assignment.get_position(blocker_id)) )
                        {
                            PRX_DEBUG_S("They need swap");
                            if( swap(solution, graph, assign, pebble_id, blocker_id, tmp_path, tmp_begin, p) )
                                return false;
                            PRX_LOG_ERROR("SWAP between %d with %d FAILED", pebble_id, blocker_id);
                        }
                        else
                        {
                            pebble_assignment_t keep_assign;
                            int reverse_index = -1;
                            int reverse_begin;
                            if( tmp_begin + 1 <= p - 1 )
                            {
                                keep_assign = assign;
                                reverse_index = solution.size();
                                solution.push_back(pebble_id, tmp_path, tmp_begin, p - 1);
                                assign.change_position(pebble_id, tmp_path[p - 1]);
                                PRX_DEBUG_COLOR("Pebble " << pebble_id << "  moved in the position : " << print_point(graph, tmp_path[p - 1]) << "   p:" << p, PRX_TEXT_GREEN);

                                reverse_begin = tmp_begin;
                                tmp_begin = p - 1;
                                for( int a = reverse_begin; a < tmp_begin; ++a )
                                {
                                    graph->get_vertex_as<pmt_graph_node_t > (tmp_path[a])->avoid_id = 0;
                                    graph->get_vertex_as<pmt_graph_node_t > (tmp_path[a])->obstacle_id = 0;
                                }
                                graph->get_vertex_as<pmt_graph_node_t > (tmp_path[tmp_begin])->obstacle_id = obstacle_global_id;
                            }
                            already_swapped = false;
                            if( !clear_vertex(solution, graph, assign, blocker_id) )
                            {
                                if( !already_swapped )
                                {
                                    if( reverse_index != -1 )
                                    {
                                        tmp_begin = reverse_begin;
                                        solution.revert(reverse_index);
                                        assign = keep_assign;

                                        //we don't care to fix the obstacles/avoidable because swap will restart the obstacles.
                                    }
                                    if( !swap(solution, graph, assign, pebble_id, blocker_id, tmp_path, tmp_begin, p) )
                                        PRX_LOG_ERROR("PUSH AND SWAP between %d with %d FAILED", pebble_id, blocker_id);
                                }
                                return false;
                            }
                        }
                    }
                }

                PRX_DEBUG_S("tmp_begin : " << tmp_begin << "  len:" << tmp_path_length);
                //PRX_DEBUG_S("Without any push or swap moving pebble " << pebble_id << "  from :" << print_point(graph, tmp_path[tmp_begin]) << " -> " << print_point(graph, tmp_path[tmp_path_length - 1]));
                if( tmp_begin + 1 <= tmp_path_length - 1 )
                {
                    solution.push_back(pebble_id, tmp_path, tmp_begin, tmp_path_length - 1);
                    assign.change_position(pebble_id, tmp_path[tmp_path_length - 1]);
                }
                print_assignment(graph, assign, PRX_TEXT_GREEN);
                return true;
            }
        }
    }
}
