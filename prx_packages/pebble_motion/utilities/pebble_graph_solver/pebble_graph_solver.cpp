
/**
 * @file pebble_graph_solver.cpp
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

#include "utilities/pebble_graph_solver/pebble_graph_solver.hpp"
#include "utilities/graph_feasibility_tester/graph_pebble_graph.hpp"
#include "utilities/kornhauser/kornhauser_graph.hpp"


#include <boost/graph/biconnected_components.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::packages::pebble_motion::pebble_graph_solver_t, prx::packages::pebble_motion::pebble_solver_t)


namespace prx
{
    using namespace util;

    namespace packages
    {
        namespace pebble_motion
        {

            pebble_graph_solver_t::pebble_graph_solver_t()
            {
                tmp_vector_index = 0;
                on_path_global = 0;
                on_mcbc_global = 0;
                last_solution_step = 0;
                last_solution_check = 0;
                connecting_path_index = 0;
            }

            pebble_graph_solver_t::~pebble_graph_solver_t() { }

            undirected_vertex_index_t pebble_graph_solver_t::add_new_vertex(undirected_graph_t* graph)
            {
                return graph->add_vertex<graph_pebble_node_t > ();
            }

            bool pebble_graph_solver_t::reduce(std::vector<pebble_step_t>* solution, undirected_graph_t* graph, pebble_assignment_t& s_new, pebble_assignment_t& s_assign, pebble_assignment_t& t_assign)
            {
                obstacle_global_id++;
                allocated_heap_t<undirected_vertex_index_t> leaves(number_of_vertices);
                undirected_vertex_index_t v_graph; //vertices on the initial graph
                undirected_vertex_index_t v_sptree, par_sptree; //vertices on the spanning tree
                pebble_assignment_t curr_assign; //On the initial graph

                initial_graph = graph;

                curr_assign = s_assign;
                pebble_spanning_tree_t spanning_tree;

                //    PRX_WARN_S("----------  INITIAL GRAPH -----------------");

                //    foreach(undirected_vertex_index_t v, boost::vertices(graph->graph))
                //    {
                //        PRX_DEBUG_S(v << " : " << print_point(graph, v));
                //    }

                leaves.restart();
                spanning_tree.compute_spanning_tree(graph);
                spanning_tree.get_leaves(leaves);

                tree_pebble_node_t* curr_v; //will be used on the reduced tree

                //    print_assignment(graph, curr_assign);

                while( !leaves.empty() )
                {
                    //        foreach(undirected_vertex_index_t v_l, leaves)
                    //        {
                    //            undirected_vertex_index_t vv = spanning_tree.get_graph_index(v_l);
                    //            PRX_INFO_S("leaf ( " << vv << ") : " << print_point(graph, vv));
                    //        }
                    v_sptree = leaves.pop_front();

                    v_graph = spanning_tree.get_graph_index(v_sptree);

                    par_sptree = spanning_tree.get_parent(v_sptree);

                    curr_v = graph->get_vertex_as<graph_pebble_node_t > (v_graph);
                    //        PRX_INFO_S("-------------------GRAPH----------------   leaf : " << v_graph << " : " << print_point(graph, curr_v->index));


                    if( t_assign.is_empty(v_graph) )
                    {
                        if( !curr_assign.is_empty(v_graph) )
                        {
                            if( !push_pebble_once(solution, graph, v_graph, curr_assign) )
                                return false;
                        }
                    }
                    else
                    {
                        if( curr_assign.is_empty(v_graph) )
                        {
                            //                print_assignment(graph, curr_assign);
                            int pebble_id = find_closest_robot(graph, v_graph, curr_assign);

                            //                PRX_ERROR_S("pebble : " << pebble_id);
                            if( pebble_id == -1 )
                                return false;
                            undirected_vertex_index_t v_del = curr_assign.get_position(pebble_id);

                            //                PRX_DEBUG_S("Case 2 move the robot " << pebble_id << "   in position " << print_point(graph, v_graph));
                            //
                            //                PRX_DEBUG_S("v_graph: " << print_point(graph, v_graph) << "   with pred : " << print_point(graph, graph->predecessors[v_graph]));
                            //                PRX_DEBUG_S("v_del: " << print_point(graph, v_del) << "   with pred : " << print_point(graph, graph->predecessors[v_del]));

                            tmp_vector_index = 0;
                            for( undirected_vertex_index_t v = v_del; v != v_graph; v = graph->predecessors[v] )
                            {

                                tmp_vector[tmp_vector_index] = v;
                                ++tmp_vector_index;
                                //                    PRX_DEBUG_S("v: " << print_point(graph, v) << "   with pred : " << print_point(graph, graph->predecessors[v]));
                                //                    robot_path.push_back(v);
                            }
                            tmp_vector[tmp_vector_index] = v_graph;
                            ++tmp_vector_index;
                            //                robot_path.push_back(v_graph);
                            std::vector<undirected_vertex_index_t> robot_path;
                            robot_path.assign(tmp_vector.begin(), tmp_vector.begin() + tmp_vector_index);
                            solution->push_back(std::make_pair(pebble_id, robot_path));
                            curr_assign.remove_assignment(v_del);
                            //                PRX_ERROR_S("add " << print_point(graph, v_graph));
                            s_new.add_assignment(v_graph, pebble_id);
                            robot_path.clear();
                        }
                        else
                        {
                            //                PRX_ERROR_S("Case 3 set the robot " << curr_assign.get_robot(v_graph) << "   in position " << print_point(graph, v_graph));
                            s_new.add_assignment(v_graph, curr_assign.get_robot(v_graph));
                            curr_assign.remove_assignment(v_graph);
                        }

                    }
                    graph->get_vertex_as<graph_pebble_node_t > (v_graph)->obstacle_id = obstacle_global_id;
                    spanning_tree.remove_vertex(v_sptree);

                    if( par_sptree != v_sptree && !spanning_tree.has_children(par_sptree) )
                        if( !leaves.has(par_sptree) )
                            leaves.push_back(par_sptree);
                }

                //    PRX_WARN_S("------------------------------------");
                //    PRX_WARN_S("------------    curr    ------------");
                //    print_assignment(graph, curr_assign);
                //    PRX_WARN_S("------------    strt    ------------");
                //    print_assignment(graph, s_assign);
                //    PRX_WARN_S("------------    s_new    ------------");
                //    print_assignment(graph, s_new);
                //    PRX_WARN_S("-----------     targ    ------------");
                //    print_assignment(graph, t_assign);
                //    PRX_WARN_S("------------------------------------");
                return true;
            }


            //bool pebble_graph_solver_t::reduce(std::vector<pebble_step_t>* solution, undirected_graph_t* graph, pebble_assignment_t& s_new, pebble_assignment_t& s_assign, pebble_assignment_t& t_assign)
            //{
            //    int graph_size = boost::num_vertices(graph->graph);
            //    tmp_vector.resize(graph_size);
            //    mcbc.resize(graph_size);
            //    static_heap.resize(graph_size);
            //    allocated_heap_t<undirected_vertex_index_t> leaves_heap(graph_size);
            //    obstacle_global_id++;
            //    std::deque<undirected_vertex_index_t> leaves;
            //    undirected_vertex_index_t v_graph; //vertices on the initial graph
            //    undirected_vertex_index_t v_sptree, par_sptree; //vertices on the spanning tree
            //    pebble_assignment_t curr_assign; //On the initial graph
            ////    std::vector<pebble_step_t> reduction_solution(k);
            ////
            ////
            ////    int reduction_solution_index = 0;
            ////    for( int i = 0; i < k; ++i )
            ////        reduction_solution[i].second.resize(2);
            //
            //    initial_graph = graph;
            //
            //    curr_assign = s_assign;
            //    pebble_spanning_tree_t spanning_tree;
            //
            //    //    PRX_WARN_S("----------  INITIAL GRAPH -----------------");
            //
            //    //    foreach(undirected_vertex_index_t v, boost::vertices(graph->graph))
            //    //    {
            //    //        PRX_DEBUG_S(v << " : " << print_point(graph, v));
            //    //    }
            //
            //    spanning_tree.compute_spanning_tree(graph);
            //    spanning_tree.get_leaves(leaves);
            //    //    PRX_ERROR_S("leaves size : " << leaves.size());
            //    for( unsigned int i = 0; i < leaves.size(); ++i )
            //    {
            //        leaves_heap.push_back(leaves[i]);
            //    }
            //
            //    tree_pebble_node_t* curr_v; //will be used on the spanning tree
            //
            //
            //    tmp_vector_index = 0;
            //    while( !leaves_heap.empty() )
            //    {
            //        //        foreach(undirected_vertex_index_t v_l, leaves)
            //        //        {
            //        //            undirected_vertex_index_t vv = spanning_tree.get_graph_index(v_l);
            //        //            PRX_INFO_S("leaf ( " << vv << ") : " << print_point(graph, vv));
            //        //        }
            //        v_sptree = leaves_heap.pop_front();
            //
            //        v_graph = spanning_tree.get_graph_index(v_sptree);
            //
            //        par_sptree = spanning_tree.get_parent(v_sptree);
            //
            //        curr_v = graph->get_vertex_as<graph_pebble_node_t > (v_graph);
            //        //        PRX_INFO_S("-------------------GRAPH----------------   leaf : " << v_graph << " : " << print_point(graph, curr_v->index));
            //
            //        if( t_assign.is_empty(v_graph) )
            //        {
            //            if( !curr_assign.is_empty(v_graph) )
            //            {
            //                if( !push_pebble_once_no_path(graph, v_graph, curr_assign) )
            //                    return false;
            //            }
            //        }
            //        else
            //        {
            //            if( curr_assign.is_empty(v_graph) )
            //            {
            //                //                print_assignment(graph, curr_assign);
            //                int pebble_id = find_closest_robot(graph, v_graph, curr_assign);
            //
            //                //                PRX_ERROR_S("pebble : " << pebble_id);
            //                if( pebble_id == -1 )
            //                    return false;
            //                undirected_vertex_index_t v_del = curr_assign.get_position(pebble_id);
            //
            //                //                PRX_DEBUG_S("Case 2 move the robot " << pebble_id << "   in position " << print_point(graph, v_graph));
            //                //
            //                //                PRX_DEBUG_S("v_graph: " << print_point(graph, v_graph) << "   with pred : " << print_point(graph, graph->predecessors[v_graph]));
            //                //                PRX_DEBUG_S("v_del: " << print_point(graph, v_del) << "   with pred : " << print_point(graph, graph->predecessors[v_del]));
            //                //                std::vector<undirected_vertex_index_t> robot_path;
            //                //                for( undirected_vertex_index_t v = v_del; v != v_graph; v = graph->predecessors[v] )
            //                //                {
            //                //                    robot_path.push_back(v);
            //                //                }
            //                //                robot_path.push_back(v_graph);
            //                //                solution->push_back(std::make_pair(pebble_id, robot_path));
            //                curr_assign.remove_assignment(v_del);
            //                s_new.add_assignment(v_graph, pebble_id);
            //                //                robot_path.clear();           
            //                reduction_solution[reduction_solution_index].first = pebble_id;
            //                reduction_solution[reduction_solution_index].second[0] = s_assign.get_position(pebble_id);
            //                reduction_solution[reduction_solution_index].second[1] = v_graph;
            //                ++reduction_solution_index;
            //            }
            //            else
            //            {
            //                //                PRX_ERROR_S("Case 3 set the robot " << curr_assign.get_robot(v_graph) << "   in position " << print_point(graph, v_graph));
            //                int pebble_id = curr_assign.get_robot(v_graph);
            //                s_new.add_assignment(v_graph, pebble_id);
            //                curr_assign.remove_assignment(v_graph);
            //                reduction_solution[reduction_solution_index].first = pebble_id;
            //                reduction_solution[reduction_solution_index].second[0] = s_assign.get_position(pebble_id);
            //                reduction_solution[reduction_solution_index].second[1] = v_graph;
            //                ++reduction_solution_index;
            //            }
            //
            //        }
            //        graph->get_vertex_as<graph_pebble_node_t > (v_graph)->obstacle_id = obstacle_global_id;
            //        spanning_tree.remove_vertex(v_sptree);
            //
            //        //        PRX_DEBUG_S("Before : \n" << leaves_heap.print());
            //        if( par_sptree != v_sptree && !spanning_tree.has_children(par_sptree) )
            //            if( !leaves_heap.has(par_sptree) )
            //                leaves_heap.push_back(par_sptree);
            //
            //        //        PRX_INFO_S("After : \n" << leaves_heap.print());
            //    }
            //
            //    //    PRX_DEBUG_S("The size of pebbles are : " << k << "   and the reduction_index: " << reduction_solution_index);
            //    //    for(int i = k-1; i>=0; --i )
            //    for( int i = 0; i < k; ++i )
            //    {
            //        //        PRX_DEBUG_S("pebble " << reduction_solution[i].first << "   from " << print_point(graph,reduction_solution[i].second[0]) << " -> " << print_point(graph,reduction_solution[i].second[1]));
            //        if( reduction_solution[i].second[0] != reduction_solution[i].second[1] )
            //        {
            //            tmp_vector_index = 0;
            //            if( !free_path(graph, reduction_solution[i].second[0], reduction_solution[i].second[1]) )
            //                return false;
            //            std::vector<undirected_vertex_index_t> pebble_path;
            //            pebble_path.push_back(reduction_solution[i].second[0]);
            //            pebble_path.insert(pebble_path.end(), tmp_vector.begin(), tmp_vector.begin() + tmp_vector_index);
            //            //            PRX_DEBUG_S("pebble " << reduction_solution[i].first << "   path: " << pebble_path.size() << "     sol; " << tmp_vector_index);
            //            solution->push_back(std::make_pair(reduction_solution[i].first, pebble_path));
            //        }
            //    }
            //
            //    //        PRX_WARN_S("------------------------------------");
            //    //        PRX_WARN_S("------------    curr    ------------");
            //    //        print_assignment(graph, curr_assign);
            //    //        PRX_WARN_S("------------    strt    ------------");
            //    //        print_assignment(graph, s_assign);
            //    //        PRX_WARN_S("------------    s_new    ------------");
            //    //        print_assignment(graph, s_new);
            //    //        PRX_WARN_S("-----------     targ    ------------");
            //    //        print_assignment(graph, t_assign);
            //    //        PRX_WARN_S("------------------------------------");
            //    return true;
            //}

            void pebble_graph_solver_t::update_node_info(undirected_graph_t* graph, undirected_vertex_index_t v, undirected_vertex_index_t u, pebble_assignment_t& assign)
            {
                graph_pebble_node_t* curr_v = graph->get_vertex_as<graph_pebble_node_t > (v);
                graph_pebble_node_t* curr_u = graph->get_vertex_as<graph_pebble_node_t > (u);
                bool u_occupied = assign.has_robot_on(u);

                //    PRX_WARN_S("update info for vertex : " << v << "   point:" << print_point(graph, curr_v->index));

                curr_v->num_trees = boost::degree(v, graph->graph);

                if( boost::degree(v, graph->graph) >= 3 )
                {
                    curr_v->trees[u].swap_dist = 0;
                    curr_v->trees[u].v_swap = v;
                }

                if( !u_occupied && !curr_u->transshipment )
                    curr_v->trees[u].holes++;

                if( u_occupied || boost::degree(u, graph->graph) > 2 )
                    curr_v->tree_for_vertex[u] = u;

                curr_v->seen.push_back(u);
                curr_v->trees[u].tree_seen.push_back(u);

                curr_v->trees[u].vertices_in_tree.push_back(u);

                foreach(undirected_vertex_index_t t, curr_u->trees | boost::adaptors::map_keys)
                {
                    if( t != v )
                    {
                        curr_v->trees[u].holes += curr_u->trees[t].holes;
                        if( curr_v->trees[u].swap_dist > curr_u->trees[t].swap_dist )
                        {
                            curr_v->trees[u].v_swap = curr_u->trees[t].v_swap;
                            curr_v->trees[u].swap_dist = curr_u->trees[t].swap_dist + graph->weights[boost::edge(v, u, graph->graph).first];
                        }

                        foreach(undirected_vertex_index_t w, curr_u->trees[t].vertices_in_tree)
                        {
                            curr_v->trees[u].vertices_in_tree.push_back(w);
                            if( graph->get_vertex_as<tree_pebble_node_t > (w)->occupied || boost::degree(w, graph->graph) > 2 )
                            {
                                curr_v->tree_for_vertex[w] = u;
                            }
                        }

                        if( !u_occupied )
                        {

                            foreach(undirected_vertex_index_t w, curr_u->trees[t].tree_seen)
                            {
                                curr_v->seen.push_back(w);
                                curr_v->trees[u].tree_seen.push_back(w);
                            }
                        }
                    }
                }
                if( curr_v->trees[u].holes > 0 )
                {
                    curr_v->num_free_trees++;
                    curr_v->free_trees.push_back(u);
                }

                PRX_ASSERT(curr_v->num_free_trees <= curr_v->num_trees);
                curr_v->checked_trees++;
                //    PRX_DEBUG_S("info at the end : " << curr_v->print_info());
            }

            bool pebble_graph_solver_t::find_solution(std::vector<pebble_step_t>* solution, undirected_graph_t* graph)
            {

                undirected_graph_t* tree = new undirected_graph_t();

                s_initial = s_assignment;
                reduce_to_tree(tree, graph);
                number_of_vertices = boost::num_vertices(tree->graph);
                number_of_edges = boost::num_edges(tree->graph);
                mcbc.resize(number_of_vertices);
                static_heap.resize(number_of_vertices);
                tmp_vector.resize(number_of_vertices);
                mcbc_blockers.resize(number_of_vertices);
                connecting_path.resize(number_of_vertices);

                pebble_assignment_t keep_assign = s_new;
                PRX_DEBUG_S("About to call tree find solution");
                if( pebble_tree_solver_t::find_solution(&tmp_solution, tree) )
                {
                    PRX_WARN_S("solution validation after find_solution");
                    PRX_INFO_S("validate all the solution : " << all_solution_validation(tree, &tmp_solution, keep_assign));
                    //        PRX_INFO_S("Convert the path");
                    convert_path(solution, graph, tree, &tmp_solution);
                    //        for( size_t i = 0; i < solution->size(); ++i )
                    //        {
                    //            PRX_ERROR_S("agent : " << solution->at(i).first);
                    //
                    //            foreach(undirected_vertex_index_t vert, solution->at(i).second)
                    //            {
                    //                PRX_DEBUG_S(print_point(graph, vert));
                    //            }
                    //        }
                    //
                    //        PRX_ERROR_S("---------  GRAPH ------------");
                    //
                    //        foreach(undirected_vertex_index_t v, boost::vertices(graph->graph))
                    //        {
                    //            PRX_DEBUG_S(print_point(graph, v));
                    //        }
                    //        PRX_ERROR_S("---------  G ------------");
                    //
                    //        foreach(undirected_vertex_index_t v, boost::vertices(tree->graph))
                    //        {
                    //            PRX_DEBUG_S(print_point(graph, v));
                    //        }        
                    return true;
                }
                PRX_ERROR_S("NO SOLUTION???");
                //    convert_path(solution, graph, tree, &tmp_solution);
                return false;
            }

            void pebble_graph_solver_t::reduce_to_tree(undirected_graph_t* tree, const undirected_graph_t * graph)
            {
                hash_t<int, undirected_vertex_index_t> new_nodes;
                undirected_vertex_index_t v_tree_s;
                undirected_vertex_index_t v_tree_t;
                undirected_vertex_index_t v_transshipment;
                undirected_vertex_index_t v_graph_s;
                undirected_vertex_index_t v_graph_t;
                undirected_edge_index_t e_new;
                double robot_space_dimension = state_space->get_dimension();
                int robot_id;
                unsigned int num_comps = boost::biconnected_components(graph->graph, graph->edge_components);
                biconnected_comps.resize(num_comps);

                foreach(undirected_edge_index_t e, boost::edges(graph->graph))
                {
                    int index = graph->edge_components[e];
                    //        PRX_DEBUG_S(e << "  has component : " << index);
                    biconnected_comps[index].b_comps_size++;
                    biconnected_comps[index].edges.push_back(e);
                }

                //    for( unsigned int i = 0; i < num_comps; ++i )
                //        PRX_INFO_S("component[" << i << "] :" << biconnected_comps[i].b_comps_size);
                //
                //    PRX_DEBUG_S("BEFORE)     v:" << boost::num_vertices(graph->graph) << "    e:" << boost::num_edges(graph->graph));
                //
                //    PRX_WARN_S("--------  BEFORE TO TREE -----------");
                //    PRX_WARN_S("------------    strt    ------------");
                //    print_assignment(graph, s_assignment);
                //    PRX_WARN_S("------------    s_new    ------------");
                //    print_assignment(graph, s_new);
                //    PRX_WARN_S("-----------     targ    ------------");
                //    print_assignment(graph, t_assignment);
                //    PRX_WARN_S("------------------------------------");

                std::vector<double> vec_point(robot_space_dimension);

                foreach(undirected_edge_index_t e, boost::edges(graph->graph))
                {
                    int edge_component_index = graph->edge_components[e];
                    v_graph_s = boost::source(e, graph->graph);
                    v_graph_t = boost::target(e, graph->graph);

                    if( to_tree.find(v_graph_s) == to_tree.end() )
                    {
                        v_tree_s = add_new_vertex(tree);
                        tree->get_vertex_as<undirected_node_t > (v_tree_s)->point = state_space->clone_point(graph->get_vertex_as<undirected_node_t > (v_graph_s)->point);
                        to_tree[v_graph_s] = v_tree_s;
                        to_graph[v_tree_s] = v_graph_s;

                        if( s_assignment.has_robot_on(v_graph_s) )
                        {
                            robot_id = s_assignment.get_robot(v_graph_s);
                            s_assignment.change_position(robot_id, v_tree_s);
                        }

                        if( s_new.has_robot_on(v_graph_s) )
                        {
                            robot_id = s_new.get_robot(v_graph_s);
                            s_new.change_position(robot_id, v_tree_s);
                            //                PRX_WARN_S(robot_id << " )  -----    s_new    ---- change : " << print_point(graph, v_graph_s) << "   with " << print_point(graph, v_tree_s));
                            //                print_assignment(graph, s_new);
                            sim_assignment.add_assignment(v_graph_s, robot_id);
                        }

                        if( t_assignment.has_robot_on(v_graph_s) )
                        {
                            robot_id = t_assignment.get_robot(v_graph_s);
                            t_assignment.change_position(robot_id, v_tree_s);
                        }
                    }
                    else
                    {
                        v_tree_s = to_tree[v_graph_s];
                    }

                    if( to_tree.find(v_graph_t) == to_tree.end() )
                    {
                        v_tree_t = add_new_vertex(tree);
                        tree->get_vertex_as<undirected_node_t > (v_tree_t)->point = state_space->clone_point(graph->get_vertex_as<undirected_node_t > (v_graph_t)->point);
                        to_tree[v_graph_t] = v_tree_t;
                        to_graph[v_tree_t] = v_graph_t;

                        if( s_assignment.has_robot_on(v_graph_t) )
                        {
                            robot_id = s_assignment.get_robot(v_graph_t);
                            s_assignment.change_position(robot_id, v_tree_t);
                        }

                        if( s_new.has_robot_on(v_graph_t) )
                        {
                            robot_id = s_new.get_robot(v_graph_t);
                            s_new.change_position(robot_id, v_tree_t);
                            //                PRX_WARN_S(robot_id << " )  -----    s_new    ---- change : " << print_point(graph, v_graph_t) << "   with " << print_point(graph, v_tree_t));
                            //                print_assignment(graph, s_new);
                            sim_assignment.add_assignment(v_graph_t, robot_id);
                        }

                        if( t_assignment.has_robot_on(v_graph_t) )
                        {
                            robot_id = t_assignment.get_robot(v_graph_t);
                            t_assignment.change_position(robot_id, v_tree_t);
                        }
                    }
                    else
                    {
                        v_tree_t = to_tree[v_graph_t];
                    }

                    if( biconnected_comps[edge_component_index].b_comps_size > 1 )
                    {

                        if( new_nodes.find(edge_component_index) == new_nodes.end() )
                        {
                            v_transshipment = add_new_vertex(tree);

                            biconnected_comps[edge_component_index].v_trans = v_transshipment;
                            biconnected_comps[edge_component_index].biconnected_class = edge_component_index;
                            transshipments_class[v_transshipment] = edge_component_index;
                            tree->get_vertex_as<undirected_node_t > (v_transshipment)->point = state_space->alloc_point();
                            tree->get_vertex_as<graph_pebble_node_t > (v_transshipment)->transshipment = true;
                            new_nodes[edge_component_index] = v_transshipment;

                            e_new = tree->add_edge<pebble_solver_edge_t > (v_transshipment, v_tree_s, 0.5);
                            //                PRX_DEBUG_S("new e#1: " << e_new);
                            tree->get_edge_as<pebble_solver_edge_t > (e_new)->steps = 50;
                            e_new = tree->add_edge<pebble_solver_edge_t > (v_transshipment, v_tree_t, 0.5);
                            //                PRX_DEBUG_S("new e#1: " << e_new);
                            tree->get_edge_as<pebble_solver_edge_t > (e_new)->steps = 50;
                            biconnected_comps[edge_component_index].nodes[v_graph_s] = false;
                            biconnected_comps[edge_component_index].nodes[v_graph_t] = false;
                            biconnected_comps[edge_component_index].vertices_size = 2;
                            biconnected_comps[edge_component_index].a_node = v_graph_s;
                            for( unsigned int i = 0; i < robot_space_dimension; ++i )
                            {
                                tree->get_vertex_as<undirected_node_t > (v_transshipment)->point->at(i) = tree->get_vertex_as<undirected_node_t > (v_transshipment)->point->at(i) + (tree->get_vertex_as<undirected_node_t > (v_tree_s)->point->at(i) + tree->get_vertex_as<undirected_node_t > (v_tree_t)->point->at(i)) / 2;
                            }

                        }
                        else
                        {
                            v_transshipment = new_nodes[edge_component_index];
                            space_point_t* p_trans = tree->get_vertex_as<undirected_node_t > (v_transshipment)->point;
                            if( !boost::edge(v_transshipment, v_tree_s, tree->graph).second )
                            {
                                e_new = tree->add_edge<pebble_solver_edge_t > (v_transshipment, v_tree_s, 0.5);
                                //                    PRX_INFO_S("new e#2: " << e_new);
                                tree->get_edge_as<pebble_solver_edge_t > (e_new)->steps = 50;
                                biconnected_comps[edge_component_index].nodes[v_graph_s] = false;
                                biconnected_comps[edge_component_index].vertices_size++;

                                for( unsigned int i = 0; i < robot_space_dimension; ++i )
                                {
                                    p_trans->at(i) = (p_trans->at(i) + tree->get_vertex_as<undirected_node_t > (v_tree_s)->point->at(i)) / 2;
                                }

                            }
                            if( !boost::edge(v_transshipment, v_tree_t, tree->graph).second )
                            {
                                e_new = tree->add_edge<pebble_solver_edge_t > (v_transshipment, v_tree_t, 0.5);
                                tree->get_edge_as<pebble_solver_edge_t > (e_new)->steps = 50;
                                //                    PRX_WARN_S("new e#3: " << e_new);
                                biconnected_comps[edge_component_index].nodes[v_graph_t] = false;
                                biconnected_comps[edge_component_index].vertices_size++;
                                for( unsigned int i = 0; i < robot_space_dimension; ++i )
                                {
                                    p_trans->at(i) = (p_trans->at(i) + tree->get_vertex_as<undirected_node_t > (v_tree_t)->point->at(i)) / 2;
                                }
                            }
                        }

                    }
                    else
                    {
                        e_new = tree->add_edge<pebble_solver_edge_t > (v_tree_s, v_tree_t, 1);
                        tree->get_edge_as<pebble_solver_edge_t > (e_new)->steps = graph->get_edge_as<pebble_solver_edge_t > (e)->steps;
                    }
                }

                //    PRX_INFO_S("Going to compute maximal biconnected components for " << biconnected_comps.size() << " biconnected graphs");
                //    compute_maximal_biconnected_components(graph);

                //    PRX_WARN_S("FINAL)      v:" << boost::num_vertices(tree->graph) << "    e:" << boost::num_edges(tree->graph));
                //    int g_pos = 1;
                //
                //    foreach(undirected_vertex_index_t v, boost::vertices(tree->graph))
                //    {
                //        PRX_DEBUG_S(g_pos << ") " << v << "  :  " << print_point(tree, v));
                //        g_pos++;
                //    }
                //
                //    foreach(undirected_edge_index_t e, boost::edges(tree->graph))
                //    {
                //        PRX_INFO_S(e);
                //    }
                //
                //    PRX_WARN_S("---------  AFTER TO TREE -----------");
                //    PRX_WARN_S("------------    strt    ------------");
                //    print_assignment(graph, s_assignment);
                //    PRX_WARN_S("------------    s_new    ------------");
                //    print_assignment(graph, s_new);
                //    PRX_WARN_S("-----------     targ    ------------");
                //    print_assignment(graph, t_assignment);
                //    PRX_WARN_S("------------------------------------");
            }

            void pebble_graph_solver_t::convert_path(std::vector<pebble_step_t>* solution, undirected_graph_t* graph, undirected_graph_t* tree, std::vector<pebble_step_t>* tmp_solution)
            {
                int start_reverse, end_reverse;

                std::deque<undirected_vertex_index_t> tmp_path(number_of_vertices);
                std::vector<undirected_vertex_index_t> p_path;
                undirected_vertex_index_t v_curr;
                int tmp_path_index = 0;

                //    print_assignment(graph, sim_assignment);
                //    for( unsigned int s = 0; s < tmp_solution->size(); ++s )
                //    {
                //        std::cout << "pebble : " << tmp_solution->at(s).first << ":";
                //        for( unsigned int i = 1; i < tmp_solution->at(s).second.size(); ++i )
                //        {
                //            std::cout << " -> " << print_point(tree, tmp_solution->at(s).second[i]);
                //        }
                //        std::cout << std::endl;
                //    }
                //    PRX_LOG_ERROR("stop");

                //    for( unsigned int s = 0; s < tmp_solution->size(); ++s )
                //    {
                //        std::cout << tmp_solution->at(s).first << " : ";
                //
                //        foreach(undirected_vertex_index_t vvv, tmp_solution->at(s).second)
                //        {
                //            std::cout << print_point(graph, vvv) << " -> ";
                //        }
                //        std::cout << std::endl;
                //
                //    }

                //    PRX_WARN_S("Converting the Path : " << tmp_solution->size());
                for( unsigned int s = 0; s < tmp_solution->size(); ++s )
                {
                    tmp_path_index = 0;
                    int pebble_id = tmp_solution->at(s).first;
                    //        std::vector<undirected_vertex_index_t> pebble_path;
                    PRX_WARN_S("---------------------  for agent : " << pebble_id << "   step " << s + 1 << "/" << tmp_solution->size() << "    path:" << tmp_solution->at(s).second.size());

                    tmp_path[tmp_path_index] = to_graph[tmp_solution->at(s).second[0]];
                    ++tmp_path_index;
                    for( unsigned int i = 1; i < tmp_solution->at(s).second.size(); ++i )
                    {
                        v_curr = tmp_solution->at(s).second[i];
                        if( !tree->get_vertex_as<graph_pebble_node_t > (v_curr)->transshipment )
                        {
                            PRX_INFO_S(tmp_solution->at(s).first << " (" << i << "/ " << tmp_solution->at(s).second.size() - 1 << "):  Its not transshipment node : " << print_point(graph, tmp_solution->at(s).second[i - 1]) << " -> " << print_point(graph, tmp_solution->at(s).second[i]));
                            v_curr = to_graph[v_curr];
                            sim_assignment.change_position(pebble_id, v_curr);
                            tmp_path[tmp_path_index] = v_curr;
                            ++tmp_path_index;
                        }
                        else
                        {
                            //                PRX_DEBUG_S(tmp_solution->at(s).first << " (" << i << "/ " << tmp_solution->at(s).second.size() - 1 << "):  WE HAVE transshipment node : " << print_point(tree, tmp_solution->at(s).second[i]));
                            //                PRX_DEBUG_S("agent " << pebble_id << "  is going from : " << print_point(tree, tmp_solution->at(s).second[i - 1]) << "     ->  " << print_point(tree, tmp_solution->at(s).second[i + 1]));
                            undirected_vertex_index_t graph_start = to_graph[tmp_solution->at(s).second[i - 1]];
                            undirected_vertex_index_t graph_goal = to_graph[tmp_solution->at(s).second[i + 1]];
                            PRX_INFO_S("size : " << tmp_solution->at(s).second.size());
                            PRX_INFO_S("TRANSSHIPMENT for agent " << pebble_id << "  is going from : " << print_point(graph, graph_start) << "     ->  " << print_point(graph, graph_goal));

                            //                PRX_INFO_S("The assignemnt now : ");
                            //                print_assignment(graph, sim_assignment);
                            tmp_vector_index = 0;
                            if( free_path(graph, graph_start, graph_goal) )
                            {
                                PRX_DEBUG_S("NICE agent : " << pebble_id << "   got free path size: " << tmp_vector_index);
                                //                    p_path.clear();
                                //                    p_path.insert(p_path.end(), tmp_vector.begin(), tmp_vector.begin() + tmp_vector_index);
                                //                                        if( !path_validation(graph, p_path) )
                                //                                            PRX_LOG_ERROR("The path after the free path is not valid");
                                for( int i = 0; i < tmp_vector_index; ++i )
                                {
                                    tmp_path[tmp_path_index] = tmp_vector[i];
                                    ++tmp_path_index;
                                }

                                //                    foreach(undirected_vertex_index_t vvvv, path)
                                //                    {
                                //                        PRX_DEBUG_S("Free path : " << print_point(graph, vvvv));
                                //                    }
                                //                    if( path.size() > 1 )
                                //                    PRX_ASSERT(path.size() > 1);
                                //                    PRX_INFO_S("the new position for robot : " << pebble_id << " now is : " << print_point(graph, path.back()));
                                sim_assignment.change_position(pebble_id, graph_goal);
                                i++;
                            }
                            else
                            {
                                //                    undirected_vertex_index_t w = tmp_solution->at(s).second[i];
                                undirected_vertex_index_t v_adj;
                                undirected_vertex_index_t v_cut;
                                //                    PRX_DEBUG_S(i << "/ " << tmp_solution->at(s).second.size() - 1 << " ) going for the clean v_cut (" << w << "):" << print_point(graph, w) << "   for class: " << graph->edge_components[boost::edge(mbc[w].first[0], mbc[w].first[1], graph->graph).first]);


                                //                    print_assignment(graph, sim_assignment);
                                //                    PRX_DEBUG_S("------------------------------------------------------------");
                                boost::tie(v_cut, v_adj) = compute_circle_with_cut(graph, tree, sim_assignment, graph_start, graph_goal);

                                PRX_ERROR_S("No free path for agent " << pebble_id << "   the v_cut:" << print_point(graph, v_cut) << "  v_adj:" << print_point(graph, v_adj) << "  is mcbc valid :" << mcbc_validation(graph));
                                PRX_WARN_S("Solution size: " << solution->size());
                                if( false && i > 1 ) //TODO: When we are just passing through. But what if i-2 is again transshipment vertex. (look kornhauser)
                                {
                                    //                        PRX_ERROR_S("solution size: " << pebble_path.size() << "      i : " << i);
                                    //                        PRX_ASSERT(pebble_path.size() == i);
                                    //                        v_cut = pebble_path[i - 1];
                                    //                        v_adj = pebble_path[i - 2];
                                    //                        sim_assignment.change_position(v_cut, v_adj);
                                    //                        pebble_path.pop_back();
                                    //                        solution->push_back(std::make_pair(pebble_id, pebble_path));
                                    //                        pebble_path.clear();
                                    //                        //                        PRX_WARN_S("v_cut (" << v_cut << "):" << print_point(graph, v_cut) << "    v_adj (" << v_adj << "):" << print_point(graph, v_adj));
                                    //                        //                        PRX_INFO_S("goint to the point : " << graph_goal << "  )  " << print_point(graph, graph_goal));
                                    //
                                    //                        pass_through(solution, sim_assignment, v_adj, v_cut, graph_goal);
                                }
                                else
                                {
                                    if( tmp_path_index > 0 )
                                    {
                                        p_path.clear();
                                        p_path.assign(tmp_path.begin(), tmp_path.begin() + tmp_path_index);
                                        //                            if( !path_validation(graph, p_path) )
                                        //                                PRX_LOG_ERROR("The path 1 is not valid");
                                        PRX_INFO_S("path that has to go in solution tmp_path_size: " << tmp_path_index << "   p_path_size:" << p_path.size());
                                        solution->push_back(std::make_pair(pebble_id, p_path));
                                        tmp_path_index = 0;
                                    }
                                    //                        PRX_INFO_S("The assignemnt now : ");
                                    //                        print_assignment(graph, sim_assignment);

                                    undirected_vertex_index_t v_empty_start = NULL;
                                    PRX_DEBUG_S("has v_adj robot: " << sim_assignment.has_robot_on(v_adj));
                                    start_reverse = solution->size();
                                    free_cycle(solution, pebble_id, graph, sim_assignment);
                                    end_reverse = solution->size() - 1;
                                    if( sim_assignment.has_robot_on(v_adj) )
                                    {
                                        PRX_ERROR_S("robot " << sim_assignment.get_robot(v_adj) << " blocking v_adj: " << print_point(graph, v_adj));

                                        graph->get_vertex_as<graph_pebble_node_t > (v_cut)->obstacle_id = obstacle_global_id;
                                        //                            start_reverse = solution->size();
                                        v_empty_start = find_closest_on_graph_with(graph, false, v_adj, sim_assignment, false);
                                        if( v_empty_start != NULL )
                                        {
                                            int pebble_iid = sim_assignment.get_robot(v_adj);
                                            PRX_DEBUG_S("going to push pebble " << sim_assignment.get_robot(v_adj) << "   to position : " << print_point(graph, v_empty_start));
                                            push_graph(solution, graph, v_adj, v_empty_start, sim_assignment, false);
                                            PRX_DEBUG_S("the pebble : " << pebble_iid << "   is now on the position : " << print_point(graph, sim_assignment.get_position(pebble_iid)));
                                            v_empty_start = NULL;
                                            end_reverse = solution->size() - 1;
                                        }
                                        else
                                        {
                                            PRX_WARN_S("WE need to push it on the mcbc");
                                            //if( !push_pebble_once(solution, tree, v_adj, sim_assignment) )
                                            //                                start_reverse = -1;
                                            v_empty_start = find_first_empty_on_mbc(graph, sim_assignment, graph_start, graph_goal);
                                            //PRX_WARN_S("empty vertex for rotation is: " << print_point(graph, v_empty_start) << "   we want to go " << print_point(graph, graph_goal));

                                            undirected_vertex_index_t v_empty = rotate(solution, sim_assignment, v_empty_start, v_cut, graph_goal);
                                            //PRX_ERROR_S("v_empty : " << print_point(initial_graph, v_empty));
                                            //print_assignment(graph, sim_assignment);
                                            //PRX_ERROR_S("MOVE ");
                                            move_once(solution, sim_assignment, v_adj, v_cut);
                                            //PRX_ERROR_S("v_empty : " << print_point(initial_graph, v_empty));
                                            //print_assignment(graph, sim_assignment);
                                            //PRX_ERROR_S("ROTATE 2");
                                            v_empty = rotate(solution, sim_assignment, v_cut, v_empty_start, v_empty);
                                            //PRX_INFO_S("The assignemnt now : ");
                                            //print_assignment(graph, sim_assignment);
                                            //PRX_LOG_ERROR("stop");
                                        }


                                    }

                                    //                        PRX_WARN_S("pebble " << pebble_id << " is going from " << print_point(graph, graph_start) << " -> " << print_point(graph, graph_goal));
                                    //                        PRX_WARN_S("v_cut (" << v_cut << "):" << print_point(graph, v_cut) << "    v_adj (" << v_adj << "):" << print_point(graph, v_adj));
                                    PRX_WARN_S("before biconnected_swap : sim_assignment size:  " << sim_assignment.size() << "    s_initial.size(): " << s_initial.size());
                                    PRX_INFO_S(all_solution_validation(graph, solution, s_initial, false));
                                    PRX_ASSERT(sim_assignment.size() == s_initial.size());
                                    PRX_DEBUG_S("Graph goal has pebble:  " << sim_assignment.has_robot_on(graph_goal));
                                    if( sim_assignment.has_robot_on(graph_goal) )
                                        PRX_ERROR_S("The blocker is : " << sim_assignment.get_robot(graph_goal));

                                    biconnected_swap(solution, sim_assignment, v_adj, v_cut, graph_start, graph_goal);
                                    PRX_WARN_S("after biconnected_swap : sim_assignment size:  " << sim_assignment.size() << "    s_initial.size(): " << s_initial.size());
                                    PRX_INFO_S(all_solution_validation(graph, solution, s_initial, false));
                                    PRX_ASSERT(sim_assignment.size() == s_initial.size());


                                    if( v_empty_start != NULL )
                                    {

                                        PRX_DEBUG_S("rotate to take out the blocker from mcbc");
                                        undirected_vertex_index_t v_empty = rotate(solution, sim_assignment, v_empty_start, v_cut, graph_start);
                                        //                            PRX_ERROR_S("v_empty : " << print_point(initial_graph, v_empty));
                                        //                            print_assignment(graph, sim_assignment);
                                        //                            PRX_ERROR_S("MOVE ");
                                        move_once(solution, sim_assignment, v_cut, v_adj);
                                        //                            PRX_ERROR_S("v_empty : " << print_point(initial_graph, v_empty));
                                        //                            print_assignment(graph, sim_assignment);
                                        //                            PRX_ERROR_S("ROTATE 2");
                                        v_empty = rotate(solution, sim_assignment, v_cut, v_empty_start, v_empty);
                                        //                            PRX_INFO_S("The assignemnt now : ");
                                        //                            print_assignment(graph, sim_assignment);
                                        //                            PRX_LOG_ERROR("stop");
                                        PRX_WARN_S("SO is it you? : " << all_solution_validation(graph, solution, s_initial, false));
                                    }
                                    revert(solution, start_reverse, end_reverse, pebble_id, pebble_id, sim_assignment);

                                    PRX_INFO_S("DONE going for the next step");


                                    //                        PRX_WARN_S("END OF REVERSE");
                                    //                        print_assignment(graph, sim_assignment);

                                }
                            }
                        }
                        PRX_WARN_S("At the end of step  " << i << "   for agent : " << pebble_id << "   at position : " << print_point(graph, sim_assignment.get_position(pebble_id)));
                        PRX_INFO_S(all_solution_validation(graph, solution, s_initial, false));
                        PRX_INFO_S("sim_assignment size:  " << sim_assignment.size() << "    s_initial.size(): " << s_initial.size());
                        PRX_ASSERT(sim_assignment.size() == s_initial.size());
                    }
                    if( tmp_path_index > 0 )
                    {
                        //            PRX_WARN_S("The path for agent : " << pebble_id);
                        //
                        //            foreach(undirected_vertex_index_t vvvv, pebble_path)
                        //            {
                        //                PRX_WARN_S("pebble_path : " << print_point(graph, vvvv));
                        //            }            
                        p_path.clear();
                        p_path.assign(tmp_path.begin(), tmp_path.begin() + tmp_path_index);
                        //            if( !path_validation(graph, p_path) )
                        //                PRX_LOG_ERROR("The path 2 is not valid");
                        solution->push_back(std::make_pair(pebble_id, p_path));
                        tmp_path_index = 0;
                    }
                    PRX_WARN_S("At the end for agent : " << pebble_id << "   at position : " << print_point(graph, sim_assignment.get_position(pebble_id)));
                    p_path.clear();

                }
                PRX_WARN_S("solution validation in convert path");
                PRX_WARN_S("solution validation : " << all_solution_validation(graph, solution, s_initial));
                //    PRX_WARN_S("----------------------------------------------");
                //    for( size_t i = 0; i < solution->size(); ++i )
                //    {
                //        PRX_ERROR_S("agent : " << solution->at(i).first);
                //
                //        foreach(undirected_vertex_index_t vert, solution->at(i).second)
                //        {
                //            PRX_DEBUG_S(print_point(graph, vert));
                //        }
                //    }
                //    PRX_WARN_S("----------------------------------------------");
            }

            bool pebble_graph_solver_t::free_path(undirected_graph_t* graph, undirected_vertex_index_t start, undirected_vertex_index_t goal)
            {
                bfs_global_id++;
                static_heap.restart();
                //    PRX_DEBUG_S("the start is (" << start << "): " << print_point(graph, start) << "  has neigh: " << boost::degree(start, graph->graph));


                graph->predecessors[goal] = goal;
                graph->get_vertex_as<graph_pebble_node_t > (goal)->bfs_id = bfs_global_id;

                static_heap.push_back(goal);
                while( !static_heap.empty() )
                {
                    undirected_vertex_index_t v = static_heap.pop_front();

                    foreach(undirected_vertex_index_t adj, boost::adjacent_vertices(v, graph->graph))
                    {
                        if( adj == start )
                        {
                            graph->predecessors[adj] = v;
                            //                PRX_DEBUG_S("new path:");
                            for( undirected_vertex_index_t p = graph->predecessors[start]; p != goal; p = graph->predecessors[p] )
                            {
                                //                        std::cout << print_point(graph, p) << "  - >  ";
                                //                    PRX_DEBUG_S("free path add :" << print_point(graph, p));
                                tmp_vector[tmp_vector_index] = p;
                                ++tmp_vector_index;
                            }
                            tmp_vector[tmp_vector_index] = goal;
                            ++tmp_vector_index;
                            return true;
                        }

                        if( !sim_assignment.has_robot_on(adj) && graph->get_vertex_as<graph_pebble_node_t > (adj)->bfs_id != bfs_global_id )
                        {
                            static_heap.push_back(adj);
                            graph->predecessors[adj] = v;
                            graph->get_vertex_as<graph_pebble_node_t > (adj)->bfs_id = bfs_global_id;
                        }
                    }
                    //            PRX_DEBUG_S("static_heap size: " << static_heap.size());
                    //            PRX_DEBUG_S("v: " << print_point(graph, v) << "     goal :" << print_point(graph, goal));

                }
                //    PRX_DEBUG_S("FREE PATH FALSE");
                return false;
            }

            bool pebble_graph_solver_t::find_closest_path_for_mcbc(const undirected_graph_t* graph, undirected_vertex_index_t start, undirected_vertex_index_t goal)
            {
                //It will start from the end so as the path will be correct order.
                bfs_global_id++;
                tmp_vector_index = 0;
                static_heap.restart();
                //    PRX_DEBUG_S("the start is (" << start << "): " << print_point(graph, start) << "  has neigh: " << boost::degree(start, graph->graph));

                graph->predecessors[goal] = goal;
                graph->get_vertex_as<graph_pebble_node_t > (goal)->bfs_id = bfs_global_id;

                static_heap.push_back(goal);
                while( !static_heap.empty() )
                {
                    //        PRX_DEBUG_S("static_heap size: " << static_heap.size());
                    undirected_vertex_index_t v = static_heap.pop_front();

                    //        PRX_DEBUG_S("v: " << print_point(graph, v) << "     start :" << print_point(graph, start) << "     goal :" << print_point(graph, goal));

                    foreach(undirected_vertex_index_t u, boost::adjacent_vertices(v, graph->graph))
                    {
                        if( u == start )
                        {
                            graph->predecessors[u] = v;
                            //                PRX_DEBUG_S("new path:");
                            for( undirected_vertex_index_t p = start; p != goal; p = graph->predecessors[p] )
                            {
                                //                        std::cout << print_point(graph, p) << "  - >  ";
                                //                    PRX_DEBUG_S(print_point(graph, p));
                                graph->get_vertex_as<graph_pebble_node_t > (p)->on_mcbc = on_mcbc_global;
                                tmp_vector[tmp_vector_index] = p;
                                tmp_vector_index++;
                            }
                            graph->get_vertex_as<graph_pebble_node_t > (goal)->on_mcbc = on_mcbc_global;
                            tmp_vector[tmp_vector_index] = goal;
                            tmp_vector_index++;
                            //                PRX_INFO_S("YES path to " << print_point(graph, goal));
                            return true;
                        }

                        graph_pebble_node_t* node = graph->get_vertex_as<graph_pebble_node_t > (u);

                        //            PRX_DEBUG_S("adj : " << print_point(graph, u) << "     obs_id: " << node->obstacle_id << " / " << obstacle_global_id << "     bfs: " << node->bfs_id << " / " << bfs_global_id);
                        if( node->bfs_id != bfs_global_id )
                        {
                            if( node->on_mcbc != on_mcbc_global )
                            {
                                static_heap.push_back(u);
                            }
                            else
                            {
                                PRX_DEBUG_S(print_point(graph, u) << "   is on the path");
                                node->on_path = on_path_global;
                            }

                            //                PRX_DEBUG_S(start <<" - " << goal << "  Changing predecessor of node : " << print_point(graph,u) << "   from: " << print_point(graph,graph->predecessors[u]) << "  ->  " << print_point(graph,v) );
                            graph->predecessors[u] = v;
                            node->bfs_id = bfs_global_id;
                        }
                    }


                }


                //    PRX_ERROR_S("No path to " << print_point(graph, goal));
                return false;
            }

            bool pebble_graph_solver_t::check_second_path(const undirected_graph_t* graph, undirected_vertex_index_t start, undirected_vertex_index_t goal)
            {
                //It will start from the end so as the path will be correct order.
                bfs_global_id++;
                static_heap.restart();
                mcbc_blockers.restart();
                graph_pebble_node_t* node;

                std::deque<undirected_vertex_index_t>::iterator it1;
                std::deque<undirected_vertex_index_t>::iterator it2;
                undirected_vertex_index_t v_support1 = NULL;
                undirected_vertex_index_t v_support2 = NULL;
                PRX_DEBUG_S("the goal is (" << goal << "): " << print_point(graph, goal) << "  has neigh: " << boost::degree(goal, graph->graph));

                graph->predecessors[goal] = goal;
                graph->get_vertex_as<graph_pebble_node_t > (goal)->bfs_id = bfs_global_id;

                static_heap.push_back(goal);
                while( !static_heap.empty() )
                {
                    //        PRX_DEBUG_S("static_heap size: " << static_heap.size());
                    undirected_vertex_index_t v = static_heap.pop_front();

                    //        PRX_DEBUG_S("v: " << print_point(graph, v) << "     goal :" << print_point(graph, goal));

                    //        graph_pebble_node_t* v_node = graph->get_vertex_as<graph_pebble_node_t > (v);

                    foreach(undirected_vertex_index_t u, boost::adjacent_vertices(v, graph->graph))
                    {
                        node = graph->get_vertex_as<graph_pebble_node_t > (u);
                        //            PRX_INFO_S(print_point(graph, u) << "    is on the path : " << node->on_path << "/" << on_path_global);
                        //            if( false && node->on_path == on_path_global )
                        //            {
                        //                it1 = std::find(mcbc.begin(), mcbc.begin() + mcbc_index, u);
                        //
                        //                //                PRX_WARN_S("it1:  " << print_point(graph, *it1) << "    c.begin: " << print_point(graph, *(circle.begin())) << "    prev: " << print_point(graph, *(it1 - 1)) << "   prev point on path: " << graph->get_vertex_as<graph_pebble_node_t > (*(it1 - 1))->on_path);
                        //                //                PRX_WARN_S("it1:  " << print_point(graph,*it1) << "    c.end: " << print_point(graph,*(circle.end()-1)) << "    prev: " << print_point(graph,*(it1 + 1)) << "   prev point on path: " << graph->get_vertex_as<graph_pebble_node_t > (*(it1 + 1))->on_path);
                        //                //                PRX_WARN_S("it1 : " << print_point(graph, *it1) << "  dist: " << std::distance(mcbc.begin(), it1) << " < " << mcbc_index - 1 << "    it2  : " << *(it1 + 1));
                        //
                        //                if( std::distance(mcbc.begin(), it1) < mcbc_index - 1 && graph->get_vertex_as<graph_pebble_node_t > (*(it1 + 1))->on_path == on_path_global )
                        //                    //if( it1 != circle.begin() && graph->get_vertex_as<graph_pebble_node_t > (*(it1 - 1))->on_path )
                        //                {
                        //
                        //                    it2 = it1 + 1;
                        //                    undirected_vertex_index_t v_mid = *(it1 + 1);
                        //                    //                    PRX_WARN_S("it1 : " << print_point(graph, *it1) << "      it2 : " << print_point(graph, *it2));
                        //                    graph->predecessors[u] = v;
                        //                    int index = std::distance(mcbc.begin(), it1);
                        //                    //                    for( unsigned int i = 0; i <= dist; ++i )
                        //                    //                    {
                        //                    //                        tmp_vector[tmp_vector_index] = mcbc[i];
                        //                    //                        ++tmp_vector_index;
                        //                    //                    }
                        //                    //                    PRX_ASSERT(tmp_vector[tmp_vector_index - 1] == *it1);
                        //                    tmp_vector_index = 0;
                        //                    //                    PRX_DEBUG_S("index: " << index <<"   goal : " <<print_point(graph,goal));
                        //                    //                    PRX_ERROR_S("---------------------- the first part of the circle ------------------");
                        //                    //                    for( int i = 0; i < index; ++i )
                        //                    //                    {
                        //                    //                        PRX_INFO_S(i << ") " << print_point(graph, mcbc[i]));
                        //                    //                    }
                        //                    for( undirected_vertex_index_t p = *it1; p != goal; p = graph->predecessors[p] )
                        //                    {
                        //                        //                        PRX_DEBUG_S(tmp_vector_index);
                        //                        tmp_vector[tmp_vector_index] = p;
                        //                        ++tmp_vector_index;
                        //                        graph->get_vertex_as<graph_pebble_node_t > (p)->on_mcbc = on_mcbc_global;
                        //                        //                        std::cout << print_point(graph, p) << "(" << goal << "-" << p << ")  - >  ";
                        //                        //                    PRX_DEBUG_S(print_point(graph, p));                        
                        //                    }
                        //
                        //
                        //                    //                    PRX_DEBUG_S("dist : " << dist << "   circle.size: " << circle.size() << "   i> " << circle.size() - dist - 1);
                        //                    int count = 0;
                        //                    int new_place = index + tmp_vector_index;
                        //                    //                    PRX_DEBUG_S("tmp_index: " << tmp_vector_index<< "   new place : " << new_place);
                        //                    for( int i = mcbc_index - 1; i > index; --i )
                        //                    {
                        //                        mcbc[new_place + count] = mcbc[i];
                        //                        //                        PRX_WARN_S(new_place + count << ") " << print_point(graph, mcbc[new_place + count]));
                        //                        count++;
                        //                    }
                        //
                        //                    //                    PRX_DEBUG_S("count : " << count);
                        //                    for( int i = 0; i < tmp_vector_index; ++i )
                        //                    {
                        //                        mcbc[index + i] = tmp_vector[i];
                        //                        //                        PRX_DEBUG_S(index +i << ") " << print_point(graph, mcbc[index +i]));
                        //                    }
                        //                    mcbc_index = new_place + count - 1;
                        //                    //                    PRX_DEBUG_S("mcbc index :  " << mcbc_index);
                        //
                        //                    //                    PRX_INFO_S("the first part of the circle");
                        //                    //                    for( int i = 0; i < mcbc_index; ++i )
                        //                    //                    {
                        //                    //                        PRX_INFO_S(print_point(graph, mcbc[i]));
                        //                    //                    }
                        //                    for( undirected_vertex_index_t p = v_mid; p != start; p = graph->predecessors[p] )
                        //                    {
                        //                        mcbc[mcbc_index] = p;
                        //                        //                        PRX_INFO_S(print_point(graph, mcbc[mcbc_index]));
                        //                        ++mcbc_index;
                        //                        graph->get_vertex_as<graph_pebble_node_t > (p)->on_mcbc = on_mcbc_global;
                        //                        //                        std::cout << print_point(graph, p) << "  - >  ";
                        //                        //                    PRX_DEBUG_S(print_point(graph, p));                        
                        //                    }
                        //
                        //
                        //                    //                    PRX_INFO_S("YES path to " << print_point(graph, goal));
                        //                    return true;
                        //                    //                    it2 = it1 + 1;
                        //                    //                    PRX_WARN_S("it1 : " << print_point(graph, *it1) << "      it2 : " << print_point(graph, *it2));
                        //                    //                    graph->predecessors[u] = v;
                        //                    //                    tmp_vector_index = 0;
                        //                    //                    int dist = std::distance(mcbc.begin(), it1);
                        //                    //                    for( unsigned int i = 0; i <= dist; ++i )
                        //                    //                    {
                        //                    //                        tmp_vector[tmp_vector_index] = mcbc[i];
                        //                    //                        ++tmp_vector_index;
                        //                    //                    }
                        //                    //                    PRX_ASSERT(tmp_vector[tmp_vector_index - 1] == *it1);
                        //                    //                    for( undirected_vertex_index_t p = graph->predecessors[*it1]; p != goal; p = graph->predecessors[p] )
                        //                    //                    {
                        //                    //                        //                        PRX_DEBUG_S(tmp_vector_index);
                        //                    //                        tmp_vector[tmp_vector_index] = p;
                        //                    //                        ++tmp_vector_index;
                        //                    //                        std::cout << print_point(graph, p) << "  - >  ";
                        //                    //                        //                    PRX_DEBUG_S(print_point(graph, p));                        
                        //                    //                    }
                        //                    //
                        //                    //                    dist = std::distance(it2, circle.end());
                        //                    //                    PRX_DEBUG_S("dist : " << dist << "   circle.size: " << circle.size() << "   i> " << circle.size() - dist - 1);
                        //                    //                    for( unsigned int i = circle.size() - 1; i > circle.size() - dist; --i )
                        //                    //                    {
                        //                    //                        tmp_vector[tmp_vector_index] = circle[i];
                        //                    //                        ++tmp_vector_index;
                        //                    //                    }
                        //                    //
                        //                    //
                        //                    //                    for( undirected_vertex_index_t p = *it2; p != start; p = graph->predecessors[p] )
                        //                    //                    {
                        //                    //                        //                        PRX_DEBUG_S(tmp_vector_index);
                        //                    //                        tmp_vector[tmp_vector_index] = p;
                        //                    //                        ++tmp_vector_index;
                        //                    //                        std::cout << print_point(graph, p) << "  - >  ";
                        //                    //                        //                    PRX_DEBUG_S(print_point(graph, p));                        
                        //                    //                    }
                        //                    //
                        //                    //                    circle.clear();
                        //                    //                    circle.insert(circle.begin(), tmp_vector.begin(), tmp_vector.begin() + tmp_vector_index);
                        //                    //
                        //                    //                    PRX_INFO_S("YES path to " << print_point(graph, goal));
                        //                    //                    return true;
                        //                }
                        //            }

                        //            PRX_DEBUG_S("adj : " << print_point(graph, u) << "     obs_id: " << node->obstacle_id << " / " << obstacle_global_id << "     bfs: " << node->bfs_id << " / " << bfs_global_id << "     v_node : " << v_node->obstacle_id << " / " << obstacle_global_id);
                        if( node->bfs_id != bfs_global_id )
                        {
                            if( node->on_mcbc != on_mcbc_global )
                            {
                                //                PRX_DEBUG_S("--- Its in  " << print_point(graph, u));
                                static_heap.push_back(u);
                            }
                            else if( node->on_path2 != on_path_global )
                            {
                                mcbc_blockers.push_back(u);
                                node->on_path2 = on_path_global;
                                if( node->on_path == on_path_global )
                                    node->other_pred = graph->predecessors[u];
                                PRX_WARN_S("point : " << print_point(graph, u) << "    is on the path: " << (node->on_path == on_path_global) << "   on mcbc: " << (node->on_mcbc == on_mcbc_global) << "   is obstacle: " << (node->obstacle_id == obstacle_global_id));
                            }
                            graph->predecessors[u] = v;
                            node->bfs_id = bfs_global_id;
                        }
                    }
                }

                PRX_ERROR_S("No path to " << print_point(graph, goal) << "   mcbc_blockers : " << mcbc_blockers.size());

                v_support1 = NULL;
                v_support2 = NULL;
                int support2_index = -1, support1_index = -1, tmp_index1 = -1, tmp_index2 = -1;
                int sup_dist = PRX_INFINITY, new_sup_dist;

                for( int i = 0; i < mcbc_index; ++i )
                {
                    node = graph->get_vertex_as<graph_pebble_node_t > (mcbc[i]);
                    if( node->on_path2 == on_path_global )
                    {
                        if( tmp_index1 != -1 )
                        {
                            //Keeps the last most closest points. Just in case element i 
                            //will be both on path and on path2. 
                            tmp_index2 = tmp_index1;
                            tmp_index1 = i;
                        }
                        else
                        {
                            //Only for the first time
                            support2_index = i;
                            tmp_index1 = i;
                        }
                    }

                    //If support2_index has been initialize and if it is not the first point that
                    //founded to be coming from the goal then 
                    if( support2_index != i && support2_index != -1 && node->on_path == on_path_global )
                    {
                        support1_index = i;
                        if( tmp_index1 != i )
                            support2_index = tmp_index1;
                        else if( tmp_index2 > support2_index )
                            support2_index = tmp_index2;

                        sup_dist = support1_index - support2_index;
                        PRX_DEBUG_S("Hi FIRST LOOP support2_index :" << support2_index << "    support1_index: " << support1_index);
                        PRX_ASSERT(sup_dist > 0);
                        break;
                    }
                }

                PRX_DEBUG_S("index1: " << support1_index << "     index2: " << support2_index);
                if( support1_index != -1 )
                {
                    if( node->on_path2 == on_path_global )
                        tmp_index1 = support1_index;

                    tmp_index2 = tmp_index1;
                    for( int i = support1_index + 1; i < mcbc_index; ++i )
                    {
                        node = graph->get_vertex_as<graph_pebble_node_t > (mcbc[i]);

                        if( node->on_path == on_path_global )
                        {
                            if( tmp_index1 != i )
                                tmp_index2 = tmp_index1;

                            new_sup_dist = i - tmp_index2;
                            if( new_sup_dist < sup_dist )
                            {

                                PRX_ASSERT(new_sup_dist != 0);
                                support2_index = tmp_index2;
                                support1_index = i;
                                PRX_DEBUG_S("Hi with support2_index :" << support2_index << "    support1_index: " << support1_index);
                                sup_dist = new_sup_dist;
                            }

                            tmp_index2 = tmp_index1;
                        }

                        if( node->on_path2 == on_path_global )
                        {
                            tmp_index1 = i;
                        }
                    }

                    v_support1 = mcbc[support1_index];
                    v_support2 = mcbc[support2_index];

                    PRX_DEBUG_S("support1 : " << print_point(graph, v_support1) << "   support2: " << print_point(graph, v_support2));
                    PRX_DEBUG_S("index1: " << support1_index << "     index2: " << support2_index);

                    PRX_ASSERT(graph->get_vertex_as<graph_pebble_node_t > (v_support1)->on_path == on_path_global);
                    PRX_ASSERT(graph->get_vertex_as<graph_pebble_node_t > (v_support2)->on_path2 == on_path_global);
                    PRX_ASSERT(support2_index != support1_index);

                    for( int i = support2_index; i < support1_index; ++i )
                        graph->get_vertex_as<graph_pebble_node_t > (mcbc[i])->on_mcbc--;

                    node = graph->get_vertex_as<graph_pebble_node_t > (v_support1);
                    if( node->on_path2 == on_path_global )
                        v_support1 = node->other_pred;
                    else
                        v_support1 = graph->predecessors[v_support1];

                    tmp_vector_index = 0;
                    for( undirected_vertex_index_t p = v_support2; p != goal; p = graph->predecessors[p] )
                    {
                        node = graph->get_vertex_as<graph_pebble_node_t > (p);
                        tmp_vector[tmp_vector_index] = p;
                        ++tmp_vector_index;
                        node->on_mcbc = on_mcbc_global;
                        node->obstacle_id = obstacle_global_id;
                        //                        std::cout << print_point(graph, p) << "(" << goal << "-" << p << ")  - >  ";
                        PRX_WARN_S("mcbc: " << print_point(graph, p));
                    }


                    for( int i = mcbc_index - 1; i > support2_index; --i )
                    {
                        tmp_vector[tmp_vector_index] = mcbc[i];
                        ++tmp_vector_index;
                        PRX_INFO_S("mcbc: " << print_point(graph, mcbc[i]));
                    }

                    for( undirected_vertex_index_t p = v_support1; p != start; p = graph->predecessors[p] )
                    {
                        node = graph->get_vertex_as<graph_pebble_node_t > (p);
                        tmp_vector[tmp_vector_index] = p;
                        ++tmp_vector_index;
                        node->on_mcbc = on_mcbc_global;
                        node->obstacle_id = obstacle_global_id;
                        //                        std::cout << print_point(graph, p) << "(" << goal << "-" << p << ")  - >  ";
                        PRX_DEBUG_S("mcbc: " << print_point(graph, p));
                    }

                    mcbc_index = support1_index - 1;
                    for( int i = 0; i < tmp_vector_index; ++i )
                    {
                        mcbc[mcbc_index] = tmp_vector[i];
                        ++mcbc_index;
                    }
                }
                else
                {

                    for( int i = 0; i < mcbc_index; ++i )
                    {
                        node = graph->get_vertex_as<graph_pebble_node_t > (mcbc[i]);
                        if( node->on_path == on_path_global )
                            support1_index = i;

                        if( node->on_path2 == on_path_global )
                        {
                            support2_index = i;
                            break;
                        }
                    }
                    v_support1 = mcbc[support1_index];
                    v_support2 = mcbc[support2_index];

                    PRX_DEBUG_S("index1: " << support1_index << "     index2: " << support2_index);
                    PRX_DEBUG_S("support1 : " << print_point(graph, v_support1) << "   support2: " << print_point(graph, v_support2));

                    tmp_vector_index = 0;
                    for( undirected_vertex_index_t p = v_support2; p != goal; p = graph->predecessors[p] )
                    {
                        node = graph->get_vertex_as<graph_pebble_node_t > (p);
                        tmp_vector[tmp_vector_index] = p;
                        ++tmp_vector_index;
                        node->on_mcbc = on_mcbc_global;
                        node->obstacle_id = obstacle_global_id;
                        //                        std::cout << print_point(graph, p) << "(" << goal << "-" << p << ")  - >  ";            
                    }

                    connecting_path_index = 0;
                    for( int i = tmp_vector_index - 1; i >= 0; --i )
                    {
                        connecting_path[connecting_path_index] = tmp_vector[i];
                        ++connecting_path_index;
                        PRX_DEBUG_S("con: " << print_point(graph, tmp_vector[i]));
                    }

                    for( int i = support2_index - 1; i > support1_index; --i )
                    {
                        connecting_path[connecting_path_index] = mcbc[i];
                        ++connecting_path_index;
                        PRX_INFO_S("con: " << print_point(graph, mcbc[i]));
                    }

                    for( undirected_vertex_index_t p = v_support1; p != start; p = graph->predecessors[p] )
                    {
                        node = graph->get_vertex_as<graph_pebble_node_t > (p);
                        connecting_path[connecting_path_index] = p;
                        ++connecting_path_index;
                        node->on_mcbc = on_mcbc_global;
                        node->obstacle_id = obstacle_global_id;
                        PRX_WARN_S("con: " << print_point(graph, p));
                    }

                    graph->get_vertex_as<graph_pebble_node_t > (v_support1)->on_path--;
                    mcbc_blockers.replace_with_last(v_support2);

                    bfs_global_id++;
                    while( !mcbc_blockers.empty() )
                    {
                        undirected_vertex_index_t v = mcbc_blockers.pop_front();

                        foreach(undirected_vertex_index_t adj, boost::adjacent_vertices(v, graph->graph))
                        {
                            node = graph->get_vertex_as<graph_pebble_node_t > (adj);
                            if( node->on_path == on_path_global )
                            {
                                graph->predecessors[adj] = v;
                                //to get the position of the element that we will start
                                //the second path
                                for( tmp_index1 = 0; mcbc[tmp_index1] != adj; ++tmp_index1 );

                                PRX_DEBUG_S("mcbc_index : " << tmp_index1 << " : " << print_point(graph, mcbc[tmp_index1]) << "    adj: " << print_point(graph, adj));
                                for( int i = tmp_index1 + 1; i < support1_index; ++i )
                                    graph->get_vertex_as<graph_pebble_node_t > (mcbc[i])->on_mcbc--;

                                tmp_vector_index = 0;
                                for( undirected_vertex_index_t p = adj; node->on_path2 != on_path_global; p = graph->predecessors[p] )
                                {
                                    node = graph->get_vertex_as<graph_pebble_node_t > (p);
                                    tmp_vector[tmp_vector_index] = p;
                                    ++tmp_vector_index;
                                    PRX_ERROR_S("is node on mcbc : " << (node->on_mcbc == on_mcbc_global));
                                    node->on_mcbc = on_mcbc_global;
                                    node->obstacle_id = obstacle_global_id;
                                    PRX_DEBUG_S("mcbc: " << print_point(graph, p));
                                }

                                for( tmp_index2 = support2_index; mcbc[tmp_index2] != tmp_vector[tmp_vector_index - 1]; ++tmp_index2 )
                                    graph->get_vertex_as<graph_pebble_node_t > (mcbc[tmp_index2])->on_mcbc--;

                                for( int i = tmp_index2 + 1; i < mcbc_index; ++i )
                                {
                                    tmp_vector[tmp_vector_index] = mcbc[i];
                                    ++tmp_vector_index;
                                    PRX_INFO_S("mcbc: " << print_point(graph, mcbc[i]));
                                }


                                mcbc_index = tmp_index1;
                                for( int i = 0; i < tmp_vector_index; ++i )
                                {
                                    mcbc[mcbc_index] = tmp_vector[i];
                                    ++mcbc_index;
                                }

                                for( int i = 0; i < connecting_path_index; ++i )
                                {
                                    mcbc[mcbc_index] = connecting_path[i];
                                    ++mcbc_index;
                                }


                                mcbc_validation(graph);
                                //                    PRX_LOG_ERROR("stop");
                                return true;
                            }

                            if( node->bfs_id != bfs_global_id && node->on_path2 != on_path_global )
                            {
                                if( node->on_mcbc != on_mcbc_global )
                                {
                                    graph->predecessors[adj] = v;
                                    mcbc_blockers.push_back(adj);
                                }
                                node->bfs_id = bfs_global_id;
                            }

                        }
                    }




                }

                mcbc_validation(graph);
                return true;
                //    //Remove the last point of the blockers because we will use it 
                //    //to connect back to the goal
                //    for( int i = 0; i < mcbc_index; ++i )
                //    {
                //        if( mcbc_blockers.has(mcbc[i]) )
                //        {
                //            v_support1 = mcbc[i];
                //            mcbc_blockers.replace_with_last(mcbc[i]);
                //            PRX_WARN_S("remove from blockers : " << print_point(graph, v_support1));
                //            break;
                //        }
                //    }
                //
                //    //Remove the last point of the on the path blockers because we 
                //    //need at least this one in order to connect the previous path
                //    for( int i = mcbc_index - 1; i >= 0; --i )
                //    {
                //        node = graph->get_vertex_as<graph_pebble_node_t > (mcbc[i]);
                //        if( node->on_path == on_path_global )
                //        {
                //            node->on_path--;
                //            PRX_WARN_S("remove from path : " << print_point(graph, mcbc[i]));
                //            break;
                //        }
                //    }
                //    bfs_global_id++;
                //    while( !mcbc_blockers.empty() )
                //    {
                //        undirected_vertex_index_t v = mcbc_blockers.pop_front();
                //
                //        foreach(undirected_vertex_index_t adj, boost::adjacent_vertices(v, graph->graph))
                //        {
                //            node = graph->get_vertex_as<graph_pebble_node_t > (adj);
                //            if( node->on_path == on_path_global )
                //            {
                //                it1 = std::find(mcbc.begin(), mcbc.begin() + mcbc_index, adj);
                //                it2 = it1 + 1;
                //                PRX_WARN_S("it1 : " << print_point(graph, *it1) << "      it2 : " << print_point(graph, *it2));
                //                if( graph->get_vertex_as<graph_pebble_node_t > (*it2)->on_path == on_path_global )
                //                {
                //                    graph->predecessors[adj] = v;
                //                    int keep_mcbc_index = std::distance(mcbc.begin(), it1);
                //                    //                    for( unsigned int i = 0; i <= dist; ++i )
                //                    //                    {
                //                    //                        tmp_vector[tmp_vector_index] = mcbc[i];
                //                    //                        ++tmp_vector_index;
                //                    //                    }
                //                    //                    PRX_ASSERT(tmp_vector[tmp_vector_index - 1] == *it1);
                //                    tmp_vector_index = 0;
                //                    //                    PRX_DEBUG_S("index: " << index <<"   goal : " <<print_point(graph,goal));
                //                    //                    PRX_ERROR_S("---------------------- the first part of the circle ------------------");
                //                    //                    for( int i = 0; i < index; ++i )
                //                    //                    {
                //                    //                        PRX_INFO_S(i << ") " << print_point(graph, mcbc[i]));
                //                    //                    }
                //                    v_support2 = NULL;
                //                    for( undirected_vertex_index_t p = *it1; v_support2 == NULL; p = graph->predecessors[p] )
                //                    {
                //                        //                        PRX_DEBUG_S(tmp_vector_index);
                //                        graph_pebble_node_t* p_node = graph->get_vertex_as<graph_pebble_node_t > (p);
                //                        if( p_node->on_path2 == on_path_global )
                //                            v_support2 = p;
                //
                //                        tmp_vector[tmp_vector_index] = p;
                //                        ++tmp_vector_index;
                //                        p_node->on_mcbc = on_mcbc_global;
                //                        //                        std::cout << print_point(graph, p) << "(" << goal << "-" << p << ")  - >  ";
                //                        //                    PRX_DEBUG_S(print_point(graph, p));                        
                //                    }
                //
                //                    PRX_INFO_S("support2 : " << print_point(graph, v_support2));
                //                    std::deque<undirected_vertex_index_t>::iterator it3 = std::find(mcbc.begin(), mcbc.begin() + mcbc_index, v_support2);
                //                    std::deque<undirected_vertex_index_t>::iterator tmp_it = it3;
                //
                //                    //Add the part of the mcbc to the tmp_vector from support2 to goal
                //                    ++tmp_it;
                //                    while( tmp_it != mcbc.begin() + mcbc_index )
                //                    {
                //                        tmp_vector[tmp_vector_index] = *tmp_it;
                //                        ++tmp_vector_index;
                //                        graph->get_vertex_as<graph_pebble_node_t > (*tmp_it)->on_mcbc = on_mcbc_global;
                //                        PRX_DEBUG_S("adding to the path sup2->goal : " << print_point(graph, *tmp_it));
                //                        ++tmp_it;
                //                    }
                //
                //                    //Remove the from mcbc all the point from v_support1 to v_support2
                //                    --it3;
                //                    while( *it3 != v_support1 )
                //                    {
                //                        graph->get_vertex_as<graph_pebble_node_t > (*it3)->on_mcbc--;
                //                        --it3;
                //                    }
                //
                //
                //                    //connects v_support1 to goal
                //                    connecting_path_index = 0;
                //                    for( undirected_vertex_index_t p = v_support1; p != goal; p = graph->predecessors[p] )
                //                    {
                //                        connecting_path[connecting_path_index] = p;
                //                        ++connecting_path_index;
                //                        graph->get_vertex_as<graph_pebble_node_t > (p)->on_mcbc = on_mcbc_global;
                //                    }
                //
                //                    //add the connection from v_support1 to goal to the tmp_vector
                //                    for( int i = connecting_path_index - 1; i >= 0; --i )
                //                    {
                //                        tmp_vector[tmp_vector_index] = connecting_path[i];
                //                        ++tmp_vector_index;
                //                    }
                //
                //                    //add the path from v_support1 to it2.
                //                    connecting_path_index = 0;
                //                    it3 = std::find(mcbc.begin(), mcbc.begin() + mcbc_index, v_support1);
                //                    --it3;
                //                    while( it3 != it2 )
                //                    {
                //                        connecting_path[connecting_path_index] = *it3;
                //                        ++connecting_path_index;
                //                        --it3;
                //                    }
                //
                //                    //                    PRX_DEBUG_S("it2: " << print_point(graph,*it2) << "     c_path: " << print_point(graph,connecting_path[connecting_path_index]));
                //
                //                    for( undirected_vertex_index_t p = *it2; p != start; p = graph->predecessors[p] )
                //                    {
                //                        connecting_path[connecting_path_index] = p;
                //                        //                        PRX_INFO_S(print_point(graph, mcbc[mcbc_index]));
                //                        ++connecting_path_index;
                //                        graph->get_vertex_as<graph_pebble_node_t > (p)->on_mcbc = on_mcbc_global;
                //                        //                        std::cout << print_point(graph, p) << "  - >  ";
                //                        //                    PRX_DEBUG_S(print_point(graph, p));                        
                //                    }
                //
                //
                //                    //We don't need anything more from mcbc so its time to 
                //                    //restart it and add the final path from it2 to start
                //                    //Starts from keep_mcbc_index where is the it1
                //                    mcbc_index = keep_mcbc_index;
                //                    for( int i = 0; i < tmp_vector_index; ++i )
                //                    {
                //                        mcbc[mcbc_index] = tmp_vector[i];
                //                        ++mcbc_index;
                //                        //                        PRX_DEBUG_S(index +i << ") " << print_point(graph, mcbc[index +i]));
                //                    }
                //                    for( int i = 0; i < connecting_path_index; ++i )
                //                    {
                //                        mcbc[mcbc_index] = connecting_path[i];
                //                        ++mcbc_index;
                //                        //                        PRX_DEBUG_S(index +i << ") " << print_point(graph, mcbc[index +i]));
                //                    }
                //
                //                    for( int i = 0; i < mcbc_index; ++i )
                //                        PRX_INFO_S("mcbc: " << print_point(graph, mcbc[i]));
                //
                //
                //                    mcbc_validation(graph);
                //
                //
                //                    return true;
                //                }
                //                node->bfs_id = bfs_global_id;
                //            }
                //
                //            if( node->bfs_id != bfs_global_id )
                //            {
                //                if( node->obstacle_id != obstacle_global_id && node->on_path2 != on_path_global )
                //                {
                //                    graph->predecessors[adj] = v;
                //                    mcbc_blockers.push_back(adj);
                //                }
                //                node->bfs_id = bfs_global_id;
                //            }
                //
                //
                //        }
                //
                //    }




                //    it1 = std::find(mcbc.begin(), mcbc.begin() + mcbc_index, v_support1);
                //    it2 = std::find(mcbc.begin(), mcbc.begin() + mcbc_index, v_support2);
                //
                //    if( it2 > it1 )
                //    {
                //        undirected_vertex_index_t v_tmp = v_support1;
                //        std::deque<undirected_vertex_index_t>::iterator tmp_it = it1;
                //        it1 = it2;
                //        v_support1 = v_support2;
                //        it2 = tmp_it;
                //        v_support2 = v_tmp;
                //    }
                //    
                //    PRX_INFO_S("AFTER   support1 : " << print_point(graph, *it1) << "   support2 : " << print_point(graph, *it2));
                //    
                //    int new_index = std::distance(mcbc.begin(), it2);   
                //    for(int i=new_index; i<mcbc_index; ++i)
                //    {
                //        graph->get_vertex_as<graph_pebble_node_t > (mcbc[i])->on_mcbc--;
                //    }
                //    mcbc_index = new_index;
                //
                //    for( undirected_vertex_index_t p = v_support2; p != goal; p = graph->predecessors[p] )
                //    {
                //        if( p == v_support1 )
                //            PRX_LOG_ERROR("noooo why it1 is predecessor of the path of t2");
                //
                //        PRX_INFO_S("new mcbc : " << print_point(graph, p));
                //        mcbc[mcbc_index] = p;
                //        ++mcbc_index;
                //        graph->get_vertex_as<graph_pebble_node_t > (p)->on_mcbc = on_mcbc_global;
                //    }
                //    
                //    mcbc[mcbc_index] = goal;
                //    ++mcbc_index;
                //    
                //    tmp_vector_index = 0;
                //    for( undirected_vertex_index_t p = v_support1; p != goal; p = graph->predecessors[p] )
                //    {
                //        if( p == v_support2 )
                //            PRX_LOG_ERROR("noooo why it2 is predecessor of the path of t1");
                //        
                //        PRX_INFO_S("new mcbc22222 : " << print_point(graph,p));
                //        tmp_vector[tmp_vector_index] = p;
                //        ++tmp_vector_index;
                //        graph->get_vertex_as<graph_pebble_node_t > (p)->on_mcbc = on_mcbc_global;
                //    }
                //    
                //    for(int i= tmp_vector_index-1; i>=0; --i)
                //    {
                //        PRX_INFO_S("new mcbc2 : " << print_point(graph,tmp_vector[i]));
                //        mcbc[mcbc_index] = tmp_vector[i];
                //        ++mcbc_index;
                //    }
                //    
                //    if(check_second_path(graph,start,*it1))
                //        return true;
            }

            undirected_vertex_index_t pebble_graph_solver_t::find_closest_vertex_with(const undirected_graph_t* graph, bool robot_on, undirected_vertex_index_t start, const pebble_assignment_t& assign)
            {
                bfs_global_id++;
                static_heap.restart();
                //    PRX_DEBUG_S("the start is (" << start << "): " << print_point(graph, start) << "  want to has robot on: " << robot_on);

                graph->predecessors[start] = start;
                graph->get_vertex_as<graph_pebble_node_t > (start)->bfs_id = bfs_global_id;

                static_heap.push_back(start);
                while( !static_heap.empty() )
                {
                    undirected_vertex_index_t v = static_heap.pop_front();

                    foreach(undirected_vertex_index_t u, boost::adjacent_vertices(v, graph->graph))
                    {
                        graph_pebble_node_t* node = graph->get_vertex_as<graph_pebble_node_t > (u);

                        if( !node->transshipment && node->obstacle_id != obstacle_global_id && assign.has_robot_on(u) == robot_on )
                        {
                            graph->predecessors[u] = v;
                            //                PRX_WARN_S("Will return the vertex : " << print_point(graph, u));
                            return u;
                        }


                        if( node->bfs_id != bfs_global_id )
                        {
                            if( node->obstacle_id != obstacle_global_id )
                                static_heap.push_back(u);
                            graph->predecessors[u] = v;
                            node->bfs_id = bfs_global_id;
                        }
                    }
                    //            PRX_DEBUG_S("static_heap size: " << static_heap.size());
                    //            PRX_DEBUG_S("v: " << print_point(graph, v) << "     goal :" << print_point(graph, goal));

                }
                return NULL;
            }

            undirected_vertex_index_t pebble_graph_solver_t::find_closest_on_graph_with(const undirected_graph_t* graph, bool robot_on, undirected_vertex_index_t start, const pebble_assignment_t& assign, bool assignment_on_tree)
            {
                bfs_global_id++;
                static_heap.restart();
                undirected_vertex_index_t v_to_check;
                //    PRX_DEBUG_S("the start is (" << start << "): " << print_point(graph, start) << "  want to has robot on: " << robot_on);

                graph->predecessors[start] = start;
                graph->get_vertex_as<graph_pebble_node_t > (start)->bfs_id = bfs_global_id;

                static_heap.push_back(start);
                while( !static_heap.empty() )
                {

                    undirected_vertex_index_t v = static_heap.pop_front();

                    foreach(undirected_vertex_index_t u, boost::adjacent_vertices(v, graph->graph))
                    {
                        graph_pebble_node_t* node = graph->get_vertex_as<graph_pebble_node_t > (u);

                        if( assignment_on_tree )
                            v_to_check = to_tree[u];
                        else
                            v_to_check = u;

                        if( !node->transshipment && node->obstacle_id != obstacle_global_id && assign.has_robot_on(v_to_check) == robot_on )
                        {
                            graph->predecessors[u] = v;
                            //                PRX_WARN_S("Will return the vertex : " << print_point(graph, u) << " transh:" << node->transshipment << "   obstacle: " << node->obstacle_id << "/" << obstacle_global_id << "   has robot: " << assign.has_robot_on(to_tree[u]));
                            return u;
                        }


                        if( node->bfs_id != bfs_global_id )
                        {
                            if( node->obstacle_id != obstacle_global_id )
                                static_heap.push_back(u);
                            graph->predecessors[u] = v;
                            node->bfs_id = bfs_global_id;
                        }
                    }
                    //            PRX_DEBUG_S("static_heap size: " << static_heap.size());
                    //            PRX_DEBUG_S("v: " << print_point(graph, v) << "     goal :" << print_point(graph, goal));

                }
                return NULL;
            }

            int pebble_graph_solver_t::find_closest_robot(const undirected_graph_t* graph, undirected_vertex_index_t v, pebble_assignment_t& assign)
            {
                undirected_vertex_index_t u = find_closest_vertex_with(graph, true, v, assign);
                if( u != NULL )
                    return assign.get_robot(u);

                return -1;
            }

            std::pair<undirected_vertex_index_t, bool> pebble_graph_solver_t::find_closest_useful_vertex(const undirected_graph_t* graph, const undirected_graph_t* tree, pebble_assignment_t& assign, undirected_vertex_index_t start, undirected_vertex_index_t w, bool with_robot)
            {
                //start and w are on the tree.
                bfs_global_id++;
                undirected_vertex_index_t useful_neigh = NULL;
                graph_pebble_node_t* w_node = tree->get_vertex_as<graph_pebble_node_t > (w);
                static_heap.restart();
                //    PRX_DEBUG_S("the start is (" << start << "): " << print_point(tree, start));

                tree->predecessors[start] = start;
                tree->get_vertex_as<graph_pebble_node_t > (start)->bfs_id = bfs_global_id;

                static_heap.push_back(start);
                while( !static_heap.empty() )
                {
                    //we need the adjacent vertices on the graph. Thats why we converted the v into the vertex on the graph
                    undirected_vertex_index_t v_tree = static_heap.pop_front();
                    undirected_vertex_index_t v = to_graph[v_tree];

                    foreach(undirected_vertex_index_t u, boost::adjacent_vertices(v, graph->graph))
                    {
                        undirected_vertex_index_t u_tree = to_tree[u];
                        graph_pebble_node_t* node = tree->get_vertex_as<graph_pebble_node_t > (u_tree);
                        if( node->obstacle_id != obstacle_global_id && boost::edge(w, u_tree, tree->graph).second )
                        {
                            if( assign.has_robot_on(u_tree) == with_robot )
                            {
                                tree->predecessors[u_tree] = v_tree;
                                //                    PRX_WARN_S("Will return the vertex : " << print_point(graph, u));
                                return std::make_pair(u_tree, true);
                            }
                            else if( useful_neigh != NULL && w_node->trees[u_tree].holes > 0 )
                            {
                                useful_neigh = u_tree;
                            }
                        }

                        if( node->bfs_id != bfs_global_id )
                        {
                            if( node->obstacle_id != obstacle_global_id )
                                static_heap.push_back(u_tree);
                            graph->predecessors[u_tree] = v_tree;
                            node->bfs_id = bfs_global_id;
                        }
                    }
                    //            PRX_DEBUG_S("static_heap size: " << static_heap.size());
                    //            PRX_DEBUG_S("v: " << print_point(graph, v) << "     goal :" << print_point(graph, goal));
                }
                return std::make_pair(useful_neigh, false);
            }

            undirected_vertex_index_t pebble_graph_solver_t::find_closest_useful_vertex(std::vector<pebble_step_t>* solution, const undirected_graph_t* graph, const undirected_graph_t* tree, pebble_assignment_t& assign, undirected_vertex_index_t v, undirected_vertex_index_t u, int b_class)
            {

                undirected_vertex_index_t v_graph = to_graph[v];
                undirected_vertex_index_t u_graph = to_graph[u];
                graph->get_vertex_as<graph_pebble_node_t > (v_graph)->obstacle_id = obstacle_global_id;
                graph->get_vertex_as<graph_pebble_node_t > (u_graph)->obstacle_id = obstacle_global_id;

                foreach(undirected_vertex_index_t adj, boost::adjacent_vertices(v_graph, graph->graph))
                {
                    if( !graph->get_vertex_as<graph_pebble_node_t > (adj)->transshipment && adj != u_graph &&
                        graph->get_vertex_as<graph_pebble_node_t > (adj)->obstacle_id != obstacle_global_id &&
                        graph->edge_components[boost::edge(v_graph, adj, graph->graph).first] == b_class &&
                        !assign.has_robot_on(to_tree[adj]) )
                    {
                        graph->predecessors[adj] = v_graph;
                        tree->predecessors[to_tree[adj]] = v;
                        return to_tree[adj];
                    }

                }

                foreach(undirected_vertex_index_t adj, boost::adjacent_vertices(u_graph, graph->graph))
                {
                    if( !graph->get_vertex_as<graph_pebble_node_t > (adj)->transshipment && adj != v_graph &&
                        graph->get_vertex_as<graph_pebble_node_t > (adj)->obstacle_id != obstacle_global_id &&
                        graph->edge_components[boost::edge(u_graph, adj, graph->graph).first] == b_class &&
                        !assign.has_robot_on(to_tree[adj]) )
                    {
                        graph->predecessors[adj] = u_graph;
                        tree->predecessors[to_tree[adj]] = u;
                        return to_tree[adj];
                    }

                }

                undirected_vertex_index_t v_start = get_adjacent_except(graph, v_graph, u_graph);

                PRX_DEBUG_S("v_start : " << print_point(graph, v_start) << "    obstacle:" << graph->get_vertex_as<graph_pebble_node_t > (v_start)->obstacle_id << " / " << obstacle_global_id);

                undirected_vertex_index_t v_empty = find_closest_on_graph_with(graph, false, v_start, assign);
                if( v_empty == NULL )
                    return NULL;

                //    PRX_DEBUG_S("v_Start : " << v_start << "     v_empty : " << v_empty);
                push_graph(solution, graph, v_start, v_empty, assign, true);
                //    push_tree(solution, tree, to_tree[v_start], to_tree[v_empty], assign);

                //    PRX_INFO_S("clear succeded with  : " << print_point(graph, v_start));
                return to_tree[v_start];
            }

            undirected_vertex_index_t pebble_graph_solver_t::find_closest_useful_vertex_except(std::vector<pebble_step_t>* solution, const undirected_graph_t* graph, const undirected_graph_t* tree, pebble_assignment_t& assign, undirected_vertex_index_t v, undirected_vertex_index_t avoid1, undirected_vertex_index_t avoid2)
            {
                undirected_vertex_index_t v_avoid1 = NULL;
                undirected_vertex_index_t v_avoid2 = NULL;
                undirected_vertex_index_t v_graph = to_graph[v];
                undirected_vertex_index_t v_ret;

                if( avoid1 != NULL )
                    v_avoid1 = to_graph[avoid1];

                if( avoid2 != NULL )
                    v_avoid2 = to_graph[avoid2];

                foreach(undirected_vertex_index_t adj, boost::adjacent_vertices(v_graph, graph->graph))
                {
                    if( adj != v_avoid1 && adj != v_avoid2 && graph->get_vertex_as<graph_pebble_node_t > (adj)->obstacle_id != obstacle_global_id )
                    {
                        v_ret = to_tree[adj];
                        if( !assign.has_robot_on(v_ret) )
                            return v_ret;
                    }
                }

                return NULL;
            }

            undirected_vertex_index_t pebble_graph_solver_t::find_first_empty_on_mbc(const undirected_graph_t* graph, pebble_assignment_t assign, undirected_vertex_index_t start, undirected_vertex_index_t goal)
            {
                bfs_global_id++;
                static_heap.restart();

                graph->get_vertex_as<graph_pebble_node_t > (start)->bfs_id = bfs_global_id;

                static_heap.push_back(start);
                while( !static_heap.empty() )
                {
                    undirected_vertex_index_t v = static_heap.pop_front();

                    if( v != goal && !assign.has_robot_on(v) )
                    {
                        return v;
                    }

                    foreach(undirected_vertex_index_t adj, boost::adjacent_vertices(v, graph->graph))
                    {
                        graph_pebble_node_t* node = graph->get_vertex_as<graph_pebble_node_t > (adj);
                        if( node->on_mcbc == on_mcbc_global && node->bfs_id != bfs_global_id )
                        {
                            static_heap.push_back(adj);
                            node->bfs_id = bfs_global_id;
                        }
                    }
                }
                return NULL;
            }

            undirected_vertex_index_t pebble_graph_solver_t::get_adjacent_cut_vertex(const undirected_graph_t* graph, undirected_vertex_index_t v)
            {

                foreach(undirected_vertex_index_t v_adj, boost::adjacent_vertices(v, graph->graph))
                {
                    if( graph->get_vertex_as<graph_pebble_node_t > (v_adj)->on_mcbc != on_mcbc_global )
                        return v_adj;
                }
                return NULL;
            }

            std::pair<undirected_vertex_index_t, undirected_vertex_index_t> pebble_graph_solver_t::compute_circle_with_cut(const undirected_graph_t* graph, const undirected_graph_t* tree, pebble_assignment_t& assign, undirected_vertex_index_t start, undirected_vertex_index_t goal)
            {
                on_path_global++;
                on_mcbc_global++;
                obstacle_global_id++;
                tmp_vector_index = 0;
                mcbc_index = 0;
                //    PRX_ERROR_S("start: " << print_point(graph, start) << "     goal : " << print_point(graph, goal));
                int empty_spots_on_circle = 0;
                find_closest_path_for_mcbc(graph, start, goal);
                //    PRX_DEBUG_S("index : " << mcbc_index << "   mcbc: " << mcbc.size());

                for( int i = 0; i < tmp_vector_index - 1; ++i )
                {
                    if( !assign.has_robot_on(tmp_vector[i]) )
                        empty_spots_on_circle++;
                    mcbc[mcbc_index] = tmp_vector[i];
                    mcbc_index++;
                    graph->get_vertex_as<graph_pebble_node_t > (tmp_vector[i])->obstacle_id = obstacle_global_id;
                    PRX_DEBUG_S("FIRST PATH) adds : " << print_point(graph, tmp_vector[i]) << "   holes on cycle: " << empty_spots_on_circle);
                }
                //    graph->get_vertex_as<graph_pebble_node_t > (path[0])->obstacle_id--;

                //    foreach(undirected_vertex_index_t vvt, boost::vertices(graph->graph))
                //    {
                //        graph_pebble_node_t* node = graph->get_vertex_as<graph_pebble_node_t > (vvt);
                //        if( graph->predecessors[vvt] != NULL )
                //            PRX_DEBUG_S(print_point(graph, vvt) << " <-- " << print_point(graph, graph->predecessors[vvt]) << "   is obst: " << node->obstacle_id << "/" << obstacle_global_id << " is on path:" << node->on_path << "/" << on_path_global);
                //        else
                //            PRX_DEBUG_S(print_point(graph, vvt) << " <--  0x0    is obst: " << node->obstacle_id << "/" << obstacle_global_id << " is on path:" << node->on_path << "/" << on_path_global);
                //    }
                //    

                //    circle.insert(circle.end(), path.begin(), path.end() - 1);

                //    PRX_INFO_S("the first part of the circle");
                //
                //    foreach(undirected_vertex_index_t vcircle, circle)
                //    {
                //        PRX_INFO_S(print_point(graph, vcircle));
                //    }

                find_closest_path_for_mcbc(graph, goal, start);

                //    PRX_INFO_S("index : " << mcbc_index << "   mcbc: " << mcbc.size());

                //    foreach(undirected_vertex_index_t vvt, boost::vertices(graph->graph))
                //    {
                //        graph_pebble_node_t* node = graph->get_vertex_as<graph_pebble_node_t > (vvt);
                //        if( graph->predecessors[vvt] != NULL )
                //            PRX_DEBUG_S(print_point(graph, vvt) << " <-- " << print_point(graph, graph->predecessors[vvt]) << "   is obst: " << node->obstacle_id << "/" << obstacle_global_id << " is on path:" << node->on_path << "/" << on_path_global);
                //        else
                //            PRX_DEBUG_S(print_point(graph, vvt) << " <--  0x0    is obst: " << node->obstacle_id << "/" << obstacle_global_id << " is on path:" << node->on_path << "/" << on_path_global);
                //    }


                if( tmp_vector_index > 0 )
                {
                    for( int i = 0; i < tmp_vector_index - 1; ++i )
                    {
                        if( !assign.has_robot_on(tmp_vector[i]) )
                            empty_spots_on_circle++;
                        mcbc[mcbc_index] = tmp_vector[i];
                        mcbc_index++;
                        graph->get_vertex_as<graph_pebble_node_t > (tmp_vector[i])->obstacle_id = obstacle_global_id;
                        PRX_DEBUG_S("SECOND PATH) adds : " << print_point(graph, tmp_vector[i]) << "   holes on cycle: " << empty_spots_on_circle);

                    }
                }
                else
                {
                    mcbc[mcbc_index] = goal;
                    mcbc_index++;
                    graph->get_vertex_as<graph_pebble_node_t > (goal)->obstacle_id = obstacle_global_id;

                    //        PRX_INFO_S("the first part of the circle");
                    //        for( int i = 0; i < mcbc_index; ++i )
                    //        {
                    //            PRX_INFO_S(print_point(graph, mcbc[i]));
                    //        }

                    if( !check_second_path(graph, start, goal) )
                        PRX_LOG_ERROR(" check second path failed :S ");


                    //        PRX_ERROR_S("The circle now : ");
                    //        for( int i = 0; i < mcbc_index; ++i )
                    //        {
                    //            PRX_DEBUG_S(mcbc[i]);
                    //        }
                    //        
                    //        for( int i = 0; i < mcbc_index; ++i )
                    //        {
                    //            if(mcbc[i]!= NULL)
                    //                PRX_INFO_S(print_point(graph, mcbc[i]));
                    //        }

                }


                //    PRX_INFO_S("the FINAL circle");
                //    for( int i = 0; i < mcbc_index; ++i )
                //    {
                //        PRX_INFO_S(print_point(graph, mcbc[i]));
                //    }

                std::pair<undirected_vertex_index_t, undirected_vertex_index_t> v_cut_pair;
                std::pair<undirected_vertex_index_t, undirected_vertex_index_t> v_full_cut_pair;
                double dist = PRX_INFINITY;
                double dist_full = PRX_INFINITY;
                double new_dist;


                //    PRX_WARN_S("start for v_Cut : " << print_point(tree, start) << "   goal for v_Cut : " << print_point(tree, goal));    

                std::deque<undirected_vertex_index_t>::iterator it_s = mcbc.begin();
                std::deque<undirected_vertex_index_t>::iterator it_g = std::find(mcbc.begin(), mcbc.begin() + mcbc_index, goal);

                std::deque<undirected_vertex_index_t>::iterator it = it_s;
                undirected_vertex_index_t v_tmp;

                for( int i = 0; i < mcbc_index; ++i )
                {
                    if( boost::degree(*it, graph->graph) > 2 )
                    {
                        new_dist = fabs(std::distance(it_s, it)) + fabs(std::distance(it_g, it));
                        if( dist > new_dist )
                        {
                            v_tmp = get_v_cut(*it, graph, tree, assign);
                            if( v_tmp != NULL )
                            {
                                v_cut_pair.first = *it;
                                v_cut_pair.second = v_tmp;
                                dist = new_dist;
                            }
                            else if( dist_full > new_dist )
                            {
                                v_tmp = get_adjacent_cut_vertex(graph, *it);
                                if( v_tmp != NULL )
                                {
                                    v_full_cut_pair.first = *it;
                                    v_full_cut_pair.second = v_tmp;
                                    dist_full = new_dist;
                                }
                            }
                        }

                    }

                    it = adjust_iterator(it + 1, mcbc, mcbc_index);
                }

                PRX_ASSERT(dist != PRX_INFINITY || dist_full != PRX_INFINITY);

                //    PRX_WARN_S("dist: " << dist << "     dist_full : " << dist_full);
                if( dist == PRX_INFINITY )
                    return v_full_cut_pair;
                return v_cut_pair;
            }

            void pebble_graph_solver_t::free_cycle(std::vector<pebble_step_t>* solution, int pebble_id, const undirected_graph_t* graph, pebble_assignment_t& assignment)
            {
                undirected_vertex_index_t v_empty;
                for( int i = 0; i < mcbc_index; ++i )
                {
                    if( assignment.has_robot_on(mcbc[i]) && assignment.get_robot(mcbc[i]) != pebble_id )
                    {
                        v_empty = find_closest_on_graph_with(graph, false, mcbc[i], assignment, false);

                        if( v_empty != NULL )
                            push_graph(solution, graph, mcbc[i], v_empty, assignment, false);
                    }
                }
            }

            undirected_vertex_index_t pebble_graph_solver_t::get_v_cut(undirected_vertex_index_t v, const undirected_graph_t* graph, const undirected_graph_t* tree, pebble_assignment_t& assign)
            {
                undirected_vertex_index_t v_cut = NULL;
                std::vector<pebble_step_t> tmp_sol;

                foreach(undirected_vertex_index_t v_adj, boost::adjacent_vertices(v, graph->graph))
                {
                    graph_pebble_node_t* node = graph->get_vertex_as<graph_pebble_node_t > (v_adj);
                    if( node->obstacle_id != obstacle_global_id && node->on_mcbc != on_mcbc_global )
                    {
                        //            if( tree->get_vertex_as<graph_pebble_node_t > (to_tree[v])->trees[to_tree[v_adj]].holes > 0 )
                        if( !assign.has_robot_on(v_adj) || push_pebble_once(&tmp_sol, graph, v_adj, assign, false) )
                        {
                            v_cut = v_adj;
                            if( !assign.has_robot_on(v_cut) )
                                return v_cut;
                        }
                    }
                }
                return v_cut;
            }

            std::vector<undirected_vertex_index_t>::iterator pebble_graph_solver_t::adjust_iterator(std::vector<undirected_vertex_index_t>::iterator it, std::vector<undirected_vertex_index_t>& vec)
            {
                if( it >= vec.end() )
                {
                    //        PRX_WARN_S("going back to begin  " << *(vec.begin()));
                    return vec.begin();
                }
                if( it < vec.begin() )
                {
                    //        PRX_WARN_S("going back to end  " << *(vec.end() - 1));
                    return vec.end() - 1;
                }
                return it;
            }

            std::deque<undirected_vertex_index_t>::iterator pebble_graph_solver_t::adjust_iterator(std::deque<undirected_vertex_index_t>::iterator it, std::deque<undirected_vertex_index_t>& vec, int last_element)
            {
                if( it >= vec.begin() + last_element )
                {
                    //        PRX_WARN_S("going back to begin  " << *(vec.begin()));
                    return vec.begin();
                }
                if( it < vec.begin() )
                {
                    //        PRX_WARN_S("going back to end  " << *(vec.end() - 1));
                    return vec.begin() + last_element - 1;
                }
                return it;
            }

            bool pebble_graph_solver_t::push_pebble_once(std::vector<pebble_step_t>* solution, const undirected_graph_t* graph, undirected_vertex_index_t v, pebble_assignment_t& assign, bool execute_push)
            {
                //        PRX_DEBUG_S("Push pebble " << assign.get_robot(v) << "   from position : " << print_point(graph, v)); 
                //        PRX_DEBUG_S("------ s_new  BOFORE PUSH ONCE -----------");
                //        print_assignment(graph,assign);    
                undirected_vertex_index_t v_empty = find_closest_vertex_with(graph, false, v, assign);
                if( v_empty != NULL )
                {
                    //        PRX_DEBUG_S(" the empty is : " << print_point(graph, v_empty));
                    if( execute_push )
                    {
                        push_tree(solution, graph, v, v_empty, assign);
                    }
                    return true;

                }
                return false;
            }

            bool pebble_graph_solver_t::push_pebble_once_no_path(const undirected_graph_t* graph, undirected_vertex_index_t v, pebble_assignment_t& assign, bool execute_push)
            {
                //        PRX_DEBUG_S("Push pebble " << assign.get_robot(v) << "   from position : " << print_point(graph, v)); 
                //        PRX_DEBUG_S("------ s_new  BOFORE PUSH ONCE -----------");
                //        print_assignment(graph,assign);    
                undirected_vertex_index_t v_empty = find_closest_vertex_with(graph, false, v, assign);
                if( v_empty != NULL )
                {
                    //        PRX_DEBUG_S(" the empty is : " << print_point(graph, v_empty));
                    if( execute_push )
                    {
                        push_tree_no_path(graph, v, v_empty, assign);
                    }
                    return true;

                }
                return false;
            }

            void pebble_graph_solver_t::push_tree(std::vector<pebble_step_t>* solution, const undirected_graph_t* graph, undirected_vertex_index_t v, undirected_vertex_index_t u, pebble_assignment_t& assign)
            {
                undirected_vertex_index_t g_pred;
                undirected_vertex_index_t v_prev;
                for( undirected_vertex_index_t p = u; p != v; p = graph->predecessors[p] )
                {
                    g_pred = graph->predecessors[p];
                    if( assign.has_robot_on(g_pred) )
                    {
                        int pebble_id = assign.get_robot(g_pred);
                        if( graph->get_vertex_as<graph_pebble_node_t > (p)->transshipment )
                        {
                            robot_path3[0] = g_pred;
                            robot_path3[1] = p;
                            robot_path3[2] = v_prev;
                            assign.change_position(pebble_id, v_prev);
                            solution->push_back(std::make_pair(pebble_id, robot_path3));
                        }
                        else
                        {
                            robot_path2[0] = g_pred;
                            robot_path2[1] = p;
                            assign.change_position(pebble_id, p);
                            solution->push_back(std::make_pair(pebble_id, robot_path2));
                        }
                    }
                    v_prev = p;
                }
            }

            void pebble_graph_solver_t::push_tree_no_path(const undirected_graph_t* graph, undirected_vertex_index_t v, undirected_vertex_index_t u, pebble_assignment_t& assign)
            {
                undirected_vertex_index_t g_pred;
                undirected_vertex_index_t v_prev;
                for( undirected_vertex_index_t p = u; p != v; p = graph->predecessors[p] )
                {
                    g_pred = graph->predecessors[p];
                    if( assign.has_robot_on(g_pred) )
                    {
                        int pebble_id = assign.get_robot(g_pred);
                        if( graph->get_vertex_as<graph_pebble_node_t > (p)->transshipment )
                        {
                            robot_path3[0] = g_pred;
                            robot_path3[1] = p;
                            robot_path3[2] = v_prev;
                            assign.change_position(pebble_id, robot_path3.back());
                        }
                        else
                        {
                            robot_path2[0] = g_pred;
                            robot_path2[1] = p;
                            assign.change_position(pebble_id, robot_path2.back());
                        }
                    }
                    v_prev = p;
                }
            }

            void pebble_graph_solver_t::push_graph(std::vector<pebble_step_t>* solution, const undirected_graph_t* graph, undirected_vertex_index_t v, undirected_vertex_index_t u, pebble_assignment_t& assign, bool assignment_on_tree)
            {
                undirected_vertex_index_t g_pred;
                print_assignment(graph, assign);
                for( undirected_vertex_index_t p = u; p != v; p = graph->predecessors[p] )
                {

                    if( assignment_on_tree )
                        g_pred = to_tree[graph->predecessors[p]];
                    else
                        g_pred = graph->predecessors[p];

                    PRX_DEBUG_S("is a robot on " << print_point(graph, g_pred) << "  the robot is : " << assign.get_robot(g_pred));
                    if( assign.has_robot_on(g_pred) )
                    {
                        int pebble_id = assign.get_robot(g_pred);
                        robot_path2[0] = g_pred;
                        if( assignment_on_tree )
                            robot_path2[1] = to_tree[p];
                        else
                            robot_path2[1] = p;
                        assign.change_position(pebble_id, robot_path2.back());
                        solution->push_back(std::make_pair(pebble_id, robot_path2));
                        PRX_INFO_S("MOVE robot " << pebble_id << "  from " << print_point(graph, g_pred) << " -> " << print_point(graph, p) << "    or back : " << print_point(graph, robot_path2.back()));
                    }
                }
            }

            undirected_vertex_index_t pebble_graph_solver_t::is_tree_root_valid(std::vector<pebble_step_t>* solution, const undirected_graph_t* graph, const undirected_graph_t* tree, pebble_assignment_t& assign, undirected_vertex_index_t v, undirected_vertex_index_t w)
            {
                graph_pebble_node_t* node = tree->get_vertex_as<graph_pebble_node_t > (v);

                if( !node->transshipment )
                    return v;

                undirected_vertex_index_t t = node->get_empty_tree_except(w);
                if( assign.has_robot_on(t) )
                {
                    tree->get_vertex_as<graph_pebble_node_t > (v)->obstacle_id = obstacle_global_id;
                    push_pebble_once(solution, tree, t, assign);
                }
                return t;
            }

            bool pebble_graph_solver_t::swap(std::vector<pebble_step_t>* solution, const undirected_graph_t* tree, int pebble1, int pebble2, undirected_vertex_index_t w, int criterion_no)
            {
                undirected_vertex_index_t v;
                undirected_vertex_index_t u;
                int start_reverse, end_reverse;

                graph_pebble_node_t* w_node = tree->get_vertex_as<graph_pebble_node_t > (w);

                if( criterion_no == 4 || criterion_no == 6 )
                {
                    //        PRX_WARN_S("Swapped order");
                    int tmp = pebble2;
                    pebble2 = pebble1;
                    pebble1 = tmp;
                    //        v = s_new.get_position(pebble2);
                    //        u = s_new.get_position(pebble1);
                }

                //    PRX_ERROR_S("HAVE TO SWAP " << pebble1 << "(" << print_point(tree, s_new.get_position(pebble1)) << ")  -  " << pebble2 << "  (" << print_point(tree, s_new.get_position(pebble2)) << ")");

                v = s_new.get_position(pebble1);
                u = s_new.get_position(pebble2);

                graph_pebble_node_t* v_node = tree->get_vertex_as<graph_pebble_node_t > (v);

                undirected_vertex_index_t t1 = w_node->tree_for_vertex[v];
                undirected_vertex_index_t t2 = w_node->tree_for_vertex[u];
                undirected_vertex_index_t t3 = NULL;

                //    for( unsigned int s = last_solution_step; s < solution->size(); ++s )
                //    {
                //        std::cout << "pebble : " << solution->at(s).first << ":";
                //        for( unsigned int i = 0; i < solution->at(s).second.size(); ++i )
                //        {
                //            std::cout << " -> " << print_point(tree, solution->at(s).second[i]);
                //        }
                //        std::cout << std::endl;
                //    }
                last_solution_step = solution->size();
                PRX_INFO_S("SWAP : " << pebble1 << "  ->  " << pebble2 << "   pos: " << print_point(tree, v) << "  -  " << print_point(tree, u));
                PRX_DEBUG_S("CRITERION " << criterion_no);
                if( criterion_no == 1 )
                {
                    if( w_node->transshipment )
                    {
                        //            PRX_DEBUG_S(" w is transshipment");

                        start_reverse = solution->size();
                        if( t1 != v )
                            if( !move(solution, tree, pebble1, v, t1, s_new) )
                                return false;
                        if( t2 != u )
                            if( !move(solution, tree, pebble2, u, t2, s_new) )
                                return false;
                        obstacle_global_id++;
                        tree->get_vertex_as<graph_pebble_node_t > (t1)->obstacle_id = obstacle_global_id;
                        tree->get_vertex_as<graph_pebble_node_t > (t2)->obstacle_id = obstacle_global_id;
                        initial_graph->get_vertex_as<graph_pebble_node_t > (to_graph[t1])->obstacle_id = obstacle_global_id;
                        initial_graph->get_vertex_as<graph_pebble_node_t > (to_graph[t2])->obstacle_id = obstacle_global_id;

                        t3 = find_closest_useful_vertex(solution, initial_graph, tree, s_new, t1, t2, transshipments_class[w]);
                        //            PRX_INFO_S("t1: " << print_point(tree, t1) << "   t2: " << print_point(tree, t2) << "   t3: " << print_point(tree, t3));

                        t1 = is_tree_root_valid(solution, initial_graph, tree, s_new, t1, w);
                        t2 = is_tree_root_valid(solution, initial_graph, tree, s_new, t2, w);
                        t3 = is_tree_root_valid(solution, initial_graph, tree, s_new, t3, w);
                        end_reverse = solution->size() - 1;
                        if( t3 == NULL )
                            return false;


                        //            PRX_DEBUG_S("before the swap  " << pebble1 << " - " << pebble2 << " t1: " << print_point(tree, t1) << " t2: " << print_point(tree, t2) << " t3: " << print_point(tree, t3));
                        if( !perform_swap(solution, tree, pebble1, pebble2, t1, t2, t3) )
                            return false;
                        //            PRX_DEBUG_S("Did the swap " << pebble1 << " - " << pebble2);
                        revert(solution, start_reverse, end_reverse, pebble1, pebble2, s_new);
                        PRX_DEBUG_S("validation : " << solution_validation(tree, solution));
                        last_solution_check = solution->size();
                        return true;
                    }
                    else
                    {

                        foreach(undirected_vertex_index_t neigh, boost::adjacent_vertices(w, tree->graph))
                        {
                            //            PRX_INFO_S("neigh : " << print_point(tree,neigh));
                            if( neigh != t1 && neigh != t2 )
                            {
                                if( !s_new.has_robot_on(neigh) )
                                {
                                    //                    PRX_INFO_S("from v: " << print_point(tree,v) << " - >  " << print_point(tree,u) << "   with w:" << print_point(tree,w));
                                    return perform_swap(solution, tree, pebble1, pebble2, v, u, neigh);
                                }
                                if( t3 == NULL && w_node->trees[neigh].holes > 0 )
                                    t3 = neigh;
                            }
                        }
                    }

                    PRX_ASSERT(t3 != NULL);
                    obstacle_global_id++;
                    tree->get_vertex_as<tree_pebble_node_t > (w)->obstacle_id = obstacle_global_id;

                    start_reverse = solution->size();


                    if( !push_pebble_once(solution, tree, t3, s_new) )
                        return false;

                    v = is_tree_root_valid(solution, initial_graph, tree, s_new, v, w);
                    u = is_tree_root_valid(solution, initial_graph, tree, s_new, u, w);
                    t3 = is_tree_root_valid(solution, initial_graph, tree, s_new, t3, w);
                    end_reverse = solution->size() - 1;

                    if( !perform_swap(solution, tree, pebble1, pebble2, v, u, t3) )
                        return false;

                    revert(solution, start_reverse, end_reverse, pebble1, pebble2, s_new);

                    PRX_DEBUG_S("validation : " << solution_validation(tree, solution));
                    last_solution_check = solution->size();
                    return true;


                }
                else if( criterion_no == 2 )
                {
                    undirected_vertex_index_t v_empty;
                    start_reverse = solution->size();
                    obstacle_global_id++;
                    if( w_node->transshipment )
                    {
                        PRX_ASSERT(!s_new.has_robot_on(w));
                        //            PRX_DEBUG_S(" w is transshipment");

                        if( v == t1 )
                        {
                            //                PRX_DEBUG_S("v == t1");
                            undirected_vertex_index_t v_t1 = v_node->get_empty_tree_except(v_node->tree_for_vertex[u]);
                            if( tree->get_vertex_as<graph_pebble_node_t > (v_t1)->transshipment )
                            {
                                v_t1 = find_closest_useful_vertex(solution, initial_graph, tree, s_new, t1, t1, transshipments_class[v_t1]);
                            }
                            if( s_new.has_robot_on(v_t1) )
                            {
                                int class_id = initial_graph->edge_components[boost::edge(to_graph[v], to_graph[v_t1], tree->graph).first];
                                v_empty = find_closest_useful_vertex(solution, initial_graph, tree, s_new, v_t1, t1, class_id);
                                push_tree(solution, tree, v_t1, v_empty, s_new);
                            }

                            //                PRX_DEBUG_S("t1: " << print_point(tree, t1) << "    v_t1:" << print_point(tree, v_t1));
                            if( !move(solution, tree, pebble1, t1, v_t1, s_new) )
                                return false;

                            tree->get_vertex_as<tree_pebble_node_t > (v_t1)->obstacle_id = obstacle_global_id;
                            initial_graph->get_vertex_as<graph_pebble_node_t > (to_graph[v_t1])->obstacle_id = obstacle_global_id;
                        }

                        if( !move(solution, tree, pebble2, u, t1, s_new) )
                            return false;

                        tree->get_vertex_as<tree_pebble_node_t > (t1)->obstacle_id = obstacle_global_id;
                        initial_graph->get_vertex_as<graph_pebble_node_t > (to_graph[t1])->obstacle_id = obstacle_global_id;

                        t3 = to_tree[find_closest_on_graph_with(initial_graph, true, to_graph[t1], s_new)];
                        //            PRX_DEBUG_S("T3 will be : " << print_point(tree,t3));
                        int blocker_pebble = s_new.get_robot(t3);
                        //            PRX_DEBUG_S("blocker id  : " << blocker_pebble);

                        if( !move(solution, tree, blocker_pebble, t3, t2, s_new) )
                            return false;
                        tree->get_vertex_as<tree_pebble_node_t > (t3)->obstacle_id = obstacle_global_id;
                        initial_graph->get_vertex_as<graph_pebble_node_t > (to_graph[t3])->obstacle_id = obstacle_global_id;

                        v_empty = find_closest_on_graph_with(initial_graph, false, to_graph[t2], s_new);

                        push_graph(solution, initial_graph, to_graph[t2], v_empty, s_new, true);

                        t1 = is_tree_root_valid(solution, initial_graph, tree, s_new, t1, w);
                        t2 = is_tree_root_valid(solution, initial_graph, tree, s_new, t2, w);
                        t3 = is_tree_root_valid(solution, initial_graph, tree, s_new, t3, w);
                        end_reverse = solution->size() - 1;
                        if( !perform_swap2(solution, tree, pebble2, pebble1, s_new.get_position(pebble2), s_new.get_position(pebble1), t3, t2) )
                            return false;
                        revert(solution, start_reverse, end_reverse, pebble1, pebble2, s_new);
                        PRX_DEBUG_S("validation : " << solution_validation(tree, solution));
                        last_solution_check = solution->size();
                        return true;
                    }
                    else
                    {

                        foreach(undirected_vertex_index_t neigh, boost::adjacent_vertices(w, tree->graph))
                        {
                            if( neigh != t1 && neigh != t2 )
                            {
                                int blocker_pebble = s_new.get_robot(neigh);

                                tree->get_vertex_as<tree_pebble_node_t > (w)->obstacle_id = obstacle_global_id;

                                if( u == t2 )
                                {
                                    //                    PRX_DEBUG_S("PEBBLE IS ON THE T2");
                                    if( !push_pebble_once(solution, tree, u, s_new) )
                                        return false;
                                    u = s_new.get_position(pebble2);
                                }
                                if( t2 != v )
                                    if( !move(solution, tree, pebble1, v, t2, s_new) )
                                        return false;

                                if( t1 != neigh )
                                    if( !move(solution, tree, blocker_pebble, neigh, t1, s_new) )
                                        return false;
                                if( !push_pebble_once(solution, tree, t1, s_new) )
                                    return false;

                                t1 = is_tree_root_valid(solution, initial_graph, tree, s_new, t1, w);
                                t2 = is_tree_root_valid(solution, initial_graph, tree, s_new, t2, w);
                                neigh = is_tree_root_valid(solution, initial_graph, tree, s_new, neigh, w);
                                end_reverse = solution->size() - 1;
                                if( !perform_swap2(solution, tree, pebble1, pebble2, t2, u, neigh, t1) )
                                    return false;
                                revert(solution, start_reverse, end_reverse, pebble1, pebble2, s_new);
                                PRX_DEBUG_S("validation : " << solution_validation(tree, solution));
                                last_solution_check = solution->size();
                                return true;
                            }
                        }
                    }

                }
                else if( criterion_no == 3 || criterion_no == 4 )
                {

                    start_reverse = solution->size();
                    obstacle_global_id++;

                    if( w_node->transshipment )
                    {
                        PRX_DEBUG_S(" w is transshipment");
                        PRX_ASSERT(!s_new.has_robot_on(w));
                        undirected_vertex_index_t v_prev = v_node->tree_for_vertex[u];
                        PRX_DEBUG_S("v_prev : " << print_point(tree, v_prev));
                        if( tree->get_vertex_as<graph_pebble_node_t > (v_prev)->transshipment )
                        {
                            v_prev = tree->get_vertex_as<tree_pebble_node_t > (v_prev)->tree_for_vertex[u];
                            PRX_DEBUG_S("v_prev : " << print_point(tree, v_prev));
                        }
                        PRX_DEBUG_S("v_prev : " << print_point(tree, v_prev));
                        PRX_DEBUG_S("u : " << print_point(tree, u));
                        PRX_DEBUG_S("v : " << print_point(tree, v));
                        if( u != v_prev )
                            if( !move(solution, tree, pebble2, u, v_prev, s_new) )
                                return false;
                        tree->get_vertex_as<tree_pebble_node_t > (v_prev)->obstacle_id = obstacle_global_id;
                        initial_graph->get_vertex_as<graph_pebble_node_t > (to_graph[v_prev])->obstacle_id = obstacle_global_id;

                        //Although its criterion 3/4 it could be that t1 is a cut vertex between 2 biconnected components 
                        //which means that t2 can be a transshipment vertex that it failed criterion 1. This means that 
                        //its going to be only one possible path through this transshipment vertex.
                        undirected_vertex_index_t v_t2 = tree->get_vertex_as<graph_pebble_node_t > (t1)->tree_for_vertex[u];
                        //            PRX_DEBUG_S("first v_t2 : " << print_point(tree, v_t2));
                        if( tree->get_vertex_as<graph_pebble_node_t > (v_t2)->transshipment )
                        {
                            v_t2 = find_closest_useful_vertex(solution, initial_graph, tree, s_new, t1, t1, transshipments_class[v_t2]);
                        }

                        PRX_DEBUG_S("v_t2 : " << print_point(tree, v_t2));
                        PRX_DEBUG_S("t1 : " << print_point(tree, t1));

                        undirected_vertex_index_t v_pebble1 = v;
                        undirected_vertex_index_t v_empty;
                        while( v_pebble1 != t1 )
                        {
                            PRX_DEBUG_S("t1: " << print_point(tree, t1) << "   v_pebble1: " << print_point(tree, v_pebble1));

                            if( s_new.has_robot_on(t1) )
                            {
                                v_empty = find_closest_useful_vertex(solution, initial_graph, tree, s_new, t1, v_t2, transshipments_class[w]);
                                push_tree(solution, tree, t1, v_empty, s_new);
                            }

                            PRX_DEBUG_S("going to push " << s_new.get_robot(v_pebble1) << "   or : " << pebble1);
                            if( !push_pebble_once(solution, tree, v_pebble1, s_new) )
                                return false;

                            PRX_DEBUG_S("going to move " << pebble2);
                            if( !move(solution, tree, pebble2, v_prev, v_pebble1, s_new) )
                                return false;

                            tree->get_vertex_as<tree_pebble_node_t > (v_prev)->obstacle_id--;
                            initial_graph->get_vertex_as<graph_pebble_node_t > (to_graph[v_prev])->obstacle_id--;
                            v_prev = v_pebble1;
                            tree->get_vertex_as<tree_pebble_node_t > (v_prev)->obstacle_id = obstacle_global_id;
                            initial_graph->get_vertex_as<graph_pebble_node_t > (to_graph[v_prev])->obstacle_id = obstacle_global_id;
                            v_pebble1 = s_new.get_position(pebble1);
                        }

                        PRX_INFO_S("solution : ");
                        for( unsigned int s = last_solution_step; s < solution->size(); ++s )
                        {
                            std::cout << "pebble : " << solution->at(s).first << ":";
                            for( unsigned int i = 0; i < solution->at(s).second.size(); ++i )
                            {
                                std::cout << " -> " << print_point(tree, solution->at(s).second[i]);
                            }
                            std::cout << std::endl;
                        }
                        print_assignment(tree, s_new);

                        tree->get_vertex_as<tree_pebble_node_t > (t1)->obstacle_id = obstacle_global_id;
                        initial_graph->get_vertex_as<graph_pebble_node_t > (to_graph[t1])->obstacle_id = obstacle_global_id;
                        t2 = find_closest_useful_vertex(solution, initial_graph, tree, s_new, t1, t1, transshipments_class[w]);
                        //            PRX_DEBUG_S("t2 : " << print_point(tree, t2));
                        tree->get_vertex_as<tree_pebble_node_t > (t2)->obstacle_id = obstacle_global_id;
                        initial_graph->get_vertex_as<graph_pebble_node_t > (to_graph[t2])->obstacle_id = obstacle_global_id;
                        t3 = find_closest_useful_vertex(solution, initial_graph, tree, s_new, t1, t2, transshipments_class[w]);
                        //            PRX_WARN_S("solution AFTER : ");
                        //            for( unsigned int s = last_solution_step; s < solution->size(); ++s )
                        //            {
                        //                std::cout << "pebble : " << solution->at(s).first << ":";
                        //                for( unsigned int i = 0; i < solution->at(s).second.size(); ++i )
                        //                {
                        //                    std::cout << " -> " << print_point(tree, solution->at(s).second[i]);
                        //                }
                        //                std::cout << std::endl;
                        //            }
                        //            print_assignment(tree,s_new);
                        if( t3 == NULL )
                        {
                            //                PRX_WARN_S("its still null");
                            t3 = to_tree[find_closest_on_graph_with(initial_graph, true, to_graph[t1], s_new)];
                            //            PRX_DEBUG_S("T3 will be : " << print_point(tree,t3));
                            int blocker_pebble = s_new.get_robot(t3);
                            //            PRX_DEBUG_S("blocker id  : " << blocker_pebble);

                            if( !move(solution, tree, blocker_pebble, t3, t2, s_new) )
                                return false;
                            tree->get_vertex_as<tree_pebble_node_t > (t3)->obstacle_id = obstacle_global_id;
                            initial_graph->get_vertex_as<graph_pebble_node_t > (to_graph[t3])->obstacle_id = obstacle_global_id;

                            v_empty = find_closest_on_graph_with(initial_graph, false, to_graph[t2], s_new);

                            push_graph(solution, initial_graph, to_graph[t2], v_empty, s_new, true);
                        }
                        //            PRX_DEBUG_S("t3 : " << print_point(tree, t3));

                    }
                    else
                    {
                        undirected_vertex_index_t t_pebble1 = v_node->tree_for_vertex[w];

                        foreach(undirected_vertex_index_t v_neigh, boost::adjacent_vertices(v, tree->graph))
                        {
                            if( v_neigh != t_pebble1 )
                            {
                                tree->get_vertex_as<tree_pebble_node_t > (v_neigh)->obstacle_id = obstacle_global_id;

                                //                PRX_DEBUG_S("obstacle : " << print_point(tree,v_neigh));
                            }
                        }
                        undirected_vertex_index_t v_pebble1 = s_new.get_position(pebble1);
                        while( v_pebble1 != t1 )
                        {
                            //            PRX_DEBUG_S(pebble1 << " on : " << print_point(tree,v_pebble1) << "    t1 : " << print_point(tree,t1));
                            if( !push_pebble_once(solution, tree, v_pebble1, s_new) )
                                return false;
                            tree->get_vertex_as<tree_pebble_node_t > (v_pebble1)->obstacle_id = obstacle_global_id;
                            v_pebble1 = s_new.get_position(pebble1);
                        }

                        if( s_new.has_robot_on(w) )
                            if( !push_pebble_once(solution, tree, w, s_new) )
                                return false;

                        obstacle_global_id++;

                        t2 = w_node->find_empty_tree_except(s_new, t1);
                        //        PRX_ERROR_S("t2: " << t2 << "    " << print_point(tree,t2));
                        t3 = w_node->find_empty_tree_except(s_new, t1, t2);
                        //        PRX_ERROR_S("t3: " << t3);
                        if( t3 == NULL )
                        {
                            t3 = w_node->find_full_tree_except(s_new, t1, t2);
                            //            PRX_ERROR_S("t3: " << t3 << "     " << print_point(tree,t3));

                            if( s_new.has_robot_on(t2) )
                            {
                                tree->get_vertex_as<tree_pebble_node_t > (w)->obstacle_id = obstacle_global_id;
                                if( !push_pebble_once(solution, tree, t2, s_new) )
                                    return false;
                            }
                            if( !move(solution, tree, s_new.get_robot(t3), t3, t2, s_new) )
                                return false;

                        }
                        else
                        {
                            //            PRX_WARN_S("t3 : " << print_point(tree,t3));
                            tree->get_vertex_as<tree_pebble_node_t > (w)->obstacle_id = obstacle_global_id;
                            if( s_new.has_robot_on(t3) )
                                if( !push_pebble_once(solution, tree, t3, s_new) )
                                    return false;
                        }

                        //        PRX_DEBUG_S("going to push and the t2 : " << print_point(tree,t2) << "    with robot : " << s_new.get_robot(t2));
                        tree->get_vertex_as<tree_pebble_node_t > (w)->obstacle_id = obstacle_global_id;
                        if( s_new.has_robot_on(t2) )
                            if( !push_pebble_once(solution, tree, t2, s_new) )
                                return false;
                    }
                    t1 = is_tree_root_valid(solution, initial_graph, tree, s_new, t1, w);
                    t2 = is_tree_root_valid(solution, initial_graph, tree, s_new, t2, w);
                    t3 = is_tree_root_valid(solution, initial_graph, tree, s_new, t3, w);
                    end_reverse = solution->size() - 1;

                    if( !perform_swap2(solution, tree, pebble1, pebble2, s_new.get_position(pebble1), s_new.get_position(pebble2), t3, t2) )
                        return false;
                    revert(solution, start_reverse, end_reverse, pebble1, pebble2, s_new);
                    PRX_DEBUG_S("validation : " << solution_validation(tree, solution));
                    last_solution_check = solution->size();
                    return true;


                }
                else if( criterion_no == 5 || criterion_no == 6 )
                {
                    start_reverse = solution->size();
                    obstacle_global_id++;
                    t1 = w_node->get_empty_tree_except(t2);
                    //        if(tree->get_vertex_as<graph_pebble_node_t > (t1)->transshipment)
                    //        {
                    //            t1 = find_closest_useful_vertex(solution, initial_graph, tree, s_new, t1, t1, transshipments_class[t1]);
                    //        }
                    t3 = w_node->get_empty_tree_except(t1, t2);

                    //        PRX_INFO_S("t3: " << t3);
                    if( t3 == NULL )
                    {
                        tree->get_vertex_as<tree_pebble_node_t > (t1)->obstacle_id = obstacle_global_id;
                        if( !push_pebble_once(solution, tree, w, s_new) )
                            return false;

                        obstacle_global_id++;
                        tree->get_vertex_as<tree_pebble_node_t > (t2)->obstacle_id = obstacle_global_id;
                        t3 = w_node->get_any_full_tree();
                        //            PRX_INFO_S("full t3: " << t3 << "     " << print_point(tree,t3));

                        if( !move(solution, tree, s_new.get_robot(t3), t3, w, s_new) )
                            return false;

                        tree->get_vertex_as<tree_pebble_node_t > (t3)->obstacle_id = obstacle_global_id;
                        if( !push_pebble_once(solution, tree, w, s_new) )
                            return false;
                    }
                    else
                    {
                        //            PRX_WARN_S("t3 : " << print_point(tree,t3));
                        tree->get_vertex_as<tree_pebble_node_t > (w)->obstacle_id = obstacle_global_id;
                        if( s_new.has_robot_on(t3) )
                            if( !push_pebble_once(solution, tree, t3, s_new) )
                                return false;
                    }

                    //        PRX_DEBUG_S("going to push and the t1 : " << print_point(tree,t1));
                    tree->get_vertex_as<tree_pebble_node_t > (w)->obstacle_id = obstacle_global_id;
                    if( s_new.has_robot_on(t1) )
                        if( !push_pebble_once(solution, tree, t1, s_new) )
                            return false;

                    t1 = is_tree_root_valid(solution, initial_graph, tree, s_new, t1, w);
                    t2 = is_tree_root_valid(solution, initial_graph, tree, s_new, t2, w);
                    t3 = is_tree_root_valid(solution, initial_graph, tree, s_new, t3, w);

                    end_reverse = solution->size() - 1;

                    obstacle_global_id++;
                    if( !perform_swap2(solution, tree, pebble1, pebble2, s_new.get_position(pebble1), s_new.get_position(pebble2), t3, t1) )
                        return false;
                    revert(solution, start_reverse, end_reverse, pebble1, pebble2, s_new);
                    PRX_DEBUG_S("validation : " << solution_validation(tree, solution));
                    last_solution_check = solution->size();
                    return true;
                }
                else
                {
                    PRX_LOG_ERROR("Criterion %d does not exist!", criterion_no);
                }
                return false;
            }

            undirected_vertex_index_t pebble_graph_solver_t::rotate(std::vector<pebble_step_t>* solution, pebble_assignment_t& assign, undirected_vertex_index_t v, undirected_vertex_index_t u, undirected_vertex_index_t empty)
            {
                int dist = 0;
                int direction = 0;

                std::deque<undirected_vertex_index_t>::iterator it_v = std::find(mcbc.begin(), mcbc.begin() + mcbc_index, v);
                std::deque<undirected_vertex_index_t>::iterator it_u = std::find(mcbc.begin(), mcbc.begin() + mcbc_index, u);
                std::deque<undirected_vertex_index_t>::iterator it = std::find(mcbc.begin(), mcbc.begin() + mcbc_index, empty);
                std::deque<undirected_vertex_index_t>::iterator it_prev;

                dist = std::distance(it_v, it_u);
                if( fabs(dist) > mcbc_index / 2 )
                {
                    direction = -PRX_SIGN(dist);
                    dist = mcbc_index - fabs(dist);
                }
                else
                    direction = PRX_SIGN(dist);

                //    PRX_DEBUG_S("dist: " << dist << "    direction : " << direction);


                //    print_assignment(graph,assign);
                //    PRX_DEBUG_S("Empty is (" << *it << "):" << print_point(initial_graph, *it));
                PRX_DEBUG_S("Rotation dist : " << fabs(dist));
                //    int count = 0;
                for( int i = 0; i < fabs(dist); ++i )
                {
                    //        PRX_WARN_S("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ Iteration : " << i);
                    for( int j = 0; j < mcbc_index - 1; ++j )
                    {
                        it_prev = adjust_iterator(it - direction, mcbc, mcbc_index);
                        int pebble_id = assign.get_robot(*it_prev);
                        //            PRX_DEBUG_S("Going to move pebble: " << pebble_id << "  :  " << print_point(initial_graph, *it_prev) << "  ->  " << print_point(initial_graph, *it));
                        if( pebble_id != -1 )
                        {
                            //                ++count;
                            robot_path2[0] = *it_prev;
                            robot_path2[1] = *it;
                            assign.change_position(pebble_id, robot_path2[1]);
                            //                PRX_INFO_S("Pebble: " << pebble_id << "    MOVED :  " << print_point(initial_graph, robot_path2[0]) << "  ->  " << print_point(initial_graph, robot_path2[1]));
                            solution->push_back(std::make_pair(pebble_id, robot_path2));
                        }
                        it = it_prev;
                    }
                    //        PRX_DEBUG_S("-------------------------------- count : " << count);
                    //        count = 0;
                }

                PRX_ASSERT(!assign.has_robot_on(*it));
                return *it;
            }

            void pebble_graph_solver_t::move_once(std::vector<pebble_step_t>* solution, pebble_assignment_t& assign, undirected_vertex_index_t v, undirected_vertex_index_t u)
            {
                robot_path2[0] = v;
                robot_path2[1] = u;
                //    PRX_INFO_S("agent " << assign.get_robot(v) << "   goes :  " << print_point(initial_graph, v) << "  ->  " << print_point(initial_graph, u));
                solution->push_back(std::make_pair(assign.get_robot(v), robot_path2));
                assign.change_position(v, u);
            }

            bool pebble_graph_solver_t::pass_through(std::vector<pebble_step_t>* solution, pebble_assignment_t& assign, undirected_vertex_index_t v_adj, undirected_vertex_index_t v_cut, undirected_vertex_index_t u)
            {
                //    PRX_ERROR_S("ROTATE 1");
                rotate(solution, assign, u, v_cut, u);
                //    PRX_ERROR_S("MOVE ");
                move_once(solution, assign, v_adj, v_cut);
                //    PRX_ERROR_S("ROTATE 2");
                rotate(solution, assign, v_cut, u, u);
                return true;
            }

            bool pebble_graph_solver_t::biconnected_swap(std::vector<pebble_step_t>* solution, pebble_assignment_t& assign, undirected_vertex_index_t v_adj, undirected_vertex_index_t v_cut, undirected_vertex_index_t v, undirected_vertex_index_t u)
            {

                PRX_INFO_S("THE ROTATION IS for pebble " << assign.get_robot(v) << "    from : " << print_point(initial_graph, v) << " -> " << print_point(initial_graph, u));
                undirected_vertex_index_t v_empty = u;
                PRX_DEBUG_S("Pebble on v_adj : " << assign.get_robot(v_adj));
                PRX_INFO_S("v_empty : " << print_point(initial_graph, v_empty) << "   is empty : " << !assign.has_robot_on(v_empty));
                PRX_ERROR_S("ROTATE 1 : " << print_point(initial_graph, v) << " - > " << print_point(initial_graph, v_cut) << "    empty: " << print_point(initial_graph, v_empty));
                v_empty = rotate(solution, assign, v, v_cut, v_empty);
                PRX_INFO_S("sim_assignment size:  " << sim_assignment.size() << "    s_initial.size(): " << s_initial.size());
                PRX_ASSERT(!assign.has_robot_on(v_empty));
                PRX_INFO_S("v_empty : " << print_point(initial_graph, v_empty) << "   is empty : " << !assign.has_robot_on(v_empty));
                PRX_DEBUG_S("Before move Pebble on  v_adj: " << print_point(initial_graph, v_adj) << "  is : " << assign.get_robot(v_adj));
                PRX_DEBUG_S("Pebble on v_cut: " << print_point(initial_graph, v_cut) << "  is : " << assign.get_robot(v_cut));
                PRX_DEBUG_S("MOVE pebble " << assign.get_robot(v_cut) << "   from: " << print_point(initial_graph, v_cut) << " - > " << print_point(initial_graph, v_adj) << "    empty: " << print_point(initial_graph, v_empty));
                move_once(solution, assign, v_cut, v_adj);
                PRX_INFO_S("sim_assignment size:  " << sim_assignment.size() << "    s_initial.size(): " << s_initial.size());
                PRX_INFO_S("v_empty : " << print_point(initial_graph, v_empty));
                PRX_DEBUG_S("AFTER move Pebble on v_cut: " << print_point(initial_graph, v_cut) << "  is : " << assign.get_robot(v_cut));
                PRX_DEBUG_S("Pebble on  v_adj: " << print_point(initial_graph, v_adj) << "  is : " << assign.get_robot(v_adj));
                PRX_ERROR_S("ROTATE 2 : " << print_point(initial_graph, v_cut) << " - > " << print_point(initial_graph, v) << "    empty: " << print_point(initial_graph, v_empty));
                v_empty = rotate(solution, assign, v_cut, v, v_empty);
                PRX_INFO_S("sim_assignment size:  " << sim_assignment.size() << "    s_initial.size(): " << s_initial.size());
                PRX_ASSERT(!assign.has_robot_on(v_empty));

                v_empty = v;
                PRX_INFO_S("v_empty : " << print_point(initial_graph, v_empty) << "   is empty : " << !assign.has_robot_on(v_empty));
                PRX_ASSERT(!assign.has_robot_on(v_empty));
                PRX_ERROR_S("ROTATE 1 : " << print_point(initial_graph, u) << " - > " << print_point(initial_graph, v_cut) << "    empty: " << print_point(initial_graph, v_empty));
                v_empty = rotate(solution, assign, u, v_cut, v_empty);
                PRX_INFO_S("sim_assignment size:  " << sim_assignment.size() << "    s_initial.size(): " << s_initial.size());
                PRX_ASSERT(!assign.has_robot_on(v_empty));
                PRX_INFO_S("v_empty : " << print_point(initial_graph, v_empty) << "   is empty : " << !assign.has_robot_on(v_empty));
                PRX_DEBUG_S("Before move Pebble on  v_cut: " << print_point(initial_graph, v_cut) << "  is : " << assign.get_robot(v_cut));
                PRX_DEBUG_S("Pebble on  v_adj: " << print_point(initial_graph, v_adj) << "  is : " << assign.get_robot(v_adj));
                PRX_DEBUG_S("MOVE pebble " << assign.get_robot(v_adj) << "    from: " << print_point(initial_graph, v_adj) << " - > " << print_point(initial_graph, v_cut) << "    empty: " << print_point(initial_graph, v_empty));
                move_once(solution, assign, v_adj, v_cut);
                PRX_INFO_S("sim_assignment size:  " << sim_assignment.size() << "    s_initial.size(): " << s_initial.size());
                PRX_ASSERT(!assign.has_robot_on(v_empty));
                PRX_INFO_S("v_empty : " << print_point(initial_graph, v_empty) << "   is empty : " << !assign.has_robot_on(v_empty));
                PRX_ERROR_S("ROTATE 2 : " << print_point(initial_graph, v_cut) << " - > " << print_point(initial_graph, u) << "    empty: " << print_point(initial_graph, v_empty));
                PRX_INFO_S("sim_assignment size:  " << sim_assignment.size() << "    s_initial.size(): " << s_initial.size());

                PRX_DEBUG_S("u: " << print_point(initial_graph, u));
                PRX_DEBUG_S("Empty vertex : " << print_point(initial_graph, v_empty));
                v_empty = rotate(solution, assign, v_cut, u, v_empty);
                PRX_INFO_S("sim_assignment size:  " << sim_assignment.size() << "    s_initial.size(): " << s_initial.size());
                PRX_ASSERT(v_empty == v);
                PRX_ASSERT(!assign.has_robot_on(v_empty));

                return true;

            }

            undirected_vertex_index_t pebble_graph_solver_t::get_adjacent_except(const undirected_graph_t* graph, undirected_vertex_index_t v, undirected_vertex_index_t avoid)
            {

                foreach(undirected_vertex_index_t adj, boost::adjacent_vertices(v, graph->graph))
                {
                    if( adj != avoid )
                    {
                        return adj;
                    }
                }

                return NULL;
            }

            bool pebble_graph_solver_t::solution_validation(const undirected_graph_t* tree, std::vector<pebble_step_t>* solution)
            {
                PRX_WARN_S("solution new  size: " << solution->size() - last_solution_check);
                undirected_vertex_index_t v_prev;
                bool is_path_valid = true;

                for( int s = last_solution_check; s < (int)solution->size(); ++s )
                {
                    v_prev = solution->at(s).second[0];
                    for( int i = 1; i < (int)solution->at(s).second.size(); ++i )
                    {
                        if( !boost::edge(v_prev, solution->at(s).second[i], tree->graph).second )
                        {
                            PRX_ERROR_S("ERROR edge(" << i << "): " << print_point(tree, v_prev) << " -> " << print_point(tree, solution->at(s).second[i]));
                            is_path_valid = false;
                        }
                        v_prev = solution->at(s).second[i];
                    }

                }
                return is_path_valid;
            }

            bool pebble_graph_solver_t::all_solution_validation(const undirected_graph_t* tree, std::vector<pebble_step_t>* solution, const pebble_assignment_t assignment, bool print_results)
            {
                //    return true;
                pebble_assignment_t assign = assignment;
                //    PRX_WARN_S("solution new  size: " << solution->size() - last_solution_check);
                undirected_vertex_index_t v_prev, v_new, v_assign;
                bool is_path_valid = true;
                //
                //    PRX_DEBUG_S("assignment at the beginning");
                //    print_assignment(tree,assignment);
                //    PRX_DEBUG_S("assign at the beginning");
                //    print_assignment(tree,assign);
                for( int s = 0; s < (int)solution->size(); ++s )
                {
                    int pebble_id = solution->at(s).first;
                    v_assign = assign.get_position(pebble_id);
                    v_prev = solution->at(s).second[0];

                    if( print_results )
                        PRX_DEBUG_S("pebble " << pebble_id << "   moving from : " << print_point(tree, v_prev) << " -> " << print_point(tree, solution->at(s).second.back()) << "  steps: " << solution->at(s).second.size());
                    if( v_assign != v_prev )
                    {
                        //            PRX_ERROR_S(s << ") pebble : " << solution->at(s).first << "  start: " << assign.get_position(solution->at(s).first) << "   thinks:" << v_prev);
                        PRX_ERROR_S(s << ") pebble : " << pebble_id << "  start: " << print_point(tree, v_assign) << "   thinks:" << print_point(tree, v_prev));
                    }
                    for( int i = 1; i < (int)solution->at(s).second.size(); ++i )
                    {
                        v_new = solution->at(s).second[i];
                        if( !boost::edge(v_prev, v_new, tree->graph).second )
                        {
                            PRX_ERROR_S("ERROR edge(" << i << "): " << print_point(tree, v_prev) << " -> " << print_point(tree, v_new));
                            is_path_valid = false;
                        }
                        v_prev = v_new;
                    }
                    assign.change_position(pebble_id, solution->at(s).second.back());

                }
                return is_path_valid;
            }

            bool pebble_graph_solver_t::path_validation(const undirected_graph_t* graph, std::vector<undirected_vertex_index_t> path)
            {

                //    foreach(undirected_vertex_index_t v, boost::vertices(graph->graph))
                //    {
                //        PRX_INFO_S(v << "  ->  " << print_point(graph,v));
                //    }

                PRX_WARN_S("p_path size: " << path.size());
                undirected_vertex_index_t v_prev = path[0];
                for( size_t i = 1; i < path.size(); ++i )
                {
                    //        PRX_DEBUG_S("Checking edge(" << i << "): " << print_point(graph, v_prev) << " -> " << print_point(graph, path[i]));
                    //                PRX_DEBUG_S(v_prev << " -> " << path[i]);
                    if( !boost::edge(v_prev, path[i], graph->graph).second )
                    {
                        PRX_ERROR_S("ERROR edge(" << i << "): " << print_point(graph, v_prev) << " -> " << print_point(graph, path[i]));
                        return false;
                    }
                    v_prev = path[i];
                }
                return true;
            }

            bool pebble_graph_solver_t::mcbc_validation(const undirected_graph_t * graph)
            {
                undirected_vertex_index_t v_prev;
                undirected_vertex_index_t v_curr;

                v_prev = mcbc[0];
                for( int i = 1; i < mcbc_index; ++i )
                {
                    v_curr = mcbc[i];
                    PRX_DEBUG_S("f_mcbc: " << print_point(graph, v_prev));
                    if( !boost::edge(v_prev, v_curr, graph->graph).second )
                    {
                        PRX_ERROR_S("edge: " << print_point(graph, v_prev) << " - " << print_point(graph, v_curr));
                        PRX_LOG_ERROR("mcbc is wrong edge: ");
                    }

                    v_prev = v_curr;
                }
                PRX_DEBUG_S("f_mcbc: " << print_point(graph, v_prev));
                if( !boost::edge(v_prev, mcbc[0], graph->graph).second )
                {
                    for( int i = 0; i < mcbc_index; ++i )
                    {
                        PRX_INFO_S(mcbc[i] << ")  " << print_point(graph, mcbc[i]));
                    }

                    PRX_ERROR_S("v_prev : " << print_point(graph, v_prev) << " - " << print_point(graph, mcbc[0]));
                    PRX_LOG_ERROR("mcbc is wrong not a full cycle");
                }

                return true;
            }

            //undirected_vertex_index_t pebble_graph_solver_t::is_cut_vertex(const undirected_graph_t* graph, undirected_vertex_index_t v, int b_class)
            //{
            //
            //    foreach(undirected_vertex_index_t u, boost::adjacent_vertices(v, graph->graph))
            //    {
            //        if( graph->edge_components[boost::edge(v, u, graph->graph).first] != b_class )
            //            return u;
            //    }
            //    return NULL;
            //}
            //
            //void pebble_graph_solver_t::compute_maximal_biconnected_components(const undirected_graph_t * graph)
            //{
            //    for( unsigned int i = 0; i < biconnected_comps.size(); ++i )
            //    {
            //        PRX_DEBUG_S(i << ")" << biconnected_comps[i].b_comps_size);
            //        if( biconnected_comps[i].b_comps_size > 1 )
            //        {
            //            PRX_INFO_S("Going for the biconnected component " << i << " with size " << biconnected_comps[i].b_comps_size);
            //            std::vector<undirected_vertex_index_t> maximal_graph;
            //            hash_t<undirected_vertex_index_t, undirected_vertex_index_t> cut_vertices;
            //            maximal_bc(maximal_graph, graph, biconnected_comps[i].biconnected_class, biconnected_comps[i].a_node);
            //
            //            foreach(undirected_vertex_index_t v, biconnected_comps[i].nodes | boost::adaptors::map_keys)
            //            {
            //                undirected_vertex_index_t u = is_cut_vertex(graph, v, biconnected_comps[i].biconnected_class);
            //                if( u != NULL )
            //                    cut_vertices[v] = u;
            //            }
            //            mbc[biconnected_comps[i].v_trans] = std::make_pair(maximal_graph, cut_vertices);
            //        }
            //    }
            //
            //    //    foreach(undirected_vertex_index_t v, mbc | boost::adaptors::map_keys)
            //    //    {
            //    //        PRX_ERROR_S("For the transshipment : " << print_point(graph, v));
            //    //
            //    //        foreach(undirected_vertex_index_t v_g, mbc[v].first)
            //    //        {
            //    //            PRX_INFO_S(print_point(graph, v_g));
            //    //        }
            //    //
            //    //        foreach(undirected_vertex_index_t v_g, mbc[v].second | boost::adaptors::map_keys)
            //    //        {
            //    //            PRX_DEBUG_S(print_point(graph, v_g));
            //    //        }
            //    //    }
            //}
            //
            //bool pebble_graph_solver_t::maximal_bc(std::vector<undirected_vertex_index_t>& mbc_graph, const undirected_graph_t* graph, int b_class, undirected_vertex_index_t v)
            //{
            //    //    PRX_DEBUG_S("maximal for (" << v << " ) : " << print_point(graph, v));
            //    //    foreach(undirected_vertex_index_t vv, boost::vertices(graph->graph))
            //    //    {
            //    //        PRX_DEBUG_S(vv << ") : " << print_point(graph,vv));
            //    //    }
            //    if( !biconnected_comps[b_class].nodes[v] )
            //    {
            //        mbc_graph.push_back(v);
            //        biconnected_comps[b_class].nodes[v] = true;
            //
            //        PRX_WARN_S(print_point(graph, v) << "  has neigh:  " << boost::degree(v, graph->graph));
            //
            //        foreach(undirected_vertex_index_t adj, boost::adjacent_vertices(v, graph->graph))
            //        {
            //            //                        PRX_DEBUG_S("adj is : (" << adj << ") : " << print_point(graph, adj));
            //            PRX_DEBUG_S(print_point(graph, adj) << "  has neigh:  " << boost::degree(adj, graph->graph) << "    mbc.size = " << mbc_graph.size());
            //            if( graph->edge_components[boost::edge(v, adj, graph->graph).first] == b_class && maximal_bc(mbc_graph, graph, b_class, adj) )
            //            {
            //                PRX_WARN_S("yeaaaah success for " << print_point(graph, v));
            //                return true;
            //
            //            }
            //            PRX_INFO_S("NO  class : " << graph->edge_components[boost::edge(v, adj, graph->graph).first]);
            //        }
            //        mbc_graph.pop_back();
            //        biconnected_comps[b_class].nodes[v] = false;
            //    }
            //    else if( (int)mbc_graph.size() == biconnected_comps[b_class].vertices_size )
            //        return true;
            //
            //    //    PRX_INFO_S("FALSE for the v  : " << print_point(graph, v));
            //    return false;
            //}
            //
            //void pebble_graph_solver_t::block_vertex(undirected_vertex_index_t graph_v, undirected_graph_t* graph, undirected_graph_t* tree)
            //{
            //    graph->get_vertex_as<graph_pebble_node_t > (graph_v)->obstacle_id = obstacle_global_id;
            //    tree->get_vertex_as<graph_pebble_node_t > (to_tree[graph_v])->obstacle_id = obstacle_global_id;
            //}
            //
            //std::pair<undirected_vertex_index_t, undirected_vertex_index_t> pebble_graph_solver_t::get_closest_v_cut(const undirected_graph_t* tree, pebble_assignment_t& assign, undirected_vertex_index_t start, undirected_vertex_index_t goal, std::vector<undirected_vertex_index_t>& mbc_graph, hash_t < undirected_vertex_index_t, undirected_vertex_index_t>& v_cuts)
            //{
            //    std::pair<undirected_vertex_index_t, undirected_vertex_index_t> v_cut_pair;
            //    undirected_vertex_index_t v_adj = NULL;
            //    double dist = PRX_INFINITY;
            //    double new_dist;
            //
            //    //    PRX_WARN_S("start for v_Cut : " << print_point(tree, start) << "   goal for v_Cut : " << print_point(tree, goal));    
            //
            //    std::vector<undirected_vertex_index_t>::iterator it_s = std::find(mbc_graph.begin(), mbc_graph.end(), start);
            //    std::vector<undirected_vertex_index_t>::iterator it_g = std::find(mbc_graph.begin(), mbc_graph.end(), goal);
            //
            //    std::vector<undirected_vertex_index_t>::iterator it = it_s;
            //
            //    for( unsigned int i = 0; i < mbc_graph.size(); ++i )
            //    {
            //        v_adj = v_cuts[*it];
            //        if( v_adj != NULL )
            //        {
            //            new_dist = fabs(std::distance(it_s, it)) + fabs(std::distance(it_g, it));
            //            if( dist > new_dist && (!assign.has_robot_on(v_adj) || tree->get_vertex_as<graph_pebble_node_t > (to_tree[*it])->trees[to_tree[v_adj]].holes > 0) )
            //            {
            //                v_cut_pair = std::make_pair(*it, v_adj);
            //                dist = new_dist;
            //            }
            //        }
            //
            //
            //        it = adjust_iterator(it + 1, mbc_graph);
            //    }
            //
            //    PRX_ASSERT(dist != PRX_INFINITY);
            //
            //    return v_cut_pair;
            //}
            //
            //std::pair<undirected_vertex_index_t, undirected_vertex_index_t> pebble_graph_solver_t::get_closest_v_cut(const undirected_graph_t* tree, pebble_assignment_t& assign, undirected_vertex_index_t start, std::vector<undirected_vertex_index_t>& mbc_graph, hash_t < undirected_vertex_index_t, undirected_vertex_index_t>& v_cuts)
            //{
            //    undirected_vertex_index_t v_cut = NULL;
            //    undirected_vertex_index_t v_adj = NULL;
            //
            //    //    PRX_WARN_S("start for v_Cut : " << print_point(tree, start));
            //
            //    std::vector<undirected_vertex_index_t>::iterator it = std::find(mbc_graph.begin(), mbc_graph.end(), start);
            //    v_adj = v_cuts[*it];
            //    if( v_adj != NULL )
            //    {
            //        if( !assign.has_robot_on(v_adj) )
            //        {
            //            return std::make_pair(*it, v_adj);
            //        }
            //        else if( tree->get_vertex_as<graph_pebble_node_t > (to_tree[*it])->trees[to_tree[v_adj]].holes > 0 )
            //        {
            //            v_cut = *it;
            //        }
            //    }
            //
            //    std::vector<undirected_vertex_index_t>::iterator it_f = it;
            //    std::vector<undirected_vertex_index_t>::iterator it_b = it;
            //
            //    for( unsigned int i = 0; i < mbc_graph.size() / 2; ++i )
            //    {
            //
            //        it_f = adjust_iterator(it_f + 1, mbc_graph);
            //        it_b = adjust_iterator(it_b - 1, mbc_graph);
            //
            //        v_adj = v_cuts[*it_f];
            //        if( v_adj != NULL )
            //        {
            //            //            PRX_DEBUG_S("Forward : " << print_point(tree, *it_f) << "    with v_cut : " << print_point(tree, v_cuts[*it_f]) << "   and holes: " << tree->get_vertex_as<graph_pebble_node_t > (to_tree[*it_f])->trees[to_tree[v_cuts[*it_f]]].holes);
            //            if( !assign.has_robot_on(v_adj) )
            //            {
            //                return std::make_pair(*it_f, v_adj);
            //            }
            //            else if( tree->get_vertex_as<graph_pebble_node_t > (to_tree[*it_f])->trees[to_tree[v_adj]].holes > 0 )
            //            {
            //                v_cut = *it_f;
            //            }
            //        }
            //
            //        v_adj = v_cuts[*it_b];
            //        if( v_adj != NULL )
            //        {
            //            //            PRX_DEBUG_S("Backward : " << print_point(tree, *it_b) << "    with v_cut : " << print_point(tree, v_cuts[*it_b]) << "   and holes: " << tree->get_vertex_as<graph_pebble_node_t > (to_tree[*it_b])->trees[to_tree[v_cuts[*it_b]]].holes);
            //            if( !assign.has_robot_on(v_adj) )
            //            {
            //                return std::make_pair(*it_b, v_adj);
            //            }
            //            else if( tree->get_vertex_as<graph_pebble_node_t > (to_tree[*it_b])->trees[to_tree[v_adj]].holes > 0 )
            //            {
            //                v_cut = *it_b;
            //            }
            //        }
            //    }
            //
            //    if( v_cut != NULL )
            //        return std::make_pair(v_cut, v_cuts[v_cut]);
            //
            //    undirected_vertex_index_t v = NULL;
            //    undirected_vertex_index_t u = NULL;
            //    return std::make_pair(v, u);
            //}

        }
    }
}
