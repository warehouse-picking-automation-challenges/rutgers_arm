/**
 * @file pebble_tree_solver.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2012, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file pebble_idd LICENSE.
 *
 * @authors Athanasios Krontiris, Andrew Dobson, Andrew Kimmel, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "utilities/pebble_tree_solver/pebble_tree_solver.hpp"
#include "utilities/tree_feasibility_tester/tree_pebble_graph.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/graph/subgraph.hpp>

PLUGINLIB_EXPORT_CLASS( prx::packages::pebble_motion::pebble_tree_solver_t, prx::packages::pebble_motion::pebble_solver_t)


namespace prx
{
    using namespace util;
    
    namespace packages
    {
        namespace pebble_motion
        {

            pebble_tree_solver_t::pebble_tree_solver_t() { }

            pebble_tree_solver_t::~pebble_tree_solver_t() { }

            undirected_vertex_index_t pebble_tree_solver_t::add_new_vertex(undirected_graph_t* graph)
            {
                return graph->add_vertex<tree_pebble_node_t > ();
            }

            bool pebble_tree_solver_t::find_solution(std::vector<pebble_step_t>* solution, undirected_graph_t* tree)
            {
                //    g = tree;

                //        PRX_DEBUG_S("Start assign:");
                //        print_assignment(tree,s_assignment);
                //        PRX_DEBUG_S("SNEW assign:");
                //        print_assignment(tree,s_new);
                //        PRX_DEBUG_S("End assign:");
                //        print_assignment(tree,t_assignment);

                allocated_heap_t<undirected_vertex_index_t> leaves(number_of_vertices);
                update_info(tree, t_assignment);

                leaves.restart();
                get_leaves(tree, leaves);

                //    PRX_DEBUG_S("leaves size : " << leaves.size());
                while( !leaves.empty() )
                {
                    undirected_vertex_index_t s = leaves.pop_front();


                    tree_pebble_node_t* tree_node = tree->get_vertex_as<tree_pebble_node_t > (s);
                    tree_node->visited = true;
                    //        PRX_ERROR_S("checking point : " << print_point(tree,s));
                    //        print_assignment(tree,s_new);
                    //        PRX_ERROR_S("-----------");
                    if( t_assignment.has_robot_on(s) )
                    {
                        //            int pebble_idd = t_assignment.get_robot(s);
                        //            PRX_WARN_S(solution->size() << " ) moving pebble : " << pebble_idd << " from start : " << print_point(tree,s_new.get_position(pebble_idd)) << " -> " << print_point(tree,t_assignment.get_position(pebble_idd)));
                        if( t_assignment.get_robot(s) != s_new.get_robot(s) )
                        {
                            int pebble_id = t_assignment.get_robot(s);
                            undirected_vertex_index_t v_start = s_new.get_position(pebble_id);
                            undirected_vertex_index_t v_targ = t_assignment.get_position(pebble_id);
                            //                PRX_ERROR_S(solution->size() << " ) moving pebble : " << pebble_id << " from start : " << print_point(tree,v_start) << " -> " << print_point(tree,v_targ));
                            try
                            {
                                //Giving goal as start so as the a_star will return easy                    
                                //order to iterate over the path.
                                astar_search<
                                        undirected_graph_type,
                                        undirected_graph_t,
                                        undirected_vertex_index_t,
                                        undirected_node_t
                                        > (*tree, v_targ, v_start);
                            }
                            catch( prx_found_goal_t e )
                            {
                                //                    PRX_DEBUG_S("V(tree): " << boost::num_vertices(tree->graph) << "    E(tree):" << boost::num_edges(tree->graph));
                                //
                                //                    foreach(undirected_edge_index_t e, boost::edges(tree->graph))
                                //                    {
                                //                        PRX_INFO_S(e << "(" << print_point(tree,boost::source(e, tree->graph)) << " - " << print_point(tree,boost::target(e, tree->graph)) << ")  weight=" << tree->weights[e]);
                                //                    }
                                //
                                //                    foreach(undirected_vertex_index_t v, boost::vertices(tree->graph))
                                //                    {
                                //                        PRX_DEBUG_S("v: " << print_point(tree,v) << "    has pred:  " << print_point(tree,tree->predecessors[v]));
                                //                    }
                                //                    std::vector<undirected_vertex_index_t> free_path;
                                //                    PRX_INFO_S("robot id : " << pebble_id << " ) " << print_point(tree,v_start) << "  -> " << print_point(tree,v_targ));
                                undirected_vertex_index_t v_curr;
                                std::vector<undirected_vertex_index_t> robot_path;

                                for( v_curr = v_start; v_curr != v_targ; v_curr = tree->predecessors[v_curr] )
                                {
                                    robot_path.push_back(v_curr);
                                }
                                robot_path.push_back(v_targ);


                                for( unsigned int i = 1; i < robot_path.size(); ++i )
                                {
                                    //                        PRX_DEBUG_S("curr : " << print_point(tree,v_curr) << "   - >  " << print_point(tree,tree->predecessors[v_curr]) << "  next node");
                                    v_curr = robot_path[i];
                                    //                        PRX_INFO_S("v_curr  (" << v_curr << "): " << print_point(tree,v_curr));
                                    //                        PRX_INFO_S("v_start (" << v_start << "): " << print_point(tree,v_start));
                                    //                        PRX_INFO_S("v_targ  (" << v_targ << "): " << print_point(tree,v_targ));
                                    //                        if( !s_new.has_robot_on(v_curr) )
                                    //                        {
                                    //                            free_path.push_back(v_curr);
                                    //                            //                            PRX_DEBUG_S("add (" << v_curr << "): " << print_point(tree,v_curr) << "    in the free path;");
                                    //                        }
                                    //                        else
                                    if( s_new.has_robot_on(v_curr) )
                                    {

                                        undirected_vertex_index_t w;
                                        int criterion_no;

                                        boost::tie(w, criterion_no) = find_branch(tree, v_start, v_curr);
                                        //                            PRX_INFO_S("found w " << w << "   criterion: " << criterion_no);

                                        if( w == NULL )
                                        {
                                            return false;
                                        }
                                        else
                                        {
                                            int pebble_id2 = s_new.get_robot(v_curr);
                                            if( !swap(solution, tree, pebble_id, pebble_id2, w, criterion_no) )
                                            {
                                                return false;
                                            }
                                            //                                free_path.clear();
                                        }

                                        //                            PRX_DEBUG_S("start : " << print_point(tree,v_start) << "     curr:" << print_point(tree,v_curr));
                                        v_start = v_curr;
                                    }
                                }
                                //                    PRX_INFO_S("--------------  s_new -------------------");
                                //                    print_assignment(tree,s_new);
                                //                    PRX_INFO_S("------------------------------------------");

                            }

                            //                PRX_WARN_S("v_start : " << print_point(tree,v_start));
                            //                PRX_WARN_S("v_targ  : " << print_point(tree,v_targ));
                        }
                    }

                    foreach(undirected_vertex_index_t v_heap, tree_node->seen)
                    {
                        tree_pebble_node_t* node = tree->get_vertex_as<tree_pebble_node_t > (v_heap);

                        if( !node->visited && node->occupied && !leaves.has(v_heap) )
                        {
                            leaves.push_back(v_heap);
                            node->visited = true;
                        }
                    }
                }
                return true;
            }

            bool pebble_tree_solver_t::reduce(std::vector<pebble_step_t>* solution, undirected_graph_t* graph, pebble_assignment_t& s_new, pebble_assignment_t& s_assign, pebble_assignment_t& t_assign)
            {
                obstacle_global_id++;
                undirected_vertex_index_t l, par;
                pebble_assignment_t curr_assign;

                curr_assign = s_assign;

                static_heap.restart();
                get_leaves(graph, static_heap);

                tree_pebble_node_t* curr_v;
                while( !static_heap.empty() )
                {
                    //        PRX_WARN_S("+++++++++++++++++++++++++++++++++++++++++++++++++");
                    //
                    //        foreach(undirected_vertex_index_t vvvv, leaves)
                    //        {
                    //            PRX_INFO_S("leaf : " << print_point(graph, vvvv));
                    //        }
                    //        PRX_WARN_S("+++++++++++++++++++++++++++++++++++++++++++++++++");
                    //


                    l = static_heap.pop_front();

                    curr_v = graph->get_vertex_as<tree_pebble_node_t > (l);
                    par = curr_v->get_parent(graph, obstacle_global_id);

                    //        PRX_INFO_S("----------------------------------------   leaf : " << l << " : " <<  print_point(graph,curr_v->index));


                    if( t_assign.is_empty(l) )
                    {
                        if( !curr_assign.is_empty(l) )
                        {
                            if( !push_pebble_once(solution, graph, l, curr_assign) )
                                return false;
                        }
                    }
                    else
                    {
                        if( curr_assign.is_empty(l) )
                        {
                            int pebble_id = find_closest_robot(graph, l, curr_assign);

                            if( pebble_id == -1 )
                                return false;
                            undirected_vertex_index_t v_del = curr_assign.get_position(pebble_id);
                            //                PRX_DEBUG_S("Case 2 move the robot " << pebble_id << "   in position " << l);

                            std::vector<undirected_vertex_index_t> robot_path;
                            for( undirected_vertex_index_t v = v_del; v != l; v = graph->predecessors[v] )
                            {
                                robot_path.push_back(v);
                            }
                            robot_path.push_back(l);
                            solution->push_back(std::make_pair(pebble_id, robot_path));
                            curr_assign.remove_assignment(v_del);
                            s_new.add_assignment(l, pebble_id);
                            robot_path.clear();
                        }
                        else
                        {
                            //                PRX_DEBUG_S("Case 3 set the robot " << curr_assign.get_robot(l) << "   in position " << print_point(graph, l));
                            s_new.add_assignment(l, curr_assign.get_robot(l));
                            curr_assign.remove_assignment(l);
                        }

                    }
                    graph->get_vertex_as<tree_pebble_node_t > (l)->obstacle_id = obstacle_global_id;

                    //        PRX_DEBUG_S("going to check " << print_point(graph, par));
                    if( par != l && graph->get_vertex_as<tree_pebble_node_t > (par)->num_unvisited_neighboors(graph, obstacle_global_id) <= 1 )
                        if( !static_heap.has(par) )
                            static_heap.push_back(par);
                    //        PRX_INFO_S("leaves size AFTER : " << leaves.size());
                }

                //    PRX_WARN_S("------------------------------------");
                //    PRX_WARN_S("------------    curr    ------------");
                //    print_assignment(graph,curr_assign);
                //    PRX_WARN_S("------------    strt    ------------");
                //    print_assignment(graph,s_assign);
                //    PRX_WARN_S("------------    snew    ------------");
                //    print_assignment(graph,s_new);
                //    PRX_WARN_S("-----------     targ    ------------");
                //    print_assignment(graph,t_assign);
                //    PRX_WARN_S("------------------------------------");

                return true;
            }

            void pebble_tree_solver_t::update_info(undirected_graph_t* graph, pebble_assignment_t& t_assign)
            {
                std::deque<undirected_vertex_index_t> leaves;
                undirected_vertex_index_t l, par;

                static_heap.restart();
                get_leaves(graph, static_heap);

                tree_pebble_node_t* curr_v;
                tree_pebble_node_t* curr_u;
                while( !static_heap.empty() )
                {
                    l = static_heap.pop_front();
                    curr_v = graph->get_vertex_as<tree_pebble_node_t > (l);
                    //                PRX_INFO_S("-----------------UPDATE INFO ----------------   leaf : " << l << " : " << print_point(graph,curr_v->index));
                    par = curr_v->get_parent2(graph);

                    if( !t_assign.is_empty(l) )
                        curr_v->occupied = true;

                    curr_v->checked = true;

                    //Tree info updates

                    foreach(undirected_vertex_index_t u, boost::adjacent_vertices(l, graph->graph))
                    {
                        //            PRX_DEBUG_S("adj is u : " << u << " / " << boost::degree(l, graph->graph));
                        curr_u = graph->get_vertex_as<tree_pebble_node_t > (u);
                        if( !curr_u->checked )
                        {
                            update_node_info(graph, u, l, t_assign);
                        }
                    }

                    if( par != l && graph->get_vertex_as<tree_pebble_node_t > (par)->num_unvisited_neighboors2(graph) <= 1 )
                        if( !static_heap.has(par) )
                            static_heap.push_back(par);
                }

                //run again from inside to outside
                static_heap.restart();
                static_heap.push_back(l);
                while( !static_heap.empty() )
                {
                    l = static_heap.pop_front();
                    //                PRX_DEBUG_S("--------------------------UPDATE INFO---------------------   leaf : " << l);        

                    foreach(undirected_vertex_index_t u, boost::adjacent_vertices(l, graph->graph))
                    {
                        tree_pebble_node_t* curr_u = graph->get_vertex_as<tree_pebble_node_t > (u);
                        if( curr_u->checked_trees != curr_u->num_trees )
                        {
                            update_node_info(graph, u, l, t_assign);
                            if( !static_heap.has(u) )
                            {
                                static_heap.push_back(u);
                            }
                        }
                    }
                }

            }

            void pebble_tree_solver_t::update_node_info(undirected_graph_t* graph, undirected_vertex_index_t v, undirected_vertex_index_t u, pebble_assignment_t& assign)
            {
                tree_pebble_node_t* curr_v = graph->get_vertex_as<tree_pebble_node_t > (v);
                tree_pebble_node_t* curr_u = graph->get_vertex_as<tree_pebble_node_t > (u);
                bool u_occupied = assign.has_robot_on(u);

                //    PRX_WARN_S("update info for vertex : " << v << "   point:" << print_point(graph,curr_v->index));

                curr_v->num_trees = boost::degree(v, graph->graph);

                if( boost::degree(v, graph->graph) >= 3 )
                {
                    curr_v->trees[u].swap_dist = 0;
                    curr_v->trees[u].v_swap = v;
                }

                if( !u_occupied )
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
                                //                        PRX_DEBUG_S("keep that");
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

            std::pair<undirected_vertex_index_t, int> pebble_tree_solver_t::find_branch(const undirected_graph_t* graph, undirected_vertex_index_t v, undirected_vertex_index_t u)
            {
                tree_pebble_node_t* v_node = graph->get_vertex_as<tree_pebble_node_t > (v);
                tree_pebble_node_t* u_node = graph->get_vertex_as<tree_pebble_node_t > (u);
                undirected_vertex_index_t w;

                //    PRX_DEBUG_S("v:" << print_point(graph,v) << "  -  " << print_point(graph,u));

                try
                {
                    astar_search<
                            undirected_graph_type,
                            undirected_graph_t,
                            undirected_vertex_index_t,
                            undirected_node_t
                            > (graph, v, u);
                }
                catch( prx_found_goal_t f_goal )
                {
                    //        PRX_DEBUG_S("found path");
                    for( undirected_vertex_index_t p = graph->predecessors[u_node->index]; p != v_node->index; p = graph->predecessors[p] )
                    {
                        //            PRX_DEBUG_S("path : " << p << "   deg: " << boost::degree(p, graph->graph));
                        if( boost::degree(p, graph->graph) > 2 )
                        {
                            //                PRX_WARN_S("the between w : " << print_point(graph,p));
                            tree_pebble_node_t* p_node = graph->get_vertex_as<tree_pebble_node_t > (p);
                            //                PRX_DEBUG_S("first:" << p_node->trees[p_node->tree_for_vertex[v_node->index]].holes << "  sec:" << p_node->trees[p_node->tree_for_vertex[u_node->index]].holes << "   min:" << PRX_MINIMUM(p_node->trees[p_node->tree_for_vertex[v_node->index]].holes, p_node->trees[p_node->tree_for_vertex[u_node->index]].holes));
                            //                PRX_DEBUG_S("OR : " << p_node->get_holes_except(v_node->index, u_node->index));
                            if( p_node->get_holes_except(v_node->index, u_node->index) > 0 )
                            {
                                //                    PRX_DEBUG_S("v:" << print_point(graph,v_node->index) << "  -  " << print_point(graph,u_node->index) << " are equivalent criterion 1.2");
                                return std::make_pair(p, 1);
                            }
                            if( PRX_MINIMUM(p_node->trees[p_node->tree_for_vertex[v_node->index]].holes, p_node->trees[p_node->tree_for_vertex[u_node->index]].holes) > 0 )
                            {
                                //                    PRX_DEBUG_S("v:" << print_point(graph,v_node->index) << "  -  " << print_point(graph,u_node->index) << " are equivalent criterion 1");
                                return std::make_pair(p, 2);
                            }


                        }
                    }
                }

                //    PRX_DEBUG_S("Going for criterion 2.v");
                if( (w = is_equivalent_criterion_2(graph, u, v)) != NULL )
                {
                    //        PRX_DEBUG_S("v:" << print_point(graph,v_node->index) << "  -  " << print_point(graph,u_node->index) << " are equivalent criterion 2.v");
                    return std::make_pair(w, 3);
                }

                //    PRX_DEBUG_S("Going for criterion 2.u");
                if( (w = is_equivalent_criterion_2(graph, v, u)) != NULL )
                {
                    //        PRX_DEBUG_S("v:" << print_point(graph,v_node->index) << "  -  " << print_point(graph,u_node->index) << " are equivalent criterion 2.u");
                    return std::make_pair(w, 4);
                }

                //    PRX_DEBUG_S("Going for criterion 4.v");
                if( boost::degree(v_node->index, graph->graph) > 2 && (w = is_equivalent_criterion_4(graph, v, u)) != NULL )
                {
                    //        PRX_DEBUG_S("v:" << print_point(graph,v_node->index) << "  -  " << print_point(graph,u_node->index) << " are equivalent criterion 4.v");
                    return std::make_pair(w, 5);
                }

                //    PRX_DEBUG_S("Going for criterion 4.u");
                if( boost::degree(u_node->index, graph->graph) > 2 && (w = is_equivalent_criterion_4(graph, u, v)) != NULL )
                {
                    u_node->equiv_class = v_node->equiv_class;
                    //        PRX_DEBUG_S("v:" << print_point(graph,v_node->index) << "  -  " << print_point(graph,u_node->index) << " are equivalent criterion 4.u");
                    return std::make_pair(w, 6);
                }

                return std::make_pair(w, -1);
            }

            undirected_vertex_index_t pebble_tree_solver_t::is_equivalent_criterion_2(const undirected_graph_t* graph, undirected_vertex_index_t v, undirected_vertex_index_t u)
            {
                tree_pebble_node_t* u_node = graph->get_vertex_as<tree_pebble_node_t > (u);
                undirected_vertex_index_t w = u_node->get_closest_swap_avoid(v);
                //    PRX_INFO_S("criterion 2 v:" << print_point(graph,v) << " - u:" << print_point(graph,u_node->index));
                if( w != NULL )
                {
                    //        PRX_WARN_S("the w : " << print_point(graph,w));
                    undirected_vertex_index_t t = u_node->tree_for_vertex[w];

                    //        foreach(undirected_vertex_index_t vt, u_node->tree_for_vertex | boost::adaptors::map_keys)
                    //        {
                    ////            PRX_INFO_S("v:" << print_point(graph,vt) << "    has tree: " << u_node->tree_for_vertex[vt]);
                    //            if( u_node->tree_for_vertex[vt] != NULL )
                    //                PRX_INFO_S(print_point(graph,u_node->tree_for_vertex[vt]));
                    //
                    //        }
                    //        PRX_DEBUG_S("holes : " << u_node->trees[t].holes << "   dist+2: " << u_node->trees[t].swap_dist + 2);
                    if( u_node->trees[t].holes >= std::floor(u_node->trees[t].swap_dist) + 2 )
                        return w;
                }
                return NULL;
            }

            undirected_vertex_index_t pebble_tree_solver_t::is_equivalent_criterion_4(const undirected_graph_t* graph, undirected_vertex_index_t v, undirected_vertex_index_t u)
            {
                //    PRX_DEBUG_S("Criterion4)  v: " << print_point(graph,v) << "     u:" << print_point(graph,u));
                tree_pebble_node_t* v_node = graph->get_vertex_as<tree_pebble_node_t > (v);
                undirected_vertex_index_t t = v_node->tree_for_vertex[u];
                //    PRX_DEBUG_S("criterion4) free_trees:" << v_node->num_free_trees << "    holes: " << v_node->get_holes_except(t, t));
                if( v_node->num_free_trees < 2 || v_node->get_holes_except(t, t) < 2 )
                    return NULL;

                return v;
            }

            bool pebble_tree_solver_t::swap(std::vector<pebble_step_t>* solution, const undirected_graph_t* graph, int pebble1, int pebble2, undirected_vertex_index_t w, int criterion_no)
            {
                undirected_vertex_index_t v;
                undirected_vertex_index_t u;
                int start_reverse, end_reverse;
                tree_pebble_node_t* w_node = graph->get_vertex_as<tree_pebble_node_t > (w);


                if( criterion_no == 4 || criterion_no == 6 )
                {
                    //        PRX_WARN_S("Swapped order");
                    int tmp = pebble2;
                    pebble2 = pebble1;
                    pebble1 = tmp;
                    //        v = s_new.get_position(pebble2);
                    //        u = s_new.get_position(pebble1);
                }

                v = s_new.get_position(pebble1);
                u = s_new.get_position(pebble2);

                tree_pebble_node_t* v_node = graph->get_vertex_as<tree_pebble_node_t > (v);

                undirected_vertex_index_t t1 = w_node->tree_for_vertex[v];
                undirected_vertex_index_t t2 = w_node->tree_for_vertex[u];
                undirected_vertex_index_t t3 = NULL;

                //    PRX_ERROR_S("SWAP : " << pebble1 << "  ->  " << pebble2);
                if( criterion_no == 1 )
                {

                    foreach(undirected_vertex_index_t neigh, boost::adjacent_vertices(w, graph->graph))
                    {
                        //            PRX_INFO_S("neigh : " << print_point(graph,neigh));
                        if( neigh != t1 && neigh != t2 )
                        {
                            if( !s_new.has_robot_on(neigh) )
                            {

                                //                    PRX_INFO_S("from v: " << print_point(graph,v) << " - >  " << print_point(graph,u) << "   with w:" << print_point(graph,w));
                                return perform_swap(solution, graph, pebble1, pebble2, v, u, neigh);
                            }

                            if( t3 == NULL && w_node->trees[neigh].holes > 0 )
                                t3 = neigh;
                        }
                    }

                    PRX_ASSERT(t3 != NULL);
                    obstacle_global_id++;
                    graph->get_vertex_as<tree_pebble_node_t > (w)->obstacle_id = obstacle_global_id;

                    start_reverse = solution->size();

                    if( !push_pebble_once(solution, graph, t3, s_new) )
                        return false;

                    end_reverse = solution->size() - 1;

                    if( !perform_swap(solution, graph, pebble1, pebble2, v, u, t3) )
                        return false;

                    revert(solution, start_reverse, end_reverse, pebble1, pebble2, s_new);

                    return true;


                }
                else if( criterion_no == 2 )
                {

                    foreach(undirected_vertex_index_t neigh, boost::adjacent_vertices(w, graph->graph))
                    {
                        if( neigh != t1 && neigh != t2 )
                        {
                            int blocker_pebble = s_new.get_robot(neigh);
                            start_reverse = solution->size();
                            obstacle_global_id++;
                            graph->get_vertex_as<tree_pebble_node_t > (w)->obstacle_id = obstacle_global_id;

                            if( u == t2 )
                            {
                                //                    PRX_DEBUG_S("PEBBLE IS ON THE T2");
                                if( !push_pebble_once(solution, graph, u, s_new) )
                                    return false;
                                u = s_new.get_position(pebble2);
                            }
                            if( !move(solution, graph, pebble1, v, t2, s_new) )
                                return false;
                            if( !move(solution, graph, blocker_pebble, neigh, t1, s_new) )
                                return false;
                            if( !push_pebble_once(solution, graph, t1, s_new) )
                                return false;
                            end_reverse = solution->size() - 1;
                            if( !perform_swap2(solution, graph, pebble1, pebble2, t2, u, neigh, t1) )
                                return false;
                            revert(solution, start_reverse, end_reverse, pebble1, pebble2, s_new);
                            return true;
                        }
                    }

                }
                else if( criterion_no == 3 || criterion_no == 4 )
                {
                    start_reverse = solution->size();
                    obstacle_global_id++;

                    undirected_vertex_index_t t_pebble1 = v_node->tree_for_vertex[w];

                    foreach(undirected_vertex_index_t v_neigh, boost::adjacent_vertices(v, graph->graph))
                    {
                        if( v_neigh != t_pebble1 )
                        {
                            graph->get_vertex_as<tree_pebble_node_t > (v_neigh)->obstacle_id = obstacle_global_id;

                            //                PRX_DEBUG_S("obstacle : " << print_point(graph,v_neigh));
                        }
                    }
                    undirected_vertex_index_t v_pebble1 = s_new.get_position(pebble1);
                    while( v_pebble1 != t1 )
                    {
                        //            PRX_DEBUG_S(pebble1 << " on : " << print_point(graph,v_pebble1) << "    t1 : " << print_point(graph,t1));
                        if( !push_pebble_once(solution, graph, v_pebble1, s_new) )
                            return false;
                        graph->get_vertex_as<tree_pebble_node_t > (v_pebble1)->obstacle_id = obstacle_global_id;
                        v_pebble1 = s_new.get_position(pebble1);
                    }

                    if( s_new.has_robot_on(w) )
                        if( !push_pebble_once(solution, graph, w, s_new) )
                            return false;



                    obstacle_global_id++;

                    t2 = w_node->find_empty_tree_except(s_new, t1);
                    //        PRX_ERROR_S("t2: " << t2 << "    " << print_point(graph,t2));
                    t3 = w_node->find_empty_tree_except(s_new, t1, t2);
                    //        PRX_ERROR_S("t3: " << t3);
                    if( t3 == NULL )
                    {
                        t3 = w_node->find_full_tree_except(s_new, t1, t2);
                        //            PRX_ERROR_S("t3: " << t3 << "     " << print_point(graph,t3));

                        if( s_new.has_robot_on(t2) )
                        {
                            graph->get_vertex_as<tree_pebble_node_t > (w)->obstacle_id = obstacle_global_id;
                            if( !push_pebble_once(solution, graph, t2, s_new) )
                                return false;
                        }
                        if( !move(solution, graph, s_new.get_robot(t3), t3, t2, s_new) )
                            return false;

                    }
                    else
                    {
                        //            PRX_WARN_S("t3 : " << print_point(graph,t3));
                        graph->get_vertex_as<tree_pebble_node_t > (w)->obstacle_id = obstacle_global_id;
                        if( s_new.has_robot_on(t3) )
                            if( !push_pebble_once(solution, graph, t3, s_new) )
                                return false;
                    }

                    //        PRX_DEBUG_S("going to push and the t2 : " << print_point(graph,t2) << "    with robot : " << s_new.get_robot(t2));
                    graph->get_vertex_as<tree_pebble_node_t > (w)->obstacle_id = obstacle_global_id;
                    if( s_new.has_robot_on(t2) )
                        if( !push_pebble_once(solution, graph, t2, s_new) )
                            return false;

                    end_reverse = solution->size() - 1;

                    obstacle_global_id++;
                    if( !perform_swap2(solution, graph, pebble1, pebble2, s_new.get_position(pebble1), s_new.get_position(pebble2), t3, t2) )
                        return false;
                    revert(solution, start_reverse, end_reverse, pebble1, pebble2, s_new);
                    return true;

                }
                else if( criterion_no == 5 || criterion_no == 6 )
                {
                    start_reverse = solution->size();
                    obstacle_global_id++;
                    t1 = w_node->get_empty_tree_except(t2);
                    t3 = w_node->get_empty_tree_except(t1, t2);

                    //        PRX_INFO_S("t3: " << t3);
                    if( t3 == NULL )
                    {
                        graph->get_vertex_as<tree_pebble_node_t > (t1)->obstacle_id = obstacle_global_id;
                        if( !push_pebble_once(solution, graph, w, s_new) )
                            return false;

                        obstacle_global_id++;
                        graph->get_vertex_as<tree_pebble_node_t > (t2)->obstacle_id = obstacle_global_id;
                        t3 = w_node->get_any_full_tree();
                        //            PRX_INFO_S("full t3: " << t3 << "     " << print_point(graph,t3));

                        if( !move(solution, graph, s_new.get_robot(t3), t3, w, s_new) )
                            return false;

                        graph->get_vertex_as<tree_pebble_node_t > (t3)->obstacle_id = obstacle_global_id;
                        if( !push_pebble_once(solution, graph, w, s_new) )
                            return false;
                    }
                    else
                    {
                        //            PRX_WARN_S("t3 : " << print_point(graph,t3));
                        graph->get_vertex_as<tree_pebble_node_t > (w)->obstacle_id = obstacle_global_id;
                        if( s_new.has_robot_on(t3) )
                            if( !push_pebble_once(solution, graph, t3, s_new) )
                                return false;
                    }

                    //        PRX_DEBUG_S("going to push and the t1 : " << print_point(graph,t1));
                    graph->get_vertex_as<tree_pebble_node_t > (w)->obstacle_id = obstacle_global_id;
                    if( s_new.has_robot_on(t1) )
                        if( !push_pebble_once(solution, graph, t1, s_new) )
                            return false;

                    end_reverse = solution->size() - 1;

                    obstacle_global_id++;
                    if( !perform_swap2(solution, graph, pebble1, pebble2, s_new.get_position(pebble1), s_new.get_position(pebble2), t3, t1) )
                        return false;
                    revert(solution, start_reverse, end_reverse, pebble1, pebble2, s_new);
                    return true;
                }
                else
                {
                    PRX_LOG_ERROR("Criterion %d does not exist!", criterion_no);
                }

                return false;
            }

            bool pebble_tree_solver_t::perform_swap(std::vector<pebble_step_t>* solution, const undirected_graph_t* graph, int pebble1, int pebble2, undirected_vertex_index_t pos1, undirected_vertex_index_t pos2, undirected_vertex_index_t w)
            {
                //        PRX_DEBUG_S("pebble " << pebble1 << "   from : " << print_point(graph,pos1) << "  ->  " << print_point(graph,w));
                if( !move(solution, graph, pebble1, pos1, w, s_new) )
                    return false;
                //        PRX_DEBUG_S("pebble " << pebble2 << "   from : " << print_point(graph,pos2) << "  ->  " << print_point(graph,pos1));
                if( !move(solution, graph, pebble2, pos2, pos1, s_new) )
                    return false;
                //        PRX_DEBUG_S("pebble " << pebble1 << "   from : " << print_point(graph,w) << "  ->  " << print_point(graph,pos2));
                if( !move(solution, graph, pebble1, w, pos2, s_new) )
                    return false;
                return true;
            }

            bool pebble_tree_solver_t::perform_swap2(std::vector<pebble_step_t>* solution, const undirected_graph_t* graph, int pebble1, int pebble2, undirected_vertex_index_t pos1, undirected_vertex_index_t pos2, undirected_vertex_index_t w1, undirected_vertex_index_t w2)
            {
                //    PRX_DEBUG_S("pebble " << pebble1 << "   from : " << print_point(graph,pos1) << "  ->  " << print_point(graph,w1));
                if( !move(solution, graph, pebble1, pos1, w1, s_new) )
                    return false;
                //    PRX_DEBUG_S("pebble " << pebble2 << "   from : " << print_point(graph,pos2) << "  ->  " << print_point(graph,w2));
                if( !move(solution, graph, pebble2, pos2, w2, s_new) )
                    return false;
                //    PRX_DEBUG_S("pebble " << pebble1 << "   from : " << print_point(graph,w1) << "  ->  " << print_point(graph,pos2));
                if( !move(solution, graph, pebble1, w1, pos2, s_new) )
                    return false;
                //    PRX_DEBUG_S("pebble " << pebble2 << "   from : " << print_point(graph,w2) << "  ->  " << print_point(graph,pos1));
                if( !move(solution, graph, pebble2, w2, pos1, s_new) )
                    return false;
                return true;
            }

            void pebble_tree_solver_t::revert(std::vector<pebble_step_t>* path, int start, int end, int pebble1, int pebble2, pebble_assignment_t& assign)
            {
                for( int i = end; i >= start; --i )
                {
                    int rid = path->at(i).first;

                    if( rid == pebble1 )
                        rid = pebble2;
                    else if( rid == pebble2 )
                        rid = pebble1;

                    std::vector<undirected_vertex_index_t> robot_path;
                    //        PRX_DEBUG_S("rid : " << rid << "   sol size: " << path->at(i).second.size());
                    for( int j = (int)path->at(i).second.size() - 1; j >= 0; --j )
                    {
                        //            PRX_INFO_S("j : " << j << "   " << print_point(graph,path->at(i).second[j]));
                        robot_path.push_back(path->at(i).second[j]);
                    }
                    assign.change_position(rid, robot_path.back());
                    path->push_back(std::make_pair(rid, robot_path));

                }
            }

            void pebble_tree_solver_t::get_leaves(const undirected_graph_t* graph, allocated_heap_t<undirected_vertex_index_t>& leaves)
            {

                //    PRX_INFO_S("vertices : " << boost::num_vertices(graph->graph) << "   leaves size:" << leaves.size() << "    leaves maxsize : " << leaves.print());

                foreach(undirected_vertex_index_t v, boost::vertices(graph->graph))
                {
                    if( !graph->get_vertex_as<tree_pebble_node_t > (v)->has_children(graph) )
                        leaves.push_back(v);
                }
            }

            undirected_vertex_index_t pebble_tree_solver_t::find_closest_vertex_with(const undirected_graph_t* graph, bool robot_on, undirected_vertex_index_t start, const pebble_assignment_t& assign)
            {
                std::deque<undirected_vertex_index_t> heap;
                graph->predecessors[start] = start;

                foreach(undirected_vertex_index_t adj, boost::adjacent_vertices(start, graph->graph))
                {
                    heap.push_back(adj);
                    graph->predecessors[adj] = start;
                }

                //    while( !heap.empty() )
                //    {
                //        undirected_vertex_index_t v = heap[0];
                //        heap.pop_front();
                //        if( std::find(obstacles.begin(), obstacles.end(), v) == obstacles.end() )
                //        {
                //            if( assign.has_robot_on(v) == robot_on )
                //            {
                //                return v;
                //            }
                //
                //            foreach(undirected_vertex_index_t adj, boost::adjacent_vertices(v, graph->graph))
                //            {
                //
                //                if( graph->predecessors[v] != adj )
                //                {
                //                    heap.push_back(adj);
                //                    graph->predecessors[adj] = v;
                //                }
                //            }
                //        }
                //    }
                return NULL;
            }

            bool pebble_tree_solver_t::push_pebble_once(std::vector<pebble_step_t>* solution, const undirected_graph_t* graph, undirected_vertex_index_t v, pebble_assignment_t& assign, bool execute_push)
            {

                //    PRX_DEBUG_S("------ s_new  BOFORE PUSH ONCE -----------");
                //    print_assignment(graph,s_new);

                undirected_vertex_index_t w = find_closest_vertex_with(graph, false, v, assign);
                if( w != NULL )
                {
                    undirected_vertex_index_t g_pred;
                    //        PRX_WARN_S("robot on node " << fg.index << " : " << assign.get_robot(fg.index));
                    for( undirected_vertex_index_t u = w; u != v; u = graph->predecessors[u] )
                    {
                        g_pred = graph->predecessors[u];
                        if( assign.has_robot_on(g_pred) )
                        {
                            int pebble_id = assign.get_robot(g_pred);
                            //                PRX_ERROR_S("remove the robot " << pebble_id << "   in position " << g_pred);
                            //                PRX_INFO_S("robot in u:" << u << " :  " << assign.get_robot(u));
                            //                PRX_DEBUG_S("pushing the robot " << pebble_id << " from " << print_point(graph,g_pred) << "   to : " << print_point(graph,u));
                            std::vector<undirected_vertex_index_t> robot_path;
                            robot_path.push_back(g_pred);
                            robot_path.push_back(u);
                            assign.change_position(pebble_id, u);
                            solution->push_back(std::make_pair(pebble_id, robot_path));
                        }
                    }
                    //        PRX_DEBUG_S("------ s_new  -----------");
                    //        print_assignment(graph,s_new);
                    return true;

                }
                return false;
            }

            bool pebble_tree_solver_t::move(std::vector<pebble_step_t>* path, const undirected_graph_t* graph, int pebble_id, undirected_vertex_index_t v_start, undirected_vertex_index_t v_target, pebble_assignment_t& assign)
            {
                try
                {
                    //Giving goal as start so as the a_star will return easy                    
                    //order to iterate over the path.
                    astar_search<
                            undirected_graph_type,
                            undirected_graph_t,
                            undirected_vertex_index_t,
                            undirected_node_t
                            > (graph, v_target, v_start);
                }
                catch( prx_found_goal_t e )
                {
                    std::vector<undirected_vertex_index_t> robot_path;
                    for( undirected_vertex_index_t v_curr = v_start; v_curr != v_target; v_curr = graph->predecessors[v_curr] )
                    {
                        robot_path.push_back(v_curr);
                        //            PRX_DEBUG_S(pebble_id << "  path : " << print_point(graph,v_curr));
                    }
                    //        if(robot_path.size() != 0)
                    //        {
                    robot_path.push_back(v_target);
                    path->push_back(std::make_pair(pebble_id, robot_path));
                    //        }
                    //                PRX_DEBUG_S(pebble_id << "  path : " << print_point(graph,v_target));
                    assign.change_position(pebble_id, v_target);
                    //        PRX_DEBUG_S("------ s_new  -----------");
                    //        print_assignment(graph,s_new);
                    return true;
                }
                return false;
            }

            int pebble_tree_solver_t::find_closest_robot(const undirected_graph_t* graph, undirected_vertex_index_t v, pebble_assignment_t& assign)
            {
                undirected_vertex_index_t u = find_closest_vertex_with(graph, true, v, assign);
                if( u != NULL )
                    return assign.get_robot(u);

                return -1;
            }


        }
    }
}





//       start_reverse = -1;
//        obstacles.clear();
//        int holes_t2, holes_t3;
//        t2 = w_node->get_empty_tree_except(t1);
//        holes_t2 = w_node->trees[t2].holes;
//        PRX_INFO_S("t2: " << print_point(graph,t2) << "     has holes:" << holes_t2);
//        undirected_vertex_index_t t3 = w_node->get_empty_tree_except(t1, t2);
//        print_assignment(graph,s_new);
//        PRX_INFO_S("t1: " << print_point(graph,t1) << "    t2: " << print_point(graph,t2) << "    t3:" << t3);
//        if( t3 != NULL )
//            PRX_DEBUG_S("t3 : " << print_point(graph,t3) << "      has holes : " << w_node->trees[t3].holes);
//        if( t3 == NULL )
//        {
//            PRX_DEBUG_S("t3 is a full tree");
//            t3 = w_node->get_any_full_tree();
//            obstacles.push_back(t1);
//            start_reverse = solution->size();
//            if( !push_pebble_once(solution, t3, s_new, obstacles) )
//                return false;
//            obstacles.push_back(t3);
//            PRX_INFO_S("robot on w : " << s_new.get_robot(w));
//            if( !push_pebble_once(solution, w, s_new, obstacles) )
//                return false;
//            holes_t2--;
//            holes_t3 = -1;
//        }
//        else
//        {
//            holes_t3 = w_node->trees[t3].holes;
//        }
//
//        PRX_ASSERT(t2 != NULL);
//        PRX_ASSERT(t3 != NULL);
//        if( holes_t2 > 1 )
//        {
//
//            foreach(undirected_vertex_index_t neigh, boost::adjacent_vertices(w, g->graph))
//            {
//                if( neigh != t1 && neigh != t2 )
//                    obstacles.push_back(neigh);
//            }
//        }
//        else
//        {
//            holes_t2 = -1;
//            obstacles.push_back(t2);
//        }
//
//        try
//        {
//            astar_search<
//                    undirected_graph_type,
//                    undirected_graph_t,
//                    undirected_vertex_index_t,
//                    undirected_node_t
//                    > (*g, t2, v);
//        }
//        catch( prx_found_goal_t f_goal )
//        {
//            if( start_reverse == -1 )
//                start_reverse = solution->size();
//            std::vector<undirected_vertex_index_t> pebble_path;
//            for( undirected_vertex_index_t p = v; p != t2; p = g->predecessors[p] )
//            {
//                pebble_path.push_back(p);
//            }
//            pebble_path.push_back(t2);
//
//            for( int i = 1; i < (int)pebble_path.size(); ++i )
//            {
//                if( holes_t2 == 1 )
//                {
//                    obstacles.clear();
//                    holes_t2 = -1;
//                    if( holes_t3 >= 1 )
//                    {
//
//                        foreach(undirected_vertex_index_t neigh, boost::adjacent_vertices(w, g->graph))
//                        {
//                            if( neigh != t1 && neigh != t3 )
//                                obstacles.push_back(neigh);
//                        }
//                    }
//                    else
//                    {
//                        obstacles.push_back(t2);
//                        obstacles.push_back(t3);
//                    }
//                }
//
//                if( holes_t3 == 1 )
//                {
//                    obstacles.clear();
//                    holes_t3 = -1;
//                    obstacles.push_back(t2);
//                    obstacles.push_back(t3);
//                }
//
//                foreach(undirected_vertex_index_t v_obs, obstacles)
//                {
//                    PRX_DEBUG_S("obst:  " << print_point(graph,v_obs));
//                }
//                if( s_new.has_robot_on(pebble_path[i]) )
//                {
//                    obstacles.push_back(pebble_path[i - 1]);
//                    if( !push_pebble_once(solution, pebble_path[i], s_new, obstacles) )
//                        return false;
//                }
//
//                holes_t2--;
//                if( holes_t2 < 1 )
//                    holes_t3--;
//            }
//
//            PRX_WARN_S("Done with the clearing");
//
//            obstacles.clear();
//            obstacles.push_back(w);
//
//            if( s_new.has_robot_on(t2) )
//                if( !push_pebble_once(solution, t2, s_new, obstacles) )
//                    return false;
//
//            if( s_new.has_robot_on(t3) )
//                if( !push_pebble_once(solution, t3, s_new, obstacles) )
//                    return false;
//            end_reverse = solution->size() - 1;
//
//            perform_swap2(solution, graph, pebble1, pebble2, v, u, t2, t3);
//            revert(solution, start_reverse, end_reverse, pebble1, pebble2, s_new);
//            return true;
//        }