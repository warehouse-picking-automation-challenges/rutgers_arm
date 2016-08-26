/**
 * @file tree_feasibility_tester.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2012, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file robot_idd LICENSE.
 *
 * @authors Athanasios Krontiris, Andrew Dobson, Andrew Kimmel, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "utilities/tree_feasibility_tester/tree_feasibility_tester.hpp"

#include <deque>
#include <boost/range/adaptor/map.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::packages::pebble_motion::tree_feasibility_tester_t, prx::packages::pebble_motion::pebble_tester_t)


namespace prx
{
    using namespace util;
    
    namespace packages
    {
        namespace pebble_motion
        {

            tree_feasibility_tester_t::tree_feasibility_tester_t() { }

            tree_feasibility_tester_t::~tree_feasibility_tester_t() { }

            undirected_vertex_index_t tree_feasibility_tester_t::add_new_vertex(undirected_graph_t* graph)
            {
                return graph->add_vertex<tree_pebble_node_t > ();
            }

            bool tree_feasibility_tester_t::pebble_test(undirected_graph_t* graph)
            {
                int global_class = 1;
                int par_class;
                std::vector<undirected_vertex_index_t> bad_vertices;
                g = graph;

                PRX_DEBUG_S("Start assign:");
                s_assignment.print();
                PRX_DEBUG_S("End assign:");
                t_assignment.print();

                pebble_assignment_t s_new;

                reduce(s_new, s_assignment, t_assignment);

                undirected_vertex_index_t s = s_new.get_first_position_with_robot();
                g->get_vertex_as<tree_pebble_node_t > (s)->equiv_class = global_class;
                g->get_vertex_as<tree_pebble_node_t > (s)->visited = true;
                global_class++;

                std::vector<undirected_vertex_index_t> heap;
                heap.push_back(s);

                while( !heap.empty() )
                {
                    s = heap[0];
                    heap.erase(heap.begin());

                    bad_vertices.clear();
                    tree_pebble_node_t* tree_node = g->get_vertex_as<tree_pebble_node_t > (s);
                    PRX_ERROR_S("checking point : " << state_space->print_point(tree_node->point, 3));
                    compute_equivalence(bad_vertices, tree_node);
                    PRX_WARN_S("bad vertices : " << bad_vertices.size());

                    if( bad_vertices.size() > 0 )
                    {
                        if( tree_node->num_free_trees == 1 )
                        {
                            PRX_INFO_S("one free tree   gc: " << global_class);
                            PRX_ASSERT(tree_node->free_trees.size() == 1);
                            if( tree_node->free_trees[0] == tree_node->v_par )
                            {

                                PRX_DEBUG_S("parent tree   gc:" << global_class);

                                foreach(undirected_vertex_index_t u, bad_vertices)
                                {
                                    PRX_DEBUG_S("bad vertex: " << state_space->print_point(g->get_vertex_as<tree_pebble_node_t > (u)->point, 3));
                                    g->get_vertex_as<tree_pebble_node_t > (u)->equiv_class = global_class;

                                }

                            }
                            else
                            {
                                PRX_DEBUG_S("other alone tree   gc:" << global_class);
                                bool new_class = false;
                                par_class = g->get_vertex_as<tree_pebble_node_t > (tree_node->v_par)->equiv_class;
                                PRX_ASSERT(par_class != -1);

                                foreach(undirected_vertex_index_t u, bad_vertices)
                                {
                                    PRX_DEBUG_S("bad vertex: " << state_space->print_point(g->get_vertex_as<tree_pebble_node_t > (u)->point, 3));
                                    if( tree_node->free_trees[0] == tree_node->tree_for_vertex[u] )
                                    {
                                        g->get_vertex_as<tree_pebble_node_t > (u)->equiv_class = global_class;
                                        new_class = true;
                                    }
                                    else
                                    {
                                        g->get_vertex_as<tree_pebble_node_t > (u)->equiv_class = par_class;
                                    }
                                }
                                if( new_class )
                                    global_class++;
                            }
                        }
                        else if( tree_node->num_free_trees == 2 )
                        {
                            PRX_INFO_S("Two free trees   gc: " << global_class);
                            PRX_ASSERT(tree_node->free_trees.size() == 2);
                            undirected_vertex_index_t t1 = tree_node->free_trees[0];
                            undirected_vertex_index_t t2 = tree_node->free_trees[1];
                            bool case_4 = (tree_node->trees[t1].holes == 1 && tree_node->trees[t2].holes > 1) || (tree_node->trees[t1].holes > 1 && tree_node->trees[t2].holes == 1);
                            //                if( (tree_node->trees[t1].holes == 1 && tree_node->trees[t2].holes == 1) || (tree_node->trees[t1].holes > 1 && tree_node->trees[t2].holes > 1) )
                            if( !case_4 || (case_4 && boost::degree(tree_node->index, g->graph) <= 2) )
                            {
                                PRX_DEBUG_S("one one holes   gc:" << global_class << "  t1[holes]:" << tree_node->trees[t1].holes << "    t2[holes]:" << tree_node->trees[t2].holes);
                                int c1, c2;

                                bool new_c1 = false;
                                bool new_c2 = false;

                                foreach(undirected_vertex_index_t u, bad_vertices)
                                {
                                    PRX_DEBUG_S("bad vertex: " << state_space->print_point(g->get_vertex_as<tree_pebble_node_t > (u)->point, 3));
                                    tree_pebble_node_t* adj_node = g->get_vertex_as<tree_pebble_node_t > (u);

                                    if( tree_node->tree_for_vertex[u] == t1 )
                                    {
                                        if( !new_c1 )
                                        {
                                            c1 = global_class;
                                            global_class++;
                                        }

                                        adj_node->equiv_class = c1;
                                        new_c1 = true;
                                    }
                                    else if( tree_node->tree_for_vertex[u] == t2 )
                                    {
                                        if( !new_c2 )
                                        {
                                            c2 = global_class;
                                            global_class++;
                                        }
                                        adj_node->equiv_class = c2;
                                        new_c2 = true;
                                    }
                                    else
                                    {
                                        PRX_LOG_ERROR("Something wrong with equivalence classes")
                                    }

                                }

                            }
                            else
                            {
                                PRX_DEBUG_S("one more than one holes   gc:" << global_class);
                                PRX_ASSERT(case_4);
                                undirected_vertex_index_t t;
                                bool new_class = false;
                                if( tree_node->trees[t1].holes > 1 )
                                    t = t1;
                                else if( tree_node->trees[t2].holes > 1 )
                                    t = t2;
                                else
                                    PRX_LOG_ERROR("Something wrong with the number of holes in the trees");

                                foreach(undirected_vertex_index_t u, bad_vertices)
                                {
                                    PRX_DEBUG_S("bad vertex: " << state_space->print_point(g->get_vertex_as<tree_pebble_node_t > (u)->point, 3));
                                    if( tree_node->tree_for_vertex[u] == t )
                                    {
                                        g->get_vertex_as<tree_pebble_node_t > (u)->equiv_class = global_class;
                                        new_class = true;
                                    }
                                }
                                if( new_class )
                                    global_class++;
                            }
                        }
                        else if( tree_node->num_free_trees >= 3 )
                        {
                            PRX_LOG_ERROR("Why am I here ?");
                        }

                        PRX_WARN_S("---------------------------------------------------------------------");

                        foreach(undirected_vertex_index_t v, boost::vertices(g->graph))
                        {
                            tree_pebble_node_t* node = g->get_vertex_as<tree_pebble_node_t > (v);
                            PRX_INFO_S("point: " << state_space->print_point(node->point, 3) << "      class:" << node->equiv_class);
                        }
                        PRX_WARN_S("---------------------------------------------------------------------");
                    }

                    foreach(undirected_vertex_index_t v_heap, tree_node->seen)
                    {
                        tree_pebble_node_t* node = g->get_vertex_as<tree_pebble_node_t > (v_heap);
                        PRX_INFO_S("point: " << state_space->print_point(node->point, 3) << "      class:" << node->equiv_class);
                        bool in_heap = std::find(heap.begin(), heap.end(), v_heap) == heap.end();
                        PRX_INFO_S("visited: " << node->visited << "     occupied: " << node->occupied << "   in the heap : " << in_heap);
                        if( !node->visited && node->occupied && std::find(heap.begin(), heap.end(), v_heap) == heap.end() )
                        {
                            PRX_WARN_S("added");
                            heap.push_back(v_heap);
                            node->visited = true;
                        }
                    }
                    PRX_WARN_S("Heap size : " << heap.size());
                }

                foreach(undirected_vertex_index_t v, boost::vertices(g->graph))
                {
                    tree_pebble_node_t* node = g->get_vertex_as<tree_pebble_node_t > (v);
                    PRX_ERROR_S("point: " << state_space->print_point(node->point, 3) << "      class:" << node->equiv_class);
                }

                foreach(int robot_id, s_new.assignments | boost::adaptors::map_values)
                {
                    undirected_vertex_index_t v_start = s_new.get_position(robot_id);
                    undirected_vertex_index_t v_end = t_assignment.get_position(robot_id);
                    if( g->get_vertex_as<tree_pebble_node_t > (v_start)->equiv_class != g->get_vertex_as<tree_pebble_node_t > (v_end)->equiv_class )
                    {
                        PRX_ERROR_S("the robot : " << robot_id << "  has classes start: " << g->get_vertex_as<tree_pebble_node_t > (v_start)->equiv_class << " -  end: " << g->get_vertex_as<tree_pebble_node_t > (v_end)->equiv_class);
                        return false;
                    }
                }

                return true;
            }

            bool tree_feasibility_tester_t::reduce(pebble_assignment_t& s_new, pebble_assignment_t& s_assign, pebble_assignment_t& t_assign)
            {
                std::deque<undirected_vertex_index_t> leaves;
                std::vector<undirected_vertex_index_t> obstacles;
                undirected_vertex_index_t l, par;
                pebble_assignment_t curr_assign;

                curr_assign = s_assign;

                get_leaves(leaves);

                tree_pebble_node_t* curr_v;
                tree_pebble_node_t* curr_u;
                while( !leaves.empty() )
                {
                    l = leaves[0];
                    curr_v = g->get_vertex_as<tree_pebble_node_t > (l);
                    PRX_INFO_S("----------------------------------------   leaf : " << l << " : " << state_space->print_point(curr_v->point, 2));
                    par = curr_v->get_parent2(g);

                    leaves.pop_front();

                    if( t_assign.is_empty(l) )
                    {
                        if( !curr_assign.is_empty(l) )
                            if( !push_pebble(l, curr_assign, obstacles) )
                                return false;
                    }
                    else
                    {
                        curr_v->occupied = true;
                        if( curr_assign.is_empty(l) )
                        {
                            int robot_id = find_closest_robot(l, curr_assign, obstacles);

                            if( robot_id == -1 )
                                return false;

                            PRX_DEBUG_S("Case 2 move the robot " << robot_id << "   in position " << l);
                            s_new.add_assignment(l, robot_id);
                        }
                        else
                        {
                            PRX_DEBUG_S("Case 3 set the robot " << curr_assign.get_robot(l) << "   in position " << l);
                            s_new.add_assignment(l, curr_assign.get_robot(l));
                            curr_assign.remove_assignment(l);
                        }

                    }
                    curr_v->checked = true;
                    obstacles.push_back(l);


                    //Tree info updates

                    foreach(undirected_vertex_index_t u, boost::adjacent_vertices(l, g->graph))
                    {
                        PRX_ERROR_S("adj is u : " << u << "   are: " << boost::degree(l, g->graph));
                        curr_u = g->get_vertex_as<tree_pebble_node_t > (u);
                        if( !curr_u->checked )
                        {
                            update_node_info(u, l, t_assign);
                        }
                    }

                    //        PRX_INFO_S("par : " << par << "-" << l << "     has neigh: " << g->get_vertex_as<tree_pebble_node_t > (par)->num_unvisited_neighboors(g));
                    //        PRX_INFO_S("leaves size : " << leaves.size(`));
                    //        PRX_WARN_S("----------    curr   --------------------");
                    //        print_assign(curr_assign);
                    //        PRX_WARN_S("----------    snew   --------------------");
                    //        print_assign(s_new);
                    //        PRX_WARN_S("------------------------------");
                    if( par != l && g->get_vertex_as<tree_pebble_node_t > (par)->num_unvisited_neighboors2(g) <= 1 )
                        if( std::find(leaves.begin(), leaves.end(), par) == leaves.end() )
                            leaves.push_back(par);
                    //        PRX_INFO_S("leaves size AFTER : " << leaves.size());
                }

                //run again from inside to outside
                leaves.push_back(l);
                while( !leaves.empty() )
                {
                    l = leaves[0];
                    PRX_DEBUG_S("-----------------------------------------------------------   leaf : " << l);
                    leaves.erase(leaves.begin());

                    foreach(undirected_vertex_index_t u, boost::adjacent_vertices(l, g->graph))
                    {
                        tree_pebble_node_t* curr_u = g->get_vertex_as<tree_pebble_node_t > (u);
                        if( curr_u->checked_trees != curr_u->num_trees )
                        {
                            update_node_info(u, l, t_assign);
                            if( std::find(leaves.begin(), leaves.end(), u) == leaves.end() )
                            {
                                leaves.push_back(u);
                            }
                        }
                    }
                }

                PRX_WARN_S("------------------------------------");
                PRX_WARN_S("------------    curr    ------------");
                print_assignment(curr_assign);
                PRX_WARN_S("------------    strt    ------------");
                print_assignment(s_assign);
                PRX_WARN_S("------------    snew    ------------");
                print_assignment(s_new);
                PRX_WARN_S("-----------     targ    ------------");
                print_assignment(t_assign);
                PRX_WARN_S("------------------------------------");

                return true;
            }

            void tree_feasibility_tester_t::compute_equivalence(std::vector<undirected_vertex_index_t>& bad_vertices, tree_pebble_node_t* node)
            {

                foreach(undirected_vertex_index_t u, node->seen)
                {
                    tree_pebble_node_t* u_node = g->get_vertex_as<tree_pebble_node_t > (u);
                    if( !u_node->occupied )
                    {
                        u_node->equiv_class = node->equiv_class;
                    }
                    else
                    {
                        test_equivalent(node, u_node);
                        if( u_node->equiv_class == -1 )
                        {
                            PRX_ERROR_S("bad vertex: " << u_node->print_point(state_space, 2));
                            bad_vertices.push_back(u);
                        }
                    }
                }
            }

            void tree_feasibility_tester_t::get_leaves(std::deque<undirected_vertex_index_t>& leaves)
            {

                foreach(undirected_vertex_index_t v, boost::vertices(g->graph))
                {
                    if( !g->get_vertex_as<tree_pebble_node_t > (v)->has_children(g) )
                        leaves.push_back(v);
                }
            }

            undirected_vertex_index_t tree_feasibility_tester_t::find_closest_vertex_with(bool robot_on, undirected_vertex_index_t start, const pebble_assignment_t& assign, const std::vector<undirected_vertex_index_t>& obstacles)
            {
                std::deque<undirected_vertex_index_t> heap;

                foreach(undirected_vertex_index_t adj, boost::adjacent_vertices(start, g->graph))
                {
                    heap.push_back(adj);
                }

                while( !heap.empty() )
                {
                    undirected_vertex_index_t v = heap[0];
                    heap.pop_front();
                    if( std::find(obstacles.begin(), obstacles.end(), v) == obstacles.end() )
                    {
                        if( assign.has_robot_on(v) == robot_on )
                        {
                            return v;
                        }

                        foreach(undirected_vertex_index_t adj, boost::adjacent_vertices(v, g->graph))
                        {
                            heap.push_back(adj);
                        }
                    }
                    PRX_INFO_S("heap size: " << heap.size());

                }
                return NULL;
            }

            bool tree_feasibility_tester_t::push_pebble(undirected_vertex_index_t v, pebble_assignment_t& assign, const std::vector<undirected_vertex_index_t>& obstacles)
            {
                undirected_vertex_index_t w = find_closest_vertex_with(false, v, assign, obstacles);
                if( w != NULL )
                {
                    try
                    {
                        astar_search<
                                undirected_graph_type,
                                undirected_graph_t,
                                undirected_vertex_index_t,
                                undirected_node_t
                                > (*g, v, w, metric);

                    }
                    catch( prx_found_goal_t fg )
                    {
                        undirected_vertex_index_t g_pred;
                        //        PRX_WARN_S("robot on node " << fg.index << " : " << assign.get_robot(fg.index));
                        for( undirected_vertex_index_t u = w; u != v; u = g->predecessors[u] )
                        {
                            g_pred = g->predecessors[u];
                            if( assign.has_robot_on(g_pred) )
                            {
                                int robot_id = assign.get_robot(g_pred);
                                //                PRX_ERROR_S("remove the robot " << robot_id << "   in position " << g_pred);
                                //                PRX_INFO_S("robot in u:" << u << " :  " << assign.get_robot(u));
                                PRX_DEBUG_S("pushing the robot " << robot_id << " from " << g_pred << "   to : " << u);
                                assign.remove_assignment(g_pred);
                                assign.add_assignment(u, robot_id);
                            }
                        }
                        return true;
                    }
                }
                return false;
            }

            int tree_feasibility_tester_t::find_closest_robot(undirected_vertex_index_t v, pebble_assignment_t& assign, const std::vector<undirected_vertex_index_t>& obstacles)
            {
                undirected_vertex_index_t u = find_closest_vertex_with(true, v, assign, obstacles);
                int robot_id = -1;
                if( u != NULL )
                {
                    robot_id = assign.get_robot(u);
                    assign.remove_assignment(u);
                }
                return robot_id;
            }

            void tree_feasibility_tester_t::update_node_info(undirected_vertex_index_t v, undirected_vertex_index_t u, pebble_assignment_t& assign)
            {
                tree_pebble_node_t* curr_v = g->get_vertex_as<tree_pebble_node_t > (v);
                tree_pebble_node_t* curr_u = g->get_vertex_as<tree_pebble_node_t > (u);
                bool u_occupied = assign.has_robot_on(u);

                PRX_WARN_S("update info for vertex : " << v << "   point:" << state_space->print_point(curr_v->point, 2));

                curr_v->num_trees = boost::degree(v, g->graph);

                if( boost::degree(v, g->graph) >= 3 )
                {
                    curr_v->trees[u].swap_dist = 0;
                    curr_v->trees[u].v_swap = v;
                }

                if( !u_occupied )
                    curr_v->trees[u].holes++;

                if( u_occupied || boost::degree(u, g->graph) > 2 )
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
                            curr_v->trees[u].swap_dist = curr_u->trees[t].swap_dist + g->weights[boost::edge(v, u, g->graph).first];
                        }

                        foreach(undirected_vertex_index_t w, curr_u->trees[t].vertices_in_tree)
                        {
                            curr_v->trees[u].vertices_in_tree.push_back(w);
                            if( g->get_vertex_as<tree_pebble_node_t > (w)->occupied || boost::degree(w, g->graph) > 2 )
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
                                //                    PRX_INFO_S("in the seen to check: " << w << "   -  " << g->get_vertex_as<tree_pebble_node_t > (w)->print_point(state_space) );

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
                PRX_DEBUG_S("info at the end : " << curr_v->print_info());
            }

            void tree_feasibility_tester_t::test_equivalent(tree_pebble_node_t* v_node, tree_pebble_node_t* u_node)
            {
                PRX_DEBUG_S("v:" << v_node->print_point(state_space, 2) << "  -  " << u_node->print_point(state_space, 2));
                try
                {
                    astar_search<
                            undirected_graph_type,
                            undirected_graph_t,
                            undirected_vertex_index_t,
                            undirected_node_t
                            > (*g, v_node->index, u_node->index, metric);
                }
                catch( prx_found_goal_t f_goal )
                {
                    PRX_DEBUG_S("found path");
                    for( undirected_vertex_index_t p = p = g->predecessors[u_node->index]; p != v_node->index; p = g->predecessors[p] )
                    {
                        PRX_DEBUG_S("path : " << p << "   deg: " << boost::degree(p, g->graph));
                        if( boost::degree(p, g->graph) > 2 )
                        {
                            PRX_WARN_S("the between w : " << g->get_vertex_as<tree_pebble_node_t > (p)->print_point(state_space, 2));
                            tree_pebble_node_t* p_node = g->get_vertex_as<tree_pebble_node_t > (p);
                            PRX_DEBUG_S("first:" << p_node->trees[p_node->tree_for_vertex[v_node->index]].holes << "  sec:" << p_node->trees[p_node->tree_for_vertex[u_node->index]].holes << "   min:" << PRX_MINIMUM(p_node->trees[p_node->tree_for_vertex[v_node->index]].holes, p_node->trees[p_node->tree_for_vertex[u_node->index]].holes));
                            PRX_DEBUG_S("OR : " << p_node->get_holes_except(v_node->index, u_node->index));
                            if( PRX_MINIMUM(p_node->trees[p_node->tree_for_vertex[v_node->index]].holes, p_node->trees[p_node->tree_for_vertex[u_node->index]].holes) > 0 || p_node->get_holes_except(v_node->index, u_node->index) > 0 )
                            {
                                PRX_DEBUG_S("v:" << state_space->print_point(v_node->point, 2) << "  -  " << state_space->print_point(u_node->point, 2) << " are equivalent criterion 1");
                                u_node->equiv_class = v_node->equiv_class;
                                return;
                            }
                        }
                    }
                }
                if( is_equivalent_criterion_2(v_node, u_node) || is_equivalent_criterion_2(u_node, v_node) )
                {
                    PRX_DEBUG_S("v:" << state_space->print_point(v_node->point, 2) << "  -  " << state_space->print_point(u_node->point, 2) << " are equivalent criterion 2");
                    u_node->equiv_class = v_node->equiv_class;
                    return;
                }
                if( boost::degree(v_node->index, g->graph) > 2 && is_equivalent_criterion_4(v_node, u_node) )
                {
                    u_node->equiv_class = v_node->equiv_class;
                    PRX_DEBUG_S("v:" << state_space->print_point(v_node->point, 2) << "  -  " << state_space->print_point(u_node->point, 2) << " are equivalent criterion 4");
                    return;
                }
                if( boost::degree(u_node->index, g->graph) > 2 && is_equivalent_criterion_4(u_node, v_node) )
                {
                    u_node->equiv_class = v_node->equiv_class;
                    PRX_DEBUG_S("v:" << state_space->print_point(v_node->point, 2) << "  -  " << state_space->print_point(u_node->point, 2) << " are equivalent criterion 4");
                    return;
                }
                return;
            }

            bool tree_feasibility_tester_t::is_equivalent_criterion_2(tree_pebble_node_t* v_node, tree_pebble_node_t* u_node)
            {
                undirected_vertex_index_t w = u_node->get_closest_swap_avoid(v_node->index);
                PRX_INFO_S("criterion 2 v:" << state_space->print_point(v_node->point, 2) << " - u:" << state_space->print_point(u_node->point, 2));
                if( w != NULL )
                {
                    PRX_WARN_S("the w : " << state_space->print_point(g->get_vertex_as<tree_pebble_node_t > (w)->point, 2));
                    undirected_vertex_index_t t = u_node->tree_for_vertex[w];
                    PRX_DEBUG_S("holes : " << u_node->trees[t].holes << "   dist+2: " << u_node->trees[t].swap_dist + 2);
                    if( u_node->trees[t].holes >= std::floor(u_node->trees[t].swap_dist) + 2 )
                        return true;
                }
                return false;
            }

            bool tree_feasibility_tester_t::is_equivalent_criterion_4(tree_pebble_node_t* v_node, tree_pebble_node_t* u_node)
            {
                undirected_vertex_index_t t = v_node->tree_for_vertex[u_node->index];
                PRX_DEBUG_S("criterion 4 : free_trees:" << v_node->num_free_trees << "    holes: " << v_node->get_holes_except(t, t));
                if( v_node->num_free_trees < 2 || v_node->get_holes_except(t, t) < 2 )
                    return false;

                return true;
            }

        }
    }
}
