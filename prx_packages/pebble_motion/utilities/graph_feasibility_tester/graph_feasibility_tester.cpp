/**
 * @file graph_feasibility_tester.cpp
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

#include "utilities/graph_feasibility_tester/graph_feasibility_tester.hpp"
#include "utilities/graph_feasibility_tester/graph_pebble_graph.hpp"
#include "prx/utilities/boost/hash.hpp"

#include <boost/graph/biconnected_components.hpp>

#include <pluginlib/class_list_macros.h>
#include <boost/graph/subgraph.hpp>

PLUGINLIB_EXPORT_CLASS( prx::packages::pebble_motion::graph_feasibility_tester_t, prx::packages::pebble_motion::pebble_tester_t)

namespace prx
{
    using namespace util;
    
    namespace packages
    {
        namespace pebble_motion
        {

            graph_feasibility_tester_t::graph_feasibility_tester_t()
            {
                g = new undirected_graph_t();
            }

            graph_feasibility_tester_t::~graph_feasibility_tester_t()
            {
                delete g;
            }

            bool graph_feasibility_tester_t::pebble_test(undirected_graph_t* graph)
            {
                reduce_to_tree(graph);
                return tree_feasibility_tester_t::pebble_test(g);
            }

            undirected_vertex_index_t graph_feasibility_tester_t::add_new_vertex(undirected_graph_t* graph)
            {
                return graph->add_vertex<graph_pebble_node_t > ();
            }

            void graph_feasibility_tester_t::reduce_to_tree(undirected_graph_t* graph)
            {
                hash_t<int, undirected_vertex_index_t> new_nodes;
                undirected_vertex_index_t v_transshipment;
                undirected_vertex_index_t v_graph_s;
                undirected_vertex_index_t v_graph_t;
                undirected_vertex_index_t v_source;
                undirected_vertex_index_t v_target;
                undirected_edge_index_t e_new;
                int robot_id;
                unsigned int num_comps = boost::biconnected_components(graph->graph, graph->edge_components);
                std::vector<int> b_comp_size;
                b_comp_size.resize(num_comps);

                for( unsigned int i = 0; i < num_comps; ++i )
                    b_comp_size[i] = 0;

                foreach(undirected_edge_index_t e, boost::edges(graph->graph))
                {
                    b_comp_size[graph->edge_components[e]]++;
                }

                for( unsigned int i = 0; i < num_comps; ++i )
                    PRX_INFO_S("component[" << i << "] :" << b_comp_size[i]);

                PRX_DEBUG_S("BEFORE)     v:" << boost::num_vertices(graph->graph) << "    e:" << boost::num_edges(graph->graph));

                std::vector<double> vec_point(state_space->get_dimension());

                foreach(undirected_edge_index_t e, boost::edges(graph->graph))
                {
                    int edge_component_index = graph->edge_components[e];
                    v_source = boost::source(e, graph->graph);
                    v_target = boost::target(e, graph->graph);

                    if( to_tree.find(v_source) == to_tree.end() )
                    {
                        v_graph_s = add_new_vertex(g);
                        g->get_vertex_as<undirected_node_t > (v_graph_s)->point = state_space->clone_point(graph->get_vertex_as<undirected_node_t > (v_source)->point);
                        to_tree[v_source] = v_graph_s;
                        to_graph[v_graph_s] = v_source;

                        if( s_assignment.has_robot_on(v_source) )
                        {
                            robot_id = s_assignment.get_robot(v_source);
                            s_assignment.remove_assignment(v_source);
                            s_assignment.add_assignment(v_graph_s, robot_id);
                        }

                        if( t_assignment.has_robot_on(v_source) )
                        {
                            robot_id = t_assignment.get_robot(v_source);
                            t_assignment.remove_assignment(v_source);
                            t_assignment.add_assignment(v_graph_s, robot_id);
                        }
                    }
                    else
                    {
                        v_graph_s = to_tree[v_source];
                    }

                    if( to_tree.find(v_target) == to_tree.end() )
                    {
                        v_graph_t = add_new_vertex(g);
                        g->get_vertex_as<undirected_node_t > (v_graph_t)->point = state_space->clone_point(graph->get_vertex_as<undirected_node_t > (v_target)->point);
                        to_tree[v_target] = v_graph_t;
                        to_graph[v_graph_t] = v_target;

                        if( s_assignment.has_robot_on(v_target) )
                        {
                            robot_id = s_assignment.get_robot(v_target);
                            s_assignment.remove_assignment(v_target);
                            s_assignment.add_assignment(v_graph_t, robot_id);
                        }

                        if( t_assignment.has_robot_on(v_target) )
                        {
                            robot_id = t_assignment.get_robot(v_target);
                            t_assignment.remove_assignment(v_target);
                            t_assignment.add_assignment(v_graph_t, robot_id);
                        }
                    }
                    else
                    {
                        v_graph_t = to_tree[v_target];
                    }

                    if( b_comp_size[edge_component_index] > 1 )
                    {
                        if( new_nodes.find(edge_component_index) == new_nodes.end() )
                        {
                            v_transshipment = add_new_vertex(g);
                            g->get_vertex_as<undirected_node_t > (v_transshipment)->point = state_space->alloc_point();
                            g->get_vertex_as<graph_pebble_node_t > (v_transshipment)->transshipment = true;
                            new_nodes[edge_component_index] = v_transshipment;

                            e_new = g->add_edge<pebble_test_edge_t > (v_transshipment, v_graph_s, 0.5);
                            PRX_DEBUG_S("new e#1: " << e_new);
                            g->get_edge_as<pebble_test_edge_t > (e_new)->steps = 50;
                            e_new = g->add_edge<pebble_test_edge_t > (v_transshipment, v_graph_t, 0.5);
                            PRX_DEBUG_S("new e#1: " << e_new);
                            g->get_edge_as<pebble_test_edge_t > (e_new)->steps = 50;


                        }
                        else
                        {
                            v_transshipment = new_nodes[edge_component_index];
                            if( !boost::edge(v_transshipment, v_graph_s, g->graph).second )
                            {
                                e_new = g->add_edge<pebble_test_edge_t > (v_transshipment, v_graph_s, 0.5);
                                PRX_INFO_S("new e#2: " << e_new);
                                g->get_edge_as<pebble_test_edge_t > (e_new)->steps = 50;
                            }
                            if( !boost::edge(v_transshipment, v_graph_t, g->graph).second )
                            {
                                e_new = g->add_edge<pebble_test_edge_t > (v_transshipment, v_graph_t, 0.5);
                                g->get_edge_as<pebble_test_edge_t > (e_new)->steps = 50;
                                PRX_WARN_S("new e#3: " << e_new);
                            }
                        }
                        for( unsigned int i = 0; i < state_space->get_dimension(); ++i )
                        {
                            g->get_vertex_as<undirected_node_t > (v_transshipment)->point->at(i) = g->get_vertex_as<undirected_node_t > (v_transshipment)->point->at(i) + (g->get_vertex_as<undirected_node_t > (v_graph_s)->point->at(i) + g->get_vertex_as<undirected_node_t > (v_graph_t)->point->at(i)) / 2;
                        }
                    }
                    else
                    {
                        e_new = g->add_edge<pebble_test_edge_t > (v_graph_s, v_graph_t, 1);
                        g->get_edge_as<pebble_test_edge_t > (e_new)->steps = graph->get_edge_as<pebble_test_edge_t > (e)->steps;
                    }

                }

                PRX_WARN_S("FINAL)      v:" << boost::num_vertices(g->graph) << "    e:" << boost::num_edges(g->graph));
                int g_pos = 1;

                foreach(undirected_vertex_index_t v, boost::vertices(g->graph))
                {
                    PRX_DEBUG_S(g_pos << ") " << v << "  :  " << g->get_vertex_as<graph_pebble_node_t > (v)->print_point(state_space));
                    g_pos++;
                }

                foreach(undirected_edge_index_t e, boost::edges(g->graph))
                {
                    PRX_INFO_S(e);
                }
            }

            void graph_feasibility_tester_t::update_node_info(undirected_vertex_index_t v, undirected_vertex_index_t u, pebble_assignment_t& assign)
            {
                graph_pebble_node_t* curr_v = g->get_vertex_as<graph_pebble_node_t > (v);
                graph_pebble_node_t* curr_u = g->get_vertex_as<graph_pebble_node_t > (u);
                bool u_occupied = assign.has_robot_on(u);

                PRX_WARN_S("update info for vertex : " << v << "   point:" << state_space->print_point(curr_v->point, 2));

                curr_v->num_trees = boost::degree(v, g->graph);

                if( boost::degree(v, g->graph) >= 3 )
                {
                    curr_v->trees[u].swap_dist = 0;
                    curr_v->trees[u].v_swap = v;
                }

                if( !u_occupied && !curr_u->transshipment )
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
                        }

                        if( !u_occupied )
                        {

                            foreach(undirected_vertex_index_t w, curr_u->trees[t].tree_seen)
                            {
                                curr_v->seen.push_back(w);
                                curr_v->trees[u].tree_seen.push_back(w);
                                if( g->get_vertex_as<graph_pebble_node_t > (w)->occupied || boost::degree(w, g->graph) > 2 )
                                {
                                    curr_v->tree_for_vertex[w] = u;
                                }
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

        }
    }
}




