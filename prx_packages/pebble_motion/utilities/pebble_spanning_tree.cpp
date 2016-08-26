/**
 * @file pebble_spanning_tree->cpp
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

#include "utilities/pebble_spanning_tree.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"

#include <boost/graph/random_spanning_tree.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/graph/depth_first_search.hpp>

namespace prx
{
    using namespace util;

    namespace packages
    {
        namespace pebble_motion
        {

            struct prx_found_vertex
            {

                undirected_vertex_index_t index;

                prx_found_vertex(undirected_vertex_index_t v)
                {
                    index = v;
                }
            };

            class prx_dijkstra_visitor_t : public boost::default_dijkstra_visitor
            {

              public:

                prx_dijkstra_visitor_t(undirected_graph_t* g, pebble_assignment_t* assign, bool with_robot)
                {
                    inner_graph = g;
                    inner_assign = assign;
                    has_robot = with_robot;
                }

                template <class undirected_graph_t>
                void examine_vertex(undirected_vertex_index_t v, undirected_graph_t& g)
                {
                    undirected_vertex_index_t assign_v = inner_graph->get_vertex_as<spanning_tree_node_t > (v)->graph_index;
                    if( inner_assign->has_robot_on(assign_v) == has_robot )
                    {
                        throw prx_found_vertex(v);
                    }
                }

              private:
                undirected_graph_t* inner_graph;
                pebble_assignment_t* inner_assign;
                bool has_robot;
            };

            class prx_spanning_tree_visitor_t : public boost::default_dfs_visitor
            {

              public:

                prx_spanning_tree_visitor_t()
                {
                    //        PRX_INFO_S("create prx_spanning_tree_visitor");
                    tree_edges = new std::vector<undirected_edge_index_t > ();
                }

                ~prx_spanning_tree_visitor_t() {
                    //        PRX_ERROR_S("DEstructor prx_spanning_tree_visitor");
                }

                void clear()
                {
                    //        PRX_WARN_S("Delete prx_spanning_tree_visitor");
                    tree_edges->clear();
                    delete tree_edges;
                }

                template <class undirected_graph_t>
                void tree_edge(undirected_edge_index_t e, undirected_graph_t& g)
                {
                    //        PRX_DEBUG_S("add e: " << e);
                    tree_edges->push_back(e);
                }

                std::vector<undirected_edge_index_t>* tree_edges;
            };

            pebble_spanning_tree_t::pebble_spanning_tree_t()
            {
                spanning_tree = new undirected_graph_t();
            }

            pebble_spanning_tree_t::~pebble_spanning_tree_t() { }

            void pebble_spanning_tree_t::copy(const pebble_spanning_tree_t* tree)
            {
                hash_t<undirected_vertex_index_t, undirected_vertex_index_t> same_nodes;

                foreach(undirected_vertex_index_t v, boost::vertices(tree->spanning_tree->graph))
                {
                    undirected_vertex_index_t v_new = spanning_tree->add_vertex<spanning_tree_node_t > ();
                    spanning_tree->get_vertex_as<spanning_tree_node_t > (v_new)->init_node(tree->spanning_tree->get_vertex_as<spanning_tree_node_t > (v)->graph_index);
                    same_nodes[v] = v_new;
                }

                foreach(undirected_edge_index_t e, boost::edges(tree->spanning_tree->graph))
                {
                    undirected_vertex_index_t v1 = same_nodes[boost::source(e, tree->spanning_tree->graph)];
                    undirected_vertex_index_t v2 = same_nodes[boost::target(e, tree->spanning_tree->graph)];

                    spanning_tree->add_edge<undirected_edge_t > (v1, v2, tree->spanning_tree->weights(e));
                }
            }

            void pebble_spanning_tree_t::compute_spanning_tree(const undirected_graph_t* g)
            {
                std::vector < undirected_edge_index_t > tree;
                hash_t<undirected_vertex_index_t, undirected_vertex_index_t> same_nodes;
                undirected_vertex_index_t s, t;
                undirected_vertex_index_t s_new, t_new;

                prx_spanning_tree_visitor_t* tree_visitor = new prx_spanning_tree_visitor_t();


                //    PRX_DEBUG_S("graph v: " << boost::num_vertices(g->graph) << "   e:" << boost::num_edges(g->graph));
                depth_first_search<
                        undirected_graph_type,
                        undirected_graph_t,
                        undirected_vertex_index_t,
                        undirected_edge_index_t,
                        prx_spanning_tree_visitor_t
                        > (g, tree_visitor);

                //    PRX_DEBUG_S("tree_edges size : " << tree_visitor->tree_edges->size());

                foreach(undirected_edge_index_t e, *(tree_visitor->tree_edges))
                {
                    s = boost::source(e, g->graph);
                    t = boost::target(e, g->graph);

                    if( same_nodes.find(s) == same_nodes.end() )
                    {
                        s_new = spanning_tree->add_vertex<spanning_tree_node_t > ();
                        //            PRX_INFO_S("add new node in the tree: " << s_new << "    for v:" << s);
                        spanning_tree->get_vertex_as<spanning_tree_node_t > (s_new)->init_node(s);
                        same_nodes[s] = s_new;
                    }
                    else
                        s_new = same_nodes[s];

                    if( same_nodes.find(t) == same_nodes.end() )
                    {
                        t_new = spanning_tree->add_vertex<spanning_tree_node_t > ();
                        //            PRX_INFO_S("add new node in the tree: " << t_new << "    for v:" << t);
                        spanning_tree->get_vertex_as<spanning_tree_node_t > (t_new)->init_node(t);
                        same_nodes[t] = t_new;
                    }
                    else
                        t_new = same_nodes[t];

                    spanning_tree->add_edge<undirected_edge_t > (s_new, t_new, g->weights(e));
                }

                tree_visitor->clear();
                delete tree_visitor;
            }

            void pebble_spanning_tree_t::get_leaves(std::deque<undirected_vertex_index_t>& leaves)
            {

                foreach(undirected_vertex_index_t v, boost::vertices(spanning_tree->graph))
                {
                    if( !spanning_tree->get_vertex_as<spanning_tree_node_t > (v)->has_children(spanning_tree) )
                        leaves.push_back(v);
                }
            }

            void pebble_spanning_tree_t::get_leaves(allocated_heap_t<undirected_vertex_index_t>& leaves)
            {

                foreach(undirected_vertex_index_t v, boost::vertices(spanning_tree->graph))
                {
                    if( !spanning_tree->get_vertex_as<spanning_tree_node_t > (v)->has_children(spanning_tree) )
                        leaves.push_back(v);
                }
            }

            int pebble_spanning_tree_t::find_closest_robot(undirected_vertex_index_t v, pebble_assignment_t& assign)
            {
                int robot_id = -1;
                prx_dijkstra_visitor_t* visitor = new prx_dijkstra_visitor_t(spanning_tree, &assign, true);
                try
                {
                    dijkstra_shortest_paths<
                            undirected_graph_type,
                            undirected_graph_t,
                            undirected_vertex_index_t,
                            prx_dijkstra_visitor_t
                            > (spanning_tree, v, visitor);

                }
                catch( prx_found_vertex fg )
                {
                    undirected_vertex_index_t assign_v = spanning_tree->get_vertex_as<spanning_tree_node_t > (fg.index)->graph_index;
                    robot_id = assign.get_robot(assign_v);
                    assign.remove_assignment(assign_v);

                }

                delete visitor;
                return robot_id;
            }

            undirected_vertex_index_t pebble_spanning_tree_t::find_closest_vertex(undirected_vertex_index_t v, pebble_assignment_t& assign)
            {
                undirected_vertex_index_t assign_v = NULL;
                prx_dijkstra_visitor_t* visitor = new prx_dijkstra_visitor_t(spanning_tree, &assign, true);
                try
                {
                    dijkstra_shortest_paths<
                            undirected_graph_type,
                            undirected_graph_t,
                            undirected_vertex_index_t,
                            prx_dijkstra_visitor_t
                            > (spanning_tree, v, visitor);
                }
                catch( prx_found_vertex fg )
                {
                    assign_v = spanning_tree->get_vertex_as<spanning_tree_node_t > (fg.index)->graph_index;
                    assign.remove_assignment(assign_v);

                }

                delete visitor;
                return assign_v;
            }

            std::pair<undirected_vertex_index_t, int> pebble_spanning_tree_t::find_closest_assignment(undirected_vertex_index_t v, pebble_assignment_t& assign)
            {
                undirected_vertex_index_t assign_v = NULL;
                int robot_id = -1;
                prx_dijkstra_visitor_t* visitor = new prx_dijkstra_visitor_t(spanning_tree, &assign, true);

                try
                {
                    dijkstra_shortest_paths<
                            undirected_graph_type,
                            undirected_graph_t,
                            undirected_vertex_index_t,
                            prx_dijkstra_visitor_t
                            > (spanning_tree, v, visitor);
                }
                catch( prx_found_vertex fg )
                {
                    assign_v = spanning_tree->get_vertex_as<spanning_tree_node_t > (fg.index)->graph_index;
                    robot_id = assign.get_robot(assign_v);
                    assign.remove_assignment(assign_v);

                }

                delete visitor;
                return std::make_pair(assign_v, robot_id);
            }

            bool pebble_spanning_tree_t::push(undirected_vertex_index_t v, pebble_assignment_t& assign, std::vector<tree_path_step_t>* path)
            {
                //    PRX_INFO_S("going for push for the robot : " << get_robot(v));
                //    foreach(undirected_vertex_index_t vtree, boost::vertices(spanning_tree->graph))
                //    PRX_INFO_S("v_tree : " << vtree << "   indices: " << spanning_tree->indices[vtree]);
                //    
                //    foreach(undirected_edge_index_t etree, boost::edges(spanning_tree->graph))
                //    PRX_INFO_S("edge :" << etree);

                prx_dijkstra_visitor_t* visitor = new prx_dijkstra_visitor_t(spanning_tree, &assign, false);
                try
                {
                    dijkstra_shortest_paths<
                            undirected_graph_type,
                            undirected_graph_t,
                            undirected_vertex_index_t,
                            prx_dijkstra_visitor_t
                            > (spanning_tree, v, visitor);

                }
                catch( prx_found_vertex fg )
                {
                    undirected_vertex_index_t g_pred;
                    undirected_vertex_index_t g_u;
                    for( undirected_vertex_index_t u = fg.index; u != v; u = spanning_tree->predecessors[u] )
                    {

                        g_pred = spanning_tree->get_vertex_as<spanning_tree_node_t > (spanning_tree->predecessors[u])->graph_index;
                        g_u = spanning_tree->get_vertex_as<spanning_tree_node_t > (u)->graph_index;
                        if( assign.has_robot_on(g_pred) )
                        {
                            int robot_id = assign.get_robot(g_pred);
                            if( path != NULL )
                            {
                                path->push_back(std::make_pair(robot_id, std::make_pair(g_pred, g_u)));
                            }
                            assign.remove_assignment(g_pred);
                            assign.add_assignment(g_u, robot_id);
                        }
                    }
                    delete visitor;
                    return true;
                }
                delete visitor;
                return false;

            }

            int pebble_spanning_tree_t::num_vertices()
            {
                return boost::num_vertices(spanning_tree->graph);
            }

            bool pebble_spanning_tree_t::has_children(undirected_vertex_index_t v)
            {
                return spanning_tree->get_vertex_as<spanning_tree_node_t > (v)->has_children(spanning_tree);
            }

            undirected_vertex_index_t pebble_spanning_tree_t::get_graph_index(undirected_vertex_index_t v)
            {
                return spanning_tree->get_vertex_as<spanning_tree_node_t > (v)->graph_index;
            }

            undirected_vertex_index_t pebble_spanning_tree_t::get_parent(undirected_vertex_index_t v)
            {
                return spanning_tree->get_vertex_as<spanning_tree_node_t > (v)->get_parent(spanning_tree);
            }

            void pebble_spanning_tree_t::remove_vertex(undirected_vertex_index_t v)
            {
                spanning_tree->clear_and_remove_vertex(v);
            }


        }
    }
}
