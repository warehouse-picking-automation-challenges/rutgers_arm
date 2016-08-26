/**
 * @file kornhauser_graph.hpp
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
#pragma once

#ifndef PRX_KORNHAUSER_GRAPH_HPP
#define	PRX_KORNHAUSER_GRAPH_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"
#include "utilities/pebble_assignment.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"

#include <boost/range/adaptor/map.hpp>

namespace prx
{
    namespace packages
    {
        namespace pebble_motion
        {

            struct pebble_subproblem_t
            {

                util::undirected_graph_t* graph;
                pebble_assignment_t s_assign;
                pebble_assignment_t e_assign;

                pebble_subproblem_t(){ }

                pebble_subproblem_t(util::undirected_graph_t * g)
                {
                    graph = g;
                }

                size_t robot_size() const
                {
                    return s_assign.size();
                }

                bool has_same_assignments()
                {
                    return s_assign.has_same_robots(e_assign);
                }

                void print() const
                {
                    PRX_WARN_S("-----------  PROBLEM --------------");
                    PRX_INFO_S("graph has v:" << boost::num_vertices(graph->graph) << "  and  e:" << boost::num_edges(graph->graph));
                    s_assign.print();
                    e_assign.print();
                    PRX_WARN_S("-----------------------------------");
                }

            };

            class info_node_t : public util::undirected_node_t
            {

              public:
                util::undirected_vertex_index_t graph_index;
            };

            class pebble_node_t : public util::undirected_node_t
            {

              public:
                util::undirected_vertex_index_t graph_index;
                std::vector<int> biconnected_classes;
                std::vector<int> graph_classes;
                std::vector<int> plank_classes;
                const util::space_t* state_space;
                bool connector;

                ~pebble_node_t()
                {
                    state_space->free_point(point);
                    biconnected_classes.clear();
                    graph_classes.clear();
                }

                void init_node(util::undirected_vertex_index_t g_index, const util::space_t* space, const util::space_point_t* state)
                {
                    graph_index = g_index;
                    connector = false;
                    state_space = space;
                    point = state_space->clone_point(state);
                }

                pebble_node_t& operator =(const pebble_node_t& node)
                {
                    graph_index = node.graph_index;
                    state_space = node.state_space;
                    point = state_space->clone_point(node.point);
                    connector = node.connector;

                    foreach(int c, node.biconnected_classes)
                    {
                        biconnected_classes.push_back(c);
                    }

                    //        foreach(int c, node.graph_classes)
                    //        {
                    //            graph_classes.push_back(c);
                    //        }       

                    return (*this);
                }

                bool add_class(util::undirected_graph_t* graph, int no_class)
                {
                    if( !has_class(no_class) )
                    {
                        biconnected_classes.push_back(no_class);
                        if( boost::out_degree(index, graph->graph) > 2 && biconnected_classes.size() > 1 )
                            connector = true;
                        return true;
                    }
                    return false;
                }

                bool add_graph_class(int no_class)
                {
                    if( !has_graph_class(no_class) )
                    {
                        graph_classes.push_back(no_class);
                        return true;
                    }
                    return false;
                }

                bool add_plank_class(int no_class)
                {
                    if( !has_plank_class(no_class) )
                    {
                        plank_classes.push_back(no_class);
                        return true;
                    }
                    return false;
                }

                bool on_plank() const
                {
                    return graph_classes.size() == 0;
                }

                bool on_graph_plank() const
                {
                    return plank_classes.size() != 0;
                }

                bool has_class(int no_class) const
                {
                    return std::find(biconnected_classes.begin(), biconnected_classes.end(), no_class) != biconnected_classes.end();
                }

                bool has_graph_class(int no_class) const
                {
                    return std::find(graph_classes.begin(), graph_classes.end(), no_class) != graph_classes.end();
                }

                bool has_plank_class(int no_class) const
                {
                    return std::find(plank_classes.begin(), plank_classes.end(), no_class) != plank_classes.end();
                }

                int get_valance(util::undirected_graph_t* graph) const
                {
                    return boost::out_degree(index, graph->graph);
                }

                bool is_connector() const
                {
                    return connector;
                }

                std::vector<int> get_classes() const
                {
                    return biconnected_classes;
                }

                std::vector<int> get_graph_classes() const
                {
                    return graph_classes;
                }

                std::vector<int> get_plank_classes() const
                {
                    return plank_classes;
                }

                std::string print_point(int prec = 3) const
                {
                    return state_space->print_point(point, prec);
                }


            };

            struct found_closest_t
            {

                found_closest_t(util::undirected_vertex_index_t v)
                {
                    index = v;
                }


                util::undirected_vertex_index_t index;
            };

            class pebble_distance_heuristic_t : public boost::astar_heuristic<util::undirected_graph_type, double>
            {

              public:

                pebble_distance_heuristic_t(const util::undirected_graph_t* graph, util::undirected_vertex_index_t goal, const util::space_t* state_space)
                {
                    inner_graph = graph;
                    goal_node = graph->operator[](goal);
                    space = state_space;
                }

                pebble_distance_heuristic_t(const util::undirected_graph_t* graph, const util::space_t* state_space)
                {
                    inner_graph = graph;
                    space = state_space;
                }

                pebble_distance_heuristic_t(const util::space_t* state_space)
                {
                    space = state_space;
                }

                void set_new_graph(const util::undirected_graph_t* graph)
                {
                    inner_graph = graph;
                }

                void set_new_goal(util::undirected_vertex_index_t goal)
                {
                    goal_node = inner_graph->operator[](goal);
                    ;
                }

                virtual ~pebble_distance_heuristic_t(){ }

                virtual double operator()(util::undirected_vertex_index_t u)
                {
                    return space->distance(inner_graph->operator[](u)->point, goal_node->point);
                }

              private:
                util::undirected_node_t* goal_node;
                const util::space_t* space;
                const util::undirected_graph_t * inner_graph;
            };

            class pebble_astar_goal_visitor_t : public boost::default_astar_visitor
            {

              public:

                pebble_astar_goal_visitor_t(){ }

                pebble_astar_goal_visitor_t(util::undirected_vertex_index_t goal) : m_goal(goal){ }

                ~pebble_astar_goal_visitor_t(){ }

                void set_new_goal(util::undirected_vertex_index_t goal)
                {
                    m_goal = goal;
                }

                template <class Graph>
                void examine_vertex(util::undirected_vertex_index_t u, Graph& graph)
                {
                    if( u == m_goal )
                        throw util::prx_found_goal_t();
                }

              private:
                util::undirected_vertex_index_t m_goal;
            };

            class pebble_bfs_visitor_t : public boost::default_bfs_visitor
            {

              public:

                pebble_bfs_visitor_t(){ }

                ~pebble_bfs_visitor_t(){ }

                void add_connectors(util::undirected_vertex_index_t v)
                {
                    m_goal.push_back(v);
                }

                template <class Graph>
                void examine_vertex(util::undirected_vertex_index_t u, Graph& graph)
                {
                    if( std::find(m_goal.begin(), m_goal.end(), u) != m_goal.end() )
                        throw found_closest_t(u);
                }

              private:
                std::vector<util::undirected_vertex_index_t> m_goal;
            };




        }
    }
}

#endif	// PRX_KORNHAUSER_GRAPH_HPP
