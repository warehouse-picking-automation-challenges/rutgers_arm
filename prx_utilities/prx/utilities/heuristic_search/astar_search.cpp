/**
 * @file astar_search.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "prx/utilities/heuristic_search/astar_search.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/definitions/random.hpp"
#include <algorithm>
#include <boost/graph/connected_components.hpp>

namespace prx
{
    namespace util
    {
        const boost::default_color_type astar_search_t::WHITE = boost::white_color;
        const boost::default_color_type astar_search_t::GRAY = boost::gray_color;
        const boost::default_color_type astar_search_t::BLACK = boost::black_color;

        astar_search_t::astar_search_t()
        {
            distance_function = NULL;
            bounded_length = PRX_INFINITY;

            search_id = 0;
            lazy_search_id = 0;
        }

        astar_search_t::astar_search_t(undirected_graph_t *g)
        {
            link_graph(g);
        }

        astar_search_t::~astar_search_t()
        {
        }

        void astar_search_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
        {
            bounded_length = parameters::get_attribute_as<double>("bounded_length", reader, template_reader, PRX_INFINITY);
        }

        void astar_search_t::link_graph(undirected_graph_t *g)
        {
            graph = g;
        }

        void astar_search_t::link_distance_function( distance_t f )
        {
            PRX_DEBUG_COLOR("Distance function linked to A*", PRX_TEXT_GREEN);
            distance_function = f;
        }

        void astar_search_t::restart()
        {
            ++lazy_search_id;
            
            // foreach(undirected_edge_index_t e, blocked_edges)
            // {
            //     abstract_edge_t* edge = graph->get_edge_as<abstract_edge_t>(e);
            //     if( edge != NULL )
            //     {
            //         edge->blocked = false;
            //     }
            // }
            
            // blocked_edges.clear();
            
            // foreach(undirected_edge_index_t e, trusted_edges)
            // {
            //     abstract_edge_t* edge = graph->get_edge_as<abstract_edge_t>(e);
            //     if( edge != NULL )
            //     {
            //         edge->trusted = false;
            //     }
            // }
            // trusted_edges.clear();
        }

        bool astar_search_t::solve(const std::vector<undirected_vertex_index_t>& starts, const std::vector<undirected_vertex_index_t>& goals)
        {
            PRX_DEBUG_COLOR("A* Solve, length bound: " << bounded_length, PRX_TEXT_MAGENTA);
            
            search_id++;
            top_node = NULL;
            undirected_vertex_index_t current_vertex, target_vertex;
            undirected_edge_index_t e;
            
            open_set.clear();
            vertex_to_node.clear();

            // boost::connected_components(graph->graph,graph->components);
            bool same_connected_component = false;

            foreach(undirected_vertex_index_t v, starts)
            {
                if(in_same_connected_component(v, goals))
                {
                    double h = heuristic(v,goals);
                    initialize_vertex(v, v, 0, h);
                    openset_insert_node(v, v, 0, h);
                    same_connected_component = true;
                }
            }         

            if(!same_connected_component)
            {
                PRX_WARN_S("No solution!!! Starts and goals are not in the same connected components");
                return false;
            }   

            while( !open_set.empty() )
            {
                top_node = open_set.remove_min();
                current_vertex = top_node->vertex;
                if (vertex_to_node.find(current_vertex)!=vertex_to_node.end())
                    vertex_to_node.erase( current_vertex );

                if( check_if_goal_found( current_vertex, goals ) )
                {
                    finalize_search(current_vertex);
                    return true;
                }

                if( examine_vertex(current_vertex) )
                {
                    foreach(undirected_edge_index_t e, boost::out_edges(current_vertex, graph->graph))
                    {
                        target_vertex = boost::target(e, graph->graph);
                        PRX_ASSERT(current_vertex != target_vertex);
                        double h = heuristic( target_vertex, goals );
                        new_node_distance = graph->weights[e] + top_node->g;
                        if( new_node_distance + h < bounded_length && examine_edge(target_vertex, current_vertex, e, new_node_distance) )
                        {                                       
                            if( discover_vertex(target_vertex, current_vertex, new_node_distance, h))
                            {
                                update_vertex(target_vertex, current_vertex, new_node_distance);
                                
                                PRX_ASSERT( graph->predecessors[target_vertex] == current_vertex );
                            }
                        }
                    }
                }

                finish_vertex(current_vertex);
                delete top_node;
            }

            return false;
        }

        bool astar_search_t::solve(undirected_vertex_index_t start, const std::vector<undirected_vertex_index_t>& goals)
        {
            std::vector<undirected_vertex_index_t> starts(1);
            starts[0] = start;
            return solve(starts, goals);
        }

        bool astar_search_t::solve(undirected_vertex_index_t start, undirected_vertex_index_t goal)
        {
            std::vector<undirected_vertex_index_t> starts(1);
            starts[0] = start;
            std::vector<undirected_vertex_index_t> goals(1);
            goals[0] = goal;
            return solve(starts, goals);
        }

        void astar_search_t::set_bounded_length ( double new_bounded_length )
        {
            bounded_length = new_bounded_length;
        }

        void astar_search_t::block_edge(undirected_edge_index_t e)
        {
            graph->get_edge_as<abstract_edge_t>(e)->blocked_id = lazy_search_id;
        }
        
        void astar_search_t::trust_edge(undirected_edge_index_t e)
        {
            graph->get_edge_as<abstract_edge_t>(e)->trusted_id = lazy_search_id;
        }

        bool astar_search_t::is_edge_trusted( undirected_edge_index_t e )
        {
            return graph->get_edge_as<abstract_edge_t>(e)->trusted_id == lazy_search_id;
        }

        bool astar_search_t::is_edge_blocked( undirected_edge_index_t e )
        {
            return graph->get_edge_as<abstract_edge_t>(e)->blocked_id == lazy_search_id;
        }


        undirected_vertex_index_t astar_search_t::get_found_goal() const
        {
            return found_goal;
        }

        undirected_vertex_index_t astar_search_t::get_start_vertex() const
        {
            return found_start;
        }

        undirected_vertex_index_t astar_search_t::extract_path(std::deque<undirected_vertex_index_t>& vertices)
        {
            vertices.clear();

            undirected_vertex_index_t v;
            for( v = found_goal; v != graph->predecessors[v]; v = graph->predecessors[v] )
            {
                vertices.push_front(v);
            }
            vertices.push_front(v);

            return v;
        }

        astar_node_t* astar_search_t::generate_new_node(undirected_vertex_index_t vertex, undirected_vertex_index_t parent, double g, double h)
        {
            return new astar_node_t(vertex, g, h);
        }

        void astar_search_t::initialize_vertex(undirected_vertex_index_t vertex, undirected_vertex_index_t parent, double g, double h)
        {
            update_vertex(vertex, parent, g);
            graph->get_vertex_as<abstract_node_t>(vertex)->search_id = search_id;
        }

        bool astar_search_t::examine_vertex(undirected_vertex_index_t vertex)
        {
            // PRX_DEBUG_COLOR("Examine new vertex [" << graph->get_vertex_as<abstract_node_t>(vertex)->point->memory[0] << "," << graph->get_vertex_as<abstract_node_t>(vertex)->point->memory[1] << "," <<graph->get_vertex_as<abstract_node_t>(vertex)->point->memory[2] << "]", PRX_TEXT_BROWN);
            return true;
        }

        bool astar_search_t::discover_vertex(undirected_vertex_index_t vertex, undirected_vertex_index_t parent, double new_distance, double h) 
        { 
            /** We are in a new search **/
            if(graph->get_vertex_as<abstract_node_t>(vertex)->search_id != search_id)
            {
                /** Must add new node since new search **/
                initialize_vertex(vertex, parent, new_distance, h);
            }
            openset_insert_node(vertex, parent, new_distance, h);

            return true;
        }

        void astar_search_t::update_vertex(undirected_vertex_index_t vertex, undirected_vertex_index_t parent, double g)
        {
            graph->distances[vertex] = g;
            graph->predecessors[vertex] = parent;
        }

        void astar_search_t::finish_vertex(undirected_vertex_index_t vertex) {}

        bool astar_search_t::examine_edge(undirected_vertex_index_t vertex, undirected_vertex_index_t parent, undirected_edge_index_t e, double new_distance)
        {
            //if( std::find(blocked_edges.begin(), blocked_edges.end(), e) != blocked_edges.end() )
            if( is_edge_blocked( e ) )
            {
                // PRX_PRINT ("Blocked edge in examine edge: " << e, PRX_TEXT_BROWN);
                return false;
            }

            if(graph->get_vertex_as<abstract_node_t>(vertex)->search_id != search_id)
            {
                // PRX_PRINT ("Search ids mismatch in examine edge: " << e, PRX_TEXT_BROWN);
                return true;
            }

            return new_distance < graph->distances[vertex];
        }

        bool astar_search_t::check_if_goal_found(undirected_vertex_index_t potential_goal, const std::vector<undirected_vertex_index_t>& actual_goals)
        {
            return std::find(actual_goals.begin(), actual_goals.end(), potential_goal) != actual_goals.end();
        }

        void astar_search_t::finalize_search(undirected_vertex_index_t vertex)
        {
            found_goal = vertex;
            undirected_vertex_index_t v = found_goal;
            while(v != graph->predecessors[v])
            {
                // PRX_DEBUG_POINT("Predecessor loop: " << v);
                v = graph->predecessors[v];
            }
            found_start = v;
        }

        astar_node_t* astar_search_t::openset_insert_node(undirected_vertex_index_t vertex, undirected_vertex_index_t parent, double g, double h)
        {
            astar_node_t* node = NULL;
            if( vertex_to_node.find(vertex) != vertex_to_node.end() )
            {
                node = vertex_to_node[vertex];
                update_vertex( vertex, parent, g );
                node->f = g + h;
                open_set.update(node, node);
            }
            else
            {
                node = generate_new_node(vertex, parent, g, h);
                vertex_to_node[vertex] = node;
                open_set.insert(node);
            }            
            return node;
        }

        double astar_search_t::heuristic(undirected_vertex_index_t current, undirected_vertex_index_t goal)
        {
            if( distance_function != NULL )
            {
                undirected_node_t *s, *t;
                s = graph->get_vertex_as<undirected_node_t > (current);
                t = graph->get_vertex_as<undirected_node_t > (goal);
                return distance_function( s->point, t->point );
            }
            else
            {
                PRX_DEBUG_COLOR("No heuristic linked to the heuristic search.", PRX_TEXT_BROWN);
            }
            return 0;
        }

        double astar_search_t::heuristic(undirected_vertex_index_t current, const std::vector<undirected_vertex_index_t>& goals)
        {
            double min_h, h;
            size_t i;

            PRX_ASSERT( goals.size() > 0 );
            min_h = heuristic(current, goals[0]);

            for( i = 1; i < goals.size(); ++i )
            {
                h = heuristic(current, goals[i]);
                if( h < min_h )
                    min_h = h;
            }

            return min_h;
        }

        bool astar_search_t::in_same_connected_component(const undirected_vertex_index_t vertex, const std::vector<undirected_vertex_index_t>& vertices)
        {
            foreach(undirected_vertex_index_t u, vertices)
            {
                if(graph->components[vertex] == graph->components[u])
                    return true;
            }
            return false;
        }

    }
}

