/**
 * @file pebble_spanning_tree.hpp
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

#ifndef PRX_PEBBLE_SPANNING_TREE_HPP
#define	PRX_PEBBLE_SPANNING_TREE_HPP


#include "prx/utilities/definitions/defs.hpp"
#include "utilities/pebble_assignment.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"
#include "utilities/allocated_heap.hpp"

#include <deque>


namespace prx
{
    namespace packages
    {
        namespace pebble_motion
        {

            class spanning_tree_node_t : public util::undirected_node_t
            {

              public:
                util::undirected_vertex_index_t graph_index;

                void init_node(util::undirected_vertex_index_t g_index)
                {
                    graph_index = g_index;
                }

                virtual bool has_children(util::undirected_graph_t* graph)
                {
                    //        PRX_WARN_S(index << ":  has " << boost::out_degree(index,graph->graph) << "   children");
                    return (boost::out_degree(index, graph->graph) > 1);
                }

                virtual util::undirected_vertex_index_t get_parent(util::undirected_graph_t* graph)
                {
                    //        PRX_ASSERT(!has_children(graph));
                    if( boost::out_degree(index, graph->graph) == 0 )
                        return index;

                    return *boost::adjacent_vertices(index, graph->graph).first;
                }
            };

            typedef std::pair<int, std::pair<util::undirected_vertex_index_t, util::undirected_vertex_index_t> > tree_path_step_t;

            /**
             * Computes a spanning tree for the pebble solver
             */
            class pebble_spanning_tree_t
            {

              public:
                pebble_spanning_tree_t();
                virtual ~pebble_spanning_tree_t();

                void copy(const pebble_spanning_tree_t* tree);

                void compute_spanning_tree(const util::undirected_graph_t* g);
                void get_leaves(std::deque<util::undirected_vertex_index_t>& leaves);
                void get_leaves(allocated_heap_t<util::undirected_vertex_index_t>& leaves);
                int find_closest_robot(util::undirected_vertex_index_t v, pebble_assignment_t& assign);
                util::undirected_vertex_index_t find_closest_vertex(util::undirected_vertex_index_t v, pebble_assignment_t& assign);
                std::pair<util::undirected_vertex_index_t, int> find_closest_assignment(util::undirected_vertex_index_t v, pebble_assignment_t& assign);
                bool push(util::undirected_vertex_index_t v, pebble_assignment_t& assign, std::vector<tree_path_step_t>* path = NULL);

                int num_vertices();

                bool has_children(util::undirected_vertex_index_t v);

                util::undirected_vertex_index_t get_graph_index(util::undirected_vertex_index_t v);
                util::undirected_vertex_index_t get_parent(util::undirected_vertex_index_t v);

                void remove_vertex(util::undirected_vertex_index_t v);

                util::undirected_graph_t* spanning_tree;
            };

        }
    }
}

#endif	// PRX_PEBBLE_SPANNING_TREE_HPP
