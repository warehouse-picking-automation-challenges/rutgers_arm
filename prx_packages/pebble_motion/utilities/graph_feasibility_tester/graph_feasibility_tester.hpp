/**
 * @file graph_feasibility_tester.hpp
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

#ifndef PRX_GRAPH_FEASIBILITY_TESTER_HPP
#define	PRX_GRAPH_FEASIBILITY_TESTER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "utilities/tree_feasibility_tester/tree_feasibility_tester.hpp"

namespace prx
{
    namespace packages
    {
        namespace pebble_motion
        {

            /**
             * A brief description of this class (no tag required).
             * 
             * A more detailed description of this class with multiple paragraphs delimited by blank lines. 
             */
            class graph_feasibility_tester_t : public tree_feasibility_tester_t
            {

              public:
                graph_feasibility_tester_t();
                virtual ~graph_feasibility_tester_t();

                virtual util::undirected_vertex_index_t add_new_vertex(util::undirected_graph_t* graph);

                virtual bool pebble_test(util::undirected_graph_t* graph);

              protected:

                util::hash_t<util::undirected_vertex_index_t, util::undirected_vertex_index_t> to_tree;
                util::hash_t<util::undirected_vertex_index_t, util::undirected_vertex_index_t> to_graph;

                virtual void reduce_to_tree(util::undirected_graph_t* graph);
                virtual void update_node_info(util::undirected_vertex_index_t v, util::undirected_vertex_index_t u, pebble_assignment_t& assign);
                //    virtual bool is_equivalent_criterion_2(tree_pebble_node_t* v_node, tree_pebble_node_t* u_node);
            };

        }
    }
}

#endif	// PRX_GRAPH_FEASIBILITY_TESTER_HPP
