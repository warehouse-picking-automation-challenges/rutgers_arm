/**
 * @file tree_feasibility_tester.hpp
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

#ifndef PRX_TREE_FEASIBILITY_TESTER_HPP
#define	PRX_TREE_FEASIBILITY_TESTER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "utilities/tree_feasibility_tester/tree_pebble_graph.hpp"
#include "utilities/pebble_tester.hpp"

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
            class tree_feasibility_tester_t : public pebble_tester_t
            {

              public:
                tree_feasibility_tester_t();
                virtual ~tree_feasibility_tester_t();

                virtual util::undirected_vertex_index_t add_new_vertex(util::undirected_graph_t* graph);

                /** @copydoc pebble_tester_t::reduce(pebble_assignment_t&, pebble_assignment_t&, pebble_assignment_t&)  */
                virtual bool reduce(pebble_assignment_t& s_new, pebble_assignment_t& s_assign, pebble_assignment_t& t_assign);

                virtual bool pebble_test(util::undirected_graph_t* graph);

              protected:


                virtual void compute_equivalence(std::vector<util::undirected_vertex_index_t>& bad_vertices, tree_pebble_node_t* node);

                virtual void get_leaves(std::deque<util::undirected_vertex_index_t>& leaves);
                virtual util::undirected_vertex_index_t find_closest_vertex_with(bool robot_on, util::undirected_vertex_index_t start, const pebble_assignment_t& assign, const std::vector<util::undirected_vertex_index_t>& obstacles);
                virtual bool push_pebble(util::undirected_vertex_index_t v, pebble_assignment_t& assign, const std::vector<util::undirected_vertex_index_t>& obstacles);
                virtual int find_closest_robot(util::undirected_vertex_index_t v, pebble_assignment_t& assign, const std::vector<util::undirected_vertex_index_t>& obstacles);
                virtual void update_node_info(util::undirected_vertex_index_t v, util::undirected_vertex_index_t u, pebble_assignment_t& assign);
                virtual void test_equivalent(tree_pebble_node_t* v_node, tree_pebble_node_t* u_node);
                virtual bool is_equivalent_criterion_2(tree_pebble_node_t* v_node, tree_pebble_node_t* u_node);
                virtual bool is_equivalent_criterion_4(tree_pebble_node_t* v_node, tree_pebble_node_t* u_node);

            };

        }
    }
}

#endif	// PRX_TREE_FEASIBILITY_TESTER_HPP
