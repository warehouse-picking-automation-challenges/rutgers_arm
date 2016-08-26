/**
 * @file kornhauser_tester.h
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

#ifndef PRX_KORNHAUSER_TESTER_H
#define	PRX_KORNHAUSER_TESTER_H

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "utilities/pebble_assignment.hpp"
#include "utilities/pebble_tester.hpp"
#include "utilities/kornhauser/kornhauser_graph.hpp"


namespace prx
{
    namespace packages
    {
        namespace pebble_motion
        {

            /**
             * Computes if a pebble test is solvable or not.
             */
            class kornhauser_tester_t : public pebble_tester_t
            {

              public:

                void reset();

                virtual util::undirected_vertex_index_t add_new_vertex(util::undirected_graph_t* graph);

                /** @copydoc pebble_tester_t::reduce(pebble_assignment_t&, pebble_assignment_t&, pebble_assignment_t&)  */
                virtual bool reduce(pebble_assignment_t& s_new, pebble_assignment_t& s_assignment, pebble_assignment_t& e_assignment);


                virtual bool pebble_test(util::undirected_graph_t* graph);

              private:
                void connected_components_subproblems(util::undirected_graph_t* g, pebble_assignment_t& s_assignment, pebble_assignment_t& e_assignment, int num_cc);

                void compute_subproblems(pebble_subproblem_t& problem, int m);
                void compute_subgraphs(util::undirected_graph_t* g, int m);
                void compute_equivalent_connectors(util::undirected_graph_t* g, int m);

                bool is_graph_trivial(util::undirected_graph_t* g);
                bool in_same_non_trivial_component(util::undirected_graph_t* g, util::undirected_vertex_index_t vi, util::undirected_vertex_index_t vj);
                bool has_unique_path(util::undirected_graph_t* g, util::undirected_vertex_index_t start, util::undirected_vertex_index_t goal);
                util::undirected_vertex_index_t find_closest_connector(util::undirected_graph_t* g, util::undirected_vertex_index_t v, int graph_class);
                int assign_pebble_to_graph(util::undirected_graph_t* g, util::undirected_vertex_index_t v, pebble_assignment_t& assignment);
                bool is_polygon(util::undirected_graph_t* g);
                bool polygon_has_solution(pebble_subproblem_t& problem);
                std::pair<util::undirected_vertex_index_t, util::undirected_edge_index_t> next_clockwise_vertex(util::undirected_graph_t* g, util::undirected_vertex_index_t v, util::undirected_edge_index_t edge);

                std::vector<pebble_subproblem_t> PP_sub;
                pebble_assignment_t start_plank;
                pebble_assignment_t end_plank;
                std::vector<pebble_subproblem_t> G_cc;
                util::hash_t< util::undirected_vertex_index_t, util::undirected_vertex_index_t> same_cc_nodes;
                util::hash_t< util::undirected_vertex_index_t, util::undirected_vertex_index_t> same_sub_nodes;
                //    std::vector<util::undirected_graph_t *> G_sub;
                std::vector<int> G_sub_blanks;
                std::vector<util::undirected_vertex_index_t> connectors;
                std::vector<int> G_bic;
                int path_length;

            };


        }
    }
}

#endif	// PRX_KORNHAUSER_TESTER_H
