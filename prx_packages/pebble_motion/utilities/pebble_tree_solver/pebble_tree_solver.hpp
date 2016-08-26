/**
 * @file pebble_tree_solver.hpp
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

#ifndef PRX_PEBBLE_TREE_SOLVER_HPP
#define	PRX_PEBBLE_TREE_SOLVER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "utilities/pebble_solver.hpp"

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
            class pebble_tree_solver_t : public pebble_solver_t
            {

              public:
                pebble_tree_solver_t();
                virtual ~pebble_tree_solver_t();

                virtual util::undirected_vertex_index_t add_new_vertex(util::undirected_graph_t* graph);

              protected:

                /** @copydoc pebble_solver_t::reduce(pebble_assignment_t&, pebble_assignment_t&, pebble_assignment_t&)  */
                virtual bool reduce(std::vector<pebble_step_t>* solution, util::undirected_graph_t* graph, pebble_assignment_t& s_new, pebble_assignment_t& s_assign, pebble_assignment_t& t_assign);

                virtual void update_info(util::undirected_graph_t* graph, pebble_assignment_t& t_assign);

                virtual bool find_solution(std::vector<pebble_step_t>* solution, util::undirected_graph_t* graph);

                /**
                 * Find the closest branch that the swap can happened. 
                 * @param v First pebble's position
                 * @param u Second pebble's position
                 * @return A pair with the closest swap vertex and the number of the criterion that found this vertex.
                 *         The number of the criterion will be used from the clear function to clear the swap vertex.     
                 */
                virtual std::pair<util::undirected_vertex_index_t, int> find_branch(const util::undirected_graph_t* graph, util::undirected_vertex_index_t v, util::undirected_vertex_index_t u);
                util::undirected_vertex_index_t is_equivalent_criterion_2(const util::undirected_graph_t* graph, util::undirected_vertex_index_t v, util::undirected_vertex_index_t u);
                util::undirected_vertex_index_t is_equivalent_criterion_4(const util::undirected_graph_t* graph, util::undirected_vertex_index_t v, util::undirected_vertex_index_t u);
                virtual bool swap(std::vector<pebble_step_t>* solution, const util::undirected_graph_t* graph, int pebble1, int pebble2, util::undirected_vertex_index_t w, int criterion_no);
                virtual bool perform_swap(std::vector<pebble_step_t>* solution, const util::undirected_graph_t* graph, int pebble1, int pebble2, util::undirected_vertex_index_t pos1, util::undirected_vertex_index_t pos2, util::undirected_vertex_index_t w);
                virtual bool perform_swap2(std::vector<pebble_step_t>* solution, const util::undirected_graph_t* graph, int pebble1, int pebble2, util::undirected_vertex_index_t pos1, util::undirected_vertex_index_t pos2, util::undirected_vertex_index_t w1, util::undirected_vertex_index_t w2);
                virtual void revert(std::vector<pebble_step_t>* path, int start, int end, int pebble1, int pebble2, pebble_assignment_t& assign);
                virtual void get_leaves(const util::undirected_graph_t* graph, allocated_heap_t<util::undirected_vertex_index_t>& leaves);
                virtual util::undirected_vertex_index_t find_closest_vertex_with(const util::undirected_graph_t* graph, bool robot_on, util::undirected_vertex_index_t start, const pebble_assignment_t& assign);
                virtual bool push_pebble_once(std::vector<pebble_step_t>* solution, const util::undirected_graph_t* graph, util::undirected_vertex_index_t v, pebble_assignment_t& assign, bool execute_push = true);
                virtual bool move(std::vector<pebble_step_t>* path, const util::undirected_graph_t* graph, int pebble_id, util::undirected_vertex_index_t v_start, util::undirected_vertex_index_t v_target, pebble_assignment_t& assign);
                virtual int find_closest_robot(const util::undirected_graph_t* graph, util::undirected_vertex_index_t v, pebble_assignment_t& assign);
                virtual void update_node_info(util::undirected_graph_t* graph, util::undirected_vertex_index_t v, util::undirected_vertex_index_t u, pebble_assignment_t& assign);
            };

        }
    }
}

#endif	// PRX_PEBBLE_TREE_SOLVER_HPP
