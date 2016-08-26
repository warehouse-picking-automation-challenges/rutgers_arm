/**
 * @file khorshid.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_KHORSHID_HPP
#define	PRX_KHORSHID_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "utilities/pmt_solver/pmt_solver.hpp"
#include <deque>


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
            class khorshid_t : public pmt_solver_t
            {

              public:
                khorshid_t();
                virtual ~khorshid_t();

                //    /** @copydoc pebble_solver_t::setup(pebble_assignment_t&, pebble_assignment_t&, int, const util::space_t*, util::distance_metric_t*)  */
                //    void setup(pebble_assignment_t& start_assign, pebble_assignment_t& target_assign, int num_robots, const util::space_t* space, util::distance_metric_t* distance_metric = NULL);
                //
                //    /** @copydoc pebble_solver_t::add_new_vertex(util::undirected_graph_t*)  */
                //    virtual util::undirected_vertex_index_t add_new_vertex(util::undirected_graph_t* graph);

              protected:

                /** @copydoc pebble_solver_t::find_solution(std::vector<pebble_step_t>* , util::undirected_graph_t* )  */
                virtual bool find_solution(std::vector<pebble_step_t>* solution, util::undirected_graph_t* graph);

                virtual bool move_to_goal(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign, int pebble_id, util::undirected_vertex_index_t v_start, util::undirected_vertex_index_t v_goal);

                /** @copydoc pebble_solver_t::swap(pebble_solution_path_t&, util::undirected_graph_t*, pebble_assignment_t&, int, int, std::deque<util::undirected_vertex_index_t>&, int, int)  */
                virtual bool swap(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign, int pebble1, int pebble2, std::deque<util::undirected_vertex_index_t>& path, int path_start, int path_end);

                /** @copydoc pebble_solver_t::check_visible_branches(pebble_solution_path_t&, util::undirected_graph_t*, pebble_assignment_t&, util::undirected_vertex_index_t, bool) */
                bool check_visible_branches(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign, util::undirected_vertex_index_t v_start, bool caller_on_w);
            };

        }
    }
}

#endif
