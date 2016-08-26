/**
 * @file pmg_solver.hpp
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
 
#ifndef PRX_PMG_SOLVER_HPP
#define	PRX_PMG_SOLVER_HPP 

#include "prx/utilities/definitions/defs.hpp"
#include "utilities/pmt_solver/pmt_graph.hpp"
#include "utilities/pmt_solver/pmt_solver.hpp"
#include <deque>

namespace prx
{
    namespace packages
    {
        namespace pebble_motion
        {

            /**
             * The implementation of the pebble_motion_graph_solver or else pmg_solver.
             *  
             * This is a pathfinding algorithm so as to solve the pebble motion problem on graphs.
             */
            class pmg_solver_t : public pmt_solver_t
            {

              public:
                pmg_solver_t();
                virtual ~pmg_solver_t();

              protected:

                /** @copydoc pebble_solver_t::find_solution(std::vector<pebble_step_t>* , util::undirected_graph_t* )  */
                virtual bool find_solution(std::vector<pebble_step_t>* solution, util::undirected_graph_t* graph);

                virtual bool move_to_goal(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign, int pebble_id, util::undirected_vertex_index_t v_start, util::undirected_vertex_index_t v_goal);
            };
        }
    }
}

#endif
