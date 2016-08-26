/**
 * @file rpmg_solver.hpp
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

#ifndef PRX_RPMG_SOLVER_HPP
#define PRX_RPMG_SOLVER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "utilities/pebble_solver.hpp"
#include "utilities/pmt_solver/pmt_solver.hpp"
#include "utilities/pmt_solver/pmt_graph.hpp"
#include <deque>

namespace prx
{
    namespace packages
    {
        namespace pebble_motion
        {

            /**
             * The implementation of the pebble_motion_graph_solver or else rpmg_solver.
             *  
             * This is a pathfinding algorithm so as to solve the pebble motion problem on graphs.
             */
            class rpmg_solver_t : public pmt_solver_t
            {

              public:
                rpmg_solver_t();
                virtual ~rpmg_solver_t();

              protected:

                virtual bool swap(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign, int pebble1, int pebble2, std::deque<util::undirected_vertex_index_t>& path, int path_start, int path_end);

                virtual bool clear_vertex(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign, int pebble_id);
                
                virtual void compute_the_order(util::undirected_graph_t* graph, pebble_assignment_t& assignment);
                
                util::hash_t<int, int> id_order;
                int resolve_pos;
                bool need_resolve;
                int swap_pebble1;
                int swap_pebble2;
            };

        }
    }
}

#endif
