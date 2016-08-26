/**
 * @file ppmt_agent.hpp
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

#ifndef PRX_PPMT_AGENT_HPP
#define	PRX_PPMT_AGENT_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "utilities/parallel_pmt_solver/ppmt_graph.hpp"
#include <deque>

namespace prx
{
    namespace packages
    {
        namespace pebble_motion
        {

            enum swap_status_t {SW_FREE, SW_EVACUATE, SW_WAIT, SW_MOVING, SW_READY, SW_SWAPPING};
            
            /**
             * A brief description of this class (no tag required).
             * 
             * A more detailed description of this class with multiple paragraphs delimited by blank lines. 
             */
            struct ppmt_agent_t
            {

              public:
                ppmt_agent_t();
                virtual ~ppmt_agent_t();

                void setup(int agent_id, int n, util::undirected_vertex_index_t v_start, util::undirected_vertex_index_t v_target);
                
                bool on_goal(util::undirected_vertex_index_t v_curr);
                                
                util::undirected_vertex_index_t get_goal();
                
                bool is_free(unsigned long global_busy);
                
                bool is_busy(unsigned long global_busy);
                
                void clear_path();                               
                
                bool has_path();
                
                void move();
                
                void add_new_step(util::undirected_vertex_index_t v);
                                
                util::undirected_vertex_index_t get_next_step();
                
                util::undirected_vertex_index_t get_current_step();
                
                util::undirected_vertex_index_t get_last_step();
                               
                void start_swap(swap_status_t new_status, util::undirected_vertex_index_t v_w, util::undirected_vertex_index_t v_neigh, util::undirected_vertex_index_t v_r, util::undirected_vertex_index_t v_q);
                
                void start_evacuate(util::undirected_vertex_index_t evacuate_target = NULL);
                
                bool is_swapping();
                
                bool is_evacuating();
                               
                void done_swap();
                
                void done_evacuate();
                                
                int id;
                unsigned long busy;
                double called;
                util::undirected_vertex_index_t v_curr;
                util::undirected_vertex_index_t goal;                
                util::undirected_vertex_index_t tmp_goal;
                util::undirected_vertex_index_t evacuate_step;
                util::undirected_vertex_index_t w;
                util::undirected_vertex_index_t neigh;
                util::undirected_vertex_index_t r;
                util::undirected_vertex_index_t q;
                int path_length;
                int path_start;
                std::deque<util::undirected_vertex_index_t> tmp_path;
                swap_status_t status;
                ppmt_agent_t* swapping_agent;
            };
        }
    }
}

#endif
