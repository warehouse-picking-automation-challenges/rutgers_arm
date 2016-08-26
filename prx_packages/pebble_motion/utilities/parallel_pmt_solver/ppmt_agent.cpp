
/**
 * @file ppmt_agent.cpp
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

#include "utilities/parallel_pmt_solver/ppmt_agent.hpp"

namespace prx
{
    using namespace util;

    namespace packages
    {
        namespace pebble_motion
        {

            ppmt_agent_t::ppmt_agent_t()
            {
                busy = 0;
                path_length = 0;
                path_start = 0;
                status = SW_FREE;
                swapping_agent = NULL;
            }

            ppmt_agent_t::~ppmt_agent_t() { }

            void ppmt_agent_t::setup(int agent_id, int n, undirected_vertex_index_t v_start, undirected_vertex_index_t v_target)
            {
                id = agent_id;
                v_curr = v_start;
                goal = v_target;
                busy = 0;
                path_length = 0;
                path_start = 0;
                tmp_path.resize(n);
                status = SW_FREE;
                swapping_agent = NULL;
            }

            bool ppmt_agent_t::on_goal(util::undirected_vertex_index_t v_curr)
            {
                if( is_swapping() && tmp_goal != NULL )
                    return v_curr == tmp_goal;
                else if( status == SW_FREE )
                    return v_curr == goal;
                return false;
            }

            util::undirected_vertex_index_t ppmt_agent_t::get_goal()
            {
                if( is_swapping() && tmp_goal != NULL )
                    return tmp_goal;
                return goal;
            }

            bool ppmt_agent_t::is_free(unsigned long global_busy)
            {
                return busy != global_busy && status == SW_FREE;
            }

            bool ppmt_agent_t::is_busy(unsigned long global_busy)
            {
                return busy == global_busy;
            }

            void ppmt_agent_t::clear_path()
            {
                path_length = 0;
                path_start = 0;
            }

            bool ppmt_agent_t::has_path()
            {
                return path_length != 0 && path_start + 1 < path_length;
            }

            void ppmt_agent_t::move()
            {
                if( status == SW_EVACUATE && evacuate_step )
                    v_curr = evacuate_step;
                else
                {
                    PRX_ASSERT(has_path());
                    path_start++;
                    v_curr = tmp_path[path_start];
                }
            }

            void ppmt_agent_t::add_new_step(undirected_vertex_index_t v)
            {
                tmp_path[path_length] = v;
                path_length++;
            }

            undirected_vertex_index_t ppmt_agent_t::get_next_step()
            {
                if( status == SW_EVACUATE && evacuate_step )
                    return evacuate_step;

                PRX_ASSERT(has_path());
                return tmp_path[path_start + 1];
            }

            undirected_vertex_index_t ppmt_agent_t::get_current_step()
            {
                return tmp_path[path_start];
            }

            undirected_vertex_index_t ppmt_agent_t::get_last_step()
            {
                return tmp_path[path_length - 1];
            }

            void ppmt_agent_t::start_swap(swap_status_t new_status, util::undirected_vertex_index_t v_w, util::undirected_vertex_index_t v_neigh, util::undirected_vertex_index_t v_r, util::undirected_vertex_index_t v_q)
            {
                status = new_status;
                w = v_w;
                neigh = v_neigh;
                r = v_r;
                q = v_q;
                clear_path();
            }
            
            void ppmt_agent_t::start_evacuate(util::undirected_vertex_index_t evacuate_target)
            {
                status = SW_EVACUATE;
                evacuate_step = evacuate_target;
            }

            bool ppmt_agent_t::is_swapping()
            {
                return status != SW_FREE && status != SW_EVACUATE;
            }

            bool ppmt_agent_t::is_evacuating()
            {
                return status == SW_EVACUATE;
            }

            void ppmt_agent_t::done_swap()
            {
                clear_path();
                status = SW_FREE;
                swapping_agent = NULL;
                tmp_goal = NULL;                
            }
            
            void ppmt_agent_t::done_evacuate()
            {
                status = SW_FREE;
                evacuate_step = NULL;
            }
        }
    }
}
