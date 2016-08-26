/**
 * @file ppmt_graph.hpp
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

#ifndef PRX_PPMT_GRAPH_HPP
#define	PRX_PPMT_GRAPH_HPP

#include "utilities/pmt_solver/pmt_graph.hpp"

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
            class ppmt_graph_node_t : public pmt_graph_node_t
            {

              public:
                unsigned long used_id; //If this node is already used in this turn.
                bool swap_lock; //If the node is used for swap
                
                ppmt_graph_node_t()
                {
                    used_id = 0;
                    swap_lock = false;
                }
                virtual ~ppmt_graph_node_t(){}
                
                bool is_vertex_used(unsigned long global_id)
                {
                    return used_id >= global_id;
                }
                
                bool is_vertex_blocked(unsigned long global_id, unsigned long global_obstacle)
                {
                    return used_id >= global_id || swap_lock || obstacle_id >= global_obstacle;
                }
                
                void block_vertex()
                {
                    used_id = PRX_INFINITY;
                }
                
                void lock(bool action)
                {
                    swap_lock = action;
                }

                std::string print()
                {
                    std::stringstream out(std::stringstream::out);

                    out << pmt_graph_node_t::print() << "  used:" << used_id;
                    return out.str();
                }
            };
        }
    }
}

#endif
