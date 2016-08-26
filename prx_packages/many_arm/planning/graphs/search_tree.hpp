/**
 * @file tree.hpp
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

#ifndef PRX_MULTI_SEARCH_HPP
#define PRX_MULTI_SEARCH_HPP

#include "prx/utilities/graph/tree.hpp"

namespace prx
{
    namespace packages
    {
        namespace multi_arm
        {
            class automaton_node_t;

            class search_node_t : public util::tree_node_t
            {
              public:
                search_node_t()
                {
                    cost = heuristic = 0.0;
                    expansions = 0;
                    auto_state = NULL;
                    record_index = 0;
                    at_goal = false;
                }

                ~search_node_t()
                {

                }

                bool operator<( const search_node_t& other )
                {
                    if( at_goal && !other.at_goal )
                        return true;

                    return ( cost + heuristic ) < ( other.cost + other.heuristic );
                }

                double cost;
                double heuristic;
                unsigned expansions;

                bool at_goal;

                automaton_node_t* auto_state;
                unsigned record_index;
            };

        }
    }
}

#endif //PRX_MULTI_SEARCH_HPP
