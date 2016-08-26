/**
 * @file jprm_star_graph.hpp
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

#ifndef PRX_JACOBIAN_PRM_STAR_GRAPH_HPP
#define PRX_JACOBIAN_PRM_STAR_GRAPH_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/motion_planners/motion_planner_edge.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
            /**
             * PRM nodes must remember the state space point they represent.  These nodes
             * additionally carry extra information for PRM and its variants.
             *
             * @brief <b> Node class used by the PRM planning structure. </b>
             *
             * @author Andrew Dobson
             */
            class mapping_node_t : public util::abstract_node_t
            {
              public:
                mapping_node_t( const util::space_t* space, const util::space_point_t* new_point, util::abstract_node_t* linked_node )
                {
                    init_node( space, new_point, linked_node );
                }
                
                ~mapping_node_t()
                { }
                
                void init_node( const util::space_t* space, const util::space_point_t* new_point, util::abstract_node_t* linked_node )
                {
                    point = space->clone_point(new_point);
                    graph_node = linked_node;
                }
                
                util::abstract_node_t* get_linked_node() const
                {
                    return graph_node;
                }
                
              protected:
                util::abstract_node_t* graph_node;                
            };

        }
    }
}

#endif
