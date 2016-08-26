/**
 * @file rearrangement_graph.hpp
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

#ifndef PRX_REARRANGEMENT_GRAPH_HPP
#define	PRX_REARRANGEMENT_GRAPH_HPP


#include "planning/modules/path_part.hpp"

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/graph/directed_graph.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"
#include "prx/simulation/state.hpp"

#include <vector>
#include <set>

namespace prx
{
    namespace packages
    {
        namespace labeled_rearrangement_manipulation
        {

            /**
             *
             */
            class constraints_node_t : public util::directed_node_t
            {

              public:

                constraints_node_t(){ }

                ~constraints_node_t(){ }

                bool has_plan()
                {
                    return part.has_plan;
                }
                
                bool is_constrained(const std::vector<unsigned>& arrangement)
                {
                    foreach(unsigned c, part.constraints)
                    {
                        if(std::find(arrangement.begin(),arrangement.end(),c) != arrangement.end())
                            return true;
                    }
                    return std::find(arrangement.begin(),arrangement.end(), part.pose_to) != arrangement.end();
                }

                path_part_t part;
            };

            class super_constraints_node_t : public util::directed_node_t
            {

              public:

                super_constraints_node_t()
                { 
                    is_cycle_node = false;
                }

                ~super_constraints_node_t(){ }

                void add_to_cycle(constraints_node_t* node)
                {
                    cycle.push_back(node->index);
                    if( cycle.size() > 1 )
                        is_cycle_node = true;
                    
                    ids.push_back(node->part.id);
                    starts.push_back(node->part.pose_from);
                    targets.push_back(node->part.pose_to);
                    constraints.insert(node->part.constraints.begin(), node->part.constraints.end());
                }
                
                bool interact_with_starts(super_constraints_node_t* node)
                {
                    std::vector<unsigned> common;
                    std::set_intersection(constraints.begin(),constraints.end(), node->starts.begin(), node->starts.end(), std::back_inserter(common));
                    return common.size() > 0;
                }
                
                bool interact_with_targets(super_constraints_node_t* node)
                {
                    std::vector<unsigned> common;
                    std::set_intersection(constraints.begin(),constraints.end(), node->targets.begin(), node->targets.end(), std::back_inserter(common));
                    return common.size() > 0;
                }

                std::string print() const
                {
                    std::stringstream out(std::stringstream::out);

                    out << is_cycle_node << "   cycle_size:" << cycle.size() << "  S:";
                    foreach(unsigned s, starts)
                    {
                        out << s << ",";
                    }
                    out << "   T:";
                    foreach(unsigned t, targets)
                    {
                        out << t << ",";
                    }                    
                    out << "   C:";
                    foreach(unsigned c, constraints)
                    {
                        out << c << ",";
                    }
                    
                    return out.str();
                }

                bool is_cycle_node;
                std::vector<util::directed_vertex_index_t> cycle;
                std::vector<unsigned> ids;
                std::vector<unsigned> starts;
                std::vector<unsigned> targets;
                std::set<unsigned> constraints;
            };
        }
    }
}

#endif	






