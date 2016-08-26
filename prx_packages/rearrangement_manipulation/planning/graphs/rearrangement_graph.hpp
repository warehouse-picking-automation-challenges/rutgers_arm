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
#include "prx/utilities/graph/undirected_graph.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"
#include "prx/simulation/state.hpp"
#include "prx/planning/motion_planners/motion_planner_edge.hpp"

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
            class rearrangement_node_t : public util::undirected_node_t
            {

              public:

                rearrangement_node_t(){ }

                ~rearrangement_node_t(){ }

                void init(const std::vector<unsigned>& arrangement)
                {
                    this->arrangement = arrangement;
                }

                void init(const util::space_t* space, const util::space_point_t* new_point, const std::vector<unsigned>& arrangement)
                {
                    point = space->clone_point(new_point);
                    this->arrangement = arrangement;
                }

                void init(const util::space_t* space, const std::vector<double>& new_point, const std::vector<unsigned>& arrangement)
                {
                    point = space->alloc_point();
                    space->set_from_vector(new_point, point);
                    this->arrangement = arrangement;
                }

                bool same_arrangement(const std::vector<unsigned>& arrangement) const
                {
                    return this->arrangement == arrangement;
                }

                std::string print() const
                {

                    std::stringstream output(std::stringstream::out);
                    
                    output << node_id << ") " ;
                    foreach(unsigned i, arrangement)
                    {
                        output << i << " , ";
                    }
                    return output.str();
                }

                std::vector<unsigned> arrangement;
            };

            class rearrangement_edge_t : public plan::motion_planner_edge_t
            {

              public:

                rearrangement_edge_t(){ }

                ~rearrangement_edge_t(){ }

                void init(util::undirected_vertex_index_t source, util::undirected_vertex_index_t target, const std::deque<path_part_t>& sequence)
                {
                    this->source = source;
                    this->target = target;
                    this->sequence = sequence;
                }

                void get_sequence(std::deque<path_part_t>& sequence, util::undirected_vertex_index_t source)
                {

                    if( this->source == source )
                    {
                        sequence = this->sequence;
                    }
                    else
                    {
                        for( int i = this->sequence.size() - 1; i >= 0; --i )
                        {
                            sequence.push_back(this->sequence[i]);
                            sequence.back().reverse();
                        }
                    }
                }

                void append_plan(std::deque<path_part_t>& sequence, util::undirected_vertex_index_t source)
                {
                    if( this->source == source )
                    {
                        sequence.insert(sequence.end(), this->sequence.begin(), this->sequence.end());
                    }
                    else
                    {
                        for( int i = this->sequence.size() - 1; i >= 0; --i )
                        {
                            sequence.push_back(this->sequence[i]);
                            sequence.back().reverse();
                        }
                    }
                }
                
                std::string print() const
                {

                    std::stringstream output(std::stringstream::out);
                    
                    output << source_vertex << " -> " << target_vertex << "   sequence_size:" << sequence.size() << std::endl;
                    foreach(path_part_t p, sequence)
                    {
                        output << p.print() << std::endl;
                    }
                    return output.str();
                }
                
                util::undirected_vertex_index_t source;
                util::undirected_vertex_index_t target;

                std::deque<path_part_t> sequence;
            };
        }
    }
}

#endif	






