/**
 * @file crs_graph.hpp
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

#ifndef PRX_CRS_GRAPH_HPP
#define	PRX_CRS_GRAPH_HPP


#include <vector>
#include <set>

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"
#include "prx/simulation/state.hpp"
#include "prx/planning/motion_planners/motion_planner_edge.hpp"

namespace prx
{
    namespace packages
    {
        namespace labeled_rearrangement_manipulation
        {

            /**
             *
             */
            class crs_node_t : public util::undirected_node_t
            {

              public:

                crs_node_t(){ }

                ~crs_node_t(){ }

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

                bool same_arrangement(const std::vector<unsigned>& arrangement)
                {
                    return this->arrangement == arrangement;
                }

                std::vector<unsigned> arrangement;
            };

            class crs_edge_t : public plan::motion_planner_edge_t
            {

              public:

                crs_edge_t(){ }

                ~crs_edge_t(){ }

                void init(util::undirected_vertex_index_t source, util::undirected_vertex_index_t target, const sim::plan_t& plan)
                {
                    this->source = source;
                    this->target = target;
                    this->plan = plan;
                }

                void get_plan(sim::plan_t& plan, util::undirected_vertex_index_t source)
                {
                    plan = this->plan;
                    if( this->source != source )
                    {                   
                        plan.reverse();
                    }
                }
                
                void append_plan(sim::plan_t& plan, util::undirected_vertex_index_t source)
                {
                    if( this->source == source )
                    {
                        plan += this->plan;
                    }
                    else
                    {
                        for( int i = (int)this->plan.size() - 1; i >= 0; --i )
                        {
                            plan.copy_onto_back(this->plan[i].control, this->plan[i].duration);

                        }
                    }
                }


                util::undirected_vertex_index_t source;
                util::undirected_vertex_index_t target;

            };
        }
    }
}

#endif	






