/**
 * @file irs.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2011, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors:  Athanasios Krontiris, Andrew Dobson, Andrew Kimmel, Zacharias Psarakis, Zakary Littlefield, James Marble, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#pragma once

#ifndef PRX_OFF_IRS_HPP
#define PRX_OFF_IRS_HPP

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"
#include "prx/planning/motion_planners/prm_star/prm_star.hpp"

namespace prx
{
    namespace plan
    {

        /**
         * Offline Incremental Roadmap Spanner:
         * A method for transforming a non sparse roadmap to a sparse one
         * 
         */
        class irs_t : public plan::prm_star_t
        {

          public:

            irs_t();
            virtual ~irs_t();

            /** @copydoc motion_planner_t::init(const util::parameter_reader_t*) */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

            /**
             * @copydoc motion_planner_t::execute() 
             * If the motion planner has a file to deserialize then it reads the graph from there instead of building a new graph. 
             *              
             * 
             * @return True if the execution was ok, otherwise false. 
             */
            // virtual bool execute();

            const util::statistics_t* get_statistics();

          protected:

            double stretch_factor;
            bool sparsify;
            
            util::sys_clock_t clock;

            /**
             * Operates the check for the same connected component in the graph that we 
             * gave as an argument.
             * 
             * @param g The graph that we want to check if the vertices are in the same 
             * connected component.
             * @param v1 The first vertex that we want to check.
             * @param v2 The second vertex that we want to check.
             * @return True if the v1 and v2 are in the same connected component, 
             *         False if the two nodes are in different connected component.
             */
            virtual bool same_connected_component_in_graph(const util::undirected_graph_t& g, const util::undirected_vertex_index_t v1, const util::undirected_vertex_index_t v2);

            /**
             * Checks if two vertices are in the same connected component.
             * 
             * @param v1 The first vertex that we want to check.
             * @param v2 The second vertex that we want to check.
             * @return True if the v1 and v2 are in the same connected component, 
             *         False if the two nodes are in different connected component.
             */
            virtual bool same_connected_component(const util::undirected_vertex_index_t v1, const util::undirected_vertex_index_t v2);

            virtual void link_node_to_neighbors(util::undirected_vertex_index_t v, const std::vector< const util::abstract_node_t* >& neighbors);
            double get_path_distance(  util::undirected_vertex_index_t start,  util::undirected_vertex_index_t goal);

            /**
             * @copydoc motion_planner_t::deserialize() const 
             */
            virtual bool deserialize();

            unsigned rejected_edges;

            util::undirected_graph_t full_graph;
        };


    }
}

#endif  // PRX_IRS_HPP

