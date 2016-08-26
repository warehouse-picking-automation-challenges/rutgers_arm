/**
 * @file irs.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2011, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Athanasios Krontiris, Andrew Dobson, Andrew Kimmel, Zakary Littlefield, James Marble, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#pragma once

#ifndef PRX_TSM_PRM_STAR_HPP
#define PRX_TSM_PRM_STAR_HPP

#include "prx/planning/motion_planners/prm_star/prm_star.hpp"

#include "utilities/definitions/manip_defs.hpp"

namespace prx
{

    namespace plan
    {
        class tsm_prm_specification_t;
    }

    namespace packages
    {

        namespace manipulation
        {

            /**
             * Task Space Metric PRM*
             * A method for quickly constructing sparse roadmaps that provide high quality solutions to motion planning queries
             * @author Andrew Kimmel
             */
            class tsm_prm_star_t : public plan::prm_star_t
            {

              public:

                tsm_prm_star_t();
                virtual ~tsm_prm_star_t();

                /** @copydoc motion_planner_t::init(const util::parameter_reader_t*) */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                /**
                 * @copydoc motion_planner_t::link_specification(specification_t*)
                 */
                virtual void link_specification(plan::specification_t* new_spec);

                /**
                 * @copydoc motion_planner_t::serialize() const 
                 */
                virtual bool serialize();

                /**
                 * @copydoc motion_planner_t::deserialize() const 
                 */
                virtual bool deserialize();

                virtual void step();

              protected:

                /**
                 * Adds a new node in the graph and trying to connect it with the 
                 * existing graph. 
                 * 
                 * @brief Add a new node to the graph.
                 *
                 * @param n_state The new state that I want to add in the graph.
                 * 
                 * @return The index for the node in the boost::graph.
                 */
                virtual std::pair<bool,util::undirected_vertex_index_t> add_node(util::space_point_t* n_state);

                /**
                 * @brief Connects the node v on the existing graph.
                 * 
                 * @param v Its the index of an existing node on the graph.
                 */
                virtual void connect_node(util::undirected_vertex_index_t v);

                /**
                 * @brief Connects the node v in a neighbor of radian rad on the existing graph.
                 * 
                 * @param v Its the index of an existing node on the graph.
                 * @param rad The diameter of the neighbor that we need to check in order to connect the new node on the graph.
                 */
                virtual void connect_node(util::undirected_vertex_index_t v, double rad);

                /**
                 * @brief Tries to link the node v with the neighbor nodes in the vector neighbors.
                 * 
                 * @param v Its the index of an existing node on the graph, that we want to connect on the graph.
                 * @param neighbors The nodes that we will try to connect with the node v.
                 */
                virtual void link_node_to_neighbors(util::undirected_vertex_index_t v, const std::vector< const util::abstract_node_t* >& neighbors);
                virtual void remove_vertex( util::undirected_vertex_index_t v );


                sim::state_t* config_to_state(util::config_t conf);

                double _x, _y, _z, _qx, _qy, _qz, _qw;

                util::space_t* end_effector_space;

                plan::tsm_prm_specification_t* tsm_prm_specification;
            };
        }
    }
}

#endif  //PRX_TSM_PRM_STAR_HPP

