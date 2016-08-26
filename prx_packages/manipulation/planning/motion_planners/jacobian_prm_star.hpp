/**
 * @file jacobian_prm_star.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2016, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Athanasios Krontiris, Andrew Dobson, Andrew Kimmel, Zakary Littlefield, James Marble, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#pragma once

#ifndef PRX_JACOBIAN_PRM_STAR_HPP
#define PRX_JACOBIAN_PRM_STAR_HPP

#include "prx/planning/motion_planners/prm_star/prm_star.hpp"
#include "planning/motion_planners/jprm_star_graph.hpp"
#include "planning/modules/manipulation_validity_checker.hpp"

// #include "utilities/definitions/manip_defs.hpp"

namespace prx
{

    namespace packages
    {

        namespace manipulation
        {
            class manipulation_world_model_t;

            /**
             * Jacobian-based PRM* which uses jacobian steering to reach target configurations
             * @author Andrew Dobson
             */
            class jacobian_prm_star_t : public plan::prm_star_t
            {

              public:

                jacobian_prm_star_t();
                virtual ~jacobian_prm_star_t();

                /** @copydoc motion_planner_t::init(const util::parameter_reader_t*) */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                virtual void link_specification(plan::specification_t* new_spec);

                std::pair<bool, util::undirected_vertex_index_t> add_node(util::space_point_t* n_state);

                virtual void resolve_query();

                virtual bool deserialize();

              protected:
                void add_links_to_metrics( util::abstract_node_t* node, manipulation_world_model_t* manip_model );
                void conf_to_point( util::space_point_t* point, const util::config_t& conf );

                virtual void remove_vertex( util::undirected_vertex_index_t v );

                std::vector< std::string > end_effector_names;
                util::hash_t< std::string, util::distance_metric_t* > end_effector_metrics;
                std::vector< mapping_node_t* > mapping_nodes;
                manipulation_validity_checker_t* manip_checker;
                
                bool resolving_query;
                bool prm_star_query;
                
                util::space_t* se3;
                std::vector< double* > _se3_mem;
            };
        }
    }
}

#endif  //PRX_JACOBIAN_PRM_STAR_HPP

