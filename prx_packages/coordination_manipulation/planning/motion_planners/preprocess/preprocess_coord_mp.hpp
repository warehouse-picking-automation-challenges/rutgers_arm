/**
 * @file preprocess_coordination_mp.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_PREPROCESS_COORD_MP_HPP
#define	PRX_PREPROCESS_COORD_MP_HPP

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"
#include "prx/planning/motion_planners/prm_star/prm_star.hpp"
#include "planning/motion_planners/preprocess/preprocess_mp_query.hpp"

namespace prx
{
    namespace packages
    {
        namespace coordination_manipulation
        {
            /**
             * PRM star for generating coordination roadmaps for two manipulators
             * 
             * @author Andrew Kimmel
             * 
             */
            class preprocess_coordination_mp_t : public plan::prm_star_t
            {

              public:

                preprocess_coordination_mp_t();
                virtual ~preprocess_coordination_mp_t();

                /** @copydoc motion_planner_t::init(const util::parameter_reader_t*) */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);
                
                virtual void link_specification(plan::specification_t* new_spec);
                
                virtual void link_query(plan::query_t* new_query);
                
                virtual void setup();
                
                virtual void resolve_query();
                
                virtual void reset();
                
                virtual bool execute();
                
                virtual void step();
  
                virtual bool serialize();

                virtual bool deserialize();

              protected:

                virtual void link_node_to_neighbors(util::undirected_vertex_index_t v, const std::vector< const util::abstract_node_t* >& neighbors);
                                
                virtual void valid_random_sample();

                virtual std::pair<bool,util::undirected_vertex_index_t> add_node(const util::space_point_t* n_state);
                
                virtual util::undirected_edge_index_t add_edge(util::undirected_vertex_index_t v, util::undirected_vertex_index_t u, double dist);

//                virtual void connect_node(util::undirected_vertex_index_t v, double rad);
                
                virtual void update_k(unsigned nr_nodes);
                
                virtual bool valid_point(sim::state_t* point_to_check);
                virtual bool valid_point(unsigned right_point, unsigned left_point);

                virtual void get_armcup_state(sim::state_t* arm_start_state, const sim::state_t* cup_safe_state, 
                                              const util::space_t* arm_state_space, const util::space_t* cup_space, const util::space_t* armcup_space, sim::state_t* armcup_state);

                preprocess_mp_query_t* preprocess_query;
                
                const util::space_t* full_arm_state_space, *full_arm_control_space;
                sim::state_t* full_armcup_state;
                
                unsigned left_arm_size, right_arm_size;
                sim::state_t* grid_point, *coordination_start_point;
                unsigned left_start_index, right_start_index;
                
                /** Preprocess stuff */
                bool preprocess_graph;
                std::vector< std::vector<bool> > valid_vertex; // right arm x left arm
                bool save_graph, save_constraints, approximate_collisions;
                
                int max_plan_size, max_objects;
                util::sys_clock_t computation_timer;

            };
        }

    }
}

#endif	// PRX_PREPROCESS_COORD_MP_HPP

