/**
 * @file astar_test_tp.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#pragma once

#ifndef PRX_ASTAR_TEST_TP_HPP
#define	PRX_ASTAR_TEST_TP_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/spaces/space.hpp"

#include "prx/simulation/plan.hpp"
#include "prx/simulation/trajectory.hpp"

#include "prx/planning/task_planners/task_planner.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/queries/query.hpp"
#include "prx/planning/problem_specifications/specification.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"

#include "planning/manipulation_world_model.hpp"
#include "planning/queries/manipulation_query.hpp"

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
            
            /**
             * A task planner for testing the manipulation task planner. This task planner executes a simple pick and place. 
             *
             * @authors Andrew Dobson
             */
            class astar_test_tp_t : public plan::task_planner_t
            {

              public:

                astar_test_tp_t();
                virtual ~astar_test_tp_t();

                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                /** @copydoc motion_planner_t::link_world_model(world_model_t* const) */
                void link_world_model(plan::world_model_t * const model);

                /** @copydoc planner_t::link_query(query_t*) */
                virtual void link_query(plan::query_t* new_query);

                /** @copydoc motion_planner_t::setup() */
                virtual void setup();

                /** @copydoc motion_planner_t::execute() */
                virtual bool execute();

                /** @copydoc motion_planner_t::succeeded() const */
                virtual bool succeeded() const;

                /** @copydoc motion_planner_t::get_statistics() */
                virtual const util::statistics_t* get_statistics();

                virtual void resolve_query();

              protected:
                void load_graph();
                void set_edge_constraints( util::undirected_edge_index_t e, std::vector< unsigned >& input_constraints );

                std::vector< util::undirected_edge_index_t > eis;

                util::space_t* state_space;
                util::space_t* control_space;
                util::undirected_vertex_index_t start_vertex;
                util::undirected_vertex_index_t goal_vertex;

                object_constraints_checker_t* OCC;
                
                util::undirected_graph_t graph;
                
                plan::motion_planning_query_t* in_query;
                
                std::vector< double* > state_memory;
                std::vector< double* > control_memory;
                
            };
        }
    }
}


#endif
