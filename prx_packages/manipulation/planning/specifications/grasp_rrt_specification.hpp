/**
 * @file motion_planning_specification.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2016, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_GRASP_RRT_SPECIFICATION_HPP
#define PRX_GRASP_RRT_SPECIFICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/graph/abstract_node.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "simulation/systems/plants/movable_body_plant.hpp"

#include "utilities/definitions/manip_defs.hpp"

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
            class grasp_evaluator_t;
        }
    }
    namespace util
    {
        class distance_metric_t;
        class parameter_reader_t;
        class goal_t;
    }
    
    namespace plan
    {
        class world_model_t;
        class sampler_t;
        class validity_checker_t;
        class local_planner_t;
        class constraints_astar_search_t;

        class ik_steering_local_planner_t;

        /**
         *
         * @author Andrew Kimmel, Rahul Shome
         */
        class grasp_rrt_specification_t : public motion_planning_specification_t
        {

          public:
            grasp_rrt_specification_t();
            virtual ~grasp_rrt_specification_t();

            /**
             * @brief Initialize the planing query from input parameters.
             *
             * Initialize the planing query from input parameters.
             * 
             * @param reader The reader for the parameters.
             * @param template_reader A template reader for reading template parameters 
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

            /**
             * @brief Clear the plan and trajectory of this query.
             *
             * Clears the plan and the trajectory of the planning query in order to 
             * reuse the planning query with the same start and stopping criterion. 
             */
            virtual void clear();

            /**
             * Prepare the planning specification to be linked to the children
             * 
             * @brief Prepare the planning specification to be linked to the children
             */
            virtual void setup(world_model_t * const model);
            
            virtual void generate_new_metrics(const std::vector< const util::abstract_node_t* >& start_nodes);



            /** @brief Distance metric  used for c-space distance queries */
            util::distance_metric_t* metric;
            /** @brief EE-distance Metrics associated with each start state */
            std::vector<util::distance_metric_t*> start_state_metrics;

            ik_steering_local_planner_t* ik_local_planner;

            packages::manipulation::grasp_evaluator_t* grasp_evaluator;

            prx::packages::manipulation::movable_body_plant_t* object_to_grasp;

            
          protected:

            unsigned max_start_state_size;
            unsigned current_start_state_size;

        };

    }
}

#endif

