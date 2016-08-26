/**
 * @file grasp_rrt_specification.cpp
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

#include "planning/specifications/grasp_rrt_specification.hpp"
#include "planning/local_planners/ik_steering_local_planner.hpp"
#include "planning/manipulation_world_model.hpp"
#include "planning/distance_functions/end_effector_distance.hpp"
#include "planning/manipulation_world_model.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"

#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/modules/heuristic_search/constraints_astar_search.hpp"
#include "planning/modules/grasp_evaluator.hpp"

#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(prx::plan::grasp_rrt_specification_t, prx::plan::specification_t)

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace plan
    {

        grasp_rrt_specification_t::grasp_rrt_specification_t()
        {
            validity_checker = NULL;
            sampler = NULL;
            metric = NULL;
            local_planner = NULL;
            astar = NULL;
            current_start_state_size = 0;
        }

        grasp_rrt_specification_t::~grasp_rrt_specification_t()
        {
            clear();

            delete ik_local_planner;
        }

        void grasp_rrt_specification_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            motion_planning_specification_t::init(reader, template_reader);
            max_start_state_size = parameters::get_attribute_as<unsigned>("max_start_state_size", reader, template_reader);             
            
            // Allocate distance metrics
            start_state_metrics.resize(max_start_state_size);
            for (unsigned i = 0; i < max_start_state_size; ++i )
            {
                start_state_metrics[i] = parameters::initialize_from_loader<distance_metric_t> ("prx_utilities", reader, "goal_biasing_metric", template_reader, "goal_biasing_metric");
            }

            grasp_evaluator = parameters::create_from_loader<packages::manipulation::grasp_evaluator_t> ("prx_planning", reader, "grasp_evaluator", template_reader, "grasp_evaluator");
            // Allocate ik steering local planner
            ik_local_planner = new ik_steering_local_planner_t();

        }

        void grasp_rrt_specification_t::setup(world_model_t * const model)
        {
            motion_planning_specification_t::setup( model );

            for (unsigned i = 0; i < max_start_state_size; ++i)
            {
                start_state_metrics[i]->link_space(state_space);
                prx::packages::manipulation::end_effector_distance_t* ee_distance = dynamic_cast< prx::packages::manipulation::end_effector_distance_t* >(start_state_metrics[i]->get_distance_function());
                if(ee_distance!=NULL)
                {
                    prx::packages::manipulation::manipulation_world_model_t* manip_model = dynamic_cast< prx::packages::manipulation::manipulation_world_model_t* >(model);
                    if(manip_model!=NULL)
                    {
                        ee_distance->link_manipulation_model(manip_model);
                    }
                    else
                    {
                        PRX_ERROR_S("Manipulation world model needs to be linked in to the end effector distance.");
                    }
                }
                else
                {
                    PRX_ERROR_S("End effector metric should be used for the secondary metric during grasping.");
                }

            }

            ik_local_planner->link_model(model);
            grasp_evaluator->link_world_model(model);

        }

        void grasp_rrt_specification_t::clear()
        {
            motion_planning_specification_t::clear();

            if (current_start_state_size > 0)
            {
                for( unsigned i = 0; i < current_start_state_size; ++i )
                {
                    start_state_metrics[i]->clear();
                }
            }

            current_start_state_size = 0;
        }


        void grasp_rrt_specification_t::generate_new_metrics(const std::vector< const abstract_node_t* >& start_nodes)
        {
            // Ensure that our metrics have been cleared out if necessary
            clear();

            // Assign the new size of the start states
            current_start_state_size = start_nodes.size();

            PRX_ASSERT(current_start_state_size < max_start_state_size );

            // Add the corresponding nodes to their metrics
            for (unsigned i = 0; i < current_start_state_size; ++i)
            {
                start_state_metrics[i]->add_point(start_nodes[i]);
            }

        }
    }
}
