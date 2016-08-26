/**
 * @file ground_truth_query_application.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Andrew Kimmel, Andrew Dobson, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "prx/planning/applications/replanning_application.hpp"
#include "prx/planning/communication/planning_comm.hpp"
#include "prx/planning/communication/simulation_comm.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/problem_specifications/specification.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/task_planners/task_planner.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/modules/stopping_criteria/element/goal_criterion.hpp"

#include "prx/utilities/goals/radial_goal_region.hpp"
#include "prx/utilities/distance_metrics/linear_metric/linear_distance_metric.hpp"
#include "prx/utilities/graph/abstract_node.hpp"

#include "prx/utilities/communication/tf_broadcaster.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"

#include "prx_simulation/query_msg.h"
#include "prx_simulation/state_msg.h"
#include "prx_simulation/plant_locations_msg.h"

#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS( prx::plan::replanning_application_t, prx::plan::planning_application_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan::comm;

    namespace plan
    {

        replanning_application_t::replanning_application_t()
        {
            goal_metric = new linear_distance_metric_t();
            num_queries = 0;
            query = NULL;
            once = false;
        }

        replanning_application_t::~replanning_application_t()
        {
            delete query;
            delete goal_metric;
        }

        void replanning_application_t::init(const parameter_reader_t* reader)
        {
            planning_application_t::init(reader);

            model->use_context(consumer_to_space_name_mapping[consumer_path]);
            state_space = model->get_state_space();
            control_space = model->get_control_space();

            planning_duration = reader->get_attribute_as<double>("planning_duration");

            //TODO: Query just needs the state and control space, I believe...
            query->state_space = state_space;
            query->control_space = control_space;

            task_planner = this->root_task;
            //task_planner = dynamic_cast<task_planner_t*>(this->root_task);
            if (task_planner == NULL || query == NULL)
            {
                PRX_FATAL_S ("Something did not initialize correctly in the replanning application init: query or task planner is NULL.");
            }
        }

        void replanning_application_t::execute()
        {
            //Replanning Execute is left intentionally empty
        }

        void replanning_application_t::process_query_callback(const prx_simulation::query_msg& msg)
        {
            process_plant_locations_callback(msg.plant_locations);
            if (once)
                num_queries++;
            PRX_INFO_S ("Process query callback!\n Consumer:" << msg.consumer << "\n Goal region radius: " << msg.goal_region_radius);

            // Construct the query
            model->use_context(consumer_to_space_name_mapping[consumer_path]);

            if (!once)
            {
                root_specifications[0]->link_spaces(state_space, control_space);
                root_specifications[0]->setup(model);
                task_planner->link_specification(root_specifications[0]);
                state_space->set_from_vector(msg.start);                
                // Set the goal and the distance metric
                radial_goal_region_t* new_goal = new radial_goal_region_t(state_space, goal_metric, msg.goal, msg.goal_region_radius);
                PRX_DEBUG_S ("Set goal");
                query->set_goal(new_goal);
                query->link_spaces(state_space, control_space);

                query->set_start_from_vector(msg.start);
                PRX_DEBUG_S("Link query");
                task_planner->link_query(query);
            }
            task_planner->setup();
            task_planner->execute();
            task_planner->resolve_query();

            if(visualize)
            {
                task_planner->update_visualization();
                ((visualization_comm_t*)vis_comm)->send_geometries();
            }

            ((planning_comm_t*)plan_comm)->publish_plan(consumer_path,query->plan );
            once = true;
        }

        void replanning_application_t::process_ground_truth_callback(const prx_simulation::state_msg& msg)
        {
        }

        void replanning_application_t::process_plant_locations_callback(const prx_simulation::plant_locations_msg& msg)
        {
        }
    }
}
