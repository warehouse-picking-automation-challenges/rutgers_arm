/**
 * @file parallel_propagation_application.cpp 
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

#include "planning/parallel_propagation_application.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/planning/world_model.hpp"
#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS( prx::plan::parallel_propagation_application_t, prx::plan::planning_application_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace plan
    {

        parallel_propagation_application_t::parallel_propagation_application_t() : as_(nh_, "/prx/"+ros::this_node::getName(),boost::bind(&parallel_propagation_application_t::executeCB, this, _1), false)
        {
        }

        void parallel_propagation_application_t::init(const parameter_reader_t* reader)
        {
            //get correct model

            parameter_reader_t wm_reader(parameters::get_attribute("base_planning",reader,NULL), global_storage);

            model = (world_model_t*)wm_reader.create_from_loader<world_model_t > ("world_model", "prx_planning");
            wm_reader.initialize(model, "world_model");
            as_.start();
        }

        void parallel_propagation_application_t::execute()
        {

        }

        void parallel_propagation_application_t::executeCB(const prx_planning::parallel_propGoalConstPtr &goal)
        {
            // PRX_INFO_S("Start: "<<ros::this_node::getName());
            model->use_context(goal->planning_context);

            model->get_full_state_space()->set_from_vector(goal->initial_state);

            state_t* state = model->get_state_space()->alloc_point();
            trajectory_t traj(model->get_state_space());
            plan_t plan(model->get_control_space());
            plan.append_onto_back(goal->duration);
            model->get_control_space()->set_from_vector(goal->initial_control,plan.back().control);

            model->propagate_plan(state, plan, traj);

            model->get_full_state_space()->copy_to_vector(result_.final_state);
            result_.collision_free = !traj.in_collision();

            as_.setSucceeded(result_);
            model->get_state_space()->free_point(state);
            // PRX_INFO_S("Done: "<<ros::this_node::getName());
        }

    }
}
