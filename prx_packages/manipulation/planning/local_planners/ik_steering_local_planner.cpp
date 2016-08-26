/**
 * @file ik_steering_local_planner.cpp
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
#include "planning/local_planners/ik_steering_local_planner.hpp"

#include "prx/planning/world_model.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "simulation/systems/plants/movable_body_plant.hpp"

#include "simulation/workspace_trajectory.hpp"

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::plan::ik_steering_local_planner_t, prx::plan::local_planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    
    namespace plan
    {

        void ik_steering_local_planner_t::init(const parameter_reader_t * const reader, const parameter_reader_t * const template_reader)
        {
            local_planner_t::init(reader, template_reader);
        }

        void ik_steering_local_planner_t::steer(const state_t* start, const state_t* goal, plan_t& plan, trajectory_t& traj, bool connect)
        {
            PRX_WARN_S ("Calling the wrong steer function for the IK_steering_local_planner");
        }

        void ik_steering_local_planner_t::steer(const state_t* start, const state_t* goal, plan_t& plan, state_t* result, bool connect)
        {
            PRX_WARN_S ("Calling the wrong steer function for the IK_steering_local_planner");
        }
            
        void ik_steering_local_planner_t::steer(const state_t* start, const config_t& goal, plan_t& plan, trajectory_t& traj)
        {
            traj.clear();
            packages::manipulation::workspace_trajectory_t ee_traj;
            manip_model->jac_steering(plan, ee_traj, goal_state, start, goal);
            this->propagate(start, plan, traj);

        }

        void ik_steering_local_planner_t::link_model(world_model_t* model)
        {
            world_model = model;
            manip_model = dynamic_cast<packages::manipulation::manipulation_world_model_t*>(model);
            goal_state = manip_model->get_state_space()->alloc_point();
        }


    }
}