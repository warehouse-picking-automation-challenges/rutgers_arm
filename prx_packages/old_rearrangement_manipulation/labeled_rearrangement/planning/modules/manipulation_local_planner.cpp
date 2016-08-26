/**
 * @file manipulation_local_planner.cpp
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
#include "planning/modules/manipulation_local_planner.hpp"
#include "../../../manipulation/simulation/manipulator_simulator.hpp"

#include "prx/planning/modules/local_planners/bvp_local_planner.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"

#include "prx/simulation/plan.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::packages::labeled_rearrangement_manipulation::manipulation_local_planner_t, prx::plan::local_planner_t)


namespace prx
{
    using namespace util;
    using namespace sim;
    namespace packages
    {
        namespace labeled_rearrangement_manipulation
        {

            manipulation_local_planner_t::manipulation_local_planner_t() { }

            manipulation_local_planner_t::~manipulation_local_planner_t() { }

            void manipulation_local_planner_t::steer(const state_t* start, const state_t* goal, plan_t& plan, trajectory_t& traj, bool connect)
            {
                state_space->copy_from_point(start);
                ((manipulation::manipulation_simulator_t*)world_model->get_simulator())->compute_relative_config();
                bvp_local_planner_t::steer(start,goal,plan,traj,connect);
            }

            void manipulation_local_planner_t::steer(const state_t* start, const state_t* goal, plan_t& plan, state_t* result, bool connect)
            {
                state_space->copy_from_point(start);
                ((manipulation::manipulation_simulator_t*)world_model->get_simulator())->compute_relative_config();
                bvp_local_planner_t::steer(start,goal,plan,result,connect);
            }

            void manipulation_local_planner_t::propagate(const state_t* start, const plan_t& plan, trajectory_t& traj)
            {
                state_space->copy_from_point(start);
                ((manipulation::manipulation_simulator_t*)world_model->get_simulator())->compute_relative_config();
                bvp_local_planner_t::propagate(start, plan, traj);
            }
        }
    }
}

