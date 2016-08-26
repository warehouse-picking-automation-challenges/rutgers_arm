/**
 * @file ik_steering_local_planner.hpp
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
#pragma once

#ifndef PRX_IK_STEERING_LOCAL_PLANNER_HPP
#define	PRX_IK_STEERING_LOCAL_PLANNER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "planning/manipulation_world_model.hpp"

namespace prx
{
    namespace plan
    {

        class ik_steering_local_planner_t : public local_planner_t
        {

          public:

            ik_steering_local_planner_t() : local_planner_t(){ }
            /**
             * @copydoc local_planner_t::init()
             */
            virtual void init(const util::parameter_reader_t * const reader, const util::parameter_reader_t * const template_reader);

            virtual ~ik_steering_local_planner_t(){ }
            /**
             * @copydoc local_planner_t::steer( const sim::state_t*, const sim::state_t*, sim::plan_t&, sim::trajectory_t& , bool connect)
             */
            virtual void steer(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& plan, sim::trajectory_t& traj, bool connect = true);
            /**
             * @copydoc local_planner_t::steer( const sim::state_t*, const sim::state_t*, sim::plan_t&, sim::state_t*, bool connect )
             */
            virtual void steer(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& plan, sim::state_t* result, bool connect = true);
            /**
             * @brief Steering function designed to work with the IK_steering of the manip_world_model
             */
            virtual void steer(const sim::state_t* start, const util::config_t& goal, sim::plan_t& plan, sim::trajectory_t& traj);

            /**
             * @param model The world model to link to this local planner.
             */
            virtual void link_model(world_model_t* model);

          protected:
            packages::manipulation::manipulation_world_model_t* manip_model;
            sim::state_t* goal_state;
        };

    }
}

#endif
