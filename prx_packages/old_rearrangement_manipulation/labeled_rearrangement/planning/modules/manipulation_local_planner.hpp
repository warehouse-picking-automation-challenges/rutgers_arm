/**
 * @file manipulation_local_planner.hpp
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

#ifndef PRX_MANIPULATION_LOCAL_PLANNER_HPP
#define	PRX_MANIPULATION_LOCAL_PLANNER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/modules/local_planners/bvp_local_planner.hpp"


namespace prx
{
    namespace packages
    {
        namespace labeled_rearrangement_manipulation
        {
            /**
             * @anchor manipulation_local_planner_t
             *
             * This local planner works as a bvp_local_planner but can also push a state
             * and compute relative configurations during grasping.
             *
             * @brief <b> Local planner which helps for the manipulation task by calling compute_relative_configurations. </b>
             *
             * @author Athanasios Krontiris
             */
            class manipulation_local_planner_t : public plan::bvp_local_planner_t
            {
              public:
                manipulation_local_planner_t();
                manipulation_local_planner_t(double prop_length);
                virtual ~manipulation_local_planner_t();


                /**             
                 * Push the start state and computes relative_configuration before call the steer function.
                 *
                 * @copydoc local_planner_t::steer( const sim::state_t*, const sim::state_t*, sim::plan_t&, sim::trajectory_t&, bool connect )
                 */
                virtual void steer(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& plan,sim::trajectory_t& traj, bool connect = true);

                /**
                 * Push the start state and computes relative_configuration before call the steer function.
                 *
                 * @copydoc local_planner_t::steer( const sim::state_t*, const sim::state_t*, sim::plan_t&, sim::state_t*, bool connect )
                 */
                virtual void steer(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& plan,sim::state_t* result, bool connect = true);

                /**
                 * Push the start state and computes relative_configuration before call the steer function.
                 *
                 * @copydoc local_planner_t::propagate( const sim::state_t*, sim::plan_t&, sim::trajectory_t&)
                 */
                virtual void propagate(const sim::state_t* start, const sim::plan_t& plan, sim::trajectory_t& traj);

            };
        }
    }
}

#endif

