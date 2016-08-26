/**
 * @file esst_local_planner.hpp
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

#ifndef PRX_ESST_LOCAL_PLANNER_HPP
#define	PRX_ESST_LOCAL_PLANNER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "utilities/particle_space.hpp"
#include <boost/thread/thread.hpp>
#include <prx_planning/parallel_propAction.h>
#include <actionlib/client/simple_action_client.h>

namespace prx
{
    namespace plan
    {

        class esst_local_planner_t : public local_planner_t
        {

          public:

            esst_local_planner_t() : local_planner_t(){ }
            /**
             * @copydoc local_planner_t::init()
             */
            virtual void init(const util::parameter_reader_t * const reader, const util::parameter_reader_t * const template_reader);

            virtual ~esst_local_planner_t(){ }
            /**
             * @copydoc local_planner_t::steer( const sim::state_t*, const sim::state_t*, sim::plan_t&, sim::trajectory_t& , bool connect)
             */
            virtual void steer(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& plan, sim::trajectory_t& traj, bool connect = true);
            /**
             * @copydoc local_planner_t::steer( const sim::state_t*, const sim::state_t*, sim::plan_t&, sim::state_t*, bool connect )
             */
            virtual void steer(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& plan, sim::state_t* result, bool connect = true);

            void link_particle_space(util::particle_space_t* p_space);

            virtual void propagate_step(const sim::state_t* start, const sim::plan_t& plan, sim::state_t* state);

            virtual void propagate(const sim::state_t* start, const sim::plan_t& plan, sim::trajectory_t& traj);

          protected:

            void setup_parallel_models();

            void parallel_steer_traj(const util::particle_point_t* particle_point, sim::trajectory_t* traj, sim::plan_t* plan, sim::plan_t* noisy_plan,
                                    int random_number, world_model_t* which_model, int start_index, int end_index);

            void parallel_steer_point(const util::particle_point_t* particle_point, util::particle_point_t* particle_result, sim::plan_t& plan, sim::plan_t* noisy_plan,
                                    int random_number, world_model_t* which_model, int start_index, int end_index);

            util::particle_space_t* particle_space;

            /** @brief Storage for computed trajectories. */
            sim::trajectory_t* trajs;
            /** @brief Storage for computed plans. */
            std::vector<sim::plan_t*> noisy_plans;

            std::vector<double> control_variance;
            /** @brief Lower bound on simulation steps to take when generating a trajectory. */
            int lower_multiple;
            /** @brief Upper bound on simulation steps to take when generating a trajectory. */
            int upper_multiple;
            
            boost::mutex mutex;

            std::vector<actionlib::SimpleActionClient<prx_planning::parallel_propAction>*> acs;

        };

    }
}

#endif
