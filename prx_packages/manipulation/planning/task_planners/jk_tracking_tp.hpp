/**
 * @file simple_pap_tp.hpp
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

#ifndef PRX_JK_TRACKING_TP_HPP
#define PRX_JK_TRACKING_TP_HPP

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
#include "utilities/definitions/manip_defs.hpp"


namespace prx
{
    namespace util
    {
        class constraints_t;
    }

    namespace plan
    {

        class validity_checker_t;
    }

    namespace packages
    {
        namespace manipulation
        {
            class manipulation_validity_checker_t;
            
            /**
             * A task planner for handling jk-tracking trees
             *
             * @authors Andrew Kimmel
             */
            class jk_tracking_tp_t : public plan::task_planner_t
            {

              public:

                jk_tracking_tp_t();
                virtual ~jk_tracking_tp_t();

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

                virtual bool resolve_ffee_query(sim::trajectory_t& ffee_trajectory);
                virtual bool track_ffee_trajectory(sim::trajectory_t& ffee_trajectory);
                bool attempt_jac_steer(sim::plan_t& output_plan, sim::state_t* start_state, sim::state_t* result_state, util::config_t& goal_config);


                void euler_state_to_config(const sim::state_t* st, util::config_t& config);
                void setup_constraints();
                void delete_constraints();
                void visualize_trajectory(const sim::trajectory_t& traj, const std::string& vis_geom_name);


                manipulation_world_model_t* manipulation_model;
                manipulator_t* manipulator;
                sim::state_t* manip_initial_state;
                util::constraints_t* valid_constraints, *tmp_constraint;
                manipulation_validity_checker_t* manip_validity_checker;
                plan::validity_checker_t* validity_checker;
                util::config_t ee_local_config;

                // First entry: ffee, second entry: manipulator
                std::vector< std::pair<std::string, std::string> > paired_manipulation_contexts;
                std::vector<movable_body_plant_t* > objects;

                /** @brief The input manipulation query */
                manipulation_query_t* manipulation_query;

                plan::motion_planning_query_t* in_query;

                std::string full_manipulator_context_name;
                std::string manipulation_task_planner_name;
                std::vector<double> object_target_vec;
                std::string object_name;

            };
        }
    }
}


#endif
