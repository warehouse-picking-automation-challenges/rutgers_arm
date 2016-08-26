/**
 * @file grasp_generator.hpp
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

#ifndef PRX_GRASP_GENERATOR_HPP
#define PRX_GRASP_GENERATOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/heuristic_search/constraints.hpp"

#include "prx/simulation/plan.hpp"
#include "prx/simulation/trajectory.hpp"

#include "prx/planning/planner.hpp"
#include "planning/specifications/grasping_specification.hpp"
#include "prx/planning/queries/query.hpp"

#include "planning/manipulation_world_model.hpp"
#include "planning/modules/grasp.hpp"
#include "planning/queries/grasping_query.hpp"
#include "prx/planning/task_planners/task_planner.hpp"

#include "planning/modules/planner_info.hpp"


namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
            /**
             * Grasp Interface class for grasp generation
             *
             * @authors Andrew Kimmel
             */
            class grasp_generator_t
            {

              public:

                grasp_generator_t();
                virtual ~grasp_generator_t();

                static pluginlib::ClassLoader<grasp_generator_t>& get_loader();

                /**
                 * @copydoc planner_t::init()
                 */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);


                /**
                 * @copydoc planner_t::link_world_model()
                 */
                virtual void link_world_model(plan::world_model_t * const model);

                virtual const std::vector<grasp_t>& compute_grasps(const util::geometry_t* ee_geom, const util::config_t& ee_local_config, movable_body_plant_t* object) = 0;

                virtual bool get_grasps(std::vector<grasp_t>& returned_grasps, const std::string& object_type);

                std::string& get_stat_string();

                void append_to_stat_string(const std::string& input);
                
              protected:
                
                /**
                 * @brief Convert the input state to an output config, if the state is an SE3 point
                 * @param state The input state of the object
                 * @param config The SE3 config of the object 
                 */
                void state_to_config(util::config_t& config, const sim::state_t* state);
                void config_to_euler_state(const util::config_t& config, sim::state_t* st);
                void config_to_state(const util::config_t& config, sim::state_t* st);
                // Manipulation world model
                manipulation_world_model_t* manipulation_model;             
                // The vector of grasp_t linked into the evaluator
                util::hash_t<std::string, std::vector<grasp_t> >  grasps;                
                /** @brief The pluginlib loader which is returned by the get_loader() class. */
                static pluginlib::ClassLoader<grasp_generator_t> loader;

                // Context of the ffee
                std::string ffee_context_name;
                //Flag to toggle saving output to a file
                bool save_grasps_to_file;
                //Name of the end effector being used
                std::string end_effector_name;
                std::string statistics;

            };
        }
    }
}


#endif
