/**
 * @file pose_generation.hpp
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

#ifndef PRX_POSE_GENERATION_TP_HPP
#define	PRX_POSE_GENERATION_TP_HPP


#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/spaces/space.hpp"

#include "prx/simulation/plan.hpp"
#include "prx/simulation/trajectory.hpp"

#include "prx/planning/task_planners/task_planner.hpp"
#include "prx/planning/problem_specifications/specification.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"

#include "planning/manipulation_world_model.hpp"
#include "planning/queries/manipulation_query.hpp"
#include "planning/queries/grasping_query.hpp"
#include "planning/task_planners/grasping_planner.hpp"


namespace prx
{
    namespace packages
    {
        using namespace manipulation; 

        namespace rearrangement_manipulation
        {

            struct grasp_pose_t
            {
                std::vector<double> state;
                std::vector<util::config_t> valid_grasps;
                std::vector<int> grasp_modes;
                std::vector<int> release_modes;

                grasp_pose_t(std::vector<double> pose_state)
                {
                    state = pose_state;
                }

                ~grasp_pose_t(){}

                void clear()
                {
                    state.clear();
                    valid_grasps.clear();
                    grasp_modes.clear();
                }

                void add_new_grasp(const util::config_t& grasp, int grasp_mode, int release_mode )
                {
                    valid_grasps.push_back(grasp);
                    grasp_modes.push_back(grasp_mode);
                    release_modes.push_back(release_mode);
                }
            };

            /**
             * Evaluate a set of grasps on a variety of poses and determine success and timing data.
             *
             * @authors Athanasios Krontiris
             */
            class pose_generation_tp_t : public plan::task_planner_t
            {

              public:

                pose_generation_tp_t();
                virtual ~pose_generation_tp_t();

                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                /**
                 * @copydoc motion_planner_t::reset()
                 */
                virtual void reset();

                /**
                 * @copydoc motion_planner_t::link_world_model()
                 */
                virtual void link_world_model(plan::world_model_t * const model);

                /**
                 * @copydoc motion_planner_t::get_statistics()
                 */
                virtual const util::statistics_t* get_statistics();

                /**
                 * @copydoc planner_t::link_specification(specification_t*)
                 */
                virtual void link_specification(plan::specification_t* new_spec);

                /**
                 * @copydoc motion_planner_t::link_query()
                 */
                virtual void link_query(plan::query_t* new_query);

                /**
                 * @copydoc motion_planner_t::setup()
                 *
                 * Will store poses and grasping data base for each pose of the specific type 
                 * of object. 
                 */
                virtual void setup();

                /**
                 * @copydoc motion_planner_t::execute()
                 */
                virtual bool execute();


                /**
                 * @copydoc motion_planner_t::succeeded() const
                 */
                virtual bool succeeded() const;

                /**
                 * @copydoc motion_planner_t::resolve_query()
                 *
                 * At the end of the resolve_query the algorithm will remove the vertices
                 * for the start and the goal for this specific query from the graph.
                 */
                virtual void resolve_query();

              protected:

                std::string prx_dir;

                /** Read from input **/
                std::string context_name;
                std::string save_folder;
                std::string environment;
                int nr_object_poses;
                std::vector< grasp_pose_t > valid_poses;


                util::hash_t<std::string, std::vector<std::pair< util::quaternion_t, std::vector<double> > > > initial_rotations;                
                std::vector<double> min_bounds, max_bounds;

                util::config_t retract_config;
                
                manipulation_world_model_t* manipulation_model;
                manipulation_context_info_t* current_manipulation_context_info;

                /** @brief The grasping planner.*/
                grasping_planner_t* grasping_planner;
                /** @brief The grasping query that the manipulation task planner will use to communicate with the grasping planner.*/
                grasping_query_t* grasping_query;

            };
        }
    }
}


#endif
