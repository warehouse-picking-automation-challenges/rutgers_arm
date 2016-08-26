/**
 * @file apc_grasping_planner.hpp
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

#ifndef PRX_APC_GRASPING_PLANNER_HPP
#define	PRX_APC_GRASPING_PLANNER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/spaces/space.hpp"

#include "prx/simulation/plan.hpp"
#include "prx/simulation/trajectory.hpp"

#include "prx/planning/planner.hpp"
#include "prx/planning/problem_specifications/specification.hpp"
#include "prx/planning/queries/query.hpp"

#include "planning/manipulation_world_model.hpp"
#include "planning/queries/grasping_query.hpp"
#include "planning/task_planners/grasping_planner.hpp"

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {

            /**
             * Manipulation task planner. Computes the path for moving an object from an
             * initial to a target position.
             *
             * @authors Andrew Kimmel, Zakary Littlefield
             */
             
            class apc_grasping_planner_t : public grasping_planner_t
            {

              public:

                apc_grasping_planner_t();
                virtual ~apc_grasping_planner_t();

                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                virtual void setup();

              protected:

                virtual bool evaluate_the_query();
                virtual void determine_top_and_front_faces(movable_body_plant_t* object, std::string& top_front_face, std::vector<std::string>& alternative_faces);

                virtual void determine_face(const util::vector_t& axis, const util::quaternion_t& rotation, std::string& face);

                virtual void prune_database(const std::vector<grasp_t>& grasp_database, const std::string& top_front_face, std::vector<bool>& non_duplicate_grasp_markers);

                // The order of the rotations to be evaluated
                std::vector<std::string> directions;
              };

        }
    }
}

#endif
