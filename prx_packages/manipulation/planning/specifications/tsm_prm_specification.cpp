/**
 * @file tsm_prm_specification.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2016, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "planning/specifications/tsm_prm_specification.hpp"
#include "planning/local_planners/ik_steering_local_planner.hpp"
#include "planning/manipulation_world_model.hpp"
#include "planning/distance_functions/end_effector_distance.hpp"
#include "planning/manipulation_world_model.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"

#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/modules/heuristic_search/constraints_astar_search.hpp"
#include "planning/modules/grasp_evaluator.hpp"

#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(prx::plan::tsm_prm_specification_t, prx::plan::specification_t)

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace plan
    {

        tsm_prm_specification_t::tsm_prm_specification_t()
        {
            validity_checker = NULL;
            sampler = NULL;
            metric = NULL;
            local_planner = NULL;
            astar = NULL;
        }

        tsm_prm_specification_t::~tsm_prm_specification_t()
        {
            clear();

            delete ik_local_planner;
        }

        void tsm_prm_specification_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            motion_planning_specification_t::init(reader, template_reader);       

            grasp_evaluator = parameters::create_from_loader<packages::manipulation::grasp_evaluator_t> ("prx_planning", reader, "grasp_evaluator", template_reader, "grasp_evaluator");
            // Allocate ik steering local planner
            ik_local_planner = new ik_steering_local_planner_t();

        }

        void tsm_prm_specification_t::setup(world_model_t * const model)
        {
            motion_planning_specification_t::setup( model );

            ik_local_planner->link_model(model);
            grasp_evaluator->link_world_model(model);
        }

        void tsm_prm_specification_t::clear()
        {
            motion_planning_specification_t::clear();
        }
    }
}
