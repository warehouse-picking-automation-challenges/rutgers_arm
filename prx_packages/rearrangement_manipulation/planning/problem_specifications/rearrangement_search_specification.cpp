/**
 * @file rearrangement_search_specification.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "planning/problem_specifications/rearrangement_search_specification.hpp"

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/world_model.hpp"

#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::rearrangement_manipulation::rearrangement_search_specification_t, prx::plan::specification_t)

namespace prx
{
    using namespace util;
    using namespace plan;

    namespace packages
    {
        namespace rearrangement_manipulation
        {

            rearrangement_search_specification_t::rearrangement_search_specification_t() { }

            rearrangement_search_specification_t::~rearrangement_search_specification_t() { }

            void rearrangement_search_specification_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
            {
                specification_t::init(reader, template_reader);
                if( parameters::has_attribute("k", reader, template_reader) )
                    k_objects = parameters::get_attribute_as<unsigned>("k", reader, template_reader);
                else
                    PRX_FATAL_S("You need to specify the number of the objects in the experiment, under the tag k !");

                goal_biasing = parameters::get_attribute_as<int>("goal_biasing", reader, template_reader, 5);
                time_limit = parameters::get_attribute_as<double>("time_limit", reader, template_reader, 1800);
                max_tries = parameters::get_attribute_as<int>("max_tries", reader, template_reader, 10);

                if( parameters::has_attribute("poses_file", reader, template_reader) )
                    poses_file = parameters::get_attribute("poses_file", reader, template_reader);
                else
                    PRX_FATAL_S("You need to specify the poses file for the experiment, under the tag poses_file !");
                
                if( parameters::has_attribute("poses_constraints_file", reader, template_reader) )
                    poses_constraints_file = parameters::get_attribute("poses_constraints_file", reader, template_reader);
                else
                    PRX_FATAL_S("You need to specify the poses constraints file for the experiment, under the tag poses_constraints_file !");
                
                if( parameters::has_attribute("transit_graph_file", reader, template_reader) )
                    transit_graph_file = parameters::get_attribute("transit_graph_file", reader, template_reader);
                else
                    PRX_FATAL_S("You need to specify the transit graph file for the experiment, under the tag transit_graph_file !");

                if( parameters::has_attribute("transfer_graph_file", reader, template_reader) )
                    transfer_graph_file = parameters::get_attribute("transfer_graph_file", reader, template_reader);
                else
                    PRX_FATAL_S("You need to specify the transfer graph file for the experiment, under the tag transfer_graph_file !");

                if( parameters::has_attribute("safe_position", reader, template_reader) )
                    safe_position = parameters::get_attribute_as<std::vector<double> >("safe_position", reader, template_reader);
                else
                    PRX_FATAL_S("No safe position is specified for the problem!");
                
                gather_statistics = parameters::get_attribute_as<bool>("gather_statistics", reader, template_reader, false);
                if(parameters::has_attribute("statistics_file", reader, template_reader) )
                    statistics_file = parameters::get_attribute("statistics_file", reader, template_reader);
                else if(gather_statistics)
                    PRX_WARN_S("You have not specified statistics file. Your statistics will not be saved.");
            }
        }
    }
}