/**
 * @file rearrangement_search_query.cpp
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

#include "planning/queries/rearrangement_search_query.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::labeled_rearrangement_manipulation::rearrangement_search_query_t, prx::plan::query_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace labeled_rearrangement_manipulation
        {

            rearrangement_search_query_t::rearrangement_search_query_t() { }

            rearrangement_search_query_t::~rearrangement_search_query_t() { }

            void rearrangement_search_query_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {

                if( parameters::has_attribute("initial_poses", reader, template_reader) )
                {

                    foreach(const parameter_reader_t* s_reader, parameters::get_list("initial_poses", reader, template_reader))
                    {
                        initial_poses.push_back(s_reader->get_attribute_as<std::vector<double> >("pose"));
                    }
                }
                else
                {
                    PRX_WARN_S("Missing initial poses for the objects from rearrangement manipulation specification!");
                }

                if( parameters::has_attribute("target_poses", reader, template_reader) )
                {

                    foreach(const parameter_reader_t* s_reader, parameters::get_list("target_poses", reader, template_reader))
                    {

                        target_poses.push_back(s_reader->get_attribute_as<std::vector<double> >("pose"));
                    }
                }
                else
                {
                    PRX_WARN_S("Missing target poses for the objects from rearrangement manipulation specification!");
                }
            }
        }
    }
}
