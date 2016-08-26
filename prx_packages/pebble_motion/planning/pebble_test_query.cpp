/**
 * @file pebble_test_query.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2012, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Athanasios Krontiris, Andrew Dobson, Andrew Kimmel, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "pebble_test_query.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS( prx::packages::pebble_motion::pebble_test_query_t, prx::plan::query_t)

namespace prx
{
    using namespace util;
    using namespace plan;
    
    namespace packages
    {
        namespace pebble_motion
        {

            pebble_test_query_t::pebble_test_query_t() { }

            pebble_test_query_t::~pebble_test_query_t() { }

            void pebble_test_query_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                const parameter_reader_t* curr_reader = NULL;

                if( reader->has_attribute("initial_state") )
                {
                    curr_reader = reader;
                }
                else if( template_reader != NULL && template_reader->has_attribute("initial_state") )
                {
                    curr_reader = template_reader;
                }
                else
                {
                    PRX_ERROR_S("Missing initial state attribute from planning query.");
                }

                if( curr_reader != NULL )
                {

                    foreach(const parameter_reader_t* initial_state_reader, curr_reader->get_list("initial_state"))
                    {
                        initial_state.push_back(std::make_pair(initial_state_reader->get_attribute("name"), initial_state_reader->get_attribute_as<int>("node_id")));
                    }
                }

                curr_reader = NULL;
                if( reader->has_attribute("target_state") )
                {
                    curr_reader = reader;
                }
                else if( template_reader != NULL && template_reader->has_attribute("target_state") )
                {
                    curr_reader = template_reader;
                }
                else
                {
                    PRX_ERROR_S("Missing target state attribute from planning query.");
                }

                if( curr_reader != NULL )
                {

                    foreach(const parameter_reader_t* target_state_reader, curr_reader->get_list("target_state"))
                    {
                        target_state.push_back(std::make_pair(target_state_reader->get_attribute("name"), target_state_reader->get_attribute_as<int>("node_id")));
                    }
                }

                PRX_ASSERT(initial_state.size() == target_state.size());

            }

        }
    }
}
