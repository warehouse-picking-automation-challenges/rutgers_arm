/**
 * @file simple_place_specification.cpp
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

#include "planning/specifications/simple_place_specification.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::simple_place_specification_t, prx::plan::specification_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {

        namespace manipulation
        {
            simple_place_specification_t::simple_place_specification_t()
            {
            }

            simple_place_specification_t::~simple_place_specification_t()
            {
                clear();
            }

            void simple_place_specification_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                specification_t::init(reader, template_reader);                       

                if( parameters::has_attribute("validity_checker", reader, template_reader) )
                {
                    validity_checker = parameters::initialize_from_loader<validity_checker_t > ("prx_planning", reader, "validity_checker", template_reader, "validity_checker");
                }
                else
                {
                    PRX_FATAL_S("Missing validity_checker attribute in simple PAP specification!");
                }                
            }
        }
        
    }
}
