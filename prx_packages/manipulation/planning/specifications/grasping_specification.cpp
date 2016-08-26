/**
 * @file grasping_specification.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "planning/specifications/grasping_specification.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::grasping_specification_t, prx::plan::specification_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {

        namespace manipulation
        {
            grasping_specification_t::grasping_specification_t()
            {
                validity_checker = NULL;
            }

            grasping_specification_t::~grasping_specification_t()
            {
                clear();
            }

            void grasping_specification_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                specification_t::init(reader, template_reader);                       
                PRX_DEBUG_COLOR("Inside grasping planner specification init", PRX_TEXT_CYAN);
                
                if( parameters::has_attribute("validity_checker", reader, template_reader) )
                {
                    validity_checker = parameters::initialize_from_loader<validity_checker_t > ("prx_planning", reader, "validity_checker", template_reader, "validity_checker");
                }
                else
                {
                    PRX_FATAL_S("Missing validity_checker attribute in grasping planning specification!");
                }
            }
            
        }
    }
}
