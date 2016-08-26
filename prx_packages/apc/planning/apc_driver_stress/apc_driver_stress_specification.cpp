
/**
 * @file simple_pap_specification.cpp
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

#include "apc_driver_stress_specification.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS(prx::packages::apc::apc_driver_stress_specification_t, prx::plan::specification_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        
        namespace apc
        {
            apc_driver_stress_specification_t::apc_driver_stress_specification_t()
            {
                sampler = NULL;
            }

            apc_driver_stress_specification_t::~apc_driver_stress_specification_t()
            {
                clear();
            }

            void apc_driver_stress_specification_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                specification_t::init(reader, template_reader);  
                if( parameters::has_attribute("sampler", reader, template_reader) )
                {
                    sampler = parameters::initialize_from_loader<sampler_t > ("prx_planning", reader, "sampler", template_reader, "sampler");
                }
                else
                {
                    PRX_FATAL_S("Missing sampler attribute in planning specification!");
                }                     
            }
            // void apc_driver_stress_specification_t::setup(world_model_t * const model)
            // {
            //     specification_t::setup( model );
                
            //     validity_checker->link_model(model);
            // }
        }
        
    }
}
