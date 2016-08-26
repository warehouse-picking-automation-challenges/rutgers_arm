/**
 * @file empty_application.cpp
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


#include "simulation/homing_application.hpp"
#include "simulation/homing_controller.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include <pluginlib/class_list_macros.h>
#include <boost/range/adaptor/map.hpp>

PLUGINLIB_EXPORT_CLASS( prx::packages::two_dim_problems::homing_application_t, prx::sim::application_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    
    namespace packages
    {        
        namespace two_dim_problems
        {
            homing_application_t::homing_application_t() 
            {
                homing_controller = NULL;
            }

            homing_application_t::~homing_application_t() { }

            void homing_application_t::init(const parameter_reader_t * const reader)
            {
                application_t::init(reader);
                
                // Get the obstacles//Index for the readers
                unsigned index = 0;
                config_list_t obstacle_configs;
                foreach(system_ptr_t ob, simulator->get_obstacles() | boost::adaptors::map_values)
                {
                    dynamic_cast< obstacle_t* >(ob.get())->update_phys_configs(obstacle_configs, index);
                    ++index;
                }
                
                // Get All Systems and Find the homing controller
                std::vector<system_ptr_t> get_systems;
                sys_graph.get_systems(get_systems);

                foreach(system_ptr_t sys, get_systems)
                {
                    homing_controller_t* check = dynamic_cast<homing_controller_t*>(sys.get());
                    if(homing_controller == NULL && check!= NULL)
                    {
                        // Pass the obstacles to the homing controller
                        homing_controller = check;
                        homing_controller->set_landmarks(obstacle_configs);
                        
                    }
                }
                
                PRX_ASSERT(homing_controller != NULL);
                
            }
            void homing_application_t::set_selected_path(const std::string& path)
            {
                std::string name;
                std::string subpath;
                boost::tie(name, subpath) = reverse_split_path(path);
                selected_path = name;
                PRX_DEBUG_COLOR("The selected path is now: " << selected_path.c_str(),PRX_TEXT_GREEN);
            }

            void homing_application_t::set_selected_point(double x, double y, double z)
            {
                selected_point.set(x, y, z);
                simulation_control->at(0) = x;
                simulation_control->at(1) = y;
            }
        }

    }
}

