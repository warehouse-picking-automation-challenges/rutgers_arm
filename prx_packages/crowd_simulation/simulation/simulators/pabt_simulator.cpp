/**
 * @file pabt_simulator.cpp
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


#include "simulation/simulators/pabt_simulator.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/simulation/systems/obstacle.hpp"
#include "prx/simulation/system_graph.hpp"

#include "boost/filesystem.hpp"
#include <boost/tuple/tuple.hpp>
#include <boost/range/adaptor/map.hpp> // boost::tie

 #include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(prx::packages::crowd::pabt_simulator_t, prx::sim::simulator_t)


namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {        
        namespace crowd
        {

            pabt_simulator_t::pabt_simulator_t() { }

            pabt_simulator_t::~pabt_simulator_t() { }

            void pabt_simulator_t::propagate_and_respond()
            {
                sim::simulation::simulation_time += ros::Duration ( sim::simulation::simulation_step );
                if( active )
                {
                    for( subsystems_iter = subsystems.begin(); subsystems_iter != subsystems.end(); ++subsystems_iter )
                    {
                        if( subsystems_iter->second->is_active() )
                        {
                            subsystems_iter->second->propagate(sim::simulation::simulation_step);
                        }
                    }
                }
            }

            void pabt_simulator_t::push_control(const control_t * const control)
            {
                input_control_space->copy_from_point(control);
                if( active )
                {
                    for( subsystems_iter = subsystems.begin(); subsystems_iter != subsystems.end(); ++subsystems_iter )
                    {
                        if( subsystems_iter->second->is_active() )
                        {
                            subsystems_iter->second->compute_control();
                        }
                    }
                }
            }
        }
    }
}
