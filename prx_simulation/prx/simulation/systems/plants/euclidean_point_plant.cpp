/**
 * @file euclidean_point_plant.cpp
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

#include "prx/simulation/systems/plants/euclidean_point_plant.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/simulation/plan.hpp"

#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::euclidean_point_plant_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    namespace sim
    {

        euclidean_point_plant_t::euclidean_point_plant_t() : kinematic_plant_t()
        {
        }

        void euclidean_point_plant_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            max_step = parameters::get_attribute_as<double>("max_step", reader, template_reader);
            dimension = parameters::get_attribute_as<double>("dimension", reader, template_reader);

            //Set up the state and control memory
            X.resize(dimension);
            U.resize(dimension);

            //Set the state and control memory to point to the actual memory
            state_memory.resize(dimension);
            control_memory.resize(dimension);
            for( unsigned i = 0; i < dimension; ++i )
            {
                state_memory[i] = &X[i];
                control_memory[i] = &U[i];
            }

            //Now we need to make the state/control space... hmmm...
            std::string space_string("X");
            if( dimension > 1 )
            {
                for( unsigned i = 0; i < dimension - 1; ++i )
                {
                    space_string += "|X";
                }
            }

            state_space = new space_t(space_string, state_memory);
            input_control_space = new space_t(space_string, control_memory);

            kinematic_plant_t::init(reader, template_reader);
        }

        void euclidean_point_plant_t::update_phys_configs(config_list_t& configs, unsigned& index) const
        {
            if( dimension >= 3 )
                root_config.set_position(X[0], X[1], X[2]);
            else
                root_config.set_position(X[0], X[1], 0.2);
            root_config.set_xyzw_orientation(0.0, 0.0, 0.0, 1.0);
            plant_t::update_phys_configs(configs, index);
        }

    }
}

