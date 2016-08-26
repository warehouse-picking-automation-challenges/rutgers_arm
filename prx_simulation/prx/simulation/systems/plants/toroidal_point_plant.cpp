/**
 * @file toroidal_point_plant.cpp 
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

#include "prx/simulation/systems/plants/toroidal_point_plant.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/simulation/plan.hpp"

#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::toroidal_point_plant_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    namespace sim
    {

        toroidal_point_plant_t::toroidal_point_plant_t() : plant_t()
        {
            reset = false;
        }

        void toroidal_point_plant_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
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

            //Now we need to make the state/control space... 
            std::string space_string("R");
            if( dimension > 1 )
            {
                for( unsigned i = 0; i < dimension - 1; ++i )
                {
                    space_string += "|R";
                }
            }

            state_space = new space_t(space_string, state_memory);
            input_control_space = new space_t(space_string, control_memory);

            plant_t::init(reader, template_reader);

            initial_state = state_space->alloc_point();
            state = state_space->alloc_point();
            prior_state = state_space->alloc_point();
            hold_state = state_space->alloc_point();

            control = input_control_space->alloc_point();
            prior_control = input_control_space->alloc_point();

            dist = 2;
            interpolation_step = 0;
        }

        void toroidal_point_plant_t::propagate(const double simulation_step)
        {
            state_space->copy_to_point(state);
            input_control_space->copy_to_point(control);

            if( state_space->equal_points(state, hold_state, PRX_ZERO_CHECK) )
                interpolation_step = 0;
            else
                reset = true;

            if( (!input_control_space->equal_points(control, prior_control, PRX_ZERO_CHECK)) || reset )
            {
                input_control_space->copy_point(prior_control, control);

                state_space->copy_point(initial_state, state);

                interpolation_step = max_step / state_space->distance(initial_state, control);

                dist = 0;
                reset = false;
            }

            dist += interpolation_step;

            if( dist <= 1 )
            {
                state_space->copy_to_point(hold_state);
                state_space->interpolate(initial_state, control, dist, state);
                state_space->copy_from_point(state);
            }
            else
            {
                state_space->copy_from_point(control);
            }

            //Store previous control and the now current state.
            state_space->copy_point(prior_state, state);
            input_control_space->copy_point(prior_control, control);
        }

        void toroidal_point_plant_t::update_phys_configs(config_list_t& configs, unsigned& index) const
        {
            if( dimension >= 3 )
                root_config.set_position(X[0], X[1], X[2]);
            else
                root_config.set_position(X[0], X[1], 0.2);
            root_config.set_xyzw_orientation(0.0, 0.0, 0.0, 1.0);
            plant_t::update_phys_configs(configs, index);
        }

        void toroidal_point_plant_t::steering_function(const state_t* start, const state_t* goal, plan_t& result_plan)
        {
            double time = std::ceil(state_space->distance(start, goal) / max_step);
            time *= simulation::simulation_step;

            result_plan.copy_onto_back(goal, time);
        }

    }
}

