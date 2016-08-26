/**
 * @file ping_pong_ball.cpp 
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


#include "prx/simulation/systems/plants/ping_pong_ball.hpp"
#include "prx/utilities/math/2d_geometry/angle.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::ping_pong_ball_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    namespace sim
    {

        ping_pong_ball_t::ping_pong_ball_t() : integration_plant_t()
        {
            state_memory = {&_x,&_y,&_z,&_vx,&_vy,&_vz};

            state_space = new space_t("THREE_D_BODY", state_memory);
            input_control_space = new space_t("EMPTY", control_memory);
        }

        ping_pong_ball_t::~ping_pong_ball_t() { }

        void ping_pong_ball_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            //    PRX_DEBUG_S("@@@Init ping_pong_ball_t : " << pathname);
            integration_plant_t::init(reader, template_reader);
        }

        void ping_pong_ball_t::propagate(const double simulation_step)
        {
            //    integrator->integrate(derivative_function, simulation_step);
            integration_plant_t::propagate(simulation_step);
        }

        void ping_pong_ball_t::update_phys_configs(config_list_t& configs, unsigned& index) const
        {
            root_config.set_position(_x,_y,_z);
            root_config.set_xyzw_orientation(0.0, 0.0, 0,  1);
            plant_t::update_phys_configs(configs, index);
        }

        void ping_pong_ball_t::update_derivative(state_t * const result)
        {
            result->memory[0] = _vx;
            result->memory[1] = _vy;
            result->memory[2] = _vz;
            result->memory[3] = 0;
            result->memory[4] = 0;
            result->memory[5] = -9.8;
            if(_z+_vz*simulation::simulation_step < 0 && _vz<0)
            {
                result->memory[5] = (-1.9 * _vz)/simulation::simulation_step;
            }

        }

    }
}