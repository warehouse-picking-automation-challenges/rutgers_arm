/**
 * @file first_order_car.cpp 
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

#include "prx/simulation/systems/plants/first_order_car.hpp"
#include "prx/utilities/math/2d_geometry/angle.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <pluginlib/class_loader.h>

#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>

#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::first_order_car_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    namespace sim
    {

        const unsigned first_order_car_t::STATE_X = 0;
        const unsigned first_order_car_t::STATE_Y = 1;
        const unsigned first_order_car_t::STATE_THETA = 2;
        const unsigned first_order_car_t::CONTROL_V = 0;
        const unsigned first_order_car_t::CONTROL_W = 1;

        first_order_car_t::first_order_car_t() : integration_plant_t()
        {
            state_memory = {&_x,&_y,&_theta};
            control_memory = {&_v,&_w};

            state_space = new space_t("SE2", state_memory);
            input_control_space = new space_t("Vector", control_memory);

            _z = 0;
        }

        first_order_car_t::~first_order_car_t() { }

        void first_order_car_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            //    PRX_DEBUG_S("@@@Init first_order_car_t : " << pathname);
            integration_plant_t::init(reader, template_reader);
            _z = parameters::get_attribute_as<double>("z", reader, template_reader, 1.5);
        }

        void first_order_car_t::propagate(const double simulation_step)
        {
            integration_plant_t::propagate(simulation_step);
            //    st[STATE_THETA] = norm_angle_pi(st[STATE_THETA]);  //Why did we have this here?
        }

        void first_order_car_t::update_phys_configs(config_list_t& configs, unsigned& index) const
        {
            root_config.set_position(_x, _y, _z);
            root_config.set_xyzw_orientation(0.0, 0.0, sin(_theta / 2.0), cos(_theta / 2.0));
            plant_t::update_phys_configs(configs, index);
        }
        
        void first_order_car_t::update_collision_info()
        {
            root_config.set_position(_x, _y, _z);
            root_config.set_xyzw_orientation(0.0, 0.0, sin(_theta / 2.0), cos(_theta / 2.0));
            plant_t::update_collision_info();
        }

        void first_order_car_t::update_derivative(state_t * const result)
        {
            result->memory[STATE_X] = cos(_theta) * _v;
            result->memory[STATE_Y] = sin(_theta) * _v;
            result->memory[STATE_THETA] = _w;
        }

    }
}

