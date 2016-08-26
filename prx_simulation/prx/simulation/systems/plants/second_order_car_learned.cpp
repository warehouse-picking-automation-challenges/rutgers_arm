/**
 * @file second_order_car.cpp 
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
#ifdef PRX_LEARN_FOUND

#include "prx/simulation/systems/plants/second_order_car_learned.hpp"
#include "prx/utilities/math/2d_geometry/angle.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::second_order_car_learned_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    namespace sim
    {

        const unsigned second_order_car_learned_t::STATE_X = 0;
        const unsigned second_order_car_learned_t::STATE_Y = 1;
        const unsigned second_order_car_learned_t::STATE_THETA = 2;
        const unsigned second_order_car_learned_t::STATE_V = 3;
        const unsigned second_order_car_learned_t::STATE_W = 4;
        const unsigned second_order_car_learned_t::CONTROL_A = 0;
        const unsigned second_order_car_learned_t::CONTROL_STA = 1;

        second_order_car_learned_t::second_order_car_learned_t()
        {
            state_memory = {&_x,&_y,&_theta,&_v,&_w};
            control_memory = {&_a,&_sta};

            state_space = new space_t("SOCar", state_memory);
            input_control_space = new space_t("XddRdd", control_memory);

            // forward = new prx::learn::forward_car_model_t();

            _z = 0;
        }

        second_order_car_learned_t::~second_order_car_learned_t() { }

        void second_order_car_learned_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            //    PRX_DEBUG_S("@@@Init second_order_wheeled_car_t : " << pathname);
            plant_t::init(reader, template_reader);
            _z = parameters::get_attribute_as<double>("z", reader, template_reader, 1.5);
        }

        void second_order_car_learned_t::propagate(const double simulation_step)
        {
            // Rewrite here to use learned model

            std::vector<double> state_current = {_x,_y,_theta,_v,_w,_a,_sta,simulation_step};
            std::vector<double> state_updated;

            forward.forward_simulate(state_current, state_updated);

            _x = state_updated[0];
            _y = state_updated[1];
            _theta = state_updated[2];
            _v = state_updated[3];
            _w = state_updated[4];

        }

        void second_order_car_learned_t::update_phys_configs(config_list_t& configs, unsigned& index) const
        {
            root_config.set_position(_x, _y, _z);
            root_config.set_xyzw_orientation(0.0, 0.0, sin(_theta / 2.0), cos(_theta / 2.0));
            plant_t::update_phys_configs(configs, index);
        }
        
        void second_order_car_learned_t::update_collision_info()
        {
            root_config.set_position(_x, _y, _z);
            root_config.set_xyzw_orientation(0.0, 0.0, sin(_theta / 2.0), cos(_theta / 2.0));
            plant_t::update_collision_info();
        }

        void second_order_car_learned_t::update_derivative(state_t * const result)
        {
            result->memory[STATE_X] = cos(_theta) * cos(_w) * _v;
            result->memory[STATE_Y] = sin(_theta) * cos(_w) * _v;
            result->memory[STATE_THETA] = sin(_w) * _v;
            result->memory[STATE_V] = _a;
            result->memory[STATE_W] = _sta;
        }

    }
}

#endif


