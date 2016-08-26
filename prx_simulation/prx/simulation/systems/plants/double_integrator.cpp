/**
 * @file double_integrator.cpp
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

#include "prx/simulation/systems/plants/double_integrator.hpp"
#include "prx/utilities/math/2d_geometry/angle.hpp"

#include "prx/utilities/spaces/space.hpp"

#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>

#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::double_integrator_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    namespace sim
    {

        const unsigned double_integrator_t::STATE_X = 0;
        const unsigned double_integrator_t::STATE_V = 1;
        const unsigned double_integrator_t::CONTROL_A = 0;

        double_integrator_t::double_integrator_t()
        {
            state_memory.push_back(&_x);
            state_memory.push_back(&_vx);

            control_memory.push_back(&_a);

            state_space = new space_t("XXd", state_memory);
            input_control_space = new space_t("Xdd", control_memory);

            _z = 0;
        }

        double_integrator_t::~double_integrator_t() { }

        void double_integrator_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            integration_plant_t::init(reader, template_reader);
            _z = parameters::get_attribute_as<double>("z", reader, template_reader, 1.5);
        }

        void double_integrator_t::propagate(const double simulation_step)
        {
            integration_plant_t::propagate(simulation_step);
        }

        void double_integrator_t::update_phys_configs(config_list_t& configs, unsigned& index) const
        {
            root_config.set_position(_x, 0, _z);
            root_config.set_xyzw_orientation(0.0, 0.0, 0.0, 1.0);
            plant_t::update_phys_configs(configs, index);
        }

        void set_param(const std::string& path, const std::string& parameter_name, const boost::any& value) { }

        void double_integrator_t::update_derivative(state_t * const result)
        {
            result->memory[STATE_X] = _vx;
            result->memory[STATE_V] = _a;
        }


    }
}

