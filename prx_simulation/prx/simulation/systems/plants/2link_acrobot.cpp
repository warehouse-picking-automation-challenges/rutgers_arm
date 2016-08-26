/**
 * @file 2link_acrobot.cpp 
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

#include "prx/simulation/systems/plants/2link_acrobot.hpp"

#include "prx/utilities/math/2d_geometry/angle.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::two_link_acrobot_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    namespace sim
    {
        const unsigned two_link_acrobot_t::STATE_THETA_1 = 0;
        const unsigned two_link_acrobot_t::STATE_THETA_2 = 1;
        const unsigned two_link_acrobot_t::STATE_V_1 = 2;
        const unsigned two_link_acrobot_t::STATE_V_2 = 3;
        const unsigned two_link_acrobot_t::CONTROL_T = 0;

        two_link_acrobot_t::two_link_acrobot_t() : integration_plant_t()
        {
            state_memory = {&_theta1,&_theta2,&_theta1dot,&_theta2dot};
            control_memory = {&_tau};

            state_space = new space_t("TwoLink", state_memory);
            input_control_space = new space_t("Rdd", control_memory);

            length = 20.0;
            m = 1.0;

        }

        two_link_acrobot_t::~two_link_acrobot_t() { }

        void two_link_acrobot_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            //    PRX_DEBUG_S("@@@Init two_link_acrobot_t : " << pathname);
            integration_plant_t::init(reader, template_reader);
        }

        void two_link_acrobot_t::propagate(const double simulation_step)
        {
            integration_plant_t::propagate(simulation_step);
            _theta1 = norm_angle_pi(_theta1);
            _theta2 = norm_angle_pi(_theta2);
        }

        void two_link_acrobot_t::update_phys_configs(config_list_t& configs, unsigned& index) const
        {
            augment_config_list(configs,index);
            configs[index].first = config_names[2];
            configs[index].second.set_position((length / 2.0) * cos(_theta1 - M_PI / 2), (length / 2.0) * sin(_theta1 - M_PI / 2), 1.5);
            configs[index].second.set_orientation(0, 0, sin((_theta1 - M_PI / 2) / 2.0), cos((_theta1 - M_PI / 2) / 2.0));
            index++;

            augment_config_list(configs,index);
            configs[index].first = config_names[1];
            configs[index].second.set_position((length) * cos(_theta1 - M_PI / 2)+(length / 2.0) * cos(_theta1 + _theta2 - M_PI / 2),
                                               (length) * sin(_theta1 - M_PI / 2)+(length / 2.0) * sin(_theta1 + _theta2 - M_PI / 2),
                                               1.5);
            configs[index].second.set_orientation(0, 0, sin((_theta1 + _theta2 - M_PI / 2) / 2.0), cos((_theta1 + _theta2 - M_PI / 2) / 2.0));
            index++;

            augment_config_list(configs,index);
            configs[index].first = config_names[0];
            configs[index].second.set_position((length) * cos(_theta1 - M_PI / 2)+(length) * cos(_theta1 + _theta2 - M_PI / 2),
                                               (length) * sin(_theta1 - M_PI / 2)+(length) * sin(_theta1 + _theta2 - M_PI / 2),
                                               1.5);
            configs[index].second.zero();
            index++;
        }

        void two_link_acrobot_t::update_derivative(state_t * const result)
        {
            double lc = .5;
            double lc2 = .25;
            double l2 = 1;
            double theta2 = _theta2;
            double theta1 = _theta1 - M_PI / 2;
            double theta1dot = _theta1dot;
            double theta2dot = _theta2dot;
            double I1 = 0.2;
            double I2 = 1.0;
            double l = 1.0;
            double g = 9.8;

            //extra term m*lc2
            double d11 = m * lc2 + m * (l2 + lc2 + 2 * l * lc * cos(theta2)) + I1 + I2;

            double d22 = m * lc2 + I2;
            double d12 = m * (lc2 + l * lc * cos(theta2)) + I2;
            double d21 = d12;

            //extra theta1dot
            double c1 = -m * l * lc * theta2dot * theta2dot * sin(theta2) - (2 * m * l * lc * theta1dot * theta2dot * sin(theta2));
            double c2 = m * l * lc * theta1dot * theta1dot * sin(theta2);
            double g1 = (m * lc + m * l) * g * cos(theta1) + (m * lc * g * cos(theta1 + theta2));
            double g2 = m * lc * g * cos(theta1 + theta2);

            result->memory[STATE_THETA_1] = _theta1dot;
            result->memory[STATE_THETA_2] = _theta2dot;

            double u2 = _tau - 1 * .1 * _theta2dot;
            double u1 = -1 * .1 * _theta1dot;
            double theta1dot_dot = (d22 * (u1 - c1 - g1) - d12 * (u2 - c2 - g2)) / (d11 * d22 - d12 * d21);
            double theta2dot_dot = (d11 * (u2 - c2 - g2) - d21 * (u1 - c1 - g1)) / (d11 * d22 - d12 * d21);


            result->memory[STATE_V_1] = theta1dot_dot;
            result->memory[STATE_V_2] = theta2dot_dot;

        }


    }
}