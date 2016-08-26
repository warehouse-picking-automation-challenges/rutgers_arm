/**
 * @file two_link_cart.cpp 
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

#include "prx/simulation/systems/plants/two_link_cart.hpp"

#include "prx/utilities/math/2d_geometry/angle.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::two_link_cart_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    namespace sim
    {
        const unsigned two_link_cart_t::STATE_THETA_1 = 0;
        const unsigned two_link_cart_t::STATE_THETA_2 = 2;
        const unsigned two_link_cart_t::STATE_V_1 = 1;
        const unsigned two_link_cart_t::STATE_V_2 = 3;
        const unsigned two_link_cart_t::STATE_X = 4;
        const unsigned two_link_cart_t::STATE_VX = 5;
        const unsigned two_link_cart_t::CONTROL_F = 0;

        two_link_cart_t::two_link_cart_t() : integration_plant_t()
        {
            state_memory = {&_theta1,&_theta1dot,&_theta2,&_theta2dot,&_x,&_xdot};
            control_memory = {&_force};

            state_space = new space_t("R|X|R|X|X|X", state_memory);
            input_control_space = new space_t("Xdd", control_memory);

        }

        two_link_cart_t::~two_link_cart_t() { }

        void two_link_cart_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            //    PRX_DEBUG_S("@@@Init two_link_cart_t : " << pathname);
            integration_plant_t::init(reader, template_reader);
        }

        void two_link_cart_t::propagate(const double simulation_step)
        {
            integration_plant_t::propagate(simulation_step);
            _theta1 = norm_angle_pi(_theta1);
            _theta2 = norm_angle_pi(_theta2);
        }

        void two_link_cart_t::update_phys_configs(config_list_t& configs, unsigned& index) const
        {
            double length = .5;
            augment_config_list(configs,index);
            configs[index].first = config_names[3];
            configs[index].second.set_position( _x , 0, 0);
            configs[index].second.set_orientation(0, 0, 0, 1);
            index++;

            augment_config_list(configs,index);
            configs[index].first = config_names[2];
            configs[index].second.set_position( _x + (length / 2.0) * cos(_theta1 + M_PI / 2), (length / 2.0) * sin(_theta1 + M_PI / 2), 0);
            configs[index].second.set_orientation(0, 0, sin((_theta1 + M_PI / 2) / 2.0), cos((_theta1 + M_PI / 2) / 2.0));
            index++;

            augment_config_list(configs,index);
            configs[index].first = config_names[1];
            configs[index].second.set_position(_x + (length) * cos(_theta1 + M_PI / 2)+(length / 2.0) * cos(_theta1 + _theta2 + M_PI / 2),
                                               (length) * sin(_theta1 + M_PI / 2)+(length / 2.0) * sin(_theta1 + _theta2 + M_PI / 2),
                                               0);
            configs[index].second.set_orientation(0, 0, sin((_theta1 + _theta2 + M_PI / 2) / 2.0), cos((_theta1 + _theta2 + M_PI / 2) / 2.0));
            index++;

            augment_config_list(configs,index);
            configs[index].first = config_names[0];
            configs[index].second.set_position(_x + (length) * cos(_theta1 + M_PI / 2)+(length) * cos(_theta1 + _theta2 + M_PI / 2),
                                               (length) * sin(_theta1 + M_PI / 2)+(length) * sin(_theta1 + _theta2 + M_PI / 2),
                                               .5);
            configs[index].second.set_orientation(0, 0, 0, 1);
            // configs[index].second.zero();
            index++;
        }

        void two_link_cart_t::update_derivative(state_t * const result)
        {

            result->memory[STATE_THETA_1] = _theta1dot;
            result->memory[STATE_THETA_2] = _theta2dot;
            result->memory[STATE_X] = _xdot;

            double s1 = sin(_theta1);
            double s2 = sin(_theta2);
            double c1 = cos(_theta1);
            double c2 = cos(_theta2);
            double s12 = sin(_theta1 - _theta2);
            double c12 = cos(_theta2 - _theta2);
            double sq1tdot = _theta1dot*_theta1dot;
            double sq2tdot = _theta2dot*_theta2dot;

            double theta1dot_dot = (2/(.05 - .025*c12*c12))*(.4905*s1 - .24525*c12*s2 - .0125*c12*s12*
                                    sq1tdot-.0125*s12*sq2tdot) + ((.00015625*c1-.000078125*c12*c2)*
                                    (-(.00015625*c1-.000078125*c12*c2)*(-.00625*c12*
                                        (-.122625*s2-.00625*s12*sq1tdot)+.00625*(-.24525*s1+.00625*s12*sq2tdot)) + 
                                    (.000078125 - .0000390625*c12*c12)*(-.0125*c2*(-.122625*s2-.00625*s12*sq1tdot) +
                                        .00625*(-_force - .025*s1*sq1tdot - .0125*s2*sq2tdot))))/
                                ((.000078125 - .0000390625*c12*c12)*(-(.00015625*c1-.000078125*c12*c2)*(.00015625*c1-.000078125*c12*c2)+
                                    (.000078125 - .0000390625*c12*c12)*(.0065625 - .00015625*c2*c2)));
            
            double theta2dot_dot = (2/(.05 - .025*c12*c12))*(-.4905*c12*s1+.4905*s2+
                                .025*s12*sq1tdot+.0125*c12*s12*sq2tdot) + 
                            (2*(.05*c1*c12 - .05*c2)*(-(.00015625*c1-.000078125*c12*c2)*
                                (-.00625*c12*(-.122625*s2-.00625*s12*sq1tdot)+
                                .00625*(-.24525*s1+.00625*s12*sq2tdot)) + 
                               (.000078125 - .0000390625*c12*c12)*(-.0125*c2*(-.122625*s2-.00625*s12*sq1tdot)+
                                .00625*(-_force - .025*s1*sq1tdot - .0125*s2*sq2tdot))))/
                            ((-.05+.025*c12*c12)*(-(.00015625*c1-.000078125*c12*c2)*(.00015625*c1-.000078125*c12*c2)+
                                (.000078125 - .0000390625*c12*c12)*(.0065625 - .00015625*c2*c2)));
            
            double xdot_dot = -(-(.00015625*c1-.000078125*c12*c2)*(-.00625*c12*
                            (-.122625*s2-.00625*s12*sq1tdot)+.00625*(-.24525*s1+.00625*s12*sq2tdot)) + 
                        (.000078125 - .0000390625*c12*c12)*(-.0125*c2*(-.122625*s2-.00625*s12*sq1tdot) +
                            .00625*(-_force - .025*s1*sq1tdot - .0125*s2*sq2tdot)))/
                        (-(.00015625*c1-.000078125*c12*c2)*(.00015625*c1-.000078125*c12*c2)+(.000078125 - .0000390625*c12*c12)*
                            (.0065625 - .00015625*c2*c2));

            result->memory[STATE_V_1] = theta1dot_dot;
            result->memory[STATE_V_2] = theta2dot_dot;
            result->memory[STATE_VX] = xdot_dot;

        }
    }
}