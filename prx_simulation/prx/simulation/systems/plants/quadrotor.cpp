/**
 * @file quadrotor.cpp 
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "prx/simulation/systems/plants/quadrotor.hpp"
#include "prx/utilities/math/2d_geometry/angle.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <pluginlib/class_loader.h>

#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>

#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::sim::quadrotor_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    namespace sim
    {        
        #define G 9.81
        #define M 1
        #define B .0000542
        #define IXX .0081
        #define IYY .0081
        #define IZZ .0142
        #define L .25
        #define D .0000011
        #define JTP .000104
        const unsigned quadrotor_t::STATE_X = 0;
        const unsigned quadrotor_t::STATE_Y = 1;
        const unsigned quadrotor_t::STATE_Z = 2;
        const unsigned quadrotor_t::STATE_ROLL = 3;
        const unsigned quadrotor_t::STATE_PITCH = 4;
        const unsigned quadrotor_t::STATE_YAW = 5;
        const unsigned quadrotor_t::STATE_X_DOT = 6;
        const unsigned quadrotor_t::STATE_Y_DOT = 7;
        const unsigned quadrotor_t::STATE_Z_DOT = 8;
        const unsigned quadrotor_t::STATE_P = 9;
        const unsigned quadrotor_t::STATE_Q = 10;
        const unsigned quadrotor_t::STATE_R = 11;


        const unsigned quadrotor_t::CONTROL_W1 = 0 ;
        const unsigned quadrotor_t::CONTROL_W2 = 1;
        const unsigned quadrotor_t::CONTROL_W3 = 2;
        const unsigned quadrotor_t::CONTROL_W4 = 3;

        quadrotor_t::quadrotor_t() : integration_plant_t()
        {
            state_memory = {&_x,&_y,&_z,&_roll,&_pitch,&_yaw,&_xdot,&_ydot,&_zdot,&_p,&_q,&_r};
            control_memory = {&_w1,&_w2,&_w3,&_w4};

            state_space = new space_t("XYZ|R|R|R|X|X|X|X|X|X", state_memory);
            input_control_space = new space_t("X|X|X|X", control_memory);
        }

        quadrotor_t::~quadrotor_t() { }

        void quadrotor_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            integration_plant_t::init(reader, template_reader);
        }

        void quadrotor_t::propagate(const double simulation_step)
        {
            integration_plant_t::propagate(simulation_step);
        }

        void quadrotor_t::update_phys_configs(config_list_t& configs,unsigned& index) const
        {
            root_config.set_position(_x, _y, _z);
            quat.set_from_euler(_roll,_pitch,_yaw);
            root_config.set_orientation(quat);
            plant_t::update_phys_configs(configs,index);
        }
        void quadrotor_t::append_contingency(plan_t& result_plan, double duration)
        {
            result_plan.link_control_space(input_control_space);
            result_plan.augment_plan(duration);
            result_plan.back().control->at(0) = 100000;
            result_plan.back().control->at(1) = 100000;
            result_plan.back().control->at(2) = 100000;
            result_plan.back().control->at(3) = 100000;
        }

        void quadrotor_t::update_derivative(state_t * const result)
        {
            result->memory[STATE_X] = _xdot;
            result->memory[STATE_Y] = _ydot;
            result->memory[STATE_Z] = _zdot;

            double o1 = PRX_SIGN(_w1)*_w1*_w1;
            double o2 = PRX_SIGN(_w2)*_w2*_w2;
            double o3 = PRX_SIGN(_w3)*_w3*_w3;
            double o4 = PRX_SIGN(_w4)*_w4*_w4;
            double u1 = B*(o1+o2+o3+o4);
            double u2 = B*L*(o4-o2);
            double u3 = B*L*(o3-o1);
            double u4 = D*(o2+o4-o1-o3);
            double O = -_w1+_w2-_w3+_w4;

            double sphi = sin(_roll);
            double cphi = cos(_roll);
            double stheta = sin(_pitch);
            double ctheta = cos(_pitch);
            double spsi = sin(_yaw);
            double cpsi = cos(_yaw);


            result->memory[STATE_X_DOT] = spsi*sphi+cpsi*stheta*cphi*u1/M - _xdot*1.2;
            result->memory[STATE_Y_DOT] = -cpsi*sphi+spsi*stheta*cphi*u1/M - _ydot*1.2;
            result->memory[STATE_Z_DOT] = -G + ctheta*cphi*u1/M;
            result->memory[STATE_P] = ((IYY-IZZ)*_q*_r-JTP*_q*O+u2)/IXX;
            result->memory[STATE_Q] = ((IZZ-IXX)*_p*_r+JTP*_p*O+u3)/IYY;
            result->memory[STATE_R] = ((IXX-IYY)*_p*_q+u4)/IZZ;
            result->memory[STATE_ROLL] = _p + sphi*tan(_pitch)*_q+cphi*tan(_pitch)*_r;
            result->memory[STATE_PITCH] = cphi*_q-sphi*_r;
            result->memory[STATE_YAW] = sphi*_q/ctheta + cphi*_r/ctheta;
        }
    }
}

