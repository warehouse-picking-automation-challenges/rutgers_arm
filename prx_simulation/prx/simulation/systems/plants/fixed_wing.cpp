/**
 * @file fixed_wing.cpp 
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

#include "prx/simulation/systems/plants/fixed_wing.hpp"
#include "prx/utilities/math/2d_geometry/angle.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <pluginlib/class_loader.h>

#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>

#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::sim::fixed_wing_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    namespace sim
    {        
        #define G 9.81
        #define k 1.86

        const unsigned fixed_wing_t::STATE_X = 0;
        const unsigned fixed_wing_t::STATE_Y = 1;
        const unsigned fixed_wing_t::STATE_Z = 2;
        const unsigned fixed_wing_t::STATE_V = 3;
        const unsigned fixed_wing_t::STATE_ROLL = 4;
        const unsigned fixed_wing_t::STATE_PITCH = 5;
        const unsigned fixed_wing_t::STATE_YAW = 6;
        const unsigned fixed_wing_t::STATE_FLIGHT = 7;
        const unsigned fixed_wing_t::STATE_THRUST = 8;

        const unsigned fixed_wing_t::CONTROL_THRUST = 0;
        const unsigned fixed_wing_t::CONTROL_ROLL = 1;
        const unsigned fixed_wing_t::CONTROL_PITCH = 2;

        fixed_wing_t::fixed_wing_t() : integration_plant_t()
        {
            state_memory = {&_x,&_y,&_z,&_v,&_roll,&_pitch,&_yaw,&_flight,&_thrust};
            control_memory = {&_des_thrust,&_des_roll,&_des_pitch};

            state_space = new space_t("FixedWing", state_memory);
            input_control_space = new space_t("AirControl", control_memory);
        }

        fixed_wing_t::~fixed_wing_t() { }

        void fixed_wing_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            integration_plant_t::init(reader, template_reader);
        }

        void fixed_wing_t::propagate(const double simulation_step)
        {
            integration_plant_t::propagate(simulation_step);
            // if(ros::this_node::getName()=="/simulation")
            //     PRX_PRINT_S(state_space->print_memory(4));
        }

        void fixed_wing_t::update_phys_configs(config_list_t& configs,unsigned& index) const
        {
            root_config.set_position(_x, _y, _z);
            quat.set_from_euler(-_roll,-_pitch,_yaw);
            root_config.set_orientation(quat);
            plant_t::update_phys_configs(configs,index);
        }
        void fixed_wing_t::append_contingency(plan_t& result_plan, double duration)
        {
            result_plan.link_control_space(input_control_space);
            result_plan.augment_plan(duration);
            result_plan.back().control->at(0) = 0;
            result_plan.back().control->at(1) = 0;
            result_plan.back().control->at(2) = 0;
        }

        void fixed_wing_t::update_derivative(state_t * const result)
        {
            double CL = .3 + 2.5*_pitch;
            double CD = .03 + .3*CL*CL;
            double cflight = cos(_flight);
            double sflight = sin(_flight);
            double spitch = sin(_pitch);
            result->memory[STATE_X] =       _v * cflight * cos(_yaw);
            result->memory[STATE_Y] =       _v * cflight * sin(_yaw);
            result->memory[STATE_Z] =       _v * sflight ;
            result->memory[STATE_V] =       ( _thrust*cos(_pitch) - k*_v*_v*CD ) - G*sflight;

            // if(_v > PRX_ZERO_CHECK)
            // {
                result->memory[STATE_FLIGHT] =  ( _thrust*spitch/_v + k*_v*CL )*cos(_roll) - G*cflight/_v;
                result->memory[STATE_YAW] =     ( _thrust*spitch/_v + k*_v*CL )*(sin(_roll)/cflight);
            // }
            // else
            // {
            //     result->memory[STATE_FLIGHT] =  0;
            //     result->memory[STATE_YAW] =     0;
            // }


            result->memory[STATE_THRUST] =  _des_thrust-_thrust;
            result->memory[STATE_ROLL] =    _des_roll-_roll;
            result->memory[STATE_PITCH] =   _des_pitch-_pitch;

            // if(ros::this_node::getName()=="/simulation")
            //     PRX_PRINT_S(state_space->print_point(result,4));
        }
    }
}

