/**
 * @file simple_pendulum.cpp 
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


#include "prx/simulation/systems/plants/simple_pendulum.hpp"
#include "prx/utilities/math/2d_geometry/angle.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::simple_pendulum_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    namespace sim
    {

        const unsigned simple_pendulum_t::STATE_THETA = 0;
        const unsigned simple_pendulum_t::STATE_V = 1;
        const unsigned simple_pendulum_t::CONTROL_TAU = 0;

        simple_pendulum_t::simple_pendulum_t() : integration_plant_t()
        {
            state_memory = {&_theta,&_thetadot};
            control_memory = {&_tau};

            state_space = new space_t("OneLink", state_memory);
            input_control_space = new space_t("Rdd", control_memory);

            length = 20.0;
            mass = 1.0;
            damp = 0.0;

        }

        simple_pendulum_t::~simple_pendulum_t() { }

        void simple_pendulum_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            //    PRX_DEBUG_S("@@@Init simple_pendulum_t : " << pathname);
            integration_plant_t::init(reader, template_reader);
        }

        void simple_pendulum_t::propagate(const double simulation_step)
        {
            //    integrator->integrate(derivative_function, simulation_step);
            integration_plant_t::propagate(simulation_step);

            //    _theta = simulation_step * _thetadot ;
            //                            //torque - mass * 9.81 * length * cos(th) * 0.5 - damp * vel) * 3 / (mass * length * length);
            //    _thetadot = simulation_step *  ((_tau - mass * (9.81) * length * cos(_theta) )*0.5 - damp * _thetadot )* 3 / ( mass * length * length );
            //    
            //    if(_thetadot<-10)
            //        _thetadot = -10;
            //    else if(_thetadot > 10)
            //        _thetadot = 10;
            //    
            //    
            //    if( _theta > PRX_PI )
            //	_theta -= PRX_2PI;
            //    if( _theta < -PRX_PI )
            //	_theta += PRX_2PI;
        }

        void simple_pendulum_t::update_phys_configs(config_list_t& configs, unsigned& index) const
        {
            root_config.set_position((length) * sin(_theta + PRX_HALF_PI), -1 * (length) * cos(_theta + PRX_HALF_PI), 1.5);
            root_config.set_xyzw_orientation(0.0, 0.0, sin((_theta) / 2.0), cos((_theta) / 2.0));
            plant_t::update_phys_configs(configs, index);
        }

        void simple_pendulum_t::update_derivative(state_t * const result)
        {
            double length = 1.0;
            result->memory[STATE_THETA] = _thetadot;
            //torque - mass * 9.81 * length * cos(th) * 0.5 - damp * vel) * 3 / (mass * length * length);
            result->memory[STATE_V] = (_tau - mass * (9.81) * length * cos(_theta)*0.5 - damp * _thetadot)* 3 / (mass * length * length);
        }

        //vector_t simple_pendulum_t::get_diff_wrt_state(state_t* in_state,control_t* in_control)
        //{
        //    double length = 1.0;
        //    vector_t return_val(4);
        //    const element_iterator_t& st = state_space->get_element_iterator(in_state);
        //    return_val[0] = return_val[3] = 0;
        //    return_val[1] = 1;
        //    return_val[2] = (-9.81/length)*cos(st[STATE_THETA]);    
        //    return return_val;
        //    
        //}
        //vector_t simple_pendulum_t::get_diff_wrt_control(state_t* in_state,control_t* in_control)
        //{
        //    
        //    double length = 1.0;
        //    vector_t return_val(2);
        //    return_val[0] = 0;
        //    return_val[1] = 1.0/(mass*length*length);
        //    
        //    return return_val;
        //}

    }
}