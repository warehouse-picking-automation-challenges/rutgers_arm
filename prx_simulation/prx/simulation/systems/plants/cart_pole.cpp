/**
 * @file cart_pole.cpp
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

#include "prx/simulation/systems/plants/cart_pole.hpp"
#include "prx/utilities/math/2d_geometry/angle.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::cart_pole_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    namespace sim
    {

#define I 10
#define L 2.5
#define M 10
#define m 5


        const unsigned cart_pole_t::STATE_X = 0;
        const unsigned cart_pole_t::STATE_THETA = 2;
        const unsigned cart_pole_t::STATE_V = 1;
        const unsigned cart_pole_t::STATE_W = 3;
        const unsigned cart_pole_t::CONTROL_A = 0;

        cart_pole_t::cart_pole_t() : integration_plant_t()
        {
            state_memory = {&_x,&_v,&_theta,&_w};
            control_memory = {&_a};

            state_space = new space_t("XXdRRd", state_memory);
            input_control_space = new space_t("Xdd", control_memory);

            _z = 0;
            length = L;
            viz_length = L;
            g = 9.8;
        }

        cart_pole_t::~cart_pole_t() { }

        void cart_pole_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            integration_plant_t::init(reader, template_reader);
            _z = parameters::get_attribute_as<double>("z", reader, template_reader, 1.5);
        }

        void cart_pole_t::propagate(const double simulation_step)
        {
            integration_plant_t::propagate(simulation_step);
            _theta = norm_angle_pi(_theta);
        }

        void cart_pole_t::update_phys_configs(config_list_t& configs, unsigned& index) const
        {
            root_config.set_position(_x, 0, _z);
            root_config.set_xyzw_orientation(0.0, 0.0, 0.0, 1.0);
            
            augment_config_list(configs,index);
            configs[index].first = root_geom;
            configs[index].second = root_config;
            index++;
            
            augment_config_list(configs,index);
            configs[index].first = config_names[1];
            configs[index].second.zero();
            configs[index].second.set_position(_x + (viz_length / 2.0) * sin(_theta), -(viz_length / 2.0) * cos(_theta), _z);
            configs[index].second.set_xyzw_orientation(0.0, 0.0, cos((_theta) / 2.0), -sin((_theta) / 2.0));

            // plant_t::update_phys_configs(configs);
        }

        void cart_pole_t::update_derivative(state_t * const result)
        {
            result->memory[STATE_X] = _v;
            result->memory[STATE_THETA] = _w;
            double mass_term = (M + m)*(I + m * L * L) - m * m * L * L * cos(_theta) * cos(_theta);
            mass_term = (1.0 / mass_term);
            result->memory[STATE_V] = ((I + m * L * L)*(_a + m * L * _w * _w * sin(_theta)) + m * m * L * L * cos(_theta) * sin(_theta) * g) * mass_term;
            result->memory[STATE_W] = ((-m * L * cos(_theta))*(_a + m * L * _w * _w * sin(_theta))+(M + m)*(-m * g * L * sin(_theta))) * mass_term;
        }

        //vector_t cart_pole_t::get_diff_wrt_state(state_t* in_state,control_t* in_control)
        //{
        //    vector_t return_val(16);
        //    const element_iterator_t& st = state_space->get_element_iterator(in_state);
        //    const element_iterator_t& ct = input_control_space->get_element_iterator(in_control);
        //    
        //    return_val[0]=return_val[1]=return_val[3]=return_val[4]=return_val[5]=return_val[6]=return_val[8]=return_val[10]=return_val[12]=return_val[14]=0;
        //    return_val[2]=return_val[7]=1;
        //    
        //    double denom = mc + mp*pow(sin(st[STATE_THETA]),2.0);
        //    
        //    double s = sin(st[STATE_THETA]);
        //    double s2 = sin(2*st[STATE_THETA]);
        //    double c = cos(st[STATE_THETA]);
        //    
        //    return_val[9]=(-1*mp*s2/pow(denom,2.0))*(ct[CONTROL_A]+mp*s*(length*pow(st[STATE_W],2.0)+g*c))+(1/denom)*(mp*c*(length*pow(st[STATE_W],2.0)+g*c) + mp*s*(length*pow(st[STATE_W],2.0)-g*s));
        //    
        //    return_val[11]=2*mp*s*length*st[STATE_W]/denom;
        //    
        //    return_val[13]=(mp*s2/(length*pow(denom,2.0)))*(ct[CONTROL_A]*c+mp*length*pow(st[STATE_W],2.0)*c*s+ (mc+mp)*g*s) + 1.0/(length*denom)*(ct[CONTROL_A]*s + mp*length*pow(st[STATE_W],2.0)*s*s - mp*length*pow(st[STATE_W],2.0)*c*c - (mp+mc)*g*c);
        //    
        //    return_val[15]=2*mp*c*s*st[STATE_W]/denom;
        //    
        //    
        //    return return_val;
        //}
        //
        //vector_t cart_pole_t::get_diff_wrt_control(state_t* in_state,control_t* in_control)
        //{
        //    vector_t return_val(4);
        //    const element_iterator_t& st = state_space->get_element_iterator(in_state);    
        //    
        //    double denom = mc + mp*pow(sin(st[STATE_THETA]),2.0);
        //    
        //    return_val[0]=return_val[1] = 0;
        //    return_val[2] = 1.0/denom;
        //    return_val[3] = -1.0*cos(st[STATE_THETA])/(length*denom);
        //    
        //    return return_val;
        //}


    }
}

