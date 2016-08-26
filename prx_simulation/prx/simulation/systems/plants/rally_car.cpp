/**
 * @file rally_car.cpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2011, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors  Zakary Littlefield, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */


#include "prx/simulation/systems/plants/rally_car.hpp"
#include "prx/utilities/math/2d_geometry/angle.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>
#include <cmath>

PLUGINLIB_EXPORT_CLASS( prx::sim::rally_car_t, prx::sim::system_t)
        
namespace prx
{
    using namespace util;
    namespace sim
    {
        #define M 1450
        #define IZ 2740
        #define LF 1.3
        #define LR 1.4
        #define R .3
        #define IF 1.8 
        #define IR 1.8
        #define H .4    
        #define B 7
        #define C 1.6
        #define D .52

        #define CRBRAKE 700
        #define CRACC 0
        #define CFBRAKE 700
        #define CFACC 1000

        const unsigned rally_car_t::STATE_X = 0;
        const unsigned rally_car_t::STATE_Y = 1;
        const unsigned rally_car_t::STATE_VX = 2;
        const unsigned rally_car_t::STATE_VY = 3;
        const unsigned rally_car_t::STATE_THETA = 4;
        const unsigned rally_car_t::STATE_THETADOT = 5;
        const unsigned rally_car_t::STATE_WF = 6;
        const unsigned rally_car_t::STATE_WR = 7;
        const unsigned rally_car_t::CONTROL_STA = 0;
        const unsigned rally_car_t::CONTROL_TF = 1;
        const unsigned rally_car_t::CONTROL_TR = 2;

        rally_car_t::rally_car_t() : integration_plant_t()
        {
            state_memory = { &_x , &_y ,&_vx,&_vy,&_theta,&_thetadot,&_wf,&_wr};
            control_memory = { &_sta ,&_tf,&_tr};
            state_space = new space_t( "RallyCar", state_memory );
            input_control_space = new space_t( "WheelControl", control_memory );

            _z = 0;    
        }

        rally_car_t::~rally_car_t() 
        {
        }

        void rally_car_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
        //    PRX_DEBUG_S("@@@Init second_order_wheeled_car_t : " << pathname);
            integration_plant_t::init(reader,template_reader);
            _z = parameters::get_attribute_as<double>("z",reader,template_reader,.5);
        }

        void rally_car_t::propagate(const double simulation_step)
        {
            integration_plant_t::propagate(simulation_step);
        }

        void rally_car_t::update_phys_configs(config_list_t& configs,unsigned& index) const
        {
            root_config.set_position(_x, _y, _z);
            root_config.set_xyzw_orientation(0.0, 0.0, sin(_theta / 2.0), cos(_theta / 2.0));
            plant_t::update_phys_configs( configs,index );
        }

        void rally_car_t::update_derivative (state_t* const result)
        {
            result->memory[STATE_X] = _vx;
            result->memory[STATE_Y] = _vy;
            result->memory[STATE_THETA] = _thetadot;

            double V = sqrt(_vx*_vx+_vy*_vy);
            double beta = atan2(_vy,_vx) - _theta;
            double V_Fx = V*cos(beta-_sta) + _thetadot*LF*sin(_sta);
            double V_Fy = V*sin(beta-_sta) + _thetadot*LF*cos(_sta);
            double V_Rx = V*cos(beta);
            double V_Ry = V*sin(beta) - _thetadot*LR;

            double s_Fx = (V_Fx - _wf*R)/(_wf*R);
            double s_Fy = V_Fy/(_wf*R);
            double s_Rx = (V_Rx - _wr*R)/(_wr*R);
            double s_Ry = V_Ry/(_wr*R);

            double s_F = sqrt(s_Fx*s_Fx+s_Fy*s_Fy);
            double s_R = sqrt(s_Rx*s_Rx+s_Ry*s_Ry);

            double mu_F = D*sin(C*atan(B*s_F));
            double mu_R = D*sin(C*atan(B*s_R));
            double mu_Fx;
            double mu_Fy;
            if(std::isfinite(s_Fx))
                mu_Fx = -1*(s_Fx/s_F)*mu_F;
            else
                mu_Fx = -mu_F;
            if(std::isfinite(s_Fy))
                mu_Fy = -1*(s_Fy/s_F)*mu_F;
            else
                mu_Fy = -mu_F;
            double mu_Rx;
            double mu_Ry;
            if(std::isfinite(s_Rx))
                mu_Rx = -1*(s_Rx/s_R)*mu_R;
            else
                mu_Rx = -mu_R;
            if(std::isfinite(s_Ry))
                mu_Ry = -1*(s_Ry/s_R)*mu_R;
            else
                mu_Ry = -mu_R;
            
            double fFz = (LR*M*(9.8) - H*M*9.8*mu_Rx) / (LF+LR+H*(mu_Fx*cos(_sta)-mu_Fy*sin(_sta)-mu_Rx));
            double fRz = M*9.8 - fFz;

            double fFx = mu_Fx * fFz;
            double fFy = mu_Fy * fFz;
            double fRx = mu_Rx * fRz;
            double fRy = mu_Ry * fRz;;


            result->memory[STATE_VX] = (fFx*cos(_theta+_sta)-fFy*sin(_theta+_sta)+fRx*cos(_theta)-fRy*sin(_theta) )/M;
            result->memory[STATE_VY] = (fFx*sin(_theta+_sta)+fFy*cos(_theta+_sta)+fRx*sin(_theta)+fRy*cos(_theta) )/M;
            result->memory[STATE_THETADOT] = ((fFy*cos(_sta)+fFx*sin(_sta))*LF - fRy*LR)/IZ;
            result->memory[STATE_WF] = (_tf-fFx*R)/IF;
            result->memory[STATE_WR] = (_tr-fRx*R)/IR;
        }
    }
}
