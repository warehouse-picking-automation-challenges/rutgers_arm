/**
 * @file quadrotor.hpp 
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
#pragma once

#ifndef PRX_QUADROTOR_HPP
#define	PRX_QUADROTOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/plants/integration_plant.hpp"

namespace prx 
{ 
    namespace sim 
    {

        /**
        */
        class quadrotor_t : public integration_plant_t
        {
            public:

                quadrotor_t();

                virtual ~quadrotor_t();

                /** @copydoc integration_plant_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*) */
                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

                /** @copydoc integration_plant_t::propagate(const double) */
                void propagate(const double simulation_step);

                void append_contingency(plan_t& result_plan, double duration);

                /** @copoydoc integration_plant_t::update_phys_configs(util::config_list_t&,unsigned& index) const */
                virtual void update_phys_configs(util::config_list_t& configs,unsigned& index) const;

            protected:

                /** @copoydoc integration_plant_t::update_derivative (state_t* const) */
                virtual void update_derivative(state_t * const result);
                const static unsigned STATE_X;
                const static unsigned STATE_Y;
                const static unsigned STATE_Z;
                const static unsigned STATE_X_DOT;
                const static unsigned STATE_Y_DOT;
                const static unsigned STATE_Z_DOT;
                const static unsigned STATE_ROLL;
                const static unsigned STATE_PITCH;
                const static unsigned STATE_YAW;
                const static unsigned STATE_P;
                const static unsigned STATE_Q;
                const static unsigned STATE_R;

                const static unsigned CONTROL_W1;
                const static unsigned CONTROL_W2;
                const static unsigned CONTROL_W3;
                const static unsigned CONTROL_W4;


                double _x;
                double _y;
                double _z;
                double _roll;
                double _pitch;
                double _yaw;
                double _xdot;
                double _ydot;
                double _zdot;
                double _p;
                double _q;
                double _r;


                double _w1;
                double _w2;
                double _w3;
                double _w4;

                mutable util::quaternion_t quat;

        };

    }
}

#endif

