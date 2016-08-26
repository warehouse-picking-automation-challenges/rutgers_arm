/**
 * @file fixed_wing.hpp 
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

#ifndef PRX_FIXED_WING_HPP
#define	PRX_FIXED_WING_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/plants/integration_plant.hpp"

namespace prx 
{ 
    namespace sim 
    {

        /**
        */
        class fixed_wing_t : public integration_plant_t
        {
            public:

                fixed_wing_t();

                virtual ~fixed_wing_t();

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
                const static unsigned STATE_V;
                const static unsigned STATE_ROLL;
                const static unsigned STATE_PITCH;
                const static unsigned STATE_YAW;
                const static unsigned STATE_FLIGHT;
                const static unsigned STATE_THRUST;

                const static unsigned CONTROL_THRUST;
                const static unsigned CONTROL_ROLL;
                const static unsigned CONTROL_PITCH;


                double _x;
                double _y;
                double _z;
                double _roll;
                double _pitch;
                double _yaw;
                double _v;
                double _flight;
                double _thrust;


                double _des_thrust;
                double _des_roll;
                double _des_pitch;

                mutable util::quaternion_t quat;

        };

    }
}

#endif

