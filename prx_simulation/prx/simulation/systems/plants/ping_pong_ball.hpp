/**
 * @file ping_pong_ball.hpp 
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
#pragma once

#ifndef PRX_PING_PONG_HPP
#define PRX_PING_PONG_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/plants/integration_plant.hpp"

namespace prx
{
    namespace sim
    {

        /**
         * Represents a pendulum system. Implementation of the integration functions 
         * for simulating a pendulum.\n
         * State: [] \n
         * Control: [].
         * 
         * @brief <b> Represents a pendulum system. </b>
         * 
         * @author Zakary Littlefield
         * 
         */
        class ping_pong_ball_t : public integration_plant_t
        {

          public:

            ping_pong_ball_t();

            virtual ~ping_pong_ball_t();

            /** @copydoc integration_plant_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*) */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /** @copydoc integration_plant_t::propagate(const double) */
            void propagate(const double simulation_step);

            /** @copoydoc integration_plant_t::update_phys_configs(util::config_list_t&, unsigned& ) const */
            virtual void update_phys_configs(util::config_list_t& configs, unsigned& index) const;

          protected:

            /** @copoydoc integration_plant_t::update_derivative (state_t* const) */
            virtual void update_derivative(state_t * const result);

            double _x,_y,_z,_vx,_vy,_vz;

        };

    }
}

#endif