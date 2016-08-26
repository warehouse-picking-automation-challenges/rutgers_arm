/**
 * @file simple_pendulum.hpp 
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

#ifndef SIMPLE_PENDULUM_HPP
#define SIMPLE_PENDULUM_HPP

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
        class simple_pendulum_t : public integration_plant_t
        {

          public:

            simple_pendulum_t();

            virtual ~simple_pendulum_t();

            /** @copydoc integration_plant_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*) */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /** @copydoc integration_plant_t::propagate(const double) */
            void propagate(const double simulation_step);

            /** @copoydoc integration_plant_t::update_phys_configs(util::config_list_t&, unsigned& ) const */
            virtual void update_phys_configs(util::config_list_t& configs, unsigned& index) const;

          protected:

            /** @copoydoc integration_plant_t::update_derivative (state_t* const) */
            virtual void update_derivative(state_t * const result);

            /**
             * Indexer for state variable : THETA.
             * @brief Indexer for state variable : THETA.
             */
            const static unsigned STATE_THETA;

            /**
             * Indexer for state variable : V.
             * @brief Indexer for state variable : V.
             */
            const static unsigned STATE_V;

            /**
             * Indexer for control variable TAU.
             * @brief Indexer for control variable TAU.
             */
            const static unsigned CONTROL_TAU;

            /**
             * Internal state memory for theta.
             * @brief Internal state memory for theta.
             */
            double _theta;

            /**
             * Internal state memory for rotational velocity.
             * @brief Internal state memory for rotational velocity.
             */
            double _thetadot;

            /**
             * Internal control memory for torque.
             * @brief Internal control memory for torque.
             */
            double _tau;

            /**
             * Length of the pendulum.
             * @brief Length of the pendulum.
             */
            double length;

            /**
             * Mass of the end of the pendulum.
             * @brief Mass of the end of the pendulum.
             */
            double mass;

            /**
             * Damping factor.
             * @brief Damping factor.
             */
            double damp;

        };

    }
}

#endif