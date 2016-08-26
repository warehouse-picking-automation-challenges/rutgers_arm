/**
 * @file cart_pole.hpp
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

#ifndef PRX_CART_POLE_HPP
#define	PRX_CART_POLE_HPP


#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/plants/integration_plant.hpp"

namespace prx
{
    namespace sim
    {

        /**
         * Represents a cart pole system. Implementation of the integration functions 
         * for simulating a cart pole system.\n
         * State: [x, xdot, theta, thetadot] \n
         * Control: [acceleration].
         * 
         * @brief <b> Represents a cart pole system. </b>
         * 
         * @author Zakary Littlefield 
         * 
         */
        class cart_pole_t : public integration_plant_t
        {

          public:
            cart_pole_t();

            virtual ~cart_pole_t();

            /** @copydoc plant_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*) */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /** @copydoc plant_t::propagate(const double) */
            virtual void propagate(const double simulation_step = 0);

            /** @copoydoc plant_t::update_phys_configs(util::config_list_t&, unsigned& index) const */
            void update_phys_configs(util::config_list_t& configs, unsigned& index) const;

          protected:
            /** @copoydoc integration_plant_t::update_derivative (state_t* const) */
            virtual void update_derivative(state_t * const result);

            /**
             * Indexer for state variable : X.
             * @brief Indexer for state variable : X.
             */
            const static unsigned STATE_X;

            /**
             * Indexer for state variable : V.
             * @brief Indexer for state variable : V.
             */
            const static unsigned STATE_V;

            /**
             * Indexer for state variable : THETA.
             * @brief Indexer for state variable : THETA.
             */
            const static unsigned STATE_THETA;

            /**
             * Indexer for state variable : W.
             * @brief Indexer for state variable : W.
             */
            const static unsigned STATE_W;

            /**
             * Indexer for control variable : A.
             * @brief Indexer for control variable : A.
             */
            const static unsigned CONTROL_A;

            /**
             * Internal state for the variable \c x, x coordinate.
             * @brief Internal state for the variable \c x, x coordinate.
             */
            double _x;

            /**
             * Internal state for the variable \c v, velocity.
             * @brief Internal state for the variable \c v, velocity.
             */
            double _v;

            /**
             * Internal state for the variable \c theta, angle.
             * @brief Internal state for the variable \c theta, angle.
             */
            double _theta;

            /**
             * Internal state for the variable \c w, rotational velocity.
             * @brief Internal state for the variable \c w, rotational velocity.
             */
            double _w;

            /**
             * Internal control for the variable \c a, acceleration.
             * @brief Internal control for the variable \c a, acceleration.
             */
            double _a;

            /**
             * Height value of the cart.
             * @brief Height value of the cart.
             */
            double _z;

            /**
             * Length of the pendulum.
             * @brief Length of the pendulum.
             */
            double length;

            /**
             * Mass of the cart and pendulum.
             * @brief Mass of the cart and pendulum.
             */
            double mc, mp;

            /**
             * Force of gravity.
             * @brief Force of gravity.
             */
            double g;

            /**
             * The length of the pendulum for visualization.
             * @brief The length of the pendulum for visualization.
             */
            double viz_length;

        };

    }
}

#endif

