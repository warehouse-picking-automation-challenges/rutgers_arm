/**
 * @file first_order_car.hpp 
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

#ifndef PRX_FIRST_ORDER_CAR_HPP
#define	PRX_FIRST_ORDER_CAR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/plants/integration_plant.hpp"

namespace prx
{
    namespace sim
    {

        /**
         * Represents a first order car. Implementation of the integration functions 
         * for simulating a first order car.\n
         * State: [x, y, z, theta] \n
         * Control: [velocity, steering angle].
         * 
         * @brief <b> Represents a first order car. </b>
         * 
         * @author Andrew Dobson
         * 
         */
        class first_order_car_t : public integration_plant_t
        {

          public:

            first_order_car_t();

            virtual ~first_order_car_t();

            /** @copydoc integration_plant_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*) */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /** @copydoc integration_plant_t::propagate(const double) */
            void propagate(const double simulation_step);

            /** @copoydoc integration_plant_t::update_phys_configs(util::config_list_t&) const */
            virtual void update_phys_configs(util::config_list_t& configs, unsigned& index) const;
            
            virtual void update_collision_info();

          protected:

            /** @copoydoc integration_plant_t::update_derivative (state_t* const) */
            virtual void update_derivative(state_t * const result);

            /**
             * Indexer for state variable : X.
             * @brief Indexer for state variable : X.
             */
            const static unsigned STATE_X;

            /**
             * Indexer for state variable : Y.
             * @brief Indexer for state variable : Y.
             */
            const static unsigned STATE_Y;

            /**
             * Indexer for state variable : THETA.
             * @brief Indexer for state variable : THETA.
             */
            const static unsigned STATE_THETA;

            /**
             * Indexer for state variable : V.
             * @brief Indexer for state variable : V.
             */
            const static unsigned CONTROL_V;

            /**
             * Indexer for state variable : W.
             * @brief Indexer for state variable : W.
             */
            const static unsigned CONTROL_W;

            /** Internal state & control memory */
            /**
             * Internal state for the variable \c x, x coordinate.
             * @brief Internal state for the variable \c x, x coordinate.
             */
            double _x;

            /**
             * Internal state for the variable \c y, y coordinate.
             * @brief Internal state for the variable \c y, y coordinate.
             */
            double _y;

            /**
             * Internal state for the variable \c theta, angle.
             * @brief Internal state for the variable \c theta, angle.
             */
            double _theta;

            /**
             * Internal control for the variable \c v, velocity.
             * @brief Internal control for the variable \c v, velocity.
             */
            double _v;

            /**
             * Internal state for the variable \c w, steering angel.
             * @brief Internal state for the variable \c w, steering angel.
             */
            double _w;

            /**
             * Height value of the car.
             * @brief Height value of the car.
             */
            double _z;

        };

    }
}

#endif

