/**
 * @file double_integrator_2d.hpp
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

#ifndef PRX_DOUBLE_INTEGRATOR_2D_HPP
#define	PRX_DOUBLE_INTEGRATOR_2D_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/plants/integration_plant.hpp"

namespace prx
{
    namespace sim
    {

        /**
         * Represents a double integrator system. Implementation of the integration functions 
         * for simulating a double integrator system. \n
         * State: [x, xdot, y, ydot] \n
         * Control: [acceleration_x, acceleration_y].
         * 
         * @brief <b> Represents a double integrator system. </b>
         * 
         * @author Zakary Littlefield
         * 
         */
        class double_integrator_2d_t : public integration_plant_t
        {

          public:
            double_integrator_2d_t();
            virtual ~double_integrator_2d_t();

            /** @copydoc integration_plant_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*) */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /** @copydoc integration_plant_t::propagate(const double) */
            void propagate(const double simulation_step);

            /** @copoydoc integration_plant_t::update_phys_configs(util::config_list_t&, unsigned&) const */
            virtual void update_phys_configs(util::config_list_t& configs, unsigned& index) const;

            /** @copydoc integration_plant_t::steering_function(const state_t*, const state_t*, plan_t&) */
            void steering_function(const state_t* start, const state_t* goal, plan_t& result_plan);

          protected:

            /** @copoydoc integration_plant_t::update_derivative (state_t* const) */
            virtual void update_derivative(state_t * const result);

            /**
             * Indexer for state variable : X.
             * @brief Indexer for state variable : X.
             */
            const static unsigned STATE_X;

            /**
             * Indexer for state variable : X_DOT.
             * @brief Indexer for state variable : X_DOT. 
             */
            const static unsigned STATE_X_DOT;

            /**
             * Indexer for state variable : Y.
             * @brief Indexer for state variable : Y.
             */
            const static unsigned STATE_Y;

            /**
             * Indexer for state variable : Y_DOT.
             * @brief Indexer for state variable : Y_DOT.
             */
            const static unsigned STATE_Y_DOT;

            /**
             * Indexer for state variable : A_X.
             * @brief Indexer for state variable : A_X.
             */
            const static unsigned CONTROL_A_X;

            /**
             * Indexer for state variable : A_Y.
             * @brief Indexer for state variable : A_Y.
             */
            const static unsigned CONTROL_A_Y;

            /**
             * Internal state for the variable \c x, x coordinate.
             * @brief Internal state for the variable \c x, x coordinate.
             */
            double _x;

            /**
             * Internal state for the variable \c xdot.
             * @brief Internal state for the variable \c xdot.
             */
            double _xdot;

            /**
             * Internal state for the variable \c y, y coordinate.
             * @brief Internal state for the variable \c y, y coordinate.
             */
            double _y;

            /**
             * Internal state for the variable \c ydot.
             * @brief Internal state for the variable \c ydot.
             */
            double _ydot;

            /**
             * Internal state for the variable \c ax.
             * @brief Internal state for the variable \c ax.
             */
            double _ax;

            /**
             * Internal state for the variable \c ay.
             * @brief Internal state for the variable \c ay.
             */
            double _ay;

            /**
             * Height value of the block.
             * @brief Height value of the block.
             */
            double _z;

        };

    }
}

#endif
