/**
 * @file disk.hpp 
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

#ifndef PRX_DISK_HPP
#define	PRX_DISK_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/plants/integration_plant.hpp"

namespace prx
{
    namespace sim
    {

        /**
         * Represents a disk system. Implementation of the integration functions 
         * for simulating a 2D disk system.\n
         * State: [x, y] \n
         * Control: [velocity, theta].
         * 
         * @brief <b> Represents a disk system. </b>
         * 
         * @author Andrew Kimmel
         * 
         */
        class disk_t : public integration_plant_t
        {

          public:

            disk_t();

            virtual ~disk_t();

            /** @copydoc integration_plant_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*) */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /** @copydoc integration_plant_t::propagate(const double) */
            void propagate(const double simulation_step);

            /** @copydoc integration_plant_t::update_phys_configs(util::config_list_t&,, unsigned& index) const */
            virtual void update_phys_configs(util::config_list_t& configs, unsigned& index) const;

            /** @copydoc integration_plant_t::steering_function(const state_t*, const state_t*, plan_t&) */
            void steering_function(const state_t* start, const state_t* goal, plan_t& result_plan);

            virtual void update_collision_info();

          protected:

            /** @copoydoc plant_t::update_derivative (state_t* const) */
            virtual void update_derivative(state_t * const result);

            virtual void set_param(const std::string& parameter_name, const boost::any& value);


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
             * Indexer for control variable : V.
             * @brief Indexer for control variable : V.
             */
            const static unsigned CONTROL_V;

            /**
             * Indexer for control variable : THETA.
             * @brief Indexer for control variable : THETA.
             */
            const static unsigned CONTROL_THETA;

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
             * Internal control for the variable \c v, velocity.
             * @brief Internal control for the variable \c v, velocity.
             */
            double _v;

            /**
             * Internal control for the variable \c theta, angle.
             * @brief Internal control for the variable \c theta, angle.
             */
            double _theta;

            /**
             * Height value of the disk.
             * @brief Height value of the disk.
             */
            double _z;

            /**
             * The maximum length of edges returned by the steering function. 
             * By default, this value is not set so as to return the full plan
             * to the desired goal state. However, in certain cases, it is better
             * to not completely connect towards the goal state.
             * 
             * @brief The maximum length of edges returned by the steering function.
             */
            int max_steps;

            /**
             * Keeps the previous state of the disk.
             * @brief Keeps the previous state of the disk.
             */
            state_t* previous_state;

        };

    }
}

#endif

