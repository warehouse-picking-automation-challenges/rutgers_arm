/**
 * @file sliding_rigid_body.hpp 
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

#ifndef PRX_SLIDING_RIGID_BODY_HPP
#define	PRX_SLIDING_RIGID_BODY_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/plants/integration_plant.hpp"

namespace prx
{
    namespace sim
    {

        /**
         * Represents a sliding rigid body system that cannot rotate. Can only move vertical or horizontal. 
         * Implementation of the integration functions for simulating a sliding rigid body system.\n
         * State: [x, y] \n
         * Control: [velocity_x, velocity_y].
         * 
         * @brief <b> Represents a sliding rigid body system. </b>
         * 
         * @author Andrew Kimmel 
         * 
         */
        class sliding_rigid_body_t : public plant_t
        {

          public:
            sliding_rigid_body_t();

            virtual ~sliding_rigid_body_t(){ }

            /** @copydoc plant_ts::init(const util::parameter_reader_t *, const util::parameter_reader_t*) */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /** @copydoc system_t::propagate(const double) */
            virtual void propagate(const double simulation_step = 0);

            /** @copoydoc plant_t::update_phys_configs(util::config_list_t&) const */
            void update_phys_configs(util::config_list_t& configs, unsigned& index) const;

            /** @copydoc plant_t::steering_function(const state_t*, const state_t*, plan_t&)*/
            void steering_function(const state_t* start, const state_t* goal, plan_t& result_plan);

            /** @copydoc plant_t::append_contingency(plan_t& , double)*/
            void append_contingency(plan_t& result_plan, double duration);

            virtual void update_collision_info();

          private:
            /** 
             * Internal state memory for x coordinate.
             * @brief Internal state memory for x coordinate.
             */
            double _x;

            /** 
             * Internal state memory for y coordinate.
             * @brief Internal state memory for y coordinate.
             */
            double _y;

            /** 
             * Internal control memory for x coordinate.
             * @brief Internal control memory for x coordinate.
             */
            double _Cx;

            /** 
             * Internal control memory for y coordinate.
             * @brief Internal control memory for y coordinate.
             */
            double _Cy;

            /** 
             * Height value of the rigid_body.
             * @brief Height value of the rigid_body.
             */
            double _z;

            /**
             * The maximum distance for the plant in a simulation step.
             * @brief The maximum distance for the plant in a simulation step.
             */
            double max_step;

            /**
             * The step for a specific distance. 
             * @brief The step for a specific distance. 
             */
            double interpolation_step;

            /**
             * The total cover before change control.
             * @brief The total cover before change control.
             */
            double dist;

            /**
             * A flag for the next step.
             * @brief A flag for the next step.
             */
            bool reset;

            /**
             * To copy the initial state before the interpolation.
             * @brief To copy the initial state before the interpolation.
             */
            state_t* initial_state;

            /**
             * To copy the plant's state into to do interpolation and copying and such.
             * @brief To copy the plant's state into to do interpolation and copying and such.
             */
            state_t* state;

            /**
             * Used to check if the rigid body is actually making progress.
             * @brief Used to check if the rigid body is actually making progress.
             */
            state_t* hold_state;

            /**
             * Holds the state from the last propagation.
             * @brief Holds the state from the last propagation.
             */
            state_t* prior_state;

            /**
             * The previously used control by the system.
             * @brief The previously used control by the system.
             */
            control_t* prior_control;

            /**
             * To copy the plant's control into in order to do equivalence checking.
             * @brief To copy the plant's control into in order to do equivalence checking.
             */
            control_t* control;

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

        };

    }
}

#endif	// PRX_DISK_HPP

