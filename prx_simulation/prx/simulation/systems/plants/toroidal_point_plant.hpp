/**
 * @file toroidal_point_plant.hpp 
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

#ifndef PRX_TOROIDAL_POINT_PLANT_HPP
#define	PRX_TOROIDAL_POINT_PLANT_HPP

#include "prx/simulation/state.hpp"
#include "prx/simulation/control.hpp"
#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/plants/plant.hpp"

namespace prx
{
    namespace sim
    {

        /**
         * Represents a dot system in toroidal space.\n
         * State: R^n \n
         * Control: R^n.
         * 
         * @brief <b> Represents a toroidal point system. </b>
         * 
         * @author Andrew Dobson 
         * 
         */
        class toroidal_point_plant_t : public plant_t
        {

          public:
            toroidal_point_plant_t();

            virtual ~toroidal_point_plant_t(){ }

            /** @copydoc plant_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*) */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /** @copydoc plant_t::propagate(const double) */
            void propagate(const double simulation_step);

            /** @copoydoc plant_t::update_phys_configs(util::config_list_t&) const */
            virtual void update_phys_configs(util::config_list_t& configs, unsigned& index) const;

            /** @copydoc plant_t::steering_function(const state_t*, const state_t*, plan_t&)*/
            void steering_function(const state_t* start, const state_t* goal, plan_t& result_plan);

          private:

            /**
             * The actual memory for the point's state, X.
             */
            std::vector< double > X;

            /**
             * The actual memory for the point's control, U.
             */
            std::vector< double > U;

            /**
             * The dimensionality of the toroidal space this point is in.
             */
            unsigned dimension;

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
        };

    }
}

#endif
