/**
 * @file double_integrator.hpp
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

#ifndef PRX_DOUBLE_INTEGRATOR_HPP
#define	PRX_DOUBLE_INTEGRATOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/plants/integration_plant.hpp"

namespace prx
{
    namespace sim
    {

        /**
         * Represents a double integrator system. Implementation of the integration functions 
         * for simulating a double integrator system. \n
         * State: [x, velocity] \n
         * Control: [acceleration].
         *  
         * @brief <b> Represents a double integrator system. </b>
         * 
         * @author Zakary Littlefield
         */
        class double_integrator_t : public integration_plant_t
        {

          public:
            double_integrator_t();
            virtual ~double_integrator_t();

            /** @copydoc integration_plant_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*)*/
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /** @copydoc integration_plant_t::propagate(const double) */
            void propagate(const double simulation_step);

            /** @copydoc integration_plant_t::update_phys_configs(util::config_list_t&, unsigned&) */
            virtual void update_phys_configs(util::config_list_t& configs, unsigned& index) const;

          protected:

            /** @copydoc integration_plant_t::update_derivative (state_t* const) */
            virtual void update_derivative(state_t * const result);

            /** 
             * Indexer for state variable : X 
             * @brief Indexer for state variable : X 
             */
            const static unsigned STATE_X;

            /** 
             * Indexer for state variable : V 
             * @brief Indexer for state variable : V 
             */
            const static unsigned STATE_V;

            /** 
             * Indexer for control variable : A 
             * @brief Indexer for control variable : A 
             */
            const static unsigned CONTROL_A;

            /** 
             * Internal state memory for state variable x (position of the system)
             * @brief Internal state memory for state variable x (position of the system)
             */
            double _x;

            /** 
             * Internal state memory for state variable v (velocity of the system)
             * @brief Internal state memory for state variable v (velocity of the system)
             */
            double _vx;

            /** 
             * Internal control memory for state variable a (acceleration of the system)
             * @brief Internal state memory for state variable a (acceleration of the system)
             */
            double _a;

            /** 
             * Height value of the block.
             * @brief Height value of the block.
             */
            double _z;

        };

    }
}

#endif

