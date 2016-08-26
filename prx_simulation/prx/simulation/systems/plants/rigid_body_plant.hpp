/**
 * @file rigid_body_plant.hpp
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

#ifndef PRX_RIGID_BODY_PLANT_HPP
#define	PRX_RIGID_BODY_PLANT_HPP

#include "prx/simulation/state.hpp"
#include "prx/simulation/control.hpp"
#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/plants/plant.hpp"
#include "prx/simulation/systems/plants/kinematic_plant.hpp"

namespace prx
{
    namespace sim
    {

        /**
         * Represents a rigid body system. Implementation of the integration functions
         * for simulating a rigid body system.\n
         * State: [x, y, theta] \n
         * Control: [x, y, theta].
         *
         * @brief <b> Represents a rigid body system. </b>
         *
         * @author Andrew Dobson
         *
         */
        class rigid_body_plant_t : public kinematic_plant_t
        {

          public:
            rigid_body_plant_t();

            virtual ~rigid_body_plant_t(){ }

            /** @copydoc plant_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*) */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /** @copoydoc plant_t::update_phys_configs(util::config_list_t&, unsigned&) const */
            virtual void update_phys_configs(util::config_list_t& configs, unsigned& index) const;

            virtual void update_collision_info();

            /** @copydoc plant_t::append_contingency(plan_t&, double)*/
            void append_contingency(plan_t& result_plan, double duration);

          private:

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
             * Internal control memory for the \c Cx coordinate.
             * @brief Internal control memory for the \c Cx coordinate.
             */
            double _Cx;

            /**
             * Internal control memory for the \c Cy coordinate.
             * @brief Internal control memory for the \c Cy coordinate.
             */
            double _Cy;

            /**
             * Internal control memory for the \c Ctheta coordinate.
             * @brief Internal control memory for the \c Ctheta coordinate.
             */
            double _Ctheta;

            /**
             * Height value of the rigid body.
             * @brief Height value of the rigid body.
             */
            double _z;
        };

    }
}

#endif
