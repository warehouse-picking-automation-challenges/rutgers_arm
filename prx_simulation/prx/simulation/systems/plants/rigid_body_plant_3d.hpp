/**
 * @file rigid_body_plant_3d.hpp
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

#ifndef PRX_RIGID_BODY_PLANT_3D_HPP
#define	PRX_RIGID_BODY_PLANT_3D_HPP

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
         * Represents a 3D rigid body system. Implementation of the integration functions
         * for simulating a 3D rigid body system.\n
         * State: [x, y, z, qx, qy, qz, qw] \n
         * Control: [x, y, z, qx, qy, qz, qw].
         *
         * @brief <b> Represents a 3D rigid body system. </b>
         *
         * @author Andrew Dobson
         *
         */
        class rigid_body_plant_3d_t : public kinematic_plant_t
        {

          public:
            rigid_body_plant_3d_t();

            virtual ~rigid_body_plant_3d_t(){ }

            /** @copydoc plant_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*) */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /** @copoydoc plant_t::update_phys_configs(util::config_list_t&, unsigned& ) const */
            void update_phys_configs(util::config_list_t& configs, unsigned& index) const;

            /** @copydoc system_t::append_contingency(plan_t&,double)*/
            void append_contingency(plan_t& result_plan, double duration);

          protected:

            /**
             * Internal state memory for the \c x position coordinate.
             * @brief Internal state memory for the \c x position coordinate.
             */
            double _x;

            /**
             * Internal state memory for the \c y position coordinate.
             * @brief Internal state memory for the \c y position coordinate.
             */
            double _y;

            /**
             * Internal state memory for the \c z position coordinate.
             * @brief Internal state memory for the \c z position coordinate.
             */
            double _z;

            /**
             * Internal state memory for the \c qx coordinate of the orientation quaternion.
             * @brief Internal state memory for the \c qx coordinate of the orientation quaternion.
             */
            double _qx;

            /**
             * Internal state memory for the \c qy coordinate of the orientation quaternion.
             * @brief Internal state memory for the \c qy coordinate of the orientation quaternion.
             */
            double _qy;

            /**
             * Internal state memory for the \c qz coordinate of the orientation quaternion.
             * @brief Internal state memory for the \c qz  coordinate of the orientation quaternion.
             */
            double _qz;

            /**
             * Internal state memory for the \c qw coordinate of the orientation quaternion.
             * @brief Internal state memory for the \c qw coordinate of the orientation quaternion.
             */
            double _qw;

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
             * Internal control memory for the \c Cz coordinate.
             * @brief Internal control memory for the \c Cz coordinate.
             */
            double _Cz;

            /**
             * Internal control memory for the \c qx orientation.
             * @brief Internal control memory for the \c qx orientation.
             */
            double _Cqx;

            /**
             * Internal control memory for the \c qy orientation.
             * @brief Internal control memory for the \c qy orientation.
             */
            double _Cqy;

            /**
             * Internal control memory for the \c qz orientation.
             * @brief Internal control memory for the \c qz orientation.
             */
            double _Cqz;

            /**
             * Internal control memory for the \c qw orientation.
             * @brief Internal control memory for the \c qw orientation.
             */
            double _Cqw;

          private:

        };

    }
}

#endif

