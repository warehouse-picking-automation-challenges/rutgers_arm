/**
 * @file euclidean_point_plant.hpp
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

#ifndef PRX_EUCLIDEAN_POINT_PLANT_HPP
#define	PRX_EUCLIDEAN_POINT_PLANT_HPP

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
         * Represents a dot system in euclidean space.\n
         * State: R^n \n
         * Control: R^n.
         *
         * @brief <b> Represents a euclidean point system. </b>
         *
         * @author Andrew Dobson
         *
         */
        class euclidean_point_plant_t : public kinematic_plant_t
        {

          public:
            euclidean_point_plant_t();

            virtual ~euclidean_point_plant_t(){ }

            /** @copydoc plant_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*) */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /** @copoydoc plant_t::update_phys_configs(util::config_list_t&) const */
            virtual void update_phys_configs(util::config_list_t& configs, unsigned& index) const;

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
             * The dimensionality of the euclidean space this point is in.
             */
            unsigned dimension;
        };

    }
}

#endif
