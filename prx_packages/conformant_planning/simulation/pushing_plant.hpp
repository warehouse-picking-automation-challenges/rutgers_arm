/**
 * @file pushing_plant.hpp 
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

#ifndef PRX_PUSHING_PLANT_HPP
#define	PRX_PUSHING_PLANT_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/plants/integration_plant.hpp"

namespace prx
{
    namespace packages
    {
        namespace conformant
        {
            class pushing_plant_t : public sim::integration_plant_t
            {

              public:

                pushing_plant_t();

                virtual ~pushing_plant_t();

                /** @copydoc integration_plant_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*) */
                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

                /** @copydoc integration_plant_t::propagate(const double) */
                void propagate(const double simulation_step);

                /** @copydoc integration_plant_t::update_phys_configs(util::config_list_t&,, unsigned& index) const */
                virtual void update_phys_configs(util::config_list_t& configs, unsigned& index) const;

                virtual void link_collision_info(sim::collision_checker_t* collision_checker);
                virtual void update_collision_info();

              protected:

                /** @copoydoc plant_t::update_derivative (state_t* const) */
                virtual void update_derivative(sim::state_t * const result);

                double _x;
                double _y;
                double _yaw;
                double _r;
                double _angle_offset;

                double _vx;
                double _vy;


                double _z;

            };
        }

    }
}

#endif

