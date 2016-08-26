/**
 * @file 3d_plant.hpp 
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

#ifndef PRX_3D_PLANT_HPP
#define	PRX_3D_PLANT_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/plants/integration_plant.hpp"

namespace prx
{
    namespace packages
    {
        namespace conformant
        {
            class threed_plant_t : public sim::integration_plant_t
            {

              public:

                threed_plant_t();

                virtual ~threed_plant_t();

                /** @copydoc integration_plant_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*) */
                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

                /** @copydoc integration_plant_t::propagate(const double) */
                void propagate(const double simulation_step);

                /** @copydoc integration_plant_t::update_phys_configs(util::config_list_t&,, unsigned& index) const */
                virtual void update_phys_configs(util::config_list_t& configs, unsigned& index) const;

                virtual void update_collision_info();

                /** @copydoc integration_plant_t::steering_function(const state_t*, const state_t*, plan_t&) */
                void steering_function(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& result_plan);
              protected:

                /** @copoydoc plant_t::update_derivative (state_t* const) */
                virtual void update_derivative(sim::state_t * const result);


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
                 * Indexer for state variable : Y.
                 * @brief Indexer for state variable : Y.
                 */
                const static unsigned STATE_Z;
                const static unsigned STATE_ROLL;
                const static unsigned STATE_PITCH;
                const static unsigned STATE_YAW;

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
                 * Internal state for the variable \c y, y coordinate.
                 * @brief Internal state for the variable \c y, y coordinate.
                 */
                double _z;
                double _roll;
                double _pitch;
                double _yaw;

                /**
                 * Internal control for the variable \c v, velocity.
                 * @brief Internal control for the variable \c v, velocity.
                 */
                double _vx;

                /**
                 * Internal control for the variable \c theta, angle.
                 * @brief Internal control for the variable \c theta, angle.
                 */
                double _vy;

                /**
                 * Height value of the disk.
                 * @brief Height value of the disk.
                 */
                double _vz;
                double _vroll;
                double _vpitch;
                double _vyaw;


            };
        }

    }
}

#endif

