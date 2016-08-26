/**
 * @file PABT_sensing_model.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_PABT_SENSING_MODEL_HPP
#define PRX_PABT_SENSING_MODEL_HPP

#include "prx/simulation/sensing/sensing_model.hpp"

namespace prx
{

    namespace packages
    {
        namespace crowd
        {
            class navigation_graph_sensor_t;

            /**
             * The VO sensing model is responsible for generating all the information necessary
             * for a VO_sensing_info class to create VOs from.  It reasons over vo-plants, other plants,
             * and obstacles in the simulation.  Using a config_sensor, velocity_sensor,
             * and geometry_sensor, this sensing model populates the neighborhood
             * information for each VO_sensing_info.
             *
             * @brief The sensing model used in VO-related applications
             *
             * @author Andrew Kimmel
             */

             class PABT_sensing_model_t : public sim::sensing_model_t
             {
              protected:

              public:
                PABT_sensing_model_t();
                virtual ~PABT_sensing_model_t();

                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);
                virtual void initialize_sensors(sim::simulator_t* sim);

                virtual void update_sensed_state( const sim::state_t* new_state);

                virtual void sense();

                /** Sensor Pointers */
                navigation_graph_sensor_t* graph_sensor;
            };
        }
    }
}

#endif


