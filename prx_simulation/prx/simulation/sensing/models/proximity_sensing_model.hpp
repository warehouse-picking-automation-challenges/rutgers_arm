/**
 * @file proximity_sensing_model.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Kimmel, Andrew Dobson, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_PROXIMITY_SENSING_MODEL_HPP
#define	PRX_PROXIMITY_SENSING_MODEL_HPP

#include "prx/simulation/sensing/sensing_model.hpp"

namespace prx
{
    namespace sim
    {
        
        class config_sensor_t;
        class geometry_sensor_t;
        
        /**
         * @author Andrew Kimmel
         */
        class proximity_sensing_model_t : public sensing_model_t 
        {
        protected:
                    
            /** Sensor Pointers */
            sim::config_sensor_t* conf_sensor;
            sim::geometry_sensor_t* geom_sensor;

        public:

            proximity_sensing_model_t();
            virtual ~proximity_sensing_model_t();

            /**
             * Initializes from the given parameters.
             * 
             * @brief Initializes from the given parameters.
             * 
             * @param reader A \ref util::parameter_reader_t with a dictionary of parameters for this system.
             * @param template_reader A template \ref util::parameter_reader_t with a dictionary of parameters.
             * Default value for the template reader is NULL. The priority goes to primary reader \c reader, if something
             * is not specified in the \c reader then it will be read from the \c template_reader 
             */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /**
             *
             */
            virtual void update_sensed_state( const sim::state_t* new_state);
            /**
             *
             */
            virtual void sense();


        };
    }
}

#endif

