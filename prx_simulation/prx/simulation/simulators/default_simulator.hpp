/**
 * @file default_simulator.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#pragma once 

#ifndef PRX_DEFAULT_SIMULATOR_HPP
#define	PRX_DEFAULT_SIMULATOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/simulation/simulators/simulator.hpp"

namespace prx
{
    namespace sim
    {

        /**
         * This is the default simulator which allows for collision
         * detection and response to be turned on/off.
         * 
         * @brief <b> A default implementation of the simulator </b>
         * 
         * @author Andrew Kimmel
         * 
         */
        class default_simulator_t : public simulator_t
        {

          public:
            default_simulator_t();
            virtual ~default_simulator_t();

            /** @copydoc simulator_t::init( const util::parameter_reader_t* const) */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /** @copydoc simulator_t::propagate_and_respond() */
            virtual void propagate_and_respond();
            
            /**
             * Overwritten to check if collision_detection is false
             */
            bool in_collision();

            /**
             * Overwritten to check if collision_detection is false
             */
            void link_collision_list(collision_list_t* collision_list);

            /**
             * Overwritten to check if collision_detection is false
             */
            collision_list_t* get_colliding_bodies();
            
            /** Sets collision detection variable */
            void set_collision_detection(bool detects);
            /** Sets collision response variable */
            void set_collision_response(bool responds);
            
            /** Returns collision detection variable */
            bool simulator_collision_detects();
            /** Returns collision response variable */
            bool simulator_collision_responds();

          protected:
              
            /** Determines if collision detection is done */
            bool collision_detection;

            /** Determines if collision response is done */
            bool collision_response;

            /** A hash map that keeps the collided systems from the last step.*/
            util::hash_t<std::string, bool> collided_systems;

            /** The previous state for all the systems in order to return the collided systems.*/
            state_t* prev_state;

            /** The current state of the simulation.*/
            state_t* state;

            /**
             * Sets the system under the location that the \c path specified, to the previous state, 
             * right before the collision.
             * 
             * @param path The slash-delimited path indicating the location of the system in the simulation tree.
             */
            void set_previous_state(const std::string& path);

          private:

        };

    }
}

#endif

