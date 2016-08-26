/**
 * @file minimal_avoidance_controller.hpp 
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#pragma once

#ifndef PRX_MINIMAL_AVOIDANCE_CONTROLLER_HPP
#define PRX_MINIMAL_AVOIDANCE_CONTROLLER_HPP

#include "prx/utilities/definitions/defs.hpp"
//#include "prx/simulation/systems/system.hpp"
#include "prx/simulation/systems/controllers/simple_controller.hpp"
#include "prx/simulation/plan.hpp"
//#include "prx/utilities/boost/hash.hpp"
//#include "prx/simulation/control.hpp"
//#include "prx/utilities/parameters/parameter_reader.hpp"
//#include "prx/utilities/spaces/space.hpp"




namespace prx
{
    namespace sim
    {
        
        class twoD_proximity_sensing_info_t;

        /**
         * @brief <b>A controller which avoids collisions by stopping. </b>
         * 
         * @author Andrew Kimmel
         */
        class minimal_avoidance_controller_t : public simple_controller_t
        {

          public:
            /**
             * Initializes internal variables and sets up the controller's state space with the time parameter.
             * 
             * @brief Initializes internal variables and sets up the controller's state space with the time parameter.
             */
            minimal_avoidance_controller_t();

            /** @copydoc system_t::~system_t() */
            virtual ~minimal_avoidance_controller_t();

            /**
             * 
             */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /**
             * Calls \c compute_control to its single subsystem.
             * 
             * @brief \c Calls compute_control to its single subsystem.
             */
            virtual void compute_control();

          protected:
              twoD_proximity_sensing_info_t* prox_info;
              
              /** @brief Contingency plan in case consumer receives no plan (or reach end of plan) */
            plan_t contingency_plan;
            
            /** Distance thresholds before contingency plan is executed */
            double obstacle_distance;
            double plant_distance;

//
//            //Child system information.
//            system_ptr_t child_system;
//           
//            const util::space_t *child_state_space;
//            const util::space_t *child_control_space;
//            
//            // Current state of child
//            state_t* current_state;
//            control_t* current_control;
        };
    }
}


#endif