/**
 * @file planar_manipulator.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#pragma once
#ifndef PRX_PLANAR_MANIPULATOR_HPP
#define PRX_PLANAR_MANIPULATOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/plants/integration_plant.hpp"
#include "prx/simulation/plan.hpp"


namespace prx
{
    namespace sim
    {
        /**
         * Represents a 3-link arm mounted on a mobile base. There is currently 
         * no end effector; this plant was designed for testing motion planners 
         * rather than solving problems that involve grasping.
         * State space: [X, Y, first link angle, second link angle, third link angle]
         * Control space: [X velocity, Y velocity, first link rotational velocity, second link rotational velocity, third link rotational velocity]
         *
         * @brief <b>Represents a 3-link arm mounted on a mobile base.</b>
         * 
         * @author Justin Cardoza
         */
        class planar_manipulator_t : public sim::integration_plant_t
        {
            public:
                planar_manipulator_t();
                virtual ~planar_manipulator_t();
                
                /** @copydoc integration_plant_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*) */
                virtual void init(const util::parameter_reader_t *reader, 
                    const util::parameter_reader_t *template_reader = NULL);
                
                /** @copoydoc integration_plant_t::update_phys_configs(util::config_list_t&, unsigned&) const */
                virtual void update_phys_configs(util::config_list_t& configs, unsigned& index) const;
                
                /** @copoydoc system_t::steering_function(const state_t* start, const state_t* goal, plan_t& result_plan) */
                virtual void steering_function(const sim::state_t *start, const sim::state_t *goal, sim::plan_t& plan);
                
            protected:
                /** @copoydoc plant_t::update_derivative (state_t* const) */
                virtual void update_derivative(sim::state_t * const result);
                
                //State: x and y position.
                double s_x, s_y;
                
                //State: link angles.
                double s_theta[3];
                
                //Controls: x and y position.
                double c_vx, c_vy;
                
                //Controls: rotational velocities for each link.
                double c_dtheta[3];
                
                //Parameter: z (height)
                double p_z;
                
                //The lengths of the links, determined at runtime.
                double lengths[3];
                
                //Indices for control and state parameters.
                const static unsigned STATE_X, STATE_Y;
                const static unsigned STATE_THETA1, STATE_THETA2, STATE_THETA3;
        };
    }
}

#endif