/**
 * @file unigripper.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2016, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_FFEE_HPP
#define	PRX_FFEE_HPP

#include "prx/utilities/definitions/defs.hpp"

#include "simulation/systems/plants/manipulator.hpp"

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
            /**
             * 
             */
            class ff_end_effector_t : public manipulator_t
            {

              public:
                ff_end_effector_t();

                virtual ~ff_end_effector_t();

                /** @copydoc plant_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*) */
                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);                

                /** @copoydoc plant_t::update_phys_configs(util::config_list_t&, unsigned&) const */
                virtual void update_phys_configs(util::config_list_t& configs, unsigned& index) const;

                virtual void update_collision_info();
                
                virtual void FK_solver( util::config_t& link_config, std::string link_name);

                virtual bool IK_solver( util::space_point_t* result_state, const util::space_point_t* start_state, const util::config_t& goal_config, std::string start_link, std::string end_link);

                virtual bool jac_steering( sim::plan_t& result_plan, workspace_trajectory_t& ee_trajectory, util::space_point_t* result_state, const util::space_point_t* start_state, const util::config_t& goal_config, std::string start_link, std::string end_link);

                virtual void steering_function(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& result_plan);

              protected:

                virtual void compute_ee_trajectory(workspace_trajectory_t& workspace_trajectory, const sim::state_t* start_state, const sim::plan_t& plan);

                virtual void state_to_config( util::config_t& result_config, const sim::state_t* state);

                virtual void create_spaces();

                KDL::Frame static_transform;

                sim::state_t* temp_state1, *temp_state2;


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

                double _roll, _pitch, _yaw;

                double _qx, _qy, _qz, _qw;

                // TRUE: Then we use the roll-pitch-yaw representation (typically for full manipulation planning)
                // FALSE: Then we use the quaternion representation (typically for f.f.e.e collision checks)
                bool plannable_manipulator;

            };
        }

    }
}

#endif
