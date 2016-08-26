/**
 * @file collision_avoidance_controller.hpp
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

#ifndef PRX_COLLISION_AVOIDANCE_CONTROLLER_HPP
#define PRX_COLLISION_AVOIDANCE_CONTROLLER_HPP

#include "prx/utilities/math/2d_geometry/arc.hpp"
#include "prx/utilities/math/2d_geometry/line.hpp"
#include "prx/utilities/math/3d_geometry/geometry_info.hpp"

#include "prx/simulation/control.hpp"
#include "prx/simulation/state.hpp"
#include "prx/simulation/systems/controllers/simple_controller.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"

#include "simulation/controllers/VO_structure/VO.hpp"
#include "simulation/controllers/VO_structure/neighbor.hpp"
#include "simulation/sensing/neighbor_sensing_info.hpp"

#include <algorithm>
#include <boost/range/adaptor/map.hpp> //adaptors


namespace prx
{
    namespace sim
    {
    }

    namespace packages
    {
        namespace crowd
        {
            /**
             * @anchor collision_avoidance_controller_t
             *
             * @author Andrew Dobson, Andrew Kimmel
             */
            class collision_avoidance_controller_t : virtual public sim::simple_controller_t
            {              
              public:
                // Simple controller functions
                collision_avoidance_controller_t();
                virtual ~collision_avoidance_controller_t();

                /**
                 *
                 */
                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

                /**
                 *
                 */
                virtual void compute_control();

                /**
                 * @copydoc system_t::propagate()
                 *
                 * This implementation simply calls the subsystem's propagate.
                 *
                 */
                virtual void propagate(const double simulation_step = 0);

                void set_id( unsigned input_id );
                void set_passive( bool input_flag );
                void set_conservative_turns( bool flag );

                // true if the agent is in queue, false otherwise
                void set_in_queue(bool flag, unsigned agent_id);

                void set_obstacles( util::hash_t<std::string, minkowski_body_t>& inobst );
                void update_max_velocity( double velocity );

                double get_current_velocity() const;

                //double get_current_orientation() const;
                
                void visualize_VOs() const;

                static double cc_time;
                static unsigned cc_calls;

                // To tell whether the agent is in queue
                bool in_queue;

                unsigned agent_id;

                // Set the orientation of agents inside the queue
                void set_queue_orientation(double queue_orientation);

              protected:
                /** @brief False: Obstacles are traditionally constructed. True: Obstacles are Chuplishly constructed.*/
                bool minimally_invasive;

                neighbor_sensing_info_t* neighbor_info;
                /** @brief */
                util::space_point_t* current_control;
                util::space_point_t* best_control;
                util::vector_t best_vector_control;
                util::vector_t temp_vector;
                /** @brief */
                unsigned nr_hrvos;
                /** @brief */
                unsigned max_hrvos;

                /** @brief */
                util::vector_t goal_state;
                /** @brief */
                util::vector_t desired_control;
                /** @brief */
                util::vector_t scratch_vector;
                
                /** @brief */
                double radius;
                /** @brief */
                double vo_radius;
                /** @brief */
                double max_vel;
                /** @brief */
                double sq_max_vel;
                /** @brief */
                double min_vel;

                /** @brief */
                double _Cx;
                /** @brief */
                double _Cy;
                /** @brief */
                double _Cz;

                // The hard indices we expect for our state
                /** @brief */
                static const unsigned int X;
                /** @brief */
                static const unsigned int Y;

                // The hard indices we expect for our control
                /** @brief */
                static const unsigned int V;
                /** @brief */
                static const unsigned int THETA;

                /** @brief */
                std::vector<VO_t> VOs;

                /** @brief */
                std::vector< util::vector_t > minkowski_vertices, vertices_2D;
                /** @brief */
                util::hash_t<std::string, minkowski_body_t>* minkowski_obstacles;

                /** @brief */
                bool passive;
                /** @brief */
                double safety;
                double desired_velocity;

                /* Force the orientation of the agent in the queue */
                double queue_orientation;

                bool conservative_turns;
                /**
                 * This variable is in order to cheat a little bit. If this variable is true the agent 
                 * will try to  avoid the other agents as normal VO. If there are a lot of agents around 
                 * and it is difficult for the agent to move then this variable is false and the agent will 
                 * just follow its desire velocity to its target. 
                 */                
                bool stay_safe;

                /**
                 * @anchor compare_intersection_point_t
                 */
                struct compare_intersection_point_t
                {
                    bool operator ()(util::intersection_point_t* a, util::intersection_point_t* b)
                    {
                        return *a < *b;
                    }
                };

                virtual void compute_desired_control();
                virtual bool velocity_is_valid( const util::vector_t& vec, double eps = 0, int excluded = -1 );
                virtual bool velocity_is_valid_obstacle_check( const util::vector_t& vec, double eps = 0, int excluded = -1 );
                virtual void compute_VOs();
                virtual void compute_best_control();
                void check_control_for_safety( bool desired_safe );

                void to_polar( util::space_point_t* polar, const util::vector_t& vector );
                void to_euclidean( util::vector_t& vector, util::space_point_t* polar );
            };
        }
    }
}

#endif



