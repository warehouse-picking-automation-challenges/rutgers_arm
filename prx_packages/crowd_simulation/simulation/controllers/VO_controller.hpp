/**
 * @file VO_controller.hpp
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

#ifndef PRX_VO_CONTROLLER_HPP
#define PRX_VO_CONTROLLER_HPP

#include "prx/utilities/math/2d_geometry/arc.hpp"
#include "prx/utilities/math/2d_geometry/line.hpp"
#include "prx/utilities/math/3d_geometry/geometry_info.hpp"

#include "prx/simulation/control.hpp"
#include "prx/simulation/state.hpp"
#include "prx/simulation/systems/controllers/simple_controller.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"

#include "simulation/controllers/VO_structure/VO.hpp"
#include "simulation/controllers/VO_structure/neighbor.hpp"
#include "simulation/sensing/VO_sensing_info.hpp"

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
             * @anchor VO_controller_t
             *
             * @author Andrew Dobson, Andrew Kimmel
             */
            class VO_controller_t : virtual public sim::simple_controller_t
            {
              protected:
                
                /////// TESTING NEW STUFFS ///////
                std::vector< util::bounds_t* > sampling_bounds;
                util::space_point_t* sample_point;
                
                /** @brief */
                bool vanishing;
                bool once;

                /** Visualization of sampled vectors */
                std::vector<double> sampled_x;
                std::vector<double> sampled_y;
                std::vector<bool> sampled_validity;

                /** @brief False: Obstacles are traditionally constructed. True: Obstacles are Chuplishly constructed.*/
                bool minimally_invasive;

                /** @brief False: Use RVO as normal. True: If closest neighboring plant < distance threshold, stop moving.*/
                bool non_vo_avoidance;
                double distance_threshold;
                double closest_neighbor;

                /** @brief */
                util::line_t lline;
                /** @brief */
                util::line_t rline;

                VO_sensing_info_t* vo_sensing_info;
                /** @brief */
                std::vector< neighbor_t* > neighborhood;

                /** @brief */
                std::vector< util::arc_t > candidate_subarcs;

                /** @brief */
                bool select_current;
                /** @brief */
                util::vector_t computed_vector_control;
                /** @brief */
                int collisions;
                /** @brief */
                unsigned nr_hrvos;
                /** @brief */
                unsigned max_hrvos;

                /** @brief */
                util::vector_t goal_state;
                /** @brief */
                bool goal_set;
                /** @brief */
                util::vector_t desired_control;
                /** @brief */
                double radius;
                /** @brief */
                double vo_radius;
                /** @brief */
                double max_vel;
                /** @brief */
                double min_vel;
                /** @brief */
                double sq_neighbor_range;
                /** @brief */
                util::vector_t curr_vel;
                /** @brief */
                util::vector_t last_pos;
                /** @brief */
                bool finished;
                /** @brief */
                bool vanished;
                /** @brief */
                std::string team_color;

                //Horizon Informations
                /** @brief */
                double safety_horizon;

                //Livelock resolution
                /** @brief */
                int resolve_counter;
                /** @brief */
                int progress_counter;
                /** @brief */
                double dist_to_goal;

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

                // The offset indices we use for our state
                /** @brief */
                unsigned int _X;
                /** @brief */
                unsigned int _Y;

                /** @brief */
                bool visualize_VOs;

                /** @brief */
                std::vector<VO_t> VOs;

                /** @brief */
                std::vector< util::line_t > lines;
                /** @brief */
                mutable util::vector_t point_a;
                /** @brief */
                mutable util::vector_t point_b;
                /** @brief */
                mutable util::vector_t point_c;
                /** @brief */
                mutable util::vector_t point_d;
                /** @brief */
                util::vector_t zero_center;
                /** @brief */
                std::vector< util::vector_t > minkowski_vertices, vertices_2D;
                /** @brief */
                util::hash_t<std::string, minkowski_body_t>* minkowski_obstacles;
                /** @brief */
                std::vector<double> angles;
                /** @brief */
                util::line_t obst_line;
                util::vector_t obst_b, obst_cent;

                /** @brief */
                mutable sim::state_t* new_control;
                /** @brief */
                mutable sim::state_t* new_state;

                std::vector< std::string > debug_colors;

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

                enum system_finish_type_t
                {
                    PRX_VO_FT_STAY,
                    PRX_VO_FT_VANISH
                };

                system_finish_type_t finish_type;

        	    enum compute_sample_control_mode
                {
                    BASIC,
                    MIXED_CONTROL,
                    BLENDED_HISTORY
                };

                compute_sample_control_mode csc_mode;

                util::vector_t prev_control_1;
                util::vector_t prev_control_2;
                util::vector_t prev_control_3;

                double prev_vel;

                bool backtracking;
                int b_step_count;

                bool in_backtracking;
                int b_back_count;
                util::vector_t b_last_center;

                int step_count_limit;
                int back_count_limit;

                unsigned num_samples;

                void* parent_controller;

                /**
                 *
                 */
                virtual void compute_desired_control();

                /**
                 *
                 */
                virtual void compute_best_sample_control(unsigned sample_size);

                /**
                 *
                 */
                virtual bool velocity_is_valid( const util::vector_t& vec, double eps = 0 );

                /**
                 *
                 */
                virtual int point_in_vos( util::vector_t* point, std::vector<bool>& vec, int index, double eps = 0.0 );

                /**
                 *
                 */
                virtual void update_vis_info(bool visualize_VO) const;

              public:
                // Simple controller functions
                VO_controller_t();
                virtual ~VO_controller_t();

                void pre_init( void* controller );

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

                /**
                 *
                 */
                virtual void compute_VOs();

                /**
                 *
                 */
                void set_obstacles(util::hash_t<std::string, minkowski_body_t>& inobst);

                /**
                 *
                 */
                virtual void update_velocity();

                /**
                 *
                 */
                virtual const util::vector_t& get_center() const;

                /**
                 *
                 */
                virtual const util::vector_t& get_velocity() const;

                /**
                 *
                 */
                double get_radius();

                /**
                 *
                 */
                bool is_finished() const;
                /**
                 *
                 */
                bool has_vanished() const;
                /**
                 *
                 */
                int num_collisions() const;

                /**
                 *
                 */
                std::vector< std::vector< std::vector< util::intersection_point_t > > >& get_points();
                /**
                 *
                 */
                std::vector< util::line_t >& get_segments();
                /**
                 *
                 */
                std::vector< util::intersection_point_t* > get_sorted_points();

                /** @brief */
                bool planning;
                /** @brief */
                bool inactive;
                /** @brief */
                bool selected;

                bool computed_vos;
                bool near_goal;
                double goal_distance;
                mutable bool was_vis;

                bool fully_reciprocal;
                bool added_to_metric;

                void toggle_vanish();

                void update_max_velocity(double velocity);

                static double cc_time;
                static unsigned cc_calls;
            };
        }
    }
}

#endif



