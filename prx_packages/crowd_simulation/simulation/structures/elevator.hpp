/**
 * @file elevator.hpp
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

#ifndef PRX_ELEVATOR_HPP
#define PRX_ELEVATOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/3d_geometry/geometry.hpp"
#include "prx/utilities/math/configurations/bounds.hpp"
#include "prx/utilities/math/configurations/config.hpp"
 
#include "simulation/controllers/VO_structure/neighbor.hpp"

#include <vector>

namespace prx
{
    namespace packages
    {
        namespace crowd
        {
            class nav_node_t;

            class elevator_t
            {
              public:
                elevator_t();
                ~elevator_t();

                void frame();

                bool in_elevator( double in_x, double in_y, double in_z );
                bool approximately_in_elevator( double in_x, double in_y, double in_z );
                bool is_open();
                double current_height();
                const nav_node_t* current_node();
                
                bool matches_bounds( double min_x, double max_x, double min_y, double max_y );
                void set_bounds( double min_x, double max_x, double min_y, double max_y );
                void add_stop( const std::pair< double, const nav_node_t* >& stop );
                void compute_shaft_geometry();

                bool has_stop_at_height( double test_height );
                double distance( double in_x, double in_y );

                void print() const;

                util::config_t configuration;
                util::geometry_t* shaft_geometry;
                std::string name;

                std::vector< util::bounds_t > bounds;
                std::vector< util::bounds_t > approximate_bounds;
                std::vector< std::pair< double, const nav_node_t* > > stops;
                
                neighbor_t neighbor;
                
              protected:
                bool moving;
                bool going_up;
                
                unsigned max_transition_time;
                unsigned frames_in_transition;
                unsigned at_stop;
                double velocity;
                double height;
            };
        }
    }
}

#endif //PRX_RAMP_HPP

