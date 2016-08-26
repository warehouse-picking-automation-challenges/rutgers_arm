/**
 * @file ramp.hpp
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

#ifndef PRX_RAMP_HPP
#define PRX_RAMP_HPP

#include "prx/utilities/math/2d_geometry/line.hpp"
#include "prx/utilities/math/configurations/vector.hpp"

#include <vector>

namespace prx
{
    namespace packages
    {
        namespace crowd
        {

            class ramp_t
            {
              public:
                ramp_t();
                ~ramp_t();

                void set_slope( double in_slope );
                void set_base( double in_base );
                void set_length( double in_length );
                void set_points( const std::vector< double >& bottom_left, const std::vector< double >& bottom_right, const std::vector< double >& top_right, const std::vector< double >& top_left );
                void set_flow( util::vector_t direction, bool upflow );
                const std::vector< std::vector< double > >& get_metric_points() const;
                void offset_effective_area( const util::vector_t& offset );
                bool near_side_wall( const double& pos_x, const double& pos_y );
                bool on_ramp( const double& pos_x, const double& pos_y, double eps = 0.0 );
                double height_on_ramp( const double& pos_x, const double& pos_y );
                double get_base();

                void print_points();

                bool is_escalator;
                bool going_up;
                std::vector< double > flow;

              protected:
                double slope;
                double base;
                double length;

                util::vector_t test_point;
                util::vector_t base_point;
                std::vector< std::vector< double > > points;
                std::vector< std::vector< double > > metric_points;
                std::vector< util::line_t > area_bounds;
            };
        }
    }
}

#endif //PRX_RAMP_HPP

