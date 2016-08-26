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

#include "simulation/structures/ramp.hpp"
#include "prx/simulation/systems/system.hpp"

#include "prx/utilities/definitions/defs.hpp"

namespace prx
{
    using namespace util;

    namespace packages
    {
        namespace crowd
        {

            ramp_t::ramp_t()
            {
                test_point.resize(2);
                base_point.resize(2);

                metric_points.resize(0);
                is_escalator = false;
            }

            ramp_t::~ramp_t()
            {

            }

            void ramp_t::set_slope( double in_slope )
            {
                slope = in_slope;
            }

            void ramp_t::set_base( double in_base )
            {
                base = in_base;
            }

            void ramp_t::set_length( double in_length )
            {
                length = in_length;
            }

            void ramp_t::set_points( const std::vector< double >& bottom_left, const std::vector< double >& bottom_right, const std::vector< double >& top_right, const std::vector< double >& top_left )
            {
                points.clear();

                points.push_back( bottom_left );
                points.push_back( bottom_right );
                points.push_back( top_right );
                points.push_back( top_left );

                //We can also compute the metric points
                metric_points.resize( 2 );

                metric_points[0].push_back( (bottom_left[0] + bottom_right[0])/2.0 );
                metric_points[0].push_back( (bottom_left[1] + bottom_right[1])/2.0 );
                metric_points[0].push_back( bottom_left[2] );
                metric_points[1].push_back( (top_left[0] + top_right[0])/2.0 );
                metric_points[1].push_back( (top_left[1] + top_right[1])/2.0 );
                metric_points[1].push_back( top_left[2] );
            }

            void ramp_t::set_flow( util::vector_t direction, bool upflow )
            {
                is_escalator = true;
                
                direction *= (0.5 * sim::simulation::simulation_step);
                if( !upflow )
                {
                    direction *= -1.0;
                }
                
                flow.resize( 2 );
                flow[0] = direction[0];
                flow[1] = direction[1];
            }

            const std::vector< std::vector< double > >& ramp_t::get_metric_points() const
            {
                return metric_points;
            }

            void ramp_t::offset_effective_area( const vector_t& offset )
            {
                //The effective area will be represented with a set of lines
                area_bounds.resize( 4 );

                for( unsigned i=0; i<4; ++i )
                {
                    area_bounds[i].set_points( vector_t( points[i][0] + offset[0], points[i][1] + offset[1] ), vector_t( points[(i+1)%4][0] + offset[0], points[(i+1)%4][1] + offset[1] ) );
                }
            }
            
            bool ramp_t::near_side_wall( const double& pos_x, const double& pos_y )
            {
                //Have the information in a point
                test_point[0] = pos_x;
                test_point[1] = pos_y;
                
                return ( fabs( area_bounds[1].side( test_point ) ) < 0.01 || fabs( area_bounds[3].side( test_point ) ) < 0.01 ) && area_bounds[0].side( test_point ) > 0 && area_bounds[2].side( test_point ) > 0;
            }

            bool ramp_t::on_ramp( const double& pos_x, const double& pos_y, double eps )
            {
                test_point[0] = pos_x;
                test_point[1] = pos_y;

                for( unsigned i=0; i<4; ++i )
                {
                    double val = (i%2 == 0) ? eps : 0;
                    if( area_bounds[i].side( test_point ) < val )
                    {
                        return false;
                    }
                }
                return true;
            }

            double ramp_t::height_on_ramp( const double& pos_x, const double& pos_y )
            {
                test_point[0] = pos_x;
                test_point[1] = pos_y;

                test_point = area_bounds[1].project( test_point );
                area_bounds[1].get_point_a( base_point );

                double distance = sqrt( (test_point[0] - base_point[0])*(test_point[0] - base_point[0]) + (test_point[1] - base_point[1])*(test_point[1] - base_point[1]) );

                return slope*distance + base + 0.82 + 0.36*slope;
            }
            
            double ramp_t::get_base()
            {
                return base;
            }

            void ramp_t::print_points()
            {
                PRX_PRINT( "POINTS: " << points.size(), PRX_TEXT_GREEN );
                PRX_PRINT( ": " << points[0][0] << " , " << points[0][1], PRX_TEXT_LIGHTGRAY );
                PRX_PRINT( ": " << points[1][0] << " , " << points[1][1], PRX_TEXT_LIGHTGRAY );
                PRX_PRINT( ": " << points[2][0] << " , " << points[2][1], PRX_TEXT_LIGHTGRAY );
                PRX_PRINT( ": " << points[3][0] << " , " << points[3][1], PRX_TEXT_LIGHTGRAY );

                PRX_PRINT( "METRIC POINTS: " << metric_points.size() , PRX_TEXT_BROWN );
                PRX_PRINT( ": " << metric_points[0][0] << " , " << metric_points[0][1] << " , " << metric_points[0][2], PRX_TEXT_LIGHTGRAY );
                PRX_PRINT( ": " << metric_points[1][0] << " , " << metric_points[1][1] << " , " << metric_points[1][2], PRX_TEXT_LIGHTGRAY );
            }
        }
    }
}


