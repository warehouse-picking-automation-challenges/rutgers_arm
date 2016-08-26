/**
 * @file VO.cpp
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

#include "simulation/controllers/VO_structure/VO.hpp"

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {
        namespace crowd
        {

            /** @brief */
            const unsigned int VO_t::_X = 0;
            /** @brief */
            const unsigned int VO_t::_Y = 1;

            VO_t::VO_t()
            {
                left_seg.invalidate();
                right_seg.invalidate();
                left_seg2.invalidate();
                right_seg2.invalidate();
                zero_vec.resize(2);
                zero_vec.zero();
                proj.resize( 2 );
                center.resize( 2 );
                intersection_points.resize( 4 );
                lines.resize(100);
                for( unsigned int i=0; i<4; ++i )
                    intersection_points[i].resize( 2 );
                check_points.resize( 6 );
                for( unsigned int i=0; i< check_points.size(); ++i )
                    check_points[i].resize(2);
                printing = false;
                horizon = PRX_INFINITY;
                vo_source = PRX_VO_SYSTEM;
            }

            VO_t::VO_t( const vector_t& l, const vector_t& r, const vector_t& v )
            {
                set_points( l, r, v );
                left_seg.invalidate();
                right_seg.invalidate();
                left_seg2.invalidate();
                right_seg2.invalidate();
                zero_vec.resize(2);
                zero_vec.zero();
                proj.resize( 2 );
                center.resize( 2 );
                intersection_points.resize( 4 );
                for( unsigned int i=0; i<4; ++i )
                    intersection_points[i].resize( 2 );
                check_points.resize( 6 );
                for( unsigned int i=0; i< check_points.size(); ++i )
                    check_points[i].resize(2);
                printing = false;
                horizon = PRX_INFINITY;
                vo_source = PRX_VO_SYSTEM;
            }

            VO_t::VO_t (const double h_radius, const vector_t& h_center, const vector_t& h_direction, const arc_t& h_arc, const double _horizon)
            {

                horizon_radius = h_radius;
                horizon_center = h_center;
                horizon_direction = h_direction;
                horizon_arc = h_arc;
                horizon = _horizon;
                vo_source = PRX_VO_HORIZON_SYSTEM;

            }

            VO_t::~VO_t()
            {}

            void VO_t::set_points( const vector_t& l, const vector_t& r, const vector_t& v )
            {
                left.set_points( v, l );
                right.set_points( v, r );
            }

            void VO_t::set_l( const vector_t& l )
            {
                left.set_point_b( l );
            }

            void VO_t::set_r( const vector_t& r )
            {
                right.set_point_b( r );
            }

            void VO_t::set_v( const vector_t& v )
            {
                left.set_point_a( v );
                right.set_point_a( v );
            }

            void VO_t::set_velocity(double v)
            {
                velocity = v;
            }

            void VO_t::get_points( vector_t& l, vector_t& r, vector_t& v )
            {
                left.get_point_b( l );
                right.get_point_b( r );
                left.get_point_a( v );
            }

            void VO_t::get_points( vector_t& l, vector_t& r, vector_t& v ) const
            {
                left.get_point_b( l );
                right.get_point_b( r );
                left.get_point_a( v );
            }

            const vector_t& VO_t::get_l( )
            {
                return left.get_point_b()->get_data();
            }

            const vector_t& VO_t::get_r( )
            {
                return right.get_point_b()->get_data();
            }

            const vector_t& VO_t::get_v( )
            {
                return right.get_point_a()->get_data();
            }

            const line_t& VO_t::get_left() const
            {
                return left;
            }

            const line_t& VO_t::get_right() const
            {
                return right;
            }

            const line_t& VO_t::get_left_seg() const
            {
                return left_seg;
            }

            line_t& VO_t::get_left_seg()
            {
                return left_seg;
            }

            const line_t& VO_t::get_right_seg() const
            {
                return right_seg;
            }

            line_t& VO_t::get_right_seg()
            {
                return right_seg;
            }

            const line_t& VO_t::get_left_seg2() const
            {
                return left_seg2;
            }

            line_t& VO_t::get_left_seg2()
            {
                return left_seg2;
            }

            const line_t& VO_t::get_right_seg2() const
            {
                return right_seg2;
            }

            line_t& VO_t::get_right_seg2()
            {
                return right_seg2;
            }

            double VO_t::get_horizon() const
            {
                return horizon;
            }

            const arc_t& VO_t::get_arc_seg() const
            {
                return arc_seg;
            }

            arc_t& VO_t::get_arc_seg()
            {
                return arc_seg;
            }

            const arc_t& VO_t::get_arc_seg2() const
            {
                return arc_seg2;
            }

            arc_t& VO_t::get_arc_seg2()
            {
                return arc_seg2;
            }

            double VO_t::get_horizon_radius() const
            {
                return horizon_radius;
            }

            const vector_t& VO_t::get_horizon_center() const
            {
                return horizon_center;
            }

            const vector_t& VO_t::get_horizon_direction() const
            {
                return horizon_direction;
            }

            const arc_t& VO_t::get_horizon_arc() const
            {
                return horizon_arc;
            }

            arc_t& VO_t::get_horizon_arc()
            {
                return horizon_arc;
            }

            bool VO_t::in_obstacle( const vector_t& point, double eps ) const
            {
                // PRX_DEBUG_COLOR("Point: " << point << " and eps: " << eps, PRX_TEXT_MAGENTA);
                switch(vo_source)
                {
                    case PRX_VO_OBSTACLE: case PRX_VO_SYSTEM:
                        //            if( norm_angle_zero( left.get_lines_angle() - right.get_lines_angle() ) < PRX_PI )
                        return ( left.side(point) < eps && right.side(point) > -eps );
                        //            else
                        //                return ( left.side(point) < eps || right.side(point) > -eps );
                    case PRX_VO_LOCO: case PRX_VO_HORIZON_LOCO:
                        if (horizon_arc.side(point) > 0)
                            return false;
                        else
                            return true;
                    case PRX_VO_HORIZON_SYSTEM:
                    {
                        line.set_points(left.get_point_a()->get_data(), right.get_point_a()->get_data());
                        return ( (left.side(point) < eps && right.side(point) > -eps && line.side(point) > -eps )|| horizon_arc.side(point) > eps );
                    }
                    case PRX_VO_HORIZON_OBSTACLE:
                    {
                        vector_t left_norm = left.get_point_b()->get_data(); vector_t right_norm = right.get_point_b()->get_data();
                        left_norm.normalize(); right_norm.normalize();
                        line.set_points(left.get_point_a()->get_data() + left_norm*velocity*horizon*0.9,
                                        right.get_point_a()->get_data() + right_norm*velocity*horizon*0.9);
                        {
                            return ( left.side(point) < eps && right.side(point) > -eps && line.side(point) > -eps  );
                        }
                    }
                    default:
                        PRX_FATAL_S("VO source type has not been defined");
                        break;
                }

            }

            void VO_t::expand( const vector_t& vela, const VO_t& other )
            {
                line.bisect( left, right );
                if( line.side( vela ) > 0 ) //If velocity A is on the left side of the center
                {
                    right = other.right;
                }
                else //If velocity A is on the right side of the center
                {
                    left = other.left;
                }
                left.intersection( &right, check_points );
                ipoint = check_points[0];
                lower = left.get_normalized_direction();
                upper = right.get_normalized_direction();
                left.set_point_a( ipoint );
                right.set_point_a( ipoint );

                lower += ipoint.get_data();
                upper += ipoint.get_data();
                left.set_point_b( lower );
                right.set_point_b( upper );
            }

            void VO_t::set_tangent_points( const vector_t& pcenter, double radius )
            {
                double sq_dist = pcenter[_X]*pcenter[_X] + pcenter[_Y]*pcenter[_Y];
                double sq_rad = radius*radius;
                double epsilon = (1- (sq_rad/sq_dist));
                double alpha = ( radius/sq_dist ) * sqrt( sq_dist - sq_rad );

                left.set_point_a( zero_vec );
                right.set_point_a( zero_vec );

                center[_X] = epsilon*pcenter[_X] - alpha*pcenter[_Y];
                center[_Y] = epsilon*pcenter[_Y] + alpha*pcenter[_X];
                left.set_point_b( center );

                center[_X] = epsilon*pcenter[_X] + alpha*pcenter[_Y];
                center[_Y] = epsilon*pcenter[_Y] - alpha*pcenter[_X];
                right.set_point_b( center );
            }

            void VO_t::set_horizon_points( const vector_t& pcenter, double radius )
            {
                double sq_dist = pcenter[_X]*pcenter[_X] + pcenter[_Y]*pcenter[_Y];
                double sq_rad = radius*radius;
                double epsilon = (1- (sq_rad/sq_dist));
                double alpha = ( radius/sq_dist ) * sqrt( sq_dist - sq_rad );

                left.set_point_a( zero_vec );
                right.set_point_a( zero_vec );

                center[_X] = epsilon*pcenter[_X] - alpha*pcenter[_Y];
                center[_Y] = epsilon*pcenter[_Y] + alpha*pcenter[_X];
                left.set_point_b( center );
                left.translate(center);

                center[_X] = epsilon*pcenter[_X] + alpha*pcenter[_Y];
                center[_Y] = epsilon*pcenter[_Y] - alpha*pcenter[_X];
                right.set_point_b( center );
                right.translate(center);
            }

            void VO_t::adapt_center( vector_t& vec, double radius )
            {
                double magn = vec.squared_norm();
                if( magn < (radius*radius + 0.1) )
                {
                    vec[_X] = ( vec[_X] * (radius+PRX_DISTANCE_CHECK) )/sqrt( magn );
                    vec[_Y] = ( vec[_Y] * (radius+PRX_DISTANCE_CHECK) )/sqrt( magn );
                }
            }

            void VO_t::construct( const vector_t& other_center, double radius, const vector_t& velb )
            {
                proj = other_center;
                adapt_center( proj, radius );
                set_tangent_points( proj, radius );
                left.translate( velb );
                right.translate( velb );
            }

            void VO_t::LOCO_construct(const arc_t& LOCO_arc)
            {
                //    PRX_ERROR_S ("Inside VO LOCO construct, print parameter arc");
                //    LOCO_arc.print();
                horizon_arc = LOCO_arc;
                vo_source = PRX_VO_LOCO;
                left.invalidate();
                right.invalidate();
            }

            void VO_t::rvo_construct( const vector_t& other_center, double radius, const vector_t& vela, const vector_t& velb, double ratio )
            {
                proj = other_center;

                if( printing )
                {
                    PRX_DEBUG_COLOR("===== PRINTING! ======\n", PRX_TEXT_BROWN);
                    PRX_DEBUG_COLOR("Center before : " << proj, PRX_TEXT_BROWN);
                    PRX_DEBUG_COLOR("This is D: " << proj.norm() << " for Radius " << radius, PRX_TEXT_BROWN );
                }
                adapt_center( proj, radius );
                if( printing )
                {
                    PRX_DEBUG_COLOR("Center After : " << proj, PRX_TEXT_BROWN);
                }
                set_tangent_points( proj, radius );

                proj = velb;
                proj *= ratio;
                proj += vela*(1-ratio);
                left.translate( proj );
                right.translate( proj );
            }

            void VO_t::hrvo_construct( const vector_t& other_center, double radius, const vector_t& vela, const vector_t& velb, double ratio )
            {
                rvo_construct( other_center, radius, vela, velb, ratio );

                VO_t vo;
                vo.construct( other_center, radius, velb );

                vector_t vertex;
                left.get_point_a( vertex );
                expand( vela, vo );
            }

            void VO_t::corridor_construct( const vector_t& other_center, double radius, double direction )
            {
                proj[0] = cos(direction);
                proj[1] = sin(direction);

                vector_t& point = lower;
                point[0] = other_center[0] + proj[0];
                point[1] = other_center[1] + proj[1];

                line_t line( other_center, point );

                if( line.side( zero_vec ) > 0 ) //We're to their left
                {
                    lower = proj;
                    upper = -proj;
                }
                else //We're to their right
                {
                    lower = -proj;
                    upper = proj;
                }
                set_points( lower, upper, zero_vec );
            }

            void VO_t::obstacle_construct( const minkowski_body_t* obst, const vector_t& my_center, double new_horizon, double max_vel, bool minimally_invasive)
            {
                double x_offset, y_offset;
                x_offset = obst->x_offset;
                y_offset = obst->y_offset;

                // if (minimally_invasive)
                // {
                unsigned v_index = 0;
                double closest = PRX_INFINITY;
                double dist = 0;
                
                const std::vector< vector_t >& o_verts = obst->vertices;
                unsigned vert_size = o_verts.size();
                
                //Find the closest minkowski point
                for( unsigned i=0; i<o_verts.size(); ++i )
                {
                    dist = my_center.squared_distance( o_verts[i] );
                    if( dist < closest )
                    {
                        closest = dist;
                        v_index = i;
                    }
                }
                
                //Point v is this closest point, and the left and right follow in order
                unsigned l_index = (v_index-1);
                //If we had an underflow, it must be the last thing in the thing
                if( l_index > vert_size )
                {
                    l_index = vert_size -1;
                }
                
                pt_r = o_verts[ (v_index+1)%vert_size ];
                pt_l = o_verts[l_index];
                pt_v = o_verts[v_index];
                
                pt_r -= my_center;
                pt_l -= my_center;
                pt_v -= my_center;
                
                //We will see if we have to adjust one of the sides
                proj = pt_v;
                proj += pt_v;
                    
                //Check (& correct) if the right side is violated
                bool right_violated = ((pt_r[0]-pt_v[0])*(proj[1]-pt_v[1]))-((pt_r[1]-pt_v[1])*(proj[0]-pt_v[0])) < 0;
                bool left_violated = ((proj[0]-pt_v[0])*(pt_l[1]-pt_v[1]))-((proj[1]-pt_v[1])*(pt_l[0]-pt_v[0])) < 0;
                if( right_violated && !left_violated )
                {
                    pt_r = pt_v;
                    pt_r += pt_v;
                }
                //Check (& correct) if the left side is violated
                else if( left_violated && !right_violated )
                {
                    pt_l = pt_v;
                    pt_l += pt_v;
                }
                
                set_points( pt_l, pt_r, pt_v );

                return;
                
                
                
                // }
                // else
                // {
                //     // DEBUG - TESTING OLD CONSTRUCTION METHOD
                //     for (unsigned int counter = 0; counter <  obst->vertices.size(); counter++)
                //     {
                //         lines[counter].set_points(zero_vec, obst->vertices[counter] - my_center);
                //     }


                //     int max_index = -1, min_index =-1;
                //     double max_angle = -PRX_INFINITY, min_angle = PRX_INFINITY, diff_angle;

                //     center_mass.set_points(my_center, vector_t(x_offset,y_offset));

                //     double test_angle = center_mass.get_lines_angle();

                //     for (unsigned int angle_index = 0; angle_index < obst->vertices.size(); angle_index++)
                //     {
                //         diff_angle = minimum_angle_with_orientation(test_angle, lines[angle_index].get_lines_angle());
                //         if (max_angle < diff_angle)
                //         {
                //             max_angle = diff_angle;
                //             max_index = angle_index;
                //         }
                //         if (min_angle > diff_angle)
                //         {
                //             min_angle = diff_angle;
                //             min_index = angle_index;
                //         }
                //     }

                //     if (new_horizon < PRX_INFINITY)
                //     {
                //         set_points((obst->vertices[max_index] - my_center),(obst->vertices[min_index] - my_center),zero_vec);
                //         horizon = new_horizon;
                //         set_source( PRX_VO_HORIZON_OBSTACLE );
                //     }
                //     else
                //     {
                //         set_points((obst->vertices[max_index] - my_center),(obst->vertices[min_index] - my_center),zero_vec);
                //         set_source( PRX_VO_OBSTACLE );
                //     }
                // }
            }

            std::vector< std::vector< intersection_point_t > >& VO_t::intersect( VO_t* other )
            {
                left.intersection( &(other->left), intersection_points[0] );
                left.intersection( &(other->right), intersection_points[1] );
                right.intersection( &(other->left), intersection_points[2] );
                right.intersection( &(other->right), intersection_points[3] );

                return intersection_points;
            }

            void VO_t::horizon_VO_construct(const vector_t& relative_position, const vector_t& my_center, const double tau, const double radius, const vector_t& velb)
            {
                // we define relative position as: pos_b - pos_a
                proj = relative_position;
                adapt_center( proj, radius );
                horizon_center = (proj) / tau;
                horizon_radius = radius / tau;
                horizon_direction = (proj);
                horizon = tau;
                vel_b = velb;

                // Set tangent points
                set_horizon_points( horizon_center, horizon_radius );
                // Translate lines by the velocity
                left.translate( velb );
                right.translate( velb );
                // Translate horizon center
                horizon_center += velb;
                // Set arc points
                horizon_arc.set_points( right.get_point_a()->get_data(), left.get_point_a()->get_data(), horizon_center);
                horizon_arc.calc_rad();
                horizon_arc.validate();
            }

            void VO_t::horizon_RVO_construct(const vector_t& relative_position, const vector_t& my_center, const double tau, const double radius, const vector_t& vela, const vector_t& velb, const double ratio, bool print)
            {
                proj = relative_position;
                adapt_center( proj, radius );
                // we define relative position as: pos_b - pos_a
                horizon_center = (proj) / tau;
                horizon_radius = (radius / tau);
                horizon_direction = (proj);
                horizon = tau;
                vel_b = velb;

                if (print)
                {
                    PRX_DEBUG_S ("Inside horizon RVO construct before calculations");
                    PRX_DEBUG_S ("Horizon of " <<  tau);
                    PRX_DEBUG_S (" Relative position: " << relative_position << " and Horizon center " << horizon_center);
                    //        relative_position.print();
                    //        horizon_center.print();
                    PRX_DEBUG_S("Vel a : " << vela << " and Vel b:  " << velb);
                    //        vela.print();
                    //        velb.print();
                }


                // Set tangent points
                set_horizon_points( horizon_center, horizon_radius );
                if (print)
                {
                    PRX_DEBUG_COLOR ("Lines after set horizon points LEFT: " << left << ", RIGHT: " << right, PRX_TEXT_RED);

                }
                // Translate lines by the velocity
                proj = velb;
                proj *= ratio;
                proj += vela*(1-ratio);
                left.translate( proj );
                right.translate( proj );
                // Translate horizon center
                horizon_center += proj;
                // Set arc points
                horizon_arc.set_points( right.get_point_a()->get_data(), left.get_point_a()->get_data(), horizon_center);
                horizon_arc.calc_rad();
                horizon_arc.validate();
                if (print)
                {
                    PRX_DEBUG_S ("Constructed horizon arc of: " << horizon_arc);
                }
            }

            void VO_t::construct_segments( double r )
            {
                center[_X] = 0;
                center[_Y] = 0;

                double l, dsq;
                bool lower_on = false, upper_on = false;

                //= Invalidate all the segments =//
                left_seg.set_valid( false );
                left_seg2.set_valid( false );
                right_seg.set_valid( false );
                right_seg2.set_valid( false );
                arc_seg.set_valid( false );
                arc_seg2.set_valid( false );

                //= Left Segment =//
                line = left;

                proj = line.project( center );
                dsq = proj.squared_norm();
                l = sqrt( r*r - dsq );
                if( dsq >= r*r )
                    left_seg.invalidate();
                else
                {
                    lower = proj;
                    upper = proj;
                    proj = line.get_normalized_direction();
                    proj *= -l;
                    lower += proj;
                    proj = line.get_normalized_direction();
                    proj *= l;
                    upper += proj;

                    lower_on = line.on_ray( lower );
                    upper_on = line.on_ray( upper );

                    if (!upper_on && !lower_on)
                    {
                        left_seg.invalidate();
                    }
                    else if( upper_on && lower_on )
                    {
                        left_seg.set_point_a( lower );
                        left_seg.set_point_b( upper );
                        left_seg.validate();
                    }
                    else
                    {
                        if( lower_on )
                            line.get_point_a( upper );
                        else
                            line.get_point_a( lower );
                        left_seg.set_points( lower, upper );
                        left_seg.validate();
                    }
                }

                //= Right Segment =//
                line = right;

                proj = line.project( center );
                dsq = proj.squared_norm();
                l = sqrt( r*r - dsq );
                if( dsq >= r*r )
                    right_seg.invalidate();
                else
                {
                    lower = proj;
                    upper = proj;
                    proj = line.get_normalized_direction();
                    proj *= -l;
                    lower += proj;
                    proj = line.get_normalized_direction();
                    proj *= l;
                    upper += proj;

                    lower_on = line.on_ray( lower );
                    upper_on = line.on_ray( upper );

                    if ( !upper_on && !lower_on )
                    {
                        right_seg.invalidate();
                    }
                    else if( upper_on && lower_on )
                    {
                        right_seg.set_point_a( lower );
                        right_seg.set_point_b( upper );
                        right_seg.validate();
                    }
                    else
                    {
                        if( lower_on )
                            line.get_point_a( upper );
                        else
                            line.get_point_a( lower );
                        right_seg.set_points( lower, upper );
                        right_seg.validate();
                    }
                }
            }

            void VO_t::approximate_horizon_VO_construct(const vector_t& a1,const vector_t& a2,const vector_t& b1,const vector_t& b2)
            {
                // PRX_LOG_DEBUG ("Approximate horizon VO constructed");
                left.set_points(a1, a2);
                right.set_points(b1,b2);
                left.translate(vector_t(0.0,0.0));
                right.translate(vector_t(0.0,0.0));

            }

            void VO_t::construct_segments( double outer, double inner, line_t& lbnd, line_t& rbnd )
            {
                //= Invalidate all the segments =//
                left_seg.set_valid( false );
                left_seg2.set_valid( false );
                right_seg.set_valid( false );
                right_seg2.set_valid( false );
                arc_seg.set_valid( false );
                arc_seg2.set_valid( false );

                // For LOCOs
                if (vo_source == 2 || vo_source == 5)
                {
                    PRX_DEBUG_S ("Inside construct segments for types 2 and 3");
                    left_seg.invalidate(); left_seg2.invalidate();
                    right_seg.invalidate(); right_seg2.invalidate();
                    horizon_arc.construct_segment( outer, inner, lbnd, rbnd, arc_seg, arc_seg2 );
                }
                //First, check the left leg
                left.construct_segment( outer, inner, lbnd, rbnd, left_seg, left_seg2 );
                //Then, check the right leg
                right.construct_segment( outer, inner, lbnd, rbnd, right_seg, right_seg2 );
                //Lastly, check the center arc
                if( vo_source >= 4 )
                {
                    horizon_arc.construct_segment( outer, inner, lbnd, rbnd, arc_seg, arc_seg2 );
                }
                else
                {
                    arc_seg.set_valid( false );
                    arc_seg2.set_valid( false );
                }
            }

            void VO_t::closest_tangent( const vector_t& cpoint, vector_t& dir, vector_t& u )
            {
                //First, construct super large segments to do testing
                left.get_point_a( lower );
                right.get_point_a( upper );
                left_seg.set_point_a( lower );
                right_seg.set_point_a( upper );
                lower += left.get_normalized_direction()*1000;
                upper += right.get_normalized_direction()*1000;
                left_seg.set_point_b( lower );
                right_seg.set_point_b( upper );
                //Find the closest point on these segments to the vel to check
                double dl = left_seg.seg_get_closest_point( cpoint, lower );
                double dr = right_seg.seg_get_closest_point( cpoint, upper );

                //Direction depends on which segment we are closer to
                //u is simply the difference from cpoint to this point
                if( dl < dr )
                {
                    dir = -left.get_normalized_direction();
                    u = lower;
                    u -= cpoint;
                }
                else
                {
                    dir = right.get_normalized_direction();
                    u = -upper;
                    u -= cpoint;
                }
                u /= 2;
            }

            void VO_t::set_printing( bool toprint )
            {
                printing = toprint;
            }

            void VO_t::set_source(int type)
            {
                switch (type)
                {
                    case 0:
                        vo_source = PRX_VO_OBSTACLE;
                        break;
                    case 1:
                        vo_source = PRX_VO_SYSTEM;
                        break;
                    case 2:
                        vo_source = PRX_VO_LOCO;
                        break;
                    case 3:
                        vo_source = PRX_VO_HORIZON_OBSTACLE;
                        break;
                    case 4:
                        vo_source = PRX_VO_HORIZON_SYSTEM;
                        break;
                    case 5:
                        vo_source = PRX_VO_HORIZON_LOCO;
                        break;
                    default:
                        break;
                }
            }

            int VO_t::get_source_type() const
            {
                return vo_source;
            }

            bool VO_t::is_printing( )
            {
                return printing;
            }

            std::string VO_t::print() const
            {
                std::stringstream out(std::stringstream::out);

                out << "Left line: " << left << "\nRight Line: " << right << "\n";
                out << "Source [" << vo_source << "] : Segments: " << (left_seg.is_valid()?"#":"x") << " " << (left_seg2.is_valid()?"#":"x")
                        << " " << (right_seg.is_valid()?"#":"x") << " " << (right_seg2.is_valid()?"#":"x") << " " <<
                        (arc_seg.is_valid()?"#":"x") << " " << (arc_seg2.is_valid()?"#":"x");

                if( left_seg.is_valid() )
                    out << (left_seg);
                if( right_seg.is_valid() )
                    out << (right_seg);
                if( left_seg2.is_valid() )
                    out << (left_seg2);
                if( right_seg2.is_valid() )
                    out << (right_seg2);
                if( arc_seg.is_valid() )
                    out << (arc_seg);
                if( arc_seg2.is_valid() )
                    out << (arc_seg2);

                return out.str();
            }
            
            bool VO_t::check_integrity() const
            {
                if( (*(left.get_point_a()))[0] != (*(left.get_point_a()))[0] ||
                    (*(left.get_point_a()))[1] != (*(left.get_point_a()))[1] ||
                    (*(left.get_point_b()))[0] != (*(left.get_point_b()))[0] ||
                    (*(left.get_point_b()))[1] != (*(left.get_point_b()))[1] ||
                    (*(right.get_point_a()))[0] != (*(right.get_point_a()))[0] ||
                    (*(right.get_point_a()))[1] != (*(right.get_point_a()))[1] ||
                    (*(right.get_point_b()))[0] != (*(right.get_point_b()))[0] ||
                    (*(right.get_point_b()))[1] != (*(right.get_point_b()))[1] )
                {
                    return false;
                } 
                
                return true;
            }

        }
    }
}
