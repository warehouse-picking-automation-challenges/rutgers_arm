/**
 * @file line.cpp 
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

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/2d_geometry/arc.hpp"
#include "prx/utilities/math/2d_geometry/line.hpp"
#include "prx/utilities/math/2d_geometry/angle.hpp"

namespace prx 
{ 
    namespace util 
    {
        
        line_t::line_t()
        {
            point_a.resize( 2 );
            point_b.resize( 2 );
            int_points.resize( 2 );
            int_points[0].resize( 2 );
            int_points[1].resize( 2 );
            hold_point.resize( 2 );
            hold_point2.resize( 2 );
            hold_point3.resize( 2 );
            dir_a.resize( 2 );
            dir_b.resize( 2 );
            valid = true;
            check_points.resize( 6 );
            for( unsigned int i=0; i< check_points.size(); ++i )
                check_points[i].resize(2);
            hold_points.resize( 2 );
            zero_vec.resize(2);
            zero_vec.zero();
        }
        
        line_t::line_t( const vector_t& a, const vector_t& b, bool inv )
        {
            int_points.resize( 2 );
            int_points[0].resize( 2 );
            int_points[1].resize( 2 );
            hold_point.resize( 2 );
            hold_point2.resize( 2 );
            hold_point3.resize( 2 );
            dir_a.resize( 2 );
            dir_b.resize( 2 );
            set_points( a, b );
            check_points.resize( 6 );
            for( unsigned int i=0; i< check_points.size(); ++i )
                check_points[i].resize(2);
            hold_points.resize( 2 );
            zero_vec.resize(2);
            zero_vec.zero();
            
            valid = inv;
        }
        
        line_t::~line_t()
        {
        }
        
        void line_t::set_points_from_line( const line_t& other )
        {
            point_a = other.point_a;
            point_b = other.point_b;
        }
        
        void line_t::set_from_angle( double& angle )
        {
            point_a[0] = point_a[1] = 0;
            point_b[0] = cos( angle );
            point_b[1] = sin( angle );
        }
        
        void line_t::swap()
        {
            vector_t tmp(point_a.get_data().get_dim());
            tmp = point_a.get_data();
            point_a = point_b;
            point_b = tmp;
        }
        
        int line_t::intersection( const two_point_primitive_t* other, std::vector< intersection_point_t >& ret ) const
        {
            //Do not perform any intersections if either prim is not valid
            if( !valid || !other->is_valid() )
            {
                ret[0].set_valid( false );
                ret[1].set_valid( false );
                return 0;
            }
            
            const line_t* tryline = dynamic_cast<const line_t*>(other);
            if( tryline )
                return intersection( tryline, ret );
            return -1;
        }
        
        int line_t::intersection( const line_t* other, std::vector< intersection_point_t >& ret ) const
        {
            // TODO: Why are there two intersection points when we are looking at two lines?
            //Do not perform any intersections if either prim is not valid
            if( !valid || !other->is_valid() )
            {
                ret[0].set_valid( false );
                ret[1].set_valid( false );
                return 0;
            }
            
            const int x = 0;
            const int y = 1;
            
            double ud = (other->point_b[y] - other->point_a[y])*(point_b[x] - point_a[x]) - (other->point_b[x] - other->point_a[x])*(point_b[y] - point_a[y]);
            double ua = (other->point_b[x] - other->point_a[x])*(point_a[y] - other->point_a[y]) - (other->point_b[y] - other->point_a[y])*(point_a[x] - other->point_a[x]);
            double ub = (point_b[x] - point_a[x])*(point_a[y] - other->point_a[y]) - (point_b[y] - point_a[y])*(point_a[x] - other->point_a[x]);
            
            if( ud == 0 )
            {
                ret[0].set_valid( false );
                ret[1].set_valid( false );
                return 0;
            }
            else
            {
                ua = ua / ud;
                ub = ub / ud;
                ret[0][x] = point_a[x] + ua*(point_b[x] - point_a[x]);
                ret[0][y] = point_a[y] + ua*(point_b[y] - point_a[y]);
                ret[0].set_valid( true );
                return 1;
            }
        }
        
        int line_t::intersection( const arc_t* other, std::vector< intersection_point_t >& ret ) const
        {
            //Do not perform any intersections if either prim is not valid
            if( !valid || !other->is_valid() )
            {
                ret[0].set_valid( false );
                ret[1].set_valid( false );
                return 0;
            }
            
            double d = 0;
            double rad = other->get_rad();
            const vector_t& center = other->get_point_c()->get_data();
            
            //Find point p
            const vector_t& p = other->project( center );
            //find magnitude, l
            double l = (p-center).norm();
            //If l < rad
            if( l < rad )
            {
                //Find the two points of intersection
                d = sqrt( rad*rad + l*l );
                const vector_t& dir = get_normalized_direction();
                ret[0] = p + dir*d;
                ret[0].set_valid( true );
                ret[1] = p - dir*d;
                ret[1].set_valid( true );
                return 2;
            }
            //Else if l == rad
            else if( l == rad )
            {
                //Report point p as the intersection point
                ret[0] = p;
                ret[0].set_valid( true );
                ret[1].set_valid( false );
                return 1;
            }
            //Else if l > rad
            else
            {
                //Report 0 intersection points
                ret[0].set_valid( false );
                ret[1].set_valid( false );
                return 0;
            }
        }
        
        double line_t::side( const vector_t& point ) const
        {
            return ((point_a[0]-point[0])*(point_b[1]-point[1])-(point_a[1]-point[1])*(point_b[0]-point[0]));
        }
        
        vector_t& line_t::project( const vector_t& point ) const
        {
            vector_t& l = hold_point;
            l = point_b.get_data();
            l -= point_a.get_data();
            l.normalize();
            vector_t& v = hold_point2;
            v = point;
            v -= point_a.get_data();
            double magn = l.dot_product( v );
            
            l *= magn;
            l += point_a.get_data();
            return l;
        }
        
        void line_t::construct_segment( double outer, double inner, line_t& lbnd, line_t& rbnd, line_t& segment, line_t& segment2 )
        {
            int nr_valid_pts = 0;
            double d = 0;
            bool valid = true;
            
            //a. Check intersection with left bound -> [0]
            lbnd.intersection( this, hold_points );
            check_points[0] = hold_points[0];
            d = check_points[0].norm();
            if( d <= outer && d >= inner && lbnd.on_ray( check_points[0].get_data() ) && on_ray( check_points[0].get_data() ) )
                check_points[0].set_valid( true );
            else
                check_points[0].set_valid( false );
            //b. Check intersection with right bound -> [1]
            rbnd.intersection( this, hold_points );
            check_points[1] = hold_points[0];
            d = check_points[1].norm();
            if( d <= outer && d >= inner && rbnd.on_ray( check_points[1].get_data() ) && on_ray( check_points[1].get_data() ) )
                check_points[1].set_valid( true );
            else
                check_points[1].set_valid( false );
            
            //b.i. Do an extra check if we have a full set of controls, which invalidates sides
            bool full_set = lbnd.get_lines_angle() == rbnd.get_lines_angle();
            if( full_set )
            {
                check_points[0].set_valid( false );
                check_points[1].set_valid( false );
            }
            
            //c. Check intersection with outer bound -> [2,3]
            find_arc_intersections( outer, lbnd, rbnd, check_points[2], check_points[3] );
            //d. Check intersection with inner bound -> [4,5]
            find_arc_intersections( inner, lbnd, rbnd, check_points[4], check_points[5] );
            //e. Segment Points <- Valid Points from Check Points
            segment_points.clear();
            for( unsigned int i=0; i<6; ++i )
            {
                if( check_points[i].is_valid() )
                {
                    valid = true;
                    for( int k=0; k<nr_valid_pts; ++k )
                        if( segment_points[k]->approximately_equal( check_points[i] ) )
                            valid = false;
                    if( valid )
                    {
                        segment_points.push_back( &check_points[i] );
                        ++nr_valid_pts;
                    }
                }
            }
            
            //f. Sort the points in ascending order if there are more than two points
            if( nr_valid_pts > 2 )
            {
                double dx = get_point_b()->get_data()[0] - get_point_a()->get_data()[0];
                for( int k=0; k<nr_valid_pts; ++k )
                {
                    segment_points[k]->set_interested_point( ( segment_points[k]->get_data()[0] - get_point_a()->get_data()[0] ) / dx );
                }
                std::sort(segment_points.begin(), segment_points.end(), compare_intersection_point_t() );
            }
            
            //Invalid point
            vector_t inval(2);
            inval[0] = PRX_INFINITY;
            
            //g. Switch on number of valid points found
            switch (nr_valid_pts)
            {
                case 0:
                    //            printf("\n");
                    //            PRX_LOG_WARNING("WTF? 0");
                    segment.set_valid( false );
                    segment.set_point_a(inval);
                    segment.set_point_b(inval);
                    segment2.set_valid( false );
                    segment2.set_point_a(inval);
                    segment2.set_point_b(inval);
                    break;
                case 1:
                    //            printf("\n");
                    //            PRX_LOG_WARNING("WTF? 1!?!");
                    get_point_a( hold_point );
                    segment.set_point_a( hold_point );
                    segment.set_point_b( *(segment_points[0]) );
                    segment.set_valid( true );
                    segment2.set_valid( false );
                    segment2.set_point_a(inval);
                    segment2.set_point_b(inval);
                    break;
                case 2:
                    //            printf("\n");
                    //            PRX_LOG_WARNING("WTF? 2!?!?!?!??!?!?!?!");
                    segment.set_point_a( *(segment_points[0]) );
                    segment.set_point_b( *(segment_points[1]) );
                    segment.set_valid( true );
                    segment2.set_valid( false );
                    segment2.set_point_a(inval);
                    segment2.set_point_b(inval);
                    break;
                case 3:
                    get_point_a( hold_point );
                    segment.set_point_a( hold_point );
                    segment.set_point_b( *(segment_points[0]) );
                    segment.set_valid( true );
                    segment2.set_point_a( *(segment_points[1] ) );
                    segment2.set_point_b( *(segment_points[2] ) );
                    segment2.set_valid( true );
                    //            printf("\n");
                    //            PRX_LOG_WARNING("Printing points in line's construct segments : hold point first");
                    //            hold_point.print();
                    //            segment_points[0]->print();
                    //            segment_points[1]->print();
                    //            segment_points[2]->print();
                    //            segment_points[3]->print();
                    break;
                case 4:
                    segment.set_point_a( *(segment_points[0]) );
                    segment.set_point_b( *(segment_points[1]) );
                    segment.set_valid( true );            
                    segment2.set_point_a( *(segment_points[2] ) );
                    segment2.set_point_b( *(segment_points[3] ) );
                    segment2.set_valid( true );
                    //            printf("\n");
                    //            PRX_LOG_WARNING("Printing points in line's construct segments : hold point first");
                    //            hold_point.print();
                    //            segment_points[0]->print();
                    //            segment_points[1]->print();
                    //            segment_points[2]->print();
                    //            segment_points[3]->print();
                    break;
                default:
                    PRX_WARN_S("ERROR: Something went wrong, we got" << nr_valid_pts << "segment points : ");
                    PRX_DEBUG_COLOR("FEASIBLE SET DATA : ", PRX_TEXT_CYAN );
                    PRX_DEBUG_COLOR("Outer : " << outer, PRX_TEXT_LIGHTGRAY );
                    PRX_DEBUG_COLOR("Inner : " << inner, PRX_TEXT_LIGHTGRAY );
                    PRX_DEBUG_COLOR("Left bound: " << lbnd, PRX_TEXT_LIGHTGRAY );
                    PRX_DEBUG_COLOR("Right bound: " << rbnd, PRX_TEXT_LIGHTGRAY );
                    PRX_DEBUG_COLOR("POINT DATA : ", PRX_TEXT_CYAN );
                    for( unsigned int i=0; i<6; ++i )
                        PRX_DEBUG_COLOR("[" << i << "] : " << check_points[i], PRX_TEXT_LIGHTGRAY );
                    PRX_DEBUG_COLOR("LINE IN QUESTION : ", PRX_TEXT_CYAN );
                    PRX_DEBUG_COLOR("Line: " << *this, PRX_TEXT_LIGHTGRAY );
                    PRX_FATAL_S("Unknown how to resolve error, aborting.");
                    break;
            }
        }
        
        void line_t::inf_construct_segment( double outer, double inner, line_t& lbnd, line_t& rbnd, line_t& segment, line_t& segment2 )
        {
            int nr_valid_pts = 0;
            double d = 0;
            bool valid = true;
            
            //a. Check intersection with left bound -> [0]
            lbnd.intersection( this, hold_points );
            check_points[0] = hold_points[0];
            d = check_points[0].norm();
            if( d <= outer && d >= inner && lbnd.on_ray( check_points[0].get_data() ) )
                check_points[0].set_valid( true );
            else
                check_points[0].set_valid( false );
            //b. Check intersection with right bound -> [1]
            rbnd.intersection( this, hold_points );
            check_points[1] = hold_points[0];
            d = check_points[1].norm();
            if( d <= outer && d >= inner && rbnd.on_ray( check_points[1].get_data() ) )
                check_points[1].set_valid( true );
            else
                check_points[1].set_valid( false );
            
            //b.i. Do an extra check if we have a full set of controls, which invalidates sides
            bool full_set = lbnd.get_lines_angle() == rbnd.get_lines_angle();
            if( full_set )
            {
                check_points[0].set_valid( false );
                check_points[1].set_valid( false );
            }
            
            //c. Check intersection with outer bound -> [2,3]
            find_inf_arc_intersections( outer, lbnd, rbnd, check_points[2], check_points[3] );
            //d. Check intersection with inner bound -> [4,5]
            find_inf_arc_intersections( inner, lbnd, rbnd, check_points[4], check_points[5] );
            //e. Segment Points <- Valid Points from Check Points
            segment_points.clear();
            for( unsigned int i=0; i<6; ++i )
            {
                if( check_points[i].is_valid() )
                {
                    valid = true;
                    for( int k=0; k<nr_valid_pts; ++k )
                        if( segment_points[k]->approximately_equal( check_points[i] ) )
                            valid = false;
                    if( valid )
                    {
                        segment_points.push_back( &check_points[i] );
                        ++nr_valid_pts;
                    }
                }
            }
            
            //f. Sort the points in ascending order if there are more than two points
            if( nr_valid_pts > 2 )
            {
                double dx = get_point_b()->get_data()[0] - get_point_a()->get_data()[0];
                for( int k=0; k<nr_valid_pts; ++k )
                {
                    segment_points[k]->set_interested_point( ( segment_points[k]->get_data()[0] - get_point_a()->get_data()[0] ) / dx );
                }
                std::sort(segment_points.begin(), segment_points.end(), compare_intersection_point_t() );
            }
            
            //g. Switch on number of valid points found
            switch (nr_valid_pts)
            {
                case 0:
                    segment.set_valid( false );
                    segment2.set_valid( false );
                    break;
                case 1:
                    get_point_a( hold_point );
                    segment.set_point_a( hold_point );
                    segment.set_point_b( *(segment_points[0]) );
                    segment.set_valid( true );
                    segment2.set_valid( false );
                    break;
                case 2:
                    segment.set_point_a( *(segment_points[0]) );
                    segment.set_point_b( *(segment_points[1]) );
                    segment.set_valid( true );
                    segment2.set_valid( false );
                    break;
                case 3:
                    get_point_a( hold_point );
                    segment.set_point_a( hold_point );
                    segment.set_point_b( *(segment_points[0]) );
                    segment.set_valid( true );
                    segment2.set_point_a( *(segment_points[1] ) );
                    segment2.set_point_b( *(segment_points[2] ) );
                    segment2.set_valid( true );
                    break;
                case 4:
                    segment.set_point_a( *(segment_points[0]) );
                    segment.set_point_b( *(segment_points[1]) );
                    segment.set_valid( true );            
                    segment2.set_point_a( *(segment_points[2] ) );
                    segment2.set_point_b( *(segment_points[3] ) );
                    segment2.set_valid( true );
                    break;
                default:
                    PRX_WARN_S("ERROR: Something went wrong, we got" << nr_valid_pts << "segment points : ");
                    PRX_DEBUG_COLOR("FEASIBLE SET DATA : ", PRX_TEXT_CYAN );
                    PRX_DEBUG_COLOR("Outer : " << outer, PRX_TEXT_LIGHTGRAY );
                    PRX_DEBUG_COLOR("Inner : " << inner, PRX_TEXT_LIGHTGRAY );
                    PRX_DEBUG_COLOR("Left bound: " << lbnd, PRX_TEXT_LIGHTGRAY );
                    PRX_DEBUG_COLOR("Right bound: " << rbnd, PRX_TEXT_LIGHTGRAY );
                    PRX_DEBUG_COLOR("POINT DATA : ", PRX_TEXT_CYAN );
                    for( unsigned int i=0; i<6; ++i )
                        PRX_DEBUG_COLOR("[" << i << "] : " << check_points[i], PRX_TEXT_LIGHTGRAY );
                    PRX_DEBUG_COLOR("LINE IN QUESTION : ", PRX_TEXT_CYAN );
                    PRX_DEBUG_COLOR("Line: " << *this, PRX_TEXT_LIGHTGRAY );
                    PRX_FATAL_S("Unknown how to resolve error, aborting.");
                    break;
            }
        }
        
        int line_t::sweep( const std::vector< intersection_point_t* >& int_points, std::vector< bool >& violated_constraints, std::vector< line_t >& hold_subsegments, int index, int delimeter )
        {
            bool out = true;
            int counter = 0;
            int nr_endpts = 0;
            
            std::vector< int > endpts;
            
            //First, count up the number of violated constraints
            for( unsigned i=0; i<violated_constraints.size(); ++i )
            {
                if( violated_constraints[i] )
                {
                    ++counter;
                }
            }
            
            //If the segment we are looking at is even valid
            if( is_valid() )
            {        
                //If the number of VOs we're in is zero
                if( counter == 0 )
                {
                    //Add the begining point as a valid endpoint
                    endpts.push_back( -1 );
                    ++nr_endpts;
                    out = true;
                }
                else
                    out = false;
                
                for( unsigned int i=0; i<int_points.size(); ++i )
                {            
                    //Get the index of the line which caused this intersection
                    const int& pt_index = int_points[i]->get_index();
                    
                    //Flip the bit for whether we are in this VO
                    violated_constraints[pt_index/delimeter] = (violated_constraints[pt_index/delimeter])?false:true;
                    
                    //Adjust the counter accordingly, so we know how many VOs we're in
                    if( violated_constraints[pt_index/delimeter] )
                    {
                        ++counter;
                    }
                    else
                    {
                        --counter;
                    }
                    //If we currently are out of all VOs
                    if( out )
                    {
                        //And the counter has become greater than 0
                        if( counter != 1 )
                        {
                            PRX_FATAL_S("line_t :: The counter is not 1 when we expect it!");
                        }
                        //Realize we have ended up in a VO
                        out = false;
                        endpts.push_back( i );
                        ++nr_endpts;
                    }
                    else
                    { 
                        if( counter == 0 )
                        {
                            out = true;
                            endpts.push_back( i );
                            ++nr_endpts;
                        }
                    }
                }
                
                //Check the endpoint
                if( out )
                {
                    endpts.push_back( PRX_INFINITY );
                    ++nr_endpts;
                }
                
                if( nr_endpts%2 != 0 )
                    PRX_FATAL_S("Calculated endpoints are odd!  Clearly something wrong!");
                
                for( int k=0; k<nr_endpts; k+=2 )
                {
                    if( endpts[k] == -1 )
                        get_point_a( hold_point );
                    else
                        hold_point = (int_points[endpts[k]])->get_data();
                    
                    if( endpts[k+1] == PRX_INFINITY )
                        get_point_b( hold_point2 );
                    else
                        hold_point2 = (int_points[endpts[k+1]])->get_data();
                    
                    hold_subsegments[k/2].set_point_a( hold_point );
                    hold_subsegments[k/2].set_point_b( hold_point2 );
                }
            }
            return nr_endpts / 2;
        }
        
        void line_t::find_arc_intersections( double r, line_t& ll, line_t& rl, intersection_point_t& pta, intersection_point_t& ptb )
        {
            hold_point = project( zero_vec );
            double dsq = hold_point.squared_norm();
            double l = sqrt( r*r - dsq );
            if( dsq >= r*r )
            {
                pta.set_valid( false );
                ptb.set_valid( false );
            }
            else
            {
                pta = hold_point;
                ptb = hold_point;
                hold_point = get_normalized_direction();
                hold_point *= -l;
                pta += hold_point;
                hold_point = get_normalized_direction();
                hold_point *= l;
                ptb += hold_point;
                
                bool lower_on = on_ray( pta.get_data() );
                bool upper_on = on_ray( ptb.get_data() );
                
                if( upper_on && lower_on )
                {
                    pta.set_valid( true );
                    ptb.set_valid( true );
                }
                else if( lower_on )
                {
                    pta.set_valid( true );
                    ptb.set_valid( false );
                }
                else if( upper_on )
                {
                    pta.set_valid( false );
                    ptb.set_valid( true );
                }
                else
                {
                    pta.set_valid( false );
                    ptb.set_valid( false );
                }
            }
            
            bool full_set = ( ll.get_lines_angle() == rl.get_lines_angle() );
            if( !full_set )
            {
                if ( ll.side( pta.get_data() ) > 0 || rl.side( pta.get_data() ) < 0 )
                    pta.set_valid( false );
                
                if ( ll.side( ptb.get_data() ) > 0 || rl.side( ptb.get_data() ) < 0 )
                    ptb.set_valid( false );
            }
        }
        
        void line_t::find_inf_arc_intersections( double r, line_t& ll, line_t& rl, intersection_point_t& pta, intersection_point_t& ptb )
        {
            hold_point = project( zero_vec );
            double dsq = hold_point.squared_norm();
            double l = sqrt( r*r - dsq );
            if( dsq >= r*r )
            {
                pta.set_valid( false );
                ptb.set_valid( false );
            }
            else
            {
                pta = hold_point;
                ptb = hold_point;
                hold_point = get_normalized_direction();
                hold_point *= -l;
                pta += hold_point;
                hold_point = get_normalized_direction();
                hold_point *= l;
                ptb += hold_point;
                
                pta.set_valid( true );
                ptb.set_valid( true );
            }
            
            bool full_set = ( ll.get_lines_angle() == rl.get_lines_angle() );
            if( !full_set )
            {
                if ( ll.side( pta.get_data() ) > 0 || rl.side( pta.get_data() ) < 0 )
                    pta.set_valid( false );
                
                if ( ll.side( ptb.get_data() ) > 0 || rl.side( ptb.get_data() ) < 0 )
                    ptb.set_valid( false );
            }
        }
        
        void line_t::calculate_tangent( const vector_t& center, double radius, bool side )
        {
            double sq_dist = center[0]*center[0] + center[1]*center[1];
            double sq_rad = radius*radius;
            double epsilon = (1- (sq_rad/sq_dist));
            double alpha = ( radius/sq_dist ) * sqrt( sq_dist - sq_rad );
            
            if( !side )
            {
                point_b[0] = epsilon*center[0] - alpha*center[1];
                point_b[1] = epsilon*center[1] + alpha*center[0];
            }
            else
            {
                point_b[0] = epsilon*center[0] + alpha*center[1];
                point_b[1] = epsilon*center[1] - alpha*center[0];
            }
        }
        
        void line_t::bisect( const line_t& left, const line_t& right )
        {
            left.intersection( &right, int_points );
            point_a = int_points[0];
            const vector_t& norml = left.get_normalized_direction();
            const vector_t& normr = right.get_normalized_direction();
            const vector_t& result = ( norml + normr ) / 2.0;
            point_b = result;
            point_b += point_a.get_data();
        }
        
        void line_t::translate( const vector_t& vec )
        {
            point_a += vec;
            point_b += vec;
        }
        
        void line_t::rotate( double angle )
        {
            double langle = norm_angle_zero( atan2(point_b[1] - point_a[1],point_b[0] - point_a[0]) );
            langle = langle + angle;
            double magn = magnitude();
            point_b[0] = magn*cos( langle );
            point_b[1] = magn*sin( langle );
            point_b += point_a.get_data();
        }
        
        bool line_t::on_ray( const vector_t& point ) const
        {
            if( fabs( side( point ) ) < PRX_ZERO_CHECK )
            {
                if( point_a[0] < point_b[0] )
                {
                    if( point[0] > point_a[0] )
                        return true;
                    return false;
                }
                else if( point_a[0] > point_b[0] )
                {
                    if( point[0] < point_a[0] )
                        return true;
                    return false;        
                }
                else
                {
                    if( point_a[1] < point_b[1] )
                    {
                        if( point[1] > point_a[1] )
                            return true;
                        return false;
                    }
                    else
                    {
                        if( point[1] < point_a[1] )
                            return true;
                        return false;            
                    }
                }
            }
            return false;
        }
        
        double line_t::magnitude( ) const
        {
            return sqrt( pow( (point_b[0]-point_a[0]), 2.0 ) + pow( (point_b[1]-point_a[1]), 2.0 ) );
        }
        
        double line_t::sq_magnitude( ) const
        {
            return pow( (point_b[0]-point_a[0]), 2.0 ) + pow( (point_b[1]-point_a[1]), 2.0 );
        }
        
        int line_t::seg_intersection ( const two_point_primitive_t* other, std::vector< intersection_point_t >& points ) const
        {
            //Do not perform any intersections if either prim is not valid
            if( !valid || !other->is_valid() )
            {
                points[0].set_valid( false );
                points[1].set_valid( false );
                return 0;
            }
            
            const line_t* tryline = dynamic_cast< const line_t* > ( other );
            if( tryline )
                return seg_intersection( tryline, points );
            return -1;
        }
        
        int line_t::seg_intersection ( const line_t* other, std::vector< intersection_point_t >& int_point ) const
        {
            //Do not perform any intersections if either prim is not valid
            if( !valid || !other->is_valid() )
            {
                int_point[0].set_valid( false );
                int_point[1].set_valid( false );
                return 0;
            }
            
            const int x = 0;
            const int y = 1;
            
            double ud = (other->point_b[y] - other->point_a[y])*(point_b[x] - point_a[x]) - (other->point_b[x] - other->point_a[x])*(point_b[y] - point_a[y]);
            double ub = (other->point_b[x] - other->point_a[x])*(point_a[y] - other->point_a[y]) - (other->point_b[y] - other->point_a[y])*(point_a[x] - other->point_a[x]);
            double ua = (point_b[x] - point_a[x])*(point_a[y] - other->point_a[y]) - (point_b[y] - point_a[y])*(point_a[x] - other->point_a[x]);
            
            if( ud == 0 )
            {
                int_point[0].set_valid( false );
                return 0;
            }
            else
            {
                ua = ua / ud;
                ub = ub / ud;
                int_point[0][x] = point_a[x] + ub*(point_b[x] - point_a[x]);
                int_point[0][y] = point_a[y] + ub*(point_b[y] - point_a[y]);
            }
            
            int_point[0].set_ua( ua );
            int_point[0].set_ub( ub );
            
            if( (ua >= 0 && ua <= 1) && (ub >= 0 && ub <= 1) )
            {
                int_point[0].set_valid( true );
                return 1;
            }
            else
            {
                int_point[0].set_valid( false );
                return 0;
            }
        }
        
        int line_t::coincident_seg_intersection ( const line_t* other, std::vector< intersection_point_t >& int_point ) const
        {
            //Do not perform any intersections if either prim is not valid
            if( !valid || !other->is_valid() )
            {
                int_point[0].set_valid( false );
                int_point[1].set_valid( false );
                return 0;
            }
            
            const int x = 0;
            const int y = 1;
            
            double ud = (other->point_b[y] - other->point_a[y])*(point_b[x] - point_a[x]) - (other->point_b[x] - other->point_a[x])*(point_b[y] - point_a[y]);
            double ub = (other->point_b[x] - other->point_a[x])*(point_a[y] - other->point_a[y]) - (other->point_b[y] - other->point_a[y])*(point_a[x] - other->point_a[x]);
            double ua = (point_b[x] - point_a[x])*(point_a[y] - other->point_a[y]) - (point_b[y] - point_a[y])*(point_a[x] - other->point_a[x]);
            
            if( ud == 0 )
            {
                int_point[0].set_valid( false );
                return 0;
            }
            else
            {
                ua = ua / ud;
                ub = ub / ud;
                int_point[0][x] = point_a[x] + ub*(point_b[x] - point_a[x]);
                int_point[0][y] = point_a[y] + ub*(point_b[y] - point_a[y]);
            }
            
            int_point[0].set_ua( ua );
            int_point[0].set_ub( ub );
            
            if( (ua > 0 && ua < 1) && (ub > 0 && ub < 1) )
            {
                int_point[0].set_valid( true );
                return 1;
            }
            else
            {
                int_point[0].set_valid( false );
                return 0;
            }
        }
        
        int line_t::seg_intersection ( const arc_t* other, std::vector< intersection_point_t >& ret, bool utype ) const
        {
            //    PRX_LOG_INFO("In line->arc segment intersection");
            
            //Do not perform any intersections if either prim is not valid
            if( !valid || !other->is_valid() )
            {
                ret[0].set_valid( false );
                ret[1].set_valid( false );
                return 0;
            }
            
            //    PRX_LOG_INFO("We continue.");
            
            int pts = other->intersection( this, ret );
            int amt = pts;
            
            double val = 0;
            double dx = 0, dy = 0;
            
            for( int i=0; i<amt; ++i )
            {
                //First, find ua of the point
                dir_a = other->get_point_a()->get_data(); 
                dir_a -= other->get_point_c()->get_data();
                dir_b = ret[i].get_data();
                dir_b -= other->get_point_c()->get_data();
                double angle = norm_angle_zero( atan2( dir_b[1], dir_b[0] ) );
                double s_angle = norm_angle_zero( atan2( dir_a[1], dir_a[0] ) );
                val = other->swept_angle( s_angle, angle )/other->get_swept_angle();
                if( val > 1 || val < 0 )
                {
                    --pts;
                    ret[i].set_valid( false );
                }
                if( utype == false )
                {
                    ret[i].set_ub( val );
                }
                else
                {
                    other->get_point_c( hold_point );
                    other->get_point_a( hold_point2 );
                    const vector_t& other_center = hold_point;
                    const vector_t& opta = hold_point2;
                    ret[i].set_ub( other->swept_angle( atan2( opta[1]-other_center[1], opta[0]-other_center[0] ), atan2( ret[i].get_data()[1]-other_center[1], ret[i].get_data()[0]-other_center[0] ) ) / other->get_swept_angle() );            
                }
                
                double ax = point_a[0];
                double bx = point_b[0];
                
                double px = ret[i][0];
                if( ax < bx )
                {
                    dx = bx - ax;
                    val = px - ax;
                    val /= dx;
                    ret[i].set_ua( val );
                    if( val > 1 || val < 0 )
                    {
                        --pts;
                        ret[i].set_valid( false );
                    }
                }
                else if( ax > bx )
                {
                    dx = ax - bx;
                    val = -( px - ax );
                    val /= dx;
                    ret[i].set_ua( val );
                    if( val > 1 || val < 0 )
                    {
                        --pts;
                        ret[i].set_valid( false );
                    }
                }
                else
                {
                    double ay = point_a[1];
                    double by = point_b[1];
                    
                    double py = ret[i][1];
                    if( ay < by )
                    {
                        dy = by - ay;
                        val = py - ay;
                        val /= dy;
                        ret[i].set_ua( val );
                        if( val > 1 || val < 0 )
                        {
                            --pts;
                            ret[i].set_valid( false );
                        }
                    }
                    else if( ay > by )
                    {
                        dy = ay - by;
                        val = -( py - ay );
                        val /= dy;
                        ret[i].set_ua( val );
                        if( val > 1 || val < 0 )
                        {
                            --pts;
                            ret[i].set_valid( false );
                        }
                    }
                    else
                    {
                        //                PRX_LOG_ERROR ("Same two points?!");
                        //                point_a.print();
                        //                point_b.print();
                        ret[i].set_valid(false);
                    }
                }
            }
            
            return pts;
        }
        
        int line_t::coincident_seg_intersection ( const arc_t* other, std::vector< intersection_point_t >& ret, bool utype ) const
        {
            //Do not perform any intersections if either prim is not valid
            if( !valid || !other->is_valid() )
            {
                ret[0].set_valid( false );
                ret[1].set_valid( false );
                return 0;
            }
            
            int pts = other->intersection( this, ret );
            int amt = pts;
            
            double val = 0;
            double dx = 0, dy = 0;
            
            for( int i=0; i<amt; ++i )
            {
                //First, find ua of the point
                dir_a = other->get_point_a()->get_data(); 
                dir_a -= other->get_point_c()->get_data();
                dir_b = ret[i].get_data();
                dir_b -= other->get_point_c()->get_data();
                double angle = norm_angle_zero( atan2( dir_b[1], dir_b[0] ) );
                double s_angle = norm_angle_zero( atan2( dir_a[1], dir_a[0] ) );
                val = other->swept_angle( s_angle, angle )/other->get_swept_angle();
                if( val >= 1 || val <= 0 )
                {
                    --pts;
                    ret[i].set_valid( false );
                }
                if( utype == false )
                {
                    ret[i].set_ub( val );
                }
                else
                {
                    other->get_point_c( hold_point );
                    other->get_point_a( hold_point2 );
                    const vector_t& other_center = hold_point;
                    const vector_t& opta = hold_point2;
                    ret[i].set_ub( other->swept_angle( atan2( opta[1]-other_center[1], opta[0]-other_center[0] ), atan2( ret[i].get_data()[1]-other_center[1], ret[i].get_data()[0]-other_center[0] ) ) / other->get_swept_angle() );            
                }
                
                double ax = point_a[0];
                double bx = point_b[0];
                
                double px = ret[i][0];
                if( ax < bx )
                {
                    dx = bx - ax;
                    val = px - ax;
                    val /= dx;
                    ret[i].set_ua( val );
                    if( val >= 1 || val <= 0 )
                    {
                        --pts;
                        ret[i].set_valid( false );
                    }
                }
                else if( ax > bx )
                {
                    dx = ax - bx;
                    val = -( px - ax );
                    val /= dx;
                    ret[i].set_ua( val );
                    if( val >= 1 || val <= 0 )
                    {
                        --pts;
                        ret[i].set_valid( false );
                    }
                }
                else
                {
                    double ay = point_a[1];
                    double by = point_b[1];
                    
                    double py = ret[i][1];
                    if( ay < by )
                    {
                        dy = by - ay;
                        val = py - ay;
                        val /= dy;
                        ret[i].set_ua( val );
                        if( val >= 1 || val <= 0 )
                        {
                            --pts;
                            ret[i].set_valid( false );
                        }
                    }
                    else if( ay > by )
                    {
                        dy = ay - by;
                        val = -( py - ay );
                        val /= dy;
                        ret[i].set_ua( val );
                        if( val >= 1 || val <= 0 )
                        {
                            --pts;
                            ret[i].set_valid( false );
                        }
                    }
                    else
                    {
                        //                PRX_LOG_ERROR ("Same two points?!");
                        //                point_a.print();
                        //                point_b.print();
                        ret[i].set_valid(false);
                    }
                }
            }
            
            return pts;
        }
        
        double line_t::distance_from_point( const vector_t& point )
        {
            hold_point = point;
            return seg_get_closest_point( point, hold_point );
        }
        
        double line_t::seg_get_closest_point( const vector_t& point, vector_t& closest_point ) const
        {
            vector_t& v_line = hold_point;
            v_line = point_b.get_data();
            v_line -= point_a.get_data();
            vector_t& v_point = hold_point2;
            v_point = point;
            v_point -= point_a.get_data();
            
            double u = v_line.dot_product( v_point );
            u /= sq_magnitude();
            
            if( u < 0 )
                closest_point = point_a.get_data();
            else if( u > 1 )
                closest_point = point_b.get_data();
            else
                closest_point = point_a.get_data() + (v_line*u);
            
            return closest_point.distance( point );    
        }
        
        double line_t::get_lines_angle() const
        {
            return atan2(point_b[1] - point_a[1],point_b[0] - point_a[0]);
        }
        
        double line_t::get_lines_slope() const
        {
            return (point_b[1] - point_a[1])/(point_b[0] - point_a[0]);
        }
        
        double line_t::get_length() const
        {
            double difx = point_b[0] - point_a[0];
            double dify = point_b[1] - point_b[1];
            return sqrt(difx*difx + dify*dify);
        }
        
        vector_t& line_t::get_normalized_direction( ) const
        {
            vector_t& ret = hold_point;
            
            ret = point_b.get_data();
            ret[0] -= point_a[0];
            ret[1] -= point_a[1];
            double magn = sqrt( ret[0]*ret[0] + ret[1]*ret[1] );
            
            ret[0] /= magn;
            ret[1] /= magn;
            
            return ret;
        }

        std::ostream& operator<<( std::ostream& out, const line_t& line )
        {
            out << "L: ( " << line.point_a[0] << ", " << line.point_a[1] << " ), ( " << line.point_b[0] << ", " << line.point_b[1] << " ) " << (line.valid?"VAL":"INV") << "    Theta: " <<  norm_angle_zero( line.get_lines_angle() ); 
            return out;
        }
    } 
}

