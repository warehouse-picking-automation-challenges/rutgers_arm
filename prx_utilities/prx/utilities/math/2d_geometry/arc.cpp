/**
 * @file arc.cpp 
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

#include "prx/utilities/math/2d_geometry/arc.hpp"
#include "prx/utilities/math/2d_geometry/angle.hpp"
#include "prx/utilities/math/2d_geometry/line.hpp"

namespace prx 
{ 
    namespace util 
    {
        
        arc_t::arc_t()
        {
            dir_a.resize( 2 );
            dir_b.resize( 2 );
            
            points.resize( 8 );
            for( unsigned i=0; i<8; ++i )
            {
                points[i].resize(2);
            }
            hold_points.resize( 2 );
            hold_point.resize( 2 );
            hold_point2.resize( 2 );
            
            calculated = false;
        }
        
        arc_t::~arc_t()
        {
        }
        
        void arc_t::set_points( const vector_t& a, const vector_t& b, const vector_t& c )
        {
            point_a = a;
            point_b = b;
            center = c;
            calc_rad();
        }
        
        void arc_t::set_point_c( const vector_t& c )
        {
            center = c;
        }
        
        void arc_t::set_point_c( const intersection_point_t& c )
        {
            center = c;
        }
        
        void arc_t::get_points( vector_t& a, vector_t& b, vector_t& c ) const
        {
            a = point_a.get_data();
            b = point_b.get_data();
            c = center.get_data();
        }
        
        void arc_t::get_point_c( vector_t& c ) const
        {
            c = center.get_data();
        }
        
        double arc_t::get_rad() const
        {
            return rad;
        }
        
        void arc_t::calc_rad()
        {
            const vector_t& dif = point_a.get_data() - center.get_data();
            rad = dif.norm();
            const vector_t& odif = point_b.get_data() - center.get_data();
            double d = odif.norm();
            double raddiff = fabs(rad - d);
            if( rad < PRX_DISTANCE_CHECK || raddiff > 0.001 )
            {
                valid = false;
            }
            else
            {
                valid = true;
            }
            calculated = true;
        }
        
        arc_t& arc_t::operator=( const arc_t& other )
        {
            point_a = other.point_a;
            point_b = other.point_b;
            center = other.center;
            rad = other.rad;
            valid = other.valid;
            calculated = other.calculated;
            calc_rad();
            return *(this);
        }
        
        const intersection_point_t* arc_t::get_point_c( ) const
        {
            return &center;
        }
        
        intersection_point_t* arc_t::get_point_c( )
        {
            return &center;
        }
        
        double arc_t::get_swept_angle() const
        {
            dir_a = point_a.get_data();
            dir_a -= center.get_data();
            dir_b = point_b.get_data();
            dir_b -= center.get_data();
            double A = norm_angle_zero( atan2( dir_a[1], dir_a[0] ) );
            double B = norm_angle_zero( atan2( dir_b[1], dir_b[0] ) );
            return swept_angle( A, B );
        }
        
        double arc_t::swept_angle( double A, double B ) const
        {
            if( A > B )
                return A - B;
            else
                return A - B + 2*PRX_PI;
        }
        
        int arc_t::intersection( const two_point_primitive_t* other, std::vector< intersection_point_t >& ret ) const
        {
            check_calc();
            const line_t* line = dynamic_cast< const line_t* > (other);
            const arc_t* arc = dynamic_cast< const arc_t* > (other);
            if( line )
                return intersection( line, ret );
            else if ( arc )
                return intersection( arc, ret );
            return -1;
        }
        
        int arc_t::intersection( const line_t* other, std::vector< intersection_point_t >& ret ) const
        {
            check_calc();
            double d = 0;
            
            //Find point p
            const vector_t p = other->project( center.get_data() );
            //find magnitude, l
            double l = (p-center.get_data()).norm();
            
            //If l < rad
            if( l < rad )
            {
                //Find the two points of intersection
                d = sqrt( rad*rad - l*l );
                const vector_t& dir = other->get_normalized_direction();
                ret[0] = p + (dir*d);
                ret[0].set_valid( true );
                ret[1] = p - (dir*d);
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
        
        int arc_t::intersection( const arc_t* other, std::vector< intersection_point_t >& ret  ) const
        {
            check_calc();
            //  PRX_LOG_DEBUG ("Inside arc intersection");
            // The two circles are |X-C0| = R0 and |X-C1| = R1.  Define U = C1 - C0
            // and V = Perp(U) where Perp(x,y) = (y,-x).  Note that Dot(U,V) = 0 and
            // |V|^2 = |U|^2.  The intersection points X can be written in the form
            // X = C0+s*U+t*V and X = C1+(s-1)*U+t*V.  Squaring the circle equations
            // and substituting these formulas into them yields
            //   R0^2 = (s^2 + t^2)*|U|^2
            //   R1^2 = ((s-1)^2 + t^2)*|U|^2.
            // Subtracting and solving for s yields
            //   s = ((R0^2-R1^2)/|U|^2 + 1)/2
            // Then replace in the first equation and solve for t^2
            //   t^2 = (R0^2/|U|^2) - s^2.
            // In order for there to be solutions, the right-hand side must be
            // nonnegative.  Some algebra leads to the condition for existence of
            // solutions,
            //   (|U|^2 - (R0+R1)^2)*(|U|^2 - (R0-R1)^2) <= 0.
            // This reduces to
            //   |R0-R1| <= |U| <= |R0+R1|.
            // If |U| = |R0-R1|, then the circles are side-by-side and just tangent.
            // If |U| = |R0+R1|, then the circles are nested and just tangent.
            // If |R0-R1| < |U| < |R0+R1|, then the two circles to intersect in two
            // points.
            
            vector_t U = other->get_point_c()->get_data() - center.get_data();
            double USqrLen = U.squared_norm();
            double R0 = this->rad, R1 = other->get_rad();
            double R0mR1 = R0 - R1;
            
            //    PRX_LOG_DEBUG ("U vector is");
            //    U.print();
            //    PRX_LOG_DEBUG (" Squared Length is %f", USqrLen);
            //    PRX_LOG_DEBUG ( " R0: %f, R1: %f, R0-R1: %f", R0, R1, R0mR1);
            if (USqrLen < PRX_ZERO_CHECK && std::fabs(R0mR1) < PRX_ZERO_CHECK)
            {
                // Circles are essentially the same.
                ret[0].set_valid( false );
                ret[1].set_valid( false );
                PRX_WARN_S ("WARNING! Coincidental arcs !");
                return 0;
            }
            
            double R0mR1Sqr = R0mR1*R0mR1;
            if (USqrLen < R0mR1Sqr)
            {
                ret[0].set_valid( false );
                ret[1].set_valid( false );
                return 0;
            }
            
            double R0pR1 = R0 + R1;
            double R0pR1Sqr = R0pR1*R0pR1;
            if (USqrLen > R0pR1Sqr)
            {
                ret[0].set_valid( false );
                ret[1].set_valid( false );
                return 0;
            }
            
            if (USqrLen < R0pR1Sqr)
            {
                if (R0mR1Sqr < USqrLen)
                {
                    double invUSqrLen = ((double)1)/USqrLen;
                    double s = (0.5)*((R0*R0-R1*R1)*invUSqrLen + 1.0);
                    vector_t tmp = center.get_data() + U*s;
                    
                    // In theory, diskr is nonnegative.  However, numerical round-off
                    // errors can make it slightly negative.  Clamp it to zero.
                    double diskr = R0*R0*invUSqrLen - s*s;
                    if (diskr < PRX_ZERO_CHECK)
                    {
                        diskr = 0.0;
                    }
                    double t = std::sqrt(diskr);
                    vector_t V(U[1], -U[0]);
                    
                    ret[0] = tmp - V*t;
                    ret[1] = tmp + V*t;
                    ret[0].set_valid( true );
                    ret[1].set_valid( true );
                    return 2;
                }
                else
                {
                    //PRX_LOG_DEBUG ("|U| = |R0-R1|, circles are tangent. ");
                    ret[0] = center.get_data() + U*(R0/R0mR1);
                    ret[0].set_valid( true );
                    ret[1].set_valid( false );
                    return 1;
                }
            }
            else
            {
                //PRX_LOG_DEBUG ("|U| = |R0+R1|, circles are tangent. ");
                ret[0] = center.get_data() + U*(R0/R0pR1);
                ret[0].set_valid( true );
                ret[1].set_valid( false );
                return 1;
            }
            
            
        }
        
        double arc_t::side( const vector_t& point ) const
        {
            v = point;
            v -= center.get_data();
            // positive: inside
            // negative: outside
            return rad - v.norm();
        }
        
        vector_t& arc_t::project( const vector_t& point ) const
        {
            check_calc();
            vector_t& v = hold_point;
            v = point;
            v -= center.get_data();
            v *= ( rad / v.norm() );
            v += center.get_data();
            return v;
        }
        
        void arc_t::construct_segment( double outer, double inner, line_t& lbnd, line_t& rbnd, arc_t& seg, arc_t& seg2 ) const 
        {   
            check_calc();
            double &r = outer;
            const double &o = rad;
            const double &d = center.norm();
            
            if( r + o > d )
            {
                const double &s = (r + o + d)/2;
                const double &h = 2*(sqrt( s * (s-r) * (s-o) * (s-d) ) ) / d;
                const double &l = sqrt( r*r - h*h );
                
                const vector_t& p = (center.get_data() / d) * l;
                double angle = norm_angle_zero( atan2( center[1], center[0] ) - PRX_PI/2.0 );
                vector_t& c = hold_point;
                hold_point[0] = cos( angle );
                hold_point[1] = sin( angle );
                c *= h;
                
                points[0] = p + c;
                points[1] = p - c;
                if( on_ray( points[0].get_data() ) )
                {
                    points[0].set_valid( true );
                    points[0].set_interested_point( norm_angle_zero( atan2( points[0][1]-center[1], points[0][0]-center[0] ) ) );
                }
                else
                {
                    points[0].set_valid( false );
                }
                if( on_ray( points[1].get_data() ) )
                {
                    points[1].set_valid( true );
                    points[1].set_interested_point( norm_angle_zero( atan2( points[1][1]-center[1], points[1][0]-center[0] ) ) );
                }
                else
                {
                    points[1].set_valid( false );
                }
            }
            
            r = inner;
            
            if( r + o > d )
            {
                const double &s = (r + o + d)/2;
                const double &h = 2*(sqrt( s * (s-r) * (s-o) * (s-d) ) ) / d;
                const double &l = sqrt( r*r - h*h );
                
                const vector_t& p = (center.get_data() / d) * l;
                double angle = norm_angle_zero( atan2( center[1], center[0] ) - PRX_PI/2.0 );
                vector_t& c = hold_point;
                hold_point[0] = cos( angle );
                hold_point[1] = sin( angle );
                c *= h;
                
                points[2] = p + c;
                points[3] = p - c;
                if( on_ray( points[2].get_data() ) )
                {
                    points[2].set_valid( true );
                    points[2].set_interested_point( norm_angle_zero( atan2( points[2][1]-center[1], points[2][0]-center[0] ) ) );
                }
                else
                {
                    points[2].set_valid( false );
                }
                if( on_ray( points[3].get_data() ) )
                {
                    points[3].set_valid( true );
                    points[3].set_interested_point( norm_angle_zero( atan2( points[3][1]-center[1], points[3][0]-center[0] ) ) );
                }
                else
                {
                    points[3].set_valid( false );
                }
            }
            
            seg_intersection( &lbnd, hold_points, true );
            if( hold_points[0].is_valid() )
            {
                points[4] = hold_points[0];
            }
            if( hold_points[1].is_valid() )
            {
                points[5] = hold_points[1];
            }
            
            seg_intersection( &rbnd, hold_points, true );
            if( hold_points[0].is_valid() )
            {
                points[6] = hold_points[0];
            }
            if( hold_points[1].is_valid() )
            {
                points[7] = hold_points[1];
            }
            
            double start_angle = norm_angle_zero( atan2( point_b[1], point_b[0] ) );
            
            bool in = ( point_b.norm() < outer && point_b.norm() > inner ) && ( rbnd.side( point_b.get_data() ) > 0 && lbnd.side( point_b.get_data() ) < 0 );
            
            std::vector< intersection_point_t* > sort_points;
            for( unsigned i=0; i<8; ++i )
            {
                if( points[i].is_valid() )
                {
                    points[i].set_index(0);
                    sort_points.push_back( &points[i] );
                }
            }
            
            foreach( intersection_point_t* pt, sort_points )
            {
                double val = pt->get_sort_value();
                if( val < start_angle )
                    pt->set_interested_point( val + (2*PRX_PI) );
            }
            
            std::sort( sort_points.begin(), sort_points.end(), compare_intersection_point_t() );
            
            std::vector< bool > constraints;
            constraints.push_back( !in );
            
            std::vector< arc_t >hold_subs;
            hold_subs.push_back( seg );
            hold_subs.push_back( seg2 );
            
            sweep( sort_points, constraints, hold_subs, -2, 1 );
            
            seg = hold_subs[0];
            seg2 = hold_subs[1];
        }
        
        int arc_t::sweep( const std::vector< intersection_point_t* >& int_points, std::vector< bool >& violated_constraints, std::vector< arc_t >& hold_subsegments, int index, int delimeter ) const
        {
            //    PRX_LOG_WARNING("= = = Performing Arc Sweep = = =");
            //    for( unsigned i=0; i<int_points.size(); ++i )
            //    {
            //        PRX_LOG_INFO("[%d] : (%f, %f)", i, int_points[i]->at(0), int_points[i]->at(1));
            //    }
            
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
                //If the number of constraints we're violating is zero
                if( counter == 0 )
                {
                    //Add the begining point as a valid endpoint
                    endpts.push_back( -1 );
                    //endpts[nr_endpts] = -1;
                    ++nr_endpts;
                    out = true;
                }
                else
                    out = false;
                
                for( unsigned int i=0; i<int_points.size(); ++i )
                {
                    //Get the index of the line which caused this intersection
                    const int& pt_index = int_points[i]->get_index();
                    //            PRX_LOG_ERROR ("arc sweep: %i", pt_index);
                    
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
                            PRX_FATAL_S("arc_t :: The counter is not 1 when we expect it!");
                        }
                        //Realize we have ended up in a VO
                        out = false;
                        endpts.push_back( i );
                        //endpts[nr_endpts] = i;
                        ++nr_endpts;
                    }
                    else
                    { 
                        if( counter == 0 )
                        {
                            out = true;
                            endpts.push_back( i );
                            //endpts[nr_endpts] = i;
                            ++nr_endpts;
                        }
                    }
                }
                
                //Check the endpoint
                if( out )
                {
                    endpts.push_back( PRX_INFINITY );
                    //endpts[nr_endpts] = PRX_INFINITY;
                    ++nr_endpts;
                }
                
                if( nr_endpts%2 != 0 )
                    PRX_FATAL_S("Calculated endpoints are odd!  Clearly something wrong!");
                
                for( int k=0; k<nr_endpts; k+=2 )
                {
                    if( endpts[k] == -1 )
                        get_point_b( hold_point );
                    else
                        hold_point = (int_points[endpts[k]])->get_data();
                    
                    if( endpts[k+1] == PRX_INFINITY )
                        get_point_a( hold_point2 );
                    else
                        hold_point2 = (int_points[endpts[k+1]])->get_data();
                    
                    hold_subsegments[k/2].set_point_b( hold_point );
                    hold_subsegments[k/2].set_point_a( hold_point2 );
                    hold_subsegments[k/2].set_point_c( center );
                    hold_subsegments[k/2].set_valid( true );
                    hold_subsegments[k/2].calc_rad();
                }
            }
            return nr_endpts / 2;
        }
        
        double arc_t::get_length() const
        {
            check_calc();
            return rad*(get_swept_angle());
        }
        
        void arc_t::check_calc() const
        {
            if( !calculated )
            {
                PRX_WARN_S("arc_t::calculated => Radius for the arc has not been calculated, use calc_rad() ");
            }
        }
        
        bool arc_t::on_ray( const vector_t& point ) const
        {
            const vector_t& v = point - center.get_data();
            return fabs( rad - v.norm() ) < PRX_DISTANCE_CHECK;
        }
        
        double arc_t::magnitude() const
        {
            return get_swept_angle()*rad;
        }
        
        double arc_t::sq_magnitude() const
        {
            const double& angle = get_swept_angle();
            return angle*angle*rad*rad;
        }
        
        int arc_t::seg_intersection( const two_point_primitive_t* other, std::vector< intersection_point_t >& ret ) const
        {
            check_calc();
            const line_t* line = dynamic_cast< const line_t* > (other);
            if( line )
                return seg_intersection( line, ret );
            return -1;
        }
        
        int arc_t::seg_intersection( const line_t* other, std::vector< intersection_point_t >& ret, bool utype ) const
        {
            check_calc();
            
            int pts = intersection( other, ret );
            int amt = pts;
            
            double val = 0;
            double dx = 0, dy = 0;
            
            for( int i=0; i<amt; ++i )
            {
                //First, find ua of the point
                dir_a = point_a.get_data(); 
                dir_a -= center.get_data();
                dir_b = ret[i].get_data();
                dir_b -= center.get_data();
                double angle = norm_angle_zero( atan2( dir_b[1], dir_b[0] ) );
                double s_angle = norm_angle_zero( atan2( dir_a[1], dir_a[0] ) );
                
                //U values between 0 and 1 !=!=!=! THIS IS POSSIBLY BUGGY !=!=!=!
                if( utype == false )
                {
                    val = swept_angle( s_angle, angle )/get_swept_angle();
                    ret[i].set_ua( val );
                    if( val > 1 || val < 0 )
                    {
                        --pts;
                        ret[i].set_valid( false );
                    }
                }
                else
                {
                    val = norm_angle_zero( atan2( ret[i][1], ret[i][0] ) );
                    double angle_a = norm_angle_zero( atan2( point_a[1], point_a[0] ) );
                    double angle_b = norm_angle_zero( atan2( point_b[1], point_b[0] ) );
                    ret[i].set_ua( val );
                    if( angle_a < angle_b )
                    {
                        if( val > angle_a && val < angle_b )
                        {
                            --pts;
                            ret[i].set_valid( false );
                        }
                    }
                    else if( angle_a > angle_b )
                    {
                        if( val > angle_a || val < angle_b )
                        {
                            --pts;
                            ret[i].set_valid( false );
                        }                
                    }
                }
                
                //If the point is even valid, let's look at ub
                double ax = (*(other->get_point_a()))[0];
                double bx = (*(other->get_point_b()))[0];
                double px = ret[i][0];
                if( ax < bx )
                {
                    dx = bx - ax;
                    val = px - ax;
                    val /= dx;
                    ret[i].set_ub( val );
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
                    ret[i].set_ub( val );
                    if( val > 1 || val < 0 )
                    {
                        --pts;
                        ret[i].set_valid( false );
                    }
                }
                else
                {
                    double ay = (*(other->get_point_a()))[1];
                    double by = (*(other->get_point_b()))[1];
                    double py = ret[i][1];
                    if( ay < by )
                    {
                        dy = by - ay;
                        val = py - ay;
                        val /= dy;
                        ret[i].set_ub( val );
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
                        ret[i].set_ub( val );
                        if( val > 1 || val < 0 )
                        {
                            --pts;
                            ret[i].set_valid( false );
                        }
                    }
                    else
                    {
                        //                PRX_LOG_WARNING ("Same two points?!");
                        ret[i].set_valid(false);
                    }
                }
            }
            
            return pts;
        }
        
        int arc_t::seg_intersection( const arc_t* other, std::vector< intersection_point_t >& ret, bool utype ) const
        {
            check_calc();
            
            int num_intersections = intersection( other, ret );
            int ret_val = 0;
            for (int i = 0; i < num_intersections; i++)
            {
                //NEED TO SET THE U VALUES!!
                if( utype == false )
                {
                    ret[i].set_ua( swept_angle( atan2( point_a[1]-center[1], point_a[0]-center[0] ), atan2( ret[i].get_data()[1]-center[1], ret[i].get_data()[0]-center[0] ) ) / get_swept_angle() );
                    ret[i].set_ub( swept_angle( atan2( other->point_a[1]-other->center[1], other->point_a[0]-other->center[0] ), atan2( ret[i].get_data()[1]-other->center[1], ret[i].get_data()[0]-other->center[0] ) ) / other->get_swept_angle() );
                }
                else
                {
                    ret[i].set_ua( norm_angle_zero( atan2( ret[i].get_data()[1]-center[1], ret[i].get_data()[0]-center[0] ) ) );
                    ret[i].set_ub( norm_angle_zero( atan2( ret[i].get_data()[1]-other->center[1], ret[i].get_data()[0]-other->center[0] ) ) );
                }
                if (point_on_seg(ret[i].get_data()) && other->point_on_seg(ret[i].get_data()))
                {
                    ret[i].set_valid(true);
                    ret_val++;
                }
                else
                {
                    ret[i].set_valid(false);
                }
            }
            return ret_val;
        }
        
        double arc_t::seg_get_closest_point( const vector_t& point, vector_t& closest_point ) const
        {
            check_calc();
            
            if( rad == 0 )
            {
                closest_point = point_a.get_data();
                return point.norm();
            }
            vector_t& pt = project( point );
            double proj_angle = norm_angle_zero( atan2( pt[1], pt[0] ) );
            double ub_angle = norm_angle_zero( atan2( point_a[1], point_a[0] ) );
            double lb_angle = norm_angle_zero( ub_angle - get_swept_angle() );
            
            double da=0, db=0;
            
            if( lb_angle == ub_angle )
            {
                closest_point = pt;
            }
            else if( lb_angle > ub_angle )
            {
                if( proj_angle < ub_angle || proj_angle > lb_angle )
                {
                    //Use intersection point
                    closest_point = pt;
                }
                else
                {
                    //Find point a or point b which is closer
                    da = minimum_angle_with_orientation( proj_angle, ub_angle );
                    db = minimum_angle_with_orientation( proj_angle, lb_angle );
                    if( da < db )
                        closest_point = point_a.get_data();
                    else
                        closest_point = point_b.get_data();
                }
            }
            else
            {
                if( proj_angle < ub_angle && proj_angle > lb_angle )
                {
                    closest_point = pt;
                }
                else
                {
                    da = minimum_angle_with_orientation( proj_angle, ub_angle );
                    db = minimum_angle_with_orientation( proj_angle, lb_angle );
                    if( da < db )
                        closest_point = point_a.get_data();
                    else
                        closest_point = point_b.get_data();
                }
            }
            
            return closest_point.distance( point );
        }
        
        bool arc_t::point_on_seg ( const vector_t& point ) const
        {
            check_calc();
            
            if (fabs(side(point)) < PRX_ZERO_CHECK)
            {
                dir_a = point_a.get_data();
                dir_a -= center.get_data();
                dir_b = point;
                dir_b -= center.get_data();
                double angle = norm_angle_zero( atan2( dir_b[1], dir_b[0] ) );
                double s_angle = norm_angle_zero( atan2( dir_a[1], dir_a[0] ) );
                double val = swept_angle( s_angle, angle )/get_swept_angle();
                
                
                PRX_ASSERT(val > 0);
                if (val > 1 )
                {
                    return false;
                }
                else
                {
                    return true;
                }
                
            }
            else
                return false;
        }
        
        std::ostream& operator<<( std::ostream& out, const arc_t& arc )
        {
            out << "A: (" << arc.point_a[0] << ", " << arc.point_a[1] << "), (" << arc.point_b[0] << ", " << arc.point_b[1] << "), Cent:( " << arc.center[0] << ", " << arc.center[1] << ") : R(" << arc.rad << ") : A(" << arc.get_swept_angle() << ") : " << (arc.valid?"VAL":"INV");
            return out;
        }

    } 
}

