/**
 * @file intersection_point.cpp 
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

#include "prx/utilities/math/2d_geometry/intersection_point.hpp"

namespace prx 
{ 
    namespace util 
    {
        
        intersection_point_t::intersection_point_t( unsigned int dim ) : point( dim ) 
        {
            ua = ub = -1;
            interested_point = -1;
            valid = false;
        }
        
        intersection_point_t::intersection_point_t( const vector_t& vec )
        {
            point = vec;
            ua = ub = -1;
            interested_point = -1;
            valid = false;
        }
        
        intersection_point_t::~intersection_point_t()
        {
        }
        
        void intersection_point_t::resize( const unsigned int dim )
        {
            point.resize( dim );
        }
        
        intersection_point_t& intersection_point_t::operator= ( const vector_t& vec )
        {
            point = vec;
            ua = ub = -1;
            return *this;
        }
        
        intersection_point_t& intersection_point_t::operator= ( const intersection_point_t& pt )
        {
            point = pt.point;
            ua = pt.ua;
            ub = pt.ub;
            interested_point = pt.interested_point;
            valid = pt.valid;
            index = pt.index;
            return *this;
        }
        
        const double& intersection_point_t::operator[] ( int index ) const
        {
            return point[index]; 
        }
        
        double& intersection_point_t::operator[] ( int index )
        {
            return point[index];
        }
        
        const double& intersection_point_t::at ( int index ) const
        {
            return point[index];
        }
        
        double& intersection_point_t::at ( int index )
        {
            return point[index];
        }
        
        bool intersection_point_t::operator==( const intersection_point_t& other )
        {
            return ( point == other.point && index/2 == other.index/2 );
        }
        
        bool intersection_point_t::operator!=( const intersection_point_t& other )
        {
            return !(operator==( other ));
        }
        
        bool intersection_point_t::approximately_equal( const intersection_point_t& other )
        {
            return point.is_approximate_equal( other.get_data() );
        }
        
        bool intersection_point_t::operator< (const intersection_point_t& other)
        {
            return interested_point < other.interested_point;
        }
        
        intersection_point_t& intersection_point_t::operator+= ( const vector_t& v )
        {
            point += v;
            return *this;
        }
        
        intersection_point_t intersection_point_t::operator+ ( const vector_t& v )
        {
            return intersection_point_t( v );
        }
        
        const vector_t& intersection_point_t::get_data() const
        {
            return point;
        }
        
        double intersection_point_t::get_sort_value() const
        {
            return interested_point;
        }
        
        void intersection_point_t::set_data( const vector_t& vec )
        {
            point = vec;
            ua = ub = -1;
        }
        
        double intersection_point_t::get_ua() const
        {
            return ua;
        }
        
        void intersection_point_t::set_ua( double in_u )
        {
            ua = in_u;
        }
        
        double intersection_point_t::get_ub() const
        {
            return ub;
        }
        
        void intersection_point_t::set_ub( double in_u )
        {
            ub = in_u;
        }
        
        double intersection_point_t::norm() const
        {
            return point.norm();
        }
        
        double intersection_point_t::sq_magnitude() const
        {
            return point[0]*point[0] + point[1]*point[1];
        }
        
        void intersection_point_t::set_ua_interested_point()
        {
            interested_point = ua;
        }
        
        void intersection_point_t::set_ub_interested_point()
        {
            interested_point = ub;
        }
        
        void intersection_point_t::set_interested_point( double val )
        {
            interested_point = val;
        }
        
        void intersection_point_t::set_index(int ind)
        {
            index = ind;
        }
        
        int intersection_point_t::get_index() const
        {
            return index;
        }
        
        bool intersection_point_t::is_valid() const
        {
            return valid;
        }
        
        void intersection_point_t::set_valid( bool flag )
        {
            valid = flag;
        }
        
        std::ostream& operator<<( std::ostream& out, const intersection_point_t& point )
        {
            if( point.valid )
            {
                out << "(" << point.point[0] << "," << point.point[1] << ") sort_value: " << point.interested_point << " (" << point.ua << " | " << point.ub << ") index:|" << point.index << "|";
            }
            else
            {
                out << "(xx.xx,xx.xx) " << point.interested_point << " ";
            }
            return out;
        }
        
    } 
}

