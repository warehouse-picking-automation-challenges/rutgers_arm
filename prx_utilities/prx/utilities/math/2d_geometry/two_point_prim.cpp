/**
 * @file two_point_prim.cpp 
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

#include "prx/utilities/math/2d_geometry/two_point_prim.hpp"

namespace prx 
{ 
    namespace util 
    {
        
        two_point_primitive_t::two_point_primitive_t()
        {
        }
        
        two_point_primitive_t::~two_point_primitive_t()
        {
        }
        
        void two_point_primitive_t::set_points( const vector_t& a, const vector_t& b )
        {
            point_a = a;
            point_b = b;
        }
        
        void two_point_primitive_t::set_point_a( const vector_t& a )
        {
            point_a = a;
        }
        
        void two_point_primitive_t::set_point_a( const intersection_point_t& a )
        {
            point_a = a;
        }
        
        void two_point_primitive_t::set_point_b( const vector_t& b )
        {
            point_b = b;
        }
        
        void two_point_primitive_t::set_point_b( const intersection_point_t& b )
        {
            point_b = b;
        }
        
        void two_point_primitive_t::get_points( vector_t& a, vector_t& b ) const
        {
            a = point_a.get_data( );
            b = point_b.get_data( );
            //    a.get_data( point_a );
            //    b.get_data( point_b );
        }
        
        void two_point_primitive_t::get_points( intersection_point_t& a, intersection_point_t& b ) const
        {
            a = point_a;
            b = point_b;
        }
        
        void two_point_primitive_t::get_point_a( vector_t& a ) const
        {
            a = point_a.get_data( );
        }
        
        void two_point_primitive_t::get_point_b( vector_t& b ) const
        {
            b = point_b.get_data( );
        }
        
        const intersection_point_t* two_point_primitive_t::get_point_a( ) const
        {
            return &point_a;
        }
        
        intersection_point_t* two_point_primitive_t::get_point_a( )
        {
            return &point_a;
        }
        
        const intersection_point_t* two_point_primitive_t::get_point_b( ) const
        {
            return &point_b;
        }
        
        intersection_point_t* two_point_primitive_t::get_point_b( )
        {
            return &point_b;
        }
        
        bool two_point_primitive_t::is_valid() const
        {
            return valid;
        }
        
        void two_point_primitive_t::set_valid( bool flag )
        {
            valid = flag;
        }
        
        void two_point_primitive_t::validate()
        {
            valid = true;
        }
        
        void two_point_primitive_t::invalidate()
        {
            valid = false;
        }
        
    } 
}

