/**
 * @file intersection_point.hpp 
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

#ifndef PRACSYS_INTERSECTION_POINT_HPP
#define PRACSYS_INTERSECTION_POINT_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/vector.hpp"

namespace prx 
{ 
    namespace util 
    {
        
        /**
         * This class is used to represent points of intersection between geometrical
         * primitives.  It stores information such as interpolation values and index
         * information to robustly handle procedures involving constraint checking.
         *
         * @brief <b> Mathematical Intersection Point class  </b>
         *
         * @author Andrew Dobson
         */
        class intersection_point_t
        {
            protected :
            /** @brief The internal storage for this intersection point's location. */
            vector_t point;
            /** @brief Primary geometry interpolation value. */
            double ua;
            /** @brief Secondary geometry interpolation value. */
            double ub;
            /** @brief Set to the interpolation value to be used in calculations. */
            double interested_point;
            /** @brief An index indicating from which constraint this intersection point originates. */
            int index;
            /** @brief A flag indicating whether the intersection point contains valid data. */
            bool valid;
            
            public :
            /**
             * @brief Parameterized constructor for intersection points with default size.
             *
             * @param dim The dimensionality of the point.  Default is 2.
             */
            intersection_point_t( unsigned int dim = 2 );
            /**
             * @brief Parameterized constructor building point from a given vector.
             *
             * @param vec Vector representing the point in space to build the point from.
             */
            intersection_point_t( const vector_t& vec );
            ~intersection_point_t();
            
            /**
             * Resizes the member \c point
             * 
             * @brief Resizes the member \c point
             * @param dim The dimension of the resizing
             */
            void resize( const unsigned int dim = 0 );
            
            /**
             * @brief Assignment operator for intersection points.
             *
             * @param vec Vector to assign values from.
             *
             * @return Self refrence for chaining assignments.
             */
            intersection_point_t& operator= ( const vector_t& vec );
            /**
             * @brief Assignment operator for intersection points.
             *
             * @param pt Other intersection point to copy.
             *
             * @return Self refrence for chaining assignments.
             */
            intersection_point_t& operator= ( const intersection_point_t& pt );
            
            /**
             * @brief Random access operator. (const version)
             *
             * @param index The index to access.
             *
             * @return Point value at index.
             */
            const double& operator[] ( int index ) const;
            /**
             * @brief Random access operator.
             *
             * @param index The index to access.
             *
             * @return Point value at index.
             */
            double& operator[] ( int index );
            /**
             * @brief Random access function for working with pointers. (const version)
             *
             * @param index The index to access.
             *
             * @return Point value at index.
             */
            const double& at ( int index ) const;
            /**
             * @brief Random access function for working with pointers.
             *
             * @param index The index to access.
             *
             * @return Point value at index.
             */
            double& at ( int index );
            
            /**
             * @brief Equivalence operator testing coordinates and index.
             *
             * @param other The other intersection point to test against.
             *
             * @return A flag indicating whether the points are the same.
             */
            bool operator==( const intersection_point_t& other );
            /**
             * @brief Non-equivalence operator testing coordinates and index.
             *
             * @param other The other intersection point to test against.
             *
             * @return A flag indicating whether the points are different.
             */
            bool operator!=( const intersection_point_t& other );
            /**
             * @brief Approximate equivalence test using coordinate information.
             *
             * @param other The other intersection point to test against.
             *
             * @return A flag indicating whether the points are the same.
             */
            bool approximately_equal( const intersection_point_t& other );
            
            /**
             * Intersection point less-than operator.  This is used for sorting points
             * along a geometric primitive.  The operator checks the interpolation value
             * for the points and returns the one with the smaller interpolation value.
             *
             * @brief Intersection point less-than operator.
             *
             * @param other The other intersection point to test.
             * 
             * @return Whether this intersection point has a smaller interpolation value than the other.
             */
            bool operator< (const intersection_point_t& other);
            
            /**
             * Intersection point increment operator which simply translates the 
             * coordinates of the point by the given vector.
             *
             * @brief Intersection point increment operator.
             *
             * @param v The vector by which to translate the point.
             *
             * @return Self reference for chaining operations.
             */
            intersection_point_t& operator+= ( const vector_t& v );
            /**
             * Intersection point plus operator.  Returns a point which is a copy of
             * itself translated by the given vector.
             *
             * @brief Intersection point addition operator.
             *
             * @param v The vector by which to translate the point.
             *
             * @return The translated intersection point.
             */
            intersection_point_t operator+ ( const vector_t& v );
            
            /**
             * Retrieves the member \c point
             * 
             * @brief Retrieves the member \c point
             * @return the class member \c point
             */
            const vector_t& get_data() const;
            
            /**
             * Retrieves the member \c interested_point
             * 
             * @brief Retrieves the member \c interested_point
             * @return Retrieves the member \c interested_point
             */
            double get_sort_value() const;
            
            /**
             * @brief Retrieves coordinate data from the intersection point.
             *
             * @param vec Reference vector which returns the coordinate data.
             */
            void set_data( const vector_t& vec );
            
            /**
             * @brief Get the primary interpolation value. 
             *
             * @return The primary interpolation value.
             */
            double get_ua() const;
            
            /**
             * @brief Set the primary interpolation value. 
             *
             * @param in_u Input interpolation value.
             */
            void set_ua( double in_u );
            
            /**
             * @brief Get the secondary interpolation value. 
             *
             * @return The secondary interpolation value.
             */
            double get_ub() const;
            
            /**
             * @brief Set the secondary interpolation value. 
             *
             * @param in_u Input interpolation value.
             */
            void set_ub( double in_u );
            
            /**
             * @brief Return the length of the vector from the origin to this point.
             * 
             * @return The norm of the vector from the origin to this point.
             */
            double norm() const;
            
            /**
             * @brief Return the squared length of the vector from the origin to this point.
             * 
             * @return The squared norm of the vector from the origin to this point.
             */
            double sq_magnitude() const;
            
            /**
             * @brief Set this point to use the primary interpolation value for calculations.
             */
            void set_ua_interested_point();
            
            /**
             * @brief Set this point to use the secondary interpolation value for calculations.
             */
            void set_ub_interested_point();
            
            /**
             * @brief Directly set the interpolation value to use for calculation. 
             *
             * @param val The value to set for interpolation value calculations.
             */
            void set_interested_point( double val );
            
            /**
             * @brief Associate an index to this intersection point.
             * 
             * @param ind The index value to associate to this point.
             */
            void set_index(int ind);
            
            /**
             * @brief Retrieve the index identifier for this point. 
             *
             * @return The index for this point.
             */
            int get_index() const;
            
            /**
             * @brief Determine if the intersection point is valid.
             *
             * @return Whether the intersection point contains valid data.
             */
            bool is_valid() const;
            
            /**
             * @brief Set the validity of this point.
             * 
             * @param flag Input flag indicating validity of the point.
             */
            void set_valid( bool flag );
        
            friend std::ostream& operator<<( std::ostream& out, const intersection_point_t& point );
        };
    } 
}

#endif





