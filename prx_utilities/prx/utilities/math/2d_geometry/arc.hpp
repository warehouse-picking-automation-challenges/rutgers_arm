/**
 * @file arc.hpp 
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

#ifndef PRACSYS_ARC_HPP
#define PRACSYS_ARC_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/2d_geometry/two_point_prim.hpp"

namespace prx 
{ 
    namespace util 
    {
        
        class line_t;
        
        /**
         * This class represents a circular arc, defined by it's center, radius, and 
         * a start and end point.
         * 
         * @brief <b> Mathematical Arc class. </b>
         *
         * @authors Andrew Dobson
         */
        
        class arc_t : public two_point_primitive_t
        {
        protected:
            /** @brief The center point of the arc */
            intersection_point_t center;
            
            /** @brief Intersection point holder */
            mutable std::vector< intersection_point_t > points; 
            
            /** @brief Holder for doing internal line intersections */
            mutable std::vector< intersection_point_t > hold_points; 
            
            /** @brief radius of the arc */
            double rad;
            
            /** @brief whether the radius has been calculated or not*/
            bool calculated;
            
            /** @brief Temporary storage for a point. */
            mutable vector_t hold_point;
            /** @brief Temporary storage for a point. */
            mutable vector_t hold_point2;
            /** @brief Temporary storage for a vector. */
            mutable vector_t dir_a;
            /** @brief Temporary storage for a vector. */
            mutable vector_t dir_b;
            /** @brief More temporary storage for a vector. */
            mutable vector_t v;
            
            /**
             * @brief <b> Comparison struct for sorting points </b>
             *
             * @author Andrew Dobson
             */
            struct compare_intersection_point_t
            {
                /**
                 * @brief Comparison operator between intersection poitns.
                 *
                 * @param a The first point for comparison.
                 * @param b The second point for comparison.
                 *
                 * @return A flag indicating if a is less than b.
                 */
                bool operator ()(intersection_point_t* a, intersection_point_t* b)
                {
                    return *a < *b;
                }
            };
            
            /**
             * Throws an error if the arc has not been calculated
             * 
             * @brief Throws an error if the arc has not been calculated
             */
            void check_calc() const;
            
        public:
            arc_t();
            virtual ~arc_t();
            
            //= Access Functions =//
            
            /**
             * Sets the three points of the arc using three vector_t's
             * passed in as parameters.
             * 
             * @brief Sets the three points of the arc
             * @param a One of the points of the arc
             * @param b One of the points of the arc
             * @param c The center of the arc
             */
            void set_points( const vector_t& a, const vector_t& b, const vector_t& c );
            
            /**
             * Sets the center point of the arc from a vector_t
             * 
             * @brief Sets the center point of the arc
             * @param c The center of the arc
             */
            void set_point_c( const vector_t& c );
            
            /**
             * Sets the center point of the arc from an intersection_point_t
             * 
             * @brief Sets the center point of the arc
             * @param c The center of the arc
             */
            void set_point_c( const intersection_point_t& c );
            
            /**
             * Retrieves the three points of the arc by reference
             * 
             * @brief Retrieves the three points of the arc 
             * @param a One of the points of the arc
             * @param b One of the points of the arc
             * @param c The center point of the arc
             */
            void get_points( vector_t& a, vector_t& b, vector_t& c ) const;
            
            /**
             * Gets the center point of the arc by reference
             * 
             * @brief Gets the center point of the arc
             * @param c The center point of the arc
             */
            void get_point_c( vector_t& c ) const;
            
            /**
             * Retrieves the center point of the arc as a const intersection_point
             * 
             * @brief Retrieves the center point of the arc
             * @return The center point of the arc
             */
            const intersection_point_t* get_point_c( ) const;
            
            /**
             * Retrieves the center point of the arc as an intersection_point
             * 
             * @brief Retrieves the center point of the arc
             * @return The center point of the arc
             */
            intersection_point_t* get_point_c( );
            
            
            /**
             * Retrieves the radius of the arc
             * 
             * @brief Retrieves the radius of the arc
             * @return The radius of the arc
             */
            double get_rad() const;
            
            /**
             * Calculates the radius of the arc
             * 
             * @brief Calculates the radius of the arc
             */
            void calc_rad();
            
            //= Operators =//
            /**
             * @brief Arc class assignment operator.
             *
             * @param other The arc object to copy.
             *
             * @return Self reference for chaining assignments.
             */
            arc_t& operator=( const arc_t& other );
            
            //= Arc info Functions =//
            
            /**
             * Retrieves the swept angle of the arc.
             * 
             * @brief Retrieves the swept angle of the arc.
             * @return The swept angle of the arc.
             */
            double get_swept_angle() const;
            
            /**
             * Computes the swept angle between two angles A and B.
             * 
             * @brief Computes the swept angle between two angles A and B.
             * @param A An angle.
             * @param B An angle.
             * @return The swept angle between two angles.
             */
            double swept_angle( double A, double B ) const;
            
            /**
             * Retrieves the length of the arc
             * 
             * @brief Retrieves the length of the arc
             * @return The length of the arc
             */
            double get_length() const;
            
            //= Infinite Primitive Functions =//
            /**
             * Computes the intersection between this arc and a generic two_point_primitive.
             * 
             * The generic primitive can either be a line or an arc.
             * 
             * The intersection points are returned by reference in a std::vector.
             * 
             * @brief Computes the intersection between this arc and a generic two_point_primitive 
             * @param other The primitive to check intersection with
             * @param ret The intersection points
             * @return The number of intersection points
             */
            virtual int intersection( const two_point_primitive_t* other, std::vector< intersection_point_t >& ret ) const;
            
            /**
             * Computes the intersection between this arc and a line
             * 
             * @brief Computes the intersection between this arc and a line 
             * @param other The primitive to check intersection with
             * @param ret The intersection points
             * @return The number of intersection points
             */
            virtual int intersection( const line_t* other, std::vector< intersection_point_t >& ret ) const;
            
            /**
             * Computes the intersection between this arc and an arc
             * 
             * @brief Computes the intersection between this arc and an arc 
             * @param other The primitive to check intersection with
             * @param ret The intersection points
             * @return The number of intersection points
             */
            virtual int intersection( const arc_t* other, std::vector< intersection_point_t >& ret  ) const;
            /**
             * Computes a signed distance indicating which side a point lies relative to
             * the arc.  Negative values indicate the point is outside the arc, and
             * positive values indicate the point is inside the arc.
             *
             * @param point The point to test.
             *
             * @return A value for what side of the arc it is ( - outside ) ( + inside )
             */
            virtual double side( const vector_t& point ) const;
            /**
             * @copydoc two_point_primitive_t::project() const 
             */
            virtual vector_t& project( const vector_t& point ) const;
            
            //= Helper functions = //
            /**
             * TODO : circle_intersection function appears to not exist... at all?
             */
            int circle_intersection ( const arc_t* other, std::vector< intersection_point_t >& ret  ) const;
            
            /**
             * Given a set of constraints, this function constructs segments which lie
             * entirely within those contraints.  This region is bound by two lines and
             * two arcs, so it is possible to return up to two segments.  
             *
             * This function operates over the current arc assuming it is just a segment.
             *
             * @param outer Outer radius circular bound.
             * @param inner Inner radius circular bound.
             * @param lbnd Left-hand line bound.
             * @param rbnd Right-hand line bound.
             * @param seg The first segment constructed by the method.
             * @param seg2 The second segment constructed by the method.
             */
            void construct_segment( double outer, double inner, line_t& lbnd, line_t& rbnd, arc_t& seg, arc_t& seg2 ) const;
            
            /**
             * Perform a sweep over the arc.  Given a set of violated constraints and
             * intersection points, this function finds segments where no constraints
             * are violated.
             *
             * @param int_points The set of intersection points from other constraints.
             * @param violated_constraints A vector indicating which constraints are initially violated.
             * @param hold_subsegments The vector containing the returned set of subsegments.
             * @param index Apparently unused?  TODO : Fix this
             * @param delimeter Common divisor to determine which constraint imposes a particular intersection point.
             * @return The number of generated segments.
             */
            int sweep( const std::vector< intersection_point_t* >& int_points, std::vector< bool >& violated_constraints, std::vector< arc_t >& hold_subsegments, int index, int delimeter ) const;
            
            //= Ray Primitive Functions =//
            /**
             * @copydoc two_point_primitive_t::on_ray() const
             */
            virtual bool on_ray( const vector_t& point ) const;
            
            //= Segment Primitive Functions =//
            
            /** @copydoc two_point_prim::magnitude() */
            virtual double magnitude() const;
            
            /** @copydoc two_point_prim::sq_magnitude() */
            virtual double sq_magnitude() const;
            
            /**
             * @copydoc two_point_primitive_t::seg_intersection() const
             */
            virtual int seg_intersection( const two_point_primitive_t* other, std::vector< intersection_point_t >& ret ) const;
            
            /**
             * Perform intersection of this arc with a line, assuming they are both
             * segments.
             *
             * @param other The other line to intersect with.
             * @param ret The returned intersection points.
             * @param utype The type of u values to construct: (false : [0,1] | true : [0,2pi] )
             * 
             * @return The number of valid intersections found.
             */
            virtual int seg_intersection( const line_t* other, std::vector< intersection_point_t >& ret, bool utype = false ) const;
            
            /**
             * Perform intersection of this arc with another arc, assuming they are both
             * segments.
             *
             * @param other The segment to intersect with.
             * @param ret The returned intersection points.
             * @param utype The type of u values to construct: (false : [0,1] | true : [0,2pi] )
             * 
             * @return The number of valid intersections found.
             */
            virtual int seg_intersection( const arc_t* other, std::vector< intersection_point_t >& ret, bool utype = false ) const;
            
            /**
             * @copydoc two_point_primitive_t::seg_get_closest_point() const
             */
            virtual double seg_get_closest_point( const vector_t& point, vector_t& closest_point ) const;
            
            /**
             * @brief Report whether a point lies on the segment defined by this arc.
             *
             * @param point The point to test.
             *
             * @return A flag indicating whether the point lies on the segment.
             */
            virtual bool point_on_seg ( const vector_t& point ) const;

            friend std::ostream& operator<<( std::ostream& out, const arc_t& arc );
        };
    } 
}

#endif

