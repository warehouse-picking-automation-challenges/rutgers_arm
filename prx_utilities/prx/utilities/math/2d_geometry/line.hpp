/**
 * @file line.hpp 
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

#ifndef PRACSYS_LINE_HPP
#define PRACSYS_LINE_HPP

#include "prx/utilities/math/2d_geometry/two_point_prim.hpp"
#include "prx/utilities/math/configurations/vector.hpp"
#include "prx/utilities/definitions/defs.hpp"

namespace prx 
{ 
    namespace util 
    {
        
        //Forward class declaration : arc_t
        class arc_t;
        
        /**
         * This class represents a line in euclidean space.  It's an extension of
         * the \ref two_point_primitive_t.
         *
         * @brief <b> Mathematical Line class  </b>
         *
         * @author Andrew Dobson
         */
        class line_t : public two_point_primitive_t
        {
            protected :
            /** @brief Temporary storage for point information. */
            mutable vector_t hold_point;
            /** @brief Temporary storage for point information. */
            mutable vector_t hold_point2;
            /** @brief Temporary storage for point information. */
            mutable vector_t hold_point3;
            /** @brief Temporary storage for vector information. */
            mutable vector_t dir_a;
            /** @brief Temporary storage for vector information. */
            mutable vector_t dir_b;
            /** @brief Temporary storage for intersection points when computing segments. */
            std::vector< intersection_point_t > hold_points;
            /** @brief Points used during segment construction. */
            std::vector< intersection_point_t > check_points;
            /** @brief Points which define the endpoints of valid subsegments of the line. */
            std::vector< intersection_point_t* > segment_points;
            /** @brief A zero vector kept around for quick computation. */
            vector_t zero_vec;
            
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
            
            public :
            line_t();
            /**
             * @brief Parameterized line class constructor.
             *
             * @param a The origin point of the line.
             * @param b The second point defining the line.
             * @param inv A flag indicating whether this line should be considered valid or not.
             */
            line_t( const vector_t& a, const vector_t& b, bool inv = true );    
            virtual ~line_t();
            
            //= Access Functions =//
            /**
             * Copies over the two points from a source line into
             * the current line.
             * 
             * @brief Constructs a line using another line.
             * @param other Line to copy information from.
             */
            void set_points_from_line( const line_t& other );
            
            /**
             * Creates a line using an angle.
             * 
             * @brief Sets a line from an angle.
             * @param angle The angle used to set the line.
             */
            void set_from_angle( double& angle );
            
            /**
             * @brief Swaps point_a and point_b.
             */
            void swap();
            
            //= Utility Functions =//
            /**
             * For a given circle, this function computes the line as one which is 
             * tangent to that circle, and passing through the origin.
             *
             * @brief Compute this line to be tangent to a circle.
             *
             * @param center The center of the circle to calculate a tangent for.
             * @param radius The radius of the circle to calculate the tangent for.
             * @param side Whether to build the left or right tangent to the circle.
             */
            void calculate_tangent( const vector_t& center, double radius, bool side );
            
            /**
             * @brief Given two intersecting lines, compute the angular bisector of these lines. 
             *
             * @param left The left line.
             * @param right The right line.
             */
            void bisect( const line_t& left, const line_t& right );
            
            /**
             * @brief Translate the line by the given vector. 
             *
             * @param vec The translation vector.
             */
            void translate( const vector_t& vec );
            
            /**
             * Rotate the line by the given angle.
             * 
             * @brief Rotates this line using an angle.
             * @param angle The angle the line is rotated by.
             */
            void rotate( double angle );
            
            /**
             * Computes the normalized direction of the line.
             * 
             * @brief Computes the normalized direction of the line.
             * @return The normalized direction of the line.
             */
            vector_t& get_normalized_direction( ) const;
            
            //= Infinite Line Functions =//
            /** @copydoc two_point_prim::intersection( const two_point_primitive_t* other, std::vector< intersection_point_t >& ret ) */
            int intersection( const two_point_primitive_t* other, std::vector< intersection_point_t >& ret ) const;
            
            /**
             * @brief Computes the intersection points between this line and another line.
             * @param other The other line used to compute intersections.
             * @param ret Contains the intersecting points between the two lines.
             * @return The number of intersection points.
             */
            int intersection( const line_t* other, std::vector< intersection_point_t >& ret ) const;
            
            /**
             * @brief Computes the intersection points between this line and an arc.
             * @param other The arc used to compute intersections.
             * @param ret Contains the intersecting points between this line and the arc.
             * @return The number of intersection points.
             */
            int intersection( const arc_t* other, std::vector< intersection_point_t >& ret ) const;
            
            /**
             * Returns a signed distance indicating on which side of the line a point lies.
             * Traversing the line from point_a to point_b, a point which lies on the right
             * side of the line returns a negative distance, while a point on the left
             * returns a positive distance.
             *
             * @brief Get a signed distance indicating what side of the line a point is.
             *
             * @param point The point to check.
             * @return A signed distance from the line, where a negative value indicates the point is on the right.
             */
            double side( const vector_t& point ) const;
            
            /**
             * @copydoc two_point_primitive_t::project()
             */
            vector_t& project( const vector_t& point ) const;
            
            /**
             * Given a set of constraints, this function constructs segments which lie
             * entirely within those contraints.  This region is bound by two lines and
             * two arcs, so it is possible to return up to two segments.  
             *
             * This function operates over the current line assuming it is just a segment.
             *
             * @param outer Outer radius circular bound.
             * @param inner Inner radius circular bound.
             * @param lbnd Left-hand line bound.
             * @param rbnd Right-hand line bound.
             * @param segment The first segment constructed by the method.
             * @param segment2 The second segment constructed by the method.
             */
            void construct_segment( double outer, double inner, line_t& lbnd, line_t& rbnd, line_t& segment, line_t& segment2 );
            
            /**
             * Given a set of constraints, this function constructs segments which lie
             * entirely within those contraints.  This region is bound by two lines and
             * two arcs, so it is possible to return up to two segments.  
             *
             * This function operates over the entire, infinite length line.
             *
             * @param outer Outer radius circular bound.
             * @param inner Inner radius circular bound.
             * @param lbnd Left-hand line bound.
             * @param rbnd Right-hand line bound.
             * @param segment The first segment constructed by the method.
             * @param segment2 The second segment constructed by the method.
             */
            void inf_construct_segment( double outer, double inner, line_t& lbnd, line_t& rbnd, line_t& segment, line_t& segment2 );
            
            /**
             * Perform a sweep over this line.  Given a set of violated constraints and
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
            int sweep( const std::vector< intersection_point_t* >& int_points, std::vector< bool >& violated_constraints, std::vector< line_t >& hold_subsegments, int index, int delimeter );    
            
            /**
             * Find the intersection points of this line (treated as a ray) with an arc
             * centered at the origin with radius r.  If any of the intersection points
             * lie outside the constraints imposed by ll and rl, they are invalidated.
             *
             * @param r Radius of the arc to test intersection with.
             * @param ll Left line constraint.
             * @param rl Right line constraint.
             * @param pta The first discovered intersection point.
             * @param ptb The second discovered intersection point.
             */
            void find_arc_intersections( double r, line_t& ll, line_t& rl, intersection_point_t& pta, intersection_point_t& ptb );
            
            /**
             * Find the intersection points of this line (infinite length) with an arc
             * centered at the origin with radius r.  If any of the intersection points
             * lie outside the constraints imposed by ll and rl, they are invalidated.
             *
             * @param r Radius of the arc to test intersection with.
             * @param ll Left line constraint.
             * @param rl Right line constraint.
             * @param pta The first discovered intersection point.
             * @param ptb The second discovered intersection point.
             */
            void find_inf_arc_intersections( double r, line_t& ll, line_t& rl, intersection_point_t& pta, intersection_point_t& ptb );
            
            /**
             * @copydoc two_point_primitive_t::on_ray() const
             */
            bool on_ray( const vector_t& point ) const;
            
            //= Segment Functions =//
            
            /** @copydoc two_point_primitive_t::magnitude() const */
            double magnitude( ) const;
            
            /** @copydoc two_point_primitive_t::sq_magnitude() const */
            double sq_magnitude( ) const;
            
            /**
             * @copydoc two_point_primitive_t::seg_intersection() const
             */
            int seg_intersection ( const two_point_primitive_t* other, std::vector< intersection_point_t >& int_point ) const;
            
            /**
             * Perform intersection of this line with another line, assuming they are both
             * segments.
             *
             * @param other The other line to intersect with.
             * @param ret The returned intersection points.
             * 
             * @return The number of valid intersections found.
             */
            int seg_intersection ( const line_t* other, std::vector< intersection_point_t >& int_point ) const;
            
            /**
             * Perform intersection of this line with an arc, assuming they are both
             * segments.
             *
             * @param other The segment to intersect with.
             * @param ret The returned intersection points.
             * 
             * @return The number of valid intersections found.
             */
            int seg_intersection ( const arc_t* other, std::vector< intersection_point_t >& int_point, bool utype = false ) const;
            
            /**
             * Perform intersection of this line with another line, assuming they are both
             * segments.  Performs extra checks under the assumption that some endpoints 
             * of these segments should be coincident.
             *
             * @param other The other line to intersect with.
             * @param ret The returned intersection points.
             * 
             * @return The number of valid intersections found.
             */
            int coincident_seg_intersection ( const line_t* other, std::vector< intersection_point_t >& int_point ) const;
            
            /**
             * Perform intersection of this line with an arc, assuming they are both
             * segments.  Performs extra checks under the assumption that some endpoints
             * of these segments should be coincident.
             *
             * @param other The segment to intersect with.
             * @param ret The returned intersection points.
             * 
             * @return The number of valid intersections found.
             */
            int coincident_seg_intersection ( const arc_t* other, std::vector< intersection_point_t >& ret, bool utype ) const;
            
            /**
             * Calculates the minimum distance between the given point and this line. 
             *
             * @param point The point to test.
             * @return The minimum distance between the line and the point.
             */
            double distance_from_point( const vector_t& point );
            
            /**
             * @copydoc two_point_primitive_t::seg_get_closest_point() const
             */
            double seg_get_closest_point( const vector_t& point, vector_t& closest_point ) const;
            
            /**
             * Computes the angle of the line.
             * 
             * @brief Computes the angle of the line.
             * @return The angle of the line.
             */
            double get_lines_angle() const;
            
            /**
             * Computes the slope of the line.
             * 
             * @brief Computes the slope of the line.
             * @return The slope of the line.
             */
            double get_lines_slope() const;
            
            /**
             * Computes the length of the line.
             * 
             * @brief Computes the length of the line.
             * @return The length of the line.
             */
            double get_length() const;
            
            friend std::ostream& operator<<(std::ostream&, const line_t&);
        };
    } 
}


#endif


