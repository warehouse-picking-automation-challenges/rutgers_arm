/**
 * @file two_point_prim.hpp 
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

#ifndef PRACSYS_TWO_POINT_PRIMITIVE_HPP
#define PRACSYS_TWO_POINT_PRIMITIVE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/2d_geometry/intersection_point.hpp"

namespace prx 
 { 
 namespace util 
 {

//TODO : The two-point primitive seems to have a fairly inconsistent interface.  Maybe unify the interface?

/**
 * The abstract class for all 2D geometries consisting of two  \ref intersction_point_t points
 * @brief <b> Abstract 2D geometry class </b>
 *
 * @authors Andrew Dobson
 */
class two_point_primitive_t
{
protected:
    /** @brief One of the points of the two-point primitive */
    intersection_point_t point_a;
    /** @brief One of the points of the two-point primitive */
    intersection_point_t point_b;
    /** @brief Whether or not this is a valid two-point primitive*/
    bool valid;
    
    /** @brief Contains intersecting points with this two-point primitive */
    std::vector< intersection_point_t > int_points;
    
public:
    two_point_primitive_t();
    virtual ~two_point_primitive_t();

    //= Access Functions =//
    /**
     * Sets the intersection points a and b
     * 
     * @brief Sets the intersection points a and b
     * @param a A vector_t containing values to set point_a with
     * @param b A vector_t containing values to set point_b with
     */
    void set_points( const vector_t& a, const vector_t& b );
    
    /**
     * Sets point_a from a vector_t
     * 
     * @brief Sets point_a
     * @param a vector_t containing values to set point_a with
     */
    void set_point_a( const vector_t& a );
    
    /**
     * Sets point_a from an intersection_point_t
     * 
     * @brief Sets point_a
     * @param a An intersection_point_t containing values to set point_a with
     */
    void set_point_a( const intersection_point_t& a );
    
    /**
     * Sets point_b from a vector_t
     * 
     * @brief Sets point_b
     * @param b vector_t containing values to set point_b with
     */
    void set_point_b( const vector_t& b );
    
    /**
     * Sets point_b from a intersection_point_t
     * 
     * @brief Sets point_b
     * @param b vector_t containing values to set point_b with
     */
    void set_point_b( const intersection_point_t& b );
    
    /**
     * Gets both point_a and point_b and stores them in vector_t
     * 
     * @brief Gets both point_a and point_b 
     * @param a A vector_t that contains a copy of point_a
     * @param b A vector_t that contains a copy of point_b
     */
    void get_points( vector_t& a, vector_t& b ) const;
    
    /**
     * Gets both point_a and point_b and stores them in intersection_point_t's
     * 
     * @brief Gets both point_a and point_b 
     * @param a An intersection_point_t containing a copy of point_a
     * @param b An intersection_point_t containing a copy of point_b
     */
    void get_points( intersection_point_t& a, intersection_point_t& b ) const;
    
    /**
     * Returns point_a as a vector_t
     * 
     * @brief Returns point_a as a vector_t
     * @param a A vector_t containing a copy of point_a
     */
    void get_point_a( vector_t& a ) const;
    
    /**
     * Returns point_a as a vector_t
     * 
     * @brief Returns point_b as a vector_t
     * @param a A vector_t containing a copy of point_b
     */
    void get_point_b( vector_t& b ) const;
    
    /**
     * Returns point_a as a const intersection_point_t pointer
     * 
     * @brief Returns point_a as an intersection_point_t pointer
     * @return A pointer to point_a
     */
    const intersection_point_t* get_point_a(  ) const;
    
    /**
     * Returns point_a as an intersection_point_t pointer
     * 
     * @brief Returns point_a as an intersection_point_t pointer
     * @return A pointer to point_a
     */
    intersection_point_t* get_point_a(  );
    
    /**
     * Returns point_a as a const intersection_point_t pointer
     * 
     * @brief Returns point_b as an intersection_point_t pointer
     * @return A pointer to point_b
     */
    const intersection_point_t* get_point_b(  ) const;
    
    /**
     * Returns point_b as an intersection_point_t pointer
     * 
     * @brief Returns point_b as an intersection_point_t pointer
     * @return A pointer to point_b
     */
    intersection_point_t* get_point_b(  );

    //= Validity funcitons =//
    /**
     * Checks the two_point_primitive's valid flag
     * 
     * @brief Checks the two_point_primitive's valid flag
     * @return True if the two_point_primitive is valid, false otherwise 
     */
    bool is_valid() const;
    
    /**
     * Sets the two_point_primtive's valid flag
     * 
     * @brief Sets the two_point_primtive's valid flag
     * @param flag Determines if the primitive is valid or not
     */
    void set_valid( bool flag );
    
    /**
     * Sets valid to true
     * 
     * @brief Sets valid to true
     */
    void validate();
    /**
     * Sets valid to false
     * 
     * @brief Sets valid to false
     */
    void invalidate();

    //= Infinite Primitive Functions =//
    /**
     * Checks this two_point_primitive for intersections with another two_point_primitive, and
     * returns the number of intersections.  The intersection points themselves are returned by 
     * reference in a std::vector
     * 
     * @brief Checks this two_point_primitive for any intersections with the other two_point_primitive
     * @param other The other primitive to check
     * @param ret The intersecting points
     * @return The number of intersecting points
     */
    virtual int intersection( const two_point_primitive_t* other, std::vector< intersection_point_t >& ret ) const = 0;
    
    /**
     * Checks the given point to determine on which "side" it lies relative to this
     * two-point primitive.
     *
     * @param point The point to be tested.
     *
     * @return A signed distance indicating which side the point is.
     */
    virtual double side( const vector_t& point ) const = 0;
    
    /**
     * Takes the given point and projects it onto the two-point primitive.
     *
     * @param point The point to project onto this primitive.
     *
     * @return The projected point.
     */
    virtual vector_t& project( const vector_t& point ) const = 0;
    
    //= Ray Primitive Functions =//
    /**
     * This function determines if the given point lies on the ray originating
     * from point_a moving in the direction of point_b.  Note that a ray for the
     * \ref arc_t class is always a complete circle.
     *
     * @param point The point to test.
     *
     * @return A flag indicating whether the point lies on the ray.
     */
    virtual bool on_ray( const vector_t& point ) const = 0;
    
    //= Segment Primitive Functions =//
    /**
     * Calculates the magnitude of the primitive
     * 
     * @brief Calculates the magnitude of the primitive
     * @return The magnitude 
     */
    virtual double magnitude() const = 0;
    
    /**
     * Calculates the squared magnitude of the primitive
     * 
     * @brief Calculates the squared magnitude of the primitive
     * @return The squared magnitude
     */
    virtual double sq_magnitude() const = 0;
    
    /**
     * Perform intersection of this primitive with another given primitive.  This
     * function treats both primitives as segments and only returns valid 
     * intersection points under this assumption.
     *
     * @param other The other two-point primitive to intersect with.
     * @param ret The returned intersection points.
     * 
     * @return The number of valid intersections found.
     */
    virtual int seg_intersection( const two_point_primitive_t* other, std::vector< intersection_point_t >& ret ) const = 0;

    /**
     * Treating the two-point primitive as a segment, this function finds the point
     * which lies on the segment and is closest to the given point.  The distance
     * used here is a simple Euclidean distance metric.
     *
     * @param point The point to test.
     * @param closest_point The closest point, returned by reference.
     *
     * @return The distance between the closest point found and the given point.
     */
    virtual double seg_get_closest_point( const vector_t& point, vector_t& closest_point ) const = 0;
};

} 
 }

#endif



