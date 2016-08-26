/**
 * @file VO.hpp
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

#ifndef PRX_VO_HPP
#define PRX_VO_HPP

#include "prx/utilities/math/configurations/vector.hpp"
#include "prx/utilities/math/2d_geometry/line.hpp"
#include "prx/utilities/math/2d_geometry/arc.hpp"
#include "prx/utilities/math/3d_geometry/geometry.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/math/2d_geometry/angle.hpp"

namespace prx
{
    namespace sim
    {
        namespace simulation
        {
            extern double simulation_step;
        }
    }

    namespace packages
    {
        namespace crowd
        {

            /**
             * @anchor minkowski_body_t
             *
             * @author Andrew Kimmel
             */
            struct minkowski_body_t
            {
                minkowski_body_t(){}

                double x_offset;
                double y_offset;

                std::vector<util::vector_t> vertices;
                
                std::string print() const
                {
                    std::stringstream out(std::stringstream::out);
                    out << "\n=-= Minkowski Obstacle =-=\n";
                    for( unsigned i=0; i<vertices.size(); ++i )
                        out << "[" << vertices[i][0] << " , " << vertices[i][1] << "]\n";

                    return out.str();
                }
            };

            /**
             * @anchor VO_t
             *
             * This class represents a velocity obstacle, to be used by the \ref VO_controller_t
             * class for distributed collision avoidance.
             *
             * @brief Explicit Velocity Obstacle representation.
             *
             * @author Andrew Dobson
             */
            class VO_t
            {
              protected:
                    // VO source type
                    /** @brief Enumeration indicating the type of construction used for this VO. */
                enum VO_source_t
                {
                    PRX_VO_OBSTACLE = 0,
                    PRX_VO_SYSTEM = 1,
                    PRX_VO_LOCO = 2,
                    PRX_VO_HORIZON_OBSTACLE = 3,
                    PRX_VO_HORIZON_SYSTEM = 4,
                    PRX_VO_HORIZON_LOCO = 5
                };
                VO_source_t vo_source;

                    // Original VO representation
                    /** @brief The left leg of the VO. */
                util::line_t left;
                    /** @brief The right leg of the VO. */
                util::line_t right;
                    /** @brief Storage for a possible valid segment on the VO's left leg. */
                util::line_t left_seg;
                    /** @brief Storage for a possible valid segment on the VO's right leg. */
                util::line_t right_seg;
                    /** @brief Temporary storage for a velocity vector used during computation. */
                util::vector_t vel_b;
                    /** @brief Storage for a possible valid segment on the VO's left leg. */
                util::line_t left_seg2;
                    /** @brief Storage for a possible valid segment on the VO's right leg. */
                util::line_t right_seg2;

                    // Circles representation
                    /** @brief Time horizon for horizon-based VOs. */
                double      horizon;
                    /** @brief Radius of the resulting horizon-based arc. */
                double      horizon_radius;
                    /** @brief Centoid of the resulting horizon-based arc. */
                util::vector_t    horizon_center;
                    /** @brief TODO WAT */
                util::vector_t    horizon_direction;
                    /** @brief The arc used to represent the rounded-end of horizon-based VOs. */
                util::arc_t       horizon_arc;

                    /** @brief Storage for a possible valid segment on the VO's arc. */
                util::arc_t       arc_seg;
                    /** @brief Storage for a possible valid segment on the VO's arc. */
                util::arc_t       arc_seg2;
                    /** @brief TODO WAT */
                double velocity;
                    //Vectors for doing operations
                    /** @brief Vector used for computation, specifically for projections. */
                util::vector_t proj;
                    /** @brief */
                util::vector_t upper;
                    /** @brief */
                util::vector_t lower;
                
                util::vector_t pt_l;
                util::vector_t pt_r;
                util::vector_t pt_v;
                
                    /** @brief Vector used for computation, remains a zero vector */
                util::vector_t zero_vec;
                    /** @brief Temporary storage for an intersection point. */
                util::intersection_point_t ipoint;
                    /** @brief Extra line storage for computational purposes. */
                mutable util::line_t line;
                    /** @brief Storage for holding the agent's center. */
                util::vector_t center;
                    /** @brief */
                std::vector< std::vector< util::intersection_point_t > > intersection_points;
                    /** @brief */
                std::vector< util::intersection_point_t > check_points;
                    /** @brief */
                std::vector< util::intersection_point_t* > segment_points;
                    /** @brief */
                std::vector< util::intersection_point_t > hold_points;
                    /** @brief */
                std::vector< util::line_t > lines;
                    /** @brief */
                util::line_t center_mass;

                    /** @brief A flag indicating whether this VO should be printing its information. */
                bool printing;

                    /** @brief Index for the VO for the logical X coordinate. */
                static const unsigned int _X;
                    /** @brief Index for the VO for the logical Y coordinate. */
                static const unsigned int _Y;

                struct compare_intersection_point_t
                {
                    bool operator ()(util::intersection_point_t* a, util::intersection_point_t* b)
                    {
                        return *a < *b;
                    }
                };

            public:
                VO_t();
                /**
                 * @brief Parameterized constructor which manually sets vo points.
                 *
                 * @param l Point for the left leg.
                 * @param r Point for the right leg.
                 * @param v Point for the VO's apex point.
                 */
                VO_t( const util::vector_t& l, const util::vector_t& r, const util::vector_t& v );
                ~VO_t();

                /**
                 * @brief Set the points which define the VO.
                 *
                 * @param l Point for the left leg.
                 * @param r Point for the right leg.
                 * @param v Point for the VO's apex point.
                 */
                void set_points( const util::vector_t& l, const util::vector_t& r, const util::vector_t& v);
                /**
                 * @brief Set the left leg point.
                 *
                 * @param l Point for the left leg.
                 */
                void set_l( const util::vector_t& l );
                /**
                 * @brief Set the right leg point.
                 *
                 * @param r Point for the right leg.
                 */
                void set_r( const util::vector_t& r );
                /**
                 * @brief Set the apex point of the VO.
                 *
                 * @param v Point for the VO's apex.
                 */
                void set_v( const util::vector_t& v );
                /**
                 * @brief Retrieve the points which comprise the VO.
                 *
                 * @param l Reference vector for retrieving the left leg point.
                 * @param r Reference vector for retrieving the right leg point.
                 * @param v Reference vector for retrieving the apex point.
                 */
                void get_points( util::vector_t& l, util::vector_t& r, util::vector_t& v );
                /**
                 * @brief Retrieve the points which comprise the VO (const object version).
                 *
                 * @param l Reference vector for retrieving the left leg point.
                 * @param r Reference vector for retrieving the right leg point.
                 * @param v Reference vector for retrieving the apex point.
                 */
                void get_points( util::vector_t& l, util::vector_t& r, util::vector_t& v ) const;
                /**
                 * @brief Retrieve the point for the VO's left leg.
                 *
                 * @return The point for the left leg of the VO.
                 */
                const util::vector_t& get_l( );
                /**
                 * @brief Retrieve the point for the VO's right leg.
                 *
                 * @return The point for the right leg of the VO.
                 */
                const util::vector_t& get_r( );
                /**
                 * @brief Retrieve the point for the VO's apex.
                 *
                 * @return The point for the apex of the VO.
                 */
                const util::vector_t& get_v( );

                /**
                 * @brief Retrieve the left leg of the VO.
                 *
                 * @return The line representing the left leg of the VO.
                 */
                const util::line_t& get_left() const;
                /**
                 * @brief Retrieve the right leg of the VO.
                 *
                 * @return The line representing the right leg of the VO.
                 */
                const util::line_t& get_right() const;
                /**
                 * @brief Retrieve the first left subsegment computed for this VO (const version).
                 *
                 * @return The line representing the first left segment.
                 */
                const util::line_t& get_left_seg() const;
                /**
                 * @brief Retrieve the first left subsegment computed for this VO.
                 *
                 * @return The line representing the first left segment.
                 */
                util::line_t& get_left_seg();
                /**
                 * @brief Retrieve the first right subsegment computed for this VO (const version).
                 *
                 * @return The line representing the first right segment.
                 */
                const util::line_t& get_right_seg() const;
                /**
                 * @brief Retrieve the first right subsegment computed for this VO.
                 *
                 * @return The line representing the first right segment.
                 */
                util::line_t& get_right_seg();
                /**
                 * @brief Retrieve the second left subsegment computed for this VO (const version).
                 *
                 * @return The line representing the second left segment.
                 */
                const util::line_t& get_left_seg2() const;
                /**
                 * @brief Retrieve the second left subsegment computed for this VO.
                 *
                 * @return The line representing the second left segment.
                 */
                util::line_t& get_left_seg2();
                /**
                 * @brief Retrieve the second right subsegment computed for this VO (const version).
                 *
                 * @return The line representing the second right segment.
                 */
                const util::line_t& get_right_seg2() const;
                /**
                 * @brief Retrieve the second right subsegment computed for this VO.
                 *
                 * @return The line representing the second right segment.
                 */
                util::line_t& get_right_seg2();
                /**
                 * @brief Retrieve the first arc subsegment computed for this VO (const version).
                 *
                 * @return The line representing the first arc segment.
                 */
                const util::arc_t& get_arc_seg() const;
                /**
                 * @brief Retrieve the first arc subsegment computed for this VO.
                 *
                 * @return The line representing the first arc segment.
                 */
                util::arc_t& get_arc_seg();
                /**
                 * @brief Retrieve the second arc subsegment computed for this VO (const version).
                 *
                 * @return The line representing the second arc segment.
                 */
                const util::arc_t& get_arc_seg2() const;
                /**
                 * @brief Retrieve the second arc subsegment computed for this VO (const version).
                 *
                 * @return The line representing the second arc segment.
                 */
                util::arc_t& get_arc_seg2();

                /**
                 * @brief Retrieve the source for this constructed VO.
                 *
                 * @return The source type for this VO.
                 */
                int get_source_type() const;
                /**
                 * @brief Set the source type for this VO.
                 *
                 * @param type The source type to set.
                 */
                void set_source(int type);

                /**
                 * @brief Indicates whether the given point violates this VO.
                 *
                 * @param point The point to test.
                 * @param eps Epsilon value which slightly grows or shrinks the VO.
                 *
                 * @return A flag indicating whether the given point violates the VO.
                 */
                bool in_obstacle( const util::vector_t& point, double eps = 0 ) const;
                /**
                 * @brief VO expansion method.
                 *
                 * As part of the Hybrid Reciprocal Velocity Obstacle framework, the regular
                 * RVO is augmented with another VO.  This auxilary function assists in this
                 * process.
                 *
                 * @param vela The agent's velocity.
                 * @param other VO used to expand this current VO.
                 */
                void expand( const util::vector_t& vela, const VO_t& other );
                /**
                 * @brief Simple VO construction method.
                 *
                 * @param other_center The center point of the other agent.
                 * @param radius The combined radius of the two agents.
                 * @param velb The velocity of the other agent.
                 */
                void construct( const util::vector_t& other_center, double radius, const util::vector_t& velb );
                /**
                 * @brief Reciprocal VO construction method.
                 *
                 * @param other_center The center point of the other agent.
                 * @param radius The combined radius of the two agents.
                 * @param vela The velocity for the agent constructing the VO.
                 * @param velb The velocity of the other agent.
                 * @param ratio The reciprocity ratio between the agents.
                 */
                void rvo_construct( const util::vector_t& other_center, double radius, const util::vector_t& vela, const util::vector_t& velb, double ratio );
                /**
                 * @brief Hybrid Reciprocal VO construction method.
                 *
                 * @param other_center The center point of the other agent.
                 * @param radius The combined radius of the two agents.
                 * @param vela The velocity for the agent constructing the VO.
                 * @param velb The velocity of the other agent.
                 * @param ratio The reciprocity ratio between the agents.
                 */
                void hrvo_construct( const util::vector_t& other_center, double radius, const util::vector_t& vela, const util::vector_t& velb, double ratio );
                /**
                 * @brief Experimental GRVO construction method.
                 *
                 * @param other_center The center point of the other agent.
                 * @param radius The combined radius of the two agents.
                 * @param direction
                 */
                void corridor_construct( const util::vector_t& other_center, double radius, double direction );
                /**
                 * @brief Construction method used for static obstacles.
                 *
                 * @param obst Information of the obstacle this VO is constructed for.
                 * @param my_center The center of the agent.
                 * @param new_horizon Selected time horizon.
                 * @param max_vel The maximum velocity of the agent.
                 * @param minimally_invasive True: Chuples' construction. False: Traditional
                 */
                void obstacle_construct (const minkowski_body_t* obst, const util::vector_t& my_center, double new_horizon, double max_vel, bool minimally_invasive = false);
                /**
                 * ????
                 */
                std::vector< std::vector< util::intersection_point_t > >& intersect( VO_t* other );
                /**
                 * @brief Constructs possible segments given a maximum bounding radius.
                 *
                 * @param r The radius used to bound the segments.
                 */
                void construct_segments( double r );
                /**
                 * @brief ????
                 *
                 * @param r
                 * @param ll
                 * @param rl
                 * @param the_line
                 * @param pta
                 * @param ptb
                 */
                void find_arc_intersections( double r, util::line_t& ll, util::line_t& rl, util::line_t& the_line, util::intersection_point_t& pta, util::intersection_point_t& ptb );
                /**
                 *
                 */
                void construct_segments( double outer, double inner, util::line_t& lbnd, util::line_t& rbnd );
                /**
                 *
                 */
                void set_tangent_points( const util::vector_t& other_center, double radius );
                /**
                 *
                 */
                void set_horizon_points( const util::vector_t& pcenter, double radius );
                /**
                 *
                 */
                void adapt_center( util::vector_t& other_center, double radius );

                // Horizon VO
                /**
                 *
                 */
                VO_t (const double h_radius, const util::vector_t& h_center, const util::vector_t& h_direction, const util::arc_t& h_arc, const double _horizon=PRX_INFINITY);

                // Horizon VO access functions
                /**
                 *
                 */
                double get_horizon() const;
                /**
                 *
                 */
                double get_horizon_radius() const;
                /**
                 *
                 */
                const util::vector_t& get_horizon_center() const;
                /**
                 *
                 */
                const util::vector_t& get_horizon_direction() const;
                /**
                 *
                 */
                const util::arc_t& get_horizon_arc() const;
                /**
                 *
                 */
                util::arc_t& get_horizon_arc();

                // -- Construction functions --
                /**
                 *
                 */
                void horizon_VO_construct (const util::vector_t& relative_position, const util::vector_t& my_center, const double tau, const double radius,  const util::vector_t& velb);
                /**
                 *
                 */
                void horizon_RVO_construct(const util::vector_t& relative_position, const util::vector_t& my_center, const double tau, const double radius, const util::vector_t& vela, const util::vector_t& velb, const double ratio, bool print = false);
                /**
                 *
                 */
                void approximate_horizon_VO_construct(const util::vector_t& a1,const util::vector_t& a2,const util::vector_t& b1,const util::vector_t& b2);
                /**
                 *
                 */
                void LOCO_construct(const util::arc_t& LOCO_arc);

                //= Other =//
                /**
                 *
                 */
                void closest_tangent( const util::vector_t& cpoint, util::vector_t& dir, util::vector_t& u );

                /**
                 *
                 */
                void set_velocity (double v);
                /**
                 *
                 */
                void set_printing( bool toprint );
                /**
                 *
                 */
                bool is_printing( );
                /**
                 *
                 */
                std::string print() const;
                
                bool check_integrity() const;
            };
        }
    }
}

#endif // PRX_VO_HPP
