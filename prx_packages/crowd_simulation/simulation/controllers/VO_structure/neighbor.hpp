/**
 * @file neighbor.hpp
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

#ifndef PRX_NEIGHBOR_HPP
#define PRX_NEIGHBOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/vector.hpp"
#include "prx/utilities/math/3d_geometry/geometry.hpp"

namespace prx
{
    namespace sim
    {

    }
    
    namespace packages
    {
        namespace crowd
        {
            /**
             * @anchor neighbor_t
             *
             * @author Andrew Dobson, Andrew Kimmel
             */
            class neighbor_t
            {
            protected:
                /** @brief The index of the agent that this neighbor correspond to.*/
                int agent_index; 
                /** @brief The triangle that the agent is inside*/
                int triangle_id;
                /** @brief */
                util::geometry_type n_geotype;
                /** @brief */
                double n_radius;
                /** @brief */
                util::vector_t n_center;
                /** @brief */
                util::vector_t n_velocity;
                /** @brief */
                int vo_index;
                /** @brief */
                bool reciprocity;
                /** @brief */
                bool valid;

                bool _obstacle;
                /** @brief */
                std::string color;
                /** @brief */
                std::string name;                

            public:
                neighbor_t();
                virtual ~neighbor_t();

                /**
                 *
                 */
                double get_radius();
                /**
                 *
                 */
                const util::vector_t& get_center() const;
                /**
                 *
                 */
                const util::vector_t& get_velocity() const;
                /**
                 *
                 */
                int get_vo_index() const;
                /**
                 *
                 */
                bool get_reciprocity() const;
                /**
                 *
                 */
                util::geometry_type get_geotype() const;
                /**
                 *
                 */
                bool is_valid() const;

                bool is_obstacle();
                /**
                 *
                 */
                std::string get_color() const;
                /**
                 *
                 */
                std::string get_name() const;

                /**
                 * @brief Returns the agent's index
                 * @details Returns the agent's index
                 * @return Returns the agent's index
                 */
                int get_agent_index() const;

                /**
                 * @brief Returns the triangle id that the agent is inside.
                 * @details Returns the triangle id that the agent is inside.
                 * @return Returns the triangle id that the agent is inside.
                 */
                int get_triangle_id() const;

                /**
                 * @brief Gets the data for the VO. 
                 * @details Gets the data for the VO.
                 * 
                 * @param x It will return the x coordinate of the center of the agent
                 * @param y It will return the y coordinate of the center of the agent
                 * @param v It will return the velocity of the agent.
                 * @param w It will return the steering angle of the agent.
                 */
                void get_vo_info(double& x, double& y, double& v, double& w);

                /**
                 *
                 */
                void update_velocity();

                /**
                 *
                 */
                void set_neighbor_geotype (util::geometry_type type);
                /**
                 *
                 */
                void set_neighbor_radius( double rad );
                /**
                 *
                 */
                void set_neighbor_center( const util::vector_t& vec );
                void set_neighbor_center(double x, double y);
                /**
                 *
                 */
                void set_neighbor_velocity( const util::vector_t& vec );
                void set_neighbor_velocity( double x, double y);
                /**
                 *
                 */
                void set_vo_index( int ind );
                /**
                 *
                 */
                void set_reciprocity( bool b );
                /**
                 *
                 */
                void set_valid( bool val );
                /**
                 *
                 */
                void set_color( std::string incolor );
                /**
                 *
                 */
                void set_name ( const std::string& new_name );

                /**
                 * @brief Sets the agent's index.
                 * @details Sets the agent's index.
                 * 
                 * @param agent_index  The index for the agent that this neighbor correspond to.
                 */
                void set_agent_index(int agent_index);

                /**
                 * @brief Sets the triangle id that the agent is inside.
                 * @details Sets the triangle id that the agent is inside.
                 * 
                 * @param triangle_id The triangle id that the agent is inside.
                 */
                void set_triangle_id(int triangle_id);

                void set_obstacle_marker( bool is_obst );

                void setup_neighbor(int agent_index, std::string pathname, util::geometry_type type, double radius, bool reciprocity, bool obstacle_marker);

                void update_data(int agent_index, int triangle_id, double x, double y, double v, double w);

                double distance(const util::vector_t& other);
            };
        }
    }
}

#endif


