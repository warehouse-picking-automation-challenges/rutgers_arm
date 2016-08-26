/**
 * @file neighbor.cpp
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

#include "simulation/controllers/VO_structure/neighbor.hpp"

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {
        namespace crowd
        {

            neighbor_t::neighbor_t()
            {
                n_radius = 0;
                n_center.resize( 2 );
                n_velocity.resize( 2 );
                valid = true;
                _obstacle = false;
                agent_index = -1;
                triangle_id = -1;
            }

            neighbor_t::~neighbor_t()
            {
            }

            double neighbor_t::get_radius()
            {
                return n_radius;
            }

            const vector_t& neighbor_t::get_center() const
            {
                return n_center;
            }

            const vector_t& neighbor_t::get_velocity() const
            {
                return n_velocity;
            }

            int neighbor_t::get_vo_index() const
            {
                return vo_index;
            }

            bool neighbor_t::get_reciprocity() const
            {
                return reciprocity;
            }

            geometry_type neighbor_t::get_geotype() const
            {
                return n_geotype;
            }

            int neighbor_t::get_agent_index() const
            {
                return agent_index;
            }

            int neighbor_t::get_triangle_id() const
            {
                return triangle_id;
            }

            void neighbor_t::get_vo_info(double& x, double& y, double& v, double& w)
            {
                x = n_center[0];
                y = n_center[1];
                v = n_velocity[0];
                w = n_velocity[1];
            }

            bool neighbor_t::is_valid( ) const
            {
                return valid;
            }

            std::string neighbor_t::get_color( ) const
            {
                return color;
            }

            void neighbor_t::update_velocity()
            {

            }

            void neighbor_t::set_neighbor_geotype(geometry_type type)
            {
                n_geotype = type;
            }

            void neighbor_t::set_neighbor_radius( double rad )
            {
                n_radius = rad;
            }

            void neighbor_t::set_neighbor_center( const vector_t& vec )
            {
                n_center = vec;
            }

            void neighbor_t::set_neighbor_center( double x, double y)
            {
                n_center[0] = x;
                n_center[1] = y;
            }

            void neighbor_t::set_neighbor_velocity( const vector_t& vec )
            {
                n_velocity = vec;
            }

            void neighbor_t::set_neighbor_velocity( double x, double y)
            {
                n_velocity[0] = x;
                n_velocity[1] = y;
            }

            void neighbor_t::set_vo_index( int ind )
            {
                vo_index = ind;
            }

            void neighbor_t::set_reciprocity( bool b )
            {
                reciprocity = b;
            }

            void neighbor_t::set_valid( bool val )
            {
                valid = val;
            }

            void neighbor_t::set_color( std::string incolor )
            {
                color = incolor;
            }

            void neighbor_t::set_name( const std::string& new_name )
            {
                name = new_name;
            }

            void neighbor_t::set_agent_index(int agent_index)
            {
                this->agent_index = agent_index;
            }

            void neighbor_t::set_triangle_id(int triangle_id)
            {
                this->triangle_id = triangle_id;
            }

            std::string neighbor_t::get_name( ) const
            {
                return name;
            }

            void neighbor_t::set_obstacle_marker( bool is_obst )
            {
                _obstacle = is_obst;
            }

            bool neighbor_t::is_obstacle()
            {
                return _obstacle;
            }

            void neighbor_t::setup_neighbor(int agent_index, std::string pathname, util::geometry_type type, double radius, bool reciprocity, bool obstacle_marker)
            {
                set_agent_index(agent_index);
                set_name(pathname);
                set_neighbor_geotype(type);
                set_neighbor_radius(radius);
                set_reciprocity(reciprocity);
                set_obstacle_marker(obstacle_marker);
            }

            void neighbor_t::update_data(int agent_index, int triangle_id, double x, double y, double v, double w)
            {
                set_agent_index(agent_index);
                set_triangle_id(triangle_id);
                n_center[0] = x;
                n_center[1] = y;
                n_velocity[0] = v; 
                n_velocity[1] = w;
            }

            double neighbor_t::distance(const vector_t& other)
            {
                return fabs(n_center[0] - other[0]) + fabs(n_center[1] - other[1]);
            }
        }
    }
}


