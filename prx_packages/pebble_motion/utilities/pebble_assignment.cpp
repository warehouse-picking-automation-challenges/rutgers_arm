/**
 * @file pebble_assignment.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2012, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file idd LICENSE.
 *
 * @authors Athanasios Krontiris, Andrew Dobson, Andrew Kimmel, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */


#include "utilities/pebble_assignment.hpp"

#include <boost/range/adaptor/map.hpp> //adaptors

namespace prx
{
    using namespace util;
    
    namespace packages
    {
        namespace pebble_motion
        {

            pebble_assignment_t::pebble_assignment_t()
            {
                assign_size = 0;
            }
            
            pebble_assignment_t::~pebble_assignment_t()
            {
                clear();
            }

            void pebble_assignment_t::clear()
            {
                assignments.clear();
            }

            void pebble_assignment_t::add_assignment(undirected_vertex_index_t v, int robot_id)
            {
                assignments[v] = robot_id;
                ++assign_size;
            }

            void pebble_assignment_t::remove_assignment(undirected_vertex_index_t v)
            {
                assignments.erase(v);
                --assign_size;
            }

            void pebble_assignment_t::change_position(int robot_id, undirected_vertex_index_t v)
            {
                remove_assignment(get_position(robot_id));
                add_assignment(v, robot_id);
            }

            void pebble_assignment_t::change_position(undirected_vertex_index_t v, undirected_vertex_index_t u)
            {
                int pebble_id = get_robot(v);
                remove_assignment(v);
                add_assignment(u, pebble_id);
            }

            int pebble_assignment_t::get_robot(undirected_vertex_index_t v) const
            {
                if( has_robot_on(v) )
                    return assignments[v];
                //    PRX_WARN_S("Warning! vertex " << v << " does not have robot");
                return -1;
            }

            undirected_vertex_index_t pebble_assignment_t::get_position(int robot_id) const
            {

                foreach(undirected_vertex_index_t v, assignments | boost::adaptors::map_keys)
                {
                    if( assignments[v] == robot_id )
                        return v;
                }
                return NULL;
            }

            bool pebble_assignment_t::same_connected_components(undirected_graph_t* g, pebble_assignment_t& assign)
            {

                foreach(undirected_vertex_index_t v, assignments | boost::adaptors::map_keys)
                {
                    if( g->components[v] != g->components[assign.get_position(assignments[v])] )
                        return false;
                }
                return true;
            }

            bool pebble_assignment_t::has_same_robots(pebble_assignment_t& assign)
            {
                if( size() != assign.size() )
                    return false;

                foreach(int robot_id, assignments | boost::adaptors::map_values)
                {
                    if( !assign.has_robot(robot_id) )
                        return false;
                }
                return true;
            }

            size_t pebble_assignment_t::size() const
            {
                PRX_ASSERT(assignments.size() == assign_size);
                return assign_size;
            }

            bool pebble_assignment_t::is_empty(undirected_vertex_index_t v) const
            {
                return assignments.find(v) == assignments.end();
            }

            bool pebble_assignment_t::has_robot_on(undirected_vertex_index_t v) const
            {
                return assignments.find(v) != assignments.end();
            }

            bool pebble_assignment_t::has_robot(int robot_id) const
            {
                return pebble_assignment_t::get_position(robot_id) != NULL;
            }

            undirected_vertex_index_t pebble_assignment_t::get_first_position_with_robot()
            {

                foreach(undirected_vertex_index_t v, assignments | boost::adaptors::map_keys)
                {
                    return v;
                }
                return NULL;
            }

            pebble_assignment_t& pebble_assignment_t::operator =(const pebble_assignment_t& assign)
            {

                foreach(undirected_vertex_index_t v, assign.assignments | boost::adaptors::map_keys)
                {
                    int id = assign.assignments[v];
                    change_position(id, v);
                    assign_size = assign.size();
                }
                return *this;
            }

            bool pebble_assignment_t::operator ==(const pebble_assignment_t& assign) const
            {
                if( size() != assign.size() )
                    return false;

                foreach(int id, assignments | boost::adaptors::map_values)
                {
                    if( get_position(id) != assign.get_position(id) )
                        return false;
                }
                return true;
            }

            bool pebble_assignment_t::operator !=(const pebble_assignment_t& assign) const
            {
                if( size() != assign.size() )
                    return false;

                foreach(int id, assignments | boost::adaptors::map_values)
                {
                    if( get_position(id) != assign.get_position(id) )
                        return true;
                }
                return false;
            }

            int pebble_assignment_t::get_robot_in_place(int place)
            {
                int i = 0;

                foreach(int robot_id, assignments | boost::adaptors::map_values)
                {
                    if( i == place )
                    {
                        return robot_id;
                    }
                    i++;
                }
                PRX_LOG_ERROR("wrong place : %d", place);
                return -1;
            }

            void pebble_assignment_t::print() const
            {

                foreach(undirected_vertex_index_t v, assignments | boost::adaptors::map_keys)
                {
                    PRX_INFO_S("place: " << v << "  robot: " << assignments[v]);
                }
                PRX_INFO_S("---------------------");
            }

        }
    }
}
