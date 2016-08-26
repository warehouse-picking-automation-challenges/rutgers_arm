/**
 * @file pebble_assignment.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2012, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Athanasios Krontiris, Andrew Dobson, Andrew Kimmel, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#ifndef PRX_PEBBLE_ASSIGNMENT_HPP
#define	PRX_PEBBLE_ASSIGNMENT_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"
#include "prx/utilities/boost/hash.hpp"

namespace prx
{
    namespace packages
    {
        namespace pebble_motion
        {

            /**
             * A class to hold the assignment of a pebble problem. 
             */

            class pebble_assignment_t
            {

              public:
                pebble_assignment_t();
                virtual ~pebble_assignment_t();
                void clear();

                void add_assignment(util::undirected_vertex_index_t v, int);
                void remove_assignment(util::undirected_vertex_index_t v);
                void change_position(int robot_id, util::undirected_vertex_index_t v);
                void change_position(util::undirected_vertex_index_t v, util::undirected_vertex_index_t u);
                //    void add_assignment(util::undirected_vertex_index_t v, bool start=true); //For testing
                int get_robot(util::undirected_vertex_index_t v) const;
                util::undirected_vertex_index_t get_position(int robot_id) const;

                bool same_connected_components(util::undirected_graph_t* g, pebble_assignment_t& assign);
                bool has_same_robots(pebble_assignment_t& assign);

                size_t size() const;
                bool is_empty(util::undirected_vertex_index_t v) const;
                bool has_robot_on(util::undirected_vertex_index_t v) const;
                bool has_robot(int robot_id) const;
                util::undirected_vertex_index_t get_first_position_with_robot();

                pebble_assignment_t& operator =(const pebble_assignment_t& assign);
                bool operator ==(const pebble_assignment_t& assign) const;
                bool operator !=(const pebble_assignment_t& assign) const;

                int get_robot_in_place(int place);
                void print() const;

                /** 
                 * util::undirected_vertex_index_t is the place that the robot has in this assignment 
                 * int is the id for the robot
                 */
                util::hash_t<util::undirected_vertex_index_t, int> assignments;

              private:
                size_t assign_size;
            };

        }
    }
}

#endif	// PRX_PEBBLE_ASSIGNMENT_HPP       