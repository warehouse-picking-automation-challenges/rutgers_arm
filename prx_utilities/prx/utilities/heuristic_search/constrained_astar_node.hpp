/**
 * @file constrained_astar_node.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_CONSTRANED_ASTAR_NODE_HPP
#define PRX_CONSTRANED_ASTAR_NODE_HPP

#include "prx/utilities/heuristic_search/astar_node.hpp"
#include "prx/utilities/heuristic_search/constraints.hpp"

namespace prx
{
    namespace util
    {

        /**
         * A structure to represent a node with constraints on the A* open set. The main purpose 
         * is to provide a place to store vertices' f-values and some lightweight 
         * comparison, hashing functions and constraints. 
         * 
         * @brief <b> A structure to represent a node with constraints on the A* open set. </b>
         * 
         * @Author: Athanasios Krontiris
         */
        class constrained_astar_node_t: public astar_node_t
        {

          public:
            constrained_astar_node_t();

            constrained_astar_node_t(util::undirected_vertex_index_t vertex);

            constrained_astar_node_t(util::undirected_vertex_index_t vertex, double g, double h);

            constrained_astar_node_t(const constrained_astar_node_t & n);

            virtual ~constrained_astar_node_t();

            virtual const constrained_astar_node_t& operator=(const constrained_astar_node_t& other);

            virtual bool operator<(const astar_node_t & n) const;

            virtual operator unsigned() const
            {
                return *(unsigned int*)(&vertex);
            }

            virtual void add_constraints(const constraints_t* new_constraints);

            /**
             * @brief Checks if the node has constraints. 
             * @details Checks if the node has constraints. By default will check if the node has any level of constraints.
             * 
             * @param valid_constraints The valid constraints that will be checked with the current constraints.
             * @return True if the nodes has any kind of constraints, otherwise false. 
             */
            virtual bool has_constraints(const constraints_t* valid_constraints);

            virtual bool path_has_vertex(util::undirected_vertex_index_t v);

            /**
             * @brief Merges two astar nodes.
             * 
             * @details Merges two astar nodes. The current node will be the new node to be added in the open_set. The node
             * passed as argument will be the predecessor of the current node. This function will merge the constraints of 
             * the new nodes and the paths. The path will be the previous path + the current vertex. This function will keep
             * the f,h,g values the same as the merged node. If you want to change these values then use the function:
             * \c void merge(const constrained_astar_node_t* node, double g, double h)
             * 
             * @param node The node that was already in the priority queue and is the predecessor of the current node. 
             */
            virtual void merge(const constrained_astar_node_t* node);

            /**
             * @brief Merges two astar nodes.
             * 
             * @details Merges two astar nodes. The current node will be the new node to be added in the open_set. The node
             * passed as argument will be the predecessor of the current node. This function will merge the constraints of 
             * the new nodes and the paths. The path will be the previous path + the current vertex. This function will assign
             * the new f,h,g values.
             * 
             * @param node The node that was already in the priority queue and is the predecessor of the current node. 
             * @param g The g that the new node will have in the priority queue.
             * @param h The heuristic value for the new node. 
             */
            virtual void merge(const constrained_astar_node_t* node, double g, double h);

            std::string print_constraints() const;

            std::string print() const;

            /** @brief The position inside the vector represents level of the constraints. The 0 position is the most important soft constraints. */
            constraints_t* constraints;
            std::deque<util::undirected_vertex_index_t > path;
        };
    }
}
#endif	

