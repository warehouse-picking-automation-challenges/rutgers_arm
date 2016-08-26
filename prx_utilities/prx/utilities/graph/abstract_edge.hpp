/**
 * @file edge.hpp
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
#ifndef PRX_EDGE_HPP
#define	PRX_EDGE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/heuristic_search/constraints.hpp" 
#include <boost/graph/adjacency_list.hpp>
#include <fstream>

namespace prx
{
    namespace util
    {

        typedef boost::adjacency_list_traits< boost::listS, boost::listS, boost::directedS >::edge_descriptor directed_edge_index_t;
        typedef boost::adjacency_list_traits< boost::listS, boost::listS, boost::undirectedS >::edge_descriptor undirected_edge_index_t;

        /**
         * An abstract edge class for graph operations. Used to abstract out the particular 
         * requirements of motion planners so the same graph can be used for all of them.
         * 
         * @brief <b> An abstract edge class for graph operations. </b>
         */
        class abstract_edge_t
        {

          public:

            abstract_edge_t()
            { 
                constraints = NULL;
                search_id = 0;
                blocked_id = trusted_id = -1;
            }

            virtual ~abstract_edge_t()
            { 
                if(constraints != NULL)
                    delete constraints;
            }


            /**
             * @brief Node id's for the source and target of this edge.
             */
            int source_vertex, target_vertex;

            /**
             * Outputs relevant information to a stream.
             * @brief Outputs relevant information to a stream.
             * @param output_stream Where to output the information.
             */
            virtual void serialize(std::ofstream& output_stream)
            {
                output_stream << source_vertex << " " << target_vertex << " ";
            }

            /**
             * Inputs relevant information from a stream. Currently does nothing.
             * @brief Inputs relevant information from a stream.
             * @param input_stream Where to get the information.
             */
            virtual void deserialize(std::ifstream& input_stream, std::string soft_constraint_type = "")
            {
            }

            /**
             * @brief Keeps the soft constraints on this edge for the minimum constraint A*. 
             */
            constraints_t* constraints;

            /**
             * @brief It is used in order to avoid initializing all the edges of the graph, every time we start a new A* search.
             */
            unsigned long search_id;

            /**
             *  @brief Integer identifier of this node.
             */
            unsigned edge_id;
            
            /** @brief Blocked edge boolean **/
            int blocked_id;
            /** @brief Trusted edge boolean **/
            int trusted_id;

        };

    }
}

#endif

