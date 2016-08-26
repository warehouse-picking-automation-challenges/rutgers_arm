/**
 * @file pmt_graph.hpp
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
#pragma once

#ifndef PRX_PMT_GRAPH_HPP
#define	PRX_PMT_GRAPH_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"
#include "utilities/pebble_assignment.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"

namespace prx
{
    namespace packages
    {
        namespace pebble_motion
        {

            /**
             * A brief description of this class (no tag required).
             * 
             * A more detailed description of this class with multiple paragraphs delimited by blank lines. 
             */
            class pmt_graph_node_t : public util::undirected_node_t
            {

              public:
                pmt_graph_node_t();
                virtual ~pmt_graph_node_t();

//                /** @copydoc abstract_node_t::serialize(std::ifstream& , const space_t*) */
//                virtual void serialize(std::ofstream& output_stream, const util::space_t* point_space);
//
//                /** @copydoc abstract_node_t::deserialize(std::ifstream& , const space_t*) */
//                virtual void deserialize(std::ifstream& input_stream, const util::space_t* point_space);

                unsigned long bfs_id; //For the BFS searches.
                unsigned long obstacle_id; //If we cannot use this vertex.
                unsigned long avoid_id; //If we can go through this vertex but we cannot stop.
                unsigned long visited_id; //For checks if we have visited or not this node.
                unsigned long on_the_path_branch_id; //To check if the branch is on the between path or not.
                bool is_branch; // boolean if it is branch or not.     
                int id;

                util::undirected_vertex_index_t v_from;

                int index_on_path;

                std::string print()
                {
                    std::stringstream out(std::stringstream::out);

                    out << id << ") bfs:" << bfs_id << "  obstacle_id:" << obstacle_id << "  visited:" << visited_id << "  avoid:" << avoid_id;
                    return out.str();
                }
            };

            typedef util::prx_astar_goal_visitor<util::undirected_graph_type, util::undirected_vertex_index_t> pmt_astar_visitor_t;

            class pmt_default_distance_heuristic_t : public boost::astar_heuristic<util::undirected_graph_type, double>
            {

              public:

                pmt_default_distance_heuristic_t(util::distance_metric_t* metric)
                {
                    distance_metric = metric;
                }

                pmt_default_distance_heuristic_t(const util::undirected_graph_t* g, util::distance_metric_t* metric, util::undirected_vertex_index_t goal, const pebble_assignment_t* assignment = NULL)
                {
                    inner_graph = g;
                    distance_metric = metric;
                    goal_node = g->operator[](goal);
                    assign = assignment;
                }

                void set_graph(const util::undirected_graph_t* g)
                {
                    inner_graph = g;
                }

                void set(util::undirected_vertex_index_t goal, const pebble_assignment_t* assignment = NULL)
                {
                    PRX_ASSERT(inner_graph != NULL);
                    goal_node = inner_graph->operator[](goal);
                    assign = assignment;
                }

                double operator()(Vertex u)
                {
                    if( distance_metric == NULL )
                        return 0;

                    if( assign != NULL && assign->has_robot_on(u) )
                        return PRX_INFINITY;
                    return distance_metric->distance_function(inner_graph->operator[](u)->point, goal_node->point);
                }



              private:
                util::undirected_node_t* goal_node;
                util::distance_metric_t* distance_metric;
                const pebble_assignment_t* assign;
                const util::undirected_graph_t * inner_graph;
            };


        }
    }
}

#endif
