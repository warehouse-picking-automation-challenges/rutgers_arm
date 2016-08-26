/**
 * @file astar_search.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_ASTAR_SEARCH_HPP
#define PRX_ASTAR_SEARCH_HPP

//#include "prx/utilities/heuristic_search/default_open_set.hpp"
#include "prx/utilities/heuristic_search/astar_open_set.hpp"
#include "prx/utilities/heuristic_search/astar_node.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"
#include <boost/graph/properties.hpp>
#include <deque>

namespace prx
{
    namespace util
    {

        /**
         * @anchor astar_search_t
         *
         * Flexible implementation of the A* heuristic graph search algorithm. To use,
         * derive from this class and provide an implementation of the single-goal
         * heuristic function. The callback functions can also be overridden to give
         * fine-grained information and control over the search process.
         *
         * @brief <b> Flexible implementation of A* search. </b>
         *
         * @author Athanasios Krontiris
         */
        class astar_search_t
        {

          public:

            astar_search_t();
            astar_search_t(undirected_graph_t *g);
            virtual ~astar_search_t();

            /**
             * @brief Initializes all the parameters for the A*.
             * @details Initializes all the parameters for the A*.
             * 
             * @param reader An input reader with a dictionary of parameters for the planner.
             * @param template_reader A template reader with a dictionary of default parameters.
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

            /**
             * Sets the graph to be searched over.
             *
             * @brief Sets the graph to be searched over.
             *
             * @param g A pointer to the graph to search.
             */
            virtual void link_graph(undirected_graph_t *g);

            /**
             * Sets the distance function to use for the heuristic.
             *
             * @param f A pointer to the distance function
             */
            virtual void link_distance_function( distance_t f );

            /**
             * @brief Restarts the search.
             * @details Restarts the search. By default it just clears the 
             * blocked and trusted edges list.
             */
            virtual void restart();

            /**
             * Perform a query on the graph with multiple potential goals from multiple starts. The
             * first goal reached will be the one that terminates the search.
             *
             * @brief Search the graph and terminate as soon as any of the goals are found.
             *
             * @param starts The list of start vertices.
             * @param goals The list of goal vertices.
             * @return True if a path was discovered, false if not.
             */
            virtual bool solve(const std::vector<undirected_vertex_index_t>& starts, const std::vector<undirected_vertex_index_t>& goals );

            /**
             * Perform a query on the graph with multiple potential goals. The
             * first goal reached will be the one that terminates the search.
             *
             * @brief Search the graph and terminate as soon as any of the goals are found.
             *
             * @param start The start vertex.
             * @param goals The list of goal vertices.
             * @return True if a path was discovered, false if not.
             */
            virtual bool solve(undirected_vertex_index_t start, const std::vector<undirected_vertex_index_t>& goals );

            /**
             * Perform a query on the graph with a single goal. Optimized for this
             * case, so if there is only one goal, ideally this version should be used.
             *
             * @brief Search the graph and terminate as soon as the goal is found.
             *
             * @param start The start vertex.
             * @param goal The goal vertex.
             * @return True if a path was discovered, false if not.
             */
            virtual bool solve(undirected_vertex_index_t start, undirected_vertex_index_t goal );

            /**
             * @brief Set the length bound for bounded-length A*.
             */
            virtual void set_bounded_length ( double new_bounded_length );


            /**
             * @brief Will add this edge in the blocked_edges list.
             * @details Will add this edge in the blocked_edges list. If an edge it is blocked will not be
             * used again in the search. 
             * 
             * @param e The index of the edge that will be added in the blocked list.
             */
            virtual void block_edge(util::undirected_edge_index_t e);
            
            /**
             * @brief Will add this edge in the trusted_edge list.
             * @details Will add this edge in the trusted_edge list. If an edge is trusted, it will not be
             * repropagated during the search. 
             * 
             * @param e The index of the edge that will be added in the trusted list.
             */
            virtual void trust_edge(util::undirected_edge_index_t e);

            /**
             * @brief Reports whether the current search trusts the specified edge.
             *
             * @param e The edge to check.
             *
             * @return True iff the edge is trusted.
             */
            virtual bool is_edge_trusted( util::undirected_edge_index_t e );
            virtual bool is_edge_blocked( util::undirected_edge_index_t e );

            /**
             * Gets the goal vertex index found by the most recent search.
             * @brief Gets the goal vertex index found by the most recent search.
             * @return The index of the found goal vertex.
             */
            undirected_vertex_index_t get_found_goal() const;

            /**
             * @brief Returns the start vertex index that worked for the most recent search.
             * @details Returns the start vertex index that worked for the most recent search. This is
             * helpful when the search has multiple start points. 
             * 
             * @return The start vertex index that worked for the most recent search.
             */
            undirected_vertex_index_t get_start_vertex() const;

            /**
             * @brief Returns the path from the goal to the start.
             * @details Returns the path from the goal to the start. It will use the found goal and will get vertices up to the point that the predecessor
             * of a vertex is the same as the node, e.g. starting node. 
             * 
             * @param vertices A deque to store the path vertices in.
             * 
             * @return The vertex of the start node that worked. 
             */
            virtual undirected_vertex_index_t extract_path(std::deque<undirected_vertex_index_t>& vertices);

          protected:
            double bounded_length;

            undirected_graph_t* graph;            

            astar_open_set_t open_set;
            undirected_vertex_index_t found_goal;
            undirected_vertex_index_t found_start;
            astar_node_t* top_node;
            double new_node_distance;

            distance_t distance_function;

            hash_t<undirected_vertex_index_t, astar_node_t*> vertex_to_node;

            /** @brief The set of edge indices that correspond to blocked edges on the graph discovered during the search */
            std::vector<util::undirected_edge_index_t> blocked_edges;
            /** @brief The set of edge indices that correspond to trusted edges on the graph discovered during the search */
            std::vector<util::undirected_edge_index_t> trusted_edges;
            /** @brief The color of the vertex corresponds to whether the vertex has not been visited, has been visited, and removed from the queue*/
            static const boost::default_color_type WHITE, GRAY, BLACK;

            virtual astar_node_t* generate_new_node(undirected_vertex_index_t vertex, undirected_vertex_index_t parent, double g, double h);

             /**
              * @brief Called to initialize the starting vertices.
              * @details Called to initialize the starting vertices. It will update the vertex on the 
              * graph and it will also add a new node to the heap. 
              * 
              * @param vertex The starting vertex
              * @param parent The predecessor of the current vertex
              * @param g The current distance of this node.
              * @param h The heuristic for this node.
              */
            virtual void initialize_vertex(undirected_vertex_index_t vertex, undirected_vertex_index_t parent, double g, double h);

            /**
             * Called when a vertex is examined. Override this if you need to do any
             * special handling at that point.
             *
             * @brief Called when a vertex is examined.
             *
             * @param vertex The vertex being examined.
             */
            virtual bool examine_vertex(undirected_vertex_index_t vertex);

            /**
             * Called when a vertex is discovered. Override this if you need to do any
             * special handling at that point.
             *
             * @brief Called when a vertex is discovered.
             *
             * @param vertex The vertex being discovered.
             */
            virtual bool discover_vertex(undirected_vertex_index_t vertex, undirected_vertex_index_t parent, double new_distance, double h);

            /**
             * @brief Updates the vertex after all the checks are successful. 
             * @details Updates the vertex after all the checks are successful. This function will be called after it is 
             * found that we have to update the vertex. After that step if the vertex is new vertex will be also added in the 
             * open set.            
             * 
             * @param vertex The vertex on the graph that we will update
             * @param pred The predecessor of this vertex.
             * @param g The distance from the starting point. 
             */
            virtual void update_vertex(undirected_vertex_index_t vertex, undirected_vertex_index_t parent, double g);

            /**
             * Called when a vertex is finished. Override this if you need to do any
             * special handling at that point.
             *
             * @brief Called when a vertex is finished.
             *
             * @param vertex The vertex being finished.
             */
            virtual void finish_vertex(undirected_vertex_index_t vertex);

            /**
             * Called when an edge is examined. Override this if you need to do any
             * special handling at that point.
             *
             * @brief Called when a edge is examined.
             *
             * @param edge The edge being examined.
             */
            virtual bool examine_edge(undirected_vertex_index_t vertex, undirected_vertex_index_t parent, undirected_edge_index_t e, double new_distance);

                        /**
             * Called when we check a vertex in solve if it is the goal
             * 
             * @param potential_goal The vertex to check
             * @param actual_goal The goal vertex
             * @return True if vertex to check is the goal vertex, false otherwise
             */
            virtual bool check_if_goal_found(undirected_vertex_index_t potential_goal, const std::vector<undirected_vertex_index_t>& actual_goals);

            /**
             * @brief Sets the start and goal vertices for the search
             * @details Traces backwards from the goal vertex to find the start vertex. By default it assigns the final vertex to the found_goal variable.
             */
            virtual void finalize_search(undirected_vertex_index_t vertex);

            /**
             * @brief Will create and add new A* node inside the open set. 
             * @details Will create and add new A* node inside the open set. 
             *              
             * @param v The vertex index of this new node.
             * @param parent The predecessor of the new node.
             * @param g The distance of this node from the start.
             * @param h The heuristic value for this node.
             */
            virtual astar_node_t* openset_insert_node(undirected_vertex_index_t vertex, undirected_vertex_index_t parent, double g, double h);

            /**
             * The heuristic function. This estimates the remaining distance from
             * a vertex to the specified goal vertex. Should be overridden in derived
             * classes to guide the search.
             *
             * @brief Estimates the remaining distance to the specified goal.
             *
             * @param current The current vertex to calculate the heuristic for.
             * @param goal The goal vertex.
             * @return The estimated distance to the goal from the current vertex.
             */
            virtual double heuristic(undirected_vertex_index_t current, undirected_vertex_index_t goal);

            /**
             * Helper function to calculate the minimum heuristic value over a set of
             * multiple goals, i.e. the distance estimate for the closest goal to a point.
             *
             * @brief Estimates the distance to the closest goal from a vertex.
             *
             * @param current The current vertex to calculate the heuristic for.
             * @param goals The list of goal vertices.
             * @return The estimated distance to the closest goal from the current vertex.
             */
            virtual double heuristic(undirected_vertex_index_t current, const std::vector<undirected_vertex_index_t>& goals);


            /**
             * @brief Checks if the vertex v is in the same connected component with one of the vertices in the vector.
             * @details Checks if the vertex v is in the same connected component with one of the vertices in the vector.
             * If they have a common connected component then it is worth it to add this vertex in the open set. 
             * 
             * @param v The start vertex that will be checked.
             * @param vertices The final vertices. 
             * 
             * @return [description]
             */
            virtual bool in_same_connected_component(const undirected_vertex_index_t vertex, const std::vector<undirected_vertex_index_t>& vertices);

            unsigned long search_id;
            unsigned long lazy_search_id;
        };
    }
}



#endif
