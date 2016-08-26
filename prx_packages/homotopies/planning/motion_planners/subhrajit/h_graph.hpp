/**
 * @file h_graph.hpp
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

#ifndef PRX_H_GRAPH_HPP
#define	PRX_H_GRAPH_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/graph/directed_graph.hpp"
#include "prx/utilities/graph/directed_edge.hpp"
#include "prx/simulation/plan.hpp"
#include "prx/simulation/trajectory.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"

#include <complex>
#include <deque>

namespace prx
{

    namespace packages
    {

        namespace homotopies
        {

            typedef std::complex<double> complex_t;

            /** Obstacle marker functions */

            //complex_t calculate_edge_signatureX(complex_t z1, complex_t z2, const std::vector<complex_t>& rep_points,
            //                                    const std::vector<int>& k_vals, const std::vector<complex_t>& Al_vals)
            //{
            //    complex_t sum(0,0);
            //    for (int l = 0; l < rep_points.size(); l++)
            //    {
            //        complex_t diff2 = z2 - rep_points[l];
            //        complex_t diff1 = z1 - rep_points[l];
            //        complex_t left_inner = std::log(std::abs(diff2));
            //        left_inner -= std::log(std::abs(diff1));
            //        
            //        // Calculate right imaginary sum
            //        double right_inner_temp = std::arg(diff2) - std::arg(diff1);
            //        double lowest_product = PRX_INFINITY;
            //        // Find the smallest sum
            //        for (unsigned i = 0; i < k_vals.size(); i++)
            //        {
            //            double test_product = std::fabs(right_inner_temp + 2*k_vals[i]*PRX_PI);
            //            if (test_product < lowest_product)
            //            {
            //                lowest_product = test_product;
            //                
            //            }
            //        }
            ////        PRX_DEBUG_COLOR("Lowest product of: " << lowest_product, PRX_TEXT_BROWN);
            //        complex_t right_inner(0, lowest_product);
            //        sum += (Al_vals[l]*(left_inner + right_inner));
            //        
            //    }
            //    return sum;
            //}
            //
            //// ---- H-Graph Specifics
            //
            //complex_t calculate_H_signatureX(trajectory_t* path, const std::vector<complex_t>& rep_points,
            //                                const std::vector<int>& k_vals, const std::vector<complex_t>& Al_vals)
            //{
            //    complex_t edge_signature(0,0);
            //    if (path->size() > 1)
            //    {
            //        complex_t point1, point2;
            //        point1 = complex_t((*path)[0]->at(0), (*path)[0]->at(1));
            //        point2 = complex_t((*path)[1]->at(0), (*path)[1]->at(1));
            //
            //        edge_signature = calculate_edge_signatureX(point1, point2, rep_points, k_vals, Al_vals);
            //        for (int i = 1; i < path->size() - 1;  i++)
            //        {
            //            point1.real() = (*path)[i]->at(0); point1.imag() = (*path)[i]->at(1);
            //            point2.real() = (*path)[i+1]->at(0); point2.imag() = (*path)[i+1]->at(1);
            //            edge_signature += calculate_edge_signatureX(point1, point2, rep_points, k_vals, Al_vals);
            //        }
            //
            ////        PRX_DEBUG_COLOR("Calculated edge signature!", PRX_TEXT_MAGENTA);
            ////        PRX_DEBUG_COLOR("H-signature: " << edge_signature, PRX_TEXT_CYAN);
            //    }
            //    else
            //    {
            //        PRX_WARN_S ("Only one point in trajectory. No signature computed");
            //    }
            //    return edge_signature;
            //}

            /**
             */
            class h_node_t : public util::directed_node_t
            {

              public:

                ~h_node_t(){}

                /**
                 * @brief Initialize the node with required space and point information.
                 *
                 * @param space The state space this node exists in.
                 * @param new_point The point in the space where this node exsists.
                 */
                void init_node(const util::space_t* space, const util::space_point_t* new_point)
                {
                    PRX_ASSERT(space->get_dimension() >= 2);
                    state_space = space;
                    point = state_space->clone_point(new_point);
                    c_point.real() = point->at(0);
                    c_point.imag() = point->at(1);
                }

                // ---------

                std::vector<complex_t> h_values;
                complex_t c_point;

                /** @brief Pointer to the state space in which this point operates. */
                const util::space_t* state_space;
                /** @brief Vertex index which is a representative node of this node. (TODO : SPARS: deprecated, remove) */
                util::directed_vertex_index_t rep;
                /** @brief Cost associated with this vertex. */
                double cost;

            };

            /**
             */
            class h_edge_t : public util::directed_edge_t
            {
              public:

                h_edge_t(){ }

                virtual ~h_edge_t(){ }

                /**
                 * @brief Clear the edge, by clearing out its plan.
                 *
                 * @param space An unused space parameter. TODO : Remove this.
                 */
                virtual void clear(const util::space_t* space)
                {
                    plan.clear();
                }

                /**
                 * @brief Initialize this edge with data.
                 *
                 * @param space The space in which this edge operates.
                 * @param new_plan The plan which generated this edge.
                 * @param new_path The trajectory that represent this edge.
                 */
                virtual void init(const util::space_t* space, const sim::plan_t& new_plan, const sim::trajectory_t& new_path)
                {
                    plan.link_control_space(space);
                    plan = new_plan;                
                    path = new_path;
                }

                /**
                 * @brief Output edge information to a stream.
                 *
                 * @param output_stream The stream to which to serialize the node information.
                 */
                virtual void serialize(std::ofstream& output_stream)
                {
                    util::directed_edge_t::serialize(output_stream);
                    //        output_stream << " " << sampling_steps;

                }

                /**
                 * @brief Read in edge information to a stream.
                 *
                 * @param input_stream The stream from which to read the node information.
                 */
                virtual void deserialize(std::ifstream& input_stream)
                {
                    util::directed_edge_t::deserialize(input_stream);
                    //        input_stream >> sampling_steps;
                }

                /** @brief The computed plan for this edge. */
                sim::plan_t plan;
                /** @brief The computed path for this edge. */
                sim::trajectory_t path;
                /** @brief A unique identifier for this edge. */
                int id;
                /** @brief The cost associated with this edge. */
                double cost;
                
                complex_t h_value;


            };

            /**
             * @anchor prm_star_found_goal
             *
             * This class is used by the boost::astar function.  It's purpose is to identify
             * when the goal node has been expanded.
             *
             * @brief <b> Boost goal identification function for A* searches. </b>
             *
             * @author Athanasios Krontiris
             */
            struct h_graph_found_goal
            {    
                /** @brief Vertex index of the goal node. */
                util::directed_vertex_index_t v_goal;

                /**
                 * @brief Function used by boost::astar to determine if the goal has been expanded.
                 *
                 * @param v The node to make the goal.
                 */
                h_graph_found_goal(util::directed_vertex_index_t v)
                {
                    v_goal = v;
                }
            };

            /**
             * @anchor h_graph_distance_heuristic
             *
             * This class is a default heuristic to be used by A* searches for PRM.
             *
             * @brief <b> Distance heuristic used by PRM A* searches. </b>
             *
             * @author Athanasios Krontiris
             */
            class h_graph_distance_heuristic : public boost::astar_heuristic<util::directed_graph_type, double>
            {

              public:
                /**
                 * Constructor for the heuristic. Initializes the distance metric for the heuristic.
                 * 
                 * @brief Constructor for the heuristic.
                 *
                 * @param metric The distance metric to be used to determine distances in the space.
                 */
                h_graph_distance_heuristic( util::distance_metric_t* metric)
                {
                    distance_metric = metric;
                }

                /**
                 * Constructor for the heuristic. Initializes the graph and the distance metric for the 
                 * heuristic.
                 * 
                 * @brief Constructor for the heuristic.
                 *
                 * @param g The graph built by PRM to link to this heuristic.
                 * @param metric The distance metric to be used to determine distances in the space.
                 */
                h_graph_distance_heuristic(const util::directed_graph_t * g, util::distance_metric_t* metric)
                {
                    inner_graph = g;       
                    distance_metric = metric;
                }

                /**
                 * @brief Constructor for the heuristic. Initializes the graph the distance metric and 
                 * a set of goals for the heuristic.
                 *
                 * @param g The graph built by PRM to link to this heuristic.
                 * @param goals The set of goals from which to draw the heuristic.
                 * @param metric The distance metric to be used to determine distances in the space.
                 */
                h_graph_distance_heuristic(const util::directed_graph_t * g, std::vector<util::directed_vertex_index_t>& goals, util::distance_metric_t* metric)
                {
                    inner_graph = g;
                    distance_metric = metric;
                    set_new_goals(goals);
                }

                /**
                 * Sets a new graph and a new set of goals.
                 * 
                 * @brief Sets a new graph and a new set of goals.
                 * 
                 * @param g The graph built by PRM to link to this heuristic.
                 * @param goals The new set of goals.
                 */
                void set(const util::directed_graph_t * g, std::vector<util::directed_vertex_index_t>& goals)
                {
                    inner_graph = g;
                    foreach(util::directed_vertex_index_t goal, goals)
                    {
                        goal_nodes.push_back(g->operator[](goal));
                    }
                }

                /**
                 * Sets new set of goals.
                 * 
                 * @brief Sets new set of goals.
                 * 
                 * @param goals The new set of goals.
                 */
                void set_new_goals(std::vector<util::directed_vertex_index_t>& goals)
                {
                    foreach(util::directed_vertex_index_t goal, goals)
                    {
                        goal_nodes.push_back(inner_graph->operator[](goal));
                    }
                }

                /**
                 * @brief Calling operator for the heuristic.
                 *
                 * @param u The vertex for which to compute the heuristic.
                 *
                 * @return The heuristic value for u.
                 */
                double operator()(Vertex u)
                {        
                    double dist = PRX_INFINITY;
                    double new_dist;
                    util::directed_node_t* new_node = inner_graph->operator[](u);
                    foreach(util::directed_node_t* goal_node, goal_nodes)
                    {
                        new_dist = distance_metric->distance_function(new_node->point, goal_node->point);
                        if(new_dist < dist)
                            dist = new_dist;
                    }               
                    return dist;
                }

              private:
                /** @brief The set of goal nodes for the heuristic. */
                std::vector<util::directed_node_t*> goal_nodes;
                /** @brief The distance metric to use to determine distance. */
                util::distance_metric_t* distance_metric;
                /** @brief The graph over which this heuristic is operating. */
                const util::directed_graph_t * inner_graph;
            };

            /**
             * @anchor default_prm_star_astar_visitor
             *
             * Boost astar visitor class used by PRM by default.
             *
             * @brief <b> Visitor class used in PRM for A* searches. </b>
             *
             * @author Athanasios Krontiris
             */
            class h_graph_astar_visitor : public boost::default_astar_visitor
            {

              public:
                /**     
                 * Default prm astar visitor constructor.
                 * 
                 * @brief Default prm astar visitor constructor.
                 */
                h_graph_astar_visitor()
                {
                    m_goals = NULL;
                }

                /**
                 * Sets new set of goals in the visitor. 
                 * 
                 * @brief Sets new set of goals in the visitor. 
                 * 
                 * @param goals The new set of goals.
                 */
                void set_new_goals(const std::vector<util::directed_vertex_index_t>* goals)
                {
                    m_goals = goals;
                }

                /**
                 * Vertex examination function called when a node is expanded.
                 * 
                 * @brief Vertex examination function called when a node is expanded.
                 *
                 * @param u The vertex to examine.
                 * @param g The graph to which this vertex belongs.
                 *
                 * @note Throws a prm_star_found_goal exception if u is the goal.
                 */
                template <class Graph_t>
                void examine_vertex(util::directed_vertex_index_t u, Graph_t& g)
                {        
                    PRX_ASSERT(m_goals != NULL);
                    foreach(util::directed_vertex_index_t m_goal, *m_goals)
                    {
                        if( u == m_goal )        
                            throw h_graph_found_goal(u);
                    }
                }

              private:
                /** @brief The stored set of goals for this A* search. */
                const std::vector<util::directed_vertex_index_t>* m_goals;
            };
        }
    }
}

#endif
