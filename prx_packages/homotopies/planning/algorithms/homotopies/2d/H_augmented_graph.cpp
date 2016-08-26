/**
 * @file h_graph_planner.cpp
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

#include "planning/algorithms/homotopies/2d/H_augmented_graph.hpp"
#include "planning/motion_planners/subhrajit/h_graph.hpp"

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/goals/goal.hpp"

#include <algorithm> 
#include <boost/assign/list_of.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/range/adaptor/map.hpp>
#include <queue>


namespace prx
{
    using namespace util;
    using namespace sim;
    
    namespace packages
    {

        namespace homotopies
        {
            H_augmented_graph_t::H_augmented_graph_t() 
            {
                s_space = NULL;

            }

            H_augmented_graph_t::~H_augmented_graph_t() 
            {


            }

            void H_augmented_graph_t::link_space(const space_t* state_space)
            {
                PRX_DEBUG_COLOR ("Link state_space", PRX_TEXT_BROWN);
                s_space = state_space;

            }

            void H_augmented_graph_t::setup(directed_graph_t* input_graph, directed_vertex_index_t start_vertex, directed_vertex_index_t goal_vertex, int num_homotopies, double h_threshold)
            {
                PRX_DEBUG_COLOR ("Setup", PRX_TEXT_BROWN);

                if (s_space == NULL)
                {
                    PRX_FATAL_S ("Null state space");
                }

                graph = input_graph;
                v_s = start_vertex;
                v_g = goal_vertex;
                max_explored_homotopies = num_homotopies;
                homotopy_threshold = std::sqrt(h_threshold);
                allow_homotopies = block_homotopies = false;
                blocked_homotopies.clear();
//                astar_heap.clear();
                PRX_DEBUG_COLOR ("H_threshold: " << homotopy_threshold, PRX_TEXT_RED);

            }

            bool H_augmented_graph_t::compute_homotopies(std::vector< edge_index_list_t >& computed_homotopies)
            {
                PRX_DEBUG_COLOR("Compute homotopies", PRX_TEXT_BROWN);
                // Create start node, add to heap
                augmented_node_t start_node;
                start_node.graph_vertex = v_s;
                start_node.parent_vertex = NULL;
                start_node.heuristic_cost = 0.0;
                start_node.actual_cost = 0.0;
                start_node.augmented_coordinate = complex_t(graph->get_vertex_as<directed_node_t>(v_s)->point->at(0), graph->get_vertex_as<directed_node_t>(v_s)->point->at(1));
                start_node.h_signature = complex_t(0,0);
                sys_clock_t homotopy_timer;
                astar_heap.push(start_node);
                // Main loop
                homotopy_timer.reset();
                while (computed_homotopies.size() < max_explored_homotopies && homotopy_timer.measure() < 60)
                {

                    augmented_node_t min_node = astar_heap.top();
                    astar_heap.pop();
                    // For each neighbor n
                    int counter = 0;
//                    PRX_DEBUG_COLOR("Current node: " << min_node.augmented_coordinate << "has heuristic cost: " << min_node.heuristic_cost, PRX_TEXT_RED);
//                    if (min_node.graph_vertex != v_s)
//                    {
//                            PRX_DEBUG_COLOR(" had parent: " <<  graph->get_vertex_as<directed_node_t>(min_node.parent_vertex)->point->at(0) << ", " <<
//                                    graph->get_vertex_as<directed_node_t>(min_node.parent_vertex)->point->at(1) , PRX_TEXT_GREEN);
//                    }
                    foreach(directed_vertex_index_t n, boost::adjacent_vertices(min_node.graph_vertex,graph->graph))
                    {
                        if (n != min_node.parent_vertex )
                        {
                //            PRX_ERROR_S ("Counter: " << counter);
                //            PRX_ERROR_S ("Graph node vertex: " << min_node.graph_vertex);
                            counter++;
                            // Set up a potentially new augmented node
                            augmented_node_t new_neighbor_node;
                            new_neighbor_node.graph_vertex = n;
                            new_neighbor_node.augmented_coordinate = complex_t(graph->get_vertex_as<directed_node_t>(n)->point->at(0), graph->get_vertex_as<directed_node_t>(n)->point->at(1));
                            directed_edge_index_t e = boost::edge(min_node.graph_vertex,n,graph->graph).first;
                            new_neighbor_node.actual_cost = min_node.actual_cost + graph->weights[e]; //s_space->distance((*graph)[min_node.graph_vertex]->point, (*graph)[n]->point);
                            new_neighbor_node.heuristic_cost = new_neighbor_node.actual_cost + s_space->distance((*graph)[n]->point, (*graph)[v_g]->point);
//                            PRX_DEBUG_COLOR("Neighbor node: " << new_neighbor_node.augmented_coordinate << "has heuristic cost: " << new_neighbor_node.heuristic_cost, PRX_TEXT_CYAN);
                            // Build the connecting path
                //            PRX_ERROR_S ("Tested Q vertex: " << min_node.graph_vertex << " with neighbor vertex: " << n);
                            new_neighbor_node.h_path_indices = min_node.h_path_indices;
                            new_neighbor_node.h_path_indices.push_back(e);
//                            PRX_WARN_S ("Got edge: " << e);
                            h_edge_t* test = graph->get_edge_as<h_edge_t>(e);
                            if (test == NULL)
                            {
                                PRX_FATAL_S ("The graph is not composed of homotopy edges!");
                            }
                            
                            // Calculate its H-signature
                            new_neighbor_node.h_signature = min_node.h_signature + graph->get_edge_as<h_edge_t>(e)->h_value;

                            bool added = false;
                            // Case 1: First time visiting this vertex
                            if (node_to_homotopies[n].empty())
                            {
            //                    PRX_INFO_S ("Case 1");
                                node_to_homotopies[n].push_back(new_neighbor_node.h_signature);
                                node_to_augmented_node[n].push_back(new_neighbor_node);
                                added = true;
                            }
                            // Case 2 and 3
                            else
                            {
                                bool same = false;
                                complex_t check_homotopy;
                                size_t i;
                                for (i = 0; i < node_to_homotopies[n].size(); i++)
                                {
                                    check_homotopy = node_to_homotopies[n][i];
                                    if (same_homotopy(check_homotopy, new_neighbor_node.h_signature))
                                    {
                                        same = true;
                                        break;
                                    }
                                }
                                // Case 2: Different homotopy
                                if (!same)
                                {
            //                        PRX_INFO_S ("Case 2");
                                    node_to_homotopies[n].push_back(new_neighbor_node.h_signature);
                                    node_to_augmented_node[n].push_back(new_neighbor_node);
                                    added = true;
                                }
                                // Case 3: Same homotopy, better f cost
                                else
                                {
            //                        PRX_INFO_S ("Case 3");
                                    if (new_neighbor_node.heuristic_cost < node_to_augmented_node[n][i].heuristic_cost)
                                    {
            //                            PRX_DEBUG_COLOR("Better cost: " << new_neighbor_node.heuristic_cost << "vs. : " << node_to_augmented_node[n][i].heuristic_cost, PRX_TEXT_GREEN);
                                        node_to_augmented_node[n][i] = new_neighbor_node;
                                        node_to_homotopies[n][i] = new_neighbor_node.h_signature;
                                        added = true;
//// 
////                                        // Method for updating heap, invariant to heap data 
//                                        std::vector<augmented_node_t> node_storage;
//                                        augmented_node_t test_node;
//                                        do
//                                        {
//                                            test_node = astar_heap.top();
//                                            astar_heap.pop();
//                                            if(test_node.h_signature == new_neighbor_node.h_signature )
//                                            {
//                                                node_storage.push_back(new_neighbor_node);
//                                            }
//                                            else
//                                            {
//                                                node_storage.push_back(test_node);
//                                            }
////                                            PRX_DEBUG_COLOR("Neighbor node: " << test_node.augmented_coordinate << "has heuristic cost: " << test_node.heuristic_cost, PRX_TEXT_CYAN);
////                                            PRX_ERROR_S ("Tested Q vertex: " << test_node.graph_vertex << " with neighbor vertex: " << n);
//                                        }
//                                        while (!astar_heap.empty());
//       
//                                        for (unsigned i = 0; i < node_storage.size(); i++)
//                                        {
//                                            astar_heap.push(node_storage[i]);
////                                            PRX_DEBUG_COLOR("Neighbor node: " << node_storage[i]->augmented_coordinate << "has heuristic cost: " << node_storage[i]->heuristic_cost, PRX_TEXT_CYAN);
////                                            PRX_ERROR_S ("Tested Q vertex: " << node_storage[i]->graph_vertex);
//                                        }
                                        
                                        
                                    }

                                }
                            }  
                            if (n == v_g)
                            {
            //                    PRX_DEBUG_COLOR("Pushed a new homotopic path!", PRX_TEXT_BROWN);
                                bool allowed = true; int i;
                                // Check if the calculated h_signature is blocked
                                for (i = 0; i < blocked_homotopies.size(); i++)
                                {
            //                        PRX_DEBUG_COLOR ("Compared current H-value: " << blocked_homotopies[i] << " to: " << new_neighbor_node.h_signature, PRX_TEXT_MAGENTA)
                                    if (this->same_homotopy(blocked_homotopies[i], new_neighbor_node.h_signature))
                                    {
                                        
                                        allowed = false;
                                        break;
                                    }
                                }
                                ;
                                if (allowed)
                                {
            //                        PRX_DEBUG_COLOR("Pushed a new homotopic path!", PRX_TEXT_GREEN);                    
                                    // Block this h-signature
                                    blocked_homotopies.push_back(new_neighbor_node.h_signature);
                                // Store the computed path to the goal
                                    computed_homotopies.push_back(new_neighbor_node.h_path_indices);
                                    PRX_DEBUG_COLOR("Found homotopy class: " << computed_homotopies.size()-1 << " at time: " << homotopy_timer.measure() << "with value: " << new_neighbor_node.h_signature , PRX_TEXT_GREEN);
                                }

                            }
                            else if (added)
                            {
                                new_neighbor_node.parent_vertex = min_node.graph_vertex;
                                astar_heap.push(new_neighbor_node);
                            }
                //                bool allowed = true;
                //                // Check if the calculated h_signature is blocked
                //                for (int i = 0; i < blocked_homotopies.size(); i++)
                //                {
                //                    if (this->same_homotopy(min_node.h_signature, new_neighbor_node.h_signature))
                //                    {
                //                        allowed = false;
                //                        break;
                //                    }
                //                }
                //                if (!allowed)
                //                {
                //                    PRX_DEBUG_COLOR ("Compared current H-value: " << min_node.h_signature << " to: " << new_neighbor_node.h_signature, PRX_TEXT_MAGENTA);
                //                }
                //                else
                //                {
                //                    PRX_DEBUG_COLOR("Pushed a new homotopic path!", PRX_TEXT_GREEN);
                //
                //                    
                //                    // Block this h-signature
                //                    blocked_homotopies.push_back(new_neighbor_node.h_signature);
                //                }
                ////                for (int x = 0; x < new_neighbor_node.h_path.size(); x++)
                ////                {
                ////                    PRX_DEBUG_COLOR("Traj state: " << new_neighbor_node.h_path[x]->at(0) << ", " << new_neighbor_node.h_path[x]->at(1), PRX_TEXT_LIGHTGRAY);
                ////                }
                //            }
                //            else
                //            {
                //                astar_heap.push(new_neighbor_node);
                //            }
                        }

                    }

                    // End of loop check if heap is still empty!
                    if (astar_heap.empty())
                    {
                        PRX_ERROR_S("Empty heap!");
                        return false;
                    }
                }
                if (computed_homotopies.size() == 0 )
                {
                    PRX_ERROR_S ("Returned no homotopies!");
                    return false;
                }
                else
                    return true;
            }
            
            
            bool H_augmented_graph_t::selective_compute_homotopies(std::vector< edge_index_list_t >& computed_homotopies, std::vector<complex_t>& selected_homotopies, std::vector<int>& index_list)
            {
                PRX_DEBUG_COLOR("Compute homotopies", PRX_TEXT_BROWN);
                // Create start node, add to heap
                augmented_node_t start_node;
                start_node.graph_vertex = v_s;
                start_node.parent_vertex = NULL;
                start_node.heuristic_cost = 0.0;
                start_node.actual_cost = 0.0;
                start_node.augmented_coordinate = complex_t(graph->get_vertex_as<directed_node_t>(v_s)->point->at(0), graph->get_vertex_as<directed_node_t>(v_s)->point->at(1));
                start_node.h_signature = complex_t(0,0);
                sys_clock_t homotopy_timer;
                astar_heap.push(start_node);
                // Main loop
                homotopy_timer.reset();
                while (computed_homotopies.size() < max_explored_homotopies && homotopy_timer.measure() < 60)
                {

                    augmented_node_t min_node = astar_heap.top();
                    astar_heap.pop();
                    // For each neighbor n
                    int counter = 0;
            //        PRX_DEBUG_COLOR("Current node: " << min_node.augmented_coordinate << "has heuristic cost: " << min_node.heuristic_cost, PRX_TEXT_RED);
            //        if (min_node.graph_vertex != v_s)
            //        {
            //                PRX_DEBUG_COLOR(" had parent: " <<  graph->get_vertex_as<directed_node_t>(min_node.parent_vertex)->point->at(0) << ", " <<
            //                        graph->get_vertex_as<directed_node_t>(min_node.parent_vertex)->point->at(1) , PRX_TEXT_GREEN);
            //        }
                    foreach(directed_vertex_index_t n, boost::adjacent_vertices(min_node.graph_vertex,graph->graph))
                    {
                        if (n != min_node.parent_vertex )
                        {
                //            PRX_ERROR_S ("Counter: " << counter);
                //            PRX_ERROR_S ("Graph node vertex: " << min_node.graph_vertex);
                            counter++;
                            // Set up a potentially new augmented node
                            augmented_node_t new_neighbor_node;
                            directed_edge_index_t e = boost::edge(min_node.graph_vertex,n,graph->graph).first;
                            new_neighbor_node.graph_vertex = n;
                            new_neighbor_node.augmented_coordinate = complex_t(graph->get_vertex_as<directed_node_t>(n)->point->at(0), graph->get_vertex_as<directed_node_t>(n)->point->at(1));
//                            new_neighbor_node.augmented_coordinate.real() = graph->get_vertex_as<directed_node_t>(n)->point->at(0);
//                            new_neighbor_node.augmented_coordinate.imag() = graph->get_vertex_as<directed_node_t>(n)->point->at(1);
                            new_neighbor_node.actual_cost = min_node.actual_cost + graph->weights[e];// s_space->distance((*graph)[min_node.graph_vertex]->point, (*graph)[n]->point);
                            new_neighbor_node.heuristic_cost = new_neighbor_node.actual_cost + s_space->distance((*graph)[n]->point, (*graph)[v_g]->point);
            //                PRX_DEBUG_COLOR("Neighbor node: " << new_neighbor_node.augmented_coordinate << "has heuristic cost: " << new_neighbor_node.heuristic_cost, PRX_TEXT_CYAN);
                            // Build the connecting path
                //            PRX_ERROR_S ("Tested Q vertex: " << min_node.graph_vertex << " with neighbor vertex: " << n);
                            
                            new_neighbor_node.h_path_indices = min_node.h_path_indices;
                            new_neighbor_node.h_path_indices.push_back(e);
                //            PRX_WARN_S ("Got edge: " << e);
                            PRX_ASSERT(graph->get_edge_as<h_edge_t>(e) != NULL);
                            
                            // Calculate its H-signature
                            new_neighbor_node.h_signature = min_node.h_signature + graph->get_edge_as<h_edge_t>(e)->h_value;
                            std::vector<complex_t>& node_h_classes = node_to_homotopies[n];
                            std::vector<augmented_node_t>& augmented_nodes = node_to_augmented_node[n];
                            
                            bool added = false;
                            // Case 1: First time visiting this vertex
                            if (node_h_classes.empty())
                            {
            //                    PRX_INFO_S ("Case 1");
                                node_h_classes.push_back(new_neighbor_node.h_signature);
                                augmented_nodes.push_back(new_neighbor_node);
                                added = true;
                            }
                            // Case 2 and 3
                            else
                            {
                                bool same = false;
                                complex_t check_homotopy;
                                size_t i;
                                for (i = 0; i < node_h_classes.size(); i++)
                                {
                                    check_homotopy = node_h_classes[i];
                                    if (same_homotopy(check_homotopy, new_neighbor_node.h_signature))
                                    {
                                        same = true;
                                        break;
                                    }
                                }
                                // Case 2: Different homotopy
                                if (!same)
                                {
            //                        PRX_INFO_S ("Case 2");
                                    node_h_classes.push_back(new_neighbor_node.h_signature);
                                    augmented_nodes.push_back(new_neighbor_node);
                                    added = true;
                                }
                                // Case 3: Same homotopy, better f cost
                                else
                                {
            //                        PRX_INFO_S ("Case 3");
                                    if (new_neighbor_node.heuristic_cost < augmented_nodes[i].heuristic_cost)
                                    {
            //                            PRX_DEBUG_COLOR("Better cost: " << new_neighbor_node.heuristic_cost << "vs. : " << augmented_nodes[i].heuristic_cost, PRX_TEXT_GREEN);
                                        augmented_nodes[i] = new_neighbor_node;
                                        node_h_classes[i] = new_neighbor_node.h_signature;
                                        added = true;
//// 
////                                        // Method for updating heap, invariant to heap data 
//                                        std::vector<augmented_node_t> node_storage;
//                                        augmented_node_t test_node;
//                                        do
//                                        {
//                                            test_node = astar_heap.top();
//                                            astar_heap.pop();
//                                            if(test_node.h_signature == new_neighbor_node.h_signature )
//                                            {
//                                                node_storage.push_back(new_neighbor_node);
//                                            }
//                                            else
//                                            {
//                                                node_storage.push_back(test_node);
//                                            }
////                                            PRX_DEBUG_COLOR("Neighbor node: " << test_node.augmented_coordinate << "has heuristic cost: " << test_node.heuristic_cost, PRX_TEXT_CYAN);
////                                            PRX_ERROR_S ("Tested Q vertex: " << test_node.graph_vertex << " with neighbor vertex: " << n);
//                                        }
//                                        while (!astar_heap.empty());
//       
//                                        for (unsigned i = 0; i < node_storage.size(); i++)
//                                        {
//                                            astar_heap.push(node_storage[i]);
////                                            PRX_DEBUG_COLOR("Neighbor node: " << node_storage[i]->augmented_coordinate << "has heuristic cost: " << node_storage[i]->heuristic_cost, PRX_TEXT_CYAN);
////                                            PRX_ERROR_S ("Tested Q vertex: " << node_storage[i]->graph_vertex);
//                                        }
                                        
                                        
                                    }

                                }
                            }  
                            if (n == v_g)
                            {
            //                    PRX_DEBUG_COLOR("Pushed a new homotopic path!", PRX_TEXT_BROWN);
                                bool allowed = true; int i;
                                // Check if the calculated h_signature is blocked
                                for (i = 0; i < blocked_homotopies.size(); i++)
                                {
            //                        PRX_DEBUG_COLOR ("Compared current H-value: " << blocked_homotopies[i] << " to: " << new_neighbor_node.h_signature, PRX_TEXT_MAGENTA)
                                    if (this->same_homotopy(blocked_homotopies[i], new_neighbor_node.h_signature))
                                    {
                                        
                                        allowed = false;
                                        break;
                                    }
                                }
                                
                                if (allowed)
                                {
                                    PRX_DEBUG_COLOR("Pushed a new homotopic path!", PRX_TEXT_GREEN);
                                    PRX_DEBUG_COLOR ("Computed H-sig: " << new_neighbor_node.h_signature, PRX_TEXT_MAGENTA);
                                    // Block this h-signature
                                    blocked_homotopies.push_back(new_neighbor_node.h_signature);
                                // Store the computed path to the goal
                                    for (unsigned j = 0; j < selected_homotopies.size(); j++)
                                    {
                                        if (this->same_homotopy(selected_homotopies[j], new_neighbor_node.h_signature))
                                        {
                                            index_list[j] = computed_homotopies.size();
                                            computed_homotopies.push_back(new_neighbor_node.h_path_indices);
                                            PRX_DEBUG_COLOR("Found homotopy class: " << computed_homotopies.size() << " at time: " << homotopy_timer.measure() << "with value: " << new_neighbor_node.h_signature , PRX_TEXT_GREEN);
                                            break;
                                        }
                                    }
                                    
                                }

                            }
                            else if (added)
                            {
                                new_neighbor_node.parent_vertex = min_node.graph_vertex;
                                astar_heap.push(new_neighbor_node);
                            }
                //                bool allowed = true;
                //                // Check if the calculated h_signature is blocked
                //                for (int i = 0; i < blocked_homotopies.size(); i++)
                //                {
                //                    if (this->same_homotopy(min_node.h_signature, new_neighbor_node.h_signature))
                //                    {
                //                        allowed = false;
                //                        break;
                //                    }
                //                }
                //                if (!allowed)
                //                {
                //                    PRX_DEBUG_COLOR ("Compared current H-value: " << min_node.h_signature << " to: " << new_neighbor_node.h_signature, PRX_TEXT_MAGENTA);
                //                }
                //                else
                //                {
                //                    PRX_DEBUG_COLOR("Pushed a new homotopic path!", PRX_TEXT_GREEN);
                //
                //                    
                //                    // Block this h-signature
                //                    blocked_homotopies.push_back(new_neighbor_node.h_signature);
                //                }
                ////                for (int x = 0; x < new_neighbor_node.h_path.size(); x++)
                ////                {
                ////                    PRX_DEBUG_COLOR("Traj state: " << new_neighbor_node.h_path[x]->at(0) << ", " << new_neighbor_node.h_path[x]->at(1), PRX_TEXT_LIGHTGRAY);
                ////                }
                //            }
                //            else
                //            {
                //                astar_heap.push(new_neighbor_node);
                //            }
                        }

                    }

                    // End of loop check if heap is still empty!
                    if (astar_heap.empty())
                    {
                        PRX_ERROR_S("Empty heap!");
                        return false;
                    }
                }
                if (computed_homotopies.size() == 0 )
                {
                    PRX_ERROR_S ("Returned no homotopies!");
                    return false;
                }
                else
                    return true;
            }


            bool H_augmented_graph_t::same_homotopy(complex_t h1, complex_t h2)
            {
//                double normed = std::norm(h1 - h2) ;
                double normed = std::abs(h1.real()-h2.real()) + std::abs(h1.imag() - h2.imag());
                if (normed < homotopy_threshold)
                    return true;
                return false;

            }
            
            void H_augmented_graph_t::set_allowed_homotopies(const std::vector<complex_t>& setA)
            {
                allowed_homotopies = setA;
                allow_homotopies = true;
            }
            void H_augmented_graph_t::set_blocked_homotopies(const std::vector<complex_t>& setB)
            {
                blocked_homotopies = setB;
                block_homotopies = true;
            }
        }
    }
}