/**
 * @file H_augmented_graph.hpp
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

#ifndef PRX_H_AUGMENTED_GRAPH_HPP
#define	PRX_H_AUGMENTED_GRAPH_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/graph/directed_graph.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/simulation/trajectory.hpp"

#include <ros/ros.h>
#include <complex>
#include <deque>

//
#include <boost/graph/detail/d_ary_heap.hpp>

// H-Graph uses complex numbers
#include <complex>

namespace prx
{
    namespace packages
    {
        namespace homotopies
        {
            typedef std::complex<double> complex_t;
            typedef std::vector<util::directed_edge_index_t> edge_index_list_t;
            struct augmented_node_t
            {
                util::directed_vertex_index_t graph_vertex;
                util::directed_vertex_index_t parent_vertex;

                complex_t augmented_coordinate;
                double actual_cost;
                double heuristic_cost;

                complex_t h_signature;
                edge_index_list_t h_path_indices;

            };
        }
    }
}

 #include <boost/version.hpp>

 #if BOOST_VERSION >= 104900
  #include <boost/heap/fibonacci_heap.hpp>
  #define HEAP_EXISTS
  typedef boost::heap::fibonacci_heap< prx::packages::homotopies::augmented_node_t, boost::heap::compare<std::greater<prx::packages::homotopies::augmented_node_t> > > heap_def;
 #else
  #include <queue>
  #define HEAP_DOES_NOT_EXIST
  typedef std::priority_queue<double,std::vector<prx::packages::homotopies::augmented_node_t>,std::greater<prx::packages::homotopies::augmented_node_t> > heap_def;
 #endif


namespace prx
{

    namespace packages
    {
        namespace homotopies
        {

            typedef heap_def augmented_heap_t;

            inline bool operator < (const augmented_node_t& x, const augmented_node_t& y)
            {
                return x.heuristic_cost < y.heuristic_cost;
            }
            inline bool operator > (const augmented_node_t& x, const augmented_node_t& y)
            {
                return x.heuristic_cost > y.heuristic_cost;
            }

            //typedef boost::graph::detail::d_ary_heap< augmented_node_t, boost::heap::compare<std::greater<augmented_node_t> > > augmented_heap2_t;

            /**
             * @anchor H_augmented_graph_t
             *
             * Implements Subhrajit's homology computation method
             * 
             * @brief <b> H-signature graph </b>
             * 
             * @author Andrew Kimmel
             */
            class H_augmented_graph_t
            {
              public:
                H_augmented_graph_t();
                virtual ~H_augmented_graph_t();

                void link_space(const util::space_t* state_space);

                void setup(util::directed_graph_t* input_graph, util::directed_vertex_index_t start_vertex, util::directed_vertex_index_t goal_vertex,  int num_homotopies, double h_threshold);

                bool compute_homotopies(std::vector< edge_index_list_t >& computed_homotopies);
                bool selective_compute_homotopies(std::vector< edge_index_list_t >& computed_homotopies, std::vector<complex_t>& selected_homotopies, std::vector<int>& index_list);


                /** Optional functions */

                void set_allowed_homotopies(const std::vector<complex_t>& setA);
                void set_blocked_homotopies(const std::vector<complex_t>& setB);

              protected:    
                /** @brief Linked spaces */
                const util::space_t* s_space;

                /** ---- H-Graph Variables Set from setup ---- */

                /** @brief Pointer to the graph */
                util::directed_graph_t* graph;

                /** @brief Start and goal vertices */
                util::directed_vertex_index_t v_s, v_g;

                /** @brief Defines the threshold for defining distinct homotopy classes */
                double homotopy_threshold;

                /** @brief The maximum number of homotopies to store */
                int max_explored_homotopies;

                /** Obstacle representative points */
                std::vector<complex_t> representative_points;

                // -- Optional (implicitly operate over set B) --
                // Allowed homotopy classes (set A)
                std::vector<complex_t> allowed_homotopies;
                bool allow_homotopies;
                // Disallowed homotopy classes (set B)
                std::vector<complex_t> blocked_homotopies;
                bool block_homotopies;

                // Helper functions
                bool same_homotopy(complex_t h1, complex_t h2);


              private:
                  // Custom ASTAR gooo
            //      std::priority_queue<double,std::vector<augmented_node_t>,std::greater<augmented_node_t> > astar_heap;
                  augmented_heap_t astar_heap;

                  // Map graph nodes to stored homotopies
                  util::hash_t<util::directed_vertex_index_t, std::vector<complex_t> > node_to_homotopies;
                  util::hash_t<util::directed_vertex_index_t, std::vector<augmented_node_t> > node_to_augmented_node;

            };
        }
    }
}

#endif
