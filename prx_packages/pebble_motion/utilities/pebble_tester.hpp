/**
 * @file pebble_tester.h
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

#ifndef PRX_PEBBLE_TEST_H
#define	PRX_PEBBLE_TEST_H

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"
#include "utilities/pebble_assignment.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"

#include <pluginlib/class_loader.h>

namespace prx
{
    namespace packages
    {
        namespace pebble_motion
        {

            class pebble_test_edge_t : public util::undirected_edge_t
            {

              public:
                int steps;

            };

            /**
             * Computes if a pebble test is solvable or not.
             */
            class pebble_tester_t
            {

              public:

                virtual ~pebble_tester_t();

                virtual void reset();

                virtual void setup(pebble_assignment_t& start_assign, pebble_assignment_t& target_assign, int num_robots, const util::space_t* space, util::distance_metric_t* distance_metric = NULL);

                virtual util::undirected_vertex_index_t add_new_vertex(util::undirected_graph_t* graph) = 0;

                /**
                 * This function will reduce a PMG problem into a PPG problem, by moving all the pebble so as the starting positions 
                 * of the pebble will occupy all the final positions of the problem, without the correct ordering. 
                 * This algorithm it is not the fastest possible solution, as there is a linear algorithm for the reduction. This 
                 * algorithm will probably take time in the order of k*n
                 * @param s_new The new assignment with the pebbles occupy the same nodes as the final positions of the problem
                 * @param s_assignment The starting assignment for the pebbles in the problem.
                 * @param e_assignment The final assignment for the pebbles in the problem.
                 * @return True if could make the reduction else False
                 */
                virtual bool reduce(pebble_assignment_t& s_new, pebble_assignment_t& s_assign, pebble_assignment_t& t_assign) = 0;

                virtual bool pebble_test(util::undirected_graph_t* graph) = 0;

                static pluginlib::ClassLoader<pebble_tester_t>& get_loader();

              protected:
                void print_assignment(const pebble_assignment_t& assign) const;

                util::undirected_graph_t* g;
                const util::space_t* state_space;
                util::distance_metric_t* metric;
                pebble_assignment_t s_assignment; // The start configuration of the pebbles
                pebble_assignment_t t_assignment; // The target configuration of the pebbles
                int k; //number of pebbles

              private:
                static pluginlib::ClassLoader<pebble_tester_t> loader;

            };


        }
    }
}

#endif	// PRX_PEBBLE_TEST_H
