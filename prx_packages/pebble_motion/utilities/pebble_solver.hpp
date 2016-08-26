/**
 * @file pebble_solver.hpp
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

#ifndef PRX_PEBBLE_SOLVER_HPP
#define	PRX_PEBBLE_SOLVER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"
#include "utilities/pebble_assignment.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "utilities/allocated_heap.hpp"
#include "utilities/pebble_solution_path.hpp"

#include <pluginlib/class_loader.h>

namespace prx
{
    namespace packages
    {
        namespace pebble_motion
        {

            struct no_solution_t
            {

                no_solution_t()
                {
                    PRX_ERROR_S("There is no solution in the problem!");
                }

                no_solution_t(std::string msg)
                {
                    PRX_ERROR_S(msg);
                }
            };

            class pebble_solver_edge_t : public util::undirected_edge_t
            {

              public:
                int steps;

            };

            /**
             * An algorithm that will return a path planning solution for a pebble problem. 
             */
            class pebble_solver_t
            {

              public:
                pebble_solver_t();
                virtual ~pebble_solver_t();

                virtual void reset();

                virtual void setup(util::undirected_graph_t* graph, pebble_assignment_t& start_assign, pebble_assignment_t& target_assign, int num_robots, const util::space_t* space, util::distance_metric_t* distance_metric = NULL);

                virtual util::undirected_vertex_index_t add_new_vertex(util::undirected_graph_t* graph) = 0;

                virtual bool execute(std::vector<pebble_step_t>* solution, util::undirected_graph_t* graph);

                /**
                 * This function checks the solution path if it is valid. The path is only the moves that each 
                 * agent will execute. The initial positions of the agents do not exist in the solution path.
                 * eg. Initial positions : 1:1 , 2:7
                 * 1: 2 3 4 
                 * 2: 5 6 3 2 
                 * 1: 3 1 
                 * This means that agent 1 will move from its current position (1) to 2, then 3, then 4. After 
                 * that agent 2 will move from position 7, to 5,6,3,2. At the end agent 1 from its current position
                 * 4 it will move to 3 and back to 1.
                 * 
                 * @param g The graph that the solution path is computed.
                 * @param solution The solution path that solves the problem.
                 * @param assignment The initial assignment of the agents.
                 * 
                 * @return Returns \c TRUE if the solution is valid, else \c FALSE. 
                 */
                virtual bool is_valid_solution(const util::undirected_graph_t* g, const std::vector<pebble_step_t>* solution, const pebble_assignment_t assignment);
                
                virtual void get_parallel_steps(std::vector<int>& steps);

                virtual void deserialize(util::undirected_graph_t* graph, std::string graphfile, const util::space_t* space );
                
                static pluginlib::ClassLoader<pebble_solver_t>& get_loader();
                
                std::string print_point(const util::undirected_graph_t* graph, util::undirected_vertex_index_t v, int prec = 2) const;                               
                
              protected:

                virtual bool find_solution(std::vector<pebble_step_t>* solution, util::undirected_graph_t* graph) = 0;

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
                virtual bool reduce(std::vector<pebble_step_t>* solution, util::undirected_graph_t* graph, pebble_assignment_t& s_new, pebble_assignment_t& s_assign, pebble_assignment_t& t_assign) = 0;

                virtual void update_info(util::undirected_graph_t* graph, pebble_assignment_t& t_assign) = 0;

                /**
                 * This function will check if the path for an agent is valid or not. Will check
                 * for collisions with other agents and it will check if the path is valid on the
                 * given graph. 
                 * 
                 * @param g The graph that the solution path is computed.
                 * @param solution The solution path that solves the problem.
                 * @param assignment The current assignment of the agents.
                 * 
                 * @return \c TRUE if the path is valid, else \c FALSE.
                 */
                virtual bool is_valid_path(const util::undirected_graph_t* g, int pebble_id, std::vector<util::undirected_vertex_index_t> path, pebble_assignment_t& assignment);

                void print_assignment(const util::undirected_graph_t* graph, const pebble_assignment_t& assign, TEXT_COLOR color = PRX_TEXT_GREEN, int prec = 2) const;
                

                util::undirected_graph_t* g;
                const util::space_t* state_space;
                util::distance_metric_t* metric;
                pebble_assignment_t s_assignment; // The start configuration of the pebbles
                pebble_assignment_t t_assignment; // The target configuration of the pebbles
                pebble_assignment_t s_new; // The new start configuration after the PMT -> PPT
                int k; //number of pebbles

                unsigned long bfs_global_id;
                unsigned long obstacle_global_id;

                allocated_heap_t<util::undirected_vertex_index_t> static_heap;

                std::deque<util::undirected_vertex_index_t> tmp_vector; //A pre-allocated vector in order to avoid allocating vectors during run time
                int tmp_vector_index; //The current index of the pre-allocated vector

                std::vector<util::undirected_vertex_index_t> robot_path2;
                std::vector<util::undirected_vertex_index_t> robot_path3;

                int number_of_vertices;
                int number_of_edges;

              private:
                static pluginlib::ClassLoader<pebble_solver_t> loader;

            };

        }
    }
}

#endif	// PRX_PEBBLE_SOLVER_HPP
