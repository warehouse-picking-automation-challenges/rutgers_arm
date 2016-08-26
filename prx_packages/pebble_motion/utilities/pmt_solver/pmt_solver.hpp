/**
 * @file pmt_solver.hpp
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

#ifndef PRX_PMT_SOLVER_HPP
#define	PRX_PMT_SOLVER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "utilities/pebble_solver.hpp"
#include "utilities/pmt_solver/pmt_graph.hpp"
#include <deque>
 
namespace prx
{
    namespace packages
    {
        namespace pebble_motion
        {

            /**
             * The implementation of the pebble_motion_graph_solver or else pmt_solver.
             *  
             * This is a pathfinding algorithm so as to solve the pebble motion problem on graphs.
             */
            class pmt_solver_t : public pebble_solver_t
            {

              public:
                pmt_solver_t();
                virtual ~pmt_solver_t();

                /** @copydoc pebble_solver_t::setup(pebble_assignment_t&, pebble_assignment_t&, int, const util::space_t*, util::distance_metric_t*)  */
                void setup(util::undirected_graph_t* graph, pebble_assignment_t& start_assign, pebble_assignment_t& target_assign, int num_robots, const util::space_t* space, util::distance_metric_t* distance_metric = NULL);

                /** @copydoc pebble_solver_t::add_new_vertex(util::undirected_graph_t*)  */
                virtual util::undirected_vertex_index_t add_new_vertex(util::undirected_graph_t* graph);
                
                /** @copydoc pebble_solver_t::deserialize(util::undirected_graph_t&, std::string) */
                virtual void deserialize(util::undirected_graph_t* graph, std::string graphfile, const util::space_t* space);

              protected:

                /** @copydoc pebble_solver_t::find_solution(std::vector<pebble_step_t>* , util::undirected_graph_t* )  */
                virtual bool find_solution(std::vector<pebble_step_t>* solution, util::undirected_graph_t* graph);

                /** @copydoc pebble_solver_t::reduce(pebble_assignment_t&, pebble_assignment_t&, pebble_assignment_t&)  */
                virtual bool reduce(std::vector<pebble_step_t>* solution, util::undirected_graph_t* graph, pebble_assignment_t& s_new, pebble_assignment_t& s_assign, pebble_assignment_t& t_assign);

                /** @copydoc pebble_solver_t::update_info(util::undirected_graph_t* , pebble_assignment_t& ) */
                virtual void update_info(util::undirected_graph_t* graph, pebble_assignment_t& t_assign);

                virtual bool move_to_goal(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign, int pebble_id, util::undirected_vertex_index_t v_start, util::undirected_vertex_index_t v_goal);

                virtual bool swap(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign, int pebble1, int pebble2, std::deque<util::undirected_vertex_index_t>& path, int path_start, int path_end);

              protected:

                pebble_solution_path_t solution_path;
                bool not_solved;

                std::deque<int> order;
                std::deque<int> called;
                int called_size;
                bool priority_cycle_detected;
                allocated_heap_t<util::undirected_vertex_index_t> tmp_heap;
                allocated_heap_t<int> push_order;
                

                std::deque<util::undirected_vertex_index_t> tmp_path;
                int tmp_path_length;
                std::deque<util::undirected_vertex_index_t> tmp_empty_path;
                int tmp_empty_path_length;
                std::deque<util::undirected_vertex_index_t> v_u_path;
                int v_u_path_length;
                std::deque<util::undirected_vertex_index_t> v_w_path;
                int v_w_path_length;
                std::deque<util::undirected_vertex_index_t> u_w_path;
                int u_w_path_length;
                
                std::deque< std::deque<util::undirected_vertex_index_t> > agent_priority_paths;
                std::deque<int> priority_path_lengths;

                std::deque< std::deque<util::undirected_vertex_index_t> > agent_paths;
                std::deque<int> path_lengths;
                int agent_paths_index;

                unsigned long visited_global_id;
                unsigned long avoid_global_id;
                unsigned long on_path_branch_global_id;
                pmt_graph_node_t* node;
                pmt_graph_node_t* w_node;
                pmt_graph_node_t* r_node;
                pmt_graph_node_t* neigh_node;

                bool already_swapped;

                std::deque<util::undirected_vertex_index_t>::iterator it_begin;

                int first_caller;

                //The four vertices for the swap    
                util::undirected_vertex_index_t w;
                util::undirected_vertex_index_t neigh;
                util::undirected_vertex_index_t r;
                util::undirected_vertex_index_t q;

                util::undirected_graph_t* initial_graph; //TODO: initial_graph its just for debug remove it

                pmt_default_distance_heuristic_t* pmt_heuristic;
                pmt_astar_visitor_t* pmt_visitor;

                virtual bool check_branch(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign);

                virtual void check_visible_branches(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign, util::undirected_vertex_index_t v_start, util::undirected_vertex_index_t u_start, std::deque<util::undirected_vertex_index_t>& uv_path, int path_start, int path_end);
                
                virtual bool check_visible_branches(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign, util::undirected_vertex_index_t v_start, bool caller_on_w);

                virtual bool clear_branch(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign, int& empty_vertices);

                /**
                 * Clears the vertex from the pebble with id \c pebble_id. Will try to move the pebble first
                 * towards its goal else it will try to move it to an empty vertex.
                 * @param solution
                 * @param graph
                 * @param assign
                 * @param pebble_id
                 * 
                 * @return 
                 */
                virtual bool clear_vertex(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign, int pebble_id);

                virtual bool need_swap(util::undirected_graph_t* graph, std::deque<util::undirected_vertex_index_t>& path, int path_start, int path_length, util::undirected_vertex_index_t initial_p2, util::undirected_vertex_index_t final_p2);


                virtual std::pair<bool, int> path_to_empty(std::deque<util::undirected_vertex_index_t>& path, const util::undirected_graph_t* graph, pebble_assignment_t& assign, util::undirected_vertex_index_t v_start);

                /**
                 * Will find the shortest path between the start and the goal. The parameter path has be 
                 * allocated from outside of the algorithm. It will avoid all the other obstacles and will 
                 * not change the predecessor of the obstacles. 
                 * @param path A preallocated deque that the path will be returned.
                 * @param graph The graph.
                 * @param v_start The starting vertex.
                 * @param v_target The goal vertex.
                 * @param path_length Because the path is preallocated it might be that has more information in it. 
                 * The path from v to u will be start constructed after the path_length of the vector. By default is 0.
                 * @param block_path If block_path has the correct value of the obstacles the path will be blocked.
                 * By default is 0 which means free node.
                 * @return A pair where the first element will be the final position in the deque path, because the preallocated deque 
                 * could be bigger than the final path. The second element if found or not a path.
                 */
                virtual std::pair<bool, int> find_a_path(std::deque<util::undirected_vertex_index_t>& path, util::undirected_graph_t* graph, pebble_assignment_t& assign, util::undirected_vertex_index_t v_start, util::undirected_vertex_index_t v_target, int path_length = 0, bool avoid_path = false, bool block_path = false);

                virtual std::pair<bool, int> find_path(std::deque<util::undirected_vertex_index_t>& path, util::undirected_graph_t* graph, pebble_assignment_t& assign, util::undirected_vertex_index_t v_start, util::undirected_vertex_index_t v_target, int path_length = 0, bool avoid_path = false, bool block_path = false);

                virtual bool is_on_path(util::undirected_graph_t* graph, util::undirected_vertex_index_t v_start, util::undirected_vertex_index_t v_target, util::undirected_vertex_index_t v_search);


                /**
                 * Computes the order of the pebbles that will be examined during the execution
                 * of the algorithm.The order is based on the final assignments of the problem 
                 * starting from the leaves of a spanning tree of the graph.
                 * @param graph The graph.
                 * @param assignment The final assignment of the pebbles. Based on this assignment the
                 * order will be computed.
                 */
                virtual void compute_the_order(util::undirected_graph_t* graph, pebble_assignment_t& assignment);

                /**
                 * Will find if a pebble is blocking the given path.
                 * @param path The path that we will check for blockers.
                 * @return Will return a pair of the blocker's id and the position on the path that the blocker is.
                 */
                virtual std::pair<int, int> find_blocker_on_path(const std::deque<util::undirected_vertex_index_t>& path, int path_length, pebble_assignment_t& assign);

                virtual util::undirected_vertex_index_t get_valid_adjacent_vertex(util::undirected_graph_t* graph, util::undirected_vertex_index_t v);

                virtual void step(pebble_solution_path_t& solution, pebble_assignment_t& assign, int pebble_id, util::undirected_vertex_index_t v_step);
                virtual void two_steps(pebble_solution_path_t& solution, pebble_assignment_t& assign, int pebble_id, util::undirected_vertex_index_t v_step1, util::undirected_vertex_index_t v_step2);

            };

        }
    }
}


#endif
