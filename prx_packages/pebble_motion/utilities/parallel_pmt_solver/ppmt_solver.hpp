/**
 * @file ppmt_solver.hpp
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

#ifndef PRX_PPMT_SOLVER_HPP
#define	PRX_PPMT_SOLVER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "utilities/parallel_pmt_solver/ppmt_graph.hpp"
#include "utilities/parallel_pmt_solver/ppmt_agent.hpp"
#include "utilities/pmt_solver/pmt_solver.hpp"
#include <deque>

namespace prx
{
    namespace packages
    {
        namespace pebble_motion
        {

            /**
             * The implementation of the parallel_pebble_motion_tree_solver or else ppmt_solver.
             *  
             * This is a pathfinding algorithm so as to solve the pebble motion problem on trees
             * in a parallel version.
             */
            class ppmt_solver_t : public pmt_solver_t
            {

              public:
                ppmt_solver_t();
                virtual ~ppmt_solver_t();
                                              
                /** @copydoc pebble_solver_t::setup(pebble_assignment_t&, pebble_assignment_t&, int, const util::space_t*, util::distance_metric_t*)  */
                virtual void setup(util::undirected_graph_t* graph, pebble_assignment_t& start_assign, pebble_assignment_t& target_assign, int num_robots, const util::space_t* space, util::distance_metric_t* distance_metric = NULL);
                             
                /** @copydoc pebble_solver_t::add_new_vertex(util::undirected_graph_t*)  */
                virtual util::undirected_vertex_index_t add_new_vertex(util::undirected_graph_t* graph);                
                
                virtual void get_parallel_steps(std::vector<int>& steps);
                
              protected:                
                
                enum moving_action_t {MA_MOVE, MA_WAIT, MA_CYCLE, MA_SWAP, MA_SWAP_FAILED};
                
                                
                /** @copydoc pebble_solver_t::find_solution(std::vector<pebble_step_t>* , util::undirected_graph_t* )  */
                virtual bool find_solution(std::vector<pebble_step_t>* solution, util::undirected_graph_t* graph);

                virtual moving_action_t move_agent(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign, ppmt_agent_t* agent);
                
                virtual bool need_swap(util::undirected_graph_t* graph, ppmt_agent_t* agent, ppmt_agent_t* blocker, util::undirected_vertex_index_t blocker_pos);
                
                virtual bool prepare_swap(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign, ppmt_agent_t* agent, ppmt_agent_t* blocker);
                
                virtual moving_action_t swap(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign, ppmt_agent_t* agent);                                

                virtual bool check_branch(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign);

                virtual bool check_visible_branches(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign, util::undirected_vertex_index_t v_start, bool caller_on_w);

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
                virtual bool clear_branch(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign, int& empty_vertices);
                
                virtual moving_action_t evacuate_vertex(util::undirected_graph_t* graph, pebble_assignment_t& assign, int pebble_id);
                
                virtual moving_action_t evacuate_to_empty(util::undirected_graph_t* graph, pebble_assignment_t& assign, ppmt_agent_t* agent);
                
                virtual std::pair<int, int> find_blocker_on_path(ppmt_agent_t* agent, pebble_assignment_t& assign);
                
                virtual std::pair<bool, int> find_sort_path(std::deque<util::undirected_vertex_index_t>& path, util::undirected_graph_t* graph, pebble_assignment_t& assign, util::undirected_vertex_index_t v_start, util::undirected_vertex_index_t v_target, int path_length = 0, bool avoid_pebbles = false, bool avoid_used_v = false);                                                                
                
                virtual moving_action_t step_agent(pebble_solution_path_t& solution, util::undirected_graph_t* graph, pebble_assignment_t& assign, ppmt_agent_t* agent);
                virtual void step(pebble_solution_path_t& solution, pebble_assignment_t& assign, int pebble_id, util::undirected_vertex_index_t v_step);
                
                virtual bool can_agent_move(util::undirected_graph_t* graph, pebble_assignment_t& assign, ppmt_agent_t* agent, ppmt_agent_t* first_agent = NULL);

              protected:
                unsigned long used_global_id;
                unsigned long busy_global_id;
                unsigned long called_global_id;
                
//                unsigned long avoid_global_id;
                std::deque<ppmt_agent_t> agents_list;
                std::deque<ppmt_agent_t*> to_evacuate;
                int evacuate_len;
                std::vector<int> par_steps;
                
//                pebble_solution_path_t solution_path;
//                bool not_solved;
//
//                std::deque<int> order;
//                std::deque<int> called;
//                int called_size;
//                bool priority_cycle_detected;
//                allocated_heap_t<util::undirected_vertex_index_t> tmp_heap;
//                allocated_heap_t<int> push_order;
//
//                ppmt_graph_node_t* node;
//                ppmt_graph_node_t* w_node;
//                ppmt_graph_node_t* r_node;
//                ppmt_graph_node_t* neigh_node;
//                
//                util::undirected_vertex_index_t w;
//                util::undirected_vertex_index_t neigh;
//                util::undirected_vertex_index_t r;
//                util::undirected_vertex_index_t q;
//                
//                util::undirected_graph_t* initial_graph; //TODO: initial_graph its just for debug remove it
//
//                pmt_default_distance_heuristic_t* pmt_heuristic;
//                pmt_astar_visitor_t* pmt_visitor;
                
            };
            
        }
    }
}

#endif