/**
 * @file pebble_solver_tp.hpp
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

#ifndef PRX_PEBBLE_SOLVER_TP_HPP
#define	PRX_PEBBLE_SOLVER_TP_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/task_planners/task_planner.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"
#include "utilities/pebble_assignment.hpp"
#include "utilities/pebble_solver.hpp"
#include "pebble_test_query.hpp"
#include "utilities/push_and_swap/PushAndSwap.hpp"

#include <iostream>

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
             * 
             * 
             */
            class pebble_solver_tp_t : public plan::task_planner_t
            {

              public:
                pebble_solver_tp_t();
                virtual ~pebble_solver_tp_t();

                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                virtual void setup();

                virtual bool execute();

                virtual const util::statistics_t* get_statistics();

                virtual bool succeeded() const;

                virtual void link_query(plan::query_t* in_query);

                virtual void resolve_query();
                
                virtual void resolve_parallel_query(std::vector<int> steps);
                
                virtual void resolve_pas_query();
                               

              protected:
                virtual void build_graphs();
                virtual void setup_PaS();
                
                virtual void serialize_graph();
                
                virtual void update_vis_info() const;

                virtual void smoother(std::vector<pebble_step_t>& plan);
                virtual void smoother_check(std::vector<pebble_step_t>& plan, int robot_id, util::undirected_vertex_index_t v_check, int& plan_pos, int& step_pos);
                
                size_t compute_solution_length(const std::vector<pebble_step_t> plan);

                pebble_test_query_t* tp_query;
                const util::space_t* full_state_space;
                util::space_t* full_control_space;

                util::space_t* robot_state_space;
                util::space_t* robot_control_space;
                int robot_control_size;

                bool real_distances;

                //Graph
                typedef std::pair<int, std::vector<double> > node_id_t;                
                typedef std::pair<int, int> edge_id_t;                
                std::vector< node_id_t > nodes_to_add;
                std::vector< edge_id_t > edges_to_add;
                typedef std::pair<util::directed_vertex_index_t, unsigned int> graph_id_t;
                util::hash_t<int, util::directed_vertex_index_t> graph_node_id;                 
                util::hash_t<int, graph_id_t> nodes_map;
                util::undirected_graph_t graph;                
                std::string graphfile;
                
                bool save_graph;
                bool parallel_version;

                //Pebble tester
                int k; //number of robots
                pebble_solver_t* p_solver;
                pebble_assignment_t s_assign;
                pebble_assignment_t t_assign;
                std::vector<pebble_step_t> solution_path;
                PaS::PushAndSwap::Plan plan_to_smooth;
                std::string test_name;
                std::string output_file;


                //Push & Swap
                PaS::Graph *pas_graph;
                util::hash_t<int, util::undirected_vertex_index_t> pas_to_pebble;
                util::hash_t<util::undirected_vertex_index_t, int> pebble_to_pas;
                util::hash_t<int, std::string> robot_id_map;
                util::hash_t<std::string, int> robot_name_map;
                std::vector<PaS::Agent> pas_robots;
                PaS::PushAndSwap::Plan pas_plan;
                bool with_push_and_swap;


                //visualization
                bool visualize_graph;
                std::string visualization_graph_name;
                std::string visualization_body;
                std::string graph_color;

                std::ofstream datafile;
                
              private:
               virtual bool find_solution();                
//                std::pair<bool, size_t> solution_validation(const undirected_graph_t* tree, std::vector<pebble_step_t>* solution, const pebble_assignment_t assignment, const pebble_assignment_t t_assignment);

            };

        }
    }
}

#endif	// PRX_PEBBLE_SOLVER_TP_HPP
