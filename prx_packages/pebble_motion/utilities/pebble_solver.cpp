/**
 * @file pebble_solver.cpp
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

#include "utilities/pebble_solver.hpp"
#include <boost/range/adaptor/map.hpp>

#include <ros/ros.h>

namespace prx
{
    using namespace util;

    namespace packages
    {
        namespace pebble_motion
        {

            pebble_solver_t::pebble_solver_t()
            {
                bfs_global_id = 0;
                obstacle_global_id = 0;
                number_of_vertices = 0;
                number_of_edges = 0;

                obstacle_global_id = 0;
                bfs_global_id = 0;

                robot_path2.resize(2);
                robot_path3.resize(3);
            }

            pebble_solver_t::~pebble_solver_t() { }

            pluginlib::ClassLoader<pebble_solver_t> pebble_solver_t::loader("prx_utilities", "prx::packages::pebble_motion::pebble_solver_t");

            pluginlib::ClassLoader<pebble_solver_t>& pebble_solver_t::get_loader()
            {
                return loader;
            }

            void pebble_solver_t::setup(undirected_graph_t* graph, pebble_assignment_t& start_assign, pebble_assignment_t& target_assign, int num_robots, const space_t* space, distance_metric_t* distance_metric)
            {
                //    PRX_INFO_S("Setup pebble_solver");
                state_space = space;
                s_assignment = start_assign;
                t_assignment = target_assign;
                metric = distance_metric;
                k = num_robots;

                number_of_vertices = boost::num_vertices(graph->graph);
                number_of_edges = boost::num_edges(graph->graph);

                static_heap.resize(number_of_vertices);
                tmp_vector.resize(number_of_vertices);
                tmp_vector_index = 0;
            }

            void pebble_solver_t::reset()
            {
                PRX_WARN_S("Reset function has not being implemented in pebble solver");
            }

            bool pebble_solver_t::execute(std::vector<pebble_step_t>* solution, undirected_graph_t* graph)
            {
                //    PRX_ERROR_S("STARTING THE PEBBLE SOLVER WITH " << k << " ROBOTS");
                //    PRX_WARN_S("------------    strt    ------------");
                //    print_assignment(graph, s_assignment);
                //    PRX_WARN_S("-----------     targ    ------------");
                //    print_assignment(graph, t_assignment);
                //    PRX_WARN_S("------------------------------------");



                //                PRX_INFO_S("Reduce the PMG - > PPG...");
                if( !reduce(solution, graph, s_new, s_assignment, t_assignment) )
                    return false;

                //    return true;
                PRX_INFO_S("Going to find the solution...");
                return find_solution(solution, graph);

            }

            bool pebble_solver_t::is_valid_path(const undirected_graph_t* g, int pebble_id, std::vector<undirected_vertex_index_t> path, pebble_assignment_t& assignment)
            {
                bool valid_path = true;
                int blocker_id;
                undirected_vertex_index_t v_curr;
                undirected_vertex_index_t v_prev = assignment.get_position(pebble_id);

                for( int i = 0; i < (int)path.size(); ++i )
                {
                    v_curr = path[i];
                    blocker_id = assignment.get_robot(v_curr);

                    if( blocker_id != -1 && blocker_id != pebble_id )
                    {
                        PRX_DEBUG_COLOR("Pebbles: " << pebble_id << "-" << blocker_id << "  will collide on: " << print_point(g, v_curr), PRX_TEXT_RED);
                        valid_path = false;
                    }

                    if( !boost::edge(v_prev, v_curr, g->graph).second && v_prev != v_curr )
                    {
                        PRX_DEBUG_COLOR(i << ") vertices: " << v_prev << " - " << v_curr, PRX_TEXT_LIGHTGRAY);
                        PRX_DEBUG_COLOR("For pebble (" << pebble_id << ")  edge: " << print_point(g, v_prev) << " -> " << print_point(g, v_curr) << " does not exist!", PRX_TEXT_RED);
                        valid_path = false;
                    }
                    v_prev = v_curr;
                    assignment.change_position(pebble_id, v_curr);
                }

                return valid_path;
            }

            bool pebble_solver_t::is_valid_solution(const util::undirected_graph_t* g, const std::vector<pebble_step_t>* solution, const pebble_assignment_t assignment)
            {
                pebble_assignment_t assign = assignment;
                //                PRX_DEBUG_COLOR("The solution size is : " << solution->size(),PRX_TEXT_MAGENTA);
                bool valid_path = true;

                for( int s = 0; s < (int)solution->size(); ++s )
                    if( !is_valid_path(g, solution->at(s).first, solution->at(s).second, assign) )
                        valid_path = false;

                return valid_path;
            }

            void pebble_solver_t::get_parallel_steps(std::vector<int>& steps)
            {
                PRX_WARN_S("This pebble solver is not parallel.");
            }

            void pebble_solver_t::deserialize(util::undirected_graph_t* graph, std::string graphfile, const space_t* space)
            {
                PRX_WARN_S("Deserialize is not Implemented!");
            }

            void pebble_solver_t::print_assignment(const undirected_graph_t* graph, const pebble_assignment_t& assign, TEXT_COLOR color, int prec) const
            {
                PRX_DEBUG_COLOR("assign size : " << assign.size(), color);
                //    PRX_LOG_ERROR("stop");

                foreach(undirected_vertex_index_t v, assign.assignments | boost::adaptors::map_keys)
                {
                    PRX_DEBUG_COLOR("place(" << v << "): " << state_space->print_point(graph->get_vertex_as<undirected_node_t > (v)->point, prec) << "   -   robot: " << assign.get_robot(v), color);
                }
            }

            std::string pebble_solver_t::print_point(const undirected_graph_t* graph, undirected_vertex_index_t v, int prec) const
            {
                std::stringstream out(std::stringstream::out);
                out << state_space->print_point(graph->get_vertex_as<undirected_node_t > (v)->point, prec);
                return out.str();
            }

        }
    }
}






