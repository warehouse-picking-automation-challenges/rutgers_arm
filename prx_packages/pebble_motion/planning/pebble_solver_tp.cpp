/**
 * @file pebble_solver_tp.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2012, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Athanasios Krontiris, Andrew Dobson, Andrew Kimmel, akrontirisary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#include "pebble_solver_tp.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <boost/graph/compressed_sparse_row_graph.hpp>

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "utilities/tree_feasibility_tester/tree_pebble_graph.hpp"
#include "utilities/pmt_solver/pmt_graph.hpp"
#include "utilities/parallel_pmt_solver/ppmt_solver.hpp"

#include <boost/range/adaptor/map.hpp>
#include <pluginlib/class_list_macros.h>
#include <boost/graph/reverse_graph.hpp>

PLUGINLIB_EXPORT_CLASS( prx::packages::pebble_motion::pebble_solver_tp_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {
        namespace pebble_motion
        {

            pebble_solver_tp_t::pebble_solver_tp_t()
            {
                tp_query = NULL;
                pas_graph = new PaS::Graph();
            }

            pebble_solver_tp_t::~pebble_solver_tp_t()
            {
                pas_graph->clear();
            }

            void pebble_solver_tp_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                PRX_INFO_S("Initializing pebble_solver_tp_t ...");
                task_planner_t::init(reader, template_reader);

                //    PRX_WARN_S("reader trace : " << reader->trace());
                //    PRX_WARN_S("template_reader trace : " << template_reader->trace());

                if( parameters::has_attribute("points", reader, template_reader) )
                {

                    foreach(const parameter_reader_t* point_reader, reader->get_list("points"))
                    {
                        int id = point_reader->get_attribute_as<int>("id");
                        nodes_to_add.push_back(std::make_pair(id, point_reader->get_attribute_as<std::vector<double> > ("point")));
                    }
                    PRX_DEBUG_COLOR("node to add : " << nodes_to_add.size(), PRX_TEXT_MAGENTA);
                }

                if( parameters::has_attribute("edges", reader, template_reader) )
                {

                    foreach(const parameter_reader_t* edge_reader, reader->get_list("edges"))
                    {
                        edges_to_add.push_back(std::make_pair(edge_reader->get_attribute_as<int>("source"), edge_reader->get_attribute_as<int>("target")));
                    }
                    PRX_DEBUG_COLOR("edges_to_add to add : " << edges_to_add.size(), PRX_TEXT_MAGENTA);
                }

                if( parameters::has_attribute("pebble_solver", reader, template_reader) )
                {
                    PRX_INFO_S("Pebble Tester type: " << parameters::get_attribute_as<std::string > ("pebble_solver/type", reader, template_reader));
                    p_solver = parameters::create_from_loader<pebble_solver_t > ("prx_utilities", reader, "pebble_solver", template_reader, "pebble_solver");
                }

                real_distances = parameters::get_attribute_as<bool>("real_distances", reader, template_reader, false);

                visualize_graph = parameters::get_attribute_as<bool>("visualize_graph", reader, template_reader, false);
                visualization_graph_name = parameters::get_attribute_as<std::string > ("visualization_graph_name", reader, template_reader, "pebble_motion/graph");
                visualization_body = parameters::get_attribute_as<std::string > ("visualization_body", reader, template_reader);
                graph_color = parameters::get_attribute_as<std::string > ("graph_color", reader, template_reader, "black");

                test_name = parameters::get_attribute_as<std::string > ("test_name", reader, template_reader, "PSG_test");
                output_file = parameters::get_attribute_as<std::string > ("output_file", reader, template_reader, "pebble_tests");

                with_push_and_swap = parameters::get_attribute_as<bool> ("with_push_and_swap", reader, template_reader, false);
                parallel_version = parameters::get_attribute_as<bool> ("parallel_version", reader, template_reader, false);

                graphfile = parameters::get_attribute_as<std::string > ("graph_file", reader, template_reader, "");
                save_graph = parameters::get_attribute_as<bool> ("save_graph", reader, template_reader, false);

                if( !graphfile.empty() )
                {
                    char* w = std::getenv("PRACSYS_PATH");
                    std::string dir(w);
                    dir += ("/prx_packages/pebble_motion/input/common/graphs/");
                    graphfile = dir + graphfile;
                }
            }

            void pebble_solver_tp_t::setup()
            {

                PRX_INFO_S("Setting up the pebble_solver_tp...");

                model->use_context("single_robot_space");
                robot_state_space = model->get_state_space();
                robot_control_space = model->get_control_space();
                robot_control_size = robot_control_space->get_dimension();

                metric->link_space(robot_state_space);
                validity_checker->link_model(model);
                local_planner->link_model(model);
                local_planner->link_sampler(sampler);
                local_planner->link_metric(metric);

                if( !graphfile.empty() && !save_graph )
                {
                    p_solver->deserialize(&graph, graphfile, robot_state_space);

                    foreach(undirected_vertex_index_t v, boost::vertices(graph.graph))
                    {
                        graph_node_id[graph[v]->node_id] = v;
                    }

                }
                else
                {
                    this->build_graphs();
                }

                PRX_DEBUG_COLOR("Done with the graph", PRX_TEXT_BROWN);

                if( save_graph )
                    serialize_graph();

                model->use_context("full_space");
                full_state_space = model->get_state_space();
                full_control_space = model->get_control_space();

                int start_node, target_node;
                control_t* initial_control = full_control_space->alloc_point();
                std::vector<double> initial_point(full_control_space->get_dimension());

                k = tp_query->initial_state.size();

                for( int robot_id = 0; robot_id < k; ++robot_id )
                {
                    PRX_ASSERT(tp_query->initial_state[robot_id].first == tp_query->target_state[robot_id].first);
                    std::string name = tp_query->initial_state[robot_id].first;
                    start_node = tp_query->initial_state[robot_id].second;
                    target_node = tp_query->target_state[robot_id].second;

                    PRX_DEBUG_COLOR("robot(" << robot_id << ") name : " << name, PRX_TEXT_GREEN);
                    PRX_DEBUG_COLOR("start: (" << graph_node_id[start_node] << " - " << graph_node_id[start_node] << ")     final(" << graph_node_id[target_node] << " - " << graph_node_id[target_node] << ")", PRX_TEXT_CYAN);
                    PRX_DEBUG_COLOR("s_assign size: " << s_assign.size(), PRX_TEXT_BLUE);
                    s_assign.add_assignment(graph_node_id[start_node], robot_id);
                    t_assign.add_assignment(graph_node_id[target_node], robot_id);


                    int index = robot_id * robot_control_size;

                    for( int i = 0; i < robot_control_size; ++i )
                    {
                        initial_point[index + i] = graph.get_vertex_as<undirected_node_t > (graph_node_id[start_node])->point->at(i);
                        //initial_control->at(index + i) = graph.get_vertex_as<undirected_node_t > (graph_node_id[start_node])->point->at(i);
                    }
                }

                full_control_space->set_from_vector(initial_point, initial_control);
                PRX_DEBUG_COLOR("Done with the robots", PRX_TEXT_CYAN);
                tp_query->plan.link_control_space(full_control_space);
                tp_query->plan.copy_onto_back(initial_control, 3);
                PRX_DEBUG_COLOR(full_control_space->print_point(initial_control, 3), PRX_TEXT_BROWN);

                p_solver->setup(&graph, s_assign, t_assign, k, robot_state_space, metric);
                PRX_DEBUG_COLOR("Done with setting up", PRX_TEXT_CYAN);

                if( with_push_and_swap )
                    setup_PaS();


            }

            void pebble_solver_tp_t::serialize_graph()
            {
                PRX_DEBUG_COLOR("output file: " << graphfile, PRX_TEXT_MAGENTA);
                std::ofstream output_stream;
                output_stream.open(graphfile.c_str());
                graph.serialize(output_stream, robot_state_space);
                output_stream.close();
                PRX_LOG_ERROR("END Serialization");
            }

            double getMetricsSolver(const std::vector<PaS::Agent> agents, const PaS::PushAndSwap::Plan &plan)
            {
                // Constructing simulation for each agent
                //std::vector <PaS::PushAndSwap::Plan> individual_simulations;
                std::vector<std::vector<unsigned int> > individual_simulations;
                for( size_t i = 0; i < agents.size(); i++ )
                {
                    std::vector <unsigned int> path;
                    for( size_t j = 0; j < plan.size(); j++ )
                    {
                        if( plan[j].first == agents[i].getID() )
                        {
                            std::vector <unsigned int> sim_path = plan[j].second;
                            if( path.size() == 0 )
                                path.insert(path.end(), sim_path.begin(), sim_path.end());
                            else
                                path.insert(path.end(), ++sim_path.begin(), sim_path.end());
                        }
                    }

                    individual_simulations.push_back(path);
                }

                int total_steps = 0;
                std::vector <int> moves_per_robot;
                for( size_t i = 0; i < individual_simulations.size(); i++ )
                {
                    int moves = 0;
                    int prev = -1;
                    for( size_t j = 0; j < individual_simulations[i].size(); j++ )
                    {
                        if( (int)individual_simulations[i][j] != prev )
                            moves++;
                        prev = individual_simulations[i][j];
                    }

                    moves_per_robot.push_back(std::max(moves - 1, 0));
                    total_steps += (std::max(moves - 1, 0));
                }

                //                PRX_DEBUG_COLOR("Total number of edges traversed - " << total_steps, PRX_TEXT_CYAN);
                return total_steps;

                // Write the results
                //    for (size_t i = 0; i < individual_simulations.size (); i++)
                //    {
                //        std::cout << "Agent " << agents[i].getID () << " - steps: " << moves_per_robot[i] << std::endl;
                //    }
            }

            bool verifySolutionSolver(std::vector<unsigned int> pos, const PaS::PushAndSwap::Plan& plan)
            {
                for( size_t i = 0; i < plan.size(); ++i )
                {
                    //        std::cout << "agent : " << plan[i].first << "  start : " << pos[plan[i].first] << "  plan first : " << plan[i].second.front() <<std::endl;
                    if( plan[i].second.front() != pos[plan[i].first] )
                    {
                        std::cout << "[" << i << "] agent : " << plan[i].first << "  start : " << pos[plan[i].first] << "  plan first : " << plan[i].second.front() << std::endl;
                        return false;
                    }
                    for( size_t j = 0; j < plan[i].second.size(); ++j )
                    {
                        for( size_t k = 0; k < pos.size(); ++k )
                        {
                            if( k == plan[i].first ) continue;
                            if( pos[k] == plan[i].second[j] )
                            {
                                std::cout << "[" << i << "," << j << "|" << plan[i].second.size() << "] agent : " << plan[i].first << "  pos[k] : " << pos[k] << "  plan first : " << plan[i].second[j] << std::endl;
                                return false;
                            }
                        }
                    }
                    pos[plan[i].first] = plan[i].second.back();
                }
                return true;
            }

            bool solution_validation(const undirected_graph_t* tree, std::vector<pebble_step_t>* solution, const pebble_assignment_t assignment, const pebble_assignment_t t_assignment)
            {
                //    return true;
                pebble_assignment_t assign = assignment;
                //    PRX_WARN_S("solution new  size: " << solution->size() - last_solution_check);
                undirected_vertex_index_t v_prev, v_new, v_assign;
                bool is_path_valid = true;
                //
                //    PRX_DEBUG_S("assignment at the beginning");
                //    print_assignment(tree,assignment);
                //    PRX_DEBUG_S("assign at the beginning");
                //    print_assignment(tree,assign);
                for( int s = 0; s < (int)solution->size(); ++s )
                {
                    //        PRX_DEBUG_S(s);
                    int pebble_id = solution->at(s).first;
                    v_assign = assign.get_position(pebble_id);
                    v_prev = solution->at(s).second[0];

                    //        if( print_results )
                    //            PRX_DEBUG_S("pebble " << pebble_id << "   moving from : " << print_point(tree, v_prev) << " -> " << print_point(tree, solution->at(s).second.back()) << "  steps: " << solution->at(s).second.size());
                    //        if( v_assign != v_prev )
                    //        {
                    //            //            PRX_ERROR_S(s << ") pebble : " << solution->at(s).first << "  start: " << assign.get_position(solution->at(s).first) << "   thinks:" << v_prev);
                    //            PRX_ERROR_S(s << ") pebble : " << pebble_id << "  start: " << print_point(tree, v_assign) << "   thinks:" << print_point(tree, v_prev));
                    //        }
                    for( int i = 1; i < (int)solution->at(s).second.size(); ++i )
                    {
                        v_new = solution->at(s).second[i];
                        if( v_prev != v_new && !boost::edge(v_prev, v_new, tree->graph).second )
                        {
                            //                PRX_ERROR_S("ERROR edge(" << i << "): " << print_point(tree, v_prev) << " -> " << print_point(tree, v_new));
                            is_path_valid = false;
                        }
                        v_prev = v_new;
                    }
                    assign.change_position(pebble_id, solution->at(s).second.back());

                }
                //                if( assign != t_assignment )
                //                {
                //                    PRX_ERROR_S("Pebbles are not in their final position");
                //                    return false;
                //                }
                return is_path_valid;
            }

            bool pebble_solver_tp_t::execute()
            {
                sys_clock_t clock;

                char* w = std::getenv("PRACSYS_PATH");
                std::string dir(w);
                dir += ("/prx_output/pebble_output/");
                std::string file = dir + output_file + "/" + test_name + ".txt";

                datafile.open(file.c_str(), std::ios::out);
                datafile << test_name;
                datafile << "\t" << boost::num_vertices(graph.graph) << "\t" << boost::num_edges(graph.graph) << "\t" << k;
                double time = 0.0;
                size_t solution_size = 0;
                bool is_valid_path = false;

                PaS::PushAndSwap::Plan plan_for_pas;

                PRX_DEBUG_COLOR("Pebble Solver Execute", PRX_TEXT_GREEN);
                clock.reset();
                bool got_solution = p_solver->execute(&solution_path, &graph);
                time = clock.measure();

                if( parallel_version )
                {
                    solution_size = compute_solution_length(solution_path);                    
                    std::vector<int> steps;
                    dynamic_cast<ppmt_solver_t*>(p_solver)->get_parallel_steps(steps);
                    PRX_DEBUG_COLOR("PARALLEL PEBBLE SOLVER FOUND SOLUTION in " << time << " secs" << "   size: " << solution_size << "  steps : " << steps.size(), PRX_TEXT_BROWN);
                    resolve_parallel_query(steps);
                    datafile << std::endl;
                    datafile.close();
                    return got_solution;
                }

                if( got_solution )
                {

                    
                    is_valid_path = p_solver->is_valid_solution(&graph, &solution_path, s_assign);
                    solution_size = compute_solution_length(solution_path);
                    //                    solution_validation(&graph, &solution_path, s_assign, t_assign);

                    datafile << "\t" << is_valid_path;
                    datafile << "\t" << time;
                    datafile << "\t" << solution_size;

                    PRX_DEBUG_COLOR("PEBBLE SOLVER FOUND SOLUTION in " << time << " secs" << "   size: " << solution_size, PRX_TEXT_BROWN);


                    if( with_push_and_swap )
                    {
                        for( unsigned int i = 0; i < solution_path.size(); ++i )
                        {
                            //        PRX_DEBUG_S(solution_path[i].first << ")  sol_size : " << sol_size << "     path_size : " << solution_path[i].second.size() - 1 <<  "    first position : " << robot_state_space->print_point(graph.get_vertex_as<undirected_node_t>(solution_path[i].second[0])->point,2) );
                            std::vector<unsigned int> path;
                            for( unsigned int j = 0; j < solution_path[i].second.size(); ++j )
                            {

                                path.push_back(pebble_to_pas[solution_path[i].second[j]]);
                            }
                            plan_for_pas.push_back(std::make_pair(solution_path[i].first, path));
                        }
                    }

                    clock.reset();
                    smoother(solution_path);
                    time = clock.measure();
                    is_valid_path = p_solver->is_valid_solution(&graph, &solution_path, s_assign);
                    solution_size = compute_solution_length(solution_path);

                    datafile << "\t" << is_valid_path;
                    datafile << "\t" << time;
                    datafile << "\t" << solution_size;
                    PRX_DEBUG_COLOR("PEBBLE SOLVER SMOOTHED SOLUTION in " << time << " secs" << "   size: " << solution_size, PRX_TEXT_CYAN);
                }
                else
                {
                    time = clock.measure();
                    datafile << "\t" << -1 << "\t" << time << "PEBBLE_SOLVER_FAILED!!!";
                    PRX_ERROR_S("PEBBLE SOLVER FAILD in " << time << " secs");
                    smoother(solution_path);
                }

                if( with_push_and_swap )
                {
                    //                    PRX_DEBUG_COLOR("pas :  robots : " << pas_robots.size() << "   graph |V|: " << pas_graph->numNodes() << "    |E|:" << pas_graph->numEdges(),PRX_TEXT_CYAN );
                    PaS::PushAndSwap pas(pas_robots, pas_graph);
                    clock.reset();
                    if( pas.solve() )
                    {
                        time = clock.measure();
                        //                        PRX_INFO_S("PaS FOUND SOLUTION in " << time << " secs");
                        pas_plan = pas.getPlan();

                        //                        std::cout << "======== Unsmoothed plan ========" << std::endl;
                        solution_size = getMetricsSolver(pas_robots, pas_plan);
                        PRX_DEBUG_COLOR("PaS SOLUTION in " << time << " secs    size: " << solution_size, PRX_TEXT_GREEN);
                        //                        std::cout << "======== Smoothed plan ========" << std::endl;
                        clock.reset();
                        pas_plan = PaS::PushAndSwap::smooth(pas_plan);
                        time = clock.measure();
                        solution_size = getMetricsSolver(pas_robots, pas_plan);
                        PRX_DEBUG_COLOR("PaS SMOOTHED SOLUTION in " << time << " secs    size: " << solution_size, PRX_TEXT_CYAN);
                    }
                    else
                    {
                        PRX_ERROR_S("PaS FAILD in " << clock.measure() << " secs");
                    }

                    //                    clock.reset();
                    //                    plan_for_pas = PaS::PushAndSwap::smooth(plan_for_pas);
                    //                    time = clock.measure();
                    //                    datafile << "\t" << time;
                    //
                    //                    solution_size = getMetricsSolver(pas_robots, plan_for_pas);
                    //                    datafile << "\t" << solution_size;
                    //                    PRX_DEBUG_COLOR("PaS SMOOTHED SOLUTION in " << time << " secs    size: " << solution_size, PRX_TEXT_GREEN);
                    //
                    //                    std::vector<unsigned int> starts;
                    //                    for( size_t i = 0; i < pas_robots.size(); ++i )
                    //                        starts.push_back(pas_robots[i].getStart());
                    //                    if( !verifySolutionSolver(starts, plan_for_pas) )
                    //                    {
                    //                        datafile << "\t" << " PaS SMOOTHED SOLUTION IS NOT CORRECT!!";
                    //                        PRX_DEBUG_COLOR("ERROR: PAS SMOOTHED SOLUTION IS NOT CORRECT!!", PRX_TEXT_RED);
                    //                    }
                }



                resolve_query();

                datafile << std::endl;
                datafile.close();

                //                //        PRX_INFO_S("solution path size : " << solution_path.size());
                ////
                ////                                PaS::PushAndSwap::Plan plan_from_smooth;
                ////                                for( unsigned int i = 0; i < solution_path.size(); ++i )
                ////                                {
                ////                                    sol_size += solution_path[i].second.size() - 1;
                ////                                    //        PRX_DEBUG_S(solution_path[i].first << ")  sol_size : " << sol_size << "     path_size : " << solution_path[i].second.size() - 1 <<  "    first position : " << robot_state_space->print_point(graph.get_vertex_as<undirected_node_t>(solution_path[i].second[0])->point,2) );
                ////                                    std::vector<unsigned int> path;
                ////                                    for( unsigned int j = 0; j < solution_path[i].second.size(); ++j )
                ////                                    {
                ////                                        path.push_back(pebble_to_pas[solution_path[i].second[j]]);
                ////                                    }
                ////                                    plan_from_smooth.push_back(std::make_pair(solution_path[i].first, path));
                ////                
                ////                                }
                ////                                PRX_DEBUG_COLOR("Full path size :  " << sol_size, PRX_TEXT_BROWN);
                ////                                datafile << "\t" << sol_size;
                ////                
                ////                
                ////                                std::vector<unsigned int> starts;
                ////                                for( size_t i = 0; i < pas_robots.size(); ++i )
                ////                                    starts.push_back(pas_robots[i].getStart());
                ////                                if( !verifySolutionSolver(starts, plan_from_smooth) )
                ////                                {
                ////                                    datafile << "\t" << " SOLUTION IS NOT CORRECT!!";
                ////                                    PRX_ERROR_S("ERROR: SOLUTION IS NOT CORRECT!!");
                ////                                }
                ////                
                ////                                clock.reset();
                ////                                plan_from_smooth = PaS::PushAndSwap::smooth(plan_from_smooth);
                ////                                double time = clock.measure();
                ////                                datafile << "\t" << time;
                //                //
                //                //                if( !verifySolutionSolver(starts, plan_from_smooth) )
                //                //                {
                //                //                    datafile << "\t" << " SOLUTION IS NOT CORRECT!!";
                //                //                    PRX_ERROR_S("ERROR: SMOOTHED SOLUTION IS NOT CORRECT!!");
                //                //                }
                //                //                //
                //                //                PRX_DEBUG_COLOR("PEBBLE SOLVER SMOOTHED SOLUTION in " << time << " secs", PRX_TEXT_CYAN);
                //
                //                //                PRX_DEBUG_COLOR("AFTER RYAN ", PRX_TEXT_GREEN);
                //                //                for( int ii = 0; ii < (int)plan_from_smooth.size(); ++ii )
                //                //                {
                //                //                    std::cout << plan_from_smooth[ii].first << ":";
                //                //                    for( int jj = 0; jj < (int)plan_from_smooth[ii].second.size(); ++jj )
                //                //                    {
                //                //                        std::cout << " " << plan_from_smooth[ii].second[jj];
                //                //                    }
                //                //                    std::cout << std::endl;
                //                //                }
                //
                //
                //                datafile << "\t" << getMetricsSolver(pas_robots, plan_from_smooth);
                //                if( time >= 180 )
                //                    time = 0.0;
                //                datafile << "\t" << time + time;
                //
                //                clock.reset();
                //                smoother(solution_path);
                //                double time3 = clock.measure();
                //
                //                PRX_DEBUG_COLOR("PEBBLE SOLVER MY SMOOTHED SOLUTION in " << time3 << " secs", PRX_TEXT_BLUE);
                //                sol_size = 0;
                //
                //                for( unsigned int i = 0; i < solution_path.size(); ++i )
                //                {
                //                    sol_size += solution_path[i].second.size();
                //                    //        PRX_DEBUG_S(solution_path[i].first << ")  sol_size : " << sol_size << "     path_size : " << solution_path[i].second.size() - 1 <<  "    first position : " << robot_state_space->print_point(graph.get_vertex_as<undirected_node_t>(solution_path[i].second[0])->point,2) );
                //                    std::vector<unsigned int> path;
                //                    for( unsigned int j = 0; j < solution_path[i].second.size(); ++j )
                //                    {
                //                        path.push_back(pebble_to_pas[solution_path[i].second[j]]);
                //                    }
                //                    plan_to_smooth.push_back(std::make_pair(solution_path[i].first, path));
                //
                //                }
                //                sol_size -= s_assign.size();
                //                PRX_DEBUG_COLOR("Full path size :  " << sol_size, PRX_TEXT_BLUE);
                //                datafile << "\t" << sol_size;
                //
                //                std::vector<unsigned int> starts2;
                //                for( size_t i = 0; i < pas_robots.size(); ++i )
                //                    starts2.push_back(pas_robots[i].getStart());
                //                if( !verifySolutionSolver(starts2, plan_to_smooth) )
                //                {
                //                    datafile << "\t" << " SOLUTION IS NOT CORRECT!!";
                //                    PRX_ERROR_S("ERROR: SOLUTION IS NOT CORRECT!!");
                //                }
                //
                //                //    clock.reset();
                //                //    PaS::PushAndSwap pas(pas_robots, pas_graph);
                //                //    if( pas.solve() )
                //                //    {
                //                //        time = clock.measure();
                //                //        PRX_INFO_S("PaS FOUND SOLUTION in " << time << " secs");
                //                //
                //                //        pas_plan = pas.getPlan();
                //                //
                //                //        std::cout << "======== Unsmoothed plan ========" << std::endl;
                //                //
                //                //        //        std::vector<unsigned int> starts;
                //                //        //        for (size_t i = 0; i < pas_robots.size(); ++i)
                //                //        //            starts.push_back(pas_robots[i].getStart());
                //                //        getMetricsSolver(pas_robots, pas_plan);
                //                //        std::cout << "======== Smoothed plan ========" << std::endl;
                //                //        clock.reset();
                //                //        pas_plan = PaS::PushAndSwap::smooth(pas_plan);
                //                //        time = clock.measure();
                //                //        PRX_INFO_S("PaS SMOOTHED SOLUTION in " << time << " secs");
                //                //        getMetricsSolver(pas_robots, pas_plan);
                //                //    }
                //                //    else
                //                //    {
                //                //        PRX_ERROR_S("PaS FAILD in " << clock.measure() << " secs");
                //                //    }
                //                resolve_query();
                //                datafile << std::endl;
                //                datafile.close();
                return true;
            }

            bool pebble_solver_tp_t::find_solution()
            {
                sys_clock_t clock;

                char* w = std::getenv("PRACSYS_PATH");
                std::string dir(w);
                dir += ("/prx_output/pebble_output/");
                std::string file = dir + output_file + "/" + test_name + ".txt";

                PRX_DEBUG_COLOR("outputfile : " << file, PRX_TEXT_BROWN);

                //                datafile.open("/Users/TDK/Research/pracsys/prx_input/experiments/pebble_tests/pmg_solver/data_PMG.txt", std::ios::app);
                datafile.open(file.c_str(), std::ios::app);
                datafile << test_name;
                datafile << "\t" << boost::num_vertices(graph.graph) << "\t" << boost::num_edges(graph.graph) << "\t" << s_assign.size();
                double time = 0.0;
                bool is_valid_path = false;
                PRX_DEBUG_COLOR("Pebble Solver Execute", PRX_TEXT_GREEN);
                clock.reset();
                if( p_solver->execute(&solution_path, &graph) )
                {
                    time = clock.measure();
                    is_valid_path = solution_validation(&graph, &solution_path, s_assign, t_assign);
                    datafile << "\t" << is_valid_path;
                    datafile << "\t" << time;
                    PRX_DEBUG_COLOR("PEBBLE SOLVER FOUND SOLUTION in " << time << " secs" << "  and the path is valid : " << is_valid_path, PRX_TEXT_BROWN);
                }
                else
                {
                    time = clock.measure();
                    datafile << "\t" << -1 << "\t" << time;
                    PRX_ERROR_S("PEBBLE SOLVER FAILD in " << time << " secs");
                    smoother(solution_path);

                    for( unsigned int i = 0; i < solution_path.size(); ++i )
                    {
                        std::vector<unsigned int> path;
                        for( unsigned int j = 0; j < solution_path[i].second.size(); ++j )
                        {
                            path.push_back(pebble_to_pas[solution_path[i].second[j]]);
                        }
                        plan_to_smooth.push_back(std::make_pair(solution_path[i].first, path));

                    }
                    resolve_pas_query();
                    return false;
                }
                //        PRX_INFO_S("solution path size : " << solution_path.size());
                int sol_size = 0;
                PaS::PushAndSwap::Plan plan_from_smooth;
                for( unsigned int i = 0; i < solution_path.size(); ++i )
                {
                    sol_size += solution_path[i].second.size() - 1;
                    //        PRX_DEBUG_S(solution_path[i].first << ")  sol_size : " << sol_size << "     path_size : " << solution_path[i].second.size() - 1 <<  "    first position : " << robot_state_space->print_point(graph.get_vertex_as<undirected_node_t>(solution_path[i].second[0])->point,2) );
                    std::vector<unsigned int> path;
                    for( unsigned int j = 0; j < solution_path[i].second.size(); ++j )
                    {
                        path.push_back(pebble_to_pas[solution_path[i].second[j]]);
                    }
                    plan_from_smooth.push_back(std::make_pair(solution_path[i].first, path));

                }
                PRX_DEBUG_COLOR("Full path size :  " << sol_size, PRX_TEXT_BROWN);
                datafile << "\t" << sol_size;


                std::vector<unsigned int> starts;
                for( size_t i = 0; i < pas_robots.size(); ++i )
                    starts.push_back(pas_robots[i].getStart());
                if( !verifySolutionSolver(starts, plan_from_smooth) )
                {
                    datafile << "\t" << " SOLUTION IS NOT CORRECT!!";
                    PRX_ERROR_S("ERROR: SOLUTION IS NOT CORRECT!!");
                }

                clock.reset();
                plan_from_smooth = PaS::PushAndSwap::smooth(plan_from_smooth);
                time = clock.measure();
                datafile << "\t" << time;

                if( !verifySolutionSolver(starts, plan_from_smooth) )
                {
                    datafile << "\t" << " SOLUTION IS NOT CORRECT!!";
                    PRX_ERROR_S("ERROR: SMOOTHED SOLUTION IS NOT CORRECT!!");
                }
                //
                PRX_DEBUG_COLOR("PEBBLE SOLVER SMOOTHED SOLUTION in " << time << " secs", PRX_TEXT_CYAN);

                //                PRX_DEBUG_COLOR("AFTER RYAN ", PRX_TEXT_GREEN);
                //                for( int ii = 0; ii < (int)plan_from_smooth.size(); ++ii )
                //                {
                //                    std::cout << plan_from_smooth[ii].first << ":";
                //                    for( int jj = 0; jj < (int)plan_from_smooth[ii].second.size(); ++jj )
                //                    {
                //                        std::cout << " " << plan_from_smooth[ii].second[jj];
                //                    }
                //                    std::cout << std::endl;
                //                }


                datafile << "\t" << getMetricsSolver(pas_robots, plan_from_smooth);
                if( time >= 180 )
                    time = 0.0;
                datafile << "\t" << time + time;

                clock.reset();
                smoother(solution_path);
                double time3 = clock.measure();

                PRX_DEBUG_COLOR("PEBBLE SOLVER MY SMOOTHED SOLUTION in " << time3 << " secs", PRX_TEXT_BLUE);
                sol_size = 0;

                for( unsigned int i = 0; i < solution_path.size(); ++i )
                {
                    sol_size += solution_path[i].second.size();
                    //        PRX_DEBUG_S(solution_path[i].first << ")  sol_size : " << sol_size << "     path_size : " << solution_path[i].second.size() - 1 <<  "    first position : " << robot_state_space->print_point(graph.get_vertex_as<undirected_node_t>(solution_path[i].second[0])->point,2) );
                    std::vector<unsigned int> path;
                    for( unsigned int j = 0; j < solution_path[i].second.size(); ++j )
                    {
                        path.push_back(pebble_to_pas[solution_path[i].second[j]]);
                    }
                    plan_to_smooth.push_back(std::make_pair(solution_path[i].first, path));

                }
                sol_size -= s_assign.size();
                PRX_DEBUG_COLOR("Full path size :  " << sol_size, PRX_TEXT_BLUE);
                datafile << "\t" << sol_size;

                std::vector<unsigned int> starts2;
                for( size_t i = 0; i < pas_robots.size(); ++i )
                    starts2.push_back(pas_robots[i].getStart());
                if( !verifySolutionSolver(starts2, plan_to_smooth) )
                {
                    datafile << "\t" << " SOLUTION IS NOT CORRECT!!";
                    PRX_ERROR_S("ERROR: SOLUTION IS NOT CORRECT!!");
                }

                //    clock.reset();
                //    PaS::PushAndSwap pas(pas_robots, pas_graph);
                //    if( pas.solve() )
                //    {
                //        time = clock.measure();
                //        PRX_INFO_S("PaS FOUND SOLUTION in " << time << " secs");
                //
                //        pas_plan = pas.getPlan();
                //
                //        std::cout << "======== Unsmoothed plan ========" << std::endl;
                //
                //        //        std::vector<unsigned int> starts;
                //        //        for (size_t i = 0; i < pas_robots.size(); ++i)
                //        //            starts.push_back(pas_robots[i].getStart());
                //        getMetricsSolver(pas_robots, pas_plan);
                //        std::cout << "======== Smoothed plan ========" << std::endl;
                //        clock.reset();
                //        pas_plan = PaS::PushAndSwap::smooth(pas_plan);
                //        time = clock.measure();
                //        PRX_INFO_S("PaS SMOOTHED SOLUTION in " << time << " secs");
                //        getMetricsSolver(pas_robots, pas_plan);
                //    }
                //    else
                //    {
                //        PRX_ERROR_S("PaS FAILD in " << clock.measure() << " secs");
                //    }
                resolve_pas_query();
                datafile << std::endl;
                datafile.close();
                return true;
            }

            const statistics_t* pebble_solver_tp_t::get_statistics()
            {
                return NULL;
            }

            bool pebble_solver_tp_t::succeeded() const
            {
                return true;
            }

            void pebble_solver_tp_t::link_query(plan::query_t* in_query)
            {
                //this will overwrite any query read from input
                if( tp_query != NULL )
                    delete tp_query;
                tp_query = dynamic_cast<pebble_test_query_t*>(in_query);
            }

            void pebble_solver_tp_t::resolve_query()
            {
                undirected_vertex_index_t v_new;

                std::vector<double> new_point(full_control_space->get_dimension());
                control_t* new_control = full_control_space->clone_point(tp_query->plan.back().control);

                for( size_t i = 0; i < full_control_space->get_dimension(); ++i )
                    new_point[i] = new_control->at(i);

                for( size_t i = 0; i < solution_path.size(); ++i )
                {
                    int index = solution_path[i].first * robot_control_size;

                    for( size_t j = 0; j < solution_path[i].second.size(); ++j )
                    {
                        v_new = solution_path[i].second[j];
                        space_point_t* graph_point = graph.get_vertex_as<undirected_node_t > (v_new)->point;
                        for( int c = 0; c < robot_control_size; ++c )
                        {
                            new_point[index + c] = graph_point->at(c);
                        }

                        full_control_space->set_from_vector(new_point, new_control);
                        tp_query->plan.copy_onto_back(new_control, simulation::simulation_step * 6);

                    }
                }
            }

            void pebble_solver_tp_t::resolve_parallel_query(std::vector<int> steps)
            {

                std::vector<double> new_point(full_control_space->get_dimension());
                control_t* new_control = full_control_space->clone_point(tp_query->plan.back().control);

                for( size_t i = 0; i < full_control_space->get_dimension(); ++i )
                    new_point[i] = new_control->at(i);

                size_t st = 0;
                size_t i = 0;

                for( st = 0; st < steps.size(); st++ )
                {
                    while( i <= steps[st] )
                    {
                        int index = solution_path[i].first * robot_control_size;
                        space_point_t* graph_point = graph.get_vertex_as<undirected_node_t > (solution_path[i].second[0])->point;
                        for( int c = 0; c < robot_control_size; ++c )
                        {
                            new_point[index + c] = graph_point->at(c);
                        }

                        i++;
                    }

                    full_control_space->set_from_vector(new_point, new_control);
                    tp_query->plan.copy_onto_back(new_control, simulation::simulation_step * 6);
                }
            }

            void pebble_solver_tp_t::resolve_pas_query()
            {
                PRX_INFO_S("in the resolve query");
                undirected_vertex_index_t v_source;
                undirected_vertex_index_t v_target;
                //    
                ////    foreach(undirected_vertex_index_t v, boost::vertices(graph.graph))
                ////    {
                ////        PRX_DEBUG_S(v << " ) " << robot_state_space->print_point(graph.get_vertex_as<undirected_node_t>(v)->point,2));
                ////    }

                //    foreach(PaS::PushAndSwap::Path step, plan_to_smooth)
                //    {
                //        PRX_INFO_S("agent : " << step.first);
                //
                //        foreach(unsigned int vs, step.second)
                //        {
                //            std::cout << vs << " , ";
                //        }
                //        std::cout << std::endl;
                //
                //    }
                for( size_t i = 0; i < plan_to_smooth.size(); ++i )
                {
                    v_source = pas_to_pebble[plan_to_smooth[i].second[0]];
                    int index = plan_to_smooth[i].first * robot_control_size;


                    for( size_t j = 0; j < plan_to_smooth[i].second.size(); ++j )
                    {
                        v_target = pas_to_pebble[plan_to_smooth[i].second[j]];
                        control_t* new_control = full_control_space->clone_point(tp_query->plan.back().control);
                        space_point_t* point = graph.get_vertex_as<undirected_node_t > (v_target)->point;
                        //            PRX_DEBUG_S( plan_to_smooth[i].first  << " ) v_target point (" << v_target << " ) " << robot_state_space->print_point(graph.get_vertex_as<undirected_node_t>(v_target)->point,2));
                        for( int c = 0; c < robot_control_size; ++c )
                        {
                            new_control->at(index + c) = point->at(c);
                        }
                        //            tp_query->plan.steps.push_back(plan_step_t(new_control, simulation::simulation_step * 50));

                        tp_query->plan.copy_onto_back(new_control, simulation::simulation_step * 4);

                        v_source = v_target;
                    }



                    //                    if( plan_to_smooth[i].second.size() == 1 )
                    //                    {
                    //                        PRX_DEBUG_COLOR("size 1 plan", PRX_TEXT_LIGHTGRAY);
                    //                        v_target = pas_to_pebble[plan_to_smooth[i].second[0]];
                    //                        control_t* new_control = full_control_space->clone_point(tp_query->plan.back().control);
                    //                        space_point_t* point = graph.get_vertex_as<undirected_node_t > (v_target)->point;
                    //                        //            PRX_DEBUG_S( plan_to_smooth[i].first  << " ) v_target point (" << v_target << " ) " << robot_state_space->print_point(graph.get_vertex_as<undirected_node_t>(v_target)->point,2));
                    //                        for( int c = 0; c < robot_control_size; ++c )
                    //                        {
                    //                            new_control->at(index + c) = point->at(c);
                    //                        }
                    //                        //            tp_query->plan.steps.push_back(plan_step_t(new_control, simulation::simulation_step * 50));
                    //
                    //                        tp_query->plan.copy_onto_back(new_control, simulation::simulation_step * 4);
                    //
                    //
                    //                    }
                    //                    else
                    //                    {
                    //                        for( size_t j = 1; j < plan_to_smooth[i].second.size(); ++j )
                    //                        {
                    //                            v_target = pas_to_pebble[plan_to_smooth[i].second[j]];
                    //                            control_t* new_control = full_control_space->clone_point(tp_query->plan.back().control);
                    //                            space_point_t* point = graph.get_vertex_as<undirected_node_t > (v_target)->point;
                    //                            //            PRX_DEBUG_S( plan_to_smooth[i].first  << " ) v_target point (" << v_target << " ) " << robot_state_space->print_point(graph.get_vertex_as<undirected_node_t>(v_target)->point,2));
                    //                            for( int c = 0; c < robot_control_size; ++c )
                    //                            {
                    //                                new_control->at(index + c) = point->at(c);
                    //                            }
                    //                            //            tp_query->plan.steps.push_back(plan_step_t(new_control, simulation::simulation_step * 50));
                    //                            if( boost::edge(v_source, v_target, graph.graph).second )
                    //                            {
                    //                                undirected_edge_index_t e = boost::edge(v_source, v_target, graph.graph).first;
                    //                                tp_query->plan.copy_onto_back(new_control, simulation::simulation_step * graph.get_edge_as<pebble_solver_edge_t > (e)->steps);
                    //                            }
                    //                            else
                    //                            {
                    //
                    //                                tp_query->plan.copy_onto_back(new_control, simulation::simulation_step);
                    //                                if( v_source != v_target )
                    //                                {
                    //                                    PRX_ERROR_S("Missing edge: " << v_source << "  -  " << v_target);
                    //                                    datafile << "\t" << "Missing edge";
                    //                                }
                    //                            }
                    //                            v_source = v_target;
                    //                        }
                    //                    }
                }
                //    PRX_WARN_S(tp_query->plan.size());
                return;
                for( size_t i = 0; i < solution_path.size(); ++i )
                {
                    v_source = solution_path[i].second[0];
                    int index = solution_path[i].first * robot_control_size;
                    for( size_t j = 1; j < solution_path[i].second.size(); ++j )
                    {
                        v_target = solution_path[i].second[j];
                        control_t* new_control = full_control_space->clone_point(tp_query->plan.back().control);
                        space_point_t* point = graph.get_vertex_as<undirected_node_t > (v_target)->point;
                        PRX_DEBUG_S(solution_path[i].first << " ) v_target point (" << v_target << " ) " << robot_state_space->print_point(graph.get_vertex_as<undirected_node_t > (v_target)->point, 2));
                        for( int c = 0; c < robot_control_size; ++c )
                        {
                            new_control->at(index + c) = point->at(c);
                        }
                        //            tp_query->plan.steps.push_back(plan_step_t(new_control, simulation::simulation_step * 50));
                        if( boost::edge(v_source, v_target, graph.graph).second )
                        {
                            undirected_edge_index_t e = boost::edge(v_source, v_target, graph.graph).first;
                            tp_query->plan.copy_onto_back(new_control, simulation::simulation_step * graph.get_edge_as<pebble_solver_edge_t > (e)->steps);
                        }
                        else
                        {
                            tp_query->plan.copy_onto_back(new_control, simulation::simulation_step);
                            PRX_ERROR_S("Missing edge: " << v_source << "  -  " << v_target);
                        }
                        v_source = v_target;
                    }
                }

                return;
                for( size_t i = 0; i < pas_plan.size(); ++i )
                {
                    int index = pas_plan[i].first * robot_control_size;
                    undirected_vertex_index_t v_source = pas_to_pebble[pas_plan[i].second[0]];
                    undirected_vertex_index_t v_target;
                    space_point_t* target;
                    //        PRX_INFO_S("for agent : " << pas_plan[i].first);
                    //        std::cout << pas_plan[i].second[0] << "  " ;
                    for( size_t p = 1; p < pas_plan[i].second.size(); ++p )
                    {
                        v_target = pas_to_pebble[pas_plan[i].second[p]];

                        target = graph.get_vertex_as<undirected_node_t > (v_target)->point;
                        //            std::cout << pas_plan[i].second[i] << " (" << robot_state_space->print_point(target,1) << ")  ";

                        control_t* new_control = full_control_space->clone_point(tp_query->plan.back().control);

                        for( int c = 0; c < robot_control_size; ++c )
                        {
                            new_control->at(index + c) = target->at(c);
                        }

                        //            PRX_DEBUG_S("v_source : " << graph.get_vertex_as<tree_pebble_node_t>(v_source)->print_point(robot_state_space) << " -> " << graph.get_vertex_as<tree_pebble_node_t>(v_target)->print_point(robot_state_space));
                        if( boost::edge(v_source, v_target, graph.graph).second )
                        {
                            undirected_edge_index_t e = boost::edge(v_source, v_target, graph.graph).first;
                            tp_query->plan.copy_onto_back(new_control, simulation::simulation_step * graph.get_edge_as<pebble_solver_edge_t > (e)->steps);
                        }

                        v_source = v_target;
                    }
                }
            }

            void pebble_solver_tp_t::build_graphs()
            {
                undirected_vertex_index_t v;

                foreach(node_id_t vec, nodes_to_add)
                {
                    v = p_solver->add_new_vertex(&graph);

                    graph[v]->point = robot_state_space->alloc_point();
                    robot_state_space->set_from_vector(vec.second, graph[v]->point);
                    graph_node_id[vec.first] = v;
                    PRX_DEBUG_COLOR("point : " << robot_state_space->print_point(graph[v]->point, 3), PRX_TEXT_BROWN);
                }

                double dist;

                foreach(edge_id_t e, edges_to_add)
                {
                    if( real_distances )
                        dist = metric->distance_function(graph[graph_node_id[e.first]]->point, graph[graph_node_id[e.second]]->point);
                    else
                        dist = 1;

                    PRX_DEBUG_COLOR("edge: " << e.first << " - " << e.second << "   source :" << graph_node_id[e.first], PRX_TEXT_CYAN);

                    undirected_edge_index_t edge = graph.add_edge<pebble_solver_edge_t > (graph_node_id[e.first], graph_node_id[e.second], dist);
                    graph.get_edge_as<pebble_solver_edge_t > (edge)->steps = 4; //plan.length()/simulation::simulation_step;
                }
            }

            void pebble_solver_tp_t::setup_PaS()
            {
                int new_pas_v;
                undirected_vertex_index_t v;

                for( int i = 0; i < graph_node_id.size(); ++i )
                {
                    new_pas_v = pas_graph->add_node();
                    v = graph_node_id[i];
                    pebble_to_pas[v] = new_pas_v;
                    pas_to_pebble[new_pas_v] = v;
                }

                PRX_DEBUG_COLOR("pebble_to_pas size: " << pebble_to_pas.size(), PRX_TEXT_CYAN);

                foreach(undirected_edge_index_t e, boost::edges(graph.graph))
                {
                    pas_graph->add_edge(pebble_to_pas[ boost::source(e, graph.graph)], pebble_to_pas[ boost::target(e, graph.graph)], 1);
                    PRX_DEBUG_COLOR("edge between :" << pebble_to_pas[boost::source(e, graph.graph)] << " - " << pebble_to_pas[boost::target(e, graph.graph)], PRX_TEXT_CYAN);
                }

                for( int robot_id = 0; robot_id < k; ++robot_id )
                {
                    PRX_DEBUG_COLOR("robot " << robot_id << "   start: " << pebble_to_pas[s_assign.get_position(robot_id)] << "  end:" << pebble_to_pas[t_assign.get_position(robot_id)], PRX_TEXT_RED);
                    pas_robots.push_back(PaS::Agent(robot_id, pebble_to_pas[s_assign.get_position(robot_id)], pebble_to_pas[t_assign.get_position(robot_id)]));
                }

            }

            void pebble_solver_tp_t::update_vis_info() const
            {
                PRX_INFO_S("pebble_solver_tp_t update_vis_info");

                std::vector<geometry_info_t> geoms;
                std::vector<config_t> configs;
                hash_t<std::string, std::vector<double> > map_params;
                std::vector<double> params;

                if( visualize_graph )
                {
                    model->use_context("single_robot_space");
                    int count = 0;
                    std::vector<std::string> system_names;
                    system_names.push_back(visualization_body);

                    foreach(undirected_edge_index_t e, boost::edges(graph.graph))
                    {
                        std::string name = visualization_graph_name + "/edge_" + int_to_str(count);
                        params.clear();

                        map_params.clear();
                        ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->compute_configs(graph[boost::source(e, graph.graph)]->point, system_names, map_params);
                        params.insert(params.end(), map_params[system_names[0]].begin(), map_params[system_names[0]].end());

                        map_params.clear();
                        ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->compute_configs(graph[boost::target(e, graph.graph)]->point, system_names, map_params);
                        params.insert(params.end(), map_params[system_names[0]].begin(), map_params[system_names[0]].end());

                        geoms.push_back(geometry_info_t(visualization_body, name, PRX_LINESTRIP, params, graph_color));
                        configs.push_back(config_t());
                        count++;
                    }

                    //        ((simulation::communication::visualization_comm_t*)simulation::communication::vis_comm)->send_geometries();
                    ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->visualization_geom_map[visualization_graph_name] = geoms;
                    ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->visualization_configs_map[visualization_graph_name] = configs;
                    geoms.clear();
                    configs.clear();
                    model->use_context("full_space");
                }
            }

            void pebble_solver_tp_t::smoother(std::vector<pebble_step_t>& plan)
            {
                sys_clock_t clock;
                undirected_vertex_index_t v_check;
                int plan_pos;
                int step_pos;
                int robot_id;
                bool pruned;
                clock.reset();

                //                PRX_DEBUG_COLOR("Before smoothing ", PRX_TEXT_MAGENTA);
                //                for( int ii = 0; ii < (int)plan.size(); ++ii )
                //                {
                //                    std::cout << plan[ii].first << ":";
                //                    for( int jj = 0; jj < (int)plan[ii].second.size(); ++jj )
                //                    {
                //                        std::cout << " " << pebble_to_pas[plan[ii].second[jj]];
                //                    }
                //                    std::cout << std::endl;
                //                }
                //                std::cout << std::endl;

                do
                {
                    pruned = false;
                    for( int i = plan.size() - 1; i >= 0 && pruned == false; --i )
                    {
                        robot_id = plan[i].first;
                        for( int j = (int)plan[i].second.size() - 1; j >= 0; --j )
                        {
                            v_check = plan[i].second[j];
                            plan_pos = i;
                            step_pos = -1;
                            //                            PRX_DEBUG_COLOR("first pos is : " << plan_pos << "     (" << j << "/" << plan[i].second.size() << ")", PRX_TEXT_LIGHTGRAY);

                            //First check if in the same plan the pebble is already passing from the same position
                            //more than one time.                            
                            for( int s = j - 1; s >= 0; --s )
                            {
                                if( plan[i].second[s] == v_check )
                                {
                                    step_pos = s;
                                }
                            }

                            smoother_check(plan, robot_id, v_check, plan_pos, step_pos);
                            //                            PRX_DEBUG_COLOR("pos after smooth_check (" << robot_id << "): " << plan_pos << "   step: " << step_pos, PRX_TEXT_LIGHTGRAY);

                            if( step_pos != -1 )
                            {
                                pruned = true;

                                //                                PRX_DEBUG_COLOR("Going to prune (" << robot_id << " | " << v_check << "): " << plan_pos << " -> " << i << "   till the position: " << step_pos, PRX_TEXT_MAGENTA);
                                //                                for( int ii = plan_pos - 5; ii < sz; ++ii )
                                //                                {
                                //                                    std::cout << plan[ii].first << ":";
                                //                                    for( int jj = 0; jj < (int)plan[ii].second.size(); ++jj )
                                //                                    {
                                //                                        std::cout << " " << plan[ii].second[jj];
                                //                                    }
                                //                                    std::cout << std::endl;
                                //                                }

                                if( plan_pos != i )
                                {
                                    //                                    std::cout << "BEGIN : " << (*(plan.begin() + plan_pos)).first << ":";
                                    //                                    for( int jj = 0; jj < (*(plan.begin() + plan_pos)).second.size(); ++jj )
                                    //                                    {
                                    //                                        std::cout << " " << (*(plan.begin() + plan_pos)).second[jj];
                                    //                                    }
                                    //                                    std::cout << std::endl;
                                    //
                                    //                                    std::cout << "END : (" << i << "/" << sz << "): " << (*(plan.end()-(sz - i + 1))).first << ":";
                                    //                                    for( int jj = 0; jj < (*(plan.end()-(sz - i + 1))).second.size(); ++jj )
                                    //                                    {
                                    //                                        std::cout << " " << (*(plan.end()-(sz - i + 1))).second[jj];
                                    //                                    }
                                    //                                    std::cout << std::endl;
                                    //
                                    //
                                    //                                    std::cout << "i plan : (" << i << "/" << sz << "): " << plan[i].first << ":";
                                    //                                    for( int jj = 0; jj < plan[i].second.size(); ++jj )
                                    //                                    {
                                    //                                        std::cout << " " << plan[i].second[jj];
                                    //                                    }
                                    //                                    std::cout << std::endl;

                                    //Erase the steps before the step that is being checked. 
                                    if( j == (int)plan[i].second.size() - 1 )
                                    {
                                        plan.erase(plan.begin() + i);
                                    }
                                    else
                                    {
                                        for( std::vector<undirected_vertex_index_t>::iterator iter = plan[i].second.begin() + j; iter >= plan[i].second.begin(); --iter )
                                        {
                                            plan[i].second.erase(iter);
                                        }
                                    }

                                    //Erase all the steps between the two points that can be erased.
                                    //                                    PRX_DEBUG_COLOR("i : " << i << "  combination: " << sz - i + 1 << "  plan_pos: " << plan_pos << "  robot_id : " << robot_id, PRX_TEXT_CYAN);
                                    for( std::vector<pebble_step_t>::iterator iter = plan.begin() + i - 1; iter > plan.begin() + plan_pos; --iter )
                                    {
                                        //                                        PRX_DEBUG_COLOR("the robots are: " << (*iter).first << "   robot_id:" << robot_id, PRX_TEXT_LIGHTGRAY);
                                        if( (*iter).first == robot_id )
                                        {
                                            //                                            PRX_DEBUG_COLOR(" Deleting a plan", PRX_TEXT_GREEN);
                                            plan.erase(iter);
                                        }
                                    }


                                    //                                std::cout << "pos plan : (" << plan_pos << "/" << sz << "): " << plan[plan_pos].first << ":";
                                    //                                for( int jj = 0; jj < plan[plan_pos].second.size(); ++jj )
                                    //                                {
                                    //                                    std::cout << " " << plan[plan_pos].second[jj];
                                    //                                }
                                    //                                std::cout << std::endl;

                                    //Erase the steps after the second point. 
                                    //                                    if( step_pos == 0 )
                                    //                                    {
                                    //                                        plan.erase(plan.begin() + plan_pos);
                                    //                                    }
                                    //                                    else
                                    //                                    {
                                    //                                    PRX_DEBUG_COLOR("plan : " << plan_pos << "/" << plan.size() << "     till pos : " << step_pos, PRX_TEXT_GREEN);
                                    for( std::vector<undirected_vertex_index_t>::iterator iter = plan[plan_pos].second.end() - 1; iter > plan[plan_pos].second.begin() + step_pos; --iter )
                                    {
                                        plan[plan_pos].second.erase(iter);
                                    }
                                    //                                    }
                                }
                                else
                                {
                                    if( step_pos == 0 && j == (int)plan[plan_pos].second.size() - 1 )
                                    {
                                        plan.erase(plan.begin() + plan_pos);
                                    }
                                    else
                                    {
                                        for( std::vector<undirected_vertex_index_t>::iterator iter = plan[plan_pos].second.begin() + j; iter > plan[plan_pos].second.begin() + step_pos; --iter )
                                        {
                                            plan[plan_pos].second.erase(iter);
                                        }
                                    }
                                }

                                //                                for( int ii = 0; ii < (int)plan.size(); ++ii )
                                //                                {
                                //                                    std::cout << plan[ii].first << ":";
                                //                                    for( int jj = 0; jj < (int)plan[ii].second.size(); ++jj )
                                //                                    {
                                //                                        std::cout << " " << pebble_to_pas[plan[ii].second[jj]];
                                //                                    }
                                //                                    std::cout << std::endl;
                                //                                }

                                break;
                            }

                        }
                    }

                }
                while( pruned );
                //                PRX_DEBUG_COLOR("AFTER smoothing ", PRX_TEXT_BROWN);
                //                for( int ii = 0; ii < (int)plan.size(); ++ii )
                //                {
                //                    std::cout << plan[ii].first << ":";
                //                    for( int jj = 0; jj < (int)plan[ii].second.size(); ++jj )
                //                    {
                //                        std::cout << " " << pebble_to_pas[plan[ii].second[jj]];
                //                    }
                //                    std::cout << std::endl;
                //                }
            }

            void pebble_solver_tp_t::smoother_check(std::vector<pebble_step_t>& plan, int robot_id, undirected_vertex_index_t v_check, int& plan_pos, int& step_pos)
            {
                for( int p = plan_pos - 1; p >= 0; --p )
                {
                    for( int s = (int)plan[p].second.size() - 1; s >= 0; --s )
                    {
                        if( plan[p].second[s] == v_check )
                        {
                            if( robot_id != plan[p].first )
                                return;

                            plan_pos = p;
                            step_pos = s;
                        }
                    }
                }
            }

            size_t pebble_solver_tp_t::compute_solution_length(const std::vector<pebble_step_t> plan)
            {
                size_t len = 0;

                for( size_t p = 0; p < plan.size(); ++p )
                {
                    len += plan[p].second.size();

                    //                    PRX_DEBUG_COLOR("ROBOT : " << plan[p].first, PRX_TEXT_BROWN);
                    //                    for( size_t s = 0; s < plan[p].second.size(); ++s )
                    //                        PRX_DEBUG_COLOR(p_solver->print_point(&graph, plan[p].second[s]), PRX_TEXT_CYAN);
                }

                len -= k;
                return len;
            }

        }
    }
}

