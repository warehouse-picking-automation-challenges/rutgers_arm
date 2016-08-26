/**
 * @file pebble_test_tp.cpp
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

#include "pebble_test_tp.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"

#include "utilities/tree_feasibility_tester/tree_pebble_graph.hpp"

#include <boost/range/adaptor/map.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::packages::pebble_motion::pebble_test_tp_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace plan;
    using namespace sim;
    
    namespace packages
    {
        namespace pebble_motion
        {

            pebble_test_tp_t::pebble_test_tp_t()
            {
                tp_query = NULL;
                pas_graph = new PaS::Graph();
            }

            pebble_test_tp_t::~pebble_test_tp_t()
            {
                pas_graph->clear();
            }

            void pebble_test_tp_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {

                PRX_INFO_S("Initializing pebble_test_tp_t ...");
                task_planner_t::init(reader, template_reader);

                PRX_WARN_S("reader trace : " << reader->trace());
                //    PRX_WARN_S("template_reader trace : " << template_reader->trace());

                foreach(const parameter_reader_t* point_reader, reader->get_list("points"))
                {
                    int id = point_reader->get_attribute_as<int>("id");
                    nodes_to_add.push_back(std::make_pair(id, point_reader->get_attribute_as<std::vector<double> > ("point")));
                }

                foreach(const parameter_reader_t* edge_reader, reader->get_list("edges"))
                {
                    edges_to_add.push_back(std::make_pair(edge_reader->get_attribute_as<int>("source"), edge_reader->get_attribute_as<int>("target")));
                }

                if( parameters::has_attribute("pebble_tester", reader, template_reader) )
                {
                    PRX_INFO_S("Pebble Tester type: " << parameters::get_attribute_as<std::string > ("pebble_tester/type", reader, template_reader));
                    p_tester = parameters::create_from_loader<pebble_tester_t > ("prx_utilities", reader,"pebble_tester", template_reader,"pebble_tester");
                    ;
                }

                real_distances = parameters::get_attribute_as<bool>("real_distances", reader, template_reader, false);

                visualize_graph = parameters::get_attribute_as<bool>("visualize_graph", reader, template_reader, false);
                visualization_graph_name = parameters::get_attribute_as<std::string > ("visualization_graph_name", reader, template_reader, "kornhauser/graph");
                visualization_body = parameters::get_attribute_as<std::string > ("visualization_body", reader, template_reader);
                graph_color = parameters::get_attribute_as<std::string > ("graph_color", reader, template_reader, "black");
            }

            void pebble_test_tp_t::setup()
            {

                PRX_INFO_S("Setting up the pebble_test_tp...");

                model->use_context("single_robot_space");
                robot_state_space = model->get_state_space();
                robot_control_space = model->get_control_space();
                robot_control_size = robot_control_space->get_dimension();

                metric->link_space(robot_state_space);
                validity_checker->link_model(model);
                local_planner->link_model(model);
                local_planner->link_sampler(sampler);
                local_planner->link_metric(metric);

                this->build_graphs();

                model->use_context("full_space");
                full_state_space = model->get_state_space();
                full_control_space = model->get_control_space();

                int start_node, target_node;
                control_t* initial_control = full_control_space->alloc_point();

                for( unsigned int robot_id = 0; robot_id < tp_query->initial_state.size(); ++robot_id )
                {
                    PRX_ASSERT(tp_query->initial_state[robot_id].first == tp_query->target_state[robot_id].first);
                    std::string name = tp_query->initial_state[robot_id].first;
                    PRX_WARN_S("id:" << robot_id << "  ->  " << name);
                    robot_id_map[robot_id] = name;
                    robot_name_map[name] = robot_id;
                    start_node = tp_query->initial_state[robot_id].second;
                    target_node = tp_query->target_state[robot_id].second;

                    s_assign.add_assignment(nodes_map[start_node].first, robot_id);
                    t_assign.add_assignment(nodes_map[target_node].first, robot_id);

                    pas_robots.push_back(PaS::Agent(robot_id, nodes_map[start_node].second, nodes_map[target_node].second));

                    int index = robot_id * robot_control_size;

                    for( int i = 0; i < robot_control_size; ++i )
                    {
                        initial_control->at(index + i) = graph.get_vertex_as<undirected_node_t > (nodes_map[start_node].first)->point->at(i);
                    }

                }
                tp_query->plan.link_control_space(full_control_space);
                tp_query->plan.copy_onto_back(initial_control, 3);
                PRX_WARN_S(full_control_space->print_point(initial_control, 3));

                p_tester->setup(s_assign, t_assign, s_assign.size(), robot_state_space, metric);
            }

            void getMetrics(const std::vector<PaS::Agent> agents, const PaS::PushAndSwap::Plan &plan)
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
                    int moves = -1; // start at -1 since the agent will be marked as 'moving' to his start
                    int prev = -1;
                    for( size_t j = 0; j < individual_simulations[i].size(); j++ )
                    {
                        if( (int)individual_simulations[i][j] != prev )
                            moves++;
                        prev = individual_simulations[i][j];
                    }

                    moves_per_robot.push_back(moves);
                    total_steps += moves;
                }

                std::cout << "Total number of edges traversed - " << total_steps << std::endl;

                // Write the results
                for( size_t i = 0; i < individual_simulations.size(); i++ )
                {
                    std::cout << "Agent " << agents[i].getID() << " - steps: " << moves_per_robot[i] << std::endl;
                }
            }

            bool pebble_test_tp_t::execute()
            {
                sys_clock_t clock;
                clock.reset();
                if( !p_tester->pebble_test(&graph) )
                {
                    PRX_ERROR_S("PEBBLE TESTER FAILD in " << clock.measure() << " secs");
                }
                else
                {
                    PRX_INFO_S("PEBBLE TESTER FOUND SOLUTION in " << clock.measure() << " secs");
                }

                clock.reset();
                PaS::PushAndSwap pas(pas_robots, pas_graph);
                if( pas.solve() )
                {
                    PRX_INFO_S("PaS FOUND SOLUTION in " << clock.measure() << " secs");

                    pas_plan = pas.getPlan();

                    std::cout << "======== Unsmoothed plan ========" << std::endl;

                    //        std::vector<unsigned int> starts;
                    //        for (size_t i = 0; i < pas_robots.size(); ++i)
                    //            starts.push_back(pas_robots[i].getStart());
                    getMetrics(pas_robots, pas_plan);
                    std::cout << "======== Smoothed plan ========" << std::endl;
                    pas_plan = PaS::PushAndSwap::smooth(pas_plan);
                    getMetrics(pas_robots, pas_plan);
                    resolve_query();
                }
                else
                {
                    PRX_ERROR_S("PaS FAILD in " << clock.measure() << " secs");
                }


                return true;
            }

            const statistics_t* pebble_test_tp_t::get_statistics()
            {
                return NULL;
            }

            bool pebble_test_tp_t::succeeded() const
            {
                return true;
            }

            void pebble_test_tp_t::link_query(query_t* in_query)
            {
                //this will overwrite any query read from input
                if( tp_query != NULL )
                    delete tp_query;
                tp_query = dynamic_cast<pebble_test_query_t*>(in_query);
            }

            void pebble_test_tp_t::resolve_query()
            {
                for( size_t i = 0; i < pas_plan.size(); ++i )
                {
                    int index = pas_plan[i].first * robot_control_size;
                    undirected_vertex_index_t v_source = pas_to_pebble[pas_plan[i].second[0]];
                    undirected_vertex_index_t v_target;
                    space_point_t* target;
                    PRX_INFO_S("for agent : " << pas_plan[i].first);
                    std::cout << pas_plan[i].second[0] << "  ";
                    for( size_t p = 1; p < pas_plan[i].second.size(); ++p )
                    {
                        v_target = pas_to_pebble[pas_plan[i].second[p]];

                        target = graph.get_vertex_as<undirected_node_t > (v_target)->point;
                        std::cout << pas_plan[i].second[i] << " (" << robot_state_space->print_point(target, 1) << ")  ";

                        control_t* new_control = full_control_space->clone_point(tp_query->plan.back().control);

                        for( int c = 0; c < robot_control_size; ++c )
                        {
                            new_control->at(index + c) = target->at(c);
                        }

                        //            PRX_DEBUG_S("v_source : " << graph.get_vertex_as<tree_pebble_node_t>(v_source)->print_point(robot_state_space) << " -> " << graph.get_vertex_as<tree_pebble_node_t>(v_target)->print_point(robot_state_space));
                        if( boost::edge(v_source, v_target, graph.graph).second )
                        {
                            undirected_edge_index_t e = boost::edge(v_source, v_target, graph.graph).first;
                            tp_query->plan.copy_onto_back(new_control, simulation::simulation_step * graph.get_edge_as<pebble_test_edge_t > (e)->steps);
                        }
                        //            else
                        //            {
                        //                PRX_WARN_S("PAS : " << pebble_to_pas[v_source] << "  -  " << pas_plan[i].second[p]);
                        //            }
                        v_source = v_target;
                    }
                    std::cout << std::endl;
                }

                //    {
                //        for( int i = 1; i < ps_plan.pathSize(); ++i )
                //        {
                //            control_t* new_control = full_control_space->clone_point(solution_plan.steps.back().control);
                //
                //            undirected_vertex_index_t start = pas_to_pebble[ps_plan.getPathAt(i - 1)];
                //            undirected_vertex_index_t end = pas_to_pebble[ps_plan.getPathAt(i)];
                //
                //            int index = ps_plan.getID() * robot_control_size;
                //
                //            element_iterator_t s = robot_state_space->get_element_iterator(g.get_vertex_as<pebble_node_t>(end)->point);
                //            element_iterator_t c = full_control_space->get_element_iterator(new_control);
                //
                //            for(int i = 0; i < robot_control_size; ++i )
                //                c[index + i] = s[i];
                //
                //            undirected_edge_index_t e = boost::edge(start,end,g.graph).first;
                //            //            PRX_DEBUG_S("new control : " << sim_control->print_point(new_control));
                //            solution_plan.steps.push_back(plan_step_t(new_control, g.get_edge_as<korn_edge_t>(e)->steps * sim_time));
                //        }
                //    }
            }

            void pebble_test_tp_t::build_graphs()
            {
                undirected_vertex_index_t v;
                int new_pas_v;

                foreach(node_id_t vec, nodes_to_add)
                {
                    v = p_tester->add_new_vertex(&graph);
                    new_pas_v = pas_graph->add_node();
                    PRX_INFO_S("new node :" << vec.first << ")  v:" << v << "   pas_v:" << new_pas_v);
                    nodes_map[vec.first] = std::make_pair(v, new_pas_v);

                    graph[v]->point = robot_state_space->alloc_point();
                    robot_state_space->set_from_vector(vec.second, graph[v]->point);
                    PRX_WARN_S("point : " << robot_state_space->print_point(graph[v]->point, 3));

                    pebble_to_pas[v] = new_pas_v;
                    pas_to_pebble[new_pas_v] = v;

                }

                //    space_point_t* new_control = robot_control_space->alloc_point();
                double dist;

                foreach(edge_id_t e, edges_to_add)
                {
                    if( real_distances )
                        dist = metric->distance_function(graph[nodes_map[e.first].first]->point, graph[nodes_map[e.second].first]->point);
                    else
                        dist = 1;

                    PRX_INFO_S("edge: " << e.first << " - " << e.second << "   source :" << nodes_map[e.first].first);
                    //        PRX_WARN_S("source point : " << robot_state_space->print_point(graph->get_vertex_as<undirected_node_t>([nodes_map[e.first].first])->point,3));      
                    plan_t plan(robot_control_space);
                    trajectory_t path(robot_state_space);
                    local_planner->steer(graph.get_vertex_as<undirected_node_t > (nodes_map[e.first].first)->point, graph.get_vertex_as<undirected_node_t > (nodes_map[e.second].first)->point, plan, path);
                    undirected_edge_index_t edge = graph.add_edge<pebble_test_edge_t > (nodes_map[e.first].first, nodes_map[e.second].first, dist);
                    graph.get_edge_as<pebble_test_edge_t > (edge)->steps = plan.length() / simulation::simulation_step;

                    pas_graph->add_edge(nodes_map[e.first].second, nodes_map[e.second].second, dist);
                    path.clear();
                    plan.clear();
                }
                //    robot_control_space->free_point(new_control);

            }

            void pebble_test_tp_t::update_vis_info() const
            {
                PRX_INFO_S("pebble_test_tp_t update_vis_info");

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

                    ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->visualization_geom_map[visualization_graph_name] = geoms;
                    ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->visualization_configs_map[visualization_graph_name] = configs;
                    geoms.clear();
                    configs.clear();
                    model->use_context("full_space");
                }
            }

        }
    }
}