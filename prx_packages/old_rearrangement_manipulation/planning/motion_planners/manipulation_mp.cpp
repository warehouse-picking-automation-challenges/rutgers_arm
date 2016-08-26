/**
 * @file prm.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "planning/motion_planners/manipulation_mp.hpp"
#include "planning/problem_specifications/manipulation_mp_specification.hpp"
#include "planning/queries/manipulator_mp_query.hpp"
#include "planning/modules/system_name_validity_checker.hpp"
#include "planning/modules/obstacle_aware_astar.hpp"


#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/modules/heuristic_search/astar_module.hpp"
#include "prx/planning/communication/visualization_comm.hpp"

#include "../../../baxter/simulation/plants/manipulator.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/compressed_sparse_row_graph.hpp>
#include <bits/stl_set.h>


PLUGINLIB_EXPORT_CLASS(prx::packages::rearrangement_manipulation::manipulation_mp_t, prx::plan::planner_t)


namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        using namespace baxter;

        namespace rearrangement_manipulation
        {

            manipulation_mp_t::manipulation_mp_t()
            {
                graph_deserialization_file = "";
                graph_deserialize_flag = false;

                char* w = std::getenv("PRACSYS_PATH");

                prx_output_dir = std::string(w) + "/prx_output/rearrangement_graphs/";
                prx_input_dir = std::string(w) + "/prx_input/rearrangement_graphs";
            }

            manipulation_mp_t::~manipulation_mp_t() { }

            void manipulation_mp_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                PRX_INFO_S("Initializing manipulation motion planner... ");
                prm_star_t::init(reader, template_reader);

                if( parameters::has_attribute("graph_deserialization_file", reader, template_reader) )
                {
                    graph_deserialization_file = parameters::get_attribute_as<std::string > ("graph_deserialization_file", reader, template_reader);
                    graph_deserialize_flag = true;
                }

                lazy_runs = parameters::get_attribute_as<int>("lazy_runs", reader, template_reader, 0);
                has_grasped_points = parameters::get_attribute_as<bool> ("has_grasped_points", reader, template_reader, true);

            }

            void manipulation_mp_t::reset()
            {
                prm_star_t::reset();
            }

            void manipulation_mp_t::link_specification(specification_t* new_spec)
            {
                prm_star_t::link_specification(new_spec);
                specs = (manipulation_mp_specification_t*)new_spec;
                extra_start_states = specs->extra_starting_states;
            }

            void manipulation_mp_t::link_query(plan::query_t* new_query)
            {
                prm_star_t::link_query(new_query);
                in_query = dynamic_cast<manipulator_mp_query_t*>(new_query);
                //                PRX_DEBUG_COLOR("Manipulator MP query collision: " << in_query->q_collision_type, PRX_TEXT_LIGHTGRAY);
            }

            void manipulation_mp_t::setup()
            {
                random_point = state_space->alloc_point();
                path1.link_space(state_space);
                path2.link_space(state_space);
                new_plan.link_control_space(control_space);
                new_plan2.link_control_space(control_space);

                local_planner->link_state_space(state_space);

                astar->link_spaces(state_space, control_space);
                astar->link_modules(validity_checker, local_planner);
                astar->link_distance_metric(metric);
                oa_astar = dynamic_cast<obstacle_aware_astar_t*>(astar);
            }

            bool manipulation_mp_t::execute()
            {
                //Read a graph from a file (specs->graph_deserialization_file), inform the graph and serialize it to a new file.
                if( specs->is_builder )
                {
                    PRX_ASSERT(specs->graph_deserialization_file != "");
                    //We will try to deserialize the graph. If it fails a PRX_FATAL will stop the execution.
                    if( deserialize_simple_graph(specs->graph_deserialization_file) )
                    {
                        inform_graph_full_space(specs->get_poses());
                        PRX_ASSERT(specs->serialization_file != "");
                        serialization_file = specs->serialization_file;
                        if( serialize() )
                        {
                            return true;
                        }
                        PRX_FATAL_S("Cannot serialize the graph to the file: " << serialization_file);
                    }
                    PRX_FATAL_S("Could not deserialize the graph from file: " << specs->graph_deserialization_file);
                    return false;

                }
                    //When the manipulation motion planner will read an informed graph from a file to use it.
                else
                {
                    deserialization_file = specs->deserialization_file;
                    PRX_ASSERT(deserialization_file != "");
                    deserialize();

                    std::vector<double> pos(specs->object_space->get_dimension());
                    specs->object_space->copy_point_to_vector(specs->get_poses()->at(0).second, pos);
                    pos[0] = 100;
                    pos[2] = -100;
                    specs->object_space->set_from_vector(pos);

                    if( specs->object_space != NULL )
                    {
                        pos[0] = -100;
                        specs->object_space->set_from_vector(pos);
                    }

                    //Add the new points because of the query poses.
                    std::vector<state_t*>& seeds = specs->get_seeds();
                    std::vector<bool>& valid_seeds = specs->get_validity();
                    for( unsigned i = 0; i < seeds.size(); ++i )
                    {
                        if( validity_checker->is_valid(seeds[i]) )
                        {
                            valid_seeds[i] = true;
                            add_node(seeds[i]);
                            update_k(num_vertices);
                        }
                        else
                        {
                            valid_seeds[i] = false;
                        }
                    }

#ifndef NDEBUG
                    for( unsigned i = 0; i < valid_seeds.size(); ++i )
                        PRX_ASSERT(valid_seeds[i]);

                    PRX_DEBUG_COLOR("query poses: " << specs->get_query_poses()->size() << "     poses: " << specs->get_poses()->size() << "     new_edges:" << new_edges.size(), PRX_TEXT_BROWN);
#endif
                    //Inform all the graph with the new poses from initial and target poses.
                    inform_graph(specs->get_query_poses());
                    //Inform the new edges with the already existed poses.
                    inform_edges(new_edges, specs->get_poses());
                }
                return true;
            }

            void manipulation_mp_t::resolve_query()
            {
                unsigned goals_size;
                std::vector<space_point_t*> goals = in_query->get_goal()->get_goal_points(goals_size);
                std::vector<undirected_vertex_index_t> v_goals;

                if( in_query->q_type == motion_planning_query_t::PRX_ADD_QUERY_POINTS_NO_COLLISIONS )
                    no_collision_query_type = true;
                else if( in_query->q_type == motion_planning_query_t::PRX_NEAR_QUERY_POINTS )
                    near_query_type = true;

                undirected_vertex_index_t v_start;
                boost::tie(remove_start, v_start) = add_node(in_query->get_start_state());
                add_extra_starting_points();
                PRX_ASSERT(!remove_start);

                undirected_vertex_index_t v_g;
                bool remove_goal;

                for(unsigned i = 0; i < goals_size; ++i)
                {
                    boost::tie(remove_goal, v_g) = add_node(goals[i]);
                    PRX_ASSERT(!remove_goal);
                    v_goals.push_back(v_g);
                    remove_goals.push_back(remove_goal);
                }

                // PRX_DEBUG_COLOR("Start: " << state_space->print_point(graph.get_vertex_as<manipulation_node_t > (v_start)->point, 5), PRX_TEXT_LIGHTGRAY);                
                // for( unsigned i = 0; i < v_goals.size(); ++i )
                //     PRX_DEBUG_COLOR("Goal (" << i << "): " << state_space->print_point(graph.get_vertex_as<manipulation_node_t > (v_goals[i])->point, 5), PRX_TEXT_LIGHTGRAY);

                // PRX_DEBUG_COLOR("In goals: " << goals.size() << "    in v_goals:" << v_goals.size(), PRX_TEXT_LIGHTGRAY);

                bool good_to_go = false;
                boost::connected_components(graph.graph, graph.components);
                for( unsigned i = 0; i < v_goals.size(); ++i )
                {
                    if( graph.components[v_start] == graph.components[v_goals[i]] )
                    {
                        good_to_go = true;
                        break;
                    }
                }

                if( !good_to_go )
                {
                    PRX_DEBUG_COLOR("Start and Goal are not in the same connected component !", PRX_TEXT_RED);
                    return;
                }

                in_query->clear();
                astar->link_graph(&graph);
                std::deque< undirected_vertex_index_t > path_vertices;
                //                PRX_DEBUG_COLOR("Manipulation_mp : collision type:" << in_query->q_collision_type << "  Lazy:" << motion_planning_query_t::PRX_LAZY_COLLISIONS << "   Reuse:" << motion_planning_query_t::PRX_ACTIVE_COLLISIONS_REUSE_EDGES ,PRX_TEXT_MAGENTA);
                if( in_query->q_collision_type == motion_planning_query_t::PRX_LAZY_COLLISIONS )
                {
                    //                    PRX_DEBUG_COLOR("Lazy runs!",PRX_TEXT_BLUE);
                    bool found_path = false;
                    int runs = 0;
                    //                    PRX_ASSERT(lazy_runs != 0);
                    sys_clock_t clock;
                    clock.reset();
                    astar->set_astar_mode(astar_module_t::PRX_NO_EDGE_INFO);
                    while( !found_path && runs < lazy_runs )
                    {
                        runs++;
                        if( astar->solve(v_start, v_goals) )
                        {
                            found_path = oa_astar->is_valid_path();
                            if( found_path )
                            {
                                path_vertices.clear();
                                astar->extract_path(v_start, astar->get_found_goal(), path_vertices);
                                PRX_DEBUG_COLOR("ASTAR FOUND A PATH size:" << path_vertices.size() << "   runs:" << runs, PRX_TEXT_BLUE);
                            }
                        }
                        else
                        {
                            //There is no path and we have to stop searching
                            PRX_DEBUG_COLOR("No solution to astar", PRX_TEXT_RED);
                            runs = lazy_runs;
                            break;
                        }
                    }

                    if( runs >= lazy_runs && !found_path )
                    {
                        astar->restart();
                        if( astar->solve(v_start, v_goals) )
                        {
                            found_path = true;
                            path_vertices.clear();
                            astar->extract_path(v_start, astar->get_found_goal(), path_vertices);
                            //                            PRX_DEBUG_COLOR("ASTAR FOUND A PATH size:" << path_vertices.size() << "   runs:" << runs, PRX_TEXT_RED);
                        }
                    }


                    if( found_path )
                    {
                        for( size_t i = 0; i < path_vertices.size() - 1; ++i )
                        {
                            undirected_edge_index_t e = boost::edge(path_vertices[i], path_vertices[i + 1], graph.graph).first;
                            manipulation_edge_t* edge = graph.get_edge_as<manipulation_edge_t > (e);
                            in_query->full_constraints.insert(edge->constraints.begin(), edge->constraints.end());
                            path1.clear();
                            new_plan.clear();
                            local_planner->steer(graph[path_vertices[i]]->point, graph[path_vertices[i + 1]]->point, new_plan, path1);
                            in_query->plan += new_plan;
                        }
                        oa_astar->get_path_constraints(in_query->constraints);
                        in_query->found_start_state = graph.get_vertex_as<manipulation_node_t > (path_vertices[0])->point;
                        in_query->goal_state = graph.get_vertex_as<manipulation_node_t > (path_vertices.back())->point;
                        //                        PRX_DEBUG_COLOR("With Constraints: " << print(in_query->constraints), PRX_TEXT_MAGENTA);
                        in_query->solution_cost = astar->get_path_cost();
                        in_query->found_path = true;
                    }
#ifndef NDEBUG
                    //                    PRX_DEBUG_COLOR("", PRX_TEXT_BROWN);
                    //                    PRX_DEBUG_COLOR("================================================================", PRX_TEXT_BROWN);
                    //                    PRX_DEBUG_COLOR("====    Found path : " << found_path << "   size: " << in_query->plan.size() << "   |c|: " << in_query->constraints.size() << "   in " << clock.measure() << " sec    ====", PRX_TEXT_RED);
                    //                    PRX_DEBUG_COLOR("====   CONSTRAINTS: " << print(in_query->constraints), PRX_TEXT_RED);
                    //                    PRX_DEBUG_COLOR("================================================================\n", PRX_TEXT_BROWN);
                    //                    PRX_DEBUG_COLOR("", PRX_TEXT_BROWN);
#endif
                }
                else if( in_query->q_collision_type == motion_planning_query_t::PRX_ACTIVE_COLLISIONS_REUSE_EDGES )
                {
                    astar->set_astar_mode(astar_module_t::PRX_REUSE_EDGE_INFO);
                    if( astar->solve(v_start, v_goals) )
                    {
                        astar->extract_path(v_start, astar->get_found_goal(), path_vertices);
                        //                        PRX_DEBUG_COLOR("ASTAR FOUND A PATH size:" << path_vertices.size(), PRX_TEXT_BLUE);

                        oa_astar->get_path_constraints(in_query->constraints);
                        oa_astar->get_foul_constraints(in_query->foul_constraints);


                        // std::vector<double> state_vec(specs->manip_state_space->get_dimension());
                        // state_space->copy_from_point(graph[path_vertices[0]]->point);
                        // specs->manip_state_space->copy_to_vector(state_vec);
                        // control_t* ctrl = control_space->alloc_point();
                        // control_space->copy_vector_to_point(state_vec, ctrl);
                        // in_query->plan.copy_onto_back(ctrl, simulation::simulation_step);
                        // control_space->free_point(ctrl);

                        for( unsigned i = 0; i < path_vertices.size() - 1; ++i )
                        {
                            undirected_edge_index_t e = boost::edge(path_vertices[i], path_vertices[i + 1], graph.graph).first;
                            manipulation_edge_t* edge = graph.get_edge_as<manipulation_edge_t > (e);
                            in_query->full_constraints.insert(edge->constraints.begin(), edge->constraints.end());

                            foreach(unsigned c, edge->constraints)
                            {
                                if( in_query->constraints.count(c) > 0 && std::find(in_query->constraints_in_order.begin(), in_query->constraints_in_order.end(), c) == in_query->constraints_in_order.end() )
                                {
                                    in_query->constraints_in_order.push_back(c);
                                }
                            }
                            //                            in_query->constraints_in_order.insert(in_query->constraints_in_order.end(), edge->constraints.begin(), edge->constraints.end());
                            PRX_DEBUG_COLOR(edge->source_vertex << "->" << edge->target_vertex << ") v:(" << graph.get_vertex_as<manipulation_node_t > (path_vertices[i])->node_id << "->" << graph.get_vertex_as<manipulation_node_t > (path_vertices[i + 1])->node_id << ") full_Constraints : " << print(edge->constraints), PRX_TEXT_LIGHTGRAY);
                            if( edge->plan.size() == 0 )
                            {
                                path1.clear();
                                new_plan.clear();
                                local_planner->steer(graph[path_vertices[i]]->point, graph[path_vertices[i + 1]]->point, new_plan, path1);
                                in_query->plan += new_plan;
                            }
                            else
                            {
                                PRX_DEBUG_COLOR("edge has plan : " << edge-> source_vertex << " -> " << edge->target_vertex, PRX_TEXT_MAGENTA);
                                edge->append_plan(in_query->plan, path_vertices[i]);
                            }
                        }

                        in_query->found_start_state = graph.get_vertex_as<manipulation_node_t > (path_vertices[0])->point;
                        in_query->goal_state = graph.get_vertex_as<manipulation_node_t > (path_vertices.back())->point;
                        in_query->solution_cost = astar->get_path_cost();
                        in_query->found_path = true;
#ifndef NDEBUG
                        // PRX_DEBUG_COLOR("", PRX_TEXT_BROWN);
                        // PRX_DEBUG_COLOR("================================================================", PRX_TEXT_BROWN);
                        // PRX_DEBUG_COLOR("====   Found path : " << in_query->found_path << "   size: " << in_query->plan.size() << "   |c|: " << in_query->constraints.size() << "  ====", PRX_TEXT_RED);
                        // PRX_DEBUG_COLOR("====   CONSTRAINTS: " << print(in_query->constraints), PRX_TEXT_RED);
                        // PRX_DEBUG_COLOR("====   ORDER CONST: " << print(in_query->constraints_in_order), PRX_TEXT_RED);
                        // if( in_query->plan.size() != 0 )
                        // {
                        //     PRX_DEBUG_COLOR("====   Plan start : " << control_space->print_point(in_query->plan[0].control, 6), PRX_TEXT_RED);
                        //     PRX_DEBUG_COLOR("====   Plan end   : " << control_space->print_point(in_query->plan.back().control, 6), PRX_TEXT_RED);
                        // }
                        // PRX_DEBUG_COLOR("================================================================", PRX_TEXT_BROWN);
                        // PRX_DEBUG_COLOR("", PRX_TEXT_BROWN);
#endif

                    }
                }

                // if( remove_start )
                // {

                //     foreach(undirected_vertex_index_t u, boost::adjacent_vertices(v_start, graph.graph))
                //     {
                //         undirected_edge_index_t e = boost::edge(v_start, u, graph.graph).first;
                //         graph.get_edge_as<manipulation_edge_t > (e)->clear(control_space);
                //         e = boost::edge(u, v_start, graph.graph).first;
                //         graph.get_edge_as<manipulation_edge_t > (e)->clear(control_space);
                //     }

                //     metric->remove_point(graph.get_vertex_as<undirected_node_t > (v_start));
                //     graph.clear_and_remove_vertex(v_start);
                //     num_vertices--;
                // }

                // for( size_t i = 0; i < remove_starts.size(); ++i )
                // {
                //     if( remove_starts[i] )
                //     {
                //         v_g = oa_astar->extra_starts[i];

                //         foreach(undirected_vertex_index_t u, boost::adjacent_vertices(v_g, graph.graph))
                //         {
                //             undirected_edge_index_t e = boost::edge(v_g, u, graph.graph).first;
                //             graph.get_edge_as<manipulation_edge_t > (e)->clear(control_space);
                //             e = boost::edge(u, v_g, graph.graph).first;
                //             graph.get_edge_as<manipulation_edge_t > (e)->clear(control_space);
                //         }
                //         metric->remove_point(graph.get_vertex_as<undirected_node_t > (v_g));
                //         graph.clear_and_remove_vertex(v_g);
                //         num_vertices--;
                //     }
                // }

                // for( size_t i = 0; i < v_goals.size(); ++i )
                // {
                //     if( remove_goals[i] )
                //     {
                //         v_g = v_goals[i];

                //         foreach(undirected_vertex_index_t u, boost::adjacent_vertices(v_g, graph.graph))
                //         {
                //             undirected_edge_index_t e = boost::edge(v_g, u, graph.graph).first;
                //             graph.get_edge_as<manipulation_edge_t > (e)->clear(control_space);
                //             e = boost::edge(u, v_g, graph.graph).first;
                //             graph.get_edge_as<manipulation_edge_t > (e)->clear(control_space);
                //         }
                //         metric->remove_point(graph.get_vertex_as<undirected_node_t > (v_g));
                //         graph.clear_and_remove_vertex(v_g);
                //         num_vertices--;
                //     }
                // }

                v_goals.clear();
                remove_goals.clear();

                oa_astar->extra_starts.clear();
                remove_starts.clear();

                no_collision_query_type = false;
                near_query_type = false;
            }

            bool manipulation_mp_t::succeeded() const
            {
                if( input_specification->get_stopping_criterion()->satisfied() )
                    return true;
                return false;
            }

            std::pair<bool, util::undirected_vertex_index_t > manipulation_mp_t::add_node(const space_point_t * n_state)
            {
                if( metric->get_nr_points() > 0 )
                {
                    const manipulation_node_t* node = metric->single_query(n_state)->as<manipulation_node_t > ();
                    if( node != NULL && (near_query_type || metric->distance_function(n_state, node->point) <= similarity_threshold) )
                    {
                        //                        PRX_DEBUG_COLOR("The point is already in the graph : " << state_space->print_point(n_state, 4), PRX_TEXT_BROWN);
                        return std::make_pair(false, node->index);
                    }
                }

                v_new = graph.add_vertex<manipulation_node_t > ();
                num_vertices++;
                graph.get_vertex_as<manipulation_node_t > (v_new)->init_node(state_space, n_state);
                //                PRX_INFO_S ("New node: " << state_space->print_point(graph[v_new]->point));

                if( delta_prm )
                    connect_node(v_new, r_n);

                else
                    connect_node(v_new);

                return std::make_pair(true, v_new);
            }

            void manipulation_mp_t::link_node_to_neighbors(undirected_vertex_index_t v, const std::vector< const abstract_node_t* >& neighbors)
            {
                const undirected_node_t* node;

                path1.clear();
                for( size_t i = 0; i < neighbors.size(); i++ )
                {
                    node = neighbors[i]->as< undirected_node_t > ();
                    new_plan.clear();

                    PRX_DEBUG_COLOR("Start steering call to local planner \nstart: " << state_space->print_point(graph[v]->point, 5) << "\nend  :" << state_space->print_point(node->point, 5), PRX_TEXT_BROWN);
                    local_planner->steer(graph[v]->point, node->point, new_plan, path1);
                    PRX_DEBUG_COLOR("-----     END     -----", PRX_TEXT_BROWN);
                    //If the path is valid
                    if( new_plan.size() != 0 && is_valid_trajectory(path1) )
                    {
                        //Add the edge to the graph
                        //dist = metric->distance_function(graph[v]->point, node->point);
                        double dist = validity_checker->trajectory_cost(path1);
                        undirected_edge_index_t e = graph.add_edge< manipulation_edge_t > (v, node->index, dist);
                        graph.get_edge_as<manipulation_edge_t > (e)->plan = new_plan;
                        graph.get_edge_as<manipulation_edge_t > (e)->id = num_edges;
                        num_edges++;
                        new_edges.push_back(e);

                        if( visualize_graph )
                            graph.get_edge_as< manipulation_edge_t > (e)->path = path1;
                    }
                    path1.clear();
                }
            }

            void manipulation_mp_t::inform_graph_full_space(const std::vector< std::pair<unsigned, sim::state_t*> >* poses)
            {
                PRX_DEBUG_COLOR("Informing the graph", PRX_TEXT_CYAN);
                system_name_validity_checker_t* checker = dynamic_cast<system_name_validity_checker_t*>(validity_checker);
                state_t* collision_check_point = NULL;
                state_t* start_point = NULL;
                state_t* end_point = NULL;
                std::vector<double> full_state_vec(specs->full_collision_object_space->get_dimension());
                unsigned object_size = specs->object_space->get_dimension();
                std::set<unsigned> edge_constraints;
                std::set<unsigned> poses_constraints;

                if( specs->transfer_mode )
                {
                    start_point = specs->object_space->alloc_point();
                    end_point = specs->object_space->alloc_point();
                    collision_check_point = state_space->alloc_point();
                }

                for( unsigned i = 0; i < specs->poses_set->size(); ++i )
                {
                    for( unsigned j = 0; j < specs->poses_set->at(i).size(); ++j )
                    {
                        const manipulation_node_t* retract_node = metric->single_query(specs->poses_set->at(i).get_retracted_state_at(j,specs->transfer_mode))->as<manipulation_node_t > ();
                        PRX_ASSERT(state_space->equal_points(retract_node->point, specs->poses_set->at(i).get_retracted_state_at(j,specs->transfer_mode)));
                        const manipulation_node_t* node;
                        if(has_grasped_points)
                        {
                            node = metric->single_query(specs->poses_set->at(i).get_state_at(j,specs->transfer_mode))->as<manipulation_node_t > ();
                            PRX_ASSERT(state_space->equal_points(node->point, specs->poses_set->at(i).get_state_at(j,specs->transfer_mode)));
                        }
                        else
                        {
                            undirected_vertex_index_t v_new = graph.add_vertex<manipulation_node_t > ();
                            num_vertices++;
                            graph.get_vertex_as<manipulation_node_t > (v_new)->init_node(state_space, specs->poses_set->at(i).get_state_at(j,specs->transfer_mode));
                            node = graph.get_vertex_as<manipulation_node_t > (v_new);
                        }
                        
                        
                        manipulation_edge_t* edge;
                        if( boost::edge(node->index, retract_node->index, graph.graph).second )
                        {
                            edge = graph.get_edge_as<manipulation_edge_t > (boost::edge(node->index, retract_node->index, graph.graph).first);
                        }
                        else
                        {
                            double dist = metric->distance_function(retract_node->point, node->point);
                            undirected_edge_index_t e = graph.add_edge<manipulation_edge_t > (retract_node->index, node->index, dist);
                            edge = graph.get_edge_as<manipulation_edge_t > (e);
                            edge->id = num_edges;
                            num_edges++;
                        }
                        edge->set_source_vertex(retract_node->index);
                        specs->poses_set->at(i).get_reaching_plan_at(edge->plan, j, specs->transfer_mode);
                        specs->poses_set->at(i).get_retracting_plan_at(edge->other_way_plan, j, specs->transfer_mode);
                    }
                }
                

                int edge_size = boost::num_edges(graph.graph);
                int edge_counter = 0;

                unsigned pl = 0;
                for( unsigned i = 0; i < poses->size(); ++i )
                    for( unsigned j = 0; j < object_size; ++j, ++pl )
                        full_state_vec[pl] = poses->at(i).second->at(j);

                //Indices for poses that are involved in this edge.
                int start_pose_id;
                int end_pose_id;

                foreach(undirected_edge_index_t e, boost::edges(graph.graph))
                {
                    PRX_INFO_S("Edge: " << edge_counter << "/" << edge_size);
                    edge_counter++;
                    path1.clear();
                    new_plan.clear();                    
                    poses_constraints.clear();
                    //Detects if any pose is participating in this edge. 
                    if( specs->transfer_mode )
                    {

                        //                        for( unsigned i = 0; i < poses->size(); ++i )
                        //                            for( unsigned j = 0; j < object_size; ++j, ++pl )
                        //                                full_state_vec[pl] = poses->at(i).second->at(j);

                        state_space->copy_from_point(graph[boost::source(e, graph.graph)]->point);
                        specs->object_space->copy_to_point(start_point);
                        PRX_DEBUG_COLOR("start posE: " << specs->object_space->print_point(start_point, 8), PRX_TEXT_CYAN);

                        state_space->copy_from_point(graph[boost::target(e, graph.graph)]->point);
                        specs->object_space->copy_to_point(end_point);
                        PRX_DEBUG_COLOR("end posE  : " << specs->object_space->print_point(end_point, 8), PRX_TEXT_CYAN);


                        PRX_DEBUG_COLOR("Start: " << state_space->print_point(graph[boost::source(e, graph.graph)]->point, 5), PRX_TEXT_LIGHTGRAY);
                        PRX_DEBUG_COLOR("End  : " << state_space->print_point(graph[boost::target(e, graph.graph)]->point, 5), PRX_TEXT_LIGHTGRAY);

                        start_pose_id = -1;
                        end_pose_id = -1;
                        pl = 0;
                        bool new_pose;
                        for( unsigned i = 0; i < poses->size(); ++i )
                        {
                            //                            PRX_DEBUG_COLOR("POSE(" << poses->at(i).first << "): " << specs->object_space->print_point(poses->at(i).second,8),PRX_TEXT_GREEN);
                            new_pose = false;
                            if( specs->object_space->equal_points(start_point, poses->at(i).second, PRX_ZERO_CHECK) )
                            {
                                start_pose_id = poses->at(i).first;
                                new_pose = true;
                            }
                            else if( specs->object_space->equal_points(end_point, poses->at(i).second, PRX_ZERO_CHECK) )
                            {
                                end_pose_id = poses->at(i).first;
                                new_pose = true;
                            }

                            if( new_pose )
                            {
                                PRX_DEBUG_COLOR("Dist start: " << specs->object_space->distance(start_point, poses->at(i).second) << "      End: " << specs->object_space->distance(end_point, poses->at(i).second), PRX_TEXT_MAGENTA);
                                poses_constraints.insert(poses->at(i).first);
                                //                                PRX_DEBUG_COLOR("POSE: " << poses->at(i).first << "   object_size: " << object_size, PRX_TEXT_RED)
                                for( unsigned j = 0; j < object_size; ++j, ++pl )
                                {
                                    if( j == 2 )
                                    {

                                        full_state_vec[pl] = -100;
                                    }
                                    else
                                    {
                                        full_state_vec[pl] = poses->at(i).second->at(j);
                                    }
                                }
                            }
                            else
                            {
                                for( unsigned j = 0; j < object_size; ++j, ++pl )
                                    full_state_vec[pl] = poses->at(i).second->at(j);
                            }
                        }
                        //                        PRX_ASSERT(false);
                        //                        PRX_DEBUG_COLOR("START: " << print(full_state_vec), PRX_TEXT_MAGENTA);
                        specs->full_collision_object_space->set_from_vector(full_state_vec);
                        //                        PRX_DEBUG_COLOR(specs->full_collision_object_space->print_memory(6), PRX_TEXT_BLUE);
                    }
                    manipulation_edge_t *edge = graph.get_edge_as<manipulation_edge_t > (e);
                    undirected_vertex_index_t v = boost::source(e, graph.graph);
                    undirected_vertex_index_t u = boost::target(e, graph.graph);
                    edge_constraints.clear();

                    if( edge->plan.size() > 0 )
                    {
                        PRX_DEBUG_COLOR("Has plan : " << edge->source_vertex << " -> " << edge->target_vertex, PRX_TEXT_BLUE);                        
                        path1.clear();
                        local_planner->propagate(graph[v]->point, edge->use_plan(v), path1);                        
                        checker->has_constraints(edge_constraints, path1);                                                
                        path1.clear();
                        local_planner->propagate(graph[u]->point, edge->use_plan(u), path1);
                        checker->has_constraints(edge_constraints, path1);
                    }
                    else
                    {
                        new_plan.clear();
                        path1.clear();
                        local_planner->steer(graph[v]->point, graph[u]->point, new_plan, path1);
                        // PRX_DEBUG_COLOR("PATH for " << graph[v]->node_id << " -> " << graph[u]->node_id << " \n" << path1.print(), PRX_TEXT_GREEN);

                        if( state_space->equal_points(path1[0], path1.back(), PRX_DISTANCE_CHECK) )
                        {
                            PRX_DEBUG_COLOR("path[0]: " << state_space->print_point(path1[0], 8), PRX_TEXT_GREEN);
                            PRX_DEBUG_COLOR("path.back: " << state_space->print_point(path1.back(), 8), PRX_TEXT_GREEN);
                            PRX_DEBUG_COLOR("source  : " << state_space->print_point(graph[boost::source(e, graph.graph)]->point, 8), PRX_TEXT_CYAN);
                            PRX_DEBUG_COLOR("target  : " << state_space->print_point(graph[boost::target(e, graph.graph)]->point, 8), PRX_TEXT_CYAN);
                            PRX_DEBUG_COLOR("Dist start: " << state_space->distance(path1[0], graph[boost::source(e, graph.graph)]->point), PRX_TEXT_RED);
                            PRX_DEBUG_COLOR("Dist end  : " << state_space->distance(path1.back(), graph[boost::target(e, graph.graph)]->point), PRX_TEXT_RED);
                        }
                        PRX_ASSERT(state_space->equal_points(path1[0], graph[boost::source(e, graph.graph)]->point, PRX_DISTANCE_CHECK));
                        //PRX_ASSERT(state_space->equal_points(path1.back(), graph[boost::target(e, graph.graph)]->point, PRX_DISTANCE_CHECK));

                        PRX_ASSERT(dynamic_cast<system_name_validity_checker_t*>(validity_checker) != NULL);
                        checker->has_constraints(edge_constraints, path1);
                    }
                    edge_constraints.insert(poses_constraints.begin(), poses_constraints.end());

                    PRX_DEBUG_COLOR("new constraints for the edge [" << edge_constraints.size() << "]: " << print(edge_constraints), PRX_TEXT_MAGENTA);
                    edge->add_constraints(edge_constraints);


                    if( specs->transfer_mode )
                    {
                        //Train the poses
                        if( start_pose_id != -1 )
                        {
                            //                            PRX_DEBUG_COLOR("Start: " << state_space->print_point(graph[boost::source(e, graph.graph)]->point, 5), PRX_TEXT_LIGHTGRAY);
                            //                            PRX_DEBUG_COLOR("End  : " << state_space->print_point(graph[boost::target(e, graph.graph)]->point, 5), PRX_TEXT_LIGHTGRAY);
                            specs->manip_state_space->copy_from_point(specs->safe_state);
                            specs->object_space->copy_from_point(start_point);
                            state_space->copy_to_point(collision_check_point);
                            //                            PRX_DEBUG_COLOR("state to be checked for poses (" << start_pose_id << ") :" << state_space->print_point(collision_check_point, 5), PRX_TEXT_BROWN);
                            checker->has_constraints(specs->poses_constraints[start_pose_id], collision_check_point);
                        }

                        if( end_pose_id != -1 )
                        {
                            //                            PRX_DEBUG_COLOR("Start: " << state_space->print_point(graph[boost::source(e, graph.graph)]->point, 5), PRX_TEXT_LIGHTGRAY);
                            //                            PRX_DEBUG_COLOR("End  : " << state_space->print_point(graph[boost::target(e, graph.graph)]->point, 5), PRX_TEXT_LIGHTGRAY);
                            specs->manip_state_space->copy_from_point(specs->safe_state);
                            specs->object_space->copy_from_point(end_point);
                            state_space->copy_to_point(collision_check_point);
                            //                            PRX_DEBUG_COLOR("state to be checked for poses (" << end_pose_id << ") :" << state_space->print_point(collision_check_point, 5), PRX_TEXT_GREEN);
                            checker->has_constraints(specs->poses_constraints[end_pose_id], collision_check_point);
                        }

#ifndef NDEBUG
                        state_space->copy_from_point(path1[0]);
                        specs->object_space->copy_to_point(start_point);

                        state_space->copy_from_point(path1.back());
                        specs->object_space->copy_to_point(end_point);


                        //                        PRX_DEBUG_COLOR("START: " << print(full_state_vec), PRX_TEXT_BLUE);
                        //if( graph[boost::source(e, graph.graph)]->node_id == 11 || graph[boost::target(e, graph.graph)]->node_id == 11 )
                        if( specs->object_space->equal_points(start_point, end_point) )
                        {
                            PRX_DEBUG_COLOR("Start: " << specs->object_space->print_point(start_point, 6), PRX_TEXT_GREEN);
                            PRX_DEBUG_COLOR("End  : " << specs->object_space->print_point(end_point, 6), PRX_TEXT_CYAN);
                            PRX_DEBUG_COLOR("Distance : " << specs->object_space->distance(start_point, end_point), PRX_TEXT_MAGENTA);
                            PRX_DEBUG_COLOR("PATH for " << graph[boost::source(e, graph.graph)]->node_id << " -> " << graph[boost::target(e, graph.graph)]->node_id << " \n" << path1.print(), PRX_TEXT_GREEN);

                        }
                        PRX_ASSERT(!specs->object_space->equal_points(start_point, end_point));
                        //                            PRX_ASSERT(false);
#endif
                    }
                }

                if( specs->transfer_mode )
                {

                    specs->object_space->free_point(start_point);
                    specs->object_space->free_point(end_point);
                    state_space->free_point(collision_check_point);
                }
            }

//             void manipulation_mp_t::inform_graph_full_space(const std::vector< std::pair<unsigned, sim::state_t*> >* poses)
//             {
//                 PRX_DEBUG_COLOR("Informing the graph", PRX_TEXT_CYAN);
//                 system_name_validity_checker_t* checker = dynamic_cast<system_name_validity_checker_t*>(validity_checker);
//                 state_t* collision_check_point = NULL;
//                 state_t* start_point = NULL;
//                 state_t* end_point = NULL;
//                 std::vector<double> full_state_vec(specs->full_collision_object_space->get_dimension());
//                 unsigned object_size = specs->object_space->get_dimension();
//                 std::set<unsigned> edge_constraints;
//                 //                PRX_DEBUG_COLOR("full objs : " << specs->full_collision_object_space->print_memory(5), PRX_TEXT_LIGHTGRAY);
//                 //                PRX_DEBUG_COLOR("Collisiion: " << specs->object_space->print_memory(5), PRX_TEXT_LIGHTGRAY);
//                 if( specs->transfer_mode )
//                 {
//                     start_point = specs->object_space->alloc_point();
//                     end_point = specs->object_space->alloc_point();
//                     collision_check_point = state_space->alloc_point();
//                 }
//                 else
//                 {
//                     for( unsigned i = 0; i < specs->poses_set->size(); ++i )
//                     {
//                         for( unsigned j = 0; j < specs->poses_set->at(i).retracted_set.size(); ++j )
//                         {
//                             const manipulation_node_t* retract_node = metric->single_query(specs->poses_set->at(i).retracted_set[j])->as<manipulation_node_t > ();
//                             const manipulation_node_t* node = metric->single_query(specs->poses_set->at(i).ungrasped_set[j])->as<manipulation_node_t > ();
//                             PRX_ASSERT(state_space->equal_points(retract_node->point, specs->poses_set->at(i).retracted_set[j]));
//                             PRX_ASSERT(state_space->equal_points(node->point, specs->poses_set->at(i).ungrasped_set[j]));
//                             manipulation_edge_t* edge;
//                             if( boost::edge(node->index, retract_node->index, graph.graph).second )
//                             {
//                                 edge = graph.get_edge_as<manipulation_edge_t > (boost::edge(node->index, retract_node->index, graph.graph).first);
//                             }
//                             else
//                             {
//                                 double dist = metric->distance_function(retract_node->point, node->point);
//                                 undirected_edge_index_t e = graph.add_edge<manipulation_edge_t > (retract_node->index, node->index, dist);
//                                 edge = graph.get_edge_as<manipulation_edge_t > (e);
//                                 edge->id = num_edges;
//                                 num_edges++;
//                             }
//                             edge->set_source_vertex(retract_node->index);
//                             //TODO: add the plans
//                             // edge->plan = specs->poses_set->at(i).reaching_plans[j];
//                             // edge->path = specs->poses_set->at(i).reaching_paths[j];
//                             // edge->other_way_plan = specs->poses_set->at(i).retract_plans[j];
//                             // edge->other_way_path = specs->poses_set->at(i).retract_paths[j];
//                         }
//                     }
//                 }

//                 int edge_size = boost::num_edges(graph.graph);
//                 int edge_counter = 0;

//                 unsigned pl = 0;
//                 for( unsigned i = 0; i < poses->size(); ++i )
//                     for( unsigned j = 0; j < object_size; ++j, ++pl )
//                         full_state_vec[pl] = poses->at(i).second->at(j);

//                 //Indices for poses that are involved in this edge.
//                 int start_pose_id;
//                 int end_pose_id;

//                 foreach(undirected_edge_index_t e, boost::edges(graph.graph))
//                 {
//                     PRX_INFO_S("Edge: " << edge_counter << "/" << edge_size);
//                     edge_counter++;
//                     path1.clear();
//                     new_plan.clear();
//                     edge_constraints.clear();
//                     if( specs->transfer_mode )
//                     {

//                         //                        for( unsigned i = 0; i < poses->size(); ++i )
//                         //                            for( unsigned j = 0; j < object_size; ++j, ++pl )
//                         //                                full_state_vec[pl] = poses->at(i).second->at(j);

//                         state_space->copy_from_point(graph[boost::source(e, graph.graph)]->point);
//                         specs->object_space->copy_to_point(start_point);
//                         PRX_DEBUG_COLOR("start posE: " << specs->object_space->print_point(start_point, 8), PRX_TEXT_CYAN);

//                         state_space->copy_from_point(graph[boost::target(e, graph.graph)]->point);
//                         specs->object_space->copy_to_point(end_point);
//                         PRX_DEBUG_COLOR("end posE  : " << specs->object_space->print_point(end_point, 8), PRX_TEXT_CYAN);


//                         PRX_DEBUG_COLOR("Start: " << state_space->print_point(graph[boost::source(e, graph.graph)]->point, 5), PRX_TEXT_LIGHTGRAY);
//                         PRX_DEBUG_COLOR("End  : " << state_space->print_point(graph[boost::target(e, graph.graph)]->point, 5), PRX_TEXT_LIGHTGRAY);

//                         start_pose_id = -1;
//                         end_pose_id = -1;
//                         pl = 0;
//                         bool new_pose;
//                         for( unsigned i = 0; i < poses->size(); ++i )
//                         {
//                             //                            PRX_DEBUG_COLOR("POSE(" << poses->at(i).first << "): " << specs->object_space->print_point(poses->at(i).second,8),PRX_TEXT_GREEN);
//                             new_pose = false;
//                             if( specs->object_space->equal_points(start_point, poses->at(i).second, PRX_ZERO_CHECK) )
//                             {
//                                 start_pose_id = poses->at(i).first;
//                                 new_pose = true;
//                             }
//                             else if( specs->object_space->equal_points(end_point, poses->at(i).second, PRX_ZERO_CHECK) )
//                             {
//                                 end_pose_id = poses->at(i).first;
//                                 new_pose = true;
//                             }

//                             if( new_pose )
//                             {
//                                 PRX_DEBUG_COLOR("Dist start: " << specs->object_space->distance(start_point, poses->at(i).second) << "      End: " << specs->object_space->distance(end_point, poses->at(i).second), PRX_TEXT_MAGENTA);
//                                 edge_constraints.insert(poses->at(i).first);
//                                 //                                PRX_DEBUG_COLOR("POSE: " << poses->at(i).first << "   object_size: " << object_size, PRX_TEXT_RED)
//                                 for( unsigned j = 0; j < object_size; ++j, ++pl )
//                                 {
//                                     if( j == 2 )
//                                     {

//                                         full_state_vec[pl] = -100;
//                                     }
//                                     else
//                                     {
//                                         full_state_vec[pl] = poses->at(i).second->at(j);
//                                     }
//                                 }
//                             }
//                             else
//                             {
//                                 for( unsigned j = 0; j < object_size; ++j, ++pl )
//                                     full_state_vec[pl] = poses->at(i).second->at(j);
//                             }
//                         }
//                         //                        PRX_ASSERT(false);
//                         //                        PRX_DEBUG_COLOR("START: " << print(full_state_vec), PRX_TEXT_MAGENTA);
//                         specs->full_collision_object_space->set_from_vector(full_state_vec);
//                         //                        PRX_DEBUG_COLOR(specs->full_collision_object_space->print_memory(6), PRX_TEXT_BLUE);
//                     }

//                     if( graph.get_edge_as<manipulation_edge_t > (e)->plan.size() > 0 )
//                     {
//                         PRX_DEBUG_COLOR("Has plan : " << graph.get_edge_as<manipulation_edge_t > (e)->source_vertex << " -> " << graph.get_edge_as<manipulation_edge_t > (e)->target_vertex, PRX_TEXT_BLUE);
//                         PRX_ASSERT(graph.get_edge_as<manipulation_edge_t > (e)->path.size() > 0);
//                         checker->has_constraints(edge_constraints, graph.get_edge_as<manipulation_edge_t > (e)->path);
//                     }
//                     else
//                     {
//                         local_planner->steer(graph[boost::source(e, graph.graph)]->point, graph[boost::target(e, graph.graph)]->point, new_plan, path1);
//                         PRX_DEBUG_COLOR("PATH for " << graph[boost::source(e, graph.graph)]->node_id << " -> " << graph[boost::target(e, graph.graph)]->node_id << " \n" << path1.print(), PRX_TEXT_GREEN);

//                         if( state_space->equal_points(path1[0], path1.back(), PRX_DISTANCE_CHECK) )
//                         {
//                             PRX_DEBUG_COLOR("path[0]: " << state_space->print_point(path1[0], 8), PRX_TEXT_GREEN);
//                             PRX_DEBUG_COLOR("path.back: " << state_space->print_point(path1.back(), 8), PRX_TEXT_GREEN);
//                             PRX_DEBUG_COLOR("source  : " << state_space->print_point(graph[boost::source(e, graph.graph)]->point, 8), PRX_TEXT_CYAN);
//                             PRX_DEBUG_COLOR("target  : " << state_space->print_point(graph[boost::target(e, graph.graph)]->point, 8), PRX_TEXT_CYAN);
//                             PRX_DEBUG_COLOR("Dist start: " << state_space->distance(path1[0], graph[boost::source(e, graph.graph)]->point), PRX_TEXT_RED);
//                             PRX_DEBUG_COLOR("Dist end  : " << state_space->distance(path1.back(), graph[boost::target(e, graph.graph)]->point), PRX_TEXT_RED);
//                         }
//                         PRX_ASSERT(state_space->equal_points(path1[0], graph[boost::source(e, graph.graph)]->point, PRX_DISTANCE_CHECK));
//                         //PRX_ASSERT(state_space->equal_points(path1.back(), graph[boost::target(e, graph.graph)]->point, PRX_DISTANCE_CHECK));

//                         PRX_ASSERT(dynamic_cast<system_name_validity_checker_t*>(validity_checker) != NULL);
//                         checker->has_constraints(edge_constraints, path1);
//                     }
//                     if( edge_constraints.size() > 0 )
//                     {
//                         //                        PRX_DEBUG_COLOR("EDGE ( " << graph[boost::source(e, graph.graph)]->node_id << "->" << graph[boost::target(e, graph.graph)]->node_id << ") : " << print(edge_constraints), PRX_TEXT_BROWN);
//                         graph.get_edge_as<manipulation_edge_t > (e)->add_constraints(edge_constraints);
//                     }

//                     if( specs->transfer_mode )
//                     {
//                         //Train the poses
//                         if( start_pose_id != -1 )
//                         {
//                             //                            PRX_DEBUG_COLOR("Start: " << state_space->print_point(graph[boost::source(e, graph.graph)]->point, 5), PRX_TEXT_LIGHTGRAY);
//                             //                            PRX_DEBUG_COLOR("End  : " << state_space->print_point(graph[boost::target(e, graph.graph)]->point, 5), PRX_TEXT_LIGHTGRAY);
//                             specs->manip_state_space->copy_from_point(specs->safe_state);
//                             specs->object_space->copy_from_point(start_point);
//                             state_space->copy_to_point(collision_check_point);
//                             //                            PRX_DEBUG_COLOR("state to be checked for poses (" << start_pose_id << ") :" << state_space->print_point(collision_check_point, 5), PRX_TEXT_BROWN);
//                             checker->has_constraints(specs->poses_constraints[start_pose_id], collision_check_point);
//                         }

//                         if( end_pose_id != -1 )
//                         {
//                             //                            PRX_DEBUG_COLOR("Start: " << state_space->print_point(graph[boost::source(e, graph.graph)]->point, 5), PRX_TEXT_LIGHTGRAY);
//                             //                            PRX_DEBUG_COLOR("End  : " << state_space->print_point(graph[boost::target(e, graph.graph)]->point, 5), PRX_TEXT_LIGHTGRAY);
//                             specs->manip_state_space->copy_from_point(specs->safe_state);
//                             specs->object_space->copy_from_point(end_point);
//                             state_space->copy_to_point(collision_check_point);
//                             //                            PRX_DEBUG_COLOR("state to be checked for poses (" << end_pose_id << ") :" << state_space->print_point(collision_check_point, 5), PRX_TEXT_GREEN);
//                             checker->has_constraints(specs->poses_constraints[end_pose_id], collision_check_point);
//                         }

// #ifndef NDEBUG
//                         state_space->copy_from_point(path1[0]);
//                         specs->object_space->copy_to_point(start_point);

//                         state_space->copy_from_point(path1.back());
//                         specs->object_space->copy_to_point(end_point);


//                         //                        PRX_DEBUG_COLOR("START: " << print(full_state_vec), PRX_TEXT_BLUE);
//                         //if( graph[boost::source(e, graph.graph)]->node_id == 11 || graph[boost::target(e, graph.graph)]->node_id == 11 )
//                         if( specs->object_space->equal_points(start_point, end_point) )
//                         {
//                             PRX_DEBUG_COLOR("Start: " << specs->object_space->print_point(start_point, 6), PRX_TEXT_GREEN);
//                             PRX_DEBUG_COLOR("End  : " << specs->object_space->print_point(end_point, 6), PRX_TEXT_CYAN);
//                             PRX_DEBUG_COLOR("Distance : " << specs->object_space->distance(start_point, end_point), PRX_TEXT_MAGENTA);
//                             PRX_DEBUG_COLOR("PATH for " << graph[boost::source(e, graph.graph)]->node_id << " -> " << graph[boost::target(e, graph.graph)]->node_id << " \n" << path1.print(), PRX_TEXT_GREEN);

//                         }
//                         PRX_ASSERT(!specs->object_space->equal_points(start_point, end_point));
//                         //                            PRX_ASSERT(false);
// #endif
//                     }
//                 }

//                 if( specs->transfer_mode )
//                 {

//                     specs->object_space->free_point(start_point);
//                     specs->object_space->free_point(end_point);
//                     state_space->free_point(collision_check_point);
//                 }
//             }

            void manipulation_mp_t::inform_graph(const std::vector< std::pair<unsigned, sim::state_t*> >* poses)
            {
                PRX_DEBUG_COLOR("Informing the graph", PRX_TEXT_CYAN);
#ifndef NDEBUG
                int cc = boost::connected_components(graph.graph, graph.components);
                std::vector<unsigned> cc_num(cc);

                foreach(undirected_vertex_index_t v, boost::vertices(graph.graph))
                {
                    cc_num[graph.components[v]]++;
                }
                std::string cc_str = "";
                for( int i = 0; i < cc; ++i )
                    cc_str += int_to_str(cc_num[i]) + " , ";
                PRX_DEBUG_COLOR("With " << boost::num_vertices(graph.graph) << " vertices ... " << boost::num_edges(graph.graph) << " edges ... " << cc << " connected components   " << cc_str, PRX_TEXT_CYAN);

                std::vector<double> pos;
                if( specs->object_space != NULL )
                    pos.resize(specs->object_space->get_dimension());
#endif
                state_t* object_point = NULL;
                if( specs->object_space != NULL )
                    object_point = specs->object_space->alloc_point();

                for( unsigned p = 0; p < poses->size(); ++p )
                {
                    specs->object_space->copy_from_point(poses->at(p).second);

                    foreach(undirected_edge_index_t e, boost::edges(graph.graph))
                    {
                        //                        PRX_DEBUG_COLOR("collision: " << specs->object_space->print_point(poses->at(p).second,9),PRX_TEXT_BLUE);
                        if( object_point != NULL )
                        {
                            state_space->copy_from_point(graph[boost::source(e, graph.graph)]->point);
                            specs->object_space->copy_to_point(object_point);
                            if( specs->object_space->equal_points(object_point, poses->at(p).second) )
                                continue;

                            state_space->copy_from_point(graph[boost::target(e, graph.graph)]->point);
                            specs->object_space->copy_to_point(object_point);
                            if( specs->object_space->equal_points(object_point, poses->at(p).second) )
                                continue;
                        }
                        path1.clear();
                        new_plan.clear();


                        //                        PRX_DEBUG_COLOR("collision: " << specs->object_space->print_memory(9),PRX_TEXT_BLUE);
                        //                        PRX_DEBUG_COLOR("object   : " << state_space->print_memory(9),PRX_TEXT_BLUE);
                        //                        PRX_DEBUG_COLOR("Start steering call to local planner \nstart: " << state_space->print_point(graph[boost::source(e, graph.graph)]->point,5), PRX_TEXT_BROWN);
                        local_planner->steer(graph[boost::source(e, graph.graph)]->point, graph[boost::target(e, graph.graph)]->point, new_plan, path1);
                        //                        PRX_DEBUG_COLOR("-----     END     -----"  << "\nend  :" << state_space->print_point(graph[boost::target(e, graph.graph)]->point,5),PRX_TEXT_BROWN);
                        //                        foreach(state_t* state, path1)
                        //                        {
                        //                            PRX_DEBUG_COLOR(state_space->print_point(state,6), PRX_TEXT_GREEN);
                        //                        }
                        PRX_ASSERT(dynamic_cast<system_name_validity_checker_t*>(validity_checker) != NULL);
                        if( !validity_checker->is_valid(path1) )
                        {
                            graph.get_edge_as<manipulation_edge_t > (e)->add_constraint(poses->at(p).first);
                        }
                    }
                }
                if( specs->object_space != NULL )
                    specs->object_space->free_point(object_point);
            }

            void manipulation_mp_t::inform_edges(const std::vector<undirected_edge_index_t>& edges, const std::vector< std::pair<unsigned, sim::state_t*> >* poses)
            {
                PRX_ASSERT(!specs->is_builder);
                PRX_DEBUG_COLOR("Inform Edges", PRX_TEXT_CYAN);
                std::vector<double> pos;
                if( specs->object_space != NULL )
                    pos.resize(specs->object_space->get_dimension());

                for( unsigned p = 0; p < poses->size(); ++p )
                {
                    specs->object_space->copy_from_point(poses->at(p).second);

                    foreach(undirected_edge_index_t e, edges)
                    {
                        path1.clear();
                        new_plan.clear();
                        if( specs->object_space != NULL )
                        {
                            //8 is the state space of the manipulator. 
                            //We need to check if the state has also the objects in the correct position.
                            PRX_ASSERT(state_space->get_dimension() > 8);

                            state_space->copy_from_point(graph[boost::source(e, graph.graph)]->point);
                            //                            specs->_manipulator->get_end_effector_position(pos);
                            //                            specs->object_space->set_from_vector(pos);
                        }
                        local_planner->steer(graph[boost::source(e, graph.graph)]->point, graph[boost::target(e, graph.graph)]->point, new_plan, path1);
                        if( !validity_checker->is_valid(path1) )
                        {
                            graph.get_edge_as<manipulation_edge_t > (e)->add_constraint(poses->at(p).first);
                        }
                    }
                }
            }

            void manipulation_mp_t::add_extra_starting_points()
            {
                undirected_vertex_index_t v_s;
                bool remove_point;

                oa_astar->extra_starts.clear();

                foreach(space_point_t* s, *extra_start_states)
                {
                    boost::tie(remove_point, v_s) = add_node(s);
                    oa_astar->extra_starts.push_back(v_s);
                    remove_starts.push_back(remove_point);
                }
            }

            bool manipulation_mp_t::serialize()
            {
                PRX_DEBUG_COLOR(" Inside manipulation motion planner serialization now, saving to file: " << serialization_file, PRX_TEXT_CYAN);
                std::ofstream fout(serialization_file.c_str());
                if( fout.good() )
                {
                    PRX_ASSERT(fout.is_open());
                    graph.serialize(fout, state_space);
                    fout.close();
                    return true;
                }
                return false;

            }

            bool manipulation_mp_t::deserialize()
            {
                plan_t new_plan;
                trajectory_t new_traj;
                new_plan.link_control_space(this->control_space);
                new_traj.link_space(this->state_space);
                PRX_DEBUG_COLOR(" Inside manipulation motion planner deserialization now, opening file: " << deserialization_file, PRX_TEXT_CYAN);
                std::ifstream input_stream(deserialization_file.c_str());
                if( input_stream.good() )
                {
                    input_stream >> num_vertices;
                    hash_t<int, undirected_vertex_index_t> node_map;
                    for( int i = 0; i < num_vertices; i++ )
                    {
                        undirected_vertex_index_t new_vertex = graph.add_vertex<manipulation_node_t > ();
                        manipulation_node_t* node = graph.get_vertex_as<manipulation_node_t > (new_vertex);
                        node->index = new_vertex;
                        node->node_id = i;
                        node->point = state_space->alloc_point();
                        node->deserialize(input_stream, state_space);
                        node_map[i] = new_vertex;
                    }
                    input_stream >> num_edges;
                    for( int i = 0; i < num_edges; i++ )
                    {
                        int from, to;
                        input_stream >> from >> to;
//                        new_plan.clear();
//                        new_traj.clear();
//                        local_planner->steer(graph[node_map[from]]->point, graph[node_map[to]]->point,new_plan, new_traj);
//                        double dist = validity_checker->trajectory_cost(new_traj);
                        undirected_edge_index_t new_edge = graph.add_edge<manipulation_edge_t > (node_map[from], node_map[to], 0);
                        manipulation_edge_t* edge = graph.get_edge_as<manipulation_edge_t > (new_edge);
                        edge->id = i;
                        edge->link_spaces(this->state_space,this->control_space);                        
                        edge->deserialize(input_stream);
                        edge->v_source = node_map[edge->source_vertex];
                        double dist;
                        input_stream >> dist;
                        graph.set_weight(new_edge,dist);
                    }
#ifndef NDEBUG
                    double val_mu = (double)boost::num_edges(graph.graph) / (double)boost::num_vertices(graph.graph);
                    double diff;
                    double val_dev = 0.0;
#endif

                    foreach(undirected_vertex_index_t nd, boost::vertices(graph.graph))
                    {
                        metric->add_point(graph[nd]);
#ifndef NDEBUG
                        diff = (double)boost::out_degree(nd, graph.graph) - val_mu;
                        val_dev += diff * diff;
#endif
                    }
                    update_k(boost::num_vertices(graph.graph));
                    input_stream.close();

#ifndef NDEBUG
                    val_dev = sqrt(val_dev);
                    int cc = boost::connected_components(graph.graph, graph.components);
                    PRX_DEBUG_COLOR("Deserialized roadmap with " << boost::num_vertices(graph.graph) << " vertices ... " << boost::num_edges(graph.graph) << " edges ... " << cc << " connected components", PRX_TEXT_GREEN);
                    PRX_DEBUG_COLOR("Average Valence: " << val_mu << "   Valence Deviation: " << val_dev, PRX_TEXT_GREEN);
#endif
                    return true;
                }
                PRX_FATAL_S("Could not deserialize the graph from file : " << deserialization_file);
                return false;

            }

            bool manipulation_mp_t::deserialize_simple_graph(std::string graph_file)
            {

                PRX_DEBUG_COLOR(" Inside manipulation motion planner deserialization now, opening file: " << graph_file, PRX_TEXT_CYAN);
                std::ifstream input_stream(graph_file.c_str());
                if( input_stream.good() )
                {
                    input_stream >> num_vertices;
                    hash_t<int, undirected_vertex_index_t> node_map;
                    for( int i = 0; i < num_vertices; i++ )
                    {
                        undirected_vertex_index_t new_vertex = graph.add_vertex<manipulation_node_t > ();
                        manipulation_node_t* node = graph.get_vertex_as<manipulation_node_t > (new_vertex);
                        node->index = new_vertex;
                        node->node_id = i;
                        node->point = state_space->alloc_point();
                        node->deserialize(input_stream, state_space);
                        node_map[i] = new_vertex;
                    }
                    input_stream >> num_edges;
                    for( int i = 0; i < num_edges; i++ )
                    {
                        int from, to;
                        double dist;
                        input_stream >> from >> to >> dist;
//                        double dist = metric->distance_function(graph[node_map[from]]->point, graph[node_map[to]]->point);
                        undirected_edge_index_t new_edge = graph.add_edge<manipulation_edge_t > (node_map[from], node_map[to], dist);
                        manipulation_edge_t* edge = graph.get_edge_as<manipulation_edge_t > (new_edge);
                        edge->id = i;
                        edge->link_spaces(this->state_space, this->control_space);                        
                    }

                    PRX_ASSERT(boost::num_vertices(graph.graph) == num_vertices);
                    PRX_ASSERT(boost::num_edges(graph.graph) == num_edges);
                    double val_mu = num_edges / num_vertices;
                    double diff;
                    double val_dev = 0.0;

                    foreach(undirected_vertex_index_t nd, boost::vertices(graph.graph))
                    {
                        metric->add_point(graph[nd]);
                        diff = (double)boost::out_degree(nd, graph.graph) - val_mu;
                        val_dev += diff * diff;
                    }
                    val_dev = sqrt(val_dev);
                    update_k(num_vertices);
                    input_stream.close();

                    PRX_DEBUG_COLOR("Deserialized roadmap with " << boost::num_vertices(graph.graph) << " vertices ... " << boost::num_edges(graph.graph) << " edges.", PRX_TEXT_MAGENTA);
                    PRX_DEBUG_COLOR("Average Valence: " << val_mu << "   Valence Deviation: " << val_dev, PRX_TEXT_GREEN);

                    return true;
                }
                PRX_FATAL_S("Could not deserialize the graph from file : " << graph_file);
                return false;
            }

            std::string manipulation_mp_t::print(const std::set<unsigned>& constraints)
            {
                std::stringstream output(std::stringstream::out);

                foreach(unsigned i, constraints)
                {
                    output << i << " , ";
                }
                return output.str();
            }

            std::string manipulation_mp_t::print(const std::vector<unsigned>& vec)
            {
                std::stringstream output(std::stringstream::out);

                foreach(unsigned i, vec)
                {
                    output << i << " , ";
                }
                return output.str();
            }

            std::string manipulation_mp_t::print(const std::vector<double>& vec)
            {
                std::stringstream output(std::stringstream::out);

                foreach(double i, vec)
                {
                    output << i << " , ";
                }
                return output.str();
            }
        }
    }
}
