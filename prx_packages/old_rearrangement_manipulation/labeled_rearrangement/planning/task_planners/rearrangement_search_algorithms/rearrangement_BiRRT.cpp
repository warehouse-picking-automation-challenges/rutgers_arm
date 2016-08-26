/**
 * @file rearrangement_search_algorithm.cpp
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

#include "planning/task_planners/rearrangement_search_algorithms/rearrangement_BiRRT.hpp"
#include "planning/task_planners/rearrangement_primitive.hpp"
#include "planning/task_planners/rearrangement_path_planner.hpp"
#include "planning/problem_specifications/rearrangement_search_specification.hpp"
#include "planning/problem_specifications/rearrangement_primitive_specification.hpp"
#include "planning/queries/rearrangement_search_query.hpp"
#include "planning/queries/rearrangement_query.hpp"
#include "planning/modules/path_part.hpp"


#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/goals/multiple_goal_states.hpp"
#include "prx/simulation/plan.hpp"
#include "prx/planning/modules/heuristic_search/astar_module.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"

#include <vector>
#include <numeric>
#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <boost/graph/connected_components.hpp>
#include <set>
#include <boost/graph/compressed_sparse_row_graph.hpp>

PLUGINLIB_EXPORT_CLASS(prx::packages::labeled_rearrangement_manipulation::rearrangement_birrt_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        using namespace manipulation;
        using namespace rearrangement_manipulation;

        namespace labeled_rearrangement_manipulation
        {

            rearrangement_birrt_t::rearrangement_birrt_t() { }

            rearrangement_birrt_t::~rearrangement_birrt_t() { }

            void rearrangement_birrt_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                rearrangement_search_algorithm_t::init(reader, template_reader);
                primitive_timer = parameters::get_attribute_as<double>("primitive_timer", reader, template_reader, 0);

                if( parameters::has_attribute("target_metric", reader, template_reader) )
                {
                    target_metric = parameters::initialize_from_loader<distance_metric_t > ("prx_utilities", reader, "target_metric", template_reader, "target_metric");
                }
                else
                {
                    PRX_FATAL_S("Missing target_metric attribute in BiRRT!");
                }

            }

            void rearrangement_birrt_t::setup()
            {
                rearrangement_search_algorithm_t::setup();
                target_metric->link_space(graph_space);
                statistics_file = prx_output_dir + "Statistics/BiRRT_" + in_specs->statistics_file;
                path_file = prx_output_dir + "paths/BiRRT_" + in_specs->statistics_file;
                print_all = true;
            }

            void rearrangement_birrt_t::resolve_query()
            {
                PRX_DEBUG_COLOR("Resolve_query for rearrangement_birrt_t", PRX_TEXT_CYAN);
                statistics_clock.reset();
                int pose_index;
                //============================//
                //         Initial Node       //
                //============================//
                for( unsigned i = 0; i < in_specs->k_objects; ++i )
                {
                    object_state_space->copy_vector_to_point(in_query->initial_poses[i], object_state);
                    if( (pose_index = detect_pose(object_state)) != -1 )
                    {
                        initial_arrangement[i] = pose_index;
                    }
                    else
                    {
                        PRX_FATAL_S("Initial pose is not included in the informed poses! " << object_state_space->print_point(object_state, 8));
                    }
                }

                v_start = add_node(initial_arrangement);
                metric->add_point(super_graph[v_start]);

                //============================//
                //         Target Node        //
                //============================//
                for( unsigned i = 0; i < in_specs->k_objects; ++i )
                {
                    object_state_space->copy_vector_to_point(in_query->target_poses[i], object_state);
                    if( (pose_index = detect_pose(object_state)) != -1 )
                    {
                        target_arrangement[i] = pose_index;
                    }
                    else
                    {
                        PRX_FATAL_S("Initial pose is not included in the informed poses! " << object_state_space->print_point(object_state, 8));
                    }
                }
                v_target = add_node(target_arrangement);
                target_metric->add_point(super_graph[v_target]);

                //================================//
                // Try to connect Start and Goal  //
                //================================//
                primitive_query->link_spaces(manip_state_space, manip_control_space);
                primitive_query->start_state = safe_state;
                primitive_query->initial_poses_ids = initial_arrangement;
                primitive_query->target_poses_ids = target_arrangement;
                primitive_query->time_limit = in_specs->time_limit - statistics_clock.measure();
                //                primitive_query->time_limit = primitive_timer;
                primitive_query->accept_partial_solutions = false;
                primitive->link_query(primitive_query);
                primitive->resolve_query();
                stats->sequence_stats.push_back(primitive->get_statistics()->as<rearrangement_primitive_statistics_t > ());

                if( primitive_query->found_solution() )
                {
                    super_graph.add_edge<rearrangement_edge_t > (v_start, v_target);
                    primitive_query->time_limit = in_specs->time_limit - statistics_clock.measure();
                    path_planner->link_query(primitive_query);
                    path_planner->resolve_query();
                    update_stats(true);
                    in_query->plan = primitive_query->plan;
                    std::ofstream fout(path_file.c_str());
                    PRX_ASSERT(fout.is_open());
                    in_query->plan.save_to_stream(fout, 6);
                    fout.close();
                    return;
                }

                while( statistics_clock.measure() < in_specs->time_limit )
                {
                    std::vector<unsigned> arrangement;
                    sample_arrangement(arrangement);
                    PRX_DEBUG_COLOR("new Arragement : " << print(arrangement), PRX_TEXT_MAGENTA);
                    if(try_connect(arrangement) )
                    {

                        primitive_query->clear();
                        in_specs->astar->link_graph(&super_graph);
                        in_specs->astar->set_astar_mode(astar_module_t::PRX_NO_EDGE_INFO);

                        foreach(undirected_vertex_index_t v, boost::vertices(super_graph.graph))
                        {
                            PRX_DEBUG_COLOR(super_graph.get_vertex_as<rearrangement_node_t > (v)->print(), PRX_TEXT_GREEN);
                        }

                        foreach(undirected_edge_index_t e, boost::edges(super_graph.graph))
                        {
                            PRX_DEBUG_COLOR(super_graph.get_edge_as<rearrangement_edge_t > (e)->source_vertex << " -> " << super_graph.get_edge_as<rearrangement_edge_t > (e)->target_vertex, PRX_TEXT_CYAN);
                        }
                        if( in_specs->astar->solve(v_start, v_target) )
                        {
                            std::deque< undirected_vertex_index_t > path_vertices;
                            PRX_ASSERT(in_specs->astar->get_found_goal() == v_target);
                            in_specs->astar->extract_path(v_start, in_specs->astar->get_found_goal(), path_vertices);
                            PRX_DEBUG_COLOR("A* got solution : " << path_vertices.size(), PRX_TEXT_MAGENTA);
                            for( int i = 0; i < (int)path_vertices.size() - 1; ++i )
                            {
                                undirected_edge_index_t e = boost::edge(path_vertices[i], path_vertices[i + 1], super_graph.graph).first;
                                rearrangement_edge_t* edge = super_graph.get_edge_as<rearrangement_edge_t > (e);
                                PRX_DEBUG_COLOR(edge->print(), PRX_TEXT_CYAN);
                                edge->append_plan(primitive_query->path_sequence, path_vertices[i]);
                            }
                        }
                        primitive_query->initial_poses_ids = initial_arrangement;
                        primitive_query->target_poses_ids = target_arrangement;
                        primitive_query->time_limit = in_specs->time_limit - statistics_clock.measure();
                        path_planner->link_query(primitive_query);
                        path_planner->resolve_query();
                        update_stats(true);
                        in_query->plan = primitive_query->plan;
                        std::ofstream fout(path_file.c_str());
                        PRX_ASSERT(fout.is_open());
                        in_query->plan.save_to_stream(fout, 6);
                        fout.close();
                        return;
                    }
                }
                update_stats(false);
            }

            bool rearrangement_birrt_t::try_connect(std::vector<unsigned>& arrangement)
            {
                undirected_vertex_index_t u;
                undirected_edge_index_t e;
                undirected_vertex_index_t v = expand(arrangement, metric);
                if( primitive_query->got_partial_solution )
                    arrangement = primitive_query->partial_solution;
                if( primitive_query->got_solution || primitive_query->got_partial_solution )
                {
                    u = node_exists(arrangement);
                    if( u == NULL )
                    {
                        u = add_node(arrangement);
                        metric->add_point(super_graph[u]);
                    }
                    if( u != v )
                    {
                        PRX_DEBUG_COLOR("Add edge start: " << print(super_graph.get_vertex_as<rearrangement_node_t > (v)->arrangement) << " -> " << print(arrangement) << "  path:" << primitive_query->path_sequence.size(), PRX_TEXT_LIGHTGRAY);
                        e = super_graph.add_edge<rearrangement_edge_t > (v, u, metric->distance_function(super_graph[v]->point, super_graph[u]->point));
                        super_graph.get_edge_as<rearrangement_edge_t > (e)->init(v, u, primitive_query->path_sequence);
                    }
                }
                else
                {
                    arrangement = super_graph.get_vertex_as<rearrangement_node_t > (v)->arrangement;
                    u = v;
                }

                if( statistics_clock.measure() < in_specs->time_limit )
                {
                    //Now the arrangement has the new arrangement from the expanded point or 
                    //the closest point from the start tree if expansion didn't happen.
                    v = expand(arrangement, target_metric);
                    if( primitive_query->got_solution )
                    {
                        PRX_DEBUG_COLOR("CONNECTION:  " << print(super_graph.get_vertex_as<rearrangement_node_t > (v)->arrangement) << " -> " << print(arrangement) << "  path:" << primitive_query->path_sequence.size(), PRX_TEXT_BROWN);
                        e = super_graph.add_edge<rearrangement_edge_t > (v, u, metric->distance_function(super_graph[v]->point, super_graph[u]->point));
                        super_graph.get_edge_as<rearrangement_edge_t > (e)->init(v, u, primitive_query->path_sequence);
                        return true;
                    }

                    if( primitive_query->got_partial_solution )
                    {
                        arrangement = primitive_query->partial_solution;

                        u = node_exists(arrangement);
                        if( u == NULL )
                        {
                            u = add_node(arrangement);
                            target_metric->add_point(super_graph[u]);
                        }
                        if( u != v )
                        {
                            PRX_DEBUG_COLOR("Add edge end: " << print(super_graph.get_vertex_as<rearrangement_node_t > (v)->arrangement) << " -> " << print(arrangement) << "  path:" << primitive_query->path_sequence.size(), PRX_TEXT_LIGHTGRAY);
                            e = super_graph.add_edge<rearrangement_edge_t > (v, u, metric->distance_function(super_graph[v]->point, super_graph[u]->point));
                            super_graph.get_edge_as<rearrangement_edge_t > (e)->init(v, u, primitive_query->path_sequence);
                        }
                    }
                }

                return false;
            }

            undirected_vertex_index_t rearrangement_birrt_t::expand(std::vector<unsigned>& arrangement, distance_metric_t* metric, bool accept_partial_solutions)
            {
                stats->num_connections_tries++;
                generate_graph_point(graph_point, arrangement);
                const rearrangement_node_t* node = metric->single_query(graph_point)->as<rearrangement_node_t > ();
                primitive_query->clear();
                primitive_query->initial_poses_ids = node->arrangement;
                primitive_query->target_poses_ids = arrangement;
                primitive_query->time_limit = in_specs->time_limit - statistics_clock.measure();
                //                primitive_query->time_limit = primitive_timer;
                primitive_query->accept_partial_solutions = accept_partial_solutions;
                primitive->link_query(primitive_query);
                primitive->resolve_query();
                stats->sequence_stats.push_back(primitive->get_statistics()->as<rearrangement_primitive_statistics_t > ());
                return node->index;
            }

            void rearrangement_birrt_t::update_stats(bool found_path)
            {
                stats->found_path = found_path;
                stats->computation_time = statistics_clock.measure();
                stats->num_v = boost::num_vertices(super_graph.graph);
                stats->num_e = boost::num_edges(super_graph.graph);
                stats->num_cc = boost::connected_components(super_graph.graph, super_graph.components);
                stats->planner_stats = path_planner->get_statistics()->as<rearrangement_primitive_statistics_t > ();
            }



        }
    }
}

