/**
 * @file rearrangement_prm.cpp
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

#include "planning/task_planners/rearrangement_search_algorithms/rearrangement_PRM.hpp"
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

PLUGINLIB_EXPORT_CLASS(prx::packages::labeled_rearrangement_manipulation::rearrangement_prm_t, prx::plan::planner_t)

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

            rearrangement_prm_t::rearrangement_prm_t() { }

            rearrangement_prm_t::~rearrangement_prm_t() { }

            void rearrangement_prm_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                rearrangement_search_algorithm_t::init(reader, template_reader);
                primitive_timer = parameters::get_attribute_as<double>("primitive_timer", reader, template_reader, 0);
            }

            void rearrangement_prm_t::setup()
            {
                rearrangement_search_algorithm_t::setup();
                statistics_file = prx_output_dir + "Statistics/PRM_" + in_specs->statistics_file;
                path_file = prx_output_dir + "paths/PRM_" + in_specs->statistics_file;
                print_all = true;
            }

            void rearrangement_prm_t::resolve_query()
            {
                PRX_DEBUG_COLOR("Resolve_query for rearrangement_prm_t", PRX_TEXT_CYAN);
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
                metric->add_point(super_graph[v_target]);

                //================================//
                // Try to connect Start and Goal  //
                //================================//
                primitive_query->link_spaces(manip_state_space, manip_control_space);
                primitive_query->start_state = safe_state;

                if(connect_nodes(v_start, v_target))
                {
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
                update_k(2);


                while( statistics_clock.measure() < in_specs->time_limit )
                {
                    std::vector<unsigned> arrangement;
                    do{
                        sample_arrangement(arrangement);
                        if(statistics_clock.measure() > in_specs->time_limit)
                        {
                            update_stats(false);
                            return;
                        }
                    }while(node_exists(arrangement) != NULL);
                    
                    if( try_connect(arrangement) )
                    {
                        primitive_query->clear();
                        in_specs->astar->link_graph(&super_graph);
                        in_specs->astar->set_astar_mode(astar_module_t::PRX_NO_EDGE_INFO);

                        // foreach(undirected_vertex_index_t v, boost::vertices(super_graph.graph))
                        // {
                        //     PRX_DEBUG_COLOR(super_graph.get_vertex_as<rearrangement_node_t > (v)->print(), PRX_TEXT_GREEN);
                        // }

                        // foreach(undirected_edge_index_t e, boost::edges(super_graph.graph))
                        // {
                        //     PRX_DEBUG_COLOR(super_graph.get_edge_as<rearrangement_edge_t > (e)->source_vertex << " -> " << super_graph.get_edge_as<rearrangement_edge_t > (e)->target_vertex, PRX_TEXT_CYAN);
                        // }
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

            bool rearrangement_prm_t::try_connect(std::vector<unsigned>& arrangement)
            {
                undirected_vertex_index_t v = add_node(arrangement);
                PRX_DEBUG_COLOR("Nodes: " << boost::num_vertices(super_graph.graph), PRX_TEXT_MAGENTA);

                bool start_connect = connect_nodes(v,v_start);
                bool end_connect = connect_nodes(v,v_target);

                if( start_connect && end_connect )
                {
                    return true;
                }

                std::vector<const abstract_node_t*> neighbors = metric->multi_query(super_graph[v], k_near);

                boost::connected_components(super_graph.graph,super_graph.components);
                foreach(const abstract_node_t* node, neighbors)
                {
                    if( statistics_clock.measure() > in_specs->time_limit )
                        return false;

                    undirected_vertex_index_t u = node->as< undirected_node_t > ()->index;
                    if(super_graph.components[v] != super_graph.components[u] && connect_nodes(v,u))
                    {                        
                        boost::connected_components(super_graph.graph,super_graph.components);
                        if(super_graph.components[v_start] == super_graph.components[v_target])
                            return true;
                    }
                }
                update_k(boost::num_vertices(super_graph.graph));
                metric->add_point(super_graph[v]);
                return false;
            }

            bool rearrangement_prm_t::connect_nodes(undirected_vertex_index_t v, undirected_vertex_index_t u)
            {
                stats->num_connections_tries++;
                PRX_DEBUG_COLOR("Try new coonection " << stats->num_connections_tries , PRX_TEXT_BROWN);
                primitive_query->clear();                
                primitive_query->initial_poses_ids = super_graph.get_vertex_as<rearrangement_node_t>(v)->arrangement;
                primitive_query->target_poses_ids = super_graph.get_vertex_as<rearrangement_node_t>(u)->arrangement;
                primitive_query->time_limit = in_specs->time_limit - statistics_clock.measure();
                primitive_query->accept_partial_solutions = false;
                primitive->link_query(primitive_query);
                primitive->resolve_query();
                stats->sequence_stats.push_back(primitive->get_statistics()->as<rearrangement_primitive_statistics_t > ());
                if( primitive_query->got_solution )
                {
                    PRX_DEBUG_COLOR("CONNECTION:  " << print(super_graph.get_vertex_as<rearrangement_node_t > (v)->arrangement) << " -> " << print(super_graph.get_vertex_as<rearrangement_node_t > (u)->arrangement) << "  path:" << primitive_query->path_sequence.size(), PRX_TEXT_BROWN);
                    undirected_edge_index_t e = super_graph.add_edge<rearrangement_edge_t > (v, u, metric->distance_function(super_graph[v]->point, super_graph[u]->point));
                    super_graph.get_edge_as<rearrangement_edge_t > (e)->init(v, u, primitive_query->path_sequence);
                    return true;
                }

                return false;
            }

            void rearrangement_prm_t::update_stats(bool found_path)
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

