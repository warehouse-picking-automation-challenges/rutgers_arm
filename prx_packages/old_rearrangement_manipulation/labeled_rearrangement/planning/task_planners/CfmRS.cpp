///**
// * @file CfmRS.cpp
// *
// * @copyright Software License Agreement (BSD License)
// * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick 
// * All Rights Reserved.
// * For a full description see the file named LICENSE.
// *
// * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
// *
// * Email: pracsys@googlegroups.com
// */
//
//#include "planning/task_planners/CfmRS.hpp"
//
//#include "prx/utilities/definitions/random.hpp"
//#include "prx/utilities/spaces/space.hpp"
//#include "prx/utilities/statistics/statistics.hpp"
//#include "prx/utilities/goals/multiple_goal_states.hpp"
//#include "prx/simulation/plan.hpp"
//#include "prx/planning/modules/heuristic_search/astar_module.hpp"
//#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
//#include "prx/planning/queries/motion_planning_query.hpp"
//
//#include "../../../manipulation/planning/modules/pose.hpp"
//#include "../../../rearrangement_manipulation/planning/task_planners/manipulator_tp.hpp"
//#include "../../../rearrangement_manipulation/planning/problem_specifications/rearrangement_manipulation_specification.hpp"
//
//
//#include <vector>
//#include <numeric>
//#include <pluginlib/class_list_macros.h>
//#include <boost/assign/list_of.hpp>
//#include <boost/graph/connected_components.hpp>
//#include <set>
//#include <boost/graph/compressed_sparse_row_graph.hpp>
//
//PLUGINLIB_EXPORT_CLASS(prx::packages::labeled_rearrangement_manipulation::cfmrs_t, prx::plan::planner_t)
//
//namespace prx
//{
//    using namespace util;
//    using namespace sim;
//    using namespace plan;
//
//    namespace packages
//    {
//        using namespace manipulation;
//        using namespace rearrangement_manipulation;
//
//        namespace labeled_rearrangement_manipulation
//        {
//
//            cfmrs_t::cfmrs_t()
//            {
//                found_path = false;
//                num_of_moved_objects = 0;
//                computation_time = 0;
//                path_length = 0;
//                path_length_smoothed = 0;
//                smoothing_time = 0;
//                full_time = 0;
//                num_cc = 0;
//                num_connections_tries = 0;
//                num_successful_connections = 0;
//                everything_time = 0;
//                average_node_connection_time = 0;
//                max_connection_time = 0;
//                min_connection_time = PRX_INFINITY;
//
//                average_node_connection_time_success = 0;
//                max_connection_time_success = 0;
//                min_connection_time_success = PRX_INFINITY;
//
//                average_node_connection_time_failed = 0;
//                max_connection_time_failed = 0;
//                min_connection_time_failed = PRX_INFINITY;
//            }
//
//            cfmrs_t::~cfmrs_t() { }
//
//            void cfmrs_t::setup()
//            {
//                mrs_t::setup();
//                init_objects.resize(k_objects);
//                rand_arrangement.resize(k_objects);
//                graph_poses.resize(k_objects);
//
//                for( unsigned i = 0; i < k_objects; ++i )
//                    init_objects[i] = i;
//
//                PRX_DEBUG_COLOR("init_objects: " << print(init_objects), PRX_TEXT_LIGHTGRAY);
//                initial_states.push_back(safe_state);
//
//                std::string space_name;
//                for( unsigned i = 0; i < k_objects; ++i )
//                {
//                    graph_space_memory.push_back(new double());
//                    graph_space_memory.push_back(new double());
//                    space_name += "XY";
//                    if( i < k_objects - 1 )
//                        space_name += "|";
//                }
//                PRX_DEBUG_COLOR("Space_name: " << space_name, PRX_TEXT_LIGHTGRAY);
//                graph_vec.resize(2 * k_objects);
//                graph_space = new space_t(space_name, graph_space_memory);
//                graph_point = graph_space->alloc_point();
//
//                metric->link_space(graph_space);
//            }
//
//            const statistics_t* cfmrs_t::get_statistics()
//            {
//                if( gather_statistics )
//                {
//                    num_cc = boost::connected_components(super_graph.graph, super_graph.components);
//                    std::string file = prx_output_dir + "Statistics/" + specs->statistics_file;
//                    std::ofstream fout(file.c_str());
//                    PRX_ASSERT(fout.is_open());
//                    average_node_connection_time /= num_connections_tries;
//                    average_node_connection_time_success /= num_successful_connections;
//                    average_node_connection_time_failed /= (num_connections_tries - num_successful_connections);
//                    fout << "objects \t found path \t object moved \t Time \t Length \t |V| \t |E| \t |CC| \t CON tries \t CON Succ \t Aver T\t Min T \t Max T\t Aver ST \t MIN ST \t MAX ST \t Aver FT \t MIN FT \t MAX FT" << std::endl;
//                    fout << k_objects << " \t " << found_path << " \t " << num_of_moved_objects << " \t " << everything_time << " \t " << path_length << " \t ";
//                    fout << boost::num_vertices(super_graph.graph) << " \t " << boost::num_edges(super_graph.graph) << " \t " << num_cc << " \t " << num_connections_tries << " \t " << num_successful_connections << " \t ";
//                    fout << average_node_connection_time << " \t " << min_connection_time << " \t " << max_connection_time << " \t " << average_node_connection_time_success << " \t " << min_connection_time_success << " \t " << max_connection_time_success << " \t " << average_node_connection_time_failed << " \t " << min_connection_time_failed << " \t " << max_connection_time_failed << std::endl;
//                    fout << connection_times.str();
//
//                    if( specs->apply_smoothing )
//                    {
//                        full_time = everything_time + smoothing_time;
//                        fout << std::endl << smoothing_time << " \t " << path_length_smoothed << " \t " << full_time;
//                    }
//
//
//                    fout << std::endl;
//
//                    fout.close();
//                }
//
//                return new statistics_t();
//            }
//
//            void cfmrs_t::resolve_query()
//            {
//                final_query = dynamic_cast<motion_planning_query_t*>(input_query);
//                final_query->plan.link_control_space(manip_control_space);
//
//                statistics_clock.reset();
//                //============================//
//                //         Initial Node       //
//                //============================//
//                v_initial = super_graph.add_vertex<crs_node_t > ();
//                for( unsigned i = 0; i < k_objects; ++i )
//                {
//                    graph_vec[i * 2] = poses_set[initial_poses_ids[i]].state->memory[0];
//                    graph_vec[i * 2 + 1] = poses_set[initial_poses_ids[i]].state->memory[1];
//                }
//                start_node = super_graph.get_vertex_as<crs_node_t > (v_initial);
//                start_node->init(graph_space, graph_vec, initial_poses_ids);
//                //                metric->add_point(super_graph[v_initial]);
//                PRX_DEBUG_COLOR("============ NEW ARRANGEMENT ============", PRX_TEXT_BLUE);
//                PRX_DEBUG_COLOR("       " << print(initial_poses_ids), PRX_TEXT_MAGENTA);
//                PRX_DEBUG_COLOR(print(graph_vec), PRX_TEXT_CYAN)
//                PRX_DEBUG_COLOR("=========================================", PRX_TEXT_BLUE);
//
//                //============================//
//                //         Target Node        //
//                //============================//
//                v_target = super_graph.add_vertex<crs_node_t > ();
//                for( unsigned i = 0; i < k_objects; ++i )
//                {
//                    graph_vec[i * 2] = poses_set[target_poses_ids[i]].state->memory[0];
//                    graph_vec[i * 2 + 1] = poses_set[target_poses_ids[i]].state->memory[1];
//                }
//                target_node = super_graph.get_vertex_as<crs_node_t > (v_target);
//                target_node->init(graph_space, graph_vec, target_poses_ids);
//                //                metric->add_point(super_graph[v_target]);
//                PRX_DEBUG_COLOR("============ NEW ARRANGEMENT ============", PRX_TEXT_BLUE);
//                PRX_DEBUG_COLOR("       " << print(target_poses_ids), PRX_TEXT_MAGENTA);
//                PRX_DEBUG_COLOR(print(graph_vec), PRX_TEXT_CYAN)
//                PRX_DEBUG_COLOR("=========================================", PRX_TEXT_BLUE);
//
//
//                if( connect_nodes(start_node, target_node) )
//                {
//                    undirected_edge_index_t e = boost::edge(v_initial, v_target, super_graph.graph).first;
//                    crs_edge_t* edge = super_graph.get_edge_as<crs_edge_t > (e);
//                    edge->get_plan(final_query->plan, v_initial);
//                    found_path = true;
//                    everything_time = statistics_clock.measure();
//                    path_length = final_query->plan.size();
//                    return;
//                }
//
//                num_cc = boost::connected_components(super_graph.graph, super_graph.components);
//                update_k(boost::num_vertices(super_graph.graph));
//                while( super_graph.components[v_initial] != super_graph.components[v_target] )
//                {
//                    if( statistics_clock.measure() > specs->time_limit )
//                    {
//                        found_path = false;
//                        everything_time = statistics_clock.measure();
//                        return;
//                    }
//                    if( sample_arrangement(rand_arrangement) )
//                    {
//                        PRX_DEBUG_COLOR("============ NEW ARRANGEMENT ============", PRX_TEXT_BLUE);
//                        PRX_DEBUG_COLOR("       " << print(rand_arrangement), PRX_TEXT_MAGENTA);
//                        PRX_DEBUG_COLOR(print(graph_vec), PRX_TEXT_CYAN);
//                        PRX_DEBUG_COLOR("vertices:" << boost::num_vertices(super_graph.graph) << "   edges:" << boost::num_edges(super_graph.graph) << "   num_cc: " << num_cc << "   metric:" << metric->get_nr_points(), PRX_TEXT_GREEN);
//
//                        undirected_vertex_index_t v = super_graph.add_vertex<crs_node_t > ();
//                        crs_node_t* v_node = super_graph.get_vertex_as<crs_node_t > (v);
//                        v_node->init(graph_space, graph_vec, rand_arrangement);
//
//                        if( connect_nodes(start_node, v_node) )
//                        {
//                            num_cc = boost::connected_components(super_graph.graph, super_graph.components);
//                            if( super_graph.components[v_initial] == super_graph.components[v_target] )
//                                break;
//                        }
//
//                        if( connect_nodes(target_node, v_node) )
//                        {
//                            num_cc = boost::connected_components(super_graph.graph, super_graph.components);
//                            if( super_graph.components[v_initial] == super_graph.components[v_target] )
//                                break;
//                        }
//
//                        std::vector<const abstract_node_t*> neighbors;
//                        neighbors = metric->multi_query(super_graph[v], k_near);
//                        PRX_DEBUG_COLOR("k_near: " << k_near << "    neighbors: " << neighbors.size(), PRX_TEXT_GREEN);
//                        PRX_DEBUG_COLOR("=========================================", PRX_TEXT_BLUE);
//
//                        for( unsigned i = 0; i < neighbors.size(); ++i )
//                        {
//                            if( statistics_clock.measure() > specs->time_limit )
//                            {
//                                found_path = false;
//                                everything_time = statistics_clock.measure();
//                                return;
//                            }
//                            if( connect_nodes(neighbors[i]->as< crs_node_t > (), v_node) )
//                            {
//                                num_cc = boost::connected_components(super_graph.graph, super_graph.components);
//                                if( super_graph.components[v_initial] == super_graph.components[v_target] )
//                                    break;
//                            }
//                        }
//
//                        metric->add_point(super_graph[v]);
//                        update_k(boost::num_vertices(super_graph.graph));
//                    }
//                    num_cc = boost::connected_components(super_graph.graph, super_graph.components);
//
//                }
//
//                specs->astar->link_graph(&super_graph);
//                specs->astar->set_astar_mode(astar_module_t::PRX_NO_EDGE_INFO);
//                if( specs->astar->solve(v_initial, v_target) )
//                {
//                    std::deque< undirected_vertex_index_t > path_vertices;
//                    PRX_ASSERT(specs->astar->get_found_goal() == v_target);
//                    specs->astar->extract_path(v_initial, specs->astar->get_found_goal(), path_vertices);
//                    for( int i = 0; i < (int)path_vertices.size() - 1; ++i )
//                    {
//                        undirected_edge_index_t e = boost::edge(path_vertices[i], path_vertices[i + 1], super_graph.graph).first;
//                        crs_edge_t* edge = super_graph.get_edge_as<crs_edge_t > (e);
//                        edge->append_plan(final_query->plan, path_vertices[i]);
//                    }
//                }
//
//                everything_time = statistics_clock.measure();
//                path_length = final_query->plan.size();
//                found_path = true;
//                return;
//            }
//
//            bool cfmrs_t::connect_nodes(const crs_node_t* v, const crs_node_t* u)
//            {
//                if( statistics_clock.measure() > specs->time_limit )
//                {
//                    found_path = false;
//                    everything_time = statistics_clock.measure();
//                    return false;
//                }
//                computation_time = statistics_clock.measure();
//                num_connections_tries++;
//                PRX_DEBUG_COLOR("============ TRY TO CONNECT ============", PRX_TEXT_BROWN);
//                PRX_DEBUG_COLOR(print(v->arrangement) << " -> " << print(u->arrangement), PRX_TEXT_MAGENTA);
//
//                std::deque<path_part_t> plan_parts;
//                plan_t plan;
//                plan.link_control_space(manip_control_space);
//                if( fmrs(plan_parts, v->arrangement, u->arrangement) && combine_path(plan, plan_parts, v->arrangement) )
//                {
//                    undirected_edge_index_t e = super_graph.add_edge<crs_edge_t > (v->index, u->index, metric->distance_function(u->point, v->point));
//                    super_graph.get_edge_as<crs_edge_t > (e)->init(v->index, u->index, plan);
//                    PRX_DEBUG_COLOR("                 YES                    ", PRX_TEXT_GREEN);
//                    PRX_DEBUG_COLOR("========================================", PRX_TEXT_BROWN);
//                    computation_time = statistics_clock.measure() - computation_time;
//                    average_node_connection_time += computation_time;
//                    min_connection_time = PRX_MINIMUM(min_connection_time, computation_time);
//                    max_connection_time = PRX_MAXIMUM(max_connection_time, computation_time);
//
//                    average_node_connection_time_success += computation_time;
//                    min_connection_time_success = PRX_MINIMUM(min_connection_time_success, computation_time);
//                    max_connection_time_success = PRX_MAXIMUM(max_connection_time_success, computation_time);
//                    connection_times << computation_time << " , 1" << std::endl;
//                    num_successful_connections++;
//                    return true;
//                }
//                PRX_DEBUG_COLOR("                 NO                     ", PRX_TEXT_RED);
//                PRX_DEBUG_COLOR("========================================", PRX_TEXT_BROWN);
//                computation_time = statistics_clock.measure() - computation_time;
//                average_node_connection_time += computation_time;
//                min_connection_time = PRX_MINIMUM(min_connection_time, computation_time);
//                max_connection_time = PRX_MAXIMUM(max_connection_time, computation_time);
//
//                average_node_connection_time_failed += computation_time;
//                min_connection_time_failed = PRX_MINIMUM(min_connection_time_failed, computation_time);
//                max_connection_time_failed = PRX_MAXIMUM(max_connection_time_failed, computation_time);
//                connection_times << computation_time << " , 0 " << std::endl;
//                return false;
//                //                computation_time = statistics_clock.measure();
////
////                objects = init_objects;
////
////                for( unsigned i = 0; i < k_objects; ++i )
////                {
////                    std::vector<unsigned> a_curr = v->arrangement;
////                    unsigned id = objects[0];
////                    objects.pop_front();
////                    PRX_DEBUG_COLOR("id: " << id << "from: " << poses_set[v->arrangement[id]].pose_id << "   goes: " << poses_set[u->arrangement[id]].pose_id << "  objects: " << print(objects), PRX_TEXT_LIGHTGRAY);
////                    plan_t plan;
////                    if( mrs(plan, id, initial_states, poses_set[u->arrangement[id]], objects, a_curr, u->arrangement) )
////                    {
////                        undirected_edge_index_t e = super_graph.add_edge<crs_edge_t > (v->index, u->index, metric->distance_function(u->point, v->point));
////                        super_graph.get_edge_as<crs_edge_t > (e)->init(v->index, u->index, plan);
////                        PRX_DEBUG_COLOR("                 YES                    ", PRX_TEXT_GREEN);
////                        PRX_DEBUG_COLOR("========================================", PRX_TEXT_BROWN);
////                        computation_time = statistics_clock.measure() - computation_time;
////                        average_node_connection_time += computation_time;
////                        min_connection_time = PRX_MINIMUM(min_connection_time, computation_time);
////                        max_connection_time = PRX_MAXIMUM(max_connection_time, computation_time);
////
////                        average_node_connection_time_success += computation_time;
////                        min_connection_time_success = PRX_MINIMUM(min_connection_time_success, computation_time);
////                        max_connection_time_success = PRX_MAXIMUM(max_connection_time_success, computation_time);
////                        connection_times << computation_time << " , 1" << std::endl;
////                        num_successful_connections++;
////                        return true;
////                    }
////                    objects.push_back(id);
////                }
////                PRX_DEBUG_COLOR("                 NO                     ", PRX_TEXT_RED);
////                PRX_DEBUG_COLOR("========================================", PRX_TEXT_BROWN);
////                computation_time = statistics_clock.measure() - computation_time;
////                average_node_connection_time += computation_time;
////                min_connection_time = PRX_MINIMUM(min_connection_time, computation_time);
////                max_connection_time = PRX_MAXIMUM(max_connection_time, computation_time);
////
////                average_node_connection_time_failed += computation_time;
////                min_connection_time_failed = PRX_MINIMUM(min_connection_time_failed, computation_time);
////                max_connection_time_failed = PRX_MAXIMUM(max_connection_time_failed, computation_time);
////                connection_times << computation_time << " , 0 " << std::endl;
////                return false;
//            }
//
//            bool cfmrs_t::sample_arrangement(std::vector<unsigned>& arrangement)
//            {
//                arrangement_size = 0;
//                std::fill(pose_checked.begin(), pose_checked.end(), false);
//                unsigned index = 0;
//                unsigned num_checked = 0;
//                while( arrangement_size != k_objects && num_checked < poses_set_length )
//                {
//                    int failures = -1;
//                    do
//                    {
//                        int bias_chance = uniform_int_random(0, 99);
//                        if( ++failures == specs->max_random_failures )
//                        {
//                            return false;
//                        }
//                        //5 % of the time we are biased towards the goal/initial positions
//                        //Poses have the same id as their place in the poses_set.
//                        if( bias_chance < specs->goal_biasing )
//                        {
//                            if( uniform_int_random(0, 1) == 0 )
//                                index = target_poses_ids[uniform_int_random(0, k_objects - 1)];
//                            else
//                                index = initial_poses_ids[uniform_int_random(0, k_objects - 1)];
//                        }
//                        else
//                        {
//                            index = uniform_int_random(0, poses_set_length - 1);
//                        }
//                    }
//                    while( pose_checked[index] );
//                    pose_checked[index] = true;
//                    ++num_checked;
//
//                    if( valid_pose(poses_set[index].state) )
//                    {
//                        PRX_ASSERT(index == poses_set[index].pose_id);
//                        graph_poses[arrangement_size] = poses_set[index].state;
//                        arrangement[arrangement_size] = index;
//                        graph_vec[arrangement_size * 2] = poses_set[index].state->memory[0];
//                        graph_vec[arrangement_size * 2 + 1] = poses_set[index].state->memory[1];
//                        arrangement_size++;
//
//                    }
//                }
//
//                //We can reach here if we check all the possible poses but none of them worked.
//                return arrangement_size == k_objects;
//
//            }
//
//            bool cfmrs_t::valid_pose(const state_t* new_pose)
//            {
//                std::string old_context = model->get_current_context();
//                model->use_context(pc_name_collision_check);
//
//                //Set as active state the new pose and iterate over the rest poses.
//                collision_object_space->copy_from_point(new_pose);
//
//                for( unsigned i = 0; i < arrangement_size; ++i )
//                {
//                    if( !validity_checker->is_valid(graph_poses[i]) )
//                    {
//                        model->use_context(old_context);
//                        return false;
//                    }
//
//                }
//                model->use_context(old_context);
//                return true;
//            }
//
//            bool cfmrs_t::node_exists(const std::vector<unsigned>& arrangement)
//            {
//
//                foreach(undirected_vertex_index_t v, boost::vertices(super_graph.graph))
//                {
//                    if( super_graph.get_vertex_as<crs_node_t > (v)->same_arrangement(arrangement) )
//                        return true;
//                }
//                return false;
//            }
//
//            void cfmrs_t::update_k(unsigned nr_nodes)
//            {
//                if( nr_nodes == 0 )
//                {
//                    k_near = 0;
//                }
//                else
//                {
//                    double d = graph_space->get_dimension();
//                    double val = (1.0 / d);
//                    k_near = PRX_MAXIMUM(std::ceil((log(nr_nodes)*2.7182818284 * (1 + val))), 1);
//                }
//            }
//        }
//    }
//}
//
