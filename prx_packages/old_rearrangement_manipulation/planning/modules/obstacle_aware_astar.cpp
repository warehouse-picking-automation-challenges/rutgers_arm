/**
 * @file obstacle_aware_astar.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "planning/modules/obstacle_aware_astar.hpp"
#include "planning/modules/system_name_validity_checker.hpp"
#include "planning/graphs/manipulation_graph.hpp"

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/planning/motion_planners/motion_planner_edge.hpp"
#include "planning/graphs/pebble_graph.hpp"
#include <pluginlib/class_list_macros.h>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/subgraph.hpp>

PLUGINLIB_EXPORT_CLASS(prx::packages::rearrangement_manipulation::obstacle_aware_astar_t, prx::plan::astar_module_t)

namespace prx
{
    namespace packages
    {
        using namespace util;
        using namespace sim;
        using namespace plan;

        namespace rearrangement_manipulation
        {

            obstacle_aware_astar_t::obstacle_aware_astar_t()
            {
                max_length = PRX_INFINITY;
                collision_penalty = PRX_INFINITY;
                new_constraint_penalty = PRX_INFINITY;
                minimum_conflict = false;
                shortest_path = true;
                system_checker = NULL;

                valid_constraints = NULL;
                obstacle_constraints = NULL;
                avoid_constraints = NULL;
            }

            void obstacle_aware_astar_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
            {
                if( parameters::has_attribute("system_name_validity_checker", reader, template_reader) )
                {
                    system_checker = static_cast<system_name_validity_checker_t*>(parameters::initialize_from_loader<validity_checker_t > ("prx_planning", reader, "system_name_validity_checker", template_reader, "system_name_validity_checker"));
                    validity_checker = system_checker;
                }
                else
                    PRX_WARN_S("Missing system_name_validity_checker attribute for obstacle_aware_astar_t!");

                max_length = parameters::get_attribute_as<double>("max_length", reader, template_reader, PRX_INFINITY);
                collision_penalty = parameters::get_attribute_as<double>("collision_penalty", reader, template_reader, PRX_INFINITY);
                new_constraint_penalty = parameters::get_attribute_as<double>("new_constraint_penalty", reader, template_reader, PRX_INFINITY);
                minimum_conflict = parameters::get_attribute_as<bool>("minimum_conflict", reader, template_reader, false);
                shortest_path = parameters::get_attribute_as<bool>("shortest_path", reader, template_reader, true);
                exact = parameters::get_attribute_as<bool>("exact", reader, template_reader, true);
            }

            void obstacle_aware_astar_t::link_validity_checker(validity_checker_t* checker)
            {
                if( dynamic_cast<system_name_validity_checker_t*>(checker) == NULL )
                {
                    PRX_WARN_S("Obstacle_aware_A* can only use system_name_validity_checker");
                    return;
                }

                if( validity_checker != NULL )
                    PRX_WARN_S("Obstacle_aware_A* will use the given system_name_validity_checker!");

                validity_checker = checker;
                system_checker = static_cast<system_name_validity_checker_t*>(checker);
            }

            void obstacle_aware_astar_t::link_valid_constraints(const std::set<unsigned>* constraints)
            {
                valid_constraints = constraints;
            }

            void obstacle_aware_astar_t::link_constraints(const std::set<unsigned>* obstacle_constraints, const std::set<unsigned>* collision_constraints, const std::set<unsigned>* avoid_constraints)
            {
                this->obstacle_constraints = obstacle_constraints;
                valid_constraints = collision_constraints;
                this->avoid_constraints = avoid_constraints;
            }

            bool obstacle_aware_astar_t::in_the_same_component(const undirected_vertex_index_t v, const std::vector<undirected_vertex_index_t> vertices)
            {
                foreach(undirected_vertex_index_t u, vertices)
                {
                    if(graph->components[v] == graph->components[u])
                        return true;
                }
                return false;
            }

            bool obstacle_aware_astar_t::solve(undirected_vertex_index_t start, const std::vector<undirected_vertex_index_t>& goals)
            {
                double cost = 0;
                path_constraints.clear();
                foul_constraints.clear();
                final_path.clear();
                open_set.clear();

                foreach(undirected_vertex_index_t v, boost::vertices(graph->graph))
                {
                    graph->get_vertex_as<manipulation_node_t > (v)->init();
                }

                boost::connected_components(graph->graph,graph->components);
                bool same_connected_component = false;
                if(in_the_same_component(start,goals))
                {
                    rearrangement_astar_node_t* nd = new rearrangement_astar_node_t(start, 0, heuristic(start, goals));
                    nd->path.push_back(start);
                    open_set.insert(nd);
                    graph->get_vertex_as<manipulation_node_t > (start)->has_constraints = false;
                    graph->distances[start] = 0;
                    same_connected_component = true;
                }

                for( unsigned i = 0; i < extra_starts.size(); ++i )
                {
                    if(in_the_same_component(extra_starts[i],goals))
                    {
                        rearrangement_astar_node_t* nd = new rearrangement_astar_node_t(extra_starts[i], 0, heuristic(extra_starts[i], goals));
                        nd->path.push_back(extra_starts[i]);
                        open_set.insert(nd);
                        graph->get_vertex_as<manipulation_node_t > (extra_starts[i])->has_constraints = false;
                        graph->distances[extra_starts[i]] = 0;
                        same_connected_component = true;
                    }
                }

                sys_clock_t clock;
                clock.reset();

#ifndef NDEBUG
                double old_cost = -1;
                double old_f = -1;

                PRX_DEBUG_COLOR("=========================== A* =====================================", PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR("Start: " << state_space->print_point(graph->get_vertex_as<manipulation_node_t > (start)->point, 5), PRX_TEXT_CYAN);
                for( unsigned i = 0; i < extra_starts.size(); ++i )
                    PRX_DEBUG_COLOR("Start (" << i << "): " << state_space->print_point(graph->get_vertex_as<manipulation_node_t > (extra_starts[i])->point, 5), PRX_TEXT_CYAN);
                // for( unsigned i = 0; i < goals.size(); ++i )
                //     PRX_DEBUG_COLOR("Goal (" << i << "): " << state_space->print_point(graph->get_vertex_as<manipulation_node_t > (goals[i])->point, 5), PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("Goals :" << goals.size(), PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("minimum_conflict: " << minimum_conflict << "    short path: " << shortest_path << "    solve mode: " << solve_mode << "/" << PRX_REUSE_EDGE_INFO << "    max_len:" << max_length, PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("blocked edges: " << blocked_edges.size() << "   OpenSet:" << open_set.size(), PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("Vertices: " << boost::num_vertices(graph->graph) << "  Edges:" << boost::num_edges(graph->graph) << "   CC: " << boost::connected_components(graph->graph, graph->components), PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("Valid Constraints: " << print(*valid_constraints), PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("Same components: " << same_connected_component, PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("====================================================================", PRX_TEXT_GREEN);
                int iterations = 0;                
#endif
                if(!same_connected_component)
                {
                    PRX_DEBUG_COLOR("No solution!!! Start and goals are not in the same connected commponents", PRX_TEXT_RED);
                    return false;
                }

                while( !open_set.empty() )
                {
                    top_node = dynamic_cast<rearrangement_astar_node_t*>(open_set.remove_min());
#ifndef NDEBUG

                    //PRX_DEBUG_COLOR("Heap: " << graph->get_vertex_as<manipulation_node_t>(top_node->vertex)->node_id << ") " << top_node->print_few(), PRX_TEXT_CYAN);
                    iterations++;

                    //                    PRX_ASSERT(node->distance > old_dist);                    
                    PRX_ASSERT(top_node->f >= old_f || top_node->f - old_f <= PRX_ZERO_CHECK);
                    old_f = top_node->f;
                    old_cost = top_node->cost;
#endif
                    if( std::find(goals.begin(), goals.end(), top_node->vertex) != goals.end() )
                    {
                        foundGoal = top_node->vertex;
                        final_path = top_node->path;
                        path_constraints = top_node->constraints;
                        foul_constraints = top_node->foul_constraints;
                        final_path_cost = top_node->cost;

                        PRX_DEBUG_COLOR("================================================================", PRX_TEXT_BLUE);
                        PRX_DEBUG_COLOR("======================     Final Path     ======================", PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("================================================================", PRX_TEXT_BLUE);
                        PRX_DEBUG_COLOR("Size       : " << final_path.size(), PRX_TEXT_GREEN);
                        PRX_DEBUG_COLOR("Path       : " << print(graph, final_path), PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("constraints: |" << path_constraints.size() << "|   : " << top_node->print_constraints(), PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("foul constraints: |" << foul_constraints.size() << "|   : " << print(foul_constraints), PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("Cost       : " << final_path_cost << " / " << get_path_cost() << " / f:" << top_node->f, PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("Time       : " << clock.measure(), PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("Iterations : " << iterations, PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("minimum_conflict: " << minimum_conflict << "    short path: " << shortest_path << "    solve mode: " << solve_mode << "/" << PRX_REUSE_EDGE_INFO << "    max_len:" << max_length, PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("Start: " << state_space->print_point(graph->get_vertex_as<manipulation_node_t > (start)->point, 5), PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("Goal : " << state_space->print_point(graph->get_vertex_as<manipulation_node_t > (foundGoal)->point, 5), PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("================================================================", PRX_TEXT_BLUE);
                        return true;
                    }

                    //                    PRX_DEBUG_COLOR("node: " << node->vertex << "    f:" << node->f << "    dist:" << node->distance << "    neighs:" << boost::degree(node->vertex, graph->graph), PRX_TEXT_GREEN);

                    //                    PRX_DEBUG_COLOR("For new node : " << graph->get_vertex_as<undirected_node_t>(node->vertex)->node_id << ")  f: " << node->f << "     dist:" << node->distance  << "    heap_size: " << open_set.size(), PRX_TEXT_CYAN);
                    new_has_constraints = (top_node->constraints.size() > 0);
                    new_dist = top_node->cost;
                    new_constraints = top_node->constraints;
                    if( examine_vertex(top_node->vertex) )
                    {
                        foreach(undirected_edge_index_t e, boost::out_edges(top_node->vertex, graph->graph))
                        {
                            undirected_vertex_index_t v = boost::target(e, graph->graph);
                            PRX_ASSERT(v != top_node->vertex);
                            //                            PRX_DEBUG_COLOR("Going to examine one neighbor : " << v, PRX_TEXT_CYAN);
                            if( !top_node->path_has_vertex(v) )
                            {
                                //                                undirected_edge_index_t e = boost::edge(node->vertex, v, graph->graph).first;
                                new_dist = top_node->cost + graph->weights[e];
                                //                                                                PRX_DEBUG_COLOR("v is fine: dist:" << dist << "    max_len:" << max_length, PRX_TEXT_GREEN);
                                if( new_dist < max_length )
                                {
                                    // PRX_DEBUG_COLOR("From node to node : " << graph->get_vertex_as<manipulation_node_t>(top_node->vertex)->node_id << " -> " << graph->get_vertex_as<manipulation_node_t>(v)->node_id, PRX_TEXT_BROWN);
                                    if( examine_edge(top_node->vertex, e, v) )
                                    {
                                        //PRX_ASSERT(cost == new_dist);
                                        rearrangement_astar_node_t * new_node = new rearrangement_astar_node_t(v);
                                        new_node->merge(top_node);
                                        new_node->add_constraints(new_constraints);
                                        new_node->add_fouls(foul_constraints);
                                        //                                        double penalty = new_node->no_constraints() * collision_penalty + new_node->no_fouls() * new_constraint_penalty;
                                        new_node->set_f(new_dist, heuristic(v, goals));
                                        open_set.insert(new_node);
                                        //                                        PRX_DEBUG_COLOR("Edge with (" << graph->get_vertex_as<manipulation_node_t>(v)->node_id << ") : f:" << new_node->f << "   d:" << new_dist << "   C:" << print(new_node->constraints),PRX_TEXT_GREEN);
                                        //                                        PRX_DEBUG_COLOR(graph->get_vertex_as<manipulation_node_t>(node->vertex)->node_id << " -> " << graph->get_vertex_as<manipulation_node_t>(v)->node_id << ")  path constraints: " << print(new_constraints), PRX_TEXT_BLUE);
                                        //                                        PRX_DEBUG_COLOR("node: " << new_node->print(), PRX_TEXT_BROWN);

                                    }
                                }
                                //                                else
                                //                                {
                                //                                    PRX_DEBUG_COLOR("\n\n\nWOOOOOOOW dist:" << dist << "    max_len:" << max_length << "\n\n\n", PRX_TEXT_LIGHTGRAY);
                                //                                }
                            }
                        }
                    }
                    delete top_node;
                }
                PRX_DEBUG_COLOR("No solution after: " << iterations << "  iterations!  The open set has: " << open_set.size() << " points    Time: " << clock.measure(), PRX_TEXT_RED);
                return false;
            }

            bool obstacle_aware_astar_t::solve(undirected_vertex_index_t start, undirected_vertex_index_t goal)
            {
                PRX_FATAL_S("Stop");
                double dist = 0;
                path_constraints.clear();
                foul_constraints.clear();
                final_path.clear();
                open_set.clear();

                foreach(undirected_vertex_index_t v, boost::vertices(graph->graph))
                {
                    graph->get_vertex_as<manipulation_node_t > (v)->init();
                }

                rearrangement_astar_node_t* nd = new rearrangement_astar_node_t(start, 0, heuristic(start, goal));
                nd->path.push_back(start);
                open_set.insert(nd);
                graph->get_vertex_as<manipulation_node_t > (start)->has_constraints = false;
                graph->distances[start] = 0;

                for( unsigned i = 0; i < extra_starts.size(); ++i )
                {
                    rearrangement_astar_node_t* nd = new rearrangement_astar_node_t(extra_starts[i], 0, heuristic(extra_starts[i], goal));
                    nd->path.push_back(extra_starts[i]);
                    open_set.insert(nd);
                    graph->get_vertex_as<manipulation_node_t > (extra_starts[i])->has_constraints = false;
                    graph->distances[extra_starts[i]] = 0;
                }

                sys_clock_t clock;
                clock.reset();

#ifndef NDEBUG
                double old_dist = -1;
                double old_f = -1;
                PRX_DEBUG_COLOR("=========================== A* =====================================", PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR("Start: " << state_space->print_point(graph->get_vertex_as<manipulation_node_t > (start)->point, 5), PRX_TEXT_CYAN);
                for( unsigned i = 0; i < extra_starts.size(); ++i )
                    PRX_DEBUG_COLOR("Start (" << i << "): " << state_space->print_point(graph->get_vertex_as<manipulation_node_t > (extra_starts[i])->point, 5), PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("Goal : " << state_space->print_point(graph->get_vertex_as<manipulation_node_t > (goal)->point, 5), PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("minimum_conflict: " << minimum_conflict << "    short path: " << shortest_path << "    solve mode: " << solve_mode << "/" << PRX_REUSE_EDGE_INFO << "    max_len:" << max_length, PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("blocked edges: " << blocked_edges.size() << "   OpenSet:" << open_set.size(), PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("Vertices: " << boost::num_vertices(graph->graph) << "  Edges:" << boost::num_edges(graph->graph) << "   CC: " << boost::connected_components(graph->graph, graph->components), PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("Valid Constraints: " << print(*valid_constraints), PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("====================================================================", PRX_TEXT_GREEN);
                int iterations = 0;
#endif
                while( !open_set.empty() )
                {
#ifndef NDEBUG
                    iterations++;
                    PRX_ASSERT(top_node->f >= old_f || top_node->f - old_f <= PRX_ZERO_CHECK);
                    old_f = top_node->f;
                    old_dist = top_node->cost;
#endif

                    top_node = dynamic_cast<rearrangement_astar_node_t*>(open_set.remove_min());
                    //                    PRX_ASSERT(node->distance > old_dist);
                    if( top_node->vertex == goal )
                    {
                        foundGoal = goal;
                        found_start = top_node->path[0];
                        final_path = top_node->path;
                        path_constraints = top_node->constraints;
                        foul_constraints = top_node->foul_constraints;
                        final_path_cost = top_node->cost;

                        PRX_DEBUG_COLOR("================================================================", PRX_TEXT_BLUE);
                        PRX_DEBUG_COLOR("======================     Final Path     ======================", PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("================================================================", PRX_TEXT_BLUE);
                        PRX_DEBUG_COLOR("Size       : " << final_path.size(), PRX_TEXT_GREEN);
                        PRX_DEBUG_COLOR("constraints: |" << path_constraints.size() << "|   : " << top_node->print_constraints(), PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("foul constraints: |" << foul_constraints.size() << "|   : " << print(foul_constraints), PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("Cost       : " << final_path_cost << " / " << get_path_cost() << "  path_length: " << final_path.size(), PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("Time       : " << clock.measure(), PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("Iterations : " << iterations, PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("minimum_conflict: " << minimum_conflict << "    short path: " << shortest_path << "    solve mode: " << solve_mode << "/" << PRX_REUSE_EDGE_INFO << "    max_len:" << max_length, PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("Start: " << state_space->print_point(graph->get_vertex_as<manipulation_node_t > (start)->point, 5), PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("Goal : " << state_space->print_point(graph->get_vertex_as<manipulation_node_t > (foundGoal)->point, 5), PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("================================================================", PRX_TEXT_BLUE);
                        return true;
                    }

                    //                    PRX_DEBUG_COLOR("node: " << node->vertex << "    f:" << node->f << "    dist:" << node->distance << "    neighs:" << boost::degree(node->vertex, graph->graph), PRX_TEXT_GREEN);

                    //                    PRX_DEBUG_COLOR("For new node : " << graph->get_vertex_as<undirected_node_t>(node->vertex)->node_id << ")  f: " << node->f << "     dist:" << node->distance  << "    heap_size: " << open_set.size(), PRX_TEXT_CYAN);
                    new_has_constraints = top_node->constraints.size() > 0;
                    new_dist = top_node->cost;
                    new_constraints = top_node->constraints;
                    if( expand_vertex(top_node->vertex) )
                    {

                        foreach(undirected_edge_index_t e, boost::out_edges(top_node->vertex, graph->graph))
                        {
                            undirected_vertex_index_t v = boost::target(e, graph->graph);
                            PRX_ASSERT(v != top_node->vertex);
                            //                            PRX_DEBUG_COLOR("Going to examine one neighbor : " << v, PRX_TEXT_CYAN);
                            if( !top_node->path_has_vertex(v) )
                            {
                                //                                undirected_edge_index_t e = boost::edge(node->vertex, v, graph->graph).first;
                                dist = graph->weights[e] + top_node->cost;
                                //                                PRX_DEBUG_COLOR("v is fine: dist:" << dist << "    max_len:" << max_length, PRX_TEXT_GREEN);
                                if( dist < max_length )
                                {
                                    if( examine_edge(top_node->vertex, e, v) )
                                    {
                                        PRX_ASSERT(dist == new_dist);
                                        rearrangement_astar_node_t * new_node = new rearrangement_astar_node_t(v);
                                        new_node->merge(top_node);
                                        new_node->add_constraints(new_constraints);
                                        new_node->add_fouls(foul_constraints);
                                        //                                        double penalty = new_node->no_constraints() * collision_penalty + new_node->no_fouls() * new_constraint_penalty;
                                        new_node->set_f(dist, heuristic(v, goal));

                                        //                                        if( new_node->constraints.size() == 0 )
                                        //                                            graph->get_vertex_as<manipulation_node_t > (v)->shortest_distance = dist;

                                        open_set.insert(new_node);
                                    }
                                }
                                else
                                {
                                    PRX_DEBUG_COLOR("\n\n\nWOOOOOOOW dist:" << dist << "    max_len:" << max_length << "\n\n\n", PRX_TEXT_LIGHTGRAY);
                                }
                            }
                        }
                    }
                    delete top_node;
                }
                PRX_DEBUG_COLOR("No solution after: " << iterations << "  iterations!", PRX_TEXT_RED);
                return false;
            }

            void obstacle_aware_astar_t::extract_path(undirected_vertex_index_t start, undirected_vertex_index_t goal, std::deque<undirected_vertex_index_t>& vertices)
            {
                vertices = final_path;
            }

            void obstacle_aware_astar_t::extract_path_constraints(std::set<unsigned>& constraints)
            {
                for( unsigned i = 0; i < final_path.size() - 1; ++i )
                {
                    undirected_edge_index_t e = boost::edge(final_path[i], final_path[i + 1], graph->graph).first;
                    manipulation_edge_t* edge = graph->get_edge_as<manipulation_edge_t > (e);
                    edge->get_valid_constraints(constraints, valid_constraints);
                }
            }

            bool obstacle_aware_astar_t::is_valid_path()
            {
                bool valid = true;
                for( size_t i = 0; i < final_path.size() - 1; ++i )
                {
                    undirected_edge_index_t e = boost::edge(final_path[i], final_path[i + 1], graph->graph).first;
                    manipulation_edge_t* edge = graph->get_edge_as<manipulation_edge_t > (e);
                    if( !edge->is_valid(valid_constraints) )
                    {
                        block_edge(e);
                        valid = false;
                    }

                }
                return valid;
            }

            void obstacle_aware_astar_t::get_path_constraints(std::set<unsigned>& constraints)
            {
                constraints.insert(path_constraints.begin(), path_constraints.end());
            }

            void obstacle_aware_astar_t::get_foul_constraints(std::set<unsigned>& constraints)
            {
                constraints.insert(foul_constraints.begin(), foul_constraints.end());
            }

            state_t* obstacle_aware_astar_t::get_start_point()
            {
                return graph->get_vertex_as<manipulation_node_t > (found_start)->point;
            }

            state_t* obstacle_aware_astar_t::get_goal_point()
            {
                return graph->get_vertex_as<manipulation_node_t > (foundGoal)->point;
            }

            bool obstacle_aware_astar_t::expand_vertex(util::undirected_vertex_index_t vertex)
            {
                manipulation_node_t* v_node = graph->get_vertex_as<manipulation_node_t > (vertex);
                if( !v_node->updated )
                {
                    if( shortest_path && new_has_constraints )
                        return false;

                    v_node->shortest_distance = top_node->cost;
                    v_node->has_constraints = new_has_constraints;
                    if( new_has_constraints )
                    {
                        v_node->constraints_sets.push_back(std::make_pair(top_node->constraints, top_node->cost));
                    }
                    v_node->updated = true;
                    return true;
                }
                return examine_vertex(vertex);
            }

            bool obstacle_aware_astar_t::examine_vertex(undirected_vertex_index_t vertex)
            {
                manipulation_node_t* v_node = graph->get_vertex_as<manipulation_node_t > (vertex);
                v_node->visited++;
                if( minimum_conflict )
                {

                    if( !v_node->updated )
                    {
                        v_node->shortest_distance = new_dist;
                        v_node->has_constraints = new_has_constraints;
                        if( new_has_constraints )
                        {
                            v_node->constraints_sets.push_back(std::make_pair(new_constraints, new_dist));
                        }
                        v_node->updated = true;
                        // PRX_DEBUG_COLOR("==========================================================================", PRX_TEXT_LIGHTGRAY);
                        // PRX_DEBUG_COLOR("Q:" << open_set.size() << ")" << v_node->print() , PRX_TEXT_BLUE);
                        // PRX_DEBUG_COLOR("==========================================================================", PRX_TEXT_LIGHTGRAY);
                        return true;
                    }

                    // PRX_DEBUG_COLOR("==========================================================================", PRX_TEXT_BLUE);
                    // PRX_DEBUG_COLOR(v_node->print() , PRX_TEXT_LIGHTGRAY);
                    // PRX_DEBUG_COLOR("NEW node :  has constraints: " << new_has_constraints << "  dist:" << new_dist << "  C:" << print(new_constraints), PRX_TEXT_CYAN);
                    // PRX_DEBUG_COLOR("==========================================================================", PRX_TEXT_BLUE);

                    if( !v_node->has_constraints )
                    {
                        // PRX_DEBUG_COLOR("curr node does not have constraints ?? : " << v_node->has_constraints, PRX_TEXT_GREEN);
                        //if the node does not have constraints and the new node has constraints then we don't add the node.                    
                        if( new_has_constraints )
                            return false;

                        //Here both new_constraints and the node's constraints are 0
                        if( fabs(v_node->shortest_distance - new_dist) > PRX_ZERO_CHECK && v_node->shortest_distance < new_dist )
                        {
                            if(fabs(v_node->shortest_distance - new_dist) < PRX_ZERO_CHECK)
                                PRX_DEBUG_COLOR("CURR DIST : " << v_node->shortest_distance << "    new_dist:" << new_dist, PRX_TEXT_GREEN);
                            return false;
                        }
                        // PRX_DEBUG_COLOR("CURR DIST : " << v_node->shortest_distance << " >   new_dist:" << new_dist, PRX_TEXT_CYAN);
                        // PRX_ASSERT(fabs(v_node->shortest_distance - new_dist) > PRX_ZERO_CHECK);

                        // v_node->shortest_distance = new_dist;

                    }
                    else if( !new_has_constraints )
                    {
                        // PRX_DEBUG_COLOR("new node does not have constraints ?? : " << new_has_constraints, PRX_TEXT_GREEN);
                        PRX_ASSERT(v_node->has_constraints);
                        //In this case the node has constraints and the new node does not have.
                        v_node->has_constraints = new_has_constraints;
                        PRX_ASSERT(!v_node->has_constraints);
                        v_node->constraints_sets.clear();
                        // v_node->shortest_distance = new_dist;
                    }
                    else
                    {
                        PRX_ASSERT(new_has_constraints && v_node->has_constraints);
                        //PRX_ASSERT(v_node->has_constraints > 0 && v_node->constraints_sets.size() > 0);
                        if( exact )
                        {
                            if(!v_node->is_set_added(new_constraints, new_dist))
                                return false;
                            v_node->has_constraints = true;
                        }
                        else
                        {
                            //Greedy Version and both the node and the new node have constraints.
                            if( new_constraints.size() > v_node->constraints_sets[0].first.size() )
                                return false;
                            else if( new_constraints.size() == v_node->constraints_sets[0].first.size() && new_dist > v_node->shortest_distance )
                                return false;

                            // v_node->shortest_distance = new_dist;
                            v_node->has_constraints = new_has_constraints;
                            v_node->constraints_sets[0].first = new_constraints;
                            v_node->constraints_sets[0].second = new_dist;

                            // return true;
                        }
                    }
                }
                else
                {
                    //Short distance function.
                    PRX_ASSERT(!v_node->has_constraints);
                    //                    PRX_ASSERT(!new_has_constraints);

                    if( new_has_constraints || v_node->shortest_distance < new_dist )
                    {
                        //If constraints != 0 then we are in shortest path with Edge information off. 
                        //PRX_ASSERT(v_node->has_constraints == 0);
                        return false;
                    }
                    // v_node->shortest_distance = new_dist;

                }
                v_node->shortest_distance = new_dist;
                return true;
            }

            bool obstacle_aware_astar_t::examine_edge(undirected_vertex_index_t start_index, undirected_edge_index_t e, undirected_vertex_index_t end_index)
            {
                path_constraints.clear();
                foul_constraints.clear();

                //check if the edge is in the blocked edges.
                if( astar_module_t::examine_edge(start_index, e, end_index) )
                {
                    //We have to clean these variables because multiple edges will be checked for each top_node
                    new_has_constraints = false; 
                    new_constraints.clear();
                    if( solve_mode == PRX_REUSE_EDGE_INFO )
                    {
                        manipulation_edge_t* edge = graph->get_edge_as<manipulation_edge_t > (e);

                        if( obstacle_constraints != NULL && !edge->is_valid(obstacle_constraints) )
                        {
                            return false;
                        }

                        new_constraints = top_node->constraints;
                        if( !edge->is_valid(valid_constraints) )
                        {
                            if( !minimum_conflict )
                                return false;

                            edge->get_valid_constraints(path_constraints, valid_constraints);
                            new_constraints.insert(path_constraints.begin(), path_constraints.end());
                            // PRX_DEBUG_COLOR(edge->source_vertex << " -> " << edge->target_vertex <<  ") Has constraints: " << new_has_constraints << "    C:" << print(new_constraints)<< "    PC:" << print(path_constraints) << "  TN:" << print(top_node->constraints), PRX_TEXT_CYAN);
                        }
                        new_has_constraints = (new_constraints.size() > 0);
                        if( avoid_constraints != NULL )
                            edge->get_valid_constraints(foul_constraints, avoid_constraints);


                    }
                    else if( solve_mode == PRX_REPROPAGATE_EDGE_INFO )
                        PRX_FATAL_S("The Repropagate edge is not implemented for the obstacle aware A*!");
                    //if the mode is PRX_NO_EDGE_INFO we only need to check the new vertex.
                    return examine_vertex(end_index);

                }
                return false;
            }

            double obstacle_aware_astar_t::heuristic(util::undirected_vertex_index_t current, util::undirected_vertex_index_t goal)
            {

                return metric->distance_function(graph->operator[](current)->point, graph->operator[](goal)->point);
            }

            double obstacle_aware_astar_t::heuristic(util::undirected_vertex_index_t current, const std::vector< util::undirected_vertex_index_t >& goals)
            {
                double min_dist = metric->distance_function(graph->operator[](current)->point, graph->operator[](goals[0])->point);

                for( unsigned i = 1; i < goals.size(); ++i )
                {
                    double dist = metric->distance_function(graph->operator[](current)->point, graph->operator[](goals[i])->point);
                    if( dist < min_dist )
                        min_dist = dist;
                }
                return min_dist;
            }

            void obstacle_aware_astar_t::set_minimum_conflict(bool flag)
            {

                minimum_conflict = flag;
            }

            void obstacle_aware_astar_t::set_shortest_path_flag(bool flag)
            {

                shortest_path = flag;
            }

            void obstacle_aware_astar_t::set_exact_flag(bool flag)
            {
                exact = flag;
            }

            void obstacle_aware_astar_t::set_max_length(double length)
            {
                max_length = length;

                // if( minimum_conflict )
                //     collision_penalty = 2 * max_length;
            }

            void obstacle_aware_astar_t::set_collision_penalty(double penalty)
            {

                collision_penalty = penalty;
            }

            double obstacle_aware_astar_t::get_path_cost() const
            {
                return final_path_cost;
            }

            std::string obstacle_aware_astar_t::print(const std::set<unsigned>& constraints)
            {
                std::stringstream output(std::stringstream::out);

                foreach(unsigned i, constraints)
                {
                    output << i << " , ";
                }
                return output.str();
            }

            std::string obstacle_aware_astar_t::print(const undirected_graph_t *graph, const std::deque<undirected_vertex_index_t>& vertices)
            {
                std::stringstream output(std::stringstream::out);

                foreach(undirected_vertex_index_t v, vertices)
                {
                    output << graph->get_vertex_as<undirected_node_t > (v)->node_id << " , ";
                }
                return output.str();
            }


        }
    }
}
