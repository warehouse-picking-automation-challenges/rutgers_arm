/**
 * @file coordination_astar.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */


#include "planning/motion_planners/coordination_astar.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::coordination_manipulation::coordination_astar_t, prx::plan::astar_module_t)



namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace coordination_manipulation
        {
            coordination_astar_t::coordination_astar_t()
            {
            }

            coordination_astar_t::~coordination_astar_t()
            {
            }

            bool coordination_astar_t::examine_vertex(undirected_vertex_index_t end_index)
            {
                return astar_module_t::examine_vertex(end_index);
            }

            bool coordination_astar_t::examine_edge(undirected_vertex_index_t start_index, undirected_edge_index_t edge, undirected_vertex_index_t end_index) //TODO: Figure out where this is called.  Fix this.
            {
                return astar_module_t::examine_edge(start_index, edge, end_index);
            }
            
            void coordination_astar_t::set_coordination_problem(double x_axis, double y_axis, coordination_heuristic_type_t bias)
            {
                PRX_PRINT ("Setting Coordination astar problem", PRX_TEXT_GREEN);
                x_axis_goal = x_axis;
                y_axis_goal = y_axis;
                bias_type = bias;
                append_actual_goal = false;
            }
            bool coordination_astar_t::check_if_goal_found(util::undirected_vertex_index_t potential_goal, util::undirected_vertex_index_t actual_goal)
            {
                
//                undirected_node_t *s, *t;
//                s = graph->get_vertex_as<undirected_node_t > (potential_goal);
//                t = graph->get_vertex_as<undirected_node_t > (actual_goal);
//                
//                space_point_t* start_point, *goal_point;
//                start_point = s->point;
//                goal_point = t->point;
//                
//                double start_x = start_point->at(0), start_y = start_point->at(1);
//                double goal_x = goal_point->at(0), goal_y = goal_point->at(1);
//                
//                PRX_PRINT("WE are calling our check goal!", PRX_TEXT_RED);
//                if (bias_type == BIAS_RIGHT)
//                {
//                    if (start_x == goal_x)
//                    {
//                        PRX_PRINT("RIGHT ARM FINISH. THIS IS GONNA BREAK THINGS!", PRX_TEXT_RED);
//                        append_actual_goal = true;
//                        return true;
//                    }
//                }
//                else if (bias_type == BIAS_LEFT)
//                {
//                    if (start_y == goal_y)
//                    {
//                        PRX_PRINT("LEFT ARM FINISH. THIS IS GONNA BREAK THINGS!", PRX_TEXT_BLUE);
//                        append_actual_goal = true;
//                        return true;
//                    }
//                }
//                else
                {
                    return astar_module_t::check_if_goal_found(potential_goal, actual_goal);
                }
            }

            bool coordination_astar_t::solve(undirected_vertex_index_t start, const std::vector<undirected_vertex_index_t>& goals)
            {
                PRX_PRINT ("MAH SOLVE BAYBEEE", PRX_TEXT_LIGHTGRAY);
                undirected_vertex_index_t u, v;
                undirected_edge_index_t e;
                double dist;
                std::vector<undirected_vertex_index_t>::const_iterator goalIterator;

                foreach(u, boost::vertices(graph->graph))
                {
                    initialize_vertex(u);
                    graph->distances[u] = PRX_INFINITY;
                    graph->colors[u] = WHITE;
                    graph->predecessors[u] = u;
                }

                open_set.clear();
                openset_insert_node(start, start, heuristic(start, goals));
                open_set.insert(new astar_node_t(start, heuristic(start, goals)));
                graph->colors[start] = GRAY;
                graph->distances[start] = 0;
                discover_vertex(start);

                while( !open_set.empty() )
                {
                    astar_node_t* u_node = open_set.remove_min();
                    u = u_node->vertex;
                    if( !examine_vertex(u) )
                    {
                        continue;
                    }

                    goalIterator = std::find(goals.begin(), goals.end(), u);
                    if( goalIterator != goals.end() )
                    {
                        foundGoal = *goalIterator;
                        return true;
                    }

                    foreach(v, boost::adjacent_vertices(u, graph->graph))
                    {
                        e = boost::edge(u, v, graph->graph).first;
                        
                        double penalization = 0.0;
                        
                        e = boost::edge(u, v, graph->graph).first;
                        
//                        undirected_node_t *s, *t;
//                        s = graph->get_vertex_as<undirected_node_t > (u);
//                        t = graph->get_vertex_as<undirected_node_t > (v);
//
//                        space_point_t* start_point, *goal_point;
//                        start_point = s->point;
//                        goal_point = t->point;
//
//                        double start_x = start_point->at(0), start_y = start_point->at(1);
//                        double goal_x = goal_point->at(0), goal_y = goal_point->at(1);
//
//                        double x_dist = goal_x - start_x;
//                        double y_dist = goal_y - start_y;
//                        
//                        if (bias_type == BIAS_RIGHT)
//                        {
//        //                    PRX_WARN_S ("BIAS RIGHT");
//                            penalization = y_dist;
//                        }
//                        else if (bias_type == BIAS_LEFT)
//                        {
//        //                    PRX_WARN_S ("BIAS LEFT");
//                            penalization = x_dist;
//                        }
                        
                        
                        dist = graph->weights[e] + graph->distances[u] + penalization;
                        
    //                    PRX_WARN_S ("dist: " << dist << "from weight: " << graph->weights[e] << " and distance: " << graph->distances[u]);
                        if( examine_edge(u, e, v) && dist < graph->distances[v] )
                        {
                            graph->distances[v] = dist;
                            graph->predecessors[v] = u;
                            relaxed_edge(e);
                            if( graph->colors[v] == WHITE )
                            {
                                graph->colors[v] = GRAY;
                                openset_insert_node(u, v, dist + heuristic(v, goals));
                                discover_vertex(v);
                            }
                        }
                        else
                        {
                            not_relaxed_edge(e);
                        }
                    }

                    graph->colors[u] = BLACK;
                    finish_vertex(u);
                    delete u_node;
                }

                return false;
            }
            
            double coordination_astar_t::heuristic(undirected_vertex_index_t current, undirected_vertex_index_t goal)
            {
                undirected_node_t *s, *t;
                s = graph->get_vertex_as<undirected_node_t > (current);
                t = graph->get_vertex_as<undirected_node_t > (goal);
                
//                space_point_t* start_point, *goal_point;
//                start_point = s->point;
//                goal_point = t->point;
//                
//                double start_x = start_point->at(0), start_y = start_point->at(1);
//                double goal_x = goal_point->at(0), goal_y = goal_point->at(1);
//                
//                if (goal_x < start_x)
//                {
//                    PRX_FATAL_S ("Goal x: " << goal_x << " less than start x: " << start_x);
//                }
//                
//                if (goal_y < start_y)
//                {
//                    PRX_FATAL_S ("Goal y: " << goal_y << " less than start y: " << start_y);
//                }
//                
//                double x_dist = goal_x - start_x;
//                double y_dist = goal_y - start_y;
//                
//                if (bias_type == BIAS_RIGHT)
//                {
////                    PRX_WARN_S ("BIAS RIGHT");
////                    graph->distances[current] += y_dist;
//                    return x_dist;
//                }
//                else if (bias_type == BIAS_LEFT)
//                {
////                    PRX_WARN_S ("BIAS LEFT");
////                    graph->distances[current] += x_dist;
//                    return y_dist;
//                }
//                else if (bias_type == JOINT_BIAS)
//                {
////                    PRX_WARN_S ("JOINT BIAS");
//                    return std::max(x_dist, y_dist);
//                }
//                else
//                {
////                    PRX_WARN_S ("ANY BIAS");
//                    return std::min(x_dist,y_dist);
//                }
                

//                
                return metric->distance_function(s->point, t->point);
            }

            double coordination_astar_t::heuristic(undirected_vertex_index_t current, const std::vector<undirected_vertex_index_t>& goals)
            {
                double minH, h;
                size_t i;

                minH = heuristic(current, goals[0]);

                for( i = 1; i < goals.size(); ++i )
                {
                    h = heuristic(current, goals[i]);
                    if( h < minH )
                        minH = h;
                }

                return minH;
            }
        }
    }
}
