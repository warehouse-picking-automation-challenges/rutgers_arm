/**
 * @file coordination_constraint_astar.hpp 
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

#include "planning/motion_planners/coordination_graph/coordination_constraint_astar.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"


namespace prx
{
    namespace packages
    {
        namespace coordination_manipulation
        {
            coordination_constraint_astar_t::coordination_constraint_astar_t()
            {
                linked_constraints = NULL;
            }

            coordination_constraint_astar_t::~coordination_constraint_astar_t()
            {
                if (nodes.size() > 0)
                {
                    
                    for(unsigned right_step = 0; right_step < max_right_size; right_step++)
                    {
                        for (unsigned left_step = 0; left_step < max_left_size; left_step++)
                        {
                            delete nodes[right_step][left_step];
                        }
                    }
                }
            }

            void coordination_constraint_astar_t::link_constraints(coordination_constraints_t* constraints)
            {
                if (linked_constraints != NULL)
                {
                    PRX_FATAL_S ("Already linked constraints!");
                }
                else
                {
                    linked_constraints = constraints;
                    max_right_size = linked_constraints->get_max_right_size();
                    max_left_size = linked_constraints->get_max_left_size();
                    nodes.resize(max_right_size);
                    colors.resize(max_right_size);
                    
                    for(unsigned right_step = 0; right_step < max_right_size; right_step++)
                    {
                        nodes[right_step].resize(max_left_size);
                        colors[right_step].resize(max_left_size);
                        for (unsigned left_step = 0; left_step < max_left_size; left_step++)
                        {
                            nodes[right_step][left_step] = new constraint_node_t(right_step, left_step);
                        }
                    }
                }
            }
            
            void coordination_constraint_astar_t::set_start_and_goal(unsigned right_plan_start, unsigned left_plan_start, unsigned right_plan_goal, unsigned left_plan_goal)
            {
                PRX_PRINT ("RIGHT START: " << right_plan_start << " RIGHT GOAL: " << right_plan_goal, PRX_TEXT_RED);
                PRX_PRINT ("LEFT START: " << left_plan_start << " LEFT GOAL: " << left_plan_goal, PRX_TEXT_BLUE);
                right_start = right_plan_start;
                left_start = left_plan_start;
                right_goal = right_plan_goal;
                left_goal = left_plan_goal;
                
                goal_node = NULL;
                start_node = nodes[right_start][left_start];
                while( !astar_heap.empty() )
                {
                    astar_heap.pop();
                }
//                astar_heap.clear();
            }
            
            void coordination_constraint_astar_t::set_coordination_problem(unsigned rob_index, unsigned lob_index, std::string right_object_type, std::string left_object_type, coordination_heuristic_type_t bias)
            {    
                PRX_PRINT ("Set coordination problem for rob: " << rob_index << " and lob: " << lob_index, PRX_TEXT_LIGHTGRAY);
                right_object = rob_index;
                left_object = lob_index;
                right_type = right_object_type;
                left_type = left_object_type;
                
                bias_type = bias; 

            }
            bool coordination_constraint_astar_t::check_if_goal_found(const constraint_node_t* current_node)
            {
                unsigned current_right_step = current_node->right_plan_step;
                unsigned current_left_step = current_node->left_plan_step;
                
                return (current_right_step == right_goal || current_left_step == left_goal);
                
//                if (bias_type == ANY_BIAS)
//                {
//                    return (current_right_step == right_goal || current_left_step == left_goal);
//                }
//                else if (bias_type == BIAS_RIGHT)
//                {
//                    append_actual_goal = true;
//                    return (current_right_step == right_goal);
//                }
//                else if (bias_type == BIAS_LEFT)
//                {
//                    append_actual_goal = true;
//                    return (current_left_step == left_goal);
//                }
//                else if (bias_type == JOINT_BIAS)
//                {
//                    return (current_right_step == right_goal && current_left_step == left_goal);
//                }


//                if (bias_type == BIAS_RIGHT || bias_type == ANY_BIAS)
//                {
//                    if (current_right_step == right_goal)
//                    {
//                        PRX_PRINT("RIGHT ARM FINISH. THIS IS GONNA BREAK THINGS!", PRX_TEXT_RED);
//                        
//                        return true;
//                    }
//                    
//                    return false;
//                }
//                if (bias_type == BIAS_LEFT || bias_type == ANY_BIAS)
//                {
//                    if (current_left_step == left_goal)
//                    {
//                        PRX_PRINT("LEFT ARM FINISH. THIS IS GONNA BREAK THINGS!", PRX_TEXT_BLUE);
//                        append_actual_goal = true;
//                        return true;
//                    }
//                    
//                    return false;
//                }
//                
//                return (current_right_step == right_goal && current_left_step == left_goal); 
                
            }
            
            void coordination_constraint_astar_t::get_valid_neighbors(const constraint_node_t* current_node, std::vector<constraint_node_t*>& neighbor_nodes)
            {
                unsigned right_step = current_node->right_plan_step;
                unsigned left_step = current_node->left_plan_step;
                int south_right, south_left;
                int north_right, north_left;
                int west_right, west_left;
                int east_right, east_left;
                
                int NW_right, NW_left;
                int NE_right, NE_left;
                int SW_right, SW_left;
                int SE_right, SE_left;
  
                // Check if we can add southern neighbors
                if (left_step > left_start)
                {
                    // Add south
                    south_right = right_step;
                    south_left = left_step-1;
                    if (linked_constraints->is_valid_state(south_right, south_left, right_object, left_object, right_type, left_type))
                    {
                        nodes[south_right][south_left]->edge_cost = 2.0;
                        
//                        nodes[south_right][south_left]->edge_cost = 6.0;
                        
//                        if (bias_type == JOINT_BIAS)
//                                nodes[south_right][south_left]->edge_cost = 1.0;
//                        else
//                                nodes[south_right][south_left]->edge_cost = 2.0;
                        neighbor_nodes.push_back(nodes[south_right][south_left]);
                    }

                    // Check if we can add southeast
                    if (right_step < right_goal)
                    {
                        SE_right = (right_step+1);
                        SE_left = (left_step-1);
                        if (linked_constraints->is_valid_state(SE_right, SE_left, right_object, left_object, right_type, left_type))
                        {
                            nodes[SE_right][SE_left]->edge_cost = 2;
//                            nodes[SE_right][SE_left]->edge_cost = 4.0;
//                            if (bias_type == JOINT_BIAS)
//                                nodes[SE_right][SE_left]->edge_cost = 2.0;
//                            else
//                                nodes[SE_right][SE_left]->edge_cost = 1.5;
                            neighbor_nodes.push_back(nodes[SE_right][SE_left]);
                        }
                    }
                    // Check if we can add southwest
                    if (right_step > right_start)
                    {
                        SW_right = (right_step-1);
                        SW_left = (left_step-1);
                        if (linked_constraints->is_valid_state(SW_right,SW_left, right_object, left_object, right_type, left_type))
                        {
                            nodes[SW_right][SW_left]->edge_cost = 2;
//                            nodes[SW_right][SW_left]->edge_cost = 8.0;
//                            if (bias_type == JOINT_BIAS)
//                                nodes[SW_right][SW_left]->edge_cost = 2.0;
//                            else
//                                nodes[SW_right][SW_left]->edge_cost = 3;
                            neighbor_nodes.push_back(nodes[SW_right][SW_left]);
                        }
                    }

                }

                //Check if we can add northern neighbors
                if (left_step < left_goal)
                {
                    north_right = (right_step);
                    north_left =  (left_step+1);
                    if (linked_constraints->is_valid_state(north_right, north_left, right_object, left_object, right_type, left_type))
                    {
                        nodes[north_right][north_left]->edge_cost = 1.001;
//                        nodes[north_right][north_left]->edge_cost = 2.0;
                        neighbor_nodes.push_back(nodes[north_right][north_left]);
                    }
                    // Check if we can add northeast
                    if (right_step < right_goal)
                    {
                        NE_right = (right_step+1);
                        NE_left = (left_step+1);
                        if (linked_constraints->is_valid_state(NE_right, NE_left, right_object, left_object, right_type, left_type))
                        {
                            nodes[NE_right][NE_left]->edge_cost = 1.0;
//                            nodes[NE_right][NE_left]->edge_cost = 1.0;
//                            if (bias_type == JOINT_BIAS)
//                                nodes[NE_right][NE_left]->edge_cost = 2.0;
//                            else
//                                nodes[NE_right][NE_left]->edge_cost = 0.5;
                            neighbor_nodes.push_back(nodes[NE_right][NE_left]);
                        }

                    }
                    // Check if we can add northwest
                    if (right_step > right_start)
                    {
                        NW_right = (right_step-1);
                        NW_left =  (left_step+1);
                        if (linked_constraints->is_valid_state(NW_right, NW_left, right_object, left_object, right_type, left_type))
                        {
                            nodes[NW_right][NW_left]->edge_cost = 2;
//                            nodes[NW_right][NW_left]->edge_cost = 4.0;
//                            if (bias_type == JOINT_BIAS)
//                                nodes[NW_right][NW_left]->edge_cost = 2.0;
//                            else
//                                nodes[NW_right][NW_left]->edge_cost = 1.5;
                            neighbor_nodes.push_back(nodes[NW_right][NW_left]);
                        }

                    }

                }

                // Check if we can add east
                if (right_step < right_goal)
                {
                    // Add east
                    east_right = (right_step+1);
                    east_left =  (left_step);
                    if (linked_constraints->is_valid_state(east_right, east_left, right_object, left_object, right_type, left_type))
                    {
                        nodes[east_right][east_left]->edge_cost = 1.001;
//                        nodes[east_right][east_left]->edge_cost = 2.0;
                        neighbor_nodes.push_back(nodes[east_right][east_left]);
                    }

                }
                // Check if we can add west
                if (right_step > right_start)
                {
                    // Add west
                    west_right = (right_step-1);
                    west_left =  (left_step);
                    if (linked_constraints->is_valid_state(west_right, west_left, right_object, left_object, right_type, left_type))
                    {
                        nodes[west_right][west_left]->edge_cost = 2.0;
//                        nodes[west_right][west_left]->edge_cost = 4.0;
//                        if (bias_type == JOINT_BIAS)
//                            nodes[west_right][west_left]->edge_cost = 1.0;
//                        else
//                            nodes[west_right][west_left]->edge_cost = 2.0;
                        neighbor_nodes.push_back(nodes[west_right][west_left]);
                    }

                }
            }


            bool coordination_constraint_astar_t::solve()
            {
                PRX_PRINT ("MAH SOLVE BAYBEEE", PRX_TEXT_LIGHTGRAY);
                double dist;

                for(unsigned i = 0; i < nodes.size(); i++)
                {
                    for(unsigned j = 0; j < nodes[i].size(); j++)
                    {
                        nodes[i][j]->initialize();
                        colors[i][j] = ASTAR_COLOR_WHITE;
                        nodes[i][j]->color = &colors[i][j];
                    }
                }
                start_node->g_cost = 0.0;
                start_node->f_cost = heuristic(start_node);
                *(start_node->color) = ASTAR_COLOR_GRAY;
                
                constraint_node_pointer_t node; 
                node.ptr = start_node;
                astar_heap.push(node);
                while( !astar_heap.empty() )
                {
                    node = astar_heap.top();
                    constraint_node_t* u_node = node.ptr;
//                    PRX_WARN_S ("Popped top node at: " << u_node->right_plan_step << "," << u_node->left_plan_step);
                    astar_heap.pop();
//                    if( !examine_vertex(u) )
//                    {
//                        continue;
//                    }

                    if( check_if_goal_found(u_node) )
                    {
                        goal_node = u_node;
                        return true;
                    }
                    
                    
                    
                    std::vector<constraint_node_t*> neighbors;
                    get_valid_neighbors(u_node, neighbors);
                    
                    if (neighbors.size() == 0)
                    {
                        PRX_ERROR_S ("This node has 0 neighbors! R: " << u_node->right_plan_step << ", L:" << u_node->left_plan_step);
                        PRX_ASSERT(false);
                    }

                    foreach(constraint_node_t* n, neighbors)
                    {
//                        PRX_WARN_S ("NEIGHBOR AT: " << n->right_plan_step << "," << n->left_plan_step);
                        
                        double penalization = 0.0;
                        
                        
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
                        
                        
                        dist = n->edge_cost + u_node->g_cost + penalization;
//                        PRX_DEBUG_COLOR("Dist: " << dist, PRX_TEXT_MAGENTA);
    //                    PRX_WARN_S ("dist: " << dist << "from weight: " << graph->weights[e] << " and distance: " << graph->distances[u]);
                        if( dist < n->g_cost )
                        {
//                            PRX_WARN_S ("Yeah cost : " << dist << " less than : " << n->g_cost);
                            n->g_cost = dist;
                            n->parent_node = u_node;
                            if (*(n->color) == ASTAR_COLOR_WHITE )
                            {
//                                PRX_ERROR_S("ADD DA NODE:  " << n->right_plan_step << "," << n->left_plan_step);
                                n->f_cost = dist + heuristic(n);
                                *(n->color) = ASTAR_COLOR_GRAY;
                                constraint_node_pointer_t new_node;
                                new_node.ptr = n;
                                astar_heap.push(new_node);
                            }

                        }
//                        PRX_ASSERT(false);
                    }
                    *(u_node->color) = ASTAR_COLOR_BLACK;

                }
                PRX_ERROR_S ("LAST NODE CHECKED: " << node.ptr->right_plan_step <<" , " << node.ptr->left_plan_step);
                return false;
            }
            
            double coordination_constraint_astar_t::heuristic(const constraint_node_t* current_node)
            {

                double right_dist = right_goal - current_node->right_plan_step;
                double left_dist = left_goal - current_node->left_plan_step;
                double return_val;
                if (right_dist < left_dist)
                    return_val = right_dist;
                else
                    return_val = left_dist;

                return return_val;

                // return (right_dist + left_dist);
//                return std::min(right_dist,left_dist);
                
//                double return_val;
//                if (bias_type == BIAS_RIGHT)
//                {
////                    PRX_WARN_S ("BIAS RIGHT");
////                    graph->distances[current] += y_dist;
//                    return_val = right_dist;
//                }
//                else if (bias_type == BIAS_LEFT)
//                {
////                    PRX_WARN_S ("BIAS LEFT");
////                    graph->distances[current] += x_dist;
//                    return_val =  left_dist;
//                }
//                else if (bias_type == JOINT_BIAS)
//                {
////                    PRX_WARN_S ("JOINT BIAS");
//                    return_val =  right_dist + left_dist;
////                    return_val =  sqrt(right_dist*right_dist + left_dist*left_dist);
//                }
//                else if (bias_type == ANY_BIAS)
//                {
////                    PRX_WARN_S ("ANY BIAS");
//                    return_val =  std::min(right_dist,left_dist);
//                }
//                else
//                {
//                    PRX_FATAL_S ("Unknown bias!");
//                }
//                PRX_DEBUG_COLOR("RETURN VAL FOR HEURISTIC: " << return_val, PRX_TEXT_MAGENTA);
//                
//                return return_val;
                
                
            }
            
            void coordination_constraint_astar_t::get_solution_path (std::deque<unsigned>& right_solution_indices, std::deque<unsigned>& left_solution_indices)
            {
                if (goal_node == NULL)
                {
                    PRX_FATAL_S ("Get solution path called before a valid goal was found");
                }
                right_solution_indices.clear();
                left_solution_indices.clear();
                constraint_node_t* current_node = goal_node;
                while ( current_node->parent_node != NULL)
                {
                    right_solution_indices.push_front(current_node->right_plan_step);
                    left_solution_indices.push_front(current_node->left_plan_step);
                    current_node = current_node->parent_node;
                }
            }

        }
    }
}
