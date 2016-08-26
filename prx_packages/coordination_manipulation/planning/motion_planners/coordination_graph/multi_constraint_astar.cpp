/**
 * @file multi_constraint_astar_t.hpp 
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

#include "planning/motion_planners/coordination_graph/multi_constraint_astar.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"


namespace prx
{
    using namespace util;
    using namespace sim;
    
    namespace packages
    {
        namespace coordination_manipulation
        {
            multi_constraint_astar_t::multi_constraint_astar_t()
            {
                linked_constraints = NULL;
                linked_right_plans = linked_left_plans = NULL;
                linked_right_tasks = linked_left_tasks = NULL;
                max_right_size = 0;
                max_left_size = 0;
            }

            multi_constraint_astar_t::~multi_constraint_astar_t()
            {
                clear_node_memory();
            }

            void multi_constraint_astar_t::clear_node_memory()
            {
                
                if (node_memory.size() > 0)
                {
                    for (unsigned i = 0; i < node_memory.size(); i++)
                    {
                        for(unsigned j = 0; j < node_memory[i].size(); j++)
                        {
                            delete node_memory[i][j];
                        }
                        
                        node_memory[i].clear();
                    }
                    node_memory.clear();
                }
            }
            
            void multi_constraint_astar_t::link_constraints(coordination_constraints_t* constraints)
            {
                if (linked_constraints != NULL)
                {
                    PRX_FATAL_S ("Already linked constraints!");
                }
                else
                {
                    linked_constraints = constraints;
                }
            }
            
            void multi_constraint_astar_t::link_task_plans(util::hash_t<std::string, std::vector<sim::plan_t> >* right_plans_map, util::hash_t<std::string, std::vector<sim::plan_t> >* left_plans_map)
            {
                linked_right_plans = right_plans_map;
                linked_left_plans = left_plans_map;
            }
            
            void multi_constraint_astar_t::link_tasks(std::vector<unsigned>* right_tasks, std::vector<unsigned>* left_tasks, std::vector<std::string>* right_objects, std::vector<std::string>* left_objects )
            {
                linked_right_tasks = right_tasks;
                linked_left_tasks = left_tasks;
                linked_right_objects = right_objects;
                linked_left_objects = left_objects;
            }
            
            void multi_constraint_astar_t::set_coordination_bias(coordination_heuristic_type_t bias)
            {
                bias_type = bias;
                
                define_batch_coordination_problem();
            }

            bool multi_constraint_astar_t::check_if_goal_found(const multi_constraint_node_t* current_node)
            {
                unsigned current_right_step = current_node->right_plan_step;
                unsigned current_left_step = current_node->left_plan_step;

                if (bias_type == BIAS_RIGHT || bias_type == ANY_BIAS)
                {
                    if (current_right_step == right_goal)
                    {
                        PRX_PRINT("RIGHT ARM FINISH. THIS IS GONNA BREAK THINGS!", PRX_TEXT_RED);
                        append_actual_goal = true;
                        return true;
                    }
                    
                    return false;
                }
                if (bias_type == BIAS_LEFT || bias_type == ANY_BIAS)
                {
                    if (current_left_step == left_goal)
                    {
                        PRX_PRINT("LEFT ARM FINISH. THIS IS GONNA BREAK THINGS!", PRX_TEXT_BLUE);
                        append_actual_goal = true;
                        return true;
                    }
                    
                    return false;
                }
                
                return (current_right_step == right_goal && current_left_step == left_goal);
                
            }
            
            void multi_constraint_astar_t::get_valid_neighbors(const multi_constraint_node_t* current_node, std::vector<multi_constraint_node_t*>& neighbor_nodes)
            {
                
                //TODO: FIX THIS
                
                int right_step = current_node->right_plan_step;
                int left_step = current_node->left_plan_step;
//                PRX_PRINT ("GOAL right: " << right_goal << " , GOAL left: " << left_goal, PRX_TEXT_GREEN);
//                
//                PRX_PRINT ("Current right: " << right_step << " and left: " << left_step, PRX_TEXT_GREEN);
                
                unsigned current_rob_step = current_node->rob_step;
                unsigned current_lob_step = current_node->lob_step;
                
                unsigned current_rob_index = current_node->rob_index;
                unsigned current_lob_index = current_node->lob_index;
                
                std::string current_rob_type = current_node->right_type_identifier;
                std::string current_lob_type = current_node->left_type_identifier;
                
                bool south_valid = false, north_valid = false, east_valid = false, west_valid = false;
                int south = left_step - 1, north = left_step + 1;
                int west = right_step - 1, east = right_step + 1;
                
                unsigned south_lob_step, south_lob_index;
                std::string south_lob_type;
                /** Check if south is valid, lob changes, false == left arm*/
                if(get_valid_object_step_index(south, south_lob_step, south_lob_index, south_lob_type, false))
                {
//                    PRX_WARN_S ("Potential valid south!");
                    if (!is_in_collision(current_rob_step, south_lob_step, current_rob_index, south_lob_index, current_rob_type, south_lob_type))
                    {
//                        PRX_ERROR_S("VAALLEEED");
                        multi_constraint_node_t* new_node = node_memory[right_step][south];
                        new_node->set(right_step, south, current_rob_step, south_lob_step, current_rob_index, south_lob_index, current_rob_type, south_lob_type);
//                        if (bias_type == JOINT_BIAS)
//                            new_node->edge_cost = 1.0;
//                        else
//                            new_node->edge_cost = 2.0;
                        new_node->edge_cost = 1.0;    
                        neighbor_nodes.push_back(new_node);
                        south_valid = true;
                    }
                }
                unsigned north_lob_step, north_lob_index;
                std::string north_lob_type;
                /** Check if north is valid, lob changes, false == left arm*/
                if(get_valid_object_step_index(north, north_lob_step, north_lob_index, north_lob_type,false))
                {
//                    PRX_WARN_S ("Potential valid north!");
                    if (!is_in_collision(current_rob_step, north_lob_step, current_rob_index, north_lob_index, current_rob_type, north_lob_type))
                    {
//                        PRX_ERROR_S("VAALLEEED");
                        multi_constraint_node_t* new_node = node_memory[right_step][north];
                        new_node->set(right_step, north, current_rob_step, north_lob_step, current_rob_index, north_lob_index, current_rob_type, north_lob_type);
//                        if (bias_type == JOINT_BIAS)
//                            new_node->edge_cost = 1.0;
//                        else
//                            new_node->edge_cost = 1.0;
                        new_node->edge_cost = 1.0;
                        neighbor_nodes.push_back(new_node);
                        north_valid = true;
                    }
                }
                unsigned east_rob_step, east_rob_index;
                std::string east_rob_type;
                /** Check if east is valid, rob changes, true == right arm*/
                if(get_valid_object_step_index(east, east_rob_step, east_rob_index, east_rob_type,true))
                {
//                    PRX_WARN_S ("Potential valid east!");
                    if (!is_in_collision(east_rob_step, current_lob_step, east_rob_index, current_lob_index, east_rob_type, current_lob_type))
                    {
//                        PRX_ERROR_S("VAALLEEED");
                        multi_constraint_node_t* new_node = node_memory[east][left_step];
                        new_node->set(east, left_step, east_rob_step, current_lob_step, east_rob_index, current_lob_index, east_rob_type, current_lob_type);
//                        if (bias_type == JOINT_BIAS)
//                            new_node->edge_cost = 1.0;
//                        else
//                            new_node->edge_cost = 1.0;
                        new_node->edge_cost = 1.0;
                        neighbor_nodes.push_back(new_node);
                        east_valid = true;
                    }
                }
                unsigned west_rob_step, west_rob_index;
                std::string west_rob_type;
                /** Check if west is valid, rob changes, true == right arm*/
                if(get_valid_object_step_index(west, west_rob_step, west_rob_index, west_rob_type, true))
                {
//                    PRX_WARN_S ("Potential valid west!");
                    if (!is_in_collision(west_rob_step, current_lob_step, west_rob_index, current_lob_index, west_rob_type, current_lob_type))
                    {
//                        PRX_ERROR_S("VAALLEEED");
                        multi_constraint_node_t* new_node = node_memory[west][left_step];
                        new_node->set(west, left_step, west_rob_step, current_lob_step, west_rob_index, current_lob_index, west_rob_type, current_lob_type);
//                        if (bias_type == JOINT_BIAS)
//                            new_node->edge_cost = 1.0;
//                        else
//                            new_node->edge_cost = 2.0;
                        new_node->edge_cost = 1.0;
                        neighbor_nodes.push_back(new_node);
                        west_valid = true;
                    }
                }
                
                /** Check if NE valid */
                if (north_valid && east_valid)
                {
//                    PRX_WARN_S ("Potential valid ne!");
                    multi_constraint_node_t* new_node = node_memory[east][north];
                    new_node->set(east, north, east_rob_step, north_lob_step, east_rob_index, north_lob_index, east_rob_type, north_lob_type);
//                    if (bias_type == JOINT_BIAS)
//                        new_node->edge_cost = 1.0;
//                    else
//                        new_node->edge_cost = 1.0;  
                    new_node->edge_cost = 1.0;
                    neighbor_nodes.push_back(new_node);
                }
                
                /** Check if NW valid */
                if (north_valid && west_valid)
                {
//                    PRX_WARN_S ("Potential valid nw!");
                    multi_constraint_node_t* new_node = node_memory[west][north];
                    new_node->set(west, north, west_rob_step, north_lob_step, west_rob_index, north_lob_index, west_rob_type, north_lob_type);
//                    if (bias_type == JOINT_BIAS)
//                        new_node->edge_cost = 1.0;
//                    else
//                        new_node->edge_cost = 2.0;
                    new_node->edge_cost = 1.0;
                    neighbor_nodes.push_back(new_node);
                }
                
                /** Check if SE valid */
                if (south_valid && east_valid)
                {
//                    PRX_WARN_S ("Potential valid se!");
                    multi_constraint_node_t* new_node = node_memory[east][south];
                    new_node->set(east, south, east_rob_step, south_lob_step, east_rob_index, south_lob_index, east_rob_type, south_lob_type);
                    
//                    if (bias_type == JOINT_BIAS)
//                        new_node->edge_cost = 1.0;
//                    else
//                        new_node->edge_cost = 1.0;
                    new_node->edge_cost = 1.0;
                    neighbor_nodes.push_back(new_node);
                }
                
                /** Check if SW valid */
                if (south_valid && west_valid)
                {
//                    PRX_WARN_S ("Potential valid sw!");
                    multi_constraint_node_t* new_node = node_memory[west][south];
                    new_node->set(west, south, west_rob_step, south_lob_step, west_rob_index, south_lob_index, west_rob_type, south_lob_type);
                    
//                    if (bias_type == JOINT_BIAS)
//                        new_node->edge_cost = 1.0;
//                    else
//                        new_node->edge_cost = 1.0;
                    new_node->edge_cost = 1.0;
                    neighbor_nodes.push_back(new_node);
                }
                
//                    PRX_WARN_S ("DONE!!");
            }


            bool multi_constraint_astar_t::solve()
            {
                PRX_PRINT ("MAH SOLVE BAYBEEE", PRX_TEXT_LIGHTGRAY);
                double dist;

                for(unsigned i = 0; i < node_memory.size(); i++)
                {
                    for(unsigned j = 0; j < node_memory[i].size(); j++)
                    {
                        node_memory[i][j]->initialize();
                    }
                }
                start_node = node_memory[0][0];
                start_node->rob_index = linked_right_tasks->front();
                start_node->lob_index = linked_left_tasks->front();
                start_node->right_type_identifier = linked_right_objects->front();
                start_node->left_type_identifier = linked_left_objects->front();
                start_node->g_cost = 0.0;
                start_node->f_cost = heuristic(start_node);
                start_node->color = ASTAR_COLOR_GRAY;
                
                multi_constraint_node_pointer_t node; 
                node.ptr = start_node;
                astar_heap.push(node);
                while( !astar_heap.empty() )
                {
                    node = astar_heap.top();
                    multi_constraint_node_t* u_node = node.ptr;
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
                    
                    
                    
                    std::vector<multi_constraint_node_t*> neighbors;
                    get_valid_neighbors(u_node, neighbors);
                    
                    if (neighbors.size() == 0)
                    {
                        PRX_ERROR_S ("This node has 0 neighbors! R: " << u_node->right_plan_step << ", L:" << u_node->left_plan_step);
                        //PRX_ASSERT(false);
                    }

                    foreach(multi_constraint_node_t* n, neighbors)
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
                            if (n->color == ASTAR_COLOR_WHITE )
                            {
//                                PRX_ERROR_S("ADD DA NODE:  " << n->right_plan_step << "," << n->left_plan_step);
                                n->f_cost = dist + heuristic(n);
                                n->color  = ASTAR_COLOR_GRAY;
                                multi_constraint_node_pointer_t new_node;
                                new_node.ptr = n;
                                astar_heap.push(new_node);
                            }

                        }
//                        PRX_ASSERT(false);
                    }
                    u_node->color = ASTAR_COLOR_BLACK;

                }
                PRX_ERROR_S ("LAST NODE CHECKED: " << node.ptr->right_plan_step <<" , " << node.ptr->left_plan_step);
                return false;
            }
            
            double multi_constraint_astar_t::heuristic(const multi_constraint_node_t* current_node)
            {

                double right_dist = (double)right_goal - (double)current_node->right_plan_step;
                double left_dist = (double)left_goal - (double)current_node->left_plan_step;
                double return_val;
                return_val =  right_dist + left_dist;
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
                
                return return_val;
                
                
            }
            
            
            void multi_constraint_astar_t::define_batch_coordination_problem()
            {
                while( !astar_heap.empty() )
                {
                    astar_heap.pop();
                }
                
                right_start = 0;
                left_start = 0;
                
                int plan_size;
                unsigned task_index;
                std::string object_type;
                
                max_left_size = 0;
                for(unsigned i = 0; i < linked_left_tasks->size(); i++)
                {
                    task_index = linked_left_tasks->at(i);
                    object_type = linked_left_objects->at(i);
                    plan_size = (*linked_left_plans)[object_type][task_index].size();
                    max_left_size += (plan_size);
                }
                
                max_right_size = 0;
                for(unsigned i = 0; i < linked_right_tasks->size(); i++)
                {
                    task_index = linked_right_tasks->at(i);
                    object_type = linked_right_objects->at(i);
                    plan_size = (*linked_right_plans)[object_type][task_index].size();
                    max_right_size += (plan_size);
                }
                if (max_right_size == 0 || max_left_size == 0)
                {
                    PRX_FATAL_S ("Batch coordination for an empty plan!");
                }
                
                right_goal = max_right_size-1;
                left_goal = max_left_size-1;
                clear_node_memory();
                node_memory.resize(max_right_size);
                for(unsigned right_step = 0; right_step < max_right_size; right_step++)
                {
                    node_memory[right_step].resize(max_left_size);
                    for (unsigned left_step = 0; left_step < max_left_size; left_step++)
                    {
                        node_memory[right_step][left_step] = new multi_constraint_node_t();
                    }
                }
                
                PRX_PRINT ("Defined batch coordination problem: RG: " << right_goal << " and LG: " << left_goal, PRX_TEXT_MAGENTA);
            }
            
            bool multi_constraint_astar_t::get_valid_object_step_index(int step_index, unsigned& ob_step_index, unsigned& object_index,  std::string& object_type_identifier, bool right_arm)
            {
                int start_check, goal_check;
                std::vector<unsigned>* tasks_to_check;
                util::hash_t<std::string, std::vector<sim::plan_t> >* plans_to_check;
                std::vector<std::string>* types_to_check;
                
                if (right_arm)
                {
                    start_check = right_start;
                    goal_check =  right_goal;
                    tasks_to_check = linked_right_tasks;
                    plans_to_check = linked_right_plans;
                    types_to_check = linked_right_objects;
                }
                else
                {
                    start_check = left_start;
                    goal_check =  left_goal;
                    tasks_to_check = linked_left_tasks;
                    plans_to_check = linked_left_plans;
                    types_to_check = linked_left_objects;
                }
                
//                if (right_arm)
//                {
//                    PRX_PRINT ("RIGHT ARM", PRX_TEXT_MAGENTA);
//                }
//                else
//                {
//                    PRX_PRINT (" LEFT ARM ", PRX_TEXT_CYAN);
//                }
//                PRX_PRINT ("Step: " << step_index << " vs start: " << start_check << " and goal: " << goal_check, PRX_TEXT_MAGENTA);
                
                if (step_index < start_check || step_index > goal_check)
                    return false;
                
//                PRX_WARN_S ("HAJIME!");
                
                int test_index = step_index;
                unsigned task_index; std::string object_check;
                for(unsigned i = 0; i < tasks_to_check->size(); i++)
                {
//                    PRX_WARN_S ("test index: " << test_index);
                    task_index = (*tasks_to_check)[i];
                    object_check = (*types_to_check)[i];
                    int plan_size = (*plans_to_check)[object_check][task_index].size();
                    
                    if (test_index - plan_size < 0)
                    {
//                        PRX_ERROR_S ("YEAAAH BABABABABABABA");
                        ob_step_index = test_index;
                        object_index = task_index;
                        object_type_identifier = object_check;
                        return true;
                    }
                    
                    test_index -= plan_size;
                    
                }
                
                if (test_index <= 0 || step_index == goal_check)
                {
                    ob_step_index = test_index;
                    object_index = task_index;
                    object_type_identifier = object_check;
                    return true;
                }
                
                return false;

                
            }
            
            bool multi_constraint_astar_t::is_in_collision(unsigned rob_step_index, unsigned lob_step_index, unsigned rob_index, unsigned lob_index, std::string right_type, std::string left_type)
            {
                return !(linked_constraints->is_valid_state(rob_step_index, lob_step_index, rob_index, lob_index, right_type, left_type));
            }
            
            void multi_constraint_astar_t::get_solution_plan (const space_t* right_arm_space, const space_t* left_arm_space, space_t* joint_arm_space, plan_t& plan_to_fill)
            {
                if (goal_node == NULL)
                {
                    PRX_FATAL_S ("Get solution path called before a valid goal was found");
                }
                
                control_t* right_zero_control, *left_zero_control;
                control_t* right_arm_control, *left_arm_control;
                control_t* joint_arm_control = joint_arm_space->alloc_point();
                
                right_zero_control = right_arm_space->alloc_point();
                right_arm_space->zero(right_zero_control);
                
                left_zero_control = left_arm_space->alloc_point();
                left_arm_space->zero(left_zero_control);
                
                plan_to_fill.clear();
                multi_constraint_node_t* current_node = goal_node;
                std::deque<unsigned> rob_steps, lob_steps;
                std::deque<unsigned> rob_indices, lob_indices;
                std::deque<std::string> rob_type_indices, lob_type_indices;
                while ( current_node != NULL)
                {
                    unsigned current_rob_step = current_node->rob_step;
                    unsigned current_lob_step = current_node->lob_step;

                    unsigned current_rob_index = current_node->rob_index;
                    unsigned current_lob_index = current_node->lob_index;
                    
                    std::string current_right_type_index = current_node->right_type_identifier;
                    std::string current_left_type_index = current_node->left_type_identifier;
                    
                    rob_steps.push_front(current_rob_step);
                    lob_steps.push_front(current_lob_step);
                    
                    rob_indices.push_front(current_rob_index);
                    lob_indices.push_front(current_lob_index);
                    
                    rob_type_indices.push_front(current_right_type_index);
                    lob_type_indices.push_front(current_left_type_index);
                    
                    current_node = current_node->parent_node;
                }
                int prev_rob_step = -1, prev_lob_step = -1;
                int prev_rob_index = -1, prev_lob_index = -1;
                for (unsigned i = 0; i < rob_steps.size(); ++i)
                {
                    unsigned current_rob_step = rob_steps[i];
                    unsigned current_lob_step = lob_steps[i];

                    unsigned current_rob_index = rob_indices[i];
                    unsigned current_lob_index = lob_indices[i];
                    
                    std::string right_type = rob_type_indices[i];
                    std::string left_type = lob_type_indices[i];
                    
                    if (prev_rob_step == current_rob_step && prev_rob_index == current_rob_index)
                    {
                        right_arm_control = right_zero_control;
                    }
                    else
                    {
                        right_arm_control = (*linked_right_plans)[right_type][current_rob_index].consume_control(simulation::simulation_step);
                    }
                    
                    
                    if (prev_lob_step == current_lob_step && prev_lob_index == current_lob_index)
                    {
                        left_arm_control = left_zero_control;
                    }
                    else
                    {
                        left_arm_control = (*linked_left_plans)[left_type][current_lob_index].consume_control(simulation::simulation_step);
                    }
                    
                    
                    right_arm_space->copy_from_point(right_arm_control);
                    left_arm_space->copy_from_point(left_arm_control);
                    
                    joint_arm_space->copy_to_point(joint_arm_control);
                    
                    plan_to_fill.copy_onto_front(joint_arm_control, simulation::simulation_step);
                    
                }
                
                joint_arm_space->free_point(joint_arm_control);
                
//                PRX_DEBUG_COLOR ("Finished filling plan: " << plan_to_fill.print(), PRX_TEXT_BROWN);
                
                
            }
            
//            void multi_constraint_astar_t::get_solution_path (std::deque<unsigned>& right_solution_indices, std::deque<unsigned>& left_solution_indices)
//            {
//                if (goal_node == NULL)
//                {
//                    PRX_FATAL_S ("Get solution path called before a valid goal was found");
//                }
//                right_solution_indices.clear();
//                left_solution_indices.clear();
//                multi_constraint_node_t* current_node = goal_node;
//                while ( current_node->parent_node != NULL)
//                {
//                    right_solution_indices.push_front(current_node->right_plan_step);
//                    left_solution_indices.push_front(current_node->left_plan_step);
//                    current_node = current_node->parent_node;
//                }
//            }

        }
    }
}
