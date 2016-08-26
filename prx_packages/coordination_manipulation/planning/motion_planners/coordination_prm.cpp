///**
// * @file irs.cpp
// * 
// * @copyright Software License Agreement (BSD License)
// * Copyright (c) 2011, Board of Regents, NSHE, obo UNR
// * All Rights Reserved.
// * For a full description see the file named LICENSE.
// *
// * @authors Athanasios Krontiris, Andrew Dobson, Andrew Kimmel, Zakary Littlefield, James Marble, Kostas Bekris 
// * 
// * Email: pracsys@googlegroups.com
// */
//
//#include "planning/motion_planners/coordination_prm.hpp"
//#include "prx/utilities/boost/boost_wrappers.hpp"
//#include "prx/utilities/distance_metrics/distance_metric.hpp"
//#include "prx/utilities/goals/goal.hpp"
//#include "prx/planning/modules/samplers/sampler.hpp"
//#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
//#include "prx/planning/modules/local_planners/local_planner.hpp"
//#include "prx/planning/modules/heuristic_search/astar_module.hpp"
//#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
//#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
//#include "prx/utilities/definitions/random.hpp"
//
//#include <boost/property_map/vector_property_map.hpp>
//#include <boost/graph/connected_components.hpp>
//
//#include <pluginlib/class_list_macros.h> 
//
//PLUGINLIB_EXPORT_CLASS(prx::packages::coordination_manipulation::coordination_prm_t, prx::plan::planner_t)
//
//
//namespace prx
//{
//    using namespace util;
//    using namespace sim;
//    using namespace plan;
//
//    namespace packages
//    {
//        namespace coordination_manipulation
//        {
//            coordination_prm_t::coordination_prm_t() 
//            {
//                prm_query = NULL;
//                full_arm_state_space = NULL;
//                full_arm_control_space = NULL;
//                right_start = NULL;
//                right_goal = NULL;
//                left_start = NULL;
//                left_goal = NULL;
//                max_step = 0.05;
//            }
//
//            coordination_prm_t::~coordination_prm_t() 
//            {
//                prm_query->left_state_space->free_point(left_start);
//                prm_query->left_state_space->free_point(left_goal);
//                
//                prm_query->right_state_space->free_point(right_start);
//                prm_query->right_state_space->free_point(right_goal);
//                
//                
//            }
//
//            void coordination_prm_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
//            {
//                PRX_INFO_S("Initializing Coordination prm motion planner ...");
//                prm_star_t::init(reader, template_reader);
//                
//                bias_left_percentage = parameters::get_attribute_as<double>("bias_left_percentage", reader, template_reader, 0.0);
//                bias_right_percentage = parameters::get_attribute_as<double>("bias_right_percentage", reader, template_reader, 0.0);
//                
//                int dice_to_roll = parameters::get_attribute_as<int>("dices_to_roll", reader, template_reader, 0);
//                
//                for(unsigned i = 0; i < dice_to_roll; i++)
//                {
//                    dice_rolls.push_back(uniform_random());
//                }
//            }
//            
//            void coordination_prm_t::link_specification(specification_t* new_spec)
//            {
//                motion_planner_t::link_specification(new_spec);
//
//                if( input_specification->astar != NULL )
//                {
//                    if( astar != NULL )
//                    {
//                        PRX_WARN_S("PRM's A* module will replaced with the A* module from specification!");
//                        delete astar;
//                    }
//                    astar = input_specification->astar;
//                }
//                if( astar == NULL )
//                    PRX_FATAL_S("No A* module has been specified in motion planner " << path);
//                
//                full_arm_state_space = input_specification->state_space;
//                full_arm_control_space = input_specification->control_space;
//
//            }
//            
//            void coordination_prm_t::link_query(query_t* new_query)
//            {
//                PRX_ASSERT(input_query == NULL);
//                input_query = (motion_planning_query_t*)new_query;
//                PRX_ASSERT(input_specification != NULL);
////                input_specification->get_stopping_criterion()->link_goal(input_query->get_goal());
//                
//                prm_query = dynamic_cast<coordination_prm_query_t*>(new_query);
//                PRX_ASSERT(prm_query != NULL);
//                
////                PRX_ASSERT(prm_query->left_plan->size() == prm_query->right_plan->size());
////                PRX_ASSERT(prm_query->left_plan->length() == prm_query->right_plan->length());
//                
//                /** Extract information necessary */
//                
//                state_space = prm_query->coordination_state_space;
//                metric->link_space(state_space);
//                
//                random_point = state_space->alloc_point();
//                path1.link_space(full_arm_state_space);
//                path2.link_space(full_arm_state_space);
//                new_plan.link_control_space(full_arm_control_space);
//                new_plan2.link_control_space(full_arm_control_space);
//                new_plan.link_state_space(full_arm_state_space);
//                new_plan2.link_state_space(full_arm_state_space);
//                right_full_plan.link_control_space(full_arm_state_space);
//                right_full_plan.link_state_space(full_arm_control_space);
//                left_full_plan.link_control_space(full_arm_control_space);
//                left_full_plan.link_state_space(full_arm_state_space);
//                
//                left_start = prm_query->left_state_space->alloc_point();
//                left_goal = prm_query->left_state_space->alloc_point();
//                left_tuned_plan.link_control_space(prm_query->left_control_space);
//                left_tuned_plan.link_state_space(prm_query->left_state_space);
//                left_untuned_plan.link_control_space(prm_query->left_control_space);
//                left_untuned_plan.link_state_space(prm_query->left_state_space);
//                left_partial_plan.link_control_space(prm_query->left_control_space);
//                left_partial_plan.link_state_space(prm_query->left_state_space);
//                
//                right_start = prm_query->right_state_space->alloc_point();
//                right_goal = prm_query->right_state_space->alloc_point();
//                right_tuned_plan.link_control_space(prm_query->right_control_space);
//                right_tuned_plan.link_state_space(prm_query->right_state_space);
//                right_untuned_plan.link_control_space(prm_query->right_control_space);
//                right_untuned_plan.link_state_space(prm_query->right_state_space);
//                right_partial_plan.link_control_space(prm_query->right_control_space);
//                right_partial_plan.link_state_space(prm_query->right_state_space);
//                
//                keep_plan.link_control_space(full_arm_control_space);
//                keep_plan.link_state_space(full_arm_state_space);
//
//                //PRX_ASSERT(false);
//                
//                coordination_astar = dynamic_cast<coordination_astar_t*>(astar);
//                if (coordination_astar == NULL)
//                {
//                    PRX_FATAL_S("Must have a coordination astar");
//                }
//                
//                coordination_astar->link_spaces(state_space, control_space);
//                coordination_astar->link_modules(validity_checker, local_planner);
//                coordination_astar->link_distance_metric(metric);
//            }
//
//            void coordination_prm_t::setup()
//            {
//                PRX_PRINT("Coordination setup adds seeded nodes (called after link_query or reset)", PRX_TEXT_RED);
//                add_node(prm_query->get_start_state());
//                update_k(num_vertices);
//
//                add_node(prm_query->ideal_goal_state);
//                update_k(num_vertices);
//                
//                add_node(prm_query->coordination_goal_state);
//                update_k(num_vertices);
//                
//                goal_right_arm = prm_query->coordination_goal_state->at(0);
//                goal_left_arm = prm_query->coordination_goal_state->at(1);
//                
//            }
//            
//            void coordination_prm_t::reset()
//            {
//                PRX_PRINT("Vertices::: " << boost::num_vertices(graph.graph) << ", Edges::: " << boost::num_edges(graph.graph), PRX_TEXT_BLUE);
//                PRX_PRINT("Resetting coordination prm", PRX_TEXT_RED);
//                
//                prm_star_t::reset();
//                PRX_PRINT("Vertices::: " << boost::num_vertices(graph.graph) << ", Edges::: " << boost::num_edges(graph.graph), PRX_TEXT_BLUE);
//                                
//                PRX_ASSERT(prm_query != NULL);
//
//                /** Clear plans and paths */
//                new_plan.clear();
//                new_plan2.clear();
//                
//                left_tuned_plan.clear();
//                left_untuned_plan.clear();
//                
//                right_tuned_plan.clear();
//                right_untuned_plan.clear();
//                
//                //PRX_ASSERT(false);
//                coordination_astar->restart();
//                input_specification->get_stopping_criterion()->reset();
//            }
//            
//            
//            void coordination_prm_t::resolve_query()
//            {
//
//                sys_clock_t resolve_timer;
//                resolve_timer.reset();
//                std::vector<space_point_t*> goals = input_query->get_goal()->get_goal_points();
//                std::vector<undirected_vertex_index_t> v_goals;
//
//                if( input_query->q_type == motion_planning_query_t::PRX_ADD_QUERY_POINTS_NO_COLLISIONS )
//                    no_collision_query_type = true;
//                else if( input_query->q_type == motion_planning_query_t::PRX_NEAR_QUERY_POINTS )
//                    near_query_type = true;
//
//                undirected_vertex_index_t v_start;
//                boost::tie(remove_start, v_start) = add_node(input_query->get_start_state());
//
//                undirected_vertex_index_t v_g;
//                bool remove_goal;
//
//                foreach(space_point_t* g, goals)
//                {
//                    PRX_PRINT("---GOAL--\n\n:::: " << state_space->print_point(g), PRX_TEXT_BROWN);
//                    boost::tie(remove_goal, v_g) = add_node(g);
//                    v_goals.push_back(v_g);
//                    //                PRX_PRINT("Adding v_goal to the v_goals : " << v_goals.size(), PRX_TEXT_CYAN);
//                    remove_goals.push_back(remove_goal);
//                }
//
//                //PRX_PRINT("Start has valence: " << boost::out_degree(v_start, graph.graph), PRX_TEXT_BLUE);
//                //PRX_PRINT("Start is: " << v_start, PRX_TEXT_LIGHTGRAY);
//
//
//                //Now, let's just check to see there is some goal that is reachable before we do anything else.
//                bool good_to_go = false;
//                unsigned num_component = boost::connected_components(graph.graph, graph.components);
//                for( unsigned i = 0; i < v_goals.size(); ++i )
//                {
//                    if( graph.components[v_start] == graph.components[v_goals[i]] )
//                    {
//                        good_to_go = true;
//                        break;
//                    }
//                }
//                if(!good_to_go)
//                {
//                     PRX_WARN_S("Start and goal(s) are not in the same connected component: |components| = " << num_component);
//                     PRX_PRINT("Val(start): " << boost::out_degree(v_start, graph.graph) << "   Val(goal): " << boost::out_degree(v_goals[0], graph.graph), PRX_TEXT_LIGHTGRAY );
//                     PRX_PRINT("Metric points: " << metric->get_nr_points(), PRX_TEXT_LIGHTGRAY);
//
//                    return;
//                }
//                // PRX_PRINT("Start and Goal are graph-connected.  Starting A*:", PRX_TEXT_LIGHTGRAY);
//                // int goal_edges = boost::num_edges(graph.graph);
//                //PRX_ERROR_S ("Edges connected to goal: " << goal_edges -  new_num_edges_start);
//
//                //            PRX_PRINT("metric size: " << metric->get_nr_points() << "     q_collision_type: " << input_query->q_collision_type, PRX_TEXT_BROWN);
//                
//                
//                
//                coordination_astar->link_graph(&graph);
//                input_query->plan.clear();
//                input_query->path.clear();
//                prm_query->partial_plan.clear();
//                
//                coordination_astar->set_astar_mode(astar_module_t::PRX_NO_EDGE_INFO);
//                bool found_left_path = false, found_right_path = false, found_path = false;
//                coordination_astar_t::coordination_heuristic_type_t right_arm_bias, left_arm_bias, arm_bias;
//                
////                if(prm_query->incremental_assignment)
////                {
////                    std::deque<undirected_vertex_index_t> left_path_vertices, right_path_vertices;
////                    // Run coordination_astar biasing X
////                    coordination_astar->set_coordination_problem(goal_right_arm, goal_left_arm, coordination_astar_t::BIAS_RIGHT);
////                    found_right_path = coordination_astar->solve(v_start, v_goals);
////                    if(found_right_path)
////                    {
////                        coordination_astar->extract_path(v_start, coordination_astar->get_found_goal(), right_path_vertices);
////                        extract_astar_path(right_path_vertices, right_full_plan, right_partial_plan, right_arm_bias);
////                        PRX_INFO_S("Found RIGHT ARM path (" << found_right_path << ") for PRX_NO_COLLISIONS");
////                    }
////                    // Run coordination_astar biasing Y
////                    coordination_astar->set_coordination_problem(goal_right_arm, goal_left_arm, coordination_astar_t::BIAS_LEFT);
////                    coordination_astar->restart();
////                    found_left_path = coordination_astar->solve(v_start, v_goals);
////                    if(found_left_path)
////                    {
////                        coordination_astar->extract_path(v_start, coordination_astar->get_found_goal(), left_path_vertices);
////                        extract_astar_path(left_path_vertices, left_full_plan, left_partial_plan, left_arm_bias);
////                        PRX_INFO_S("Found LEFT ARM path (" << found_left_path << ") for PRX_NO_COLLISIONS");
////                    }
////                    
////                    // Return shortest path
////                    
////                    if (found_right_path && found_left_path)
////                    {
////                        if (left_full_plan.size() < right_full_plan.size())
////                        {
////                            PRX_PRINT("Selected left arm", PRX_TEXT_GREEN);
////                            prm_query->plan = left_full_plan;
////                            prm_query->partial_plan = left_partial_plan;
////                            arm_bias = left_arm_bias;
////                        }
////                        else
////                        {
////                            PRX_PRINT("Selected right arm", PRX_TEXT_GREEN);
////                            prm_query->plan = right_full_plan;
////                            prm_query->partial_plan = right_partial_plan;
////                            arm_bias = right_arm_bias;
////                            
////                        }
////                        
////                    }
////                    else if (found_right_path)
////                    {
////                        PRX_PRINT("COULD ONLY SELECT right arm", PRX_TEXT_GREEN);
////                        prm_query->plan = right_full_plan;
////                        prm_query->partial_plan = right_partial_plan;
////                        arm_bias = right_arm_bias;
////                    }
////                    else if (found_left_path)
////                    {
////                        PRX_PRINT("COULD ONLY SELECT left arm", PRX_TEXT_GREEN);
////                        prm_query->plan = left_full_plan;
////                        prm_query->partial_plan = left_partial_plan;
////                        arm_bias = left_arm_bias;
////                    }
////                    else
////                    {
////                        PRX_FATAL_S ("Could not find valid path!!!");
////                    }
////                    
////                        
////                    if (arm_bias == coordination_astar_t::BIAS_LEFT)
////                    {
////
////                        prm_query->left_finished = true;
////                        if (prm_query->partial_plan.size() == 0)
////                            prm_query->right_finished = true;
////                        else
////                            prm_query->right_finished = false;
////                    }
////                    else if (arm_bias == coordination_astar_t::BIAS_RIGHT)
////                    {
////                        prm_query->right_finished = true;
////                        if (prm_query->partial_plan.size() == 0)
////                            prm_query->left_finished = true;
////                        else
////                            prm_query->left_finished = false;
////                    }
////                    else if (arm_bias == coordination_astar_t::ANY_BIAS)
////                    {
////                        prm_query->right_finished = true;
////                        prm_query->left_finished = true;
////                    }
////                    else
////                    {
////                        PRX_FATAL_S("Bad state to be in");
////                    }
////                }
////                else
//                {
//                    std::deque<undirected_vertex_index_t> joint_path_vertices;
//                    if (prm_query->max_velocity_bias)
//                        coordination_astar->set_coordination_problem(goal_right_arm, goal_left_arm, coordination_astar_t::ANY_BIAS);
//                    else
//                        coordination_astar->set_coordination_problem(goal_right_arm, goal_left_arm, coordination_astar_t::JOINT_BIAS);
//                    found_path = coordination_astar->solve(v_start, v_goals);
//                    if(found_path)
//                    {
//                        PRX_INFO_S("Found JOINT ARM path (" << found_right_path << ") for PRX_NO_COLLISIONS");
//                        coordination_astar->extract_path(v_start, coordination_astar->get_found_goal(), joint_path_vertices);
//                        extract_astar_path(joint_path_vertices, left_full_plan, left_partial_plan, arm_bias);
//                        prm_query->plan = left_full_plan;
//                        prm_query->partial_plan = left_partial_plan;
//                        if (arm_bias == coordination_astar_t::BIAS_LEFT)
//                        {
//
//                            prm_query->left_finished = true;
//                            if (prm_query->partial_plan.size() == 0)
//                                prm_query->right_finished = true;
//                            else
//                                prm_query->right_finished = false;
//                        }
//                        else if (arm_bias == coordination_astar_t::BIAS_RIGHT)
//                        {
//                            prm_query->right_finished = true;
//                            if (prm_query->partial_plan.size() == 0)
//                                prm_query->left_finished = true;
//                            else
//                                prm_query->left_finished = false;
//                        }
//                        else
//                        {
//                            prm_query->right_finished = true;
//                            prm_query->left_finished = true;
//                        }
//                    }
//                }
//                v_goals.clear();
//                remove_goals.clear();
//
//                no_collision_query_type = false;
//                near_query_type = false;
//            }
//            
//            void coordination_prm_t::extract_astar_path(const std::deque<undirected_vertex_index_t>& path_vertices, sim::plan_t& full_plan, sim::plan_t& partial_plan, coordination_astar_t::coordination_heuristic_type_t& arm_bias)
//            {
//                full_plan.clear();
//                partial_plan.clear();
//                keep_plan.clear();
//                double dist;
//                 PRX_INFO_S("coordination_astar found solution : " << path_vertices.size());
//                bool generate_traj = false;
//                bool simultaneous_finish = true;
//                for( size_t i = 0; i < path_vertices.size() - 1; ++i )
//                {
////                        PRX_PRINT("INPUT QUERY PLAN SIZE: " << input_query->plan.size(), PRX_TEXT_BLUE);
////                        PRX_PRINT("INPUT QUERY PATH SIZE: " << input_query->path.size(), PRX_TEXT_RED);
//                    undirected_edge_index_t e = boost::edge(path_vertices[i], path_vertices[i + 1], graph.graph).first;
//                    //                        PRX_DEBUG_POINT("edge plan size: " << graph.get_edge_as<prm_star_edge_t > (e)->plan.size() << "     path_size:" << graph.get_edge_as<prm_star_edge_t > (e)->path.size());
//
////                        if( i != 0 )
////                        {
////                            input_query->path.resize(input_query->path.size() - 1);
////                        }
////
////                        path1.clear();
//                    new_plan.clear();
//
//                    space_point_t* start = graph[path_vertices[i]]->point;
//                    space_point_t* goal = graph[path_vertices[i + 1]]->point;
//
//                    bool valid_steer = varying_velocity_steer(start,goal, new_plan, path1, dist, generate_traj);
//                    PRX_PRINT("SOLUTION from: " << state_space->print_point(start) << " to: " << state_space->print_point(goal), PRX_TEXT_BROWN);
//
////                        bool valid_trajectory = is_valid_trajectory(path1);
////                        
////                        PRX_ASSERT(valid_steer && valid_trajectory);
//
//                    if (prm_query->incremental_assignment)
//                    {
//                        PRX_PRINT("Incremental assignment baybee", PRX_TEXT_GREEN);
//                        keep_plan+= new_plan;
//
//                        double current_right_arm = start->at(0), current_left_arm = start->at(1);
//                        PRX_PRINT("Current start left arm: " << current_left_arm << " vs. " << goal_left_arm, PRX_TEXT_BLUE);
//                        PRX_PRINT("Current start right arm: " << current_right_arm << " vs. " << goal_right_arm, PRX_TEXT_RED);
////                            if (current_right_arm == goal_right_arm && current_left_arm == goal_left_arm && simultaneous_finish)
////                            {
////                                PRX_PRINT("BOTH ARMS FINISHED?!", PRX_TEXT_BROWN);
////                                prm_query->left_finished = true;
////                                prm_query->right_finished = true;
////                                input_query->plan += new_plan;
////                                input_query->path += path1;
////                            }
////                            else 
//                        if (current_left_arm == goal_left_arm && current_right_arm != goal_right_arm)
//                        {
//                            PRX_PRINT("Left arm finished first", PRX_TEXT_BLUE);
//                            simultaneous_finish = false;
//                            partial_plan += right_tuned_plan;
//                            arm_bias = coordination_astar_t::BIAS_LEFT;
//                            PRX_DEBUG_COLOR("FULL PLAN: \n" << new_plan.print(), PRX_TEXT_BROWN);
//                            PRX_DEBUG_COLOR("PARTIAL PLAN: \n" << prm_query->partial_plan.print(), PRX_TEXT_GREEN);
//                            PRX_DEBUG_COLOR("Final plan\n" << input_query->plan.print(5), PRX_TEXT_RED);
//                        }
//                        else if (current_right_arm == goal_right_arm && current_left_arm != goal_left_arm)
//                        {
//                            PRX_PRINT("Right arm finished first", PRX_TEXT_RED);
//                            simultaneous_finish = false;
//                            partial_plan += left_tuned_plan;
//                            arm_bias = coordination_astar_t::BIAS_RIGHT;
//                            PRX_DEBUG_COLOR("FULL PLAN: \n" << new_plan.print(), PRX_TEXT_BROWN);
//                            PRX_DEBUG_COLOR("PARTIAL PLAN: \n" << prm_query->partial_plan.print(), PRX_TEXT_GREEN);
//                            PRX_DEBUG_COLOR("Final plan\n" << input_query->plan.print(5), PRX_TEXT_RED);
//                        }
//                        else if (simultaneous_finish)
//                        {
//                            PRX_PRINT("Both finished?", PRX_TEXT_GREEN);
//                            PRX_DEBUG_COLOR("FULL PLAN: \n" << new_plan.print(), PRX_TEXT_BROWN);
//                            full_plan += new_plan;
//                            arm_bias = coordination_astar_t::ANY_BIAS;
//                            PRX_DEBUG_COLOR("Final plan\n" << input_query->plan.print(5), PRX_TEXT_RED);
////                                input_query->path += path1;
//                        }
//                        else
//                        {
//                            simultaneous_finish = true;
//                            PRX_WARN_S ("An arm has decided it isn't finished!");
//                            partial_plan.clear();
//                            full_plan = keep_plan;
//                            arm_bias = coordination_astar_t::ANY_BIAS;
//
//                        }
//                    }
//                    else
//                    {
//                        PRX_PRINT("Added to plan", PRX_TEXT_BLUE);
//                        full_plan += new_plan;
//                        arm_bias = coordination_astar_t::ANY_BIAS;
////                            input_query->path += path1;
//                    }
//                        
//                    
//                }
//                        
////                bool final_check = is_valid_trajectory(input_query->path);
////
////                PRX_PRINT("Final path is valid? " << final_check << "\n" << input_query->path.print(8), PRX_TEXT_BLUE);
////                PRX_PRINT("Final plan\n" << input_query->plan.print(8), PRX_TEXT_RED);
////                PRX_ASSERT(final_check);
//
// 
//            }            
//            
//            void coordination_prm_t::valid_random_sample()
//            {
//                sampler->sample(state_space, random_point);
//                
//                double rolled_dice = dice_rolls[num_generated];
//                
//                bool bias_towards_left = (rolled_dice <= bias_left_percentage);
//                bool bias_towards_right = ((1.0-rolled_dice) <= bias_right_percentage);
//                
//                if (bias_towards_left && !bias_towards_right)
//                {
//                    PRX_PRINT ("Bias towards left!", PRX_TEXT_BLUE);
//                    random_point->at(1) = goal_left_arm;
//                }
//                else if (!bias_towards_left && bias_towards_right)
//                {
//                    PRX_PRINT ("Bias towards right!", PRX_TEXT_RED);
//                    random_point->at(0) = goal_right_arm;
//                }
//                
//                ++num_generated;
//            }
//
//            void coordination_prm_t::link_node_to_neighbors(undirected_vertex_index_t v, const std::vector< const abstract_node_t* >& neighbors)
//            {
//                const undirected_node_t* node;
//
//                //PRX_PRINT("Num Neighbors: " << neighbors.size(), PRX_TEXT_BROWN);
//
//                path1.clear();
//                //PRX_PRINT("Linking node: " << v, PRX_TEXT_GREEN);
//                //PRX_PRINT("Linking to neighbors: " << neighbors.size(), PRX_TEXT_RED);
//                for( size_t i = 0; i < neighbors.size(); i++ )
//                {
//                    node = neighbors[i]->as< undirected_node_t > ();
//                    new_plan.clear();
//                    path1.clear();
////                    PRX_PRINT("Attempting edge connection between: " << state_space->print_point(graph[v]->point) <<" and : " << state_space->print_point(node->point), PRX_TEXT_LIGHTGRAY);
//
//                    //If the path is valid
//                    double dist;
//                    bool generate_traj = true;
//                    bool valid_steer = varying_velocity_steer(graph[v]->point, node->point, new_plan, path1, dist, generate_traj);
//                    bool valid_trajectory = false;
//                    if (valid_steer)
//                    {
//                        valid_trajectory = is_valid_trajectory(path1);
//                    }
//                    
////                    PRX_WARN_S ("Valid steer: " << valid_steer <<" and valid trajectory: " << valid_trajectory);
//                    if(  valid_steer && valid_trajectory )
//                    {
////                        PRX_PRINT("// =======================================================", PRX_TEXT_LIGHTGRAY);
////                        PRX_PRINT("// =======================================================", PRX_TEXT_LIGHTGRAY);
////                        PRX_PRINT("// =======================================================", PRX_TEXT_LIGHTGRAY);
////                        PRX_PRINT("// ================......===========......================", PRX_TEXT_LIGHTGRAY);
////                        PRX_PRINT("// ==================.....=========.....==================", PRX_TEXT_LIGHTGRAY);
////                        PRX_PRINT("// ===================.....=======.....===================", PRX_TEXT_LIGHTGRAY);
////                        PRX_PRINT("// ====================.....=====.....====================", PRX_TEXT_LIGHTGRAY);
////                        PRX_PRINT("// =====================.....===.....=====================", PRX_TEXT_LIGHTGRAY);
////                        PRX_PRINT("// ======================.....=.....======================", PRX_TEXT_LIGHTGRAY);
////                        PRX_PRINT("// =======================.........=======================", PRX_TEXT_LIGHTGRAY);
////                        PRX_PRINT("// ========================.......========================", PRX_TEXT_LIGHTGRAY);
////                        PRX_PRINT("// ============ WOOOHOOOOOOOOOOOOOOOOOOOOOOOOOO===========", PRX_TEXT_LIGHTGRAY);
////                        PRX_PRINT("// ========================.......========================", PRX_TEXT_LIGHTGRAY);
////                        PRX_PRINT("// =========....==========.........==========....=========", PRX_TEXT_LIGHTGRAY);
////                        PRX_PRINT("// =======........=======.....=.....=======........=======", PRX_TEXT_LIGHTGRAY);
////                        PRX_PRINT("// ======...====...=====.....===.....=====...====...======", PRX_TEXT_LIGHTGRAY);
////                        PRX_PRINT("// ======..........====.....=====.....====..........======", PRX_TEXT_LIGHTGRAY);
////                        PRX_PRINT("// ======..===========.....=======.....===..==============", PRX_TEXT_LIGHTGRAY);
////                        PRX_PRINT("// ======....==....==.....=========.....==....==....======", PRX_TEXT_LIGHTGRAY);
////                        PRX_PRINT("// =======........=......===========......=........=======", PRX_TEXT_LIGHTGRAY);
////                        PRX_PRINT("// =======================================================", PRX_TEXT_LIGHTGRAY);
////                        PRX_PRINT("// =======================================================", PRX_TEXT_LIGHTGRAY);
////                        PRX_PRINT("// =======================================================\n", PRX_TEXT_LIGHTGRAY);
////                        PRX_PRINT("Added edge with weight: " << dist, PRX_TEXT_GREEN);
//                        //Add the edge to the graph
////                        double dist = metric->distance_function(graph[v]->point, node->point);
//                        undirected_edge_index_t e = graph.add_edge< prm_star_edge_t > (v, node->index, dist);
//                        graph.get_edge_as<prm_star_edge_t > (e)->id = num_edges;
//                        num_edges++;
////                        graph.get_edge_as< prm_star_edge_t > (e)->path = path1;
////                        graph.get_edge_as< prm_star_edge_t > (e)->plan = new_plan;
//                    }
//                    // else
//                    // {
//                    //     PRX_PRINT("Plan to neighbor [" << i << "] is invalid!", PRX_TEXT_LIGHTGRAY );
//                    // }
//
//                    path1.clear();
//                }
//            }
//            
//            bool coordination_prm_t::varying_velocity_steer(const state_t* start, const state_t* goal, plan_t& plan, trajectory_t& traj, double& edge_dist, bool generate_trajectory)
//            {
////                PRX_PRINT("Computing varying velocity steer from: " << state_space->print_point(start) << " to: " << state_space->print_point(goal), PRX_TEXT_BROWN);
//                double start_x = start->at(0), start_y = start->at(1);
//                double goal_x = goal->at(0), goal_y = goal->at(1);
//                
//                double left_steps = std::fabs(goal_y - start_y) - 1;
//                if (left_steps < 0)
//                    left_steps = 0;
//                double right_steps = std::fabs(goal_x - start_x) -1;
//                if (right_steps < 0)
//                    right_steps = 0;
//                
////                PRX_PRINT("LEFT FULL PLAN SIZE: " << prm_query->left_plan->size() << " and micro-plan size: " << left_steps, PRX_TEXT_BLUE);
////                PRX_PRINT("RIGHT FULL PLAN SIZE: " << prm_query->right_plan->size() << " micro-plan size: " << right_steps, PRX_TEXT_RED);
//                
//                
//                get_arm_start_and_goal(start_y,goal_y, prm_query->left_plan, left_start,left_goal, left_untuned_plan);
//                get_arm_start_and_goal(start_x,goal_x, prm_query->right_plan, right_start,right_goal, right_untuned_plan);
//                
//                edge_dist = left_steps + right_steps;
//                
//                /** Penalize moving backwards */
//                if (goal_x < start_x)
//                    edge_dist += (start_x - goal_x);
//                if (goal_y < start_y)
//                    edge_dist +=(start_y - goal_y);
//                
//                double max_steps;
//                if (left_steps > right_steps)
//                {
////                    PRX_PRINT("LEFT LARGER!", PRX_TEXT_MAGENTA);
//                    if (prm_query->max_velocity_bias && goal_x != prm_query->coordination_goal_state->at(0))
//                        edge_dist += (left_steps - right_steps);
//                    max_steps = left_untuned_plan.size() - 1;
//                    left_tuned_plan = left_untuned_plan;
//                    left_tuned_plan.pop_front();
//                    if(!get_tuned_plan(prm_query->right_state_space, prm_query->right_control_space, max_steps, right_untuned_plan, right_tuned_plan))
//                    {
//                        right_tuned_plan = right_untuned_plan;
//                        while (right_tuned_plan.size() < left_tuned_plan.size())
//                        {
//                            right_tuned_plan.copy_onto_back(right_tuned_plan.back().control, simulation::simulation_step);
//                        }
////                        return false;
//                    }
//                    while (left_tuned_plan.size() < right_tuned_plan.size())
//                    {
//                        left_tuned_plan.copy_onto_back(left_tuned_plan.back().control, simulation::simulation_step);
//                    }
//                }
//                else if (right_steps > left_steps)
//                {
////                    PRX_PRINT("RIGHT LARGER!", PRX_TEXT_MAGENTA);
//                    if (prm_query->max_velocity_bias && goal_y != prm_query->coordination_goal_state->at(1) )
//                        edge_dist += (right_steps - left_steps);
//                    max_steps = right_untuned_plan.size() - 1;
//                    right_tuned_plan = right_untuned_plan;
//                    right_tuned_plan.pop_front();
//                    if (!get_tuned_plan(prm_query->left_state_space, prm_query->left_control_space, max_steps, left_untuned_plan, left_tuned_plan))
//                    {
//                        left_tuned_plan = left_untuned_plan;
//                        while (left_tuned_plan.size() < right_tuned_plan.size())
//                        {
//                            left_tuned_plan.copy_onto_back(left_tuned_plan.back().control, simulation::simulation_step);
//                        }
////                        return false;
//                    }
//                    while (right_tuned_plan.size() < left_tuned_plan.size())
//                    {
//                        right_tuned_plan.copy_onto_back(right_tuned_plan.back().control, simulation::simulation_step);
//                    }
//                }
//                else
//                {
////                    PRX_PRINT("WE ARE WINNER HAHAHA!", PRX_TEXT_MAGENTA);
//                    left_tuned_plan = left_untuned_plan;
//                    if (left_tuned_plan.size() > 1)
//                        left_tuned_plan.pop_front();
//                    right_tuned_plan = right_untuned_plan;
//                    if (right_tuned_plan.size() > 1)
//                        right_tuned_plan.pop_front();
////                    while (right_tuned_plan.size() < left_tuned_plan.size())
////                    {
////                        right_tuned_plan.copy_onto_back(right_tuned_plan.back().control, simulation::simulation_step);
////                    }
////                    while (left_tuned_plan.size() < right_tuned_plan.size())
////                    {
////                        left_tuned_plan.copy_onto_back(left_tuned_plan.back().control, simulation::simulation_step);
////                    }
//                }
//                
//                PRX_ASSERT(left_tuned_plan.size() == right_tuned_plan.size());
//                combine_tuned_plans(left_tuned_plan,right_tuned_plan,plan);
//                
//                edge_dist = plan.size();
//                
//                if (generate_trajectory)
//                {
////                    PRX_PRINT("Generating left arm traj", PRX_TEXT_BLUE);
//                    get_armcup_path(prm_query->left_arm, left_start, left_tuned_plan, prm_query->left_cup_safe_state, prm_query->left_state_space, prm_query->left_cup_space, prm_query->left_armcup_space, prm_query->left_armcup_path);
////                    PRX_PRINT("Generating right arm traj", PRX_TEXT_RED);
//                    get_armcup_path(prm_query->right_arm, right_start, right_tuned_plan, prm_query->right_cup_safe_state, prm_query->right_state_space, prm_query->right_cup_space, prm_query->right_armcup_space, prm_query->right_armcup_path);
//
//                    PRX_ASSERT(prm_query->left_armcup_path.size() == prm_query->right_armcup_path.size());
//                    combine_armcup_paths(prm_query->left_armcup_path,prm_query->right_armcup_path,prm_query->left_armcup_space, prm_query->right_armcup_space,traj);
//                    
//                }
//                
//                return (plan.size() > 0);
//
//            }
//            void coordination_prm_t::get_armcup_path(baxter::baxter_arm_t* arm, sim::state_t* arm_start_state, sim::plan_t& arm_plan, const sim::state_t* cup_safe_state, const util::space_t* arm_state_space, 
//                                                     const util::space_t* cup_space, const util::space_t* armcup_space,  sim::trajectory_t& armcup_path)
//            {
//                armcup_path.clear();
//                arm_plan.copy_onto_front(arm_start_state, simulation::simulation_step);
//                sim::state_t* armcup_point = armcup_space->alloc_point();
//                
//                double arm_open = 0.000000;
//                double arm_closed = 1.00000;
//                
//                config_t object_config;
//                std::vector<double> config_vector;
//                vector_t returned_pos;
//                quaternion_t yn90_orient, returned_quat;
//                yn90_orient.set(0, sin(-PRX_HALF_PI / 2), 0, cos(PRX_HALF_PI / 2));
//                
//                unsigned grasp_control_index = arm_state_space->get_dimension() - 1; // grasp control index should be the last spot...
//                for (unsigned i = 0; i < arm_plan.size(); i ++)
//                {
//                    arm_state_space->copy_from_point(arm_plan[i].control); // COPYING CONTROLS INTO STATE?! BAAAAAADDDD.... ok for now.
////                    PRX_PRINT("ARM STATE: " << arm_state_space->print_memory(5), PRX_TEXT_CYAN);
//                    if (arm_plan[i].control->at(grasp_control_index) == arm_open)
//                    {
//                        cup_space->copy_from_point(cup_safe_state);
//                    }
//                    else if (arm_plan[i].control->at(grasp_control_index) == arm_closed)
//                    {
//                    // TODO: Check arm->get_end_effector_offset_configuration alternative
////                        arm->get_end_effector_offset_configuration(object_config,arm_plan[i].control , 0.0,0.0,0.05);
//                        
////                        PRX_PRINT("PRE Retrieved config: " << object_config.print(), PRX_TEXT_LIGHTGRAY);
//                        object_config.get(returned_pos, returned_quat);
//                        returned_quat *= yn90_orient;
////                        PRX_PRINT("POST Retrieved config: " << object_config.print(), PRX_TEXT_CYAN);
//                        object_config.set_orientation(returned_quat);
////                        PRX_PRINT("POSTPOSTPOST Retrieved config: " << object_config.print(), PRX_TEXT_MAGENTA);
//                        object_config.copy_to_vector(config_vector);
//                        cup_space->set_from_vector(config_vector);
//                    }
//                    else
//                    {
//                        PRX_FATAL_S("Something bad happened- apparently we are in a state of halfway grasp???");
//                    }
////                    PRX_PRINT("CUP STATE: " << armcup_space->print_memory(5), PRX_TEXT_MAGENTA);
//                    armcup_space->copy_to_point(armcup_point);
//                    armcup_path.copy_onto_back(armcup_point);
//                }
//                
//                armcup_space->free_point(armcup_point);
//                arm_plan.pop_front();
//            }
//            
//            
//            void coordination_prm_t::combine_armcup_paths(sim::trajectory_t& left_armcup_path, sim::trajectory_t& right_armcup_path, 
//                                                 const util::space_t* left_armcup_space, const util::space_t* right_armcup_space, sim::trajectory_t& combined_path)
//            {
//                combined_path.clear();
//                
//                sim::state_t* combined_point = full_arm_state_space->alloc_point();
//                
//                for(unsigned i = 0; i < left_armcup_path.size(); i++)
//                {
//                    left_armcup_space->copy_from_point(left_armcup_path[i]);
//                    right_armcup_space->copy_from_point(right_armcup_path[i]);
//                    full_arm_state_space->copy_to_point(combined_point);
//                    combined_path.copy_onto_back(combined_point);
//                }
//                
//                full_arm_state_space->free_point(combined_point);
//                
//            }
//            
//            void coordination_prm_t::combine_tuned_plans(const plan_t& left_plan, const plan_t& right_plan, plan_t& combined_plan)
//            {
//                combined_plan.clear();
//                
//                /** Create full plan */                
//                control_t* new_control = full_arm_control_space->alloc_point();
//                for(unsigned i = 0; i < left_plan.size(); i++)
//                {
//                    double dur = left_plan[i].duration;
//                    prm_query->left_control_space->copy_from_point(left_plan[i].control);
//                    prm_query->right_control_space->copy_from_point(right_plan[i].control);
//                    full_arm_control_space->copy_to_point(new_control);
//                    combined_plan.copy_onto_back(new_control,dur);
//                                        
//                }
//                
//                full_arm_control_space->free_point(new_control);
//            }
//            
//            bool coordination_prm_t::get_tuned_plan(const space_t* arm_state_space, const space_t* arm_control_space, double total_steps, plan_t& untuned_plan, plan_t& tuned_plan)
//            {
//                tuned_plan.clear();
//                
//                /** Calculate average distance in the plan */
//                double average_distance = 0.0;
//                std::vector<double> distances;
//                for (unsigned i = 0; i < untuned_plan.size()-1; i++)
//                {
//                    double dist = arm_control_space->distance(untuned_plan[i].control, untuned_plan[i+1].control);
////                    PRX_WARN_S ("Dist: " << dist);
//                    average_distance += dist;
//                    distances.push_back(dist);
//                }
//                if (untuned_plan.size() > 1)
//                {
//                    average_distance /= (untuned_plan.size()-1);
//                }
////                PRX_PRINT("Average distance: " << average_distance, PRX_TEXT_CYAN);
//                /** Calculate the real distance */
//                double true_steps = total_steps;
//                double real_distance= 0.0;
//                double threshold_distance = (0.8*average_distance);
//                for (unsigned i = 0; i < distances.size(); i++)
//                {
//                    if (distances[i] < threshold_distance)
//                    {
//                        true_steps-= 1.0;
//                        if (distances[i] < PRX_ZERO_CHECK)
//                        {
//                            true_steps-= 1.0;
//                        }
//                    }
//                    else
//                    {
//                        real_distance += distances[i];
//                    }
//                }
//                
////                PRX_PRINT("True steps: " << true_steps << " vs total_steps: " << total_steps, PRX_TEXT_LIGHTGRAY);
//                double rescaled_distance = 0.0;
//                if (true_steps >= 1.0)
//                {
//                    rescaled_distance = real_distance / (true_steps);
//                }
//                if (rescaled_distance > max_step)
//                {
//                    PRX_ERROR_S (" OH HELLLLL NO " );
//                    return false;
//                }
//                
////                PRX_PRINT("New max step: " << rescaled_distance << " and untuned plan size: " << untuned_plan.size(), PRX_TEXT_BROWN);
//                
//                if (rescaled_distance < PRX_ZERO_CHECK || std::fabs(max_step - rescaled_distance)  < PRX_ZERO_CHECK)
//                {
////                    PRX_PRINT("Rescaled distance is fine, copying plan)", PRX_TEXT_CYAN);
//                    if (total_steps <= PRX_ZERO_CHECK)
//                        total_steps = 1.0;
//                    while (tuned_plan.size() < total_steps)
//                    {
//                        tuned_plan.copy_onto_back(untuned_plan.back().control,  1 * simulation::simulation_step);
//                    }
//                    return true;
//                }
//                else
//                {
//                    bool copy_back = true;
//                    control_t* new_control = arm_control_space->alloc_point();
//                    control_t* interpolated_control = arm_control_space->alloc_point();
//                    control_t* current, *next;
//                    double remainder = 0;
//                    unsigned current_index;
//                    double current_distance = 0.0;
//                    for(current_index = 0; current_index < untuned_plan.size()-1; current_index++)
//                    {
//                        copy_back = true;
//                        current = untuned_plan[current_index].control;
//                        next = untuned_plan[current_index+1].control;
//                        PRX_ASSERT(remainder > -1*PRX_ZERO_CHECK);
//
//                        current_distance = arm_control_space->distance(current,next);
//                        if (current_distance < threshold_distance)
//                        {
//                            tuned_plan.copy_onto_back(current, 1 * simulation::simulation_step);
//                            /** If distance is 0, we most likely have a grasp*/
//                            if (current_distance < PRX_DISTANCE_CHECK)
//                            {
//                                tuned_plan.copy_onto_back(next, 1 * simulation::simulation_step);
//                                current_index++;
//                                copy_back = false;
//                            }
//                            remainder = 0.0;
//                            current_distance = 0.0;
//                        }
//                        else
//                        {
//                            bool got_em = true;
////                            while (current_distance < remainder)
////                            {
////                                PRX_ERROR_S ("SHIEEET: " << current_index << " with dist: " << current_distance);
////                                current_index++;
////                                got_em = false;
////                                if (current_index < untuned_plan.size()-1)
////                                {
////                                    next = untuned_plan[current_index+1].control;
////                                    current_distance = arm_control_space->distance(current,next);
////                                    got_em = true;
////                                }
////                                else
////                                {
////                                    tuned_plan.copy_onto_back(untuned_plan.back().control, 1 * simulation::simulation_step);
////                                    got_em = false;
////                                    break;
////                                }
////                            }
//                            if (current_distance < remainder)
//                            {
//                                PRX_WARN_S ("Current distance is less than remaining");
//                                tuned_plan.copy_onto_back(current, 1 * simulation::simulation_step);
//                                remainder = 0.0;
//                                got_em = false;
//                            }
//
//                            if (got_em)
//                            {
//                                double temp_max_distance = 0.0;
//                                if (remainder > PRX_ZERO_CHECK)
//                                {
////                                    PRX_PRINT("Remainder: " << remainder, PRX_TEXT_GREEN);
//                                    arm_control_space->interpolate(current, next, (remainder/current_distance), interpolated_control);
////                                    PRX_PRINT("CURRENT: " << arm_control_space->print_point(current), PRX_TEXT_GREEN);
////                                    PRX_PRINT("NEXT: " << arm_control_space->print_point(next), PRX_TEXT_RED);
////                                    PRX_PRINT("INTERP: " << arm_control_space->print_point(interpolated_control), PRX_TEXT_BLUE);
//                                    tuned_plan.copy_onto_back(interpolated_control, 1 * simulation::simulation_step);
//                                    temp_max_distance += remainder;
//                                }
////                                else
////                                {
////                                    arm_control_space->copy_point(interpolated_control, current);
////                                }
//
//
//                                temp_max_distance += rescaled_distance;
//
//                                while(temp_max_distance <= current_distance)
//                                {
//                                    double t = (temp_max_distance/current_distance);
//                                    arm_control_space->interpolate(current, next, t, new_control);
//                                    tuned_plan.copy_onto_back(new_control, 1 * simulation::simulation_step );
//                                    temp_max_distance += rescaled_distance;
//                                }
//
//                                remainder = temp_max_distance - current_distance;
//                            }
//                        }
//                    }
//                    if (current_distance > remainder)
//                    {
//                        double temp_max_distance = 0.0;
//    //                    PRX_WARN_S("AFTER LOOP!");
//                        if (remainder > PRX_ZERO_CHECK )
//                        {
//    //                        PRX_PRINT("Remainder: " << remainder, PRX_TEXT_GREEN);
//                            arm_control_space->interpolate(current, next, (remainder/current_distance), interpolated_control);
//    //                        PRX_PRINT("CURRENT: " << arm_control_space->print_point(current), PRX_TEXT_GREEN);
//    //                        PRX_PRINT("NEXT: " << arm_control_space->print_point(next), PRX_TEXT_RED);
//    //                        PRX_PRINT("INTERP: " << arm_control_space->print_point(interpolated_control), PRX_TEXT_BLUE);
//                            tuned_plan.copy_onto_back(interpolated_control, 1 * simulation::simulation_step);
//                            temp_max_distance += remainder;
//                        }
//
//                        temp_max_distance += rescaled_distance;
//
//                        while(temp_max_distance <= current_distance)
//                        {
//                            double t = (temp_max_distance/current_distance);
//                            arm_control_space->interpolate(current, next, t, new_control);
//                            tuned_plan.copy_onto_back(new_control, 1 * simulation::simulation_step );
//                            temp_max_distance += rescaled_distance;
//                        }
//                    }
//                    else if (current_distance > PRX_ZERO_CHECK)
//                    {
//                        tuned_plan.copy_onto_back(current, 1 * simulation::simulation_step );
//                    }
//                   
//                    /** This is to ensure we get the ungrasp state out of the plan*/
//                    if(copy_back)
//                    {
//                        tuned_plan.copy_onto_back(untuned_plan.back().control,  1 * simulation::simulation_step);
//                    }
//                    
////                    PRX_PRINT("Tuned plan: \n" << tuned_plan.print(4), PRX_TEXT_GREEN);
////                    PRX_PRINT("UNTUNED plan: \n" << untuned_plan.print(4), PRX_TEXT_BLUE);
//                    while (tuned_plan.size() < total_steps)
//                    {
////                        PRX_WARN_S("Fixing tuned plan. No. Tuned plan size: " << tuned_plan.size());
//                        tuned_plan.copy_onto_back(untuned_plan.back().control,  1 * simulation::simulation_step);
//                    }
//
////                    PRX_PRINT("Tuned plan size: " << tuned_plan.size() << " vs max steps: " << total_steps, PRX_TEXT_GREEN);
//                    
//                
////                    for (unsigned i = 0; i < tuned_plan.size()-1; i++)
////                    {
////                        double dist = arm_control_space->distance(tuned_plan[i].control, tuned_plan[i+1].control);
////                        PRX_WARN_S ("Dist: " << dist);
////                    }
////                    PRX_ASSERT(false);
//                    return true;
//
//    //                for(unsigned i = 0; i < untuned_plans.size()-1; i++)
//    //                {
//    //                    double dist = arm_control_space->distance(untuned_plan[i].control, untuned_plan[i+1].control);
//    //                    PRX_PRINT("Dist: " << dist << " vs: rescaled: " << rescaled_distance, PRX_TEXT_GREEN);
//    //                    tuned_plan.copy_onto_back(untuned_plan[i].control, simulation::simulation_step);
//    //                    while (dist > rescaled_distance)
//    //                    {
//    //                        
//    //                    }
//    //                    
//    //                }
//    //
//    //                for( double i = 1; i < total_steps; ++i )
//    //                {
//    //                    double t = i / total_steps;
//    //
//    //                    arm_state_space->interpolate(start, goal, t, interpolated_state);
//    //
//    ////                    duration = std::ceil(state_space->distance(prev_state, interpolated_state) / max_step);
//    //                    arm_state_space->copy_point(prev_state, interpolated_state);
//    //
//    //
//    //                    for( unsigned j = 0; j < arm_control_space->get_dimension(); j++ )
//    //                    {
//    //                        new_control->at(j) = interpolated_state->at(j);
//    //                    }
//    //                    tuned_plan.copy_onto_back(new_control, duration * simulation::simulation_step);
//    //                }
//    //
//    ////                duration = std::ceil(state_space->distance(prev_state, goal) / max_step);
//    //                for( unsigned j = 0; j < arm_control_space->get_dimension(); j++ )
//    //                {
//    //                    new_control->at(j) = goal->at(j);
//    //                }
//    //                tuned_plan.copy_onto_back(new_control, duration * simulation::simulation_step);
//    //                
//    //                arm_control_space->free_point(new_control);
//    //                arm_state_space->free_point(prev_state);
//    //                arm_state_space->free_point(interpolated_state);
//                }
//            }
//            
//            void coordination_prm_t::get_arm_start_and_goal(unsigned start_point, unsigned goal_point, plan_t* arm_plan, state_t* start, state_t* goal, plan_t& seed_plan  )
//            {
//                seed_plan.clear();
////                PRX_WARN_S ("PREEEE START INDEX: " << start_point << ", GOAL INDEX: " << goal_point);
//                
//                if (start_point >= arm_plan->size())
//                    start_point = arm_plan->size()-1;
//                if (goal_point >= arm_plan->size())
//                    goal_point = arm_plan->size()-1;
////                PRX_ERROR_S ("POSTTTT START INDEX: " << start_point << ", GOAL INDEX: " << goal_point);
//                
//                control_t* start_control = arm_plan->at(start_point).control;
//                prm_query->left_state_space->copy_point(start, start_control);
//
//
//                control_t* goal_control = arm_plan->at(goal_point).control;
//                prm_query->left_state_space->copy_point(goal, goal_control);
//
//                
//                /** CHECK PLAN */
//                for (unsigned i = 0; i < arm_plan->size() - 1; i++)
//                {
//                    double dist = prm_query->left_control_space->distance(arm_plan->at(i).control,arm_plan->at(i+1).control);
//                    if (dist > max_step)
//                    {
//                        PRX_ERROR_S("DIST: " << dist);
//                        PRX_PRINT("point1: " << prm_query->left_control_space->print_point(arm_plan->at(i).control), PRX_TEXT_CYAN);
//                        PRX_PRINT("point2: " << prm_query->left_control_space->print_point(arm_plan->at(i+1).control), PRX_TEXT_MAGENTA);
//                        PRX_ASSERT(false);
//                    }
//                }
////                PRX_WARN_S ("START INDEX: " << start_point << ", GOAL INDEX: " << goal_point);
//                int start_index = start_point;
//                int goal_index = goal_point;
//                if (start_index > goal_index)
//                {
//                    for(int i = goal_index; i <= start_index; i++ )
//                    {
//                        seed_plan.copy_onto_front(arm_plan->at(i).control,simulation::simulation_step );
//                    }
//                }
//                else if (start_index < goal_index)
//                {
//                    for(int i = start_index; i <= goal_index; i++ )
//                    {
//                        seed_plan.copy_onto_back(arm_plan->at(i).control,simulation::simulation_step );
//                    }
//                }
//                else
//                {
//                    seed_plan.copy_onto_back(arm_plan->at(goal_index).control,simulation::simulation_step );
//                }
//                
////                PRX_PRINT("Untuned plan: " << seed_plan.print() << "\n\n", PRX_TEXT_GREEN);
//            }
//
//        }        
//
//    }
//}