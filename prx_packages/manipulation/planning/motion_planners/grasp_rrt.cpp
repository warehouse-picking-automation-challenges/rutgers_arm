// /**
//  * @file grasp_rrt.cpp
//  *
//  * @copyright Software License Agreement (BSD License)
//  * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
//  * All Rights Reserved.
//  * For a full description see the file named LICENSE.
//  *
//  * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
//  *
//  * Email: pracsys@googlegroups.com
//  */

// #include "planning/motion_planners/grasp_rrt.hpp"
// #include "prx/planning/motion_planners/rrt/rrt_statistics.hpp"

// #include "prx/utilities/definitions/sys_clock.hpp"
// #include "prx/utilities/definitions/string_manip.hpp"
// #include "prx/utilities/definitions/random.hpp"
// #include "prx/utilities/distance_metrics/distance_metric.hpp"
// #include "prx/planning/modules/samplers/sampler.hpp"
// #include "prx/utilities/goals/radial_goal_region.hpp"
// #include "prx/planning/modules/local_planners/local_planner.hpp"
// #include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
// #include "prx/planning/modules/validity_checkers/validity_checker.hpp"
// #include "prx/planning/problem_specifications/motion_planning_specification.hpp"
// #include "prx/planning/queries/motion_planning_query.hpp"
// #include "prx/planning/communication/visualization_comm.hpp"
// #include "prx/planning/motion_planners/motion_planner.hpp"
// #include "planning/modules/grasp_evaluator.hpp"

// #include "utilities/distance_metrics/motion_planning_task_space_node.hpp"
// #include "utilities/distance_metrics/task_space_metric.hpp"


// #include <pluginlib/class_list_macros.h>
// #include <boost/graph/subgraph.hpp>

// PLUGINLIB_EXPORT_CLASS( prx::packages::manipulation::grasp_rrt_t, prx::plan::planner_t)

// namespace prx
// {
//     using namespace util;
//     using namespace sim;
//     using namespace plan;
//     namespace packages
//     {
//         namespace manipulation
//         {

//             grasp_rrt_t::grasp_rrt_t()
//             {
//                 iteration_count = 0;
//                 color_map = {"red","blue","green","yellow","black"};
//                 solution_number = 0;
//                 state_space = control_space = NULL;
//                 satisfied_goal_vertex.clear();
//                 max_edges = 0;
//                 max_goals = 0;
//                 object_pose = NULL;
//                 std::vector<double*> state_dim = {&_x,&_y,&_z,&_qx,&_qy,&_qz,&_qw};
//                 end_effector_space = new space_t("SE3", state_dim);
//                 reached_grasp_goal = false;
//             }

//             grasp_rrt_t::~grasp_rrt_t()
//             {
//                 state_space->free_point(sample_point);
//                 //    delete statistics;
//                 for( unsigned i = 0; i < max_points; i++ )
//                 {
//                     state_space->free_point(pre_alloced_points[i]);
//                 }
//                 reset();
//             }

//             void grasp_rrt_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
//             {
//                 motion_planner_t::init(reader, template_reader);

//                 visualize_tree = parameters::get_attribute_as<bool>("visualize_tree", reader, template_reader, true);
//                 visualize_solution = parameters::get_attribute_as<bool>("visualize_solution", reader, template_reader, true);
//                 visualization_tree_name = parameters::get_attribute_as<std::string > ("visualization_tree_name", reader, template_reader, ros::this_node::getName() + "/rrt/graph");
//                 visualization_solution_name = parameters::get_attribute_as<std::string > ("visualization_solution_name", reader, template_reader, ros::this_node::getName() + "/rrt/solutions");
//                 graph_color = parameters::get_attribute_as<std::string > ("graph_color", reader, template_reader, "white");
//                 solution_color = parameters::get_attribute_as<std::string > ("solution_color", reader, template_reader, "green");
//                 visualization_body = "";
//                 if( parameters::has_attribute("visualization_bodies", reader, template_reader) )
//                     visualization_body = (parameters::get_attribute_as<std::vector<std::string> >("visualization_bodies", reader, template_reader))[0];
//                 visualization_body = parameters::get_attribute_as<std::string > ("visualization_body", reader, template_reader, visualization_body);
//                 radius_solution = parameters::get_attribute_as<bool>("radius_solution", reader, template_reader, false);
//                 collision_checking = parameters::get_attribute_as<bool>("collision_checking", reader, template_reader, true);
//                 max_points = parameters::get_attribute_as<int>("max_points", reader, template_reader, 100000);
//                 goal_bias_rate = parameters::get_attribute_as<double>("goal_bias_rate", reader, template_reader, 0.0);
//                 use_boost_random = parameters::get_attribute_as<bool>("use_boost_random", reader, template_reader, false);
//             }

//             void grasp_rrt_t::reset()
//             {
//                 if( metric != NULL )
//                 {
//                     if( metric->get_nr_points() != 0 )
//                     {
//                         metric->clear();
//                     }
//                 }
//                 tree.clear();
//                 iteration_count = 0;
//                 point_number = 0;
//                 satisfied_goal_vertex.clear();
//                 states_to_check.clear();
//                 input_specification->get_stopping_criterion()->reset();
//                 start_vertices.clear();
//             }

//             void grasp_rrt_t::setup()
//             {
//                 states_to_check.clear();
//                 start_vertices.clear();
//                 trajectory.link_space(state_space);
//                 trajectory.resize(500);
//                 tree.pre_alloc_memory<motion_planning_task_space_node_t<rrt_node_t> , rrt_edge_t > (max_points);
//                 // states_to_check.push_back(state_space->clone_point(input_specification->get_seeds()[0]));
//                 sample_point = state_space->alloc_point();
//                 clock.reset();
//                 if( pre_alloced_points.empty() )
//                 {
//                     for( unsigned i = 0; i < max_points; i++ )
//                     {
//                         pre_alloced_points.push_back(state_space->alloc_point());
//                     }
//                 }
//                 point_number = 0;
//                 input_specification->get_stopping_criterion()->reset();
//                 tmp_constraint = input_specification->validity_checker->alloc_constraint();


//                 char* w = std::getenv("PRACSYS_PATH");
//                 std::string dir(w);
//                 dir += ("/prx_output/");
//                 int hz = 1.0/simulation::simulation_step;
//                 file_name = dir + int_to_str(hz)+"Hz.txt";

//             }

//             void grasp_rrt_t::link_specification(specification_t* new_spec)
//             {
//                 const space_t* old_state_space = state_space;
//                 const space_t* old_control_space = control_space;

//                 motion_planner_t::link_specification( new_spec );

//                 //Now, if we've changed state space
//                 if( old_state_space != NULL && old_state_space != state_space )
//                 {
//                     //Free the old points, and allocate new ones
//                     for( unsigned i=0; i<pre_alloced_points.size(); ++i )
//                     {
//                         old_state_space->free_point(pre_alloced_points[i]);
//                         pre_alloced_points[i] = state_space->alloc_point();
//                     }
//                 }

//                 grasp_rrt_specification = dynamic_cast< grasp_rrt_specification_t* > (new_spec);
//                 PRX_ASSERT(grasp_rrt_specification != NULL);
//             }

//             void grasp_rrt_t::link_query(query_t* new_query)
//             {
//                 motion_planner_t::link_query(new_query);
//                 reset();
//                 if(input_query->search_mode == LAZY_SEARCH)
//                 {
//                     PRX_ERROR_S("RRT is not supposed to be invoked with Lazy Search. Reverting the mode to STANDARD_SEARCH");
//                     input_query->search_mode = STANDARD_SEARCH;
//                 }
//             }

//             void grasp_rrt_t::random_sample()
//             {
                
//             }

//             state_t* grasp_rrt_t::config_to_state(config_t conf)
//             {
//                 std::vector<double> conf_vec;
//                 conf_vec.push_back(conf.get_position()[0]);
//                 conf_vec.push_back(conf.get_position()[1]);
//                 conf_vec.push_back(conf.get_position()[2]);
//                 conf_vec.push_back(conf.get_orientation()[0]);
//                 conf_vec.push_back(conf.get_orientation()[1]);
//                 conf_vec.push_back(conf.get_orientation()[2]);
//                 conf_vec.push_back(conf.get_orientation()[3]);
//                 state_t* conf_point = end_effector_space->alloc_point();
//                 end_effector_space->set_from_vector(conf_vec, conf_point);
//                 return conf_point;
//             }

//             tree_vertex_index_t grasp_rrt_t::add_to_tree(state_t* point_to_add, bool start_node)
//             {
                
//                 tree_vertex_index_t v = tree.add_vertex<motion_planning_task_space_node_t<rrt_node_t>, rrt_edge_t > ();
                
//                 if(!start_node)
//                 {
//                     state_space->copy_point(pre_alloced_points[point_number], point_to_add);
//                     tree[v]->point = pre_alloced_points[point_number];
//                 }
//                 else
//                 {
//                     tree[v]->point = state_space->clone_point(point_to_add);
//                 }


//                 config_t ee_conf;
//                 grasp_rrt_specification->grasp_evaluator->FK(point_to_add, ee_conf);

//                 task_space_node_t* ts_node = new task_space_node_t(state_space, point_to_add, config_to_state(ee_conf), ee_conf);

//                 //motion_planning_task_space_node_t<rrt_node_t>* msts_node = dynamic_cast< motion_planning_task_space_node_t<rrt_node_t>* >(
//                 motion_planning_task_space_node_t<rrt_node_t>* msts_node = tree.get_vertex_as< motion_planning_task_space_node_t<rrt_node_t> >(v);
//                 if(msts_node != NULL)
//                 {
//                     msts_node->task_space_node = ts_node;
//                     ts_node->motion_planning_node = msts_node;
//                     metric->add_point(msts_node->task_space_node);
//                     return v;
//                 }
//                 else
//                 {
//                     PRX_ERROR_S("Shit be broke");
//                 }



//                 // grasp_rrt_specification->start_state_metrics[0]->add_point(tree[v]);
//             }

//             void grasp_rrt_t::increment_point_number()
//             {
//                 point_number++;
//                 //Increase the preallocated buffer
//                 if(point_number == max_points)
//                 {
//                     for(unsigned i=0;i<max_points;i++)
//                     {
//                         pre_alloced_points.push_back(state_space->alloc_point());
//                     }
//                     max_points*=2;
//                 }
//             }

//             bool grasp_rrt_t::check_path_for_collisions_and_constraints(constraints_t* constraints, trajectory_t& path_to_evaluate)
//             {
//                 //Check if the path is valid and whether any of the constraints on the path are valid
//                 if(input_query->search_mode==STANDARD_SEARCH)
//                     return validity_checker->validate_and_generate_constraints(constraints, path_to_evaluate) && !constraints->has_intersection(input_query->active_constraints);
//                 if(input_query->search_mode==TRUSTED_MCR || input_query->search_mode==UNTRUSTED_MCR)
//                     return validity_checker->validate_and_generate_constraints(constraints, path_to_evaluate);
//                 PRX_FATAL_S("Running RRT in an invalid search mode");
//             }

//             std::pair<bool, tree_vertex_index_t> grasp_rrt_t::add_node(space_point_t* n_state, state_t* jk_node, plan_t jk_plan)
//             {
//                 //get the nearest state on the tree, to the randomly sampled point
//                 tree_vertex_index_t nearest = nearest_vertex(n_state);
//                 tree_vertex_index_t v;
//                 //If the sampled point is already in the tree
//                 if( state_space->equal_points(n_state,tree[nearest]->point))
//                 {
//                     return std::make_pair(false, nearest);
//                 }

//                 //propagate toward the sampled point
//                 plan_t plan;
//                 plan.link_control_space(control_space);
                
//                 if(jk_node == NULL)
//                 {
//                     local_planner->steer(tree[nearest]->point, n_state, plan, trajectory, false);
//                     prune_trajectory(plan,trajectory);
//                 }
//                 else
//                 {
//                     local_planner->propagate(jk_node, jk_plan, trajectory);
//                     // prune_trajectory(jk_plan,trajectory);
//                     plan = jk_plan;
//                     nearest = nearest_vertex(jk_node);

//                     // PRX_ASSERT(state_space->distance(n_state, trajectory [trajectory.size() - 1]) < 0.001);
//                 }
//                 //Get pruned connected state
//                 state_space->copy_point(n_state, trajectory[trajectory.size() - 1]);
//                 //Check collisions over the trajectory
//                 // if( !collision_checking || (validity_checker->is_valid(trajectory) && trajectory.size() > 1) )
//                 if( !collision_checking || ( check_path_for_collisions_and_constraints(tmp_constraint, trajectory) && trajectory.size() > 1) )
//                 {
//                     //*******************Add a new vertex ******************
//                     // v = tree.add_vertex<rrt_node_t, rrt_edge_t > ();
//                     // state_space->copy_point(pre_alloced_points[point_number], n_state);
//                     // tree[v]->point = pre_alloced_points[point_number];
//                     // metric->add_point(tree[v]);

//                     // grasp_rrt_specification->start_state_metrics[0]->add_point(tree[v]);

//                     v = add_to_tree(n_state);
//                     tree.get_vertex_as<motion_planning_task_space_node_t<rrt_node_t> >(v)->cost = tree.get_vertex_as<motion_planning_task_space_node_t<rrt_node_t> >(nearest)->cost;
//                     tree.get_vertex_as<motion_planning_task_space_node_t<rrt_node_t> >(v)->cost += validity_checker->trajectory_cost(trajectory);
//                     state_space->copy_point(states_to_check[0], tree[v]->point);
//                     //******************Added a new vertex *****************

//                     //********************Add a new edge *******************
//                     tree_edge_index_t e = tree.add_edge<rrt_edge_t > (nearest, v);
//                     get_edge(e)->plan = plan;
//                     if( visualize_tree )
//                     {
//                         get_edge(e)->trajectory = trajectory;
//                     }
//                     //*******************Added a new edge ******************

//                     increment_point_number();
//                     return std::make_pair(true, v);

//                 }
//                 return std::make_pair(false, v);
//             }


//             bool grasp_rrt_t::execute()
//             {
//                 //Do nothing since there is no preprocessing for rrt planners
//                 throw stopping_criteria_t::stopping_criteria_satisfied(" Stopping criteria is satisfied ");
//                 return true;
//             }
                

//             void grasp_rrt_t::step()
//             {                
//                 iteration_count++;
//                 //Sample a point at random
//                 bool goal_biasing = false;
//                 plan_t jk_approach(control_space);
//                 state_t* jk_near_node = NULL;
//                 bool grasp_goal = false;
//                 if( roll_weighted_die({goal_bias_rate, (1 - goal_bias_rate)}, use_boost_random) == 0 )
//                 {
//                     // This will ONLY check the first goal.
//                     // TODO: Multiple goals?
//                     // unsigned num_of_goals = 1;
//                     // std::vector<state_t*> goal_points = input_query->get_goal()->get_goal_points(num_of_goals);
//                     // if(num_of_goals == 1)
//                     // {
//                     //     state_space->copy_point(sample_point, input_query->get_goal()->get_first_goal_point());
//                     // }
//                     // else if (num_of_goals > 1)
//                     // {
//                     //     state_space->copy_point(sample_point, goal_points[uniform_int_random(0,num_of_goals-1)]);
//                     // }
//                     // else
//                     // {
//                     //     PRX_ERROR_S("No Goal Points to sample from!");
//                     //     sampler->sample(state_space, sample_point);
//                     // }
//                     goal_biasing = true;
//                     std::vector<state_t*> closest_states;
//                     if(object_pose == NULL)
//                     {
//                         object_pose = grasp_rrt_specification->object_to_grasp->get_state_space()->alloc_point();
//                     }
//                     else
//                     {
//                         grasp_rrt_specification->object_to_grasp->get_state_space()->copy_to_point(object_pose);
//                     }

                    
//                     grasp_rrt_specification->grasp_evaluator->states_closest_to_grasp(closest_states, metric, object_pose, 1);
//                     if(closest_states.size()>0)
//                     {   
//                         jk_near_node = closest_states[0];
//                         grasp_goal = grasp_rrt_specification->grasp_evaluator->grasp_extension(jk_approach, sample_point, jk_near_node, object_pose);

//                         // if(grasp_goal)
//                         // {
//                         //     trajectory_t trajectory(state_space);
//                         //     local_planner->propagate(jk_near_node, jk_approach, trajectory);

//                         //     PRX_ASSERT(state_space->distance(sample_point, trajectory [trajectory.size() - 1]) < 0.001);
//                         // }
//                     }
//                     else
//                     {
//                         PRX_ERROR_S("No Grasp to sample from!");
//                         // sampler->sample(state_space, sample_point);
//                     }


                    
//                 }
//                 else
//                 {
//                         // Otherwise, our dice said to randomly sample
//                     sampler->sample(state_space, sample_point);
//                 }

//                 //Vertex index of added vertex
//                 tree_vertex_index_t added_vertex;
//                 //Check whether the sampled point could be added to the tree
//                 bool add_check = false;;

//                 if(!goal_biasing)
//                 {
//                     boost::tie(add_check, added_vertex) = add_node(sample_point);
//                 }
//                 else
//                 {
//                     if(jk_approach.size()>0)
//                     {
//                         boost::tie(add_check, added_vertex) = add_node(sample_point, jk_near_node, jk_approach);
//                     }
//                 }

//                 //If a new node was added to the tree
//                 if(add_check)
//                 {
//                     //Check if the added point satisfies the goal
//                     //PRX_ASSERT(input_query->get_goal()->satisfied(states_to_check[0]) == input_query->get_goal()->satisfied(tree[added_vertex]->point));
//                     //PRX_DEBUG_COLOR(input_query->get_goal() << " : " << state_space->print_point(states_to_check[0], 3), PRX_TEXT_LIGHTGRAY);
//                     if(grasp_goal)
//                     {
//                         satisfied_goal_vertex.clear();
//                         satisfied_goal_vertex.push_back(added_vertex);
//                         PRX_PRINT("\nSatisfied the goal!", PRX_TEXT_LIGHTGRAY);
//                         reached_grasp_goal = true;
//                         throw stopping_criteria_t::stopping_criteria_satisfied(" Stopping criteria is satisfied ");
//                     }
                    
//                 }


//                 PRX_STATUS("RRT Step ["<<iteration_count<<"] ... "<<point_number, PRX_TEXT_BROWN);
//             }

//             bool grasp_rrt_t::succeeded() const
//             {
//                 if( input_specification->get_stopping_criterion()->satisfied() )
//                     return true;
//                 return false;
//             }

//             tree_vertex_index_t grasp_rrt_t::retrace_path(plan_t& plan, tree_vertex_index_t new_v, tree_vertex_index_t root_v)
//             {
//                 bool retraced_till_root = true;
//                 while( get_vertex(new_v)->get_parent() != new_v )
//                 {
//                     tree_edge_index_t e = tree.edge(get_vertex(new_v)->get_parent(), new_v);
//                     rrt_edge_t* edge = get_edge(e);

//                     plan_t hold_plan(edge->plan);
//                     hold_plan += plan;
//                     plan = hold_plan;

//                     //If finding a path to an intermediate node and not the root of the tree
//                     if(new_v == root_v)
//                     {   
//                         retraced_till_root = false;
//                         return new_v;
//                     }

//                     new_v = get_vertex(new_v)->get_parent();
//                 }

//                 //If passed in a root vertex but reached the root of the tree.
//                 if(retraced_till_root && start_vertex != root_v)
//                 {
//                     PRX_ERROR_S("Tried to retrace to a specified vertex but ended up retracing to the root of the tree.");
//                 }

//                 return new_v;
//             }

//             void grasp_rrt_t::resolve_query()
//             {
//                 reached_grasp_goal = false;

//                 solution_number++;
//                 //*******************Add the start vertex ******************
//                 start_vertices.clear();
//                 foreach(state_t* start_st, input_query->get_start_states())
//                 {
//                     if(input_specification->validity_checker->is_valid(start_st))
//                     {
//                         start_vertices.push_back(add_to_tree(start_st, true));
//                     }
//                 }
//                 start_vertex = start_vertices[0];
//                 // start_vertex = tree.add_vertex<rrt_node_t, rrt_edge_t > ();
//                 // tree[start_vertex]->point = state_space->clone_point(input_query->get_start_state());

//                 // PRX_WARN_S("Start state: " << state_space->print_point(tree[start_vertex]->point, 3));
//                 // if( !input_specification->validity_checker->is_valid( tree[start_vertex]->point ) )
//                 // {
//                 //     PRX_ERROR_S("RRT start state is NOT VALID: aborting!!");
//                 //     return;
//                 // }

//                 // grasp_rrt_specification->generate_new_metrics(start_node_vec);
//                 // metric->add_point(tree[start_vertex]);
//                 states_to_check.clear();
//                 states_to_check.push_back(state_space->clone_point(tree[start_vertex]->point));
//                 //******************Added the start vertex *****************

//                 //*******************Step till stopping criterion succeeds ******************
//                 try
//                 {
//                     do
//                     {
//                         step();

//                     }while(!succeeded() && ros::ok() && !reached_grasp_goal);
//                 }
//                 catch( stopping_criteria_t::stopping_criteria_satisfied e )                
//                 {
//                     //If no goal was satisfied during any of the steps
//                     if(satisfied_goal_vertex.empty())
//                     {
//                         PRX_DEBUG_COLOR("No Goal Was Satisfied.", PRX_TEXT_CYAN);
//                         input_query->found_solution = false;
//                     }
//                     else
//                     {   
//                         //Some goal was satisfied
//                         input_query->found_solution = true;
//                         PRX_ASSERT(satisfied_goal_vertex.size()==1);
//                         tree_vertex_index_t new_v = satisfied_goal_vertex[0];
//                         //Retrace a path from the satisfied goal to the start and store the resultant plan in the input query
//                         tree_vertex_index_t reached_start = retrace_path(input_query->plan, new_v, start_vertex);
//                         //Propagate and generate the path for the input query
//                         local_planner->propagate(tree[reached_start]->point, input_query->plan, input_query->path);
//                         //Generate constraints over the solution path (do not need the returned boolean)          
//                         check_path_for_collisions_and_constraints(tmp_constraint, input_query->path);
//                         //Find the constraints on the path that are currently active
//                         tmp_constraint->intersect(input_query->path_constraints, input_query->active_constraints);
//                         //Find the cost of the solution path
//                         input_query->solution_cost = input_query->path.length();
//                         PRX_PRINT("Found a solution of length "<<input_query->solution_cost<<" to the goal "<<state_space->print_point(tree[new_v]->point, 3), PRX_TEXT_CYAN);
//                         input_query->satisfied_start_state = state_space->clone_point(input_query->path[0]);
//                         input_query->satisfied_goal_state = state_space->clone_point(input_query->path.back());
//                         PRX_PRINT("Solution path: \n"<<input_query->path.print(3), PRX_TEXT_LIGHTGRAY);
//                         // PRX_PRINT("Solution path: "<<input_query->plan.print(3), PRX_TEXT_GREEN);
//                     }
//                 }

//                 //If the tree needs to be visualized then update_vis_info needs to be explicitly called here.
//                 if(visualize_tree || visualize_solution)
//                 {
//                     update_vis_info();
//                     ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->send_geometries();
//                 }

//             }

//             void grasp_rrt_t::update_vis_info() const
//             {
//                 if(tree.num_vertices()<=1)
//                 {
//                     //If the tree is empy then skip visualization
//                     return;
//                 }

//                 std::vector<geometry_info_t> geoms;
//                 std::vector<config_t> configs;
//                 hash_t<std::string, std::vector<double> > map_params;


//                 std::vector<double> params;
//                 PRX_WARN_S("Vis sol name: " << visualization_solution_name << " , tree name: " << visualization_tree_name);

//                 int count;
//                 if( visualize_tree )
//                 {
//                     PRX_WARN_S("Visualizing tree! " << visualization_body);
//                     count = 0;
//                     std::vector<std::string> system_names;
//                     system_names.push_back(visualization_body);

//                     foreach(tree_edge_t* e, tree.edges())
//                     {
//                         std::string name = visualization_tree_name + "/edge_" + int_to_str(count);
//                         params.clear();

//                         foreach(state_t* state, get_edge(e->get_index())->trajectory)
//                         {
//                             map_params.clear();
//                             ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->compute_configs(state, system_names, map_params);
//                             params.insert(params.end(), map_params[system_names[0]].begin(), map_params[system_names[0]].end());

//                         }

                        

//                         geoms.push_back(geometry_info_t(visualization_body, name, PRX_LINESTRIP, params, graph_color));
//                         configs.push_back(config_t());

//                         count++;
//                     }

//                     for(int i=count; i<=max_edges; ++i)
//                     {
//                         params.clear();
//                         params = {0,0,0,0,0,0};
//                         geoms.push_back(geometry_info_t(visualization_body, visualization_tree_name + "/edge_" + int_to_str(i), PRX_LINESTRIP, params, graph_color));
//                         configs.push_back(config_t());
//                     }

//                     //Update the maximum number edges in a generated tree to properly reset visualized edges.
//                     if(point_number>max_edges)
//                     {   
//                         max_edges = point_number;
//                     }

//                     unsigned num_goals;
//                     std::vector<state_t*> goal_points = input_query->get_goal()->get_goal_points(num_goals);
//                     goal_points.resize(num_goals);
//                     int goal_iter;
//                     for(goal_iter=0; goal_iter<goal_points.size(); ++goal_iter)
//                     {
//                         state_t* state = goal_points[goal_iter];
//                         std::string name = visualization_tree_name + "/goal_" + int_to_str(goal_iter);
//                         params.clear();
//                         params.push_back(0.01);
//                         ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->compute_configs(state, system_names, map_params);
//                         geoms.push_back(geometry_info_t(visualization_body, name, PRX_SPHERE, params, "red"));
//                         configs.push_back(config_t(vector_t(map_params[visualization_body][0], map_params[visualization_body][1], map_params[visualization_body][2]), quaternion_t()));

//                     }

//                     for( ; goal_iter<max_goals; ++goal_iter)
//                     {
//                         std::string name = visualization_tree_name + "/goal_" + int_to_str(goal_iter);
//                         geoms.push_back(geometry_info_t(visualization_body, name, PRX_SPHERE, params, "red"));
//                         configs.push_back(config_t(vector_t(0, 0, 0), quaternion_t()));
//                     }

//                     //Update the maximum number of goals in a generated tree to properly reset visualized goals.
//                     if(num_goals>max_goals)
//                     {
//                         max_goals = num_goals;
//                     }

//                     for(int start_iter=0; start_iter<input_query->get_start_states().size(); ++start_iter)
//                     {
//                         state_t* state = input_query->get_start_states()[start_iter];
//                         std::string name = visualization_tree_name + "/start_" + int_to_str(start_iter);
//                         params.clear();
//                         params.push_back(0.01);
//                         ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->compute_configs(state, system_names, map_params);
//                         geoms.push_back(geometry_info_t(visualization_body, name, PRX_SPHERE, params, "green"));
//                         configs.push_back(config_t(vector_t(map_params[visualization_body][0], map_params[visualization_body][1], map_params[visualization_body][2]), quaternion_t()));

//                     }

//                     ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->visualization_geom_map[visualization_tree_name] = geoms;
//                     ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->visualization_configs_map[visualization_tree_name] = configs;

//                     geoms.clear();
//                     configs.clear();
//                 }


//                 if( visualize_solution )
//                 {
//                     PRX_WARN_S("Visualizing solution! " << visualization_body);
//                     std::vector<std::string> system_names;
//                     system_names.push_back(visualization_body);
//                     if( input_query->path.size() < 2 )
//                     {
//                         params.clear();
//                         params = {0,0,0,0,0,0};

//                         std::string name = visualization_solution_name + "/" + visualization_body + "/path";//_" + int_to_str(solution_number);
//                         geoms.push_back(geometry_info_t(visualization_body, name, PRX_LINESTRIP, params, color_map[solution_number % color_map.size()]));
//                         configs.push_back(config_t());
//                         ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->visualization_geom_map[visualization_solution_name] = geoms;
//                         ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->visualization_configs_map[visualization_solution_name] = configs;
//                         geoms.clear();
//                         configs.clear();
//                     }
//                     else
//                     {
//                         params.clear();
//                         for( size_t i = 0; i < input_query->path.size(); i++ )
//                         {
//                             map_params.clear();
//                             //HACK HACK HACK
//                             // params.push_back(input_query->path[i]->at(0));
//                             // params.push_back(input_query->path[i]->at(1));
//                             // params.push_back(input_query->path[i]->at(2));
//                             ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->compute_configs(input_query->path[i], system_names, map_params);
//                             map_params[visualization_body][0]+=0.005;
//                             params.insert(params.end(), map_params[visualization_body].begin(), map_params[visualization_body].end());
//                             //params.back() += 3; //this is so the solution will be above
//                         }

//                         std::string name = visualization_solution_name + "/" + visualization_body + "/path";//_" + int_to_str(solution_number);
//                         geoms.push_back(geometry_info_t(visualization_body, name, PRX_LINESTRIP, params, color_map[solution_number % color_map.size()]));
//                         configs.push_back(config_t());
//                         ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->visualization_geom_map[visualization_solution_name] = geoms;
//                         ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->visualization_configs_map[visualization_solution_name] = configs;
//                         geoms.clear();
//                         configs.clear();
//                     }

//                 }
//             }

//             rrt_node_t* grasp_rrt_t::get_vertex(tree_vertex_index_t v) const
//             {
//                 return tree.get_vertex_as<motion_planning_task_space_node_t<rrt_node_t>  > (v);
//             }

//             rrt_edge_t* grasp_rrt_t::get_edge(tree_edge_index_t e) const
//             {
//                 return tree.get_edge_as<rrt_edge_t > (e);
//             }

//             tree_vertex_index_t grasp_rrt_t::nearest_vertex(const state_t* state) const
//             {
//                 abstract_node_t* inner_node = metric->single_query(state)->as< task_space_node_t >()->motion_planning_node;
//                 PRX_ASSERT(inner_node != NULL);
//                 rrt_node_t* inner_tree_node = dynamic_cast < rrt_node_t* >(inner_node);
//                 motion_planning_task_space_node_t<rrt_node_t>* msts_node = tree.get_vertex_as< motion_planning_task_space_node_t<rrt_node_t> >(inner_tree_node->get_index());

//                 // motion_planning_task_space_node_t<rrt_node_t>* msts_node = dynamic_cast< motion_planning_task_space_node_t<rrt_node_t>* > ( dynamic_cast< rrt_node_t* > ( inner_node ) );
//                 // PRX_ASSERT();
//                 PRX_ASSERT(msts_node != NULL);
//                 if(msts_node!=NULL)
//                 {
//                     return msts_node->get_index();
//                 }
//                 else
//                 {
//                     PRX_ERROR_S("Shit be broke too");
//                 }
//             }

//             bool grasp_rrt_t::serialize()
//             {
//                 PRX_WARN_S("Serialize in RRT is unimplemented");
//                 return false;
//             }

//             bool grasp_rrt_t::deserialize()
//             {
//                 PRX_WARN_S("Deserialize in RRT is unimplemented");
//                 return false;
//             }

//             const statistics_t* grasp_rrt_t::get_statistics()
//             {
//                 statistics = new rrt_statistics_t();
//                 statistics->as<rrt_statistics_t > ()->num_vertices = tree.num_vertices();
//                 statistics->as<rrt_statistics_t > ()->solution_quality = validity_checker->trajectory_cost(input_query->path);
//                 statistics->time = clock.measure();
//                 statistics->steps = iteration_count;

//                 return statistics;
//             }


//             void grasp_rrt_t::prune_trajectory(sim::plan_t& new_plan, sim::trajectory_t& new_trajectory, sim::state_t* new_end_state)
//             {
//                 unsigned num_states = new_trajectory.size();
//                 for(unsigned i=1;i<num_states;i++)
//                 {
//                     if(input_query->get_goal()->satisfied(new_trajectory[i]))
//                     {
//                         if(new_end_state!=NULL)
//                             state_space->copy_point(new_end_state,new_trajectory[i]);
//                         double duration = i*simulation::simulation_step;
//                         new_plan.trim(duration);
//                         new_trajectory.chop(i+1);
//                         break;
//                     }
//                 }
//             }

//         }
//     }

// }
