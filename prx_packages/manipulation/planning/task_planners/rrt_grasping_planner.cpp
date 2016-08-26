// // // /**
// // //  * @file rrt_grasping_planner.cpp
// // //  *
// // //  * @copyright Software License Agreement (BSD License)
// // //  * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
// // //  * All Rights Reserved.
// // //  * For a full description see the file named LICENSE.
// // //  *
// // //  * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
// // //  *
// // //  * Email: pracsys@googlegroups.com
// // //  */

// // // #include "planning/task_planners/rrt_grasping_planner.hpp"

// // // #include "prx/utilities/definitions/string_manip.hpp"
// // // #include "prx/utilities/definitions/random.hpp"
// // // #include "prx/utilities/math/configurations/config.hpp"
// // // #include "prx/utilities/statistics/statistics.hpp"
// // // #include "prx/utilities/distance_metrics/distance_metric.hpp"

// // // #include "prx/planning/motion_planners/motion_planner.hpp"
// // // #include "prx/planning/modules/samplers/sampler.hpp"
// // // #include "prx/planning/modules/validity_checkers/validity_checker.hpp"
// // // #include "prx/planning/modules/local_planners/local_planner.hpp"

// // // #include "planning/manipulation_world_model.hpp"
// // // #include "planning/queries/grasping_query.hpp"
// // // #include "planning/specifications/grasping_specification.hpp"
// // // #include "planning/modules/object_constraints_checker.hpp"
// // // #include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"

// // // #include <yaml-cpp/yaml.h>
// // // #include <boost/range/adaptor/map.hpp>
// // // #include <boost/filesystem.hpp>
// // // #include "boost/filesystem/operations.hpp"
// // // #include "boost/filesystem/path.hpp"
// // // #include <pluginlib/class_list_macros.h>

// // // #include "prx/planning/modules/validity_checkers/ignore_list_validity_checker.hpp"

// // // #include "prx/utilities/graph/abstract_node.hpp"
// // // #include <boost/assign/list_of.hpp>

// // // #include "prx/utilities/goals/multiple_goal_states.hpp"


// // // PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::rrt_grasping_planner_t, prx::plan::planner_t)

// // // namespace prx
// // // {
// // //     using namespace util;
// // //     using namespace sim;
// // //     using namespace plan;

// // //     namespace packages
// // //     {
// // //         namespace manipulation
// // //         {
            
// // //             rrt_grasping_planner_t::rrt_grasping_planner_t()// : grasping_planner_t()
// // //             {
// // //                 shelf_plane.resize(4);
// // //                 shelf_plane[0] = 100;
// // //                 shelf_plane[1] = 0;
// // //                 shelf_plane[2] = 0;
// // //                 shelf_plane[3] = 0;
// // //                 half_space_side = true;
// // //                 hs_goal = new half_space_goal_t();
// // //             }

// // //             rrt_grasping_planner_t::~rrt_grasping_planner_t()
// // //             {
// // //                 delete hs_goal;
// // //             }

// // //             void rrt_grasping_planner_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
// // //             {
// // //                 task_planner_t::init(reader, template_reader);
// // //                 grasping_planner_t::init(reader, template_reader);
// // //                 PRX_DEBUG_COLOR("Initializing RRT Grasping planner ...", PRX_TEXT_CYAN);

// // //                 if(parameters::has_attribute("shelf_plane", reader, NULL))
// // //                 {
// // //                     shelf_plane = parameters::get_attribute_as<std::vector<double> >("shelf_plane", reader, template_reader);
// // //                     half_space_side = parameters::get_attribute_as<bool>("half_space_side", reader, template_reader, true);
// // //                     if(shelf_plane.size()<4)
// // //                     {
// // //                         PRX_WARN_S("The shelf plane has to have all the parameters of the plane.");
// // //                     }
// // //                     else
// // //                     {
// // //                         PRX_PRINT("Half Space Set up with equation: "<<shelf_plane[0]<<"x + "<<shelf_plane[1]<<"y + "<<shelf_plane[2]<<"z + "<<shelf_plane[3]<<(half_space_side?" >= ":" <= ")<<"0.", PRX_TEXT_MAGENTA);
// // //                     }
// // //                 }
// // //                 else
// // //                 {
// // //                     PRX_WARN_S("Need to specify plane of the face of the shelf as [a,b,c,d], True/False where ax + by + cz + d <=(True) or >=(False) 0 ");
// // //                 }



// // //                 if( reader->has_attribute("planners") )
// // //                 {
// // //                     PRX_PRINT("RRT Planners under RRT Grasping Planner", PRX_TEXT_CYAN);
// // //                     std::string template_name;
// // //                     util::parameter_reader_t::reader_map_t planner_map = reader->get_map("planners");

// // //                     foreach(const util::parameter_reader_t::reader_map_t::value_type key_value, planner_map)
// // //                     {
// // //                         const util::parameter_reader_t* child_template_reader = NULL;
// // //                         std::string planner_name = key_value.first;

// // //                         if( key_value.second->has_attribute("template") )
// // //                         {
// // //                             template_name = key_value.second->get_attribute("template");
// // //                             child_template_reader = new util::parameter_reader_t(ros::this_node::getName() + "/" + template_name, global_storage);
// // //                         }
// // //                         motion_planning_specification_t* new_spec = dynamic_cast< motion_planning_specification_t* >( output_specifications[planner_name] );
                        
// // //                         //If we are initializing one of the motion planners
// // //                         if( new_spec != NULL )
// // //                         {

// // //                             query_t* new_query = dynamic_cast< motion_planning_query_t* >( output_queries[planner_name] );

// // //                             std::string planning_context_name = parameters::get_attribute_as<std::string > ("planning_context_name", key_value.second, child_template_reader);
// // //                             planner_info_map[planning_context_name] = new planner_info_t(space_names[planner_name], planning_context_name, planners[planner_name], new_spec, new_query);
// // //                             PRX_PRINT("Mapping planner "<<planner_name<<" to the context name "<<planning_context_name, PRX_TEXT_GREEN);
// // //                         }

// // //                         if( child_template_reader != NULL )
// // //                         {
// // //                             delete child_template_reader;
// // //                             child_template_reader = NULL;
// // //                         }

// // //                     }
// // //                 }
// // //             }

// // //             void rrt_grasping_planner_t::link_world_model(world_model_t * const model)
// // //             {
// // //                 task_planner_t::link_world_model(model);
// // //                 grasping_planner_t::link_world_model(model);

// // //                 hs_goal->setup(manipulation_model, shelf_plane, half_space_side);
                
// // //                 std::string old_context = manipulation_model->get_current_context();
// // //                 foreach(planner_info_t* planner_info, planner_info_map | boost::adaptors::map_values)
// // //                 {
// // //                     // manipulation_model->use_context(planner_info->construction_context_name);
// // //                     manipulation_model->use_context(planner_info->planning_context_name);
// // //                     planner_info->setup(manipulation_model);
// // //                     // planner_info->planner->setup();
// // //                     // manipulation_model->use_context(planner_info->planning_context_name);
// // //                 }
// // //                 manipulation_model->use_context(old_context);
// // //             }

// // //             void rrt_grasping_planner_t::link_query(query_t* new_query)
// // //             {
// // //                 grasping_planner_t::link_query(new_query);
// // //                 current_manipulation_context_info = manipulation_model->get_current_manipulation_info();
// // //                 active_planner = planner_info_map[current_manipulation_context_info->arm_context_name];
// // //                 parent_planner = parent_planner_info_map[current_manipulation_context_info->arm_context_name];
// // //                 PRX_PRINT("Link Query:: context name - "<<current_manipulation_context_info->arm_context_name<<(active_planner==NULL?" but could not find a planner.":" and found a planner."), PRX_TEXT_MAGENTA);
// // //             }

// // //             void rrt_grasping_planner_t::update_plane_specs(std::vector<double> new_shelf_plane, bool new_half_space_side)
// // //             {
// // //                 shelf_plane = new_shelf_plane;
// // //                 half_space_side = new_half_space_side;
// // //             }

// // //             bool rrt_grasping_planner_t::compute_rrt_solution(plan_t& plan, state_t* satisifed_goal, planner_info_t* current_planner, state_t* start, const space_t* start_space, std::vector<state_t* >& roadmap_neighbors)
// // //             {
// // //                 // PRX_PRINT("Compute RRT Solution::: "<< manipulation_model->get_full_state_space()->print_memory(3), PRX_TEXT_BLUE);
// // //                 if(current_planner == NULL)
// // //                 {
// // //                     PRX_ERROR_S("Attempted to call compute solution on a planner that doesn't exist.");
// // //                     return false;
// // //                 }

// // //                 std::string old_context = current_planner->manip_model->get_current_context();

// // <<<<<<< local
// // //                 bool roadmap_conn = roadmap_neighbors.size()>0;
// // //                 current_planner->manip_model->use_context(current_planner->planning_context_name);
// // //                 // PRX_PRINT("Using context:: "<<current_planner->planning_context_name<<" as opposed to old context "<<old_context, PRX_TEXT_CYAN);
// // //                 // current_planner->setup(manipulation_model);
// // //                 current_planner->planner->reset();
// // //                 current_planner->query->clear();
// // =======
// //                 bool roadmap_conn = roadmap_neighbors.size()>0;
// //                 current_planner->manip_model->use_context(current_planner->planning_context_name);
// //                 // PRX_PRINT("Using context:: "<<current_planner->planning_context_name<<" as opposed to old context "<<old_context, PRX_TEXT_CYAN);
// //                 // current_planner->setup(manipulation_model);
// //                 // current_planner->planner->reset();
// //                 current_planner->query->clear();
// // >>>>>>> other

// // //                 state_t* context_goal = current_planner->state_space->alloc_point();
// // //                 state_t* context_start = current_planner->state_space->alloc_point();

// // //                 if(roadmap_conn)
// // //                 {
// // //                     current_planner->state_space->copy_point(context_goal, roadmap_neighbors[0]);
// // //                     PRX_PRINT("Using roadmap connections", PRX_TEXT_CYAN);
// // //                 }
// // //                 else
// // //                 {
// // //                     current_planner->manip_model->use_context(old_context);
// // //                     return false;
// // //                     current_planner->state_space->zero(context_goal);
// // //                     PRX_PRINT("Using half space goal", PRX_TEXT_CYAN);
// // //                 }
                
// // //                 manipulation_model->convert_spaces(current_planner->state_space, context_start, start_space, start);


// // //                 current_planner->query->setup(context_start, context_goal, LAZY_SEARCH, validity_checker->alloc_constraint(), false);
                

// // //                 multiple_goal_states_t* ms_goal;
// // //                 if(roadmap_conn)
// // //                 { 
// // //                     ms_goal = new multiple_goal_states_t(current_planner->state_space, current_planner->specification->metric, roadmap_neighbors.size());
// // //                     ms_goal->append_multiple_goal_states(roadmap_neighbors);
// // //                     current_planner->query->set_goal(ms_goal);
// // //                 }
// // //                 else
// // //                 {
// // //                     // current_planner->query->copy_start(context_start);
// // //                     hs_goal = new half_space_goal_t();
// // //                     hs_goal->setup(current_planner->manip_model, shelf_plane, half_space_side);
// // //                     hs_goal->link_space(current_planner->state_space);
// // //                     current_planner->query->set_goal(hs_goal);
// // //                 }

// // //                 current_planner->specification->clear_seeds();
// // //                 // std::vector< std::vector<double> > seeds_vec;
// // //                 // seeds_vec.resize(1);
// // //                 // current_planner->state_space->copy_point_to_vector(context_start, seeds_vec[0]);
// // //                 // current_planner->specification->set_seeds_vec(seeds_vec);
// // //                 current_planner->specification->add_seed(context_start);


// // //                 current_planner->planner->setup();
                
// // //                 current_planner->planner->link_query(current_planner->query);

// // <<<<<<< local
// // //                 try
// // //                 {
// // //                     current_planner->planner->execute();
// // //                 }
// // //                 catch( stopping_criteria_t::stopping_criteria_satisfied e )                
// // //                 {
// // =======
// //                 // try
// //                 // {
// //                 //     current_planner->planner->execute();
// //                 // }
// //                 // catch( stopping_criteria_t::stopping_criteria_satisfied e )                
// //                 // {
// // >>>>>>> other

// // <<<<<<< local
// // //                 }
// // =======
// //                 // }
// // >>>>>>> other

// // //                 PRX_PRINT("\n\nRESOLVING THE QUERY ON THE RRT.", PRX_TEXT_GREEN);
// // //                 current_planner->planner->resolve_query();

// // //                 current_planner->manip_model->use_context(old_context);
// // //                 current_planner->state_space->free_point(context_goal);
// // //                 current_planner->state_space->free_point(context_start);

// // //                 if(current_planner->query->plan.size()>0)//current_planner->query->found_solution)
// // //                 {
// // //                     if(roadmap_conn)
// // //                     {
// // //                         manipulation_model->get_state_space()->copy_to_point(satisifed_goal);
// // //                         PRX_PRINT("Out of all the multiple goals, achieved the goal:: "<< manipulation_model->get_state_space()->print_point(satisifed_goal, 3), PRX_TEXT_LIGHTGRAY);
// // //                     }
// // //                     else
// // //                     {
// // //                         manipulation_model->convert_spaces(manipulation_model->get_state_space(), satisifed_goal, current_planner->state_space, hs_goal->get_last_satisfied_point() );
// // //                     }

// // //                     manipulation_model->convert_plan(plan, manipulation_model->get_control_space(), current_planner->query->plan, current_planner->control_space);


// // //                     // PRX_PRINT("At the end of Compute RRT Solution::: "<< manipulation_model->get_full_state_space()->print_memory(3), PRX_TEXT_BLUE);
// // //                     return true;
// // //                 }
// // //                 else
// // //                 {
// // //                     PRX_PRINT("Query plan! - "<<current_planner->query->plan.size(), PRX_TEXT_CYAN);
// // //                 }
                
                

// // //                 return false;

// // //             }

// // //             void rrt_grasping_planner_t::reverse_plan_from_path(plan_t& reverse_plan, trajectory_t& path)
// // //             {
// // //                 trajectory_t reverse_path;
// // //                 reverse_path.link_space(grasping_query->state_space);
// // //                 reverse_path.reverse_trajectory(path);


// // //                 for(unsigned i = 0; i < reverse_path.size() - 1; ++i)
// // //                 {
// // //                     tmp_plan.clear();
// // //                     manipulation_model->steering_function(tmp_plan, reverse_path[i], reverse_path[i+1]);
// // //                     reverse_plan+=tmp_plan;
// // //                 }

// // //                 PRX_PRINT("Reversing plan..."<<validity_checker->is_valid(reverse_path), PRX_TEXT_RED);

// // //                 tmp_plan.clear();
// // //                 reverse_path.clear();
// // //             }

            

// // //             // void rrt_grasping_planner_t::link_parent_planner_info_map(util::hash_t<std::string, planner_info_t*> in_planner_info_map)
// // //             // {
// // //             //     parent_planner_info_map = in_planner_info_map;
// // //             // }

// // //             void rrt_grasping_planner_t::update_object_with_relative_grasp(config_t ee_config, config_t relative_grasp)
// // //             {
// // //                 config_t tmp_config;
// // //                 tmp_config = relative_grasp;
// // //                 tmp_config.relative_to_global(ee_config);
// // //                 std::vector<double> vec;
// // //                 vec.resize(7);
// // //                 tmp_config.get_position(vec[0], vec[1], vec[2]);
// // //                 tmp_config.get_orientation().get(vec[3], vec[4], vec[5], vec[6]);
// // //                 grasping_query->object->get_state_space()->set_from_vector(vec);
// // //             }



// // //             std::vector<state_t*> rrt_grasping_planner_t::get_nearest_points_on_roadmap(state_t* grasping_state, const space_t* grasping_space, config_t rel_grasp)
// // //             {                
// // //                 std::vector<state_t*> nearest_points_on_roadmap;


// // //                 std::string old_context = parent_planner->manip_model->get_current_context();
// // //                 parent_planner->manip_model->use_context(parent_planner->planning_context_name);

// // //                 PRX_PRINT("Using context:: "<<parent_planner->planning_context_name<<" as opposed to old context "<<old_context, PRX_TEXT_CYAN);


// // //                 state_t* context_grasp = parent_planner->state_space->alloc_point();
// // //                 manipulation_model->convert_spaces(parent_planner->state_space, context_grasp, grasping_space, grasping_state);


// // //                 abstract_node_t* grasp_node = new abstract_node_t();
// // //                 grasp_node->point = context_grasp;

// // //                 const std::vector<const abstract_node_t*> nearest_nodes_on_roadmap = parent_planner->specification->metric->radius_query(grasp_node, 2);

// // //                 state_t* model_state = manipulation_model->get_state_space()->alloc_point();

// // //                 state_t* context_start = parent_planner->state_space->alloc_point();
// // //                 state_t* context_goal = parent_planner->state_space->alloc_point();
                
// // //                 config_t ee_conf, grasping_conf;

// // //                 parent_planner->state_space->copy_from_point(context_grasp);
// // //                 manipulation_model->FK(grasping_conf);
// // //                 update_object_with_relative_grasp(grasping_conf, rel_grasp);
// // //                 manipulation_model->get_state_space()->copy_to_point(model_state);
// // //                 manipulation_model->push_state(model_state);
                

// // //                 //NEED THIS INFO FROM SOMEWHERE ELSE
// // //                 if(parent_planner->state_space->get_dimension()==9)
// // //                 {
// // //                     double state_arr[] = {0,1.57,0,0,-1.70,0,0,0,0};
// // //                     std::vector<double> graph_state_vec(state_arr, state_arr+9);
// // //                     parent_planner->state_space->copy_vector_to_point(graph_state_vec, context_goal);
// // //                 }
// // //                 else if(parent_planner->state_space->get_dimension()==8)
// // //                 {
// // //                     double state_arr[] = {1.57,0,0,-1.70,0,0,0,0};
// // //                     std::vector<double> graph_state_vec(state_arr, state_arr+8);
// // //                     parent_planner->state_space->copy_vector_to_point(graph_state_vec, context_goal);
// // //                 }
// // //                 else
// // //                 {
// // //                     PRX_FATAL_S("Currently set up only to work with the left or the right arm.");
// // //                 }

// // //                 PRX_PRINT(parent_planner->state_space->print_point(context_goal, 3), PRX_TEXT_LIGHTGRAY);
                
// // //                 constraints_t* tmp_constraints = parent_planner->specification->validity_checker->alloc_constraint();
// // //                 std::vector< std::string > constraint_names;
// // //                 parent_planner->specification->validity_checker->get_constraint_names(constraint_names);
// // //                 parent_planner->setup_search(LAZY_SEARCH, tmp_constraints, constraint_names);
                
// // //                 bool first_astar_search = true;

// <<<<<<< local
//                 PRX_PRINT("\n\n\n\n\nFinding the "<< nearest_nodes_on_roadmap.size()<<" nearest points on the roadmap from "<<grasping_conf, PRX_TEXT_RED);
//                 for(unsigned i =0; i<nearest_nodes_on_roadmap.size(); ++i)
//                 {
//                     if(i>50)
//                         break;

//                     if(i>20 && (nearest_points_on_roadmap.size()>0 && ((double)(i/nearest_points_on_roadmap.size())<10)))
//                         break;
                    
//                     parent_planner->state_space->copy_point(context_start, nearest_nodes_on_roadmap[i]->point);
//                     PRX_PRINT(parent_planner->state_space->get_space_name(), PRX_TEXT_RED);
//                     parent_planner->state_space->copy_from_point(context_start);
//                     manipulation_model->FK(ee_conf);
//                     update_object_with_relative_grasp(ee_conf, rel_grasp);
//                     manipulation_model->get_state_space()->copy_to_point(model_state);
//                     manipulation_model->push_state(model_state);
//                     double dist = get_config_distance(grasping_conf, ee_conf);
//                     PRX_PRINT("Distance between the points in workspace to "<<ee_conf<<" is "<<dist, PRX_TEXT_BLUE);
//                     PRX_PRINT("["<<i<<"] -- "<<parent_planner->state_space->print_point(context_start,3), (dist<0.2?PRX_TEXT_GREEN:PRX_TEXT_RED));
//                     PRX_PRINT("["<<i<<"] -- Target "<<parent_planner->state_space->print_point(context_goal,3), (dist<0.2?PRX_TEXT_GREEN:PRX_TEXT_RED));
// =======
// // //                 PRX_PRINT("\n\n\n\n\nFinding the nearest points on the roadmap from "<<grasping_conf, PRX_TEXT_RED);
// // //                 for(unsigned i =0; i<nearest_nodes_on_roadmap.size(); ++i)
// // //                 {
// // //                     parent_planner->state_space->copy_point(context_start, nearest_nodes_on_roadmap[i]->point);
// // //                     PRX_PRINT(parent_planner->state_space->get_space_name(), PRX_TEXT_RED);
// // //                     parent_planner->state_space->copy_from_point(context_start);
// // //                     manipulation_model->FK(ee_conf);
// // //                     update_object_with_relative_grasp(ee_conf, rel_grasp);
// // //                     manipulation_model->get_state_space()->copy_to_point(model_state);
// // //                     manipulation_model->push_state(model_state);
// // //                     double dist = get_config_distance(grasping_conf, ee_conf);
// // //                     PRX_PRINT("Distance between the points in workspace to "<<ee_conf<<" is "<<dist, PRX_TEXT_BLUE);
// // //                     PRX_PRINT("["<<i<<"] -- "<<parent_planner->state_space->print_point(context_start,3), (dist<0.2?PRX_TEXT_GREEN:PRX_TEXT_RED));
// // //                     PRX_PRINT("["<<i<<"] -- Target "<<parent_planner->state_space->print_point(context_goal,3), (dist<0.2?PRX_TEXT_GREEN:PRX_TEXT_RED));
// >>>>>>> other

// // //                     if(dist<0.3)
// // //                     {

// <<<<<<< local
//                         parent_planner->query->clear();
//                         parent_planner->query->setup(context_start, context_goal, LAZY_SEARCH, validity_checker->alloc_constraint(), false);
//                         parent_planner->query->restart_astar_search = first_astar_search;
//                         parent_planner->query->lazy_iterations = 10;
//                         first_astar_search = false;
//                         PRX_PRINT("\n\nRESOLVING THE QUERY ON THE PRM!!", PRX_TEXT_GREEN);
//                         parent_planner->planner->resolve_query();
// =======
// // //                         parent_planner->query->clear();
// // //                         parent_planner->query->setup(context_start, context_goal, LAZY_SEARCH, validity_checker->alloc_constraint(), false);
// // //                         parent_planner->query->restart_astar_search = first_astar_search;
// // //                         first_astar_search = false;
// // //                         PRX_PRINT("\n\nRESOLVING THE QUERY ON THE PRM!!", PRX_TEXT_GREEN);
// // //                         parent_planner->planner->resolve_query();
// >>>>>>> other

// <<<<<<< local
//                         if(parent_planner->query->plan.size()>0)
//                         {
//                             PRX_PRINT("Point is connected to the majorly connected component of the roadmap.", PRX_TEXT_GREEN);
//                             state_t* new_neighbor = parent_planner->state_space->alloc_point();
//                             parent_planner->state_space->copy_point(new_neighbor, context_start);
//                             nearest_points_on_roadmap.push_back(new_neighbor);
//                             if(nearest_points_on_roadmap.size()>10)
//                                 break;
//                         }
//                     }
//                 }
//                 PRX_PRINT("NUMBER OF NEAR POINTS ARE - - - "<<nearest_points_on_roadmap.size()<<"\n\n\n\n\n", PRX_TEXT_RED);
// =======
// // //                         if(parent_planner->query->plan.size()>0)
// // //                         {
// // //                             PRX_PRINT("Point is connected to the majorly connected component of the roadmap.", PRX_TEXT_GREEN);
// // //                             state_t* new_neighbor = parent_planner->state_space->alloc_point();
// // //                             parent_planner->state_space->copy_point(new_neighbor, context_start);
// // //                             nearest_points_on_roadmap.push_back(new_neighbor);
// // //                         }
// // //                     }
// // //                 }
// // //                 PRX_PRINT("NUMBER OF NEAR POINTS ARE - - - "<<nearest_points_on_roadmap.size()<<"\n\n\n\n\n", PRX_TEXT_RED);
// >>>>>>> other



// // //                 parent_planner->manip_model->use_context(old_context);
// // //                 parent_planner->state_space->free_point(context_start);
// // //                 parent_planner->state_space->free_point(context_goal);
// // //                 manipulation_model->get_state_space()->free_point(model_state);
// // //                 parent_planner->state_space->free_point(context_grasp);
// // //                 parent_planner->specification->validity_checker->free_constraint(tmp_constraints);
// // //                 return nearest_points_on_roadmap;
// // //             }


// // //             double rrt_grasping_planner_t::get_config_distance(config_t& c1, config_t& c2)
// // //             {
// // //                 double x1, x2, y1, y2, z1, z2;
// // //                 c1.get_position(x1,y1,z1);
// // //                 c2.get_position(x2,y2,z2);
// // //                 return std::sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) + (z2-z1)*(z2-z1) );
// // //             }

// // //             // bool rrt_grasping_planner_t::evaluate_the_relative_config(const grasp_t* grasp)
// // //             // {
// // //             //     PRX_PRINT("Evaluating the relative config", PRX_TEXT_CYAN);
// // //             //     PRX_ASSERT(grasp != NULL);

// // //             //     grasping_query->object->get_state_space()->copy_to_point(original_object_state);
// // //             //     config_t object_pose;

// // //             //     int obj_index = 0;
// // //             //     foreach(state_t* obj_state, grasping_query->object_states)
// // //             //     {
// // //             //         grasp_data_t* data = new grasp_data_t();
// // //             //         *data->relative_grasp = *grasp;

// // //             //         if(grasping_query->mode == grasping_query_t::PRX_GRASP_WITH_COLLISION)
// // //             //         {
// // //             //             bool got_solution = false;
// // //             //             state_to_config(object_pose, obj_state);
                        
// // //             //             // PRX_PRINT("End-Effector configuration: " << ee_local_config, PRX_TEXT_MAGENTA);
// // //             //             // PRX_PRINT("Grasp structure's relative config: " << grasp->relative_config, PRX_TEXT_LIGHTGRAY);
// // //             //             // PRX_PRINT("Current object pose: " << object_pose, PRX_TEXT_LIGHTGRAY);
                        
// // //             //             config_t grasping_config = ee_local_config;
// // //             //             grasping_config.relative_to_global(grasp->relative_config);
// // //             //             grasping_config.relative_to_global( object_pose );

// // //             //             // PRX_PRINT("Results in a final grasping config: " << grasping_config, PRX_TEXT_CYAN);

// // //             //             config_t retracted_config;
// // //             //             grasping_query->object->get_state_space()->copy_from_point(obj_state);

// // //             //             int first_mode = manipulation_model->get_current_grasping_mode();

// // //             //             if(first_mode != grasp->release_mode)
// // //             //             {
// // //             //                 manipulation_model->engage_grasp(tmp_plan, grasp->release_mode, false);
// // //             //             }

// // //             //             data->setup(grasping_query->state_space, grasping_query->control_space, validity_checker);
// // //             //             //using the releasing state as tmp variable in order to get the initial state of the manipulator.
// // //             //             manipulator_info->full_arm_state_space->copy_to_point(data->releasing_state);
// // //             //             PRX_PRINT("Grasping TP :: Seed state: " << manipulator_info->full_arm_state_space->print_point(data->releasing_state,5), PRX_TEXT_CYAN);

// // //             //             //TODO: We need to be using validate_and_generate_constraints in place of is_valid if we're in soft constraints mode...?

// // //             //             bool successIK = false;
// // //             //             int num_tries = 2;
// // //             //             while(--num_tries>0)
// // //             //             {   
// // //             //                 PRX_PRINT("IK Tries: "<<num_tries, PRX_TEXT_MAGENTA);
// // //             //                 if(manipulation_model->IK(data->releasing_state, data->releasing_state, grasping_config, false))  
// // //             //                 {                   
// // //             //                     if( validity_checker->validate_and_generate_constraints(new_constraints, data->releasing_state) )
// // //             //                     {
// // //             //                         successIK = true;
// // //             //                         break;
// // //             //                     }
// // //             //                     else
// // //             //                     {
// // //             //                         PRX_PRINT("Found an IK for releasing state " << manipulator_info->full_arm_state_space->print_point(data->releasing_state, 5),PRX_TEXT_MAGENTA);
// // //             //                         sim::collision_list_t* colliding_bodies = manipulation_model->get_colliding_bodies(data->releasing_state);
// // //             //                         foreach(collision_pair_t cp, colliding_bodies->get_body_pairs())
// // //             //                         {
// // //             //                             PRX_PRINT("COLLISION DETECTED BETWEEN : "<<reverse_split_path( cp.first ).first<<"  and "<<reverse_split_path( cp.second ).first, PRX_TEXT_RED);
// // //             //                         }
// // //             //                         grasping_query->reason_for_failure="Initial IK invalid : in collision";
// // //             //                     }
// // //             //                 }
// // //             //                 else
// // //             //                 {
// // //             //                     grasping_query->reason_for_failure="Initial IK failed";
// // //             //                 }
// // //             //             }

// // //             //             PRX_DEBUG_COLOR("Grasping TP :: Releasing state (" << successIK << ") " << manipulator_info->full_arm_state_space->print_point(data->releasing_state,5), PRX_TEXT_CYAN);
// // //             //             if(successIK)
// // //             //             {
// // //             //                 PRX_PRINT("-Grasping TP :: Releasing state is valid " << manipulator_info->full_arm_state_space->print_point(data->releasing_state, 5),PRX_TEXT_MAGENTA);
// // //             //                 PRX_PRINT("-Object: " << manipulation_model->get_active_space()->print_memory(5),PRX_TEXT_BLUE);
                            


// // //             //                 retracted_config = grasping_query->retraction_config;
// // //             //                 retracted_config.relative_to_global( grasping_config );

// // //             //                 //Set the world to the releasing state
// // //             //                 manipulator_info->full_arm_state_space->copy_from_point(data->releasing_state);
// // //             //                 grasping_query->object->get_state_space()->copy_from_point(obj_state);

// // //             //                 //Grasp
// // //             //                 // PRX_PRINT("12345::: "<< manipulation_model->get_full_state_space()->print_memory(3), PRX_TEXT_LIGHTGRAY);
// // //             //                 int last_mode = manipulation_model->get_current_grasping_mode();
// // //             //                 manipulation_model->engage_grasp(tmp_plan, grasp->grasping_mode, true);
// // //             //                 // PRX_PRINT("54321::: "<< manipulation_model->get_full_state_space()->print_memory(3), PRX_TEXT_BROWN);
// // //             //                 //Grasping in releasing point becomes the grasping state
// // //             //                 manipulator_info->full_arm_state_space->copy_to_point(data->grasping_state);


// // //             //                 config_t forward_relative = object_pose;
// // //             //                 forward_relative.global_to_relative(grasping_config);
// // //             //                 //Need to delete previous points
// // //             //                 std::vector<state_t*> roadmap_goals = get_nearest_points_on_roadmap(data->grasping_state, manipulator_info->full_arm_state_space, forward_relative);
// // //             //                 PRX_PRINT("Nearest points... ", PRX_TEXT_BLUE);
// // //             //                 foreach(state_t* st, roadmap_goals)
// // //             //                 {
// // //             //                     PRX_PRINT(active_planner->state_space->print_point(st, 4), PRX_TEXT_MAGENTA);
// // //             //                 }


// // //             //                 PRX_PRINT("Before rrt    "<<manipulation_model->get_full_state_space()->print_memory(2), PRX_TEXT_MAGENTA);
                            

// // //             //                 //Find a motion plan from grasping state that takes the object out of the shelf
// // //             //                 PRX_PRINT("Finding retracting plan with object in hand...", PRX_TEXT_GREEN);
// // //             //                 bool found_rrt_solution = compute_rrt_solution(data->retracting_plan, data->retracted_close_state, active_planner, data->grasping_state, manipulator_info->full_arm_state_space, roadmap_goals);

// // //             //                 PRX_PRINT("",PRX_TEXT_CYAN);
// // //             //                 PRX_PRINT("After rrt     "<<manipulation_model->get_full_state_space()->print_memory(2), PRX_TEXT_MAGENTA);
// // //             //                 manipulator_info->full_arm_state_space->copy_to_point(data->grasping_state);
// // //             //                 grasping_query->object->get_state_space()->copy_from_point(obj_state);
// // //             //                 manipulation_model->engage_grasp(tmp_plan, grasp->grasping_mode, true);

                            
// // //             //                 if(found_rrt_solution)
// // //             //                 {
// // //             //                     //Found a retracted closed state outside the shelf
// // //             //                     manipulator_info->full_arm_state_space->copy_from_point(data->retracted_close_state);
// // //             //                     //Ungrasp
// // //             //                     manipulation_model->engage_grasp(tmp_plan, last_mode, false);
// // //             //                     //Set to retracted_open_state
// // //             //                     manipulator_info->full_arm_state_space->copy_to_point(data->retracted_open_state);

// // //             //                     PRX_PRINT("\n"<<manipulator_info->full_arm_state_space->print_point(data->retracted_open_state,3)<<" is open and \n"<<manipulator_info->full_arm_state_space->print_point(data->retracted_close_state,3)<<" is closed", PRX_TEXT_RED);
// // //             //                     PRX_PRINT("\n"<<manipulator_info->full_arm_state_space->print_point(data->releasing_state,3)<<" is open and \n"<<manipulator_info->full_arm_state_space->print_point(data->grasping_state,3)<<" is closed", PRX_TEXT_RED);
// // //             //                 }
// // //             //                 else
// // //             //                 {
// // //             //                     //Just ungrasp
// // //             //                     manipulation_model->engage_grasp(tmp_plan, last_mode, false);
// // //             //                 }

// // //             //                 //Set both the arm and the object back to the releasing state
// // //             //                 manipulator_info->full_arm_state_space->copy_from_point(data->releasing_state);
// // //             //                 grasping_query->object->get_state_space()->copy_from_point(obj_state);

// // //             //                 // PRX_PRINT("54321::: "<< manipulation_model->get_full_state_space()->print_memory(3), PRX_TEXT_BROWN);


                            
// // //             //                 tmp_plan.clear();
// // //             //                 //compute_rrt_solution(tmp_plan, data->retracted_open_state, active_planner, data->releasing_state, manipulator_info->full_arm_state_space);

     

// // //             //                 bool was_last_check_valid = false;


// // //             //                 //Keeping the subsequent logic the same for reasoning about success. Currently only correct for Reach and Retract.


// // //             //                 //Compute the IK steering for the retraction plan
// // //             //                 // if(manipulation_model->IK_steering(data->retracting_plan, data->retracted_open_state, data->releasing_state, retracted_config))
// // //             //                 PRX_PRINT((found_rrt_solution?"Finding retracting plan without object in hand...":"Skipping remaining evaluations..."), PRX_TEXT_GREEN);
// // //             //                 if( found_rrt_solution && compute_rrt_solution(tmp_plan, data->retracted_open_state, active_planner, data->releasing_state, manipulator_info->full_arm_state_space, roadmap_goals))// compute_rrt_solution(data->retracting_plan, data->retracted_open_state, active_planner, data->releasing_state, manipulator_info->full_arm_state_space))
// // //             //                 {
// // //             //                     PRX_PRINT("--Grasping TP :: IK steering for retraction, retracted_open_state:" << manipulator_info->full_arm_state_space->print_point(data->retracted_open_state, 5),PRX_TEXT_MAGENTA)
// // //             //                     PRX_PRINT("--Object: " << manipulation_model->get_active_space()->print_memory(5),PRX_TEXT_BLUE);

// // //             //                     tmp_path.clear();
// // //             //                     manipulation_model->propagate_plan(data->releasing_state, tmp_plan, tmp_path);
// // //             //                     reverse_plan_from_path(data->reaching_plan, tmp_path);


// // //             //                     //Checks if the retracted state is collision free before we check for the entire path.
// // //             //                     if( validity_checker->validate_and_generate_constraints( new_constraints, data->retracted_open_state ) )
// // //             //                     {
// // //             //                         PRX_PRINT("---Grasping TP :: Retracted state is valid", PRX_TEXT_MAGENTA);
// // //             //                         //Propagates the plan from the IK steering and checks if the whole path is collision free.
// // //             //                         tmp_path.clear();
// // //             //                         manipulation_model->propagate_plan(data->releasing_state, data->retracting_plan, tmp_path);
// // //             //                         data->open_retracting_constraints = validity_checker->alloc_constraint();
                                    
// // //             //                         was_last_check_valid = validity_checker->validate_and_generate_constraints(data->open_retracting_constraints, tmp_path);
// // //             //                         PRX_PRINT("IS VALID! "<<validity_checker->is_valid(tmp_path), PRX_TEXT_RED);
// // //             //                         if( was_last_check_valid || true)
// // //             //                         {
// // //             //                             PRX_PRINT("----Grasping TP :: Retraction trajectory is valid , end of path: " << manipulator_info->full_arm_state_space->print_point(tmp_path.back(),5) , ( was_last_check_valid? PRX_TEXT_MAGENTA : PRX_TEXT_RED));
// // //             //                             PRX_PRINT("----Object: " << manipulation_model->get_active_space()->print_memory(5),PRX_TEXT_BLUE);
// // //             //                             //The manipulator should be at the retracted_open_state right now. 

// // //             //                             // if(manipulation_model->IK_steering(data->reaching_plan, tmp_state, data->retracted_open_state, data->releasing_state, grasping_config ))
// // //             //                             // reverse_plan_from_path(data->reaching_plan, tmp_path);
// // //             //                             if(true)
// // //             //                             {
// // //             //                                 PRX_PRINT("-----Grasping TP :: IK steer toward object worked (" << data->reaching_plan.size() << "), the new realizing state  : " <<  manipulator_info->full_arm_state_space->print_point(tmp_path[0],5),PRX_TEXT_MAGENTA)
// // //             //                                 PRX_PRINT("-----Object: " << manipulation_model->get_active_space()->print_memory(5),PRX_TEXT_BLUE);                                        
// // //             //                                 //Propagates the plan from the IK steering and checks if the whole path is collision free.
// // //             //                                 tmp_path.clear();
// // //             //                                 manipulation_model->propagate_plan(data->retracted_open_state, data->reaching_plan, tmp_path);
// // //             //                                 data->open_reaching_constraints = validity_checker->alloc_constraint();  
                                            
// // //             //                                 was_last_check_valid = validity_checker->validate_and_generate_constraints(data->open_reaching_constraints, tmp_path);
// // //             //                                 PRX_PRINT("IS VALID! "<<validity_checker->is_valid(tmp_path), PRX_TEXT_RED);
// // //             //                                 if( was_last_check_valid || true)
// // //             //                                 {  
// // //             //                                     PRX_PRINT("------Grasping TP :: Reaching plan is collision free , end of reaching path: " << manipulator_info->full_arm_state_space->print_point(tmp_path.back(),5),( was_last_check_valid? PRX_TEXT_MAGENTA : PRX_TEXT_RED))
// // //             //                                     PRX_PRINT("------Object: " << manipulation_model->get_active_space()->print_memory(5),PRX_TEXT_BLUE);
// // //             //                                     //Everything from the open mode are valid. Now we have to check and the grasping states. 
// // //             //                                     //Setting up the manipulator and the object that the pose. 
// // //             //                                     manipulator_info->full_arm_state_space->copy_from_point(data->releasing_state);
// // //             //                                     tmp_plan.clear();
// // //             //                                     int last_mode = manipulation_model->get_current_grasping_mode();
// // //             //                                     PRX_PRINT("------Current mode: " << last_mode,PRX_TEXT_BLUE);
// // //             //                                     manipulation_model->engage_grasp(tmp_plan, grasp->grasping_mode, true);
// // //             //                                     manipulator_info->full_arm_state_space->copy_to_point(data->grasping_state);
// // //             //                                     PRX_PRINT("-------Grasping TP :: Got the grasped state: " << manipulator_info->full_arm_state_space->print_point(data->grasping_state,5),PRX_TEXT_MAGENTA)
// // //             //                                     tmp_path.clear();                                                
// // //             //                                     manipulation_model->propagate_plan(data->grasping_state, data->retracting_plan, tmp_path);

// // //             //                                     data->close_retracting_constraints = validity_checker->alloc_constraint();
// // //             //                                     was_last_check_valid = validity_checker->validate_and_generate_constraints(data->close_retracting_constraints, tmp_path);
// // //             //                                     PRX_PRINT("IS VALID! "<<validity_checker->is_valid(tmp_path), PRX_TEXT_RED);
// // //             //                                     if( was_last_check_valid || true)
// // //             //                                     {
// // //             //                                         PRX_PRINT("-------Grasping TP :: retract with object valid and back of the path is: " << manipulator_info->full_arm_state_space->print_point(tmp_path.back(),5),( was_last_check_valid? PRX_TEXT_MAGENTA : PRX_TEXT_RED))
// // //             //                                         PRX_PRINT("-------Object: " << manipulation_model->get_active_space()->print_memory(5),PRX_TEXT_BLUE);
// // //             //                                         manipulator_info->full_arm_state_space->copy_point(data->retracted_close_state, tmp_path.back());
// // //             //                                         PRX_PRINT("-------Grasping TP :: the retracted closed state is : " << manipulator_info->full_arm_state_space->print_point(data->retracted_close_state,5),PRX_TEXT_MAGENTA)                                                
// // //             //                                         tmp_path.clear();
// // //             //                                         manipulation_model->propagate_plan(data->retracted_close_state, data->reaching_plan, tmp_path);

// // //             //                                         data->close_reaching_constraints = validity_checker->alloc_constraint();
// // //             //                                         was_last_check_valid = validity_checker->validate_and_generate_constraints(data->close_reaching_constraints, tmp_path);
// // //             //                                         PRX_PRINT("IS VALID! "<<validity_checker->is_valid(tmp_path), PRX_TEXT_RED);
// // //             //                                         if( was_last_check_valid || true)
// // //             //                                         {
// // //             //                                             PRX_PRINT("--------Grasping TP :: reach with object valid  path_end: " << manipulator_info->full_arm_state_space->print_point(tmp_path.back(),5),( was_last_check_valid? PRX_TEXT_MAGENTA : PRX_TEXT_RED))
// // //             //                                             PRX_PRINT("--------Object: " << manipulation_model->get_active_space()->print_memory(5),PRX_TEXT_BLUE);
                                                        
// // //             //                                             got_solution = true;
// // //             //                                             data->reaching_plan.append_onto_back(3);
// // //             //                                             data->retracting_plan.append_onto_back(3);
// // //             //                                             data->reaching_plan.append_onto_front(3);
// // //             //                                             data->retracting_plan.append_onto_front(3);
// // //             //                                         }
// // //             //                                         else
// // //             //                                         {
// // //             //                                             grasping_query->reason_for_failure="Reach with object invalid";
// // //             //                                         }
// // //             //                                     }
// // //             //                                     else
// // //             //                                     {
// // //             //                                         grasping_query->reason_for_failure="Retract with object invalid";                        
// // //             //                                     }                                            
// // //             //                                     manipulation_model->engage_grasp(tmp_plan, last_mode, false); 
// // //             //                                     manipulator_info->full_arm_state_space->copy_from_point(data->releasing_state);
// // //             //                                     PRX_PRINT("Previous grasping mode: "<<last_mode,PRX_TEXT_BLUE);                                           
// // //             //                                 }
// // //             //                                 else
// // //             //                                 {
// // //             //                                     grasping_query->reason_for_failure="Reaching path to the object invalid";
// // //             //                                 }
// // //             //                             }
// // //             //                             else
// // //             //                             {
// // //             //                                 grasping_query->reason_for_failure="Reaching IK steering failed";
// // //             //                             }
// // //             //                         }
// // //             //                         else
// // //             //                         {
// // //             //                             grasping_query->reason_for_failure="Retracting path invalid";
// // //             //                         }
// // //             //                     }
// // //             //                     else
// // //             //                     {
// // //             //                         grasping_query->reason_for_failure="Retracted state invalid";
// // //             //                     }
// // //             //                 }
// // //             //                 else
// // //             //                 {
// // //             //                     grasping_query->reason_for_failure="Retracting IK steering failed";
// // //             //                 }
// // //             //             }

// // //             //             if(first_mode != grasp->release_mode)
// // //             //             {
// // //             //                 manipulation_model->engage_grasp(tmp_plan, first_mode, false);
// // //             //             }
// // //             //             grasping_query->object->get_state_space()->copy_from_point(original_object_state);

// // //             //             if(!got_solution)
// // //             //             {
// // //             //                 PRX_PRINT("----Grasp Planner Failed : " << grasping_query->reason_for_failure, PRX_TEXT_RED);
// // //             //                 delete data;
// // //             //                 for(int i = obj_index-1; i >= 0; --i)
// // //             //                 {
// // //             //                     grasping_query->remove_last_data_from(i);
// // //             //                 }
// // //             //                 return false;
// // //             //             }
// // //             //             else
// // //             //             {
// // //             //                 PRX_PRINT("----Grasping TP :: Grasp success at object location: " << object_pose, PRX_TEXT_GREEN);
// // //             //                 // PRX_DEBUG_COLOR(" - open reaching " << data->open_reaching_constraints->print(), PRX_TEXT_LIGHTGRAY );
// // //             //                 // PRX_DEBUG_COLOR(" - closed reaching " << data->close_reaching_constraints->print(), PRX_TEXT_LIGHTGRAY );
// // //             //                 // PRX_DEBUG_COLOR(" - open retracting " << data->open_retracting_constraints->print(), PRX_TEXT_LIGHTGRAY );
// // //             //                 // PRX_DEBUG_COLOR(" - closed retracting " << data->close_retracting_constraints->print(), PRX_TEXT_LIGHTGRAY );
// // //             //             }
// // //             //         }
// // //             //         grasping_query->set_data(obj_index, data);
// // //             //         obj_index++;
// // //             //     }
// // //             //     return true;
// // //             // }

            

// // //         }
// // //     }
// // // }
