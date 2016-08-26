// /**
//  * @file grasp_rrt_tp.cpp
//  *
//  * @copyright Software License Agreement (BSD License)
//  * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
//  * All Rights Reserved.
//  * For a full description see the file named LICENSE.
//  *
//  * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
//  *
//  * Email: pracsys@googlegroups.com
//  */

// #include "planning/task_planners/grasp_rrt_tp.hpp"

// #include "prx/utilities/definitions/string_manip.hpp"
// #include "prx/utilities/definitions/random.hpp"
// #include "prx/utilities/math/configurations/config.hpp"
// #include "prx/utilities/statistics/statistics.hpp"
// #include "prx/utilities/heuristic_search/constraints.hpp"
// #include "prx/utilities/goals/multiple_goal_states.hpp"
 
// #include "prx/planning/communication/visualization_comm.hpp"
// #include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"

// #include "planning/modules/object_constraints_checker.hpp"
// #include "planning/specifications/manipulation_specification.hpp"
// #include "planning/specifications/grasping_specification.hpp"
// #include "planning/goals/half_space_goal.hpp"
// #include "planning/specifications/grasp_rrt_specification.hpp"
// #include "planning/modules/grasp_evaluator.hpp"

// #include <boost/range/adaptor/map.hpp>
// #include <boost/assign/list_of.hpp>
// #include <pluginlib/class_list_macros.h>
// #include <string>

// PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::grasp_rrt_tp_t, prx::plan::planner_t)

// namespace prx
// {
//     using namespace util;
//     using namespace sim;
//     using namespace plan;

//     namespace packages
//     {
//         namespace manipulation
//         {

//             grasp_rrt_tp_t::grasp_rrt_tp_t()
//             {
//                 grasping_planner = NULL;
//                 tmp_constraint = NULL;
//                 validity_checker = NULL;
//                 use_ee_dist_in_planners = false;
//                 shelf_plane_goal = NULL;
//                 _grasp_success_time = _grasp_failure_time = _mp_success_time = _mp_failure_time = 0;
//                 _grasp_success_count = _grasp_failure_count = _mp_success_count = _mp_failure_count = 0;
                
//             }

//             grasp_rrt_tp_t::~grasp_rrt_tp_t()
//             {

//                 if (validity_checker != NULL)
//                 {
//                     validity_checker->free_constraint(tmp_constraint);
//                 }
                
//                 foreach( end_effector_distance_t* dist, ee_distances )
//                 {
//                     delete dist;
//                 }

//                 delete grasping_query;
//                 delete feasible_query;
//             }

//             void grasp_rrt_tp_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
//             {
//                 PRX_DEBUG_COLOR("Initializing Manipulation task planner ...", PRX_TEXT_CYAN);

//                 std::string template_name;
//                 task_planner_t::init(reader, template_reader);
//                 // TODO: What does this comment refer to?
//                 //Added for safety (since we are checking nullness later) ZL

//                 // TODO: This was unused: Is it necessary? AK
//                 //random_feasible_grasp = parameters::get_attribute_as< bool > ("random_feasible_grasp", reader, template_reader, true);
                
//                 num_candidate_connections = parameters::get_attribute_as< unsigned >("num_candidate_connections", reader, template_reader, 10);

//                 // TODO: Ideally, this variable is tied to each specific motion planner, rather than being global.
//                 use_ee_dist_in_planners = parameters::get_attribute_as< bool >("use_ee_dist_in_planners", reader, template_reader );

//                 force_connection_plans = parameters::get_attribute_as< bool >("force_connection_plans", reader, template_reader );
//                 skip_connectivity_check = parameters::get_attribute_as< bool >("skip_connectivity_check", reader, template_reader );
//                 // Check if we have defined a half-space goal
//                 sampler_t* apc_sampler = parameters::initialize_from_loader<sampler_t > ("prx_planning", reader, "sampler", template_reader, "sampler") ;
//                 neighborhood_sampler = dynamic_cast< prx::packages::apc::apc_sampler_t* >(apc_sampler);
//                 if(neighborhood_sampler == NULL)
//                 {
//                     PRX_ERROR_S("Neighborhood sampler must be apc sampler.");
//                 }

//                 if(parameters::has_attribute("shelf_plane", reader, NULL))
//                 {
//                     shelf_plane = parameters::get_attribute_as<std::vector<double> >("shelf_plane", reader, template_reader);
//                     half_space_side = parameters::get_attribute_as<bool>("half_space_side", reader, template_reader);
//                     if(shelf_plane.size()<4)
//                     {
//                         PRX_WARN_S("The shelf plane has to have all the parameters of the plane.");
//                     }
//                     else
//                     {
//                         //PRX_PRINT("Half Space Set up with equation: "<<shelf_plane[0]<<"x + "<<shelf_plane[1]<<"y + "<<shelf_plane[2]<<"z + "<<shelf_plane[3]<<(half_space_side?" >= ":" <= ")<<"0.", PRX_TEXT_MAGENTA);
//                         shelf_plane_goal = new half_space_goal_t();
//                     }
//                 }

//                 //initialize the motion planners
//                 if( reader->has_attribute("planners") )
//                 {
//                     parameter_reader_t::reader_map_t planner_map = reader->get_map("planners");

//                     foreach(const parameter_reader_t::reader_map_t::value_type key_value, planner_map)
//                     {
//                         const parameter_reader_t* child_template_reader = NULL;
//                         std::string planner_name = key_value.first;

//                         if( key_value.second->has_attribute("template") )
//                         {
//                             template_name = key_value.second->get_attribute("template");
//                             child_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + template_name, global_storage);
//                         }
//                         motion_planning_specification_t* new_spec = dynamic_cast< motion_planning_specification_t* >( output_specifications[planner_name] );

//                         //If we are initializing one of the motion planners
//                         if( new_spec != NULL )
//                         {
//                             //Link the motion planner to the specification's stopping criterion
//                             new_spec->get_stopping_criterion()->link_motion_planner((motion_planner_t*)planners[planner_name]);
//                             //Make sure we get the query as a motion planning query
//                             query_t* new_query = dynamic_cast< motion_planning_query_t* >( output_queries[planner_name] );
//                             // Get the planner's planning context name
//                             std::string planning_context_name = parameters::get_attribute_as< std::string >("planning_context_name", key_value.second, child_template_reader);
//                             // Based on whether or not it is a connection planner, store in the appropriate mapping
//                             bool is_connection_planner = parameters::get_attribute_as<bool>("is_connection_planner", key_value.second, child_template_reader, false);
//                             if (!is_connection_planner)
//                                 planner_info_map[planning_context_name] = new planner_info_t(space_names[planner_name], planning_context_name, planners[planner_name], new_spec, new_query);
//                             else
//                             {
//                                 PRX_PRINT ("Found connection planner!", PRX_TEXT_MAGENTA);
//                                 connection_info_map[planning_context_name] = new planner_info_t(space_names[planner_name], planning_context_name, planners[planner_name], new_spec, new_query);
//                                 connection_planner_names[planning_context_name] = planner_name;
//                             }
//                         }
//                         //Otherwise, this must be the grasp planner
//                         else
//                         {
//                             //Double check that this is the case.
//                             grasping_specification = dynamic_cast< grasping_specification_t* >(output_specifications[planner_name]);
//                             if( grasping_specification == NULL )
//                             {
//                                 PRX_FATAL_S("Found a planner with an unknown specification type in the Manipulation task planner.");
//                             }
//                             grasping_planner = dynamic_cast< grasping_planner_t *>( planners[planner_name] );
//                             grasping_planner->link_specification( output_specifications[planner_name] );
//                         }

//                         if( child_template_reader != NULL )
//                         {
//                             delete child_template_reader;
//                             child_template_reader = NULL;
//                         }

//                     }
//                 }
                
//                 IK_steer_movements = parameters::get_attribute_as<bool>("IK_steer_grasping", reader, template_reader, false);

//                 serialize_flag = parameters::get_attribute_as<bool>("serialize_flag", reader, template_reader, false);

//                 neighborhood = parameters::get_attribute_as<double>("neighborhood", reader, template_reader, 0.2);


                


//             }

//             void grasp_rrt_tp_t::setup()
//             {
//                 std::string old_context = manipulation_model->get_current_context();

//                 // Setup the motion planners (via planner info)
//                 foreach(planner_info_t* planner_info, planner_info_map | boost::adaptors::map_values)
//                 {
//                     manipulation_model->use_context(planner_info->construction_context_name);

//                     //We have to override the distance function to be the end-effector distance
//                     // if( use_ee_dist_in_planners )
//                     // {
//                     //     ee_distances.push_back( new end_effector_distance_t() );
//                     //     ee_distances.back()->link_manipulation_model( manipulation_model );
//                     //     planner_info->specification->metric->link_distance_function( ee_distances.back() );
//                     // }

//                     planner_info->setup(manipulation_model);
//                     planner_info->planner->setup();
//                     manipulation_model->use_context(planner_info->planning_context_name);
//                 }

//                 // Setup the connection planners (via planner info)
//                 foreach(planner_info_t* planner_info, connection_info_map | boost::adaptors::map_values)
//                 {
//                     manipulation_model->use_context(planner_info->construction_context_name);

//                     //We have to override the distance function to be the end-effector distance
//                     if( use_ee_dist_in_planners )
//                     {
//                         ee_distances.push_back( new end_effector_distance_t() );
//                         ee_distances.back()->link_manipulation_model( manipulation_model );
//                         planner_info->specification->metric->link_distance_function( ee_distances.back() );
//                     }


//                     planner_info->setup(manipulation_model);
//                     planner_info->planner->setup();
//                     if (shelf_plane_goal != NULL)
//                     {
//                         PRX_PRINT ("Found shelf plane goal> Linking into Connection Planner", PRX_TEXT_BLUE);
//                         shelf_plane_goal->setup(manipulation_model, shelf_plane, half_space_side);
//                         shelf_plane_goal->link_space(manipulation_model->get_state_space());
//                         planner_info->query->set_goal(shelf_plane_goal);
//                     }
//                     manipulation_model->use_context(planner_info->planning_context_name);
//                 }

//                 //Setup the grasping planner
//                 grasping_planner->setup();

//                 // Revert context
//                 manipulation_model->use_context(old_context);
//                 grasping_query = new grasping_query_t();
//                 feasible_query = new grasping_query_t();

//             }

//             void grasp_rrt_tp_t::reset()
//             {
                
//             }

//             void grasp_rrt_tp_t::link_world_model(world_model_t * const model)
//             {
//                 PRX_PRINT("=================================================", PRX_TEXT_BLUE);
//                 PRX_PRINT(" Calling link_world_model() for manipulation tp!", PRX_TEXT_CYAN);
//                 PRX_PRINT("=================================================", PRX_TEXT_BLUE);
//                 task_planner_t::link_world_model(model);
                
//                 manipulation_model = dynamic_cast<manipulation_world_model_t*>(model);
//                 if(manipulation_model == NULL)
//                     PRX_FATAL_S("The manipulation task planner can work only with manipulation world model!");
//                 grasping_planner->link_world_model(model);

//                 neighborhood_sampler->link_world_model(model);
//             }

//             const statistics_t* grasp_rrt_tp_t::get_statistics()
//             {
//                 PRX_WARN_S("Get statistics for manipulation task planner is not implemented!");
//                 return new statistics_t();
//             }

//             void grasp_rrt_tp_t::link_specification(specification_t* new_spec)
//             {
//                 task_planner_t::link_specification(new_spec);
                
//                 manip_specification = dynamic_cast< manipulation_specification_t* >( new_spec );
//                 if( manip_specification == NULL )
//                 {
//                     PRX_FATAL_S("Manipulation task planner requires a specification of type manipulation_specification!");
//                 }
                
//                 validity_checker = manip_specification->validity_checker;

//                 tmp_constraint = validity_checker->alloc_constraint();
                
//                 // Ask the validity check to get the constraint names
//                 validity_checker->get_constraint_names( constraint_names );
//             }

//             void grasp_rrt_tp_t::link_query(query_t* new_query)
//             {
//                 task_planner_t::link_query(new_query);
//                 manipulation_query = dynamic_cast<manipulation_query_t*>(new_query);
//                 if(manipulation_query == NULL)
//                     PRX_FATAL_S("The manipulation task planner operates only over a manipulation query as input!");

//                 PRX_DEBUG_COLOR("link query grasp_rrt_tp context name: " << manipulation_query->manipulation_context_name, PRX_TEXT_BROWN);
//                 manipulation_model->use_context(manipulation_query->manipulation_context_name);
//                 current_manipulation_context_info = manipulation_model->get_current_manipulation_info();
//                 active_planner = planner_info_map[current_manipulation_context_info->arm_context_name];
//                 manipulation_query->link_spaces(current_manipulation_context_info->full_arm_state_space,current_manipulation_context_info->full_arm_control_space);

//                 object_constraints_checker_t* constraint_checker = dynamic_cast< object_constraints_checker_t* >( validity_checker );
//                 if( constraint_checker != NULL )
//                 {
//                     std::string object_name = (manipulation_query->object != NULL ? manipulation_query->object->get_pathname() : "" );
//                     constraint_checker->setup_checker( object_name );

//                     object_constraints_checker_t* planner_constraint_checker = dynamic_cast< object_constraints_checker_t* >( active_planner->specification->validity_checker );
//                     if( planner_constraint_checker != NULL )
//                     {
//                         planner_constraint_checker->setup_checker( object_name );
//                     }
//                     else
//                     {
//                         PRX_FATAL_S("Manipulation task planner is set up to do soft constraints, but its planner " << active_planner->planner->get_name() << " does not have a object constraints validity checker!");
//                     }
//                 }

//                 engage_grasp_plan.link_control_space(current_manipulation_context_info->full_arm_control_space);
//             }

//             bool grasp_rrt_tp_t::serialize()
//             {
//                 std::string old_context = manipulation_model->get_current_context();
//                 foreach(planner_info_t* planner_info, planner_info_map | boost::adaptors::map_values)
//                 {
//                     manipulation_model->use_context(planner_info->planning_context_name);
//                     if(!planner_info->planner->serialize())
//                         PRX_FATAL_S("Planner " << planner_info->planner->get_name() << " failed to serialize!");
//                 }
//                 manipulation_model->use_context(old_context);
//                 return true;
//             }

//             bool grasp_rrt_tp_t::deserialize()
//             {
//                 std::string old_context = manipulation_model->get_current_context();
//                 foreach(planner_info_t* planner_info, planner_info_map | boost::adaptors::map_values)
//                 {
//                     manipulation_model->use_context(planner_info->planning_context_name);
//                     if(!planner_info->planner->deserialize())
//                         PRX_FATAL_S("Planner " << planner_info->planner->get_name() << " failed to deserialize!");
//                 }
//                 manipulation_model->use_context(old_context);                
//                 return true;
//             }

//             std::vector< grasp_t > grasp_rrt_tp_t::get_grasps( std::vector<sim::state_t* >& input_states )
//             {
//                 std::vector< grasp_t > ret_vec;
//                 resolve_grasp_query( input_states, GRASPING_CONFIGURATIONS, 1, NULL );
//                 for( unsigned i=0; i < grasping_query->grasp_data[0].size(); ++i )
//                 {
//                     ret_vec.push_back( *(grasping_query->grasp_data[0][i]->relative_grasp) );
//                 }
//                 return ret_vec;
//             }


//             bool grasp_rrt_tp_t::succeeded() const
//             {
//                 return true;
//                 //return false;
//             }

//             bool grasp_rrt_tp_t::execute()
//             {
                
//                 std::string old_context = manipulation_model->get_current_context();

//                 // The motion planners must execute
//                 foreach(planner_info_t* planner_info, planner_info_map | boost::adaptors::map_values)
//                 {
//                     manipulation_model->use_context(planner_info->construction_context_name);
//                     try
//                     {
//                         PRX_INFO_S("Executing "<<planner_info->planner->get_name()<<" in context "<<manipulation_model->get_current_context());
//                         if(!planner_info->planner->execute())
//                             PRX_FATAL_S("Planner " << planner_info->planner->get_name() << " failed to execute!");
//                     }
//                     catch( stopping_criteria_t::stopping_criteria_satisfied e )                
//                     {

//                     }
//                 }

//                 // The connection planners must also execute
//                 foreach(planner_info_t* planner_info, connection_info_map | boost::adaptors::map_values)
//                 {
//                     manipulation_model->use_context(planner_info->construction_context_name);
//                     try
//                     {
//                         PRX_INFO_S("Executing "<<planner_info->planner->get_name()<<" in context "<<manipulation_model->get_current_context());
//                         if(!planner_info->planner->execute())
//                             PRX_FATAL_S("Planner " << planner_info->planner->get_name() << " failed to execute!");
//                     }
//                     catch( stopping_criteria_t::stopping_criteria_satisfied e )                
//                     {

//                     }
//                 }
//                 manipulation_model->use_context(old_context);

//                 if( serialize_flag )
//                     serialize();
//                 if (visualize)
//                 {
//                     update_vis_info();
//                     ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->send_geometries();
//                 }

//                 return true;
//             }

//             void grasp_rrt_tp_t::resolve_query()
//             {
//                 PRX_PRINT("=============================================================",PRX_TEXT_RED);
//                 PRX_PRINT("  Manip TP :: Resolve Query : Type: (" << manipulation_query->task_mode << ")  Search: (" << manipulation_query->astar_mode << ")",PRX_TEXT_MAGENTA);
//                 PRX_PRINT("=============================================================",PRX_TEXT_RED);

//                 if (manipulation_query->task_mode == TASK_MOVE)
//                 {
//                     manipulation_query->found_path = execute_move( manipulation_query->plan, manipulation_query->path_constraints, manipulation_query->manipulator_initial_state, manipulation_query->manipulator_target_state );
//                 }
//                 else
//                 {
//                     resolve_task();
//                 }

//                 PRX_PRINT("=======================================",PRX_TEXT_BLUE);
//                 PRX_PRINT("  Manip TP :: Resolve Query End", PRX_TEXT_MAGENTA);
//                 PRX_PRINT("=======================================",PRX_TEXT_BLUE);

//                 PRX_PRINT("====================================", PRX_TEXT_BLUE);
//                 PRX_PRINT("  Path ended up with " << manipulation_query->path_constraints->print(), PRX_TEXT_CYAN);
//                 PRX_PRINT("====================================", PRX_TEXT_BLUE);

//             }

//             void grasp_rrt_tp_t::resolve_task()
//             {
//                 // get the current grasping mode
//                 int previous_grasping_mode = manipulation_model->get_current_grasping_mode();

//                 // Set up the task planner based on what pick() is currently doing
//                 manipulation_model->enable_IK_steering(IK_steer_movements,manipulation_model->get_current_context());
//                 manipulation_model->get_state_space()->copy_from_point(manipulation_query->manipulator_initial_state);
//                 PRX_DEBUG_COLOR("--Manip tp :: RESOLVE TASK "<<manipulation_model->get_full_state_space()->print_memory(1),PRX_TEXT_MAGENTA)

//                 // Store the object states. In Pick and Place, both states should not be NULL.
//                 std::vector<state_t* > object_states;
//                 if (manipulation_query->object_initial_state != NULL)
//                     object_states.push_back(manipulation_query->object_initial_state);  
//                 if (manipulation_query->object_target_state != NULL)
//                     object_states.push_back(manipulation_query->object_target_state);  

//                 //Helping variable for the best plan.
//                 plan_t best_plan(current_manipulation_context_info->full_arm_control_space);
//                 trajectory_t solution_trajectory( current_manipulation_context_info->full_arm_state_space );
//                 constraints_t* best_constraints = validity_checker->alloc_constraint();
//                 double best_cost = PRX_INFINITY;

//                 //The grasping data that eventually will work
//                 int best_grasp_index = -1;
//                 grasp_data_t* best_grasp_data;

//                 // Remember the state of the manipulation model
//                 state_t* actual_start_state = manipulation_model->get_state_space()->clone_point(manipulation_query->manipulator_initial_state);
//                 state_t* actual_goal_state = NULL;
//                 if(manipulation_query->manipulator_target_state != NULL)
//                     actual_goal_state = manipulation_model->get_state_space()->clone_point(manipulation_query->manipulator_target_state);

//                 if( manipulation_query->grasp_evaluation == GRASP_EXHAUSTIVE )
//                 {
//                     // Resolve grasp query
//                     // Gets all the grasping plans that will work for the specific states of the object.
//                     // if(resolve_grasp_query(object_states, GRASPING_PLANS, manipulation_query->default_open_mode, NULL))
//                     // {
//                     //     PRX_PRINT("--Manip tp :: Pick : Optimal solution: Found " << grasping_query->grasp_data[0].size() << " grasps to test" ,PRX_TEXT_BROWN);

//                     //     //At the beggining the path constraints has all the possible valid constraints in order to be able to select a path with less constraints.
//                     //     constraints_t* solution_constraints = validity_checker->alloc_constraint();
//                     //     *best_constraints =  *manipulation_query->valid_constraints;

//                     //     //Helping variable for the optimal mode.
//                     //     plan_t solution_plan(current_manipulation_context_info->full_arm_control_space);

//                     //     //Iterate over all the grasping data in order to find the optimal one, smaller plan with less constraints. 
//                     //     for( unsigned grasp_index = 0; grasp_index < grasping_query->grasp_data[0].size(); ++grasp_index )
//                     //     {
//                     //         //Get the vertical slice of grasp data for each state index for this particular grasp
//                     //         std::vector< grasp_data_t* > data;
//                     //         for( unsigned state_index=0; state_index < grasping_query->grasp_data.size(); ++state_index )
//                     //         {
//                     //             data.push_back( grasping_query->grasp_data[state_index][grasp_index] );
//                     //         }

//                     //         //Make sure to put the object in the query's start state
//                     //         manipulation_query->object->get_state_space()->copy_from_point(manipulation_query->object_initial_state);

//                     //         //Clear out any previous solution we may have had
//                     //         solution_plan.clear();
//                     //         solution_constraints->clear();

//                     //         //Compute a plan for this grasp which makes it through all of the states we need (again, only should have multiple states in P&P)
//                     //         if( compute_motion_plan( solution_plan, solution_constraints, object_states, data, actual_start_state, actual_goal_state ) )
//                     //         {
//                     //             //Propagate the plan we got in order to compute its cost
//                     //             manipulation_model->propagate_plan( actual_start_state, solution_plan, solution_trajectory );
//                     //             //TODO: Still have to figure out how we can be using cost functions
//                     //             double solution_cost = solution_trajectory.length();
                                
//                     //             // If we have better constraints OR we do not have worse constraints and a better cost
//                     //             if( (*solution_constraints < *best_constraints) || ( !(*best_constraints < *solution_constraints) && (solution_cost < best_cost) ))
//                     //             {
//                     //                 // Then remember this grasp as our best bet so far
//                     //                 best_plan = solution_plan;
//                     //                 best_cost = solution_cost;
//                     //                 *best_constraints = *solution_constraints;
//                     //                 best_grasp_index = grasp_index;
//                     //             }
//                     //         }
//                     //     }

//                     //     validity_checker->free_constraint(solution_constraints);
//                     // }
//                 }
//                 else if ( manipulation_query->grasp_evaluation == GRASP_GREEDY )
//                 {
//                     //Extract from the grasping planner the configurations we consider for evaluation
//                     if(resolve_grasp_query(object_states, GRASPING_CONFIGURATIONS, manipulation_query->default_open_mode, NULL))
//                     {
//                         // foreach(std::string connection_planner_name, connection_planner_names)
//                         // {
//                         //     grasp_rrt_specification_t* grasp_rrt_specification = dynamic_cast<grasp_rrt_specification_t* >(output_specifications[connection_planner_name]);
//                         //     grasp_rrt_specification->grasp_evaluator->link_grasp_database(grasping_query->grasp_data[0]);
//                         // }
//                         std::vector< grasp_t > grasp_vec;
//                         for( unsigned i=0; i < grasping_query->grasp_data[0].size(); ++i )
//                         {
//                             grasp_vec.push_back( *(grasping_query->grasp_data[0][i]->relative_grasp) );
//                         }


//                         _mp_clock.reset();
//                         // while(_mp_clock.measure()<300)
//                         // {
//                         compute_motion_plan(best_plan, best_constraints, actual_goal_state, object_states, actual_start_state, grasp_vec);
//                         PRX_PRINT("The plan returned is "<<best_plan.print(3), PRX_TEXT_GREEN);
//                         // }


//                         //Then, for each candidate grasping configuration
//                         // for(unsigned grasp_index = 0; grasp_index<grasping_query->grasp_data[0].size() && best_grasp_index == -1; ++grasp_index)                            
//                         // {

//                         //     //Perform a query for the specific configuration we want
//                         //     feasible_query->clear();
//                         //     feasible_query->setup(current_manipulation_context_info->full_arm_state_space, current_manipulation_context_info->full_arm_control_space, GRASPING_PLANS, manipulation_query->astar_mode, manipulation_query->task_mode, manipulation_query->manipulator_retracts, manipulation_query->object, object_states, manipulation_query->retraction_config, grasping_query->grasp_data[0][grasp_index]->relative_grasp,  manipulation_query->valid_constraints);
//                         //     grasping_planner->link_query(feasible_query);
                            
//                         //     current_manipulation_context_info->manipulator->scale_control_bounds(0.1);
//                         //     grasping_planner->resolve_query();
//                         //     current_manipulation_context_info->manipulator->scale_control_bounds(10);

//                         //     PRX_PRINT("Tested grasp [" << grasp_index << "]", PRX_TEXT_BLUE);

//                         //     //If we found a grasping plan
//                         //     if(feasible_query->found_grasp)
//                         //     {

//                         //         //Get the vertical slice of grasp data for each state index for this particular grasp
//                         //         std::vector< grasp_data_t* > data;
//                         //         for( unsigned state_index=0; state_index < feasible_query->grasp_data.size(); ++state_index )
//                         //         {
//                         //             data.push_back( feasible_query->grasp_data[state_index][0] );
//                         //         }

//                         //         PRX_PRINT ("Found greedy grasp!", PRX_TEXT_BROWN);

//                         //         //Clear out the plan and constraints for what we are testing
//                         //         best_plan.clear();
//                         //         best_constraints->clear();

//                         //         //See if we can get a whole motion plan
//                         //         if( compute_motion_plan(best_plan, best_constraints, object_states, data, actual_start_state, actual_goal_state))
//                         //         {
//                         //             best_grasp_index = grasp_index;
//                         //         }
//                         //     }
//                         // }
//                     }
//                 }
//                 else //SUGGESTED
//                 {
//                     // if(manipulation_query->task_mode != TASK_PLACE )
//                     // {
//                     //     PRX_ASSERT(manipulation_query->suggested_grasp != NULL);
//                     // }

//                     // _grasp_clock.reset();
//                     // if(resolve_grasp_query(object_states, GRASPING_PLANS, manipulation_query->default_open_mode, manipulation_query->suggested_grasp))
//                     // {
//                     //     _grasp_success_time += _grasp_clock.measure();
//                     //     _grasp_success_count++;

//                     //     //Get the vertical slice of grasp data for each state index for this particular grasp
//                     //     std::vector< grasp_data_t* > data;
//                     //     for( unsigned state_index=0; state_index < grasping_query->grasp_data.size(); ++state_index )
//                     //     {
//                     //         data.push_back( grasping_query->grasp_data[state_index][0] );
//                     //     }

//                     //     best_plan.clear();
//                     //     best_constraints->clear();

//                     //     _mp_clock.reset();
//                     //     if( compute_motion_plan(best_plan, best_constraints, object_states, data, actual_start_state, actual_goal_state))
//                     //     {
//                     //         _mp_success_time+=_mp_clock.measure();
//                     //         _mp_success_count++;
//                     //         // append_to_stat_file("\n||||||||||||||||MP Success time:  "+boost::lexical_cast<std::string>(_mp_success_time/(double)(_mp_success_count)));
//                     //         best_grasp_index = 0;
//                     //     }
//                     //     else
//                     //     {
//                     //         _mp_failure_time+=_mp_clock.measure();
//                     //         _mp_failure_count++;
//                     //         // append_to_stat_file("\n||||||||||||||||MP Failure time:  "+boost::lexical_cast<std::string>(_mp_failure_time/(double)(_mp_failure_count)));

//                     //     }
//                     // }
//                     // else
//                     // {
//                     //     _grasp_failure_time+=_grasp_clock.measure();
//                     //     _grasp_failure_count++;
//                     //     // append_to_stat_file(    "\n||||||||||||||||Grasp failure time: "+boost::lexical_cast<std::string>(_grasp_failure_time/(double)(_grasp_failure_count)));
//                     // }
//                 }

                
//                 //If we found a valid grasp!
//                 if(manipulation_query->found_path)
//                 {
//                     PRX_PRINT("Found a grasp we are happy with: [" << best_grasp_index << "]", PRX_TEXT_GREEN);
//                     manipulation_query->plan += best_plan;
//                     manipulation_query->path_constraints->merge( best_constraints );

//                     // manipulation_query->relative_grasp = grasping_query->grasp_data[0][best_grasp_index]->relative_grasp;
//                     PRX_DEBUG_COLOR("--Manip TP :: Pick&Place End -- True "<<manipulation_model->get_full_state_space()->print_memory(1),PRX_TEXT_MAGENTA)
//                 }
//                 else
//                 {
//                     PRX_DEBUG_COLOR("Failed grasp query! Reason: " << grasping_query->reason_for_failure, PRX_TEXT_LIGHTGRAY);
//                 }

//                 // Restore the grasping mode
//                 restore_grasping_mode( previous_grasping_mode, actual_start_state );

//                 // Free the points
//                 manipulation_model->get_state_space()->free_point(actual_start_state);
//                 if (actual_goal_state != NULL)
//                     manipulation_model->get_state_space()->free_point(actual_goal_state);

//                 validity_checker->free_constraint(best_constraints);
//             }


//             void grasp_rrt_tp_t::restore_grasping_mode( int previous_grasping_mode, sim::state_t* start_state)
//             {

//                 // if we ended up in a different grasping mode, make sure we return to the original grasping mode
//                 if(manipulation_model->get_current_grasping_mode() != previous_grasping_mode)
//                 {
//                     engage_grasp_plan.clear();
//                     manipulation_model->push_state(start_state);
//                     manipulation_model->engage_grasp(engage_grasp_plan,previous_grasping_mode, true);
//                 }

//             }

//             bool grasp_rrt_tp_t::resolve_grasp_query(std::vector<state_t*>& object_states, grasping_query_type_t grasping_query_mode, int default_open_mode, grasp_t* suggested_grasp)
//             {
//                 plan_t plan;
//                 int current_end_effector_mode = -1;
//                 //We store the current state of the manipulator to restore the manipulator after the grasping planner is finished.
//                 state_t* our_full_manipulator = current_manipulation_context_info->full_manipulator_state_space->alloc_point();
//                 state_t* push_the_state = manipulation_model->get_state_space()->alloc_point();

//                 if(current_manipulation_context_info->is_end_effector_closed())
//                 {
//                     current_end_effector_mode = manipulation_model->get_current_grasping_mode();
//                     //We are going to open the hand and use the default_open_mode for that. 
//                     engage_grasp_plan.clear();
//                     manipulation_model->push_state(push_the_state);
//                     manipulation_model->engage_grasp(engage_grasp_plan, default_open_mode, false);
//                     current_manipulation_context_info->full_manipulator_state_space->copy_to_point(our_full_manipulator);
//                 }

//                 //We need the grasping planner's validity checker to have the right idea about constraint names

//                 grasping_query->clear();
//                 grasping_query->setup(current_manipulation_context_info->full_arm_state_space, current_manipulation_context_info->full_arm_control_space, grasping_query_mode, manipulation_query->astar_mode, manipulation_query->task_mode, manipulation_query->manipulator_retracts, manipulation_query->object, object_states, manipulation_query->retraction_config, suggested_grasp, manipulation_query->valid_constraints);

//                 // APC SPECIFIC: Scaling the bounds to slow down grasping plans from IK steering
//                 //current_manipulation_context_info->manipulator->scale_control_bounds(0.1);
//                 grasping_planner->link_query( grasping_query );
//                 grasping_planner->resolve_query();
//                 //current_manipulation_context_info->manipulator->scale_control_bounds(10);

//                 current_manipulation_context_info->full_manipulator_state_space->copy_from_point(our_full_manipulator);            
//                 manipulation_model->get_state_space()->free_point(push_the_state);
//                 return grasping_query->found_grasp;
//             }

//             void grasp_rrt_tp_t::generate_goal(std::vector<state_t*>& candidate_goals, state_t* open_full_start_state, state_t* object_state, grasp_data_t* grasp_data)
//             {
//                 // Remember the grasping mode and the world model state before we do anything else...
//                 int previous_grasping_mode = manipulation_model->get_current_grasping_mode();
//                 // TODO: Prealloc this
//                 state_t* original_world_model_state = manipulation_model->get_state_space()->alloc_point();
                
//                 PRX_PRINT("=====================================================", PRX_TEXT_LIGHTGRAY);
//                 PRX_PRINT("= Generating a goal: Points in metric:  " << active_planner->specification->metric->get_nr_points(), PRX_TEXT_CYAN);
//                 PRX_PRINT("=====================================================", PRX_TEXT_LIGHTGRAY);
//                 PRX_ASSERT (grasp_data->retracted_open_state != NULL || grasp_data->retracted_closed_state != NULL );

//                 // Use the retracted state that is not NULL
//                 state_t* full_query_state = ( grasping_query->task_mode == TASK_PICK && !grasping_query->manipulator_retracts ) ?  grasp_data->retracted_open_state : grasp_data->retracted_closed_state;

//                 current_manipulation_context_info->full_arm_state_space->copy_from_point(full_query_state);

//                 state_t* converted_query_state = current_manipulation_context_info->arm_state_space->alloc_point();

//                 const std::vector< const abstract_node_t* > nearest_points = active_planner->specification->metric->multi_query(converted_query_state, num_candidate_connections);
                
//                 PRX_PRINT("Querying from: " << current_manipulation_context_info->arm_state_space->print_point( converted_query_state, 4 ), PRX_TEXT_LIGHTGRAY);

//                 plan_t connection_plan( manipulation_model->get_control_space() );
//                 trajectory_t connection_path( manipulation_model->get_state_space() );

//                 // --- Connection Verification Phase ---
                
//                 // Create the full_closed_start_state
//                 state_t* closed_full_start_state = manipulation_model->get_state_space()->clone_point(open_full_start_state);
//                 if ( !(manipulation_query->task_mode == TASK_PICK && !manipulation_query->manipulator_retracts) )
//                 {
//                     manipulation_model->push_state(open_full_start_state);
//                     //Engage grasp, so the manipulator will be moving that object
//                     manipulation_model->engage_grasp( connection_plan, grasp_data->relative_grasp->grasping_mode, false );
//                     connection_plan.clear();
//                     closed_full_start_state = manipulation_model->get_state_space()->alloc_point();
//                 }
                
//                 // -- Compute the correct relative config
                
//                 // Place object in the correct state
//                 manipulation_query->object->get_state_space()->copy_from_point( object_state );
//                 //Move the manipulator to the released state
//                 manipulation_model->push_state( grasp_data->releasing_state );


//                 //DEBUG: get the ee config at the releasing state
//                 config_t releasing_config;
//                 manipulation_model->FK( releasing_config );


//                 // Everything but PICK w/out retract will reason about holding the object during connection verification
//                 if ( !(manipulation_query->task_mode == TASK_PICK && !manipulation_query->manipulator_retracts) )
//                 {
//                     //Engage grasp, so the manipulator will be moving that object
//                     manipulation_model->engage_grasp( connection_plan, grasp_data->relative_grasp->grasping_mode, true );
//                     connection_plan.clear();
//                 }

//                 active_planner->setup_search( LAZY_SEARCH, manipulation_query->valid_constraints, constraint_names );
//                 // Remember the old context
//                 std::string old_context = manipulation_model->get_current_context();

//                 manipulation_model->push_state(closed_full_start_state);

//                 // Switch to planning context
//                 manipulation_model->use_context(active_planner->planning_context_name);

//                 // Get the connected Arm Start State
//                 state_t* converted_start_state = manipulation_model->get_state_space()->alloc_point();


//                 foreach(const abstract_node_t* node, nearest_points)
//                 {
//                     bool is_connected = false;
//                     if(skip_connectivity_check)
//                     {
//                         is_connected = true;
//                     }
//                     else
//                     {
//                         is_connected = active_planner->compute_solution(connection_plan, tmp_constraint, converted_start_state, node->point, false );
//                     }

//                     if (is_connected)
//                     {
//                         PRX_PRINT ("VALID CONNECTED GOAL STATE! State: " << manipulation_model->get_state_space()->print_point( node->point, 5 ) << "  C-space distance: " << manipulation_model->get_state_space()->distance( converted_query_state, node->point ), PRX_TEXT_GREEN);
//                         candidate_goals.push_back(node->point);
//                     }
//                     else
//                     {
//                         PRX_PRINT ("INVALID CONNECTED GOAL STATE! State: " << manipulation_model->get_state_space()->print_point( node->point, 5 ) << "  C-space distance: " << manipulation_model->get_state_space()->distance( converted_query_state, node->point ), PRX_TEXT_RED);
//                     }

//                     //Debug: Let's see what the workspace ee distances look like?
//                     config_t near_config;
//                     manipulation_model->push_state( node->point );
//                     manipulation_model->FK( near_config );
                    
//                     PRX_PRINT("Position distance: " << near_config.get_position().distance( releasing_config.get_position() ), PRX_TEXT_LIGHTGRAY );
                    

//                     active_planner->query->restart_astar_search = false;
//                 }

//                 manipulation_model->get_state_space()->free_point(converted_start_state);

//                 // Restore context back to normal
//                 manipulation_model->use_context(old_context);

//                 restore_grasping_mode( previous_grasping_mode, original_world_model_state );

//                 manipulation_model->get_state_space()->free_point(original_world_model_state);
//                 manipulation_model->get_state_space()->free_point(closed_full_start_state);


//             }

//             bool grasp_rrt_tp_t::compute_connection_plan(state_t* start_state, state_t* object_state, grasp_data_t* grasp_data, unsigned state_index )
//             {

//                 PRX_PRINT("We have gotten into the compute connection plan!", PRX_TEXT_RED);
//                 std::vector<state_t*> candidate_goals;
//                 state_t* connection_start_state = start_state; // renaming

//                 // Generate candidate goal states (only when we are not trying to get out of the shelf)
//                 if (shelf_plane_goal == NULL)
//                 {
//                     generate_goal(candidate_goals, connection_start_state, object_state, grasp_data);
                    
//                     if (candidate_goals.empty())
//                     {
//                         append_to_stat_file("\n   Empty candidate goals");
//                         return false;
//                     }
//                 }


//                 // Try to retrieve the correct connection planner info
//                 planner_info_t* active_connection_planner = (connection_info_map.find(active_planner->planning_context_name) != connection_info_map.end()) ? connection_info_map[active_planner->planning_context_name] : NULL;

//                 // TODO: Prealloc somewhere?
//                 plan_t connection_plan( manipulation_model->get_control_space() );
//                 trajectory_t retracting_path( manipulation_model->get_state_space() );
//                 trajectory_t reaching_path( manipulation_model->get_state_space() );
//                 state_t* reached_connection_state = manipulation_model->get_state_space()->alloc_point();

//                 // //====================================================
//                 // //= Planning with the Object (always need released)
//                 // //====================================================

//                 // We can skip this object retraction plan only if we are solving PICK without RETRACT
//                 if( !( manipulation_query->task_mode == TASK_PICK && !manipulation_query->manipulator_retracts ) )
//                 {
//                     PRX_PRINT("=====================================================", PRX_TEXT_BROWN);
//                     PRX_PRINT("= Computing a Retraction Plan (OBJECT)! ", PRX_TEXT_GREEN);
//                     PRX_PRINT("=====================================================", PRX_TEXT_BROWN);
                    
//                     //Move the object back to its original state according to the query
//                     manipulation_query->object->get_state_space()->copy_from_point( object_state );
//                     //Move the manipulator to the released state
//                     manipulation_model->push_state( grasp_data->releasing_state );
//                     //Engage grasp, so the manipulator will be moving that object
//                     manipulation_model->engage_grasp( connection_plan, grasp_data->relative_grasp->grasping_mode, true );

//                     //manipulation_model->propagate_plan(grasp_data->grasping_state, grasp_data->retracting_plan, retracting_path);

//                     // Clear the tmp plan for use
//                     connection_plan.clear();
//                     append_to_stat_file("\n       Retracting with object");
//                     // if( !manipulation_model->IK_steering( connection_plan, grasp_data->retracted_closed_state, grasp_data->grasping_state, retraction_config ) )
//                     if( !resolve_connection( connection_plan, retracting_path, grasp_data->closed_retracting_constraints, reached_connection_state, candidate_goals, active_connection_planner, grasp_data->retracted_closed_state, connection_start_state ) )
//                     {
//                         PRX_PRINT ("FAILURE: Retracting with object failed", PRX_TEXT_LIGHTGRAY);
//                         manipulation_model->get_state_space()->free_point( reached_connection_state );
//                         return false;
//                     }
//                 }

//                 //First, Retracting with the object; we have to validity check the retraction IK trajectory with the object
//                 if( (manipulation_query->task_mode == TASK_PICK && manipulation_query->manipulator_retracts) || manipulation_query->task_mode == TASK_PICK_AND_MOVE || (manipulation_query->task_mode == TASK_PICK_AND_PLACE && state_index == 0) )
//                 {
//                     manipulation_model->get_state_space()->copy_point( grasp_data->retracted_closed_state, reached_connection_state );
//                     grasp_data->retracting_plan += connection_plan;
//                 }

//                 //Second, Reaching with the object, Compute and validate the reaching trajectory with the object
//                 if( (manipulation_query->task_mode == TASK_PLACE && manipulation_query->manipulator_retracts) || (manipulation_query->task_mode == TASK_PICK_AND_PLACE && state_index == 1) || manipulation_query->task_mode == TASK_PLACE)
//                 {
//                     //Reverse the retraction trajectory to make it a baseline for the reaching trajectory
//                     reaching_path.clear();
//                     reaching_path.reverse_trajectory( retracting_path );
//                     //Re-steer over all of the pairs of states in the reaching trajectory to get a reaching plan
//                     connection_plan.clear();
//                     for( unsigned i=0; i < reaching_path.size() -1; ++i )
//                     {
//                         manipulation_model->steering_function( reaching_path[i], reaching_path[i+1], connection_plan );
//                     }
//                     reaching_path.clear();
//                     //Repropagate the reaching plan to get an updated/refined/true reaching trajectory
//                     manipulation_model->propagate_plan( reached_connection_state, connection_plan, reaching_path );

//                     //DEBUG DEBUG DEBUG
//                     // PRX_PRINT("Goal (reached_connection_state): " << manipulation_model->get_state_space()->print_point(reached_connection_state, 5), PRX_TEXT_GREEN );
//                     // PRX_PRINT("Repropagated    : " << manipulation_model->get_state_space()->print_point(reaching_path.back(), 5), PRX_TEXT_GREEN );
                    
//                     //Validate and generate constraints over the resulting reaching trajectory
//                     if( !validity_checker->validate_and_generate_constraints( grasp_data->closed_reaching_constraints, reaching_path ) )
//                     {
//                         //If that path ended up not being valid, we must also report failure here
//                         PRX_PRINT ("FAILURE: Reaching with object failed", PRX_TEXT_LIGHTGRAY);
//                         manipulation_model->get_state_space()->free_point( reached_connection_state );
//                         return false;
//                     }

//                     manipulation_model->get_state_space()->copy_point( grasp_data->retracted_closed_state, reached_connection_state );
//                     connection_plan += grasp_data->reaching_plan;
//                     grasp_data->reaching_plan = connection_plan;
//                 }

//                 // //====================================================
//                 // //= Planning without the Object (always need released)
//                 // //====================================================

//                 if( manipulation_query->task_mode != TASK_PLACE )
//                 {
//                     PRX_PRINT("=====================================================", PRX_TEXT_BROWN);
//                     PRX_PRINT("= Computing a Retraction Plan (NO OBJECT)! ", PRX_TEXT_RED);
//                     PRX_PRINT("=====================================================", PRX_TEXT_BROWN);
                    
//                     //Move the object to it's original state
//                     manipulation_query->object->get_state_space()->copy_from_point( object_state );
//                     //Move the manipulator to the released state
//                     manipulation_model->push_state( grasp_data->releasing_state );
//                     //Disengage grasp, so the manipulator is not holding the object
//                     manipulation_model->engage_grasp( connection_plan, grasp_data->relative_grasp->release_mode, true );

//                     //Do the actual planning and see if that works
//                     connection_plan.clear();
//                     retracting_path.clear();

//                     append_to_stat_file("\n       Retracting without object");
//                     // if( !manipulation_model->IK_steering( connection_plan, grasp_data->retracted_closed_state, grasp_data->grasping_state, retraction_config ) )
//                     if( !resolve_connection( connection_plan, retracting_path, grasp_data->open_retracting_constraints, reached_connection_state, candidate_goals, active_connection_planner, grasp_data->retracted_open_state, connection_start_state ) )
//                     {
//                         PRX_PRINT ("FAILURE: Retracting without object failed", PRX_TEXT_LIGHTGRAY);
//                         manipulation_model->get_state_space()->free_point( reached_connection_state );
//                         return false;
//                     }
             
//                 }
                
//                 if( (manipulation_query->task_mode == TASK_PLACE && manipulation_query->manipulator_retracts) || (manipulation_query->task_mode == TASK_PICK_AND_PLACE && state_index == 1) )
//                 {
//                     manipulation_model->get_state_space()->copy_point( grasp_data->retracted_open_state, reached_connection_state );
//                     grasp_data->retracting_plan += connection_plan;
//                 }

//                 if( manipulation_query->task_mode == TASK_PICK || manipulation_query->task_mode == TASK_PICK_AND_MOVE || (manipulation_query->task_mode == TASK_PICK_AND_PLACE && state_index == 0) )
//                 {
//                     //Reverse the retraction trajectory to make it a baseline for the reaching trajectory
//                     reaching_path.clear();
//                     reaching_path.reverse_trajectory( retracting_path );
//                     // PRX_PRINT("Initial reversed trajectory: " << reaching_path.print(5), PRX_TEXT_CYAN);
//                     //Re-steer over all of the pairs of states in the reaching trajectory to get a reaching plan
//                     connection_plan.clear();
//                     for( unsigned i=0; i < reaching_path.size() -1; ++i )
//                     {
//                         manipulation_model->steering_function( reaching_path[i], reaching_path[i+1], connection_plan );
//                     }
//                     //Repropagate the reaching plan to get an updated/refined/true reaching trajectory
//                     manipulation_model->propagate_plan( reached_connection_state, connection_plan, reaching_path );
                    
//                     //DEBUG DEBUG DEBUG
//                     // PRX_PRINT("Goal (reached_connection_state): " << manipulation_model->get_state_space()->print_point(reached_connection_state, 5), PRX_TEXT_GREEN );
//                     // PRX_PRINT("Repropagated    : " << manipulation_model->get_state_space()->print_point(*(reaching_path.begin()), 5), PRX_TEXT_GREEN );
//                     // PRX_PRINT("Repropagated reversed trajectory: " << reaching_path.print(5), PRX_TEXT_CYAN);
                    
//                     //Validate and generate constraints over the resulting reaching trajectory
//                     if( !validity_checker->validate_and_generate_constraints( grasp_data->open_reaching_constraints, reaching_path ) )
//                     {
//                         //If that path ended up not being valid, we must also report failure here
//                         PRX_PRINT ("FAILURE: Reaching without object failed", PRX_TEXT_LIGHTGRAY);
//                         manipulation_model->get_state_space()->free_point( reached_connection_state );
//                         return false;
//                     }

//                     manipulation_model->get_state_space()->copy_point( grasp_data->retracted_open_state, reached_connection_state );
//                     connection_plan += grasp_data->reaching_plan;
//                     grasp_data->reaching_plan = connection_plan;
//                 }
                
//                 manipulation_model->get_state_space()->free_point( reached_connection_state );
//                 return true;
//             }

//             bool grasp_rrt_tp_t::resolve_connection( plan_t& output_plan, trajectory_t& output_path, constraints_t* output_constraints, state_t* output_state, const std::vector< sim::state_t* >& goal_states, planner_info_t* input_connection_planner, state_t* grasp_start_state, state_t* connection_start_state)
//             {

//                 PRX_PRINT ("-----RESOLVE CONNECTION WITH GOAL STATES OF SIZE: " << goal_states.size(), PRX_TEXT_LIGHTGRAY );
//                 bool solution_found = false;

//                 // Remember the old context
//                 std::string old_context = manipulation_model->get_current_context();

//                 // Copy the Full Arm Start State into the model
//                 manipulation_model->push_state(grasp_start_state);

//                 // Switch to planning context
//                 manipulation_model->use_context(active_planner->planning_context_name);

//                 // Get the conerted Arm Start State
//                 state_t* converted_start_state = manipulation_model->get_state_space()->alloc_point();

//                 // If we have specified a connection planner and have found it in the mapping..
//                 if (input_connection_planner != NULL)
//                 {

//                     //DEBUG DEBUG
//                     // PRX_PRINT("About to compute a connection plan:", PRX_TEXT_BROWN);
//                     // PRX_PRINT("Start state: " << manipulation_model->get_state_space()->print_point(converted_start_state, 5), PRX_TEXT_CYAN );
//                     // foreach( state_t* gl, goal_states )
//                     // {
//                     //     PRX_PRINT("Goal: " << manipulation_model->get_state_space()->print_point(gl, 5), PRX_TEXT_LIGHTGRAY );
//                     // }

//                     // Compute the motion plan to one of the goal states
//                     input_connection_planner->setup_search( manipulation_query->astar_mode, manipulation_query->valid_constraints, constraint_names );   

//                     bool try_IK_steering_first = (shelf_plane_goal == NULL);
//                     solution_found = input_connection_planner->compute_solution(output_plan, output_constraints, converted_start_state, goal_states, try_IK_steering_first );
//                     if(solution_found)
//                     {
//                         if(input_connection_planner->query->get_goal()->size() > 0)
//                         {
//                             append_to_stat_file("\n     ___RRT");
//                         }
//                         else
//                         {
//                             append_to_stat_file("\n     ___IK.Steering");
//                         }
//                     }

//                     // // If we are trying to get out of the shelf, and have found a solution, we must update the start
//                     // // state that is used.
//                     // if (shelf_plane_goal != NULL && solution_found)
//                     // {
//                     //     append_to_stat_file("\n   RRT shelf plan succeeded.");

//                     //     manipulation_model->use_context(old_context);
//                     //     manipulation_model->propagate_plan( grasp_start_state , output_plan, output_path );
//                     //     manipulation_model->get_state_space()->copy_point( grasp_start_state, output_path.back() );
//                     //     manipulation_model->push_state(grasp_start_state);
//                     //     manipulation_model->get_state_space()->copy_point( grasp_start_state, output_path[0] );
//                     //     manipulation_model->use_context(active_planner->planning_context_name);
//                     //     manipulation_model->get_state_space()->copy_to_point(converted_start_state);       

//                     //     // Get new connection states   
//                     //     const std::vector< const abstract_node_t* > nearest_points = active_planner->specification->metric->multi_query(converted_start_state, num_candidate_connections);

//                     //     // We must compute a connection plan now, reset the solution found
//                     //     solution_found = false;   
//                     //     // First we attempt to IK steer to the connected states
//                     //     for(unsigned i=0; i < nearest_points.size() && !solution_found; ++i)
//                     //     {
//                     //         //manipulation_model->get_state_space()->copy_point(near_state, nearest_points[i]);
//                     //         state_t* near_state = dynamic_cast<state_t*>(nearest_points[i]->point); 
//                     //         solution_found = attempt_IK_steer(output_plan, output_constraints, converted_start_state, near_state);
//                     //         // If IK steering succeeded, let's try to compute a successful move
//                     //         if (solution_found)
//                     //         {
//                     //             append_to_stat_file("\n   IK Connection Success.");
//                     //             // Now we check the connectedness of the candidate state
//                     //             plan_t new_plan(manipulation_model->get_control_space());
//                     //             active_planner->setup_search( LAZY_SEARCH, manipulation_query->valid_constraints, constraint_names );
//                     //             solution_found = active_planner->compute_solution(new_plan, tmp_constraint, connection_start_state, near_state, false );

//                     //             if (solution_found)
//                     //             {
//                     //                 append_to_stat_file("\n   RRT shelf connection plan Success.");
//                     //                 output_plan += new_plan;
//                     //             }

//                     //         }
//                     //     }

//                     //     if (!solution_found)
//                     //     {
//                     //         append_to_stat_file("\n   RRT shelf connection plan failed.");
//                     //     }
//                     // }
//                     // else
//                     // {
//                     //     append_to_stat_file("\n   RRT shelf plan failed.");
//                     // }
//                 }
//                 // If we have no connection planner
//                 // then we must call IK steering to try to get to the candidate goal states
//                 else
//                 {
//                     solution_found = false;

//                     PRX_PRINT("No connection planner found. Trying IK steering", PRX_TEXT_RED);

//                     // IK steer to connection goals
//                     for( unsigned i=0; i<goal_states.size() && !solution_found; ++i )
//                     {
//                         state_t* goal_state = goal_states[i];
//                         solution_found = attempt_IK_steer(output_plan, output_constraints, converted_start_state, goal_state);

//                     } 
//                     if (!solution_found)
//                     {
//                         PRX_PRINT("Failed IK steering.",PRX_TEXT_BROWN);
//                     }
//                     else
//                     {
//                         append_to_stat_file("\n   ___IK Steering");
//                     }
//                 }

//                 manipulation_model->get_state_space()->free_point(converted_start_state);
//                 // Restore context back to normal
//                 manipulation_model->use_context(old_context);

//                 if (solution_found)
//                 {
//                     append_to_stat_file("\n   Connection plan Success.");
//                     manipulation_model->propagate_plan( grasp_start_state , output_plan, output_path );
//                     manipulation_model->get_state_space()->copy_point( output_state, output_path.back() );
//                 }
//                 else
//                 {
//                     append_to_stat_file("\n   Connection plan Failure.");
//                 }

//                 return solution_found;

//             }

//             bool grasp_rrt_tp_t::attempt_IK_steer(plan_t& output_plan, constraints_t* output_constraints, state_t* start_state, state_t* goal_state)
//             {

//                 PRX_PRINT("Trying IK steering first...",PRX_TEXT_BLUE);
//                 config_t goal_config;
//                 manipulation_model->get_state_space()->copy_from_point(goal_state);
//                 manipulation_model->FK(goal_config);
//                 state_t* result_state = manipulation_model->get_state_space()->alloc_point();
//                 plan_t new_plan(manipulation_model->get_control_space());
//                 bool success = manipulation_model->IK_steering( new_plan, result_state, start_state, goal_state, goal_config);
//                 manipulation_model->get_state_space()->free_point(result_state);
//                 trajectory_t traj(manipulation_model->get_state_space());
//                 if(success)
//                 {
//                     PRX_PRINT("IK steering success...",PRX_TEXT_GREEN);
//                     manipulation_model->propagate_plan(start_state,new_plan,traj);
//                     bool valid_constrained_trajectory = false;
//                     tmp_constraint->clear();
//                     valid_constrained_trajectory = validity_checker->validate_and_generate_constraints(tmp_constraint, traj);

//                     if(valid_constrained_trajectory)
//                     {
//                         PRX_PRINT("Valid IK steering trajectory...",PRX_TEXT_CYAN);
//                         if (manipulation_query->astar_mode == STANDARD_SEARCH || manipulation_query->astar_mode == LAZY_SEARCH )
//                         {
//                             if (manipulation_query->valid_constraints->has_intersection(tmp_constraint))
//                             {
//                                 return false;
//                             }
//                         }
//                         output_constraints->merge(tmp_constraint);

//                         manipulation_model->convert_plan(output_plan, manipulation_model->get_current_manipulation_info()->full_arm_control_space, new_plan, manipulation_model->get_control_space());
                        
//                         return true;
//                     }
//                 }
//                 return false;
//             }


//             void grasp_rrt_tp_t::get_neighborhood_connected_states(std::vector<state_t*>& neighborhood_states, config_t workspace_point, int number_of_neighbors, state_t* initial_state)
//             {
//                 neighborhood_states.clear();
//                 int max_tries = 500;
//                 sim::state_t* sample_point = manipulation_model->get_state_space()->alloc_point();
//                 constraints_t* constraints = validity_checker->alloc_constraint();
//                 plan_t plan(manipulation_model->get_current_manipulation_info()->full_arm_control_space);
//                 while(neighborhood_states.size()<number_of_neighbors && --max_tries>0 && ros::ok())
//                 {
//                     if(neighborhood_sampler->workspace_near_sample(workspace_point, neighborhood, manipulation_model->get_state_space(), sample_point))
//                     {
//                         plan.clear();
//                         if(execute_move(plan, constraints, sample_point, initial_state))
//                         {
//                             neighborhood_states.push_back(manipulation_model->get_state_space()->clone_point(sample_point));
//                             PRX_PRINT("Added a point to neighborhood states["<<neighborhood_states.size()<<"]", PRX_TEXT_RED);
//                         }
//                     }
//                 }
//                 manipulation_model->get_state_space()->free_point(sample_point);
//                 validity_checker->free_constraint(constraints);
//             }

//             bool grasp_rrt_tp_t::compute_motion_plan(sim::plan_t& the_plan, util::constraints_t* the_constraints, sim::state_t* reached_goal, const std::vector<sim::state_t*>& object_states, sim::state_t* the_start, std::vector<grasp_t>& the_datas)
//             {

//                 int previous_grasping_mode = manipulation_model->get_current_grasping_mode();
//                 // TODO: Prealloc this
//                 state_t* actual_start_state = manipulation_model->get_state_space()->alloc_point();
//                 append_to_stat_file("\n   ---Pick");

//                 planner_info_t* input_connection_planner = (connection_info_map.find(active_planner->planning_context_name) != connection_info_map.end()) ? connection_info_map[active_planner->planning_context_name] : NULL;

//                 bool solution_found = false;

//                 // plan_t output_plan( manipulation_model->get_control_space() );
//                 // Remember the old context
//                 std::string old_context = manipulation_model->get_current_context();

//                 // Copy the Full Arm Start State into the model
//                 manipulation_model->push_state(the_start);

//                 // Switch to planning context
//                 manipulation_model->use_context(active_planner->planning_context_name);

//                 // Get the conerted Arm Start State
//                 state_t* converted_start_state = manipulation_model->get_state_space()->alloc_point();
//                 constraints_t* output_constraints = validity_checker->alloc_constraint();
//                 //TODO:: Figure out where to dealloc output constraints
//                 // If we have specified a connection planner and have found it in the mapping..
//                 if (input_connection_planner != NULL)
//                 {

//                     std::vector<state_t*> neighborhood_states;
//                     std::vector<double> object_state_vec;
//                     manipulation_query->object->get_state_space()->copy_point_to_vector(object_states[0], object_state_vec);
//                     config_t object_state_config(object_state_vec);
//                     get_neighborhood_connected_states(neighborhood_states, object_state_vec, 5, converted_start_state);

//                     // Compute the motion plan to one of the goal states
//                     input_connection_planner->setup_search( manipulation_query->astar_mode, manipulation_query->valid_constraints, constraint_names );   

//                     grasp_rrt_specification_t* grasp_rrt_specification = dynamic_cast<grasp_rrt_specification_t* >(output_specifications[connection_planner_names[active_planner->planning_context_name]]);
//                     grasp_rrt_specification->object_to_grasp = manipulation_query->object;
//                     PRX_ASSERT(grasp_rrt_specification!=NULL);
//                     grasp_evaluator_t* grasp_evaluator = grasp_rrt_specification->grasp_evaluator;
//                     grasp_evaluator->link_grasping_database(the_datas);

//                     bool try_IK_steering_first = false;

//                     std::vector<state_t*> goal_states;
//                     //PICK
//                     solution_found = input_connection_planner->compute_solution(the_plan, output_constraints, neighborhood_states, goal_states, try_IK_steering_first );
//                     if(solution_found)
//                     {
//                         append_to_stat_file("\n     ___RRT");
//                         state_t* satisfied_start = input_connection_planner->query->satisfied_start_state;
//                         state_t* satisfied_goal = input_connection_planner->query->satisfied_goal_state;
//                         reached_goal = satisfied_goal;
//                         plan_t move_plan(manipulation_model->get_current_manipulation_info()->full_arm_control_space);
//                         constraints_t* move_constr = validity_checker->alloc_constraint();
//                         if(execute_move(move_plan, move_constr, converted_start_state, satisfied_start))
//                         {
//                             move_plan += the_plan;
//                             the_plan = move_plan;
//                         }
//                         else
//                         {
//                             PRX_ERROR_S("Connected state could not be reached by move.");
//                             solution_found = false;
//                         }

//                     }

//                 }


//                 manipulation_query->found_path = solution_found;

//                 manipulation_model->use_context(old_context);
//                 restore_grasping_mode (previous_grasping_mode, actual_start_state);
               

//                 // if(manipulation_query->task_mode == TASK_PICK )
//                 // {
//                 //     manipulation_query->found_path = execute_pick( the_plan, the_constraints, the_datas[0], the_start );
//                 // }
//                 // else if(manipulation_query->task_mode == TASK_PLACE )
//                 // {
//                 //     manipulation_query->found_path = execute_place( the_plan, the_constraints, the_datas[0], the_start );
//                 // }
//                 // else if(manipulation_query->task_mode == TASK_PICK_AND_PLACE)
//                 // {
//                 //     // If PLACE connection plan failed, return false
//                 //     state_t* correct_start_state = the_datas[0]->retracted_closed_state;// (the_datas[0]->retracted_open_state != NULL ? the_datas[0]->retracted_open_state : the_datas[0]->retracted_closed_state );
//                 //     //Back up old grasp data
//                 //     old_grasp_data->copy_from_data(the_datas[1]);

//                 //     append_to_stat_file("\n   ---Place");
//                 //     manipulation_query->found_path = compute_connection_plan(correct_start_state, object_states[1], the_datas[1], 1);
//                 //     restore_grasping_mode (previous_grasping_mode, actual_start_state);
//                 //     if (!manipulation_query->found_path)
//                 //     {
//                 //         //Restore old grasp data
//                 //         the_datas[1]->copy_from_data(old_grasp_data);
//                 //         append_to_stat_file("\n   Connection plan to target pose failed.");
//                 //         //If we are forcing the connection plans to succeed then we give up further processing for this grasp
//                 //         if(force_connection_plans)
//                 //         {
//                 //             return false;
//                 //         }
//                 //         else
//                 //         {
//                 //             append_to_stat_file("\n   Continue despite connection failure.");
//                 //             PRX_PRINT("##############  Failed the connection plan to target but continuing with what we have.", PRX_TEXT_CYAN);
//                 //         }
//                 //     }

//                 //     manipulation_query->found_path = execute_pick_and_place( the_plan, the_constraints, the_datas[0], the_datas[1] );                
//                 // }
//                 // else if(manipulation_query->task_mode == TASK_PICK_AND_MOVE)
//                 // {
//                 //     manipulation_query->found_path = execute_pick_and_move( the_plan, the_constraints, the_datas[0], the_start, the_goal );                
//                 // }

//                 return manipulation_query->found_path;
//             }


//             bool grasp_rrt_tp_t::compute_motion_plan(plan_t& the_plan, constraints_t* the_constraints, const std::vector<state_t*>& object_states, const std::vector<grasp_data_t*>& the_datas, state_t* the_start, state_t* the_goal)
//             {

//                 int previous_grasping_mode = manipulation_model->get_current_grasping_mode();
//                 // TODO: Prealloc this
//                 state_t* actual_start_state = manipulation_model->get_state_space()->alloc_point();

//                 //Back up old grasp data
//                 grasp_data_t* old_grasp_data = new grasp_data_t();
//                 old_grasp_data->copy_from_data(the_datas[0]);

//                 append_to_stat_file("\n   ---Pick");
//                 // Compute Connection plan
//                 manipulation_query->found_path = compute_connection_plan(the_start, object_states[0], the_datas[0], 0);

//                 restore_grasping_mode (previous_grasping_mode, actual_start_state);
//                 if (!manipulation_query->found_path)
//                 {
//                     //Restore old grasp data
//                     the_datas[0]->copy_from_data(old_grasp_data);
//                     append_to_stat_file("\n   Connection plan failed.");
//                     //If we are forcing the connection plans to succeed then we give up further processing for this grasp
//                     if(force_connection_plans)
//                     {
//                         return false;
//                     }
//                     else
//                     {
//                         append_to_stat_file("\n   Continue despite connection failure.");
//                         PRX_PRINT("##############  Failed the connection plan but continuing with what we have.", PRX_TEXT_CYAN);
//                     }
//                 }

//                 if(manipulation_query->task_mode == TASK_PICK )
//                 {
//                     manipulation_query->found_path = execute_pick( the_plan, the_constraints, the_datas[0], the_start );
//                 }
//                 else if(manipulation_query->task_mode == TASK_PLACE )
//                 {
//                     manipulation_query->found_path = execute_place( the_plan, the_constraints, the_datas[0], the_start );
//                 }
//                 else if(manipulation_query->task_mode == TASK_PICK_AND_PLACE)
//                 {
//                     // If PLACE connection plan failed, return false
//                     state_t* correct_start_state = the_datas[0]->retracted_closed_state;// (the_datas[0]->retracted_open_state != NULL ? the_datas[0]->retracted_open_state : the_datas[0]->retracted_closed_state );
//                     //Back up old grasp data
//                     old_grasp_data->copy_from_data(the_datas[1]);

//                     append_to_stat_file("\n   ---Place");
//                     manipulation_query->found_path = compute_connection_plan(correct_start_state, object_states[1], the_datas[1], 1);
//                     restore_grasping_mode (previous_grasping_mode, actual_start_state);
//                     if (!manipulation_query->found_path)
//                     {
//                         //Restore old grasp data
//                         the_datas[1]->copy_from_data(old_grasp_data);
//                         append_to_stat_file("\n   Connection plan to target pose failed.");
//                         //If we are forcing the connection plans to succeed then we give up further processing for this grasp
//                         if(force_connection_plans)
//                         {
//                             return false;
//                         }
//                         else
//                         {
//                             append_to_stat_file("\n   Continue despite connection failure.");
//                             PRX_PRINT("##############  Failed the connection plan to target but continuing with what we have.", PRX_TEXT_CYAN);
//                         }
//                     }

//                     manipulation_query->found_path = execute_pick_and_place( the_plan, the_constraints, the_datas[0], the_datas[1] );                
//                 }
//                 else if(manipulation_query->task_mode == TASK_PICK_AND_MOVE)
//                 {
//                     manipulation_query->found_path = execute_pick_and_move( the_plan, the_constraints, the_datas[0], the_start, the_goal );                
//                 }

//                 return manipulation_query->found_path;

//             }


//             bool grasp_rrt_tp_t::execute_move(plan_t& plan, constraints_t* path_constraints, state_t* manipulator_initial_state, state_t* manipulator_target_state)
//             {
//                 //If the currently used end-effector is closed, we will be moving the object, so we cannot trust the roadmap.
//                 bool closed = current_manipulation_context_info->manipulator->is_end_effector_closed( current_manipulation_context_info->end_effector_index );
//                 if( closed )
//                 {
//                     active_planner->setup_search( force_untrusted_mode( manipulation_query->astar_mode ), manipulation_query->valid_constraints, constraint_names );
//                 }
//                 else
//                 {
//                     active_planner->setup_search( manipulation_query->astar_mode, manipulation_query->valid_constraints, constraint_names );                    
//                 }

//                 manipulation_model->enable_IK_steering(IK_steer_movements,manipulation_model->get_current_context());
//                 manipulation_model->get_state_space()->copy_from_point(manipulator_initial_state);
//                 PRX_DEBUG_COLOR("--Manip TP :: Move "<<manipulation_model->get_full_state_space()->print_memory(1),PRX_TEXT_MAGENTA);
//                 bool success = active_planner->compute_solution(plan, path_constraints, manipulator_initial_state, manipulator_target_state, manipulation_query->ik_steer_paths, manipulation_query->update_constraints);
//                 if(success)
//                     manipulation_model->push_state(manipulator_target_state);
//                 PRX_DEBUG_COLOR("--Manip TP :: Move End "<< (success?"True":"False") ,PRX_TEXT_MAGENTA);
//                 return success;
//             }

//             bool grasp_rrt_tp_t::execute_pick(plan_t& plan, constraints_t* path_constraints, grasp_data_t* data, state_t* actual_start_state)
//             {
//                 state_t* converted_start_state = manipulation_model->get_state_space()->alloc_point();
//                 // PRX_PRINT ("EXECUTE PICK WORLD MODEL BEFORE PUSH: " << manipulation_model->get_full_state_space()->print_memory(3), PRX_TEXT_BLUE);
//                 manipulation_query->object->get_state_space()->copy_from_point(manipulation_query->object_initial_state);
//                 //Need to enforce that we are planning from the start state, but with the correct releasing mode.
//                 manipulation_model->push_state( actual_start_state );
//                 // PRX_PRINT ("EXECUTE PICK WORLD MODEL AFTER PUSH: " << manipulation_model->get_full_state_space()->print_memory(3), PRX_TEXT_BLUE);
//                 //current_manipulation_context_info->full_arm_state_space->copy_fromt_point(data->relative_grasp->release_mode);
//                 manipulation_model->engage_grasp( plan, data->relative_grasp->release_mode, false );

//                 manipulation_model->get_state_space()->copy_to_point(converted_start_state);
//                 // PRX_PRINT ("EXECUTE PICK WORLD MODEL ENGAGE GRASP PUSH: " << manipulation_model->get_full_state_space()->print_memory(3), PRX_TEXT_BLUE);
//                 //manipulation_model->get_state_space()->copy_to_point( actual_start_state );
//                 // PRX_PRINT("retracted open state in execute pick: " << current_manipulation_context_info->full_arm_state_space->print_point(data->retracted_open_state, 3), PRX_TEXT_LIGHTGRAY);
//                 // PRX_PRINT("Object initial state: " <<manipulation_query->object->get_state_space()->print_point(manipulation_query->object_initial_state,3), PRX_TEXT_BLUE);
//                 //Then see if we can plan to that state
//                 active_planner->setup_search( manipulation_query->astar_mode, manipulation_query->valid_constraints, constraint_names );
                
//                 if(active_planner->compute_solution(plan, path_constraints, converted_start_state, data->retracted_open_state, manipulation_query->ik_steer_paths, manipulation_query->update_constraints))
//                 {
//                     // PRX_PRINT("EXECUTE PICK: motion planner returned plan of length: " << plan.length(), PRX_TEXT_MAGENTA);
//                     // PRX_PRINT("EXECUTE PICK: data's reaching plan is of length: " << data->reaching_plan.length(), PRX_TEXT_MAGENTA);

//                     plan += data->reaching_plan;
//                     path_constraints->merge(data->open_reaching_constraints);

//                     engage_grasp_after_pick( plan, data );

//                     // If we are in retracting mode, add the plans and constraints as well
//                     if ( manipulation_query->manipulator_retracts || manipulation_query->task_mode == TASK_PICK_AND_PLACE || manipulation_query->task_mode == TASK_PICK_AND_MOVE )
//                     {
//                         plan += data->retracting_plan;
//                         path_constraints->merge(data->closed_retracting_constraints);
//                         // PRX_PRINT("EXECUTE PICK: data's retracting plan is of length: " << data->reaching_plan.length(), PRX_TEXT_CYAN);
//                     }

//                     manipulation_model->get_state_space()->free_point(converted_start_state);
//                     return true;
//                 }

//                 manipulation_model->get_state_space()->free_point(converted_start_state);
//                 return false;
//             }

//             bool grasp_rrt_tp_t::execute_place(plan_t& plan, constraints_t* path_constraints, grasp_data_t* data, state_t* actual_start_state)
//             {
//                 PRX_PRINT ("FULL STATE: " << manipulation_model->get_full_state_space()->print_memory(4), PRX_TEXT_RED);
//                 active_planner->setup_search( force_untrusted_mode( manipulation_query->astar_mode ), manipulation_query->valid_constraints, constraint_names );
//                 //PRX_PRINT("Object initial state: " <<manipulation_query->object->get_state_space()->print_point(manipulation_query->object_initial_state,3), PRX_TEXT_BLUE);

//                 if(active_planner->compute_solution(plan, path_constraints, actual_start_state, data->retracted_closed_state, manipulation_query->ik_steer_paths, manipulation_query->update_constraints))
//                 {
//                     plan += data->reaching_plan;
//                     path_constraints->merge(data->closed_reaching_constraints);

//                     if ( manipulation_query->manipulator_retracts )
//                     {
//                         manipulation_model->push_state(data->grasping_state);
//                         manipulation_model->engage_grasp(plan, data->relative_grasp->release_mode, false);
//                         plan += data->retracting_plan;

//                         manipulation_model->push_state(data->retracted_open_state);

//                         if(manipulation_query->default_open_mode != data->relative_grasp->release_mode)
//                             manipulation_model->engage_grasp(plan, manipulation_query->default_open_mode, false);

//                         path_constraints->merge(data->open_retracting_constraints);
//                     }

//                     return true;
//                 }
//                 return false;
//             }

//             bool grasp_rrt_tp_t::execute_pick_and_place(sim::plan_t& plan, util::constraints_t* path_constraints, grasp_data_t* pick_data, grasp_data_t* place_data )
//             {
//                 PRX_DEBUG_COLOR("\n\nGetting suggested P&P.\n\n", PRX_TEXT_MAGENTA);
//                 manipulation_query->object->get_state_space()->copy_from_point(manipulation_query->object_initial_state);
//                 if( execute_pick( plan, path_constraints, pick_data, manipulation_query->manipulator_initial_state ) )
//                 {
//                     //The tmp_state has been initialized at the beginning of the resolve query.
//                     PRX_DEBUG_COLOR("--Manip TP :: P&P suggested_pick: "<< manipulation_model->get_full_state_space()->print_memory(7),PRX_TEXT_LIGHTGRAY);
//                     engage_grasp_after_pick( plan, pick_data );
//                     PRX_DEBUG_COLOR("--Manip TP :: P&P successful_pick: "<< manipulation_model->get_full_state_space()->print_memory(7),PRX_TEXT_LIGHTGRAY);
//                     if( execute_place( plan, path_constraints, place_data, pick_data->retracted_closed_state ) )
//                     {
//                         return true;
//                     }
//                     else
//                     {
//                         append_to_stat_file( "\n  Failed to execute place." );
//                     }
//                 }
//                 else
//                 {
//                     append_to_stat_file(  "\n  Failed to execute pick." );
//                 }
//                 return false;
//             }

//             bool grasp_rrt_tp_t::execute_pick_and_move(plan_t& plan, constraints_t* path_constraints, grasp_data_t* pick_data, state_t* manipulator_initial_state, state_t* manipulator_target_state)
//             {
//                 PRX_DEBUG_COLOR("\n\nGetting suggested P&M.\n\n", PRX_TEXT_MAGENTA);
//                 manipulation_query->object->get_state_space()->copy_from_point(manipulation_query->object_initial_state);
//                 if( execute_pick( plan, path_constraints, pick_data, manipulation_query->manipulator_initial_state ) )
//                 {
//                     //The tmp_state has been initialized at the beginning of the resolve query.
//                     PRX_DEBUG_COLOR("--Manip TP :: P&M suggested_pick: "<< manipulation_model->get_full_state_space()->print_memory(7),PRX_TEXT_LIGHTGRAY);
//                     engage_grasp_after_pick( plan, pick_data );
//                     PRX_DEBUG_COLOR("--Manip TP :: P&M successful_pick: "<< manipulation_model->get_full_state_space()->print_memory(7),PRX_TEXT_LIGHTGRAY);
//                     if( execute_move( plan, path_constraints, pick_data->retracted_closed_state, manipulator_target_state ) )
//                     {
//                         return true;
//                     }
//                 }
//                 return false;
//             }


//             void grasp_rrt_tp_t::engage_grasp_after_pick(plan_t& plan, grasp_data_t* data)
//             {
//                 manipulation_model->push_state(data->releasing_state);
//                 // PRX_DEBUG_COLOR("--After Push state: "<< manipulation_model->get_full_state_space()->print_memory(7),PRX_TEXT_CYAN);
//                 manipulation_model->engage_grasp(plan, data->relative_grasp->grasping_mode, true);
//             }

//             search_mode_t grasp_rrt_tp_t::force_untrusted_mode( search_mode_t input_mode )
//             {
//                 if( input_mode == STANDARD_SEARCH )
//                 {
//                     return LAZY_SEARCH;
//                 }
//                 else if( input_mode == TRUSTED_MCR )
//                 {
//                     return UNTRUSTED_MCR;
//                 }
//                 return input_mode;
//             }


//             void grasp_rrt_tp_t::update_vis_info() const
//             {
//                 foreach(planner_info_t* planner_info, planner_info_map | boost::adaptors::map_values)
//                 {
//                     manipulation_model->use_context(planner_info->construction_context_name);
//                     planner_info->planner->update_visualization();
//                 }
//                 foreach(planner_info_t* planner_info, connection_info_map | boost::adaptors::map_values)
//                 {
//                     manipulation_model->use_context(planner_info->construction_context_name);
//                     planner_info->planner->update_visualization();
//                 }
//             }

//         }
//     }
// }
