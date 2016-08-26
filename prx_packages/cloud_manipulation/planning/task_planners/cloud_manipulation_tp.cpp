// /**
//  * @file manipulation_tp.cpp
//  *
//  * @copyright Software License Agreement (BSD License)
//  * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick 
//  * All Rights Reserved.
//  * For a full description see the file named LICENSE.
//  *
//  * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
//  *
//  * Email: pracsys@googlegroups.com
//  */

// #include "planning/task_planners/cloud_manipulation_tp.hpp"

// #include "prx/utilities/definitions/string_manip.hpp"
// #include "prx/utilities/definitions/random.hpp"
// #include "prx/utilities/math/configurations/config.hpp"
// #include "prx/utilities/distance_metrics/ann_metric/ann_distance_metric.hpp"
// #include "prx/utilities/goals/goal.hpp"
// #include "prx/utilities/math/configurations/bounds.hpp"
// #include "prx/utilities/goals/goal_state.hpp"
// #include "prx/utilities/statistics/statistics.hpp"

// #include "prx/planning/world_model.hpp"
// #include "prx/planning/modules/validity_checkers/validity_checker.hpp"
// #include "prx/planning/modules/local_planners/local_planner.hpp"
// #include "prx/planning/problem_specifications/motion_planning_specification.hpp"
// #include "prx/planning/queries/motion_planning_query.hpp"
// #include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
// #include "prx/planning/motion_planners/motion_planner.hpp"
// #include "prx/planning/motion_planners/prm_star/prm_star_statistics.hpp"

// #include "../../../manipulation/planning/modules/samplers/manip_sampler.hpp"
// #include "planning/problem_specifications/cloud_manipulation_specification.hpp"
// #include "planning/queries/cloud_manipulation_query.hpp"

// #include <pluginlib/class_list_macros.h>
// #include <boost/assign/list_of.hpp>
// #include <boost/graph/subgraph.hpp>
// #include <boost/range/adaptor/map.hpp>

// #include <boost/graph/connected_components.hpp>
// #include <boost/graph/biconnected_components.hpp>
// #include <boost/graph/compressed_sparse_row_graph.hpp>
// #include "prx/planning/motion_planners/prm_star/prm_star.hpp"
// #include "prx/planning/modules/stopping_criteria/element/iteration_criterion.hpp"

// PLUGINLIB_EXPORT_CLASS(prx::packages::cloud_manipulation::cloud_manipulation_tp_t, prx::plan::planner_t)

// namespace prx
// {
//     using namespace util;
//     using namespace sim;
//     using namespace plan;

//     namespace packages
//     {
//         using namespace manipulation;
//         namespace cloud_manipulation
//         {

//             cloud_manipulation_tp_t::cloud_manipulation_tp_t()
//             {
//                 poses_set_length = 0;

//                 stable_pose_memory.push_back(new double());
//                 stable_pose_memory.push_back(new double());
//                 stable_pose_space = new space_t("XY", stable_pose_memory);
//                 serialize_grasped_graph = false;
//                 serialize_ungrasped_graph = false;
//                 deserialize_grasped_graph = false;
//                 deserialize_ungrasped_graph = false;
//                 graph_builder = false;
//                 statistics_time = 0;
//             }

//             cloud_manipulation_tp_t::~cloud_manipulation_tp_t()
//             {
//                 manip_state_space->free_point(released_point);
//                 manip_state_space->free_point(grasped_point);
//                 manip_state_space->free_point(retracted_point);
//                 manip_state_space->free_point(raised_point);

//                 manip_state_space->free_point(safe_state);
//                 manip_control_space->free_point(safe_control);
//                 manip_control_space->free_point(tmp_manip_ctrl);

//                 real_object_space->free_point(real_initial_poses);

//                 object_state_space->free_point(object_point);

//                 stable_pose_space->free_point(stable_pose_point);

//                 retract_plan.clear();
//                 raise_plan.clear();

//                 retract_path.clear();
//                 raise_path.clear();




//             }

//             void cloud_manipulation_tp_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
//             {
//                 task_planner_t::init(reader, template_reader);
//                 PRX_INFO_S("Initializing Manipulation task planner ...");

//                 if( parameters::has_attribute("manipulator_path", reader, template_reader) )
//                     manipulator_path = parameters::get_attribute("manipulator_path", reader, template_reader);
//                 else
//                     PRX_FATAL_S("Task planner does not know where to find the manipulator plant. You need to specify a path for the manipulator");

//                 if( parameters::has_attribute("pc_name_manipulator_only", reader, template_reader) )
//                     pc_name_manipulator_only = parameters::get_attribute("pc_name_manipulator_only", reader, template_reader);
//                 else
//                     PRX_FATAL_S("Planning context for manipulator only is not specified!");

//                 if( parameters::has_attribute("pc_name_object_only", reader, template_reader) )
//                     pc_name_object_only = parameters::get_attribute("pc_name_object_only", reader, template_reader);
//                 else
//                     PRX_FATAL_S("Planning context for object only is not specified!");

//                 if( parameters::has_attribute("pc_name_manipulator_with_object", reader, template_reader) )
//                     pc_name_manipulator_with_object = parameters::get_attribute("pc_name_manipulator_with_object", reader, template_reader);
//                 else
//                     PRX_FATAL_S("Planning context for manipulator with cup is not specified!");

//                 if( parameters::has_attribute("pc_name_manipulator_with_active_object", reader, template_reader) )
//                     pc_name_manipulator_with_active_object = parameters::get_attribute("pc_name_manipulator_with_active_object", reader, template_reader);
//                 else
//                     PRX_FATAL_S("Planning context for manipulator with active object is not specified!");

//                 if( parameters::has_attribute("pc_name_real_world", reader, template_reader) )
//                     pc_name_real_world = parameters::get_attribute("pc_name_real_world", reader, template_reader);
//                 else
//                     PRX_FATAL_S("Planning context for real world is not specified!");


//                 if( parameters::has_attribute("pc_name_grasp_planning", reader, template_reader) )
//                     pc_name_grasp_planning = parameters::get_attribute("pc_name_grasp_planning", reader, template_reader);
//                 else
//                     PRX_FATAL_S("Planning context for planning world is not specified!");

//                 augment_iterations = parameters::get_attribute_as<int>("augment_iterations", reader, template_reader, 500);
//                 graph_generation_mode = parameters::get_attribute_as<int>("graph_generation_mode", reader, template_reader, 0);
//                 visualize_graph = parameters::get_attribute_as<bool>("visualize_graph", reader, template_reader, false);
//                 tree_based_planning_mode = parameters::get_attribute_as<bool>("tree_based_planning_mode", reader, template_reader, false);
//                 best_manipulation_combination = parameters::get_attribute_as<bool>("best_manipulation_combination", reader, template_reader, true);


//                 if( reader->has_attribute("stable_pose_space") )
//                     stable_pose_space->init(reader->get_child("stable_pose_space").get());
//                 else if( template_reader != NULL )
//                     stable_pose_space->init(template_reader->get_child("stable_pose_space").get());
//                 else
//                     PRX_FATAL_S("Missing stable pose space from manipulation task planner!");

//                 if( parameters::has_attribute("manip_sampler", reader, template_reader) )
//                     manip_sampler = static_cast<manip_sampler_t*>(parameters::initialize_from_loader<sampler_t > ("prx_planning", reader, "manip_sampler", template_reader, "manip_sampler"));
//                 else
//                     PRX_FATAL_S("Missing sampler attribute for manipulation task planner!");

//                 if( parameters::has_attribute("cloud_robotics_stat_file", reader, template_reader) )
//                     cloud_statistics_file = parameters::get_attribute_as<std::string>("cloud_robotics_stat_file", reader, template_reader, "stat_file");


//                 ungrasped_name = parameters::get_attribute("ungrasped_name", reader, template_reader);
//                 grasped_name = parameters::get_attribute("grasped_name", reader, template_reader);
//                 std::string element;
//                 if( parameters::has_attribute("graph_specifications", reader, template_reader) )
//                 {
//                     const parameter_reader_t* specification_template_reader = NULL;
//                     element = "graph_specifications/" + ungrasped_name;
//                     if( parameters::has_attribute(element, reader, template_reader) )
//                     {
//                         if( parameters::has_attribute(element + "/template", reader, template_reader) )
//                         {
//                             specification_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + reader->get_attribute(element + "/template"));
//                         }
//                         output_specifications[ungrasped_name] = parameters::initialize_from_loader<specification_t > ("prx_planning", reader, element, specification_template_reader, "");

//                         if( specification_template_reader != NULL )
//                         {
//                             delete specification_template_reader;
//                             specification_template_reader = NULL;
//                         }
//                     }
//                     else
//                     {
//                         PRX_FATAL_S("Missing motion planning specification for the ungrasped graph!");
//                     }

//                     element = "graph_specifications/" + grasped_name;
//                     if( parameters::has_attribute(element, reader, template_reader) )
//                     {
//                         if( parameters::has_attribute(element + "/template", reader, template_reader) )
//                         {
//                             specification_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + reader->get_attribute(element + "/template"));
//                         }
//                         output_specifications[grasped_name] = parameters::initialize_from_loader<specification_t > ("prx_planning", reader, element, specification_template_reader, "");
//                         if( specification_template_reader != NULL )
//                         {
//                             delete specification_template_reader;
//                             specification_template_reader = NULL;
//                         }
//                     }
//                     else
//                     {
//                         PRX_FATAL_S("Missing motion planning specification for the grasped graph!");
//                     }
//                 }

//                 if( parameters::has_attribute("graph_goals", reader, template_reader) )
//                 {
//                     element = "graph_goals/first";
//                     if( parameters::has_attribute(element, reader, template_reader) )
//                         first_part_goal = (goal_state_t*)parameters::initialize_from_loader<goal_t > ("prx_utilities", reader, element, template_reader, element);
//                     else
//                         PRX_FATAL_S("Missing first's part goal_t variable!");

//                     element = "graph_goals/second";
//                     if( parameters::has_attribute(element, reader, template_reader) )
//                         second_part_goal = (goal_state_t*)parameters::initialize_from_loader<goal_t > ("prx_utilities", reader, element, template_reader, element);
//                     else
//                         PRX_FATAL_S("Missing seconds's part goal_t variable!");

//                     element = "graph_goals/third";
//                     if( parameters::has_attribute(element, reader, template_reader) )
//                         third_part_goal = (goal_state_t*)parameters::initialize_from_loader<goal_t > ("prx_utilities", reader, element, template_reader, element);
//                     else
//                         PRX_FATAL_S("Missing seconds's part goal_t variable!");
//                 }

//                 output_queries["first"] = new motion_planning_query_t();
//                 output_queries["second"] = new motion_planning_query_t();
//                 output_queries["third"] = new motion_planning_query_t();

//                 bool tree_set_ungrasped = false;
//                 bool tree_set_grasped = false;
//             }

//             void cloud_manipulation_tp_t::reset()
//             {
//                 retract_path.clear();
//                 retract_plan.clear();
//                 raise_path.clear();
//                 raise_plan.clear();

//                 poses_set_length = 0;

//             }

//             void cloud_manipulation_tp_t::link_world_model(world_model_t * const model)
//             {
//                 task_planner_t::link_world_model(model);
//             }

//             const statistics_t* cloud_manipulation_tp_t::get_statistics()
//             {
//                 PRX_WARN_S("Get statistics for manipulation tp is not implemented!");
//                 return new statistics_t();
//             }

//             void cloud_manipulation_tp_t::link_specification(specification_t* new_spec)
//             {
//                 task_planner_t::link_specification(new_spec);
//                 specs = static_cast<cloud_manipulation_specification_t*>(new_spec);

//             }

//             void cloud_manipulation_tp_t::link_query(query_t* new_query)
//             {
//                 //                //PRX_DEBUG_POINT("Linking Query");
//                 task_planner_t::link_query(new_query);
//                 manip_query = static_cast<cloud_manipulation_query_t*>(new_query);
//             }

//             void cloud_manipulation_tp_t::setup()
//             {
//                 PRX_DEBUG_COLOR("Setup manipulation tp ...", PRX_TEXT_CYAN);
//                 _manipulator = static_cast<manipulator_plant_t*>(model->get_system(manipulator_path).get());
//                 model->use_context(pc_name_manipulator_with_object);
//                 mo_space = model->get_state_space();

//                 model->use_context(pc_name_real_world);
//                 real_object_space = model->get_active_space();
//                 real_initial_poses = real_object_space->alloc_point();

//                 model->use_context(pc_name_manipulator_with_active_object);
//                 manip_state_space = model->get_state_space();
//                 manip_control_space = model->get_control_space();
//                 object_state_space = model->get_active_space();
//                 tmp_control_vec.resize(manip_control_space->get_dimension());

//                 grasped_point = mo_space->alloc_point();
//                 raised_point = mo_space->alloc_point();
//                 released_point = manip_state_space->alloc_point();
//                 retracted_point = manip_state_space->alloc_point();

//                 object_point = object_state_space->alloc_point();
//                 object_pos.resize(object_state_space->get_dimension());
//                 initial_pose = object_state_space->alloc_point();
//                 goal_pose = object_state_space->alloc_point();

//                 retract_path.link_space(manip_state_space);
//                 raise_path.link_space(mo_space);

//                 retract_plan.link_control_space(manip_control_space);
//                 raise_plan.link_control_space(manip_control_space);
//                 resolve_query_plan_1.link_control_space(manip_control_space);
//                 resolve_query_plan_2.link_control_space(manip_control_space);
//                 resolve_query_plan_3.link_control_space(manip_control_space);
//                 resolve_query_min_plan.link_control_space(manip_control_space);

//                 stable_pose_point = stable_pose_space->alloc_point();

//                 poses_set_length = 0;

//                 specs->link_spaces(manip_state_space, manip_control_space);
//                 specs->setup(model);

//                 manip_sampler->link_info(_manipulator, manip_state_space, object_state_space);

//                 //                output_specifications[ungrasped_name]->validity_checker = specs->validity_checker;
//                 output_specifications[ungrasped_name]->local_planner = specs->local_planner;
// //                output_specifications[ungrasped_name]->sampler = specs->sampler;
//                 output_specifications[ungrasped_name]->link_spaces(manip_state_space, manip_control_space);
//                 ((manip_sampler_t*)output_specifications[ungrasped_name]->sampler)->link_info(_manipulator, manip_state_space, object_state_space, false);
//                 output_specifications[ungrasped_name]->setup(model);

//                 //                output_specifications[grasped_name]->validity_checker = specs->validity_checker;
//                 output_specifications[grasped_name]->local_planner = specs->local_planner;
//                 output_specifications[grasped_name]->link_spaces(mo_space, manip_control_space);
//                 ((manip_sampler_t*)output_specifications[grasped_name]->sampler)->link_info(_manipulator, manip_state_space, object_state_space);
//                 output_specifications[grasped_name]->setup(model);

//                 first_part_goal->link_space(manip_state_space);
//                 second_part_goal->link_space(mo_space);
//                 third_part_goal->link_space(manip_state_space);

//                 safe_state = manip_state_space->alloc_point();
//                 manip_state_space->set_from_vector(specs->safe_position, safe_state);

//                 safe_control = manip_control_space->alloc_point();
//                 tmp_manip_ctrl = manip_control_space->alloc_point();
//                 manip_control_space->set_from_vector(specs->safe_position, safe_control);

//                 graph_query = dynamic_cast<motion_planning_query_t*>(output_queries["first"]);
//                 graph_query->set_goal(first_part_goal);
//                 graph_query->link_spaces(manip_state_space, manip_control_space);
//                 //For the first search the start will always be the safe state.
//                 graph_query->link_start(safe_state);
//                 graph_query->q_collision_type = cloud_manipulation_query_t::PRX_LAZY_COLLISIONS;

//                 graph_query = dynamic_cast<motion_planning_query_t*>(output_queries["second"]);
//                 graph_query->set_goal(second_part_goal);
//                 graph_query->link_spaces(mo_space, manip_control_space);
//                 graph_query->q_collision_type = cloud_manipulation_query_t::PRX_LAZY_COLLISIONS;

//                 graph_query = dynamic_cast<motion_planning_query_t*>(output_queries["third"]);
//                 //Because the goal for the last part will always be the safe state.
//                 third_part_goal->set_goal_state(safe_state);
//                 graph_query->set_goal(third_part_goal);
//                 graph_query->link_spaces(manip_state_space, manip_control_space);
//                 graph_query->q_collision_type = cloud_manipulation_query_t::PRX_LAZY_COLLISIONS;

//                 real_object_point = real_object_space->alloc_point();
//                 real_initial_object_point = real_object_space->alloc_point();

//                 if( deserialize_ungrasped_graph )
//                 {
//                     planners[ungrasped_name]->set_param("", "deserialize_flag", deserialize_ungrasped_graph);
//                     planners[ungrasped_name]->set_param("", "deserialization_file", ungrasped_graph_file_name);
//                 }

//                 if( deserialize_grasped_graph )
//                 {
//                     planners[grasped_name]->set_param("", "deserialize_flag", deserialize_grasped_graph);
//                     planners[grasped_name]->set_param("", "deserialization_file", grasped_graph_file_name);

//                 }






//                 //                grasped_metric->link_space(state_space);
//                 //                grasped_pose_metric->link_space(state_space);
//                 //                pose_metric->link_space(state_space);
//             }

//             bool cloud_manipulation_tp_t::serialize()
//             {
//                 bool serialized = false;
//                 if( serialize_ungrasped_graph )
//                 {
//                     model->use_context(pc_name_manipulator_only);
//                     motion_planner_t* motion_planner = dynamic_cast<motion_planner_t*>(planners[ungrasped_name]);
//                     motion_planner->set_param("", "serialize_flag", serialize_ungrasped_graph);
//                     motion_planner->set_param("", "serialization_file", ungrasped_graph_file_name);
//                     motion_planner->serialize();
//                     serialized = true;
//                 }
//                 if( serialize_grasped_graph )
//                 {
//                     model->use_context(pc_name_manipulator_with_object);
//                     motion_planner_t* motion_planner = dynamic_cast<motion_planner_t*>(planners[grasped_name]);
//                     motion_planner->set_param("", "serialize_flag", serialize_grasped_graph);
//                     motion_planner->set_param("", "serialization_file", grasped_graph_file_name);
//                     motion_planner->serialize();
//                     serialized = true;
//                 }

//                 return serialized;
//             }

//             bool cloud_manipulation_tp_t::deserialize()
//             {
//                 if( deserialize_ungrasped_graph )
//                 {
//                     model->use_context(pc_name_manipulator_only);
//                     dynamic_cast<motion_planner_t*>(planners[ungrasped_name])->deserialize();
//                 }
//                 if( deserialize_grasped_graph )
//                 {
//                     model->use_context(pc_name_manipulator_with_object);
//                     dynamic_cast<motion_planner_t*>(planners[grasped_name])->deserialize();
//                 }
//                 return true;
//             }

//             bool cloud_manipulation_tp_t::succeeded() const
//             {
//                 //if( input_specification->get_stopping_criterion()->satisfied() )
//                 return true;
//                 //return false;
//             }

//             bool cloud_manipulation_tp_t::execute2()
//             {
//                 motion_planner_t* motion_planner;

//                 model->use_context(pc_name_manipulator_only);
//                 graph_specification = static_cast<motion_planning_specification_t*>(output_specifications[ungrasped_name]);
//                 motion_planner = dynamic_cast<motion_planner_t*>(planners[ungrasped_name]);
//                 graph_specification->get_stopping_criterion()->link_motion_planner(motion_planner);
//                 motion_planner->link_specification(graph_specification);
//                 //                std::vector<bounds_t*> bounds_ungrasped = graph_specification->state_space->get_bounds();
//                 //                bounds_ungrasped.back()->set_bounds(0,0);
//                 //                graph_specification->state_space->set_bounds(bounds_ungrasped);
//                 motion_planner->setup();

//                 model->use_context(pc_name_manipulator_with_object);
//                 graph_specification = dynamic_cast<motion_planning_specification_t*>(output_specifications[grasped_name]);
//                 motion_planner = dynamic_cast<motion_planner_t*>(planners[grasped_name]);
//                 graph_specification->clear_seeds();
//                 graph_specification->get_stopping_criterion()->link_motion_planner(motion_planner);
//                 motion_planner->link_specification(graph_specification);
//                 //                std::vector<bounds_t*> bounds_grasped = graph_specification->state_space->get_bounds();
//                 //                bounds_ungrasped.back()->set_bounds(1,1);
//                 //                graph_specification->state_space->set_bounds(bounds_grasped);
//                 motion_planner->setup();

//                 if( graph_builder )
//                 {
//                     if( !deserialize_grasped_graph )
//                         PRX_FATAL_S("Builder and no deserializations?");
//                     deserialize();
//                     serialize();
//                 }
//                 else
//                 {
//                     deserialize();
//                 }

//                 return true;
//             }

//             bool cloud_manipulation_tp_t::compute_grasp(cloud_manipulation_query_t* manip_query)
//             {
//                 model->use_context(pc_name_object_only);
//                 poses_set.resize(poses_set.size() + manip_query->get_initial_poses()->size() + manip_query->get_target_poses()->size());
//                 const std::vector< std::vector< double > >* poses = manip_query->get_initial_poses();
//                 unsigned j = 0;
//                 grasped_point = mo_space->alloc_point();

//                 std::stringstream ss_poses;
//                 ss_poses << "\nPOSES";

//                 PRX_DEBUG_COLOR(poses->size() << " and " << poses_set.size(), PRX_TEXT_CYAN);
//                 for( unsigned i = 0; i < poses->size(); ++poses_set_length, ++i )
//                 {
//                     ss_poses << "\n";

//                     poses_set[poses_set_length].state = object_state_space->alloc_point();
//                     object_state_space->copy_vector_to_point(poses->at(i), poses_set[poses_set_length].state);
//                     for( unsigned p = 0; p < poses->at(i).size(); ++p, ++j )
//                     {
//                         real_initial_poses->memory[j] = poses->at(i)[p];
//                         ss_poses << poses->at(i)[p] << ", ";
//                     }
//                 }
//                 PRX_PRINT(ss_poses.str(), PRX_TEXT_BLUE);
//                 real_object_space->copy_from_point(real_initial_poses);
//                 PRX_DEBUG_COLOR("Num of poses: " << poses_set_length, PRX_TEXT_LIGHTGRAY);
//                 //                PRX_DEBUG_COLOR("initial state: " << real_object_space->print_point(real_initial_poses,4),PRX_TEXT_MAGENTA);

//                 poses = manip_query->get_target_poses();
//                 for( unsigned i = 0; i < poses->size(); ++poses_set_length, ++i )
//                 {
//                     poses_set[poses_set_length].state = object_state_space->alloc_point();
//                     object_state_space->copy_vector_to_point(poses->at(i), poses_set[poses_set_length].state);
//                 }
//                 PRX_DEBUG_COLOR("Num of poses: " << poses_set_length, PRX_TEXT_LIGHTGRAY);




//                 //                poses = manip_query->get_extra_poses();


//                 PRX_DEBUG_COLOR("Done sampling poses: " << poses_set_length << "   num_poses:" << specs->num_poses, PRX_TEXT_LIGHTGRAY);

//                 util::config_t con;
//                 std::vector<double> man;
//                 std::vector<double> obj;
//                 //                bool chk = _manipulator->IK_solver(con, man);
//                 std::stringstream ss;


//                 bool good_point;
//                 bool good_grasp;
//                 for( unsigned i = poses_set_length - 2; i < (unsigned)poses_set_length; ++i )
//                 {
//                     for( unsigned j = 0; j < specs->max_different_grasps; ++j )
//                     {
//                         int tries = -1;

//                         PRX_DEBUG_COLOR("Trying grasp ", PRX_TEXT_BROWN);

//                         good_point = false;
//                         good_grasp = false;

//                         do
//                         {
//                             model->use_context(pc_name_manipulator_with_object);
//                             good_point = (manip_sampler->sample_near_object(grasped_point, mo_space, poses_set[i].state) && validity_checker->is_valid(grasped_point));
//                             PRX_DEBUG_COLOR(mo_space->print_point(grasped_point, 4), PRX_TEXT_RED);
//                             ++tries;
//                         }
//                         while( !(good_point && (good_grasp = valid_grasp())) && tries < specs->max_tries );


//                         if( good_point && good_grasp )
//                         {

//                             PRX_DEBUG_COLOR("Got good point (" << i << "," << j << "): tries:" << tries << "/" << specs->max_tries, PRX_TEXT_MAGENTA);
//                             poses_set[i].grasped_set.push_back(mo_space->clone_point(grasped_point));
//                             poses_set[i].ungrasped_set.push_back(manip_state_space->clone_point(released_point));

//                             config_t c;
//                             _manipulator->get_end_effector_offset_configuration(c, released_point);
//                             double x_p, y_p, z_p, qx, qy, qz, qw;
//                             c.get_position(x_p, y_p, z_p);
//                             double* quat = new double[4];
//                             c.get_xyzw_orientation(quat);
//                             qx = quat[0];
//                             qy = quat[1];
//                             qz = quat[2];
//                             qw = quat[3];

//                             ////PRX_DEBUG_POINT("\nADDING POINT \n" << mo_space->print_point(grasped_point, 4) << "\n" << x_p << "," << y_p << "," << z_p << "," << qx << "," << qy << "," << qz << "," << qw << "," << manip_state_space->print_point(released_point, 4));

//                             //PRX_ASSERT(mo_space->equal_points(grasped_point, raise_path[0]));
//                             PRX_ASSERT(manip_state_space->equal_points(released_point, retract_path[0]));
//                             //                                poses_set[i].raised_set.push_back(mo_space->clone_point(raise_path.back()));
//                             poses_set[i].retracted_set.push_back(manip_state_space->clone_point(retract_path.back()));

//                             manip_state_space->copy_point_to_vector(released_point, tmp_control_vec);
//                             manip_control_space->copy_vector_to_point(tmp_control_vec, tmp_manip_ctrl);
//                             //
//                             retract_plan.copy_onto_front(tmp_manip_ctrl, retract_plan[0].duration);
//                             poses_set[i].retracting_plans.push_back(retract_plan);
//                             poses_set[i].reaching_plans.push_back(plan_t());
//                             poses_set[i].reaching_plans.back().reverse_plan(retract_plan);
//                             poses_set[i].reaching_plans.back().copy_onto_front(retract_plan[retract_plan.size() - 1].control, retract_plan[retract_plan.size() - 1].duration);
//                             PRX_DEBUG_COLOR("Retracting plan: \n" << poses_set[i].retracting_plans.back().print(), PRX_TEXT_GREEN);
//                             PRX_DEBUG_COLOR("Reaching plan: \n" << poses_set[i].reaching_plans.back().print(), PRX_TEXT_BROWN);

//                         }
//                     }



//                     if( i < manip_query->important_poses() )
//                     {
//                         if( poses_set[i].grasped_set.size() == 0 )
//                         {
//                             PRX_ERROR_S("One of the Initial/target poses cannot be grasped! The problem cannot be solved!");
//                             PRX_ERROR_S(object_state_space->print_point(poses_set[i].state));

//                             manip_query->plan = retract_plan;

//                             return false;
//                         }
//                     }
//                 }
//                 return true;
//             }

//             bool cloud_manipulation_tp_t::execute()
//             {



//                 //                compute_grasp(manip_query);


//                 //PRX_DEBUG_POINT("Starting ser/deser");
//                 motion_planner_t* motion_planner;
//                 //                const prm_star_statistics_t* statistics;
//                 int seed_num = 0;

//                 //Building the open hand graph.  
//                 model->use_context(pc_name_manipulator_only);
//                 graph_specification = static_cast<motion_planning_specification_t*>(output_specifications[ungrasped_name]);
                
//                 PRX_DEBUG_COLOR("Trying the cast ungrasped "<< dynamic_cast<manipulation::manip_sampler_t*>(output_specifications[ungrasped_name]->sampler),PRX_TEXT_BLUE);
//                 PRX_DEBUG_COLOR("Trying the cast grasped "<< dynamic_cast<manipulation::manip_sampler_t*>(output_specifications[grasped_name]->sampler),PRX_TEXT_BLUE);

//                 motion_planner = dynamic_cast<motion_planner_t*>(planners[ungrasped_name]);


//                 graph_specification->clear_seeds();
//                 for( int i = 0; i < poses_set_length; ++i )
//                 {
//                     for( unsigned j = 0; j < poses_set[i].retracted_set.size(); ++j )
//                     {
//                         //                        graph_specification->add_seed(poses_set[i].retracted_set[j]);
//                         seed_num++;
//                     }
//                 }
//                 //                graph_specification->add_seed(safe_state);


//                 graph_specification->get_stopping_criterion()->link_motion_planner(motion_planner);

//                 motion_planner->link_specification(graph_specification);
//                 if( !tree_based_planning_mode )
//                 {
//                     motion_planner->setup();
//                 }


//                 std::stringstream out(std::stringstream::out);

//                 for( boost::unordered_map<std::string, sim::system_ptr_t, string_hash>::const_iterator val = model->get_obstacles().begin(); val != model->get_obstacles().end(); val++ )
//                 {
//                     sim::obstacle_t* obs = static_cast<sim::obstacle_t*>(val->second.get());


//                     out << "Key: " << val->first << " , " << obs->get_root_configuration().print() << std::endl;
//                 }
//                 PRX_DEBUG_COLOR(out.str(), PRX_TEXT_RED);



//                 char* w = std::getenv("PRACSYS_PATH");
//                 std::string dir(w);
//                 dir += ("/prx_output/");
//                 std::string file = dir + cloud_statistics_file;
//                 PRX_INFO_S("File directory is: " << file);
//                 std::ofstream cloud_stat;
//                 cloud_stat.open(file.c_str());
//                 if( !cloud_stat.is_open() )
//                     PRX_PRINT(file.c_str() << "\n\n\n!!!! OUTPUT FILE DID NOT OPEN !!!!!!\n\n\n", PRX_TEXT_RED);

//                 double graph_timings = 0;
//                 double serz_timings = 0;



//                 _clock.reset();
//                 if( deserialize_ungrasped_graph )
//                 {
//                     //                    motion_planner->deserialize();
//                 }
//                 else
//                 {
//                     try
//                     {
//                         PRX_DEBUG_COLOR("Start building ungrasped graph", PRX_TEXT_MAGENTA);
//                         if( !tree_based_planning_mode )
//                         {
//                             if( graph_generation_mode == 1 || graph_generation_mode == 4 )
//                             {
//                                 motion_planner->execute();
//                             }
//                         }
//                     }
//                     catch( stopping_criteria_t::stopping_criteria_satisfied e )
//                     {
//                         //statistics = dynamic_cast<const prm_star_statistics_t*>(motion_planner->get_statistics());
//                         //PRX_PRINT("Ungrasped statistics : |seed| : " << seed_num << " |V|: " << statistics->num_vertices << "    |E|:" << statistics->num_edges, PRX_TEXT_MAGENTA);

//                     }
//                 }
//                 graph_timings = _clock.measure();
//                 if( serialize_ungrasped_graph )
//                 {
//                     motion_planner->set_param("", "serialize_flag", serialize_ungrasped_graph);
//                     motion_planner->set_param("", "serialization_file", ungrasped_graph_file_name);
//                     motion_planner->serialize();
//                 }
//                 _clock.reset();
//                 if( !tree_based_planning_mode )
//                 {
//                     if( graph_generation_mode == 0 )
//                     {
//                         motion_planner->deserialize();
//                     }
//                     else if( graph_generation_mode == 1 || graph_generation_mode == 4 )
//                     {
//                         motion_planner->serialize();
//                     }
//                 }
//                 serz_timings = _clock.measure();
//                 //                //PRX_DEBUG_POINT("End of ungrasped deserialization");
//                 //Building grasped graph

//                 //statistics = dynamic_cast<const prm_star_statistics_t*>(motion_planner->get_statistics());
//                 //PRX_PRINT("Ungrasped statistics : |seed| : " << seed_num << " |V|: " << statistics->num_vertices << "    |E|:" << statistics->num_edges, PRX_TEXT_MAGENTA);

//                 //cloud_stat << "\nUngrasped graph construction\nVertices: \n" << statistics->num_vertices << "\nEdges: \n" << statistics->num_edges;
//                 if( best_manipulation_combination )
//                     cloud_stat << "\nConstruction time: \n" << graph_timings << "\nDeserialization time \n" << serz_timings;
//                 else
//                     cloud_stat << "\nConstruction time: \n" << graph_timings << "\nSerialization time \n" << serz_timings;


//                 model->use_context(pc_name_manipulator_with_object);

//                 graph_specification = dynamic_cast<motion_planning_specification_t*>(output_specifications[grasped_name]);
//                 motion_planner = dynamic_cast<motion_planner_t*>(planners[grasped_name]);


//                 graph_specification->clear_seeds();
//                 seed_num = 0;
//                 for( int i = 0; i < poses_set_length; ++i )
//                 {
//                     for( unsigned j = 0; j < poses_set[i].raised_set.size(); ++j )
//                     {
//                         //                        graph_specification->add_seed(poses_set[i].raised_set[j]);
//                         seed_num++;
//                     }
//                 }


//                 graph_specification->get_stopping_criterion()->link_motion_planner(motion_planner);

//                 motion_planner->link_specification(graph_specification);

//                 //PRX_DEBUG_POINT("Sanity's sake!");
//                 if( !tree_based_planning_mode )
//                 {
//                     motion_planner->setup();
//                 }

//                 graph_timings = 0;
//                 serz_timings = 0;

//                 _clock.reset();
//                 if( deserialize_grasped_graph )
//                 {
//                     //motion_planner->deserialize();
//                 }
//                 else
//                 {
//                     try
//                     {
//                         if( !tree_based_planning_mode )
//                         {
//                             PRX_DEBUG_COLOR("Start building grasped graph", PRX_TEXT_MAGENTA);
//                             if( graph_generation_mode == 2 || graph_generation_mode == 4 )
//                             {
//                                 motion_planner->execute();
//                             }
//                         }
//                     }
//                     catch( stopping_criteria_t::stopping_criteria_satisfied e )
//                     {
//                         //statistics = dynamic_cast<const prm_star_statistics_t*>(motion_planner->get_statistics());
//                         //PRX_PRINT("Grasped statistics : |seed| : " << seed_num << " |V|: " << statistics->num_vertices << "    |E|:" << statistics->num_edges, PRX_TEXT_MAGENTA);

//                     }
//                 }
//                 graph_timings = _clock.measure();

//                 if( serialize_grasped_graph )
//                 {
//                     motion_planner->set_param("", "serialize_flag", serialize_grasped_graph);
//                     motion_planner->set_param("", "serialization_file", grasped_graph_file_name);
//                     motion_planner->serialize();
//                 }



//                 _clock.reset();
//                 if( !tree_based_planning_mode )
//                 {
//                     if( graph_generation_mode == 0 )
//                     {
//                         motion_planner->deserialize();
//                     }
//                     else if( graph_generation_mode == 2 || graph_generation_mode == 4 )
//                     {
//                         motion_planner->serialize();
//                     }
//                 }
//                 serz_timings = _clock.measure();
//                 //statistics = dynamic_cast<const prm_star_statistics_t*>(motion_planner->get_statistics());
//                 //PRX_PRINT("Grasped statistics : |seed| : " << seed_num << " |V|: " << statistics->num_vertices << "    |E|:" << statistics->num_edges, PRX_TEXT_MAGENTA);

//                 //cloud_stat << "\nGrasped graph construction\nVertices: \n" << statistics->num_vertices << "\nEdges: \n" << statistics->num_edges;
//                 //cloud_stat << "\nConstruction time: \n" << graph_timings << "\nDeserialization time \n" << serz_timings;
//                 if( best_manipulation_combination )
//                     cloud_stat << "\nConstruction time: \n" << graph_timings << "\nDeserialization time \n" << serz_timings;
//                 else
//                     cloud_stat << "\nConstruction time: \n" << graph_timings << "\nSerialization time \n" << serz_timings;

//                 //                if( !graph_builder )
//                 //                    resolve_query();

//                 //cloud_stat << "\nResolve Query\nTime: \n" << statistics_time << "\nPath length: \n" << min_total_path_found << "\nPlan size: \n" << manip_query->plan.size();
//                 cloud_stat.close();
//                 PRX_DEBUG_COLOR("End of Execute", PRX_TEXT_RED);
//                 return true;
//             }

//             void cloud_manipulation_tp_t::resolve_query()
//             {
//                 PRX_DEBUG_COLOR("Inside Resolve Query ", PRX_TEXT_BROWN);
//                 manip_query->link_spaces(manip_state_space, manip_control_space);
//                 object_state_space->copy_vector_to_point(manip_query->get_initial_pose(), initial_pose);
//                 object_state_space->copy_vector_to_point(manip_query->get_target_pose(), goal_pose);



//                 int start_pose_index = similar_pose(initial_pose);
//                 int target_pose_index = similar_pose(goal_pose);
//                 //                int start_pose_index = similar_pose(goal_pose);
//                 //                int target_pose_index = similar_pose(initial_pose);
//                 start_pose_index = 0;
//                 target_pose_index = 1;

//                 std::stringstream ss;
//                 for( unsigned i = 0; i < poses_set[start_pose_index].state->memory.size(); ++i )
//                     ss << poses_set[start_pose_index].state->memory.at(i) << ", ";
//                 PRX_DEBUG_COLOR(ss.str(), PRX_TEXT_RED);
//                 ss.str("");
//                 for( unsigned i = 0; i < poses_set[target_pose_index].state->memory.size(); ++i )
//                     ss << poses_set[target_pose_index].state->memory.at(i) << ", ";
//                 PRX_DEBUG_COLOR(ss.str(), PRX_TEXT_BLUE);

//                 //                PRX_DEBUG_COLOR(model->get_full_state_space()->print_point(poses_set[start_pose_index].state,4),PRX_TEXT_RED);
//                 //                PRX_DEBUG_COLOR(model->get_full_state_space()->print_point(poses_set[target_pose_index].state,4),PRX_TEXT_BLUE);
//                 //PRX_DEBUG_POINT("START AND GOAL " << start_pose_index << " -> " << target_pose_index << " out of " << poses_set.size() << " poses");
//                 //poses_set[start_pose_index].state
//                 if( start_pose_index == -1 || target_pose_index == -1 )
//                     return;


//                 manipulate(start_pose_index, target_pose_index, manip_query);

//                 PRX_PRINT("TOTAL RESOLVE QUERY TIME TAKEN " << statistics_time, PRX_TEXT_CYAN);




//                 PRX_PRINT("END OF RESOLVE QUERY : " << manip_query->plan.size(), PRX_TEXT_RED);
//             }

//             void cloud_manipulation_tp_t::manipulate(unsigned start_pose_index, unsigned target_pose_index)
//             {
//                 manipulate(start_pose_index, target_pose_index, manip_query);
//             }

//             void cloud_manipulation_tp_t::manipulate(std::vector<double> start_pose, std::vector<double> end_pose)
//             {
//                 init_poses.clear();
//                 tgt_poses.clear();
//                 init_poses.push_back(start_pose);
//                 tgt_poses.push_back(end_pose);
//                 manip_query->set_initial_poses(init_poses);
//                 manip_query->set_target_poses(tgt_poses);
//                 PRX_DEBUG_COLOR("POSES EXISTING " << poses_set_length, PRX_TEXT_GREEN);
//                 compute_grasp(manip_query);
//                 PRX_DEBUG_COLOR("NEW POSES " << poses_set_length, PRX_TEXT_GREEN);
//                 PRX_DEBUG_COLOR("MOVING FROM " << poses_set_length - 2 << " to " << poses_set_length - 1, PRX_TEXT_GREEN);
//                 manipulate(poses_set_length - 2, poses_set_length - 1, manip_query);
//             }


//             //The manipulation function to generate the plan and path in a specific environment which is preset. The start and goal pose indices form the query and the results are updated in the manipulation query which it accepts as argument.

//             void cloud_manipulation_tp_t::manipulate(unsigned start_pose_index, unsigned target_pose_index, cloud_manipulation_query_t* manip_query)
//             {
//                 /*
//                  Note: Major changes: Activity of the plants in the second phase
//                  * Order of the plan pieces (1st phase, 2nd phase, 3rd phase, retracts, reaches, raises, placements)
//                  * Starting states of the 2nd and 3rd stage queries
                 
//                  */
//                 char* w = std::getenv("PRACSYS_PATH");
//                 std::string dir(w);
//                 dir += ("/prx_output/");
//                 std::string file = dir + cloud_statistics_file;
//                 PRX_INFO_S("File directory is: " << file);
//                 std::ofstream cloud_stat;
//                 cloud_stat.open(file.c_str(), std::fstream::app);
//                 if( !cloud_stat.is_open() )
//                     PRX_PRINT(file.c_str() << "\n\n\n!!!! OUTPUT FILE DID NOT OPEN !!!!!!\n\n\n", PRX_TEXT_RED);
//                 cloud_stat << "\nManipulate";

//                 unsigned manip_try_counter = 0;
//                 unsigned manip_success_counter = 0;
//                 unsigned manip_found = 0;
//                 _clock.reset();
//                 statistics_time = 0;

//                 std::vector< plant_t* > all_plants;


//                 ((motion_planning_query_t*)output_queries["first"])->plan.clear();
//                 ((motion_planning_query_t*)output_queries["second"])->plan.clear();
//                 ((motion_planning_query_t*)output_queries["third"])->plan.clear();
//                 if( tree_based_planning_mode )
//                 {
//                     ((motion_planning_query_t*)output_queries["first"])->q_collision_type = cloud_manipulation_query_t::PRX_NO_COLLISIONS;
//                     ((motion_planning_query_t*)output_queries["second"])->q_collision_type = cloud_manipulation_query_t::PRX_NO_COLLISIONS;
//                     ((motion_planning_query_t*)output_queries["third"])->q_collision_type = cloud_manipulation_query_t::PRX_NO_COLLISIONS;
//                 }

//                 min_total_path_found = PRX_INFINITY;
//                 double path_length = 0;
//                 resolve_query_plan_1.clear();
//                 resolve_query_plan_2.clear();
//                 resolve_query_plan_3.clear();
//                 resolve_query_min_plan.clear();

//                 manip_query->plan.clear();

//                 bool reverse_transfer = false;

//                 manip_state_space->copy_from_point(safe_state);
//                 //For the first part the initial state is always set to be the safe state.
//                 real_object_space->copy_to_point(real_object_point);
//                 real_object_space->copy_to_point(real_initial_object_point);
//                 real_object_point_vec.resize(real_object_space->get_dimension());
//                 real_object_space->copy_point_to_vector(real_object_point, real_object_point_vec);


//                 PRX_DEBUG_COLOR("initial state: " << real_object_space->print_point(real_initial_poses, 4), PRX_TEXT_MAGENTA);


//                 //                //PRX_DEBUG_POINT("safe state: " << manip_state_space->print_point(safe_state, 4));
//                 for( unsigned i = 0; i < poses_set[start_pose_index].ungrasped_set.size(); ++i )
//                 {

//                     reverse_transfer = false;
//                     PRX_DEBUG_COLOR("\n\n\n First pose loop " << model->get_state_space()->print_memory(3), PRX_TEXT_RED);
//                     PRX_PRINT("Going for the first pose : " << i << "/" << poses_set[start_pose_index].ungrasped_set.size(), PRX_TEXT_BROWN);
//                     //                    first_part_goal->set_goal_state(poses_set[start_pose_index].ungrasped_set[i]);
//                     path_length = 0;
//                     first_part_goal->set_goal_state(poses_set[start_pose_index].retracted_set[i]); // retracting_plans[i].back().control);


//                     model->use_context(pc_name_grasp_planning);
//                     real_object_space->copy_from_point(real_initial_object_point);
//                     real_object_space->copy_to_point(real_object_point);
//                     real_object_space->copy_point_to_vector(real_object_point, real_object_point_vec);
//                     all_plants.clear();
//                     model->get_system_graph().get_plants(all_plants);

//                     //all_plants[1]->set_active(true, all_plants[1]->get_pathname());

//                     for( unsigned ap = 0; ap < all_plants.size(); ++ap )
//                     {
//                         PRX_DEBUG_COLOR(all_plants[ap]->get_pathname() << " is active -> " << all_plants[ap]->is_active() << " at " << all_plants[ap]->get_state_space()->print_memory(3), PRX_TEXT_MAGENTA);
//                     }
//                     PRX_DEBUG_COLOR("\n\n CHECKING VALIDITY AT \nstate ::" << model->get_full_state_space()->print_memory(3) << "\nactive::" << model->get_active_space()->print_memory(3) << "\ncurrent:" << model->get_state_space()->print_memory(3) << "\n\n", PRX_TEXT_RED);







//                     model->use_context(pc_name_real_world);

//                     all_plants.clear();
//                     model->get_system_graph().get_plants(all_plants);

//                     //all_plants[1]->set_active(true, all_plants[1]->get_pathname());

//                     for( unsigned ap = 0; ap < all_plants.size(); ++ap )
//                     {
//                         PRX_DEBUG_COLOR(all_plants[ap]->get_pathname() << " is active -> " << all_plants[ap]->is_active() << " at " << all_plants[ap]->get_state_space()->print_memory(3), PRX_TEXT_MAGENTA);
//                     }
//                     PRX_DEBUG_COLOR("\n\n CHECKING VALIDITY AT \nstate ::" << model->get_full_state_space()->print_memory(3) << "\nactive::" << model->get_active_space()->print_memory(3) << "\ncurrent:" << model->get_state_space()->print_memory(3) << "\n\n", PRX_TEXT_RED);

//                     ////PRX_DEBUG_POINT("CHECKING THIS PLANNING CONTEXT");


//                     //PRX_DEBUG_COLOR("REAL WORLD STATE: " << model->get_state_space()->print_memory(4), PRX_TEXT_BROWN);
//                     PRX_DEBUG_COLOR("BF: " << model->get_full_state_space()->print_memory(4), PRX_TEXT_CYAN);
//                     //output_specifications[ungrasped_name]->validity_checker->link_model(model);
//                     graph_specification = static_cast<motion_planning_specification_t*>(output_specifications[ungrasped_name]);
//                     if( tree_based_planning_mode )
//                     {
//                         graph_specification->clear_seeds();
//                         graph_specification->add_seed(safe_state);
//                         output_specifications[grasped_name]->get_stopping_criterion()->reset();
//                         output_specifications[ungrasped_name]->get_stopping_criterion()->reset();
//                         if( tree_set_ungrasped )
//                         {
//                             PRX_DEBUG_COLOR("Ungrasped reset", PRX_TEXT_GREEN);
//                             planners[ungrasped_name]->reset();
//                         }
//                         else
//                         {
//                             tree_set_ungrasped = true;
//                         }
//                         PRX_DEBUG_COLOR("Ungrasped setup", PRX_TEXT_GREEN);
//                         planners[ungrasped_name]->setup();

//                     }

//                     //                    planners[ungrasped_name]->link_specification(graph_specification);

//                     //                    planners[ungrasped_name]->link_query(output_queries["first"]);

//                     //                    PRX_PRINT("\n\n USING CONTEXT" << pc_name_real_world, PRX_TEXT_RED);
//                     PRX_DEBUG_COLOR("Ungrasped link query", PRX_TEXT_GREEN);
//                     planners[ungrasped_name]->link_query(output_queries["first"]);
//                     _clock.reset();
//                     try
//                     {
//                         if( tree_based_planning_mode )
//                         {
//                             PRX_DEBUG_COLOR("Ungrasped execute", PRX_TEXT_MAGENTA);
// //                            bounds_ungrasped = model->get_state_space()->get_bounds();
// //                            bounds_ungrasped.back()->set_bounds(0, 0);
// //                            
// //                            model->get_state_space()->set_bounds(bounds_ungrasped);
//                             planners[ungrasped_name]->execute();
// //                            bounds_ungrasped.back()->set_bounds(0, 1);
// //                            model->get_state_space()->set_bounds(bounds_ungrasped);
//                         }
//                     }
//                     catch( stopping_criteria_t::stopping_criteria_satisfied e )
//                     {
//                         //                        statistics = dynamic_cast<const prm_star_statistics_t*>(motion_planner->get_statistics());
//                         //                        PRX_DEBUG_COLOR("Grasped statistics : |seed| : " << seed_num << " |V|: " << statistics->num_vertices << "    |E|:" << statistics->num_edges, PRX_TEXT_MAGENTA);

//                     }
//                     PRX_DEBUG_COLOR("Ungrasped resolve query", PRX_TEXT_GREEN);
//                     planners[ungrasped_name]->resolve_query();
//                     //statistics_time += _clock.measure();
//                     manip_try_counter++;
//                     //PRX_DEBUG_COLOR("REAL WORLD STATE: " << model->get_state_space()->print_memory(4), PRX_TEXT_BROWN);
//                     PRX_DEBUG_COLOR("AF: " << model->get_full_state_space()->print_memory(4), PRX_TEXT_RED);
//                     PRX_DEBUG_COLOR(manip_state_space->print_point(first_part_goal->get_goal_points().at(0), 4), PRX_TEXT_BLUE);
//                     util::config_t plan_con;
//                     std::stringstream ss;

//                     //                    ss.str("");
//                     //                    manip_state_space->copy_from_point(((motion_planning_query_t*)output_queries["first"])->plan.at(0).control);
//                     //                    _manipulator->get_end_effector_configuration(plan_con);
//                     //                    ss << plan_con.get_position().at(0) << ", " << plan_con.get_position().at(1) << ", " << plan_con.get_position().at(2) << ", ";
//                     //                    ss << plan_con.get_orientation().get_x() << ", " << plan_con.get_orientation().get_y() << ", " << plan_con.get_orientation().get_z() << ", " << plan_con.get_orientation().get_w();
//                     //
//                     //                    PRX_DEBUG_COLOR("START FROM " << ss.str(), PRX_TEXT_GREEN);
//                     //
//                     //                    ss.str("");
//                     //                    manip_state_space->copy_from_point(((motion_planning_query_t*)output_queries["first"])->plan.at(((motion_planning_query_t*)output_queries["first"])->plan.size() - 1).control);
//                     //                    _manipulator->get_end_effector_configuration(plan_con);
//                     //                    ss << plan_con.get_position().at(0) << ", " << plan_con.get_position().at(1) << ", " << plan_con.get_position().at(2) << ", ";
//                     //                    ss << plan_con.get_orientation().get_x() << ", " << plan_con.get_orientation().get_y() << ", " << plan_con.get_orientation().get_z() << ", " << plan_con.get_orientation().get_w();
//                     //
//                     //                    PRX_DEBUG_COLOR("END AT " << ss.str(), PRX_TEXT_RED);

//                     //PRX_DEBUG_POINT("FIRST LEG :: " << i << " :: " << ((motion_planning_query_t*)output_queries["first"])->plan.size());
//                     PRX_PRINT("FIRST LEG :: " << i << " :: " << ((motion_planning_query_t*)output_queries["first"])->plan.size(), PRX_TEXT_GREEN);
//                     /*statistics_time=_clock.measure();
//                     PRX_PRINT("!!!! FINALLY MINIMUM OF " << ((motion_planning_query_t*)output_queries["first"])->path.length() << " !!!", PRX_TEXT_GREEN);
//                     PRX_PRINT("TOTAL TIME TAKEN TO SOLVE PROBLEM::: " << statistics_time, PRX_TEXT_MAGENTA);
//                     cloud_stat<<"\nMin path\n"<<((motion_planning_query_t*)output_queries["first"])->path.length()<<"\nTime taken\n"<<statistics_time<<"\nPlanner tries\n"<<manip_try_counter<<"\nPlanner successes\n"<<manip_success_counter<<"\nManipulation paths found\n"<<manip_found;
//                     cloud_stat.close();
//                     //manip_query->plan=((motion_planning_query_t*)output_queries["first"])->plan;
//                     return;
//                      */


//                     if( ((motion_planning_query_t*)output_queries["first"])->plan.size() != 0 )
//                     {
//                         /*statistics_time=_clock.measure();
//                         PRX_PRINT("!!!! FINALLY MINIMUM OF " << ((motion_planning_query_t*)output_queries["first"])->path.length() << " !!!", PRX_TEXT_GREEN);
//                         PRX_PRINT("TOTAL TIME TAKEN TO SOLVE PROBLEM::: " << statistics_time, PRX_TEXT_MAGENTA);
//                         cloud_stat<<"\nMin path\n"<<((motion_planning_query_t*)output_queries["first"])->path.length()<<"\nTime taken\n"<<statistics_time<<"\nPlanner tries\n"<<manip_try_counter<<"\nPlanner successes\n"<<manip_success_counter<<"\nManipulation paths found\n"<<manip_found;
//                         cloud_stat.close();
//                         //manip_query->plan=((motion_planning_query_t*)output_queries["first"])->plan;
//                         return;*/
//                         manip_success_counter++;
//                         PRX_DEBUG_COLOR("\n\n\n When first worked " << model->get_state_space()->print_memory(3), PRX_TEXT_RED);
//                         resolve_query_plan_1 = ((motion_planning_query_t*)output_queries["first"])->plan;
//                         path_length += ((motion_planning_query_t*)output_queries["first"])->path.length();
//                         PRX_PRINT(" First leg path " << i << "::" << path_length, PRX_TEXT_BROWN);

//                         PRX_DEBUG_COLOR("Going for the middle part first plan size: " << ((motion_planning_query_t*)output_queries["first"])->plan.size(), PRX_TEXT_BROWN);
//                         graph_query = dynamic_cast<motion_planning_query_t*>(output_queries["second"]);

//                         PRX_DEBUG_COLOR(" START AT " << mo_space->print_point(poses_set[start_pose_index].grasped_set[i], 4), PRX_TEXT_CYAN);

//                         if( !reverse_transfer )
//                         {
//                             graph_query->link_start(poses_set[start_pose_index].grasped_set[i]);
//                         }


//                         for( unsigned j = 0; j < poses_set[target_pose_index].grasped_set.size(); ++j )
//                         {
//                             PRX_DEBUG_COLOR("\n\n\n Second pose loop " << model->get_state_space()->print_memory(3), PRX_TEXT_RED);
//                             PRX_DEBUG_COLOR(" GOAL AT " << mo_space->print_point(poses_set[target_pose_index].grasped_set[j], 4), PRX_TEXT_CYAN);
//                             if( !reverse_transfer )
//                             {
//                                 PRX_DEBUG_COLOR("FORWARD TRANSFER " << i << " to " << j, PRX_TEXT_BROWN);
//                                 second_part_goal->set_goal_state(poses_set[target_pose_index].grasped_set[j]);
//                             }
//                             else
//                             {
//                                 PRX_DEBUG_COLOR("REVERSED TRANSFER " << j << " to " << i, PRX_TEXT_BROWN);
//                                 graph_query->link_start(poses_set[target_pose_index].grasped_set[j]);
//                                 second_part_goal->set_goal_state(poses_set[start_pose_index].grasped_set[i]);
//                             }

//                             PRX_DEBUG_COLOR("BF: " << model->get_full_state_space()->print_memory(4), PRX_TEXT_CYAN);
//                             //model->use_context(pc_name_manipulator_with_object);
//                             model->use_context(pc_name_grasp_planning);
//                             all_plants.clear();
//                             model->get_system_graph().get_plants(all_plants);

//                             //                            all_plants[1]->set_active(true, all_plants[1]->get_pathname());

//                             //Set conditional activity of the cups so that they are considered during the collision checking in the grasped planner
//                             for( unsigned ap = 0; ap < all_plants.size(); ++ap )
//                             {
//                                 PRX_DEBUG_COLOR(all_plants[ap]->get_pathname() << " is active -> " << all_plants[ap]->is_active() << " at " << all_plants[ap]->get_state_space()->print_memory(3), PRX_TEXT_MAGENTA);
//                                 if( all_plants[ap]->get_state_space()->get_dimension() != poses_set[start_pose_index].state->memory.size() )
//                                     continue;
//                                 state_t* plant_s = all_plants[ap]->get_state_space()->alloc_point();
//                                 if( all_plants[ap]->get_state_space()->distance(poses_set[start_pose_index].state, plant_s) < 0.01 && all_plants[ap]->get_pathname() != "simulator/cup_a" )
//                                 {
//                                     all_plants[ap]->set_active(false, all_plants[ap]->get_pathname());
//                                     //PRX_PRINT("^^^ Too Near", PRX_TEXT_CYAN);
//                                 }
//                                 else
//                                 {
//                                     //                                    if( !all_plants[ap]->is_active() )
//                                     //                                        all_plants[ap]->set_active(true, all_plants[ap]->get_pathname());

//                                 }
//                                 //                                if(all_plants[ap]->get_pathname()!="simulator/cup_a" && !all_plants[ap]->is_active())
//                                 //                                {
//                                 //                                    all_plants[ap]->set_active(true,all_plants[ap]->get_pathname());
//                                 //                                }

//                             }
//                             //                            all_plants[0].get_state_space()->print_memory();
//                             //                            all_plants[0].set_active();
//                             //                            all_plants[0].get_pathname();

//                             //output_specifications[ungrasped_name]->validity_checker->link_model(model);
//                             graph_specification = static_cast<motion_planning_specification_t*>(output_specifications[grasped_name]);

//                             if( tree_based_planning_mode )
//                             {
//                                 graph_specification->clear_seeds();
//                                 if( !reverse_transfer )
//                                 {
//                                     graph_specification->add_seed(poses_set[start_pose_index].grasped_set[i]);
//                                 }
//                                 else
//                                 {
//                                     graph_specification->add_seed(poses_set[target_pose_index].grasped_set[j]);
//                                 }
//                                 if( tree_set_grasped )
//                                 {
//                                     PRX_DEBUG_COLOR("Grasped reset", PRX_TEXT_GREEN);
//                                     planners[grasped_name]->reset();
//                                 }
//                                 else
//                                 {
//                                     tree_set_grasped = true;
//                                 }
//                                 output_specifications[grasped_name]->get_stopping_criterion()->reset();
//                                 output_specifications[ungrasped_name]->get_stopping_criterion()->reset();
//                                 //planners[grasped_name]->link_specification(graph_specification);
//                                 PRX_DEBUG_COLOR("Grasped setup", PRX_TEXT_GREEN);
//                                 planners[grasped_name]->setup();
//                             }
//                             PRX_DEBUG_COLOR("Grasped link query", PRX_TEXT_GREEN);
//                             planners[grasped_name]->link_query(output_queries["second"]);
//                             //                            PRX_PRINT("\n\n USING CONTEXT"<<pc_name_manipulator_with_object, PRX_TEXT_RED);
//                             ////PRX_DEBUG_POINT("\n\n USING CONTEXT" << pc_name_manipulator_with_object);

//                             _clock.reset();
//                             try
//                             {
//                                 if( tree_based_planning_mode )
//                                 {
//                                     PRX_DEBUG_COLOR("Grasped execute", PRX_TEXT_MAGENTA);
//                                     planners[grasped_name]->execute();
//                                 }
//                             }
//                             catch( stopping_criteria_t::stopping_criteria_satisfied e )
//                             {
//                                 //                        statistics = dynamic_cast<const prm_star_statistics_t*>(motion_planner->get_statistics());
//                                 //                        PRX_DEBUG_COLOR("Grasped statistics : |seed| : " << seed_num << " |V|: " << statistics->num_vertices << "    |E|:" << statistics->num_edges, PRX_TEXT_MAGENTA);

//                             }
//                             PRX_DEBUG_COLOR("Grasped resolve query", PRX_TEXT_GREEN);
//                             planners[grasped_name]->resolve_query();
//                             //statistics_time += _clock.measure();
//                             manip_try_counter++;
//                             PRX_DEBUG_COLOR("AF: " << model->get_full_state_space()->print_memory(4), PRX_TEXT_RED);


//                             //                            ss.str("");
//                             //                            manip_state_space->copy_from_point(graph_query->plan.at(0).control);
//                             //                            _manipulator->get_end_effector_configuration(plan_con);
//                             //                            ss << plan_con.get_position().at(0) << ", " << plan_con.get_position().at(1) << ", " << plan_con.get_position().at(2) << ", ";
//                             //                            ss << plan_con.get_orientation().get_x() << ", " << plan_con.get_orientation().get_y() << ", " << plan_con.get_orientation().get_z() << ", " << plan_con.get_orientation().get_w();
//                             //
//                             //                            PRX_DEBUG_COLOR("START FROM " << ss.str(), PRX_TEXT_GREEN);
//                             //
//                             //                            ss.str("");
//                             //                            manip_state_space->copy_from_point(graph_query->plan.at(graph_query->plan.size() - 1).control);
//                             //                            _manipulator->get_end_effector_configuration(plan_con);
//                             //                            ss << plan_con.get_position().at(0) << ", " << plan_con.get_position().at(1) << ", " << plan_con.get_position().at(2) << ", ";
//                             //                            ss << plan_con.get_orientation().get_x() << ", " << plan_con.get_orientation().get_y() << ", " << plan_con.get_orientation().get_z() << ", " << plan_con.get_orientation().get_w();
//                             //
//                             //                            PRX_DEBUG_COLOR("END AT " << ss.str(), PRX_TEXT_RED);


//                             PRX_DEBUG_COLOR(" GOAL AT " << mo_space->print_point(poses_set[target_pose_index].grasped_set[j], 4), PRX_TEXT_CYAN);

//                             //PRX_DEBUG_POINT("SECOND LEG : " << i << ", " << j << " :: " << graph_query->plan.size());
//                             PRX_PRINT("SECOND LEG : " << i << ", " << j << " :: " << graph_query->plan.size(), PRX_TEXT_CYAN);
//                             //                            PRX_PRINT(graph_query->path.print(), PRX_TEXT_CYAN);

//                             if( tree_based_planning_mode )
//                             {
//                                 if( graph_query->plan.size() == 0 && reverse_transfer )
//                                 {
//                                     reverse_transfer = false;
//                                     continue;
//                                 }

//                                 if( graph_query->plan.size() == 0 && !reverse_transfer )
//                                 {
//                                     reverse_transfer = true;
//                                     j--;
//                                     continue;
//                                 }
//                             }

//                             if( graph_query->plan.size() != 0 )
//                             {
//                                 manip_success_counter++;
//                                 PRX_DEBUG_COLOR("\n\n\n Second plan worked " << model->get_state_space()->print_memory(3), PRX_TEXT_RED);

//                                 if( tree_based_planning_mode )
//                                 {
//                                     if( reverse_transfer )
//                                     {
// //                                        PRX_DEBUG_POINT("REVERSING PLAN");
//                                         reverse_transfer = false;
//                                         graph_query->plan.copy_onto_front(manip_control_space->clone_point(poses_set[target_pose_index].ungrasped_set[j]), graph_query->plan.at(0).duration);
//                                         graph_query->plan.at(0).control->memory.back() = 1;
//                                         graph_query->plan.reverse();
//                                     }
//                                 }
//                                 resolve_query_plan_2 = graph_query->plan;
//                                 path_length += graph_query->path.length();
//                                 PRX_PRINT(" Second leg path " << i << "::" << path_length, PRX_TEXT_BROWN);


//                                 PRX_DEBUG_COLOR("Going for the third part second plan size: " << ((motion_planning_query_t*)output_queries["second"])->plan.size(), PRX_TEXT_BROWN);
//                                 //((motion_planning_query_t*)output_queries["third"])->link_start(poses_set[target_pose_index].ungrasped_set[j]);

//                                 ((motion_planning_query_t*)output_queries["third"])->link_start(poses_set[target_pose_index].retracted_set[i]); //retracting_plans[j].back().control);

//                                 ////PRX_DEBUG_POINT(model->get_control_space()->print_point(poses_set[target_pose_index].retracting_plans[j].back().control, 4));
//                                 //We don't need to set the goal because always will be the safe state.
//                                 model->use_context(pc_name_real_world);



//                                 //UPDATING THE STATE SPACE TO REFLECT A SUCCESSFUL TRANSFER
//                                 //ASSUMING SIMILAR OBJECTS
//                                 PRX_DEBUG_COLOR("START " << object_state_space->print_point(poses_set[start_pose_index].state, 3) << " vs \nReal World Space " << real_object_space->print_memory(3), PRX_TEXT_BROWN);
//                                 for( unsigned ri = 0, obj_i = 0; ri < (real_object_space->get_dimension() / object_state_space->get_dimension()); ++ri, obj_i += object_state_space->get_dimension() )
//                                 {
//                                     PRX_DEBUG_COLOR(ri << "th Object", PRX_TEXT_RED);
//                                     double obj_dist = 0;
//                                     std::stringstream ss;
//                                     for( unsigned inner_i = obj_i; inner_i < obj_i + 3; ++inner_i )
//                                     {
//                                         ss << "\n" << poses_set[target_pose_index].state->at(inner_i - obj_i) << " / " << real_object_point_vec[inner_i];
//                                         obj_dist += pow((poses_set[start_pose_index].state->at(inner_i - obj_i) - real_object_point_vec[inner_i]), 2);
//                                     }
//                                     PRX_DEBUG_COLOR("Distance " << obj_dist << " between " << ss.str(), PRX_TEXT_CYAN);
//                                     if( obj_dist < 0.001 )
//                                     {
//                                         for( unsigned inner_i = obj_i; inner_i < obj_i + object_state_space->get_dimension(); ++inner_i )
//                                         {
//                                             real_object_point_vec[inner_i] = poses_set[target_pose_index].state->at(inner_i - obj_i);
//                                             real_object_space->copy_vector_to_point(real_object_point_vec, real_object_point);
//                                             real_object_space->copy_from_point(real_object_point);

//                                         }
//                                         PRX_DEBUG_COLOR("Moving object at index " << obj_i << " , point : " << real_object_space->print_memory(4) << " \nTarget was " << object_state_space->print_point(poses_set[target_pose_index].state, 3), PRX_TEXT_BLUE);

//                                         break;
//                                     }
//                                 }
//                                 PRX_DEBUG_COLOR("STARTED WITH " << real_object_space->print_point(real_initial_object_point, 3) << " vs \nReal World Space " << real_object_space->print_memory(3), PRX_TEXT_BROWN);

//                                 //#################UPDATION OF STATE SPACE COMPLETE BEFORE THIRD PHASE




//                                 all_plants.clear();
//                                 model->get_system_graph().get_plants(all_plants);

//                                 //all_plants[1]->set_active(true, all_plants[1]->get_pathname());

//                                 for( unsigned ap = 0; ap < all_plants.size(); ++ap )
//                                 {
//                                     PRX_DEBUG_COLOR(all_plants[ap]->get_pathname() << " is active -> " << all_plants[ap]->is_active() << " at " << all_plants[ap]->get_state_space()->print_memory(3), PRX_TEXT_MAGENTA);
//                                 }

//                                 //PRX_DEBUG_COLOR("REAL WORLD: " << model->get_state_space()->print_memory(4), PRX_TEXT_BROWN);
//                                 PRX_DEBUG_COLOR("BF: " << model->get_full_state_space()->print_memory(4), PRX_TEXT_CYAN);
//                                 //output_specifications[ungrasped_name]->validity_checker->link_model(model);
//                                 graph_specification = static_cast<motion_planning_specification_t*>(output_specifications[ungrasped_name]);
//                                 //planners[ungrasped_name]->link_specification(graph_specification);
//                                 if( tree_based_planning_mode )
//                                 {
//                                     if( tree_set_ungrasped )
//                                     {
//                                         planners[ungrasped_name]->reset();
//                                     }
//                                     else
//                                     {
//                                         tree_set_ungrasped = true;
//                                     }
//                                     graph_specification->clear_seeds();
//                                     graph_specification->add_seed(poses_set[target_pose_index].retracted_set[i]);
//                                     output_specifications[grasped_name]->get_stopping_criterion()->reset();
//                                     output_specifications[ungrasped_name]->get_stopping_criterion()->reset();


//                                     //                                    planners[ungrasped_name]->link_specification(graph_specification);
//                                     planners[ungrasped_name]->setup();
//                                 }
//                                 planners[ungrasped_name]->link_query(output_queries["third"]);
//                                 PRX_DEBUG_COLOR(manip_state_space->print_point(safe_state, 4), PRX_TEXT_CYAN);
//                                 PRX_DEBUG_COLOR(manip_state_space->print_point(poses_set[target_pose_index].retracted_set[i], 4), PRX_TEXT_CYAN);
//                                 //                                PRX_PRINT("\n\n USING CONTEXT" << pc_name_real_world, PRX_TEXT_RED);
//                                 _clock.reset();
//                                 try
//                                 {
//                                     if( tree_based_planning_mode )
//                                     {
//                                         PRX_DEBUG_COLOR("Executing tree", PRX_TEXT_MAGENTA);
//                                         planners[ungrasped_name]->execute();
//                                     }
//                                 }
//                                 catch( stopping_criteria_t::stopping_criteria_satisfied e )
//                                 {
//                                     //                        statistics = dynamic_cast<const prm_star_statistics_t*>(motion_planner->get_statistics());
//                                     //                        PRX_DEBUG_COLOR("Grasped statistics : |seed| : " << seed_num << " |V|: " << statistics->num_vertices << "    |E|:" << statistics->num_edges, PRX_TEXT_MAGENTA);

//                                 }
//                                 planners[ungrasped_name]->resolve_query();
//                                 //statistics_time += _clock.measure();
//                                 manip_try_counter++;
//                                 PRX_DEBUG_COLOR("AF: " << model->get_full_state_space()->print_memory(4), PRX_TEXT_RED);

//                                 //                                ss.str("");
//                                 //                                manip_state_space->copy_from_point(((motion_planning_query_t*)output_queries["third"])->plan.at(0).control);
//                                 //                                _manipulator->get_end_effector_configuration(plan_con);
//                                 //                                ss << plan_con.get_position().at(0) << ", " << plan_con.get_position().at(1) << ", " << plan_con.get_position().at(2) << ", ";
//                                 //                                ss << plan_con.get_orientation().get_x() << ", " << plan_con.get_orientation().get_y() << ", " << plan_con.get_orientation().get_z() << ", " << plan_con.get_orientation().get_w();
//                                 //
//                                 //                                PRX_DEBUG_COLOR("START FROM " << ss.str(), PRX_TEXT_GREEN);
//                                 //
//                                 //                                ss.str("");
//                                 //                                manip_state_space->copy_from_point(((motion_planning_query_t*)output_queries["third"])->plan.at(((motion_planning_query_t*)output_queries["third"])->plan.size() - 1).control);
//                                 //                                _manipulator->get_end_effector_configuration(plan_con);
//                                 //                                ss << plan_con.get_position().at(0) << ", " << plan_con.get_position().at(1) << ", " << plan_con.get_position().at(2) << ", ";
//                                 //                                ss << plan_con.get_orientation().get_x() << ", " << plan_con.get_orientation().get_y() << ", " << plan_con.get_orientation().get_z() << ", " << plan_con.get_orientation().get_w();
//                                 //
//                                 //                                PRX_DEBUG_COLOR("END AT " << ss.str(), PRX_TEXT_RED);


//                                 //PRX_DEBUG_POINT("THIRD LEG : " << i << "," << j << " :: " << ((motion_planning_query_t*)output_queries["third"])->plan.size());
//                                 PRX_PRINT("THIRD LEG : " << i << "," << j << " :: " << ((motion_planning_query_t*)output_queries["third"])->plan.size(), PRX_TEXT_BLUE);

//                                 if( ((motion_planning_query_t*)output_queries["third"])->plan.size() != 0 )
//                                 {
//                                     manip_success_counter++;
//                                     manip_found++;
//                                     PRX_DEBUG_COLOR("\n\n\n Third plan worked " << model->get_state_space()->print_memory(3), PRX_TEXT_RED);
//                                     resolve_query_plan_3 = ((motion_planning_query_t*)output_queries["third"])->plan;
//                                     path_length += ((motion_planning_query_t*)output_queries["third"])->path.length();
//                                     PRX_PRINT(" Third leg path " << i << "::" << path_length, PRX_TEXT_GREEN);
//                                     PRX_PRINT("!!!! FOUND A SOLUTION AT " << i << ", " << j << " WITH LENGTH " << path_length << " !!!  and minimum till now has been " << min_total_path_found << " !!!", PRX_TEXT_BLUE);
//                                     //PRX_DEBUG_POINT("!!!! FOUND A SOLUTION AT " << i << ", " << j << " WITH LENGTH " << path_length << " !!!  and minimum till now has been " << min_total_path_found << " !!!");
//                                     if( path_length < min_total_path_found )
//                                     {
//                                         min_total_path_found = path_length;
//                                         //mo_space->copy_from_point(graph_query->path.back());
//                                         //manip_state_space->copy_point(released_point,graph_query->path.back());
//                                         //model->propagate_plan(released_point,(motion_planning_query_t*)output_queries["third"])->plan,retract_path);
//                                         //PRX_ASSERT(specs->validity_checker->is_valid(retract_path));
//                                         manip_query->plan = ((motion_planning_query_t*)output_queries["first"])->plan;
//                                         //                                        manip_query->plan.append_last_onto_back(2);
//                                         manip_query->plan += poses_set[start_pose_index].reaching_plans[i];
//                                         manip_query->plan.copy_onto_back(manip_query->plan.back().control, manip_query->plan.back().duration);
//                                         manip_query->plan.back().control->memory.back() = 1;
//                                         ////PRX_DEBUG_POINT("\n 1st leg Plan::: \n" << ((motion_planning_query_t*)output_queries["first"])->plan.print());
//                                         //PRX_DEBUG_COLOR("EXTRAS: " << i << ") " << poses_set[start_pose_index].reaching_plans[i].size() << "  ,   " << poses_set[start_pose_index].reaching_plans[i].size(), PRX_TEXT_LIGHTGRAY);
//                                         //                                        manip_query->plan.append_last_onto_back(2);
//                                         //manip_query->plan += poses_set[start_pose_index].reaching_plans[i];
//                                         ////PRX_DEBUG_POINT("\n Reaching Plan::: \n" << poses_set[start_pose_index].reaching_plans[i].print());
//                                         //manip_query->plan.append_last_onto_back(2);
//                                         //manip_query->plan += poses_set[start_pose_index].lifting_plans[i];
//                                         ////PRX_DEBUG_POINT("\n Lifting Plan::: \n" << poses_set[start_pose_index].lifting_plans[i].print());
//                                         //manip_query->plan.append_last_onto_back(2);
//                                         //                                        for(unsigned plan_i=0;plan_i<((motion_planning_query_t*)output_queries["second"])->plan.size(); ++plan_i)
//                                         //                                        {
//                                         //                                            ((motion_planning_query_t*)output_queries["second"])->plan.at(plan_i).duration*=5;
//                                         //                                        }
//                                         manip_query->plan += ((motion_planning_query_t*)output_queries["second"])->plan;
//                                         manip_query->plan.copy_onto_back(manip_query->plan.back().control, manip_query->plan.back().duration);
//                                         manip_query->plan.back().control->memory.back() = 0;
//                                         //                                        manip_query->plan.append_last_onto_back(2);
//                                         manip_query->plan += poses_set[target_pose_index].retracting_plans[j];
//                                         ////PRX_DEBUG_POINT("\n 2nd leg Plan::: \n" << ((motion_planning_query_t*)output_queries["second"])->plan.print());
//                                         //                                        manip_query->plan.append_last_onto_back(2);
//                                         //manip_query->plan += poses_set[target_pose_index].placing_plans[j];
//                                         ////PRX_DEBUG_POINT("\n Placing Plan::: \n" << poses_set[target_pose_index].placing_plans[j].print());
//                                         //manip_query->plan.append_last_onto_back(2);
//                                         //manip_query->plan += poses_set[target_pose_index].retracting_plans[j];
//                                         ////PRX_DEBUG_POINT("\n Retracting Plan::: \n" << poses_set[target_pose_index].retracting_plans[j].print());
//                                         //PRX_DEBUG_COLOR("EXTRAS: " << j << ") " << poses_set[target_pose_index].placing_plans[j].size() << "  ,   " << poses_set[target_pose_index].retracting_plans[j].size(), PRX_TEXT_LIGHTGRAY);
//                                         manip_query->plan += ((motion_planning_query_t*)output_queries["third"])->plan;
//                                         ////PRX_DEBUG_POINT("\n 3rd leg Plan::: \n" << ((motion_planning_query_t*)output_queries["third"])->plan.print());

//                                     }
//                                     //                                    return;
//                                 }
//                             }
//                             if( min_total_path_found < PRX_INFINITY && !best_manipulation_combination )
//                             {
//                                 break;
//                             }
//                         }

//                     }
//                     if( min_total_path_found < PRX_INFINITY && !best_manipulation_combination )
//                     {
//                         break;
//                     }
//                 }

//                 if( min_total_path_found == PRX_INFINITY && !tree_based_planning_mode && false )
//                 {
//                     PRX_DEBUG_COLOR("Did not find a solution so expanding roadmap.....", PRX_TEXT_LIGHTGRAY);
//                     if( dynamic_cast<plan::prm_star_t*>(planners[ungrasped_name]) != NULL )
//                     {
//                         iteration_criterion_t* iter_ungrasp = (iteration_criterion_t*)(output_specifications[ungrasped_name]->get_stopping_criterion()->get_satisfied_criterion().at(0));
//                         iter_ungrasp->set_max_iterations(iter_ungrasp->get_max_iterations() + augment_iterations);
//                         PRX_DEBUG_COLOR("Augmenting to size " << iter_ungrasp->get_max_iterations(), PRX_TEXT_RED);
//                         //                        dynamic_cast<plan::prm_star_t*>(planners[ungrasped_name])->augment_roadmap_criterion(2);
//                     }
//                     if( dynamic_cast<plan::prm_star_t*>(planners[grasped_name]) != NULL )
//                     {
//                         iteration_criterion_t* iter_grasp = dynamic_cast<iteration_criterion_t*>(output_specifications[grasped_name]->get_stopping_criterion()->get_satisfied_criterion().at(0));
//                         iter_grasp->set_max_iterations(iter_grasp->get_max_iterations() + augment_iterations);
//                         //                        dynamic_cast<plan::prm_star_t*>(planners[grasped_name])->augment_roadmap_criterion(2);
//                     }
//                     model->use_context(pc_name_manipulator_only);
//                     try
//                     {
//                         if( !tree_based_planning_mode )
//                         {
//                             planners[ungrasped_name]->execute();
//                         }
//                     }
//                     catch( stopping_criteria_t::stopping_criteria_satisfied e )
//                     {
//                     }
//                     model->use_context(pc_name_manipulator_with_object);
//                     try
//                     {
//                         if( !tree_based_planning_mode )
//                         {
//                             planners[grasped_name]->execute();
//                         }
//                     }
//                     catch( stopping_criteria_t::stopping_criteria_satisfied e )
//                     {
//                     }
//                     manipulate(start_pose_index, target_pose_index);
//                 }
//                 statistics_time = _clock.measure();
//                 PRX_PRINT("!!!! FINALLY MINIMUM OF " << min_total_path_found << " !!!", PRX_TEXT_GREEN);
//                 PRX_PRINT("TOTAL TIME TAKEN TO SOLVE PROBLEM::: " << statistics_time, PRX_TEXT_MAGENTA);
//                 cloud_stat << "\nMin path\n" << min_total_path_found << "\nTime taken\n" << statistics_time << "\nPlanner tries\n" << manip_try_counter << "\nPlanner successes\n" << manip_success_counter << "\nManipulation paths found\n" << manip_found;
//                 cloud_stat.close();
//             }

//             void cloud_manipulation_tp_t::set_param(const std::string& parameter_name, const boost::any& value)
//             {
//                 PRX_DEBUG_COLOR("Set param from manipulation_tp : " << parameter_name, PRX_TEXT_MAGENTA);

//                 if( parameter_name == "serialize_flag" )
//                 {
//                     serialize_grasped_graph = boost::any_cast<bool>(value);
//                     serialize_ungrasped_graph = boost::any_cast<bool>(value);
//                 }
//                 else if( parameter_name == "deserialize_flag" )
//                 {
//                     deserialize_grasped_graph = boost::any_cast<bool>(value);
//                     deserialize_ungrasped_graph = boost::any_cast<bool>(value);
//                 }
//                 else if( parameter_name == "ungrasped_graph_file_name" )
//                 {
//                     ungrasped_graph_file_name = boost::any_cast<std::string>(value);
//                 }
//                 else if( parameter_name == "grasped_graph_file_name" )
//                 {
//                     grasped_graph_file_name = boost::any_cast<std::string>(value);
//                 }
//                 else if( parameter_name == "graph_builder" )
//                 {
//                     graph_builder = boost::any_cast<bool>(value);
//                 }
//             }

//             void cloud_manipulation_tp_t::update_vis_info() const
//             {

//                 foreach(planner_t* planner, planners | boost::adaptors::map_values)
//                 {
//                     planner->update_visualization();
//                 }
//             }

//             std::pair<const std::vector<pose_t>*, int> cloud_manipulation_tp_t::get_grasping_info() const
//             {
//                 return std::make_pair(&poses_set, poses_set_length);
//             }

//             bool cloud_manipulation_tp_t::valid_move(plan_t& plan, trajectory_t& path, const state_t* manip_start, const state_t* start, config_t & goal_config)
//             {
//                 PRX_DEBUG_COLOR("Valid_Move to " << goal_config.print(), PRX_TEXT_BLUE);
//                 if( _manipulator->IK_steering(plan, manip_control_space, manip_start, goal_config) )
//                 {
//                     PRX_DEBUG_COLOR("Going to propagate : " << manip_state_space->serialize_point(manip_start, 5), PRX_TEXT_BROWN);
//                     local_planner->propagate(start, plan, path);

//                     //TODO Validity checking might have some problem here

//                     if( path.size() != 0 && path.size() < 15 )// validity_checker->is_valid(path) )
//                     {
//                         return true;
//                     }
//                     else
//                         PRX_DEBUG_COLOR("Path  : \n" << path.print() << "   \nis not valid", PRX_TEXT_RED);
//                 }
//                 //                else
//                 //                    PRX_DEBUG_COLOR("IK steering false ", PRX_TEXT_RED);
//                 return false;
//             }

//             bool cloud_manipulation_tp_t::valid_grasp()
//             {
//                 mo_space->copy_from_point(grasped_point);
//                 manip_state_space->copy_to_point(released_point);
//                 object_state_space->copy_to_point(object_point);

//                 double x, y, z;
//                 _manipulator->get_end_effector_offset_configuration(tmp_config, grasped_point);
//                 tmp_config.get_position(x, y, z);
//                 PRX_DEBUG_COLOR("GRASPING AT (" << x << "," << y << "," << z << ")" << "[ " << tmp_config.get_orientation().get_x() << "," << tmp_config.get_orientation().get_y() << "," << tmp_config.get_orientation().get_z() << "," << tmp_config.get_orientation().get_w() << " ]", PRX_TEXT_BLUE);
//                 _manipulator->get_end_effector_offset_configuration(tmp_config, released_point);
//                 tmp_config.get_position(x, y, z);
//                 PRX_DEBUG_COLOR("RELEASED AT (" << x << "," << y << "," << z << ")" << "[ " << tmp_config.get_orientation().get_x() << "," << tmp_config.get_orientation().get_y() << "," << tmp_config.get_orientation().get_z() << "," << tmp_config.get_orientation().get_w() << " ]", PRX_TEXT_RED);
//                 //tmp_config.set_position(x, y, z + specs->raise_distance);
//                 tmp_config.set_position(x, y, z + specs->raise_distance);

//                 PRX_DEBUG_COLOR("Raise Distance::: " << specs->raise_distance, PRX_TEXT_GREEN);

//                 raise_path.clear();
//                 raise_plan.clear();
//                 //Here released point is still as the grasped point because the last element is still 1.
//                 if( true )//valid_move(raise_plan, raise_path, released_point, grasped_point, tmp_config) )
//                 {
//                     PRX_DEBUG_COLOR("\nGRASP:: " << mo_space->print_point(grasped_point, 3) << "\nRAISED:: " << manip_state_space->print_point(released_point, 3), PRX_TEXT_RED);
//                     //                    PRX_DEBUG_COLOR("\n RAISE PLAN:: " << raise_plan.print(), PRX_TEXT_CYAN);
//                     model->use_context(pc_name_manipulator_with_active_object);
//                     released_point->memory.back() = 0; //Now released point is the correct released point
//                     z = object_point->memory[2];
//                     object_point->memory[2] = -100;
//                     object_state_space->copy_from_point(object_point);
//                     if( validity_checker->is_valid(released_point) )
//                     {
//                         _manipulator->get_end_effector_offset_configuration(tmp_config, released_point);
//                         tmp_config.get_position(x, y, z);
//                         PRX_DEBUG_COLOR("THE MAGIC HAPPENS AT (" << x << "," << y << "," << z << ")" << "[ " << tmp_config.get_orientation().get_x() << "," << tmp_config.get_orientation().get_y() << "," << tmp_config.get_orientation().get_z() << "," << tmp_config.get_orientation().get_w() << " ]", PRX_TEXT_RED);

//                         _manipulator->get_end_effector_offset_configuration(tmp_config, released_point, 0, 0, -(specs->retract_distance));
//                         PRX_DEBUG_COLOR("RELEASED AT (" << tmp_config.get_position().at(0) << "," << tmp_config.get_position().at(1) << "," << tmp_config.get_position().at(2) << ")" << "[ " << tmp_config.get_orientation().get_x() << "," << tmp_config.get_orientation().get_y() << "," << tmp_config.get_orientation().get_z() << "," << tmp_config.get_orientation().get_w() << " ]", PRX_TEXT_RED);
//                         PRX_DEBUG_COLOR("Retract Distance::: " << specs->retract_distance, PRX_TEXT_GREEN);
//                         retract_path.clear();
//                         retract_plan.clear();
//                         object_point->memory[2] = z;
//                         object_state_space->copy_from_point(object_point);
//                         if( valid_move(retract_plan, retract_path, released_point, released_point, tmp_config) )
//                         {
//                             PRX_DEBUG_COLOR("\nGRASP:: " << mo_space->print_point(grasped_point, 3) << "\nRELEASED:: " << manip_state_space->print_point(released_point, 3), PRX_TEXT_RED);
//                             //PRX_DEBUG_COLOR("\n RETRACT PLAN :: " << retract_plan.print(), PRX_TEXT_BROWN);
//                             return true;
//                         }
//                     }
//                     else
//                     {
//                         PRX_DEBUG_COLOR("\nExiting because  " << manip_state_space->print_point(released_point, 3) << " is not valid in full space :: \n" << manip_state_space->serialize_point(released_point, 3), PRX_TEXT_RED);
//                     }
//                 }
//                 return false;
//             }

//             int cloud_manipulation_tp_t::similar_pose(state_t * pose)
//             {
//                 for( int i = 0; i < poses_set_length; ++i )
//                     if( poses_set[i].equal(object_state_space, pose) )
//                         return i;

//                 return -1;
//             }

//         }
//     }
// }
