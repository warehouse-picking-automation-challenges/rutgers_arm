// /**
//  * @file pap_trajectory_generator_tp.cpp
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

// #include "planning/task_planners/pap_trajectory_generator_tp.hpp"

// #include "prx/utilities/definitions/string_manip.hpp"
// #include "prx/utilities/definitions/random.hpp"
// #include "prx/utilities/math/configurations/config.hpp"
// #include "prx/utilities/statistics/statistics.hpp"
// #include "prx/utilities/distance_metrics/distance_metric.hpp"

// #include "prx/planning/motion_planners/motion_planner.hpp"
// #include "prx/planning/modules/samplers/sampler.hpp"
// #include "prx/planning/modules/validity_checkers/validity_checker.hpp"
// #include "prx/planning/modules/local_planners/local_planner.hpp"

// #include "planning/manipulation_world_model.hpp"
// #include "planning/queries/grasping_query.hpp"

// #include <yaml-cpp/yaml.h>
// #include <boost/range/adaptor/map.hpp>
// #include <boost/filesystem.hpp>
// #include "boost/filesystem/operations.hpp"
// #include "boost/filesystem/path.hpp"
// #include "pap_trajectory_generator_tp.hpp"
// #include <pluginlib/class_list_macros.h>


// PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::pap_trajectory_generator_tp_t ,prx::plan::planner_t)

// namespace prx
// {
//     using namespace util;
//     using namespace sim;
//     using namespace plan;

//     namespace packages
//     {
//         namespace manipulation
//         {

//             pap_trajectory_generator_tp_t::pap_trajectory_generator_tp_t()
//             {
//             }

//             pap_trajectory_generator_tp_t::~pap_trajectory_generator_tp_t()
//             {

//             }

//             void pap_trajectory_generator_tp_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
//             {
//                 PRX_INFO_S("Initializing pap_trajectory_generator_tp_t task planner ...");
//                 deserialize_plans = parameters::get_attribute_as<bool>("deserialize_plans", reader, template_reader, false);
//                 if (!deserialize_plans)
//                 {
//                     task_planner_t::init(reader,template_reader);
//                 }
//                 full_manipulator_context_name = parameters::get_attribute("full_manipulator_context_name", reader, template_reader);
//                 left_context_name = parameters::get_attribute("left_context_name", reader, template_reader);
//                 right_context_name = parameters::get_attribute("right_context_name", reader, template_reader);
//                 left_target_vec = parameters::get_attribute_as<std::vector<double> >("left_target", reader, template_reader);
//                 right_target_vec = parameters::get_attribute_as<std::vector<double> >("right_target", reader, template_reader);
//                 manipulation_task_planner_name = parameters::get_attribute("manipulation_task_planner_name", reader, template_reader);

                
//                 left_pose_file = parameters::get_attribute("left_pose_file", reader, template_reader);
//                 right_pose_file = parameters::get_attribute("right_pose_file", reader, template_reader);
//                 char* w = std::getenv("PRACSYS_PATH");
//                 std::string dir(w);
//                 dir += ("/prx_roadmaps/");

//                 std::string file = dir + left_pose_file; //the yaml file with the list of configurations
//                 YAML::Node doc1 = YAML::LoadFile(file);
//                 for(unsigned i=0;i<doc1.size();i++)
//                 {
//                     config_t config;
//                     std::vector<double> vec;

//                     vec.resize(7);
//                     vec[0]=doc1[i]["pose"][0].as<double>();
//                     vec[1]=doc1[i]["pose"][1].as<double>();
//                     vec[2]=doc1[i]["pose"][2].as<double>();
//                     vec[3]=doc1[i]["pose"][3].as<double>();
//                     vec[4]=doc1[i]["pose"][4].as<double>();
//                     vec[5]=doc1[i]["pose"][5].as<double>();
//                     vec[6]=doc1[i]["pose"][6].as<double>();

//                     config.set_position(vec[0],vec[1],vec[2]);
//                     config.set_xyzw_orientation(vec[3],vec[4],vec[5],vec[6]);
//                     potential_left_valid_poses.push_back(vec);
//                     if (deserialize_plans)
//                     {
//                         left_valid_poses.push_back(vec);
//                     }
//                     PRX_PRINT("Read in LEFT vec: [" << potential_left_valid_poses.back()[0] << "," << potential_left_valid_poses.back()[1] << "," << potential_left_valid_poses.back()[2] << ","
//                              << potential_left_valid_poses.back()[3] << "," << potential_left_valid_poses.back()[4] << "," << potential_left_valid_poses.back()[5] << "," << potential_left_valid_poses.back()[6], PRX_TEXT_MAGENTA);
//                 }

//                 file = dir + right_pose_file; //the yaml file with the list of configurations
//                 YAML::Node doc2 = YAML::LoadFile(file);
//                 for(unsigned i=0;i<doc2.size();i++)
//                 {
//                     config_t config;
//                     std::vector<double> vec;

//                     vec.resize(7);
//                     vec[0]=doc2[i]["pose"][0].as<double>();
//                     vec[1]=doc2[i]["pose"][1].as<double>();
//                     vec[2]=doc2[i]["pose"][2].as<double>();
//                     vec[3]=doc2[i]["pose"][3].as<double>();
//                     vec[4]=doc2[i]["pose"][4].as<double>();
//                     vec[5]=doc2[i]["pose"][5].as<double>();
//                     vec[6]=doc2[i]["pose"][6].as<double>();

//                     config.set_position(vec[0],vec[1],vec[2]);
//                     config.set_xyzw_orientation(vec[3],vec[4],vec[5],vec[6]);
//                     potential_right_valid_poses.push_back(vec);
//                     if (deserialize_plans)
//                     {
//                         right_valid_poses.push_back(vec);
//                     }
//                     PRX_PRINT("Read in RIGHT vec: [" << potential_right_valid_poses.back()[0] << "," << potential_right_valid_poses.back()[1] << "," << potential_right_valid_poses.back()[2] << ","
//                              << potential_right_valid_poses.back()[3] << "," << potential_right_valid_poses.back()[4] << "," << potential_right_valid_poses.back()[5] << "," << potential_right_valid_poses.back()[6], PRX_TEXT_MAGENTA);
//                 }
                
//             }

//             void pap_trajectory_generator_tp_t::link_world_model(world_model_t * const model)
//             {
//                 if (!deserialize_plans)
//                 {
//                     task_planner_t::link_world_model(model);
//                 }
//                 manipulation_model = dynamic_cast<manipulation_world_model_t*>(model);
//                 if(manipulation_model == NULL)
//                     PRX_FATAL_S("The manipulation task planner can work only with manipulation world model!");
//             }

//             void pap_trajectory_generator_tp_t::link_query(query_t* new_query)
//             {
//                 task_planner_t::link_query(new_query);
//                 in_query = static_cast<motion_planning_query_t*>(new_query);
//             }

//             void pap_trajectory_generator_tp_t::setup()
//             {
//                 manipulation_model->use_context(full_manipulator_context_name);
//                 manip_initial_state = manipulation_model->get_state_space()->alloc_point();
//                 full_state_space = manipulation_model->get_state_space();
//                 full_control_space = manipulation_model->get_control_space();
                
//                 manipulation_model->use_context(left_context_name);
//                 left_state_space = manipulation_model->get_current_manipulation_info()->full_arm_state_space;
//                 left_control_space = manipulation_model->get_current_manipulation_info()->full_arm_control_space;
                
//                 manipulation_model->use_context(right_context_name);
//                 right_state_space = manipulation_model->get_current_manipulation_info()->full_arm_state_space;
//                 right_control_space = manipulation_model->get_current_manipulation_info()->full_arm_control_space;
                
//                 manipulation_model->use_context(full_manipulator_context_name);

//                 if (!deserialize_plans)
//                 {
//                     foreach(planner_t* planner, planners | boost::adaptors::map_values)
//                     {
//                         PRX_INFO_S("setup planner: " << planner->get_name());
//                         planner->setup();
//                     }
//                 }
//             }

//             bool pap_trajectory_generator_tp_t::execute()
//             {
//                 if (!deserialize_plans)
//                 {
//                     foreach(planner_t* planner, planners | boost::adaptors::map_values)
//                     {                    
//                         PRX_INFO_S("execute planner: " << planner->get_name());
//                         planner->execute();
//                     }                
//                     config_t retract_config;
//                     retract_config.set_position(0,0,-.05);
//                     retract_config.set_orientation(0,0,0,1);
//                     PRX_DEBUG_COLOR("EXECUTE from pap_trajectory_generator_tp_t ...  left context:" << left_context_name,PRX_TEXT_RED);
//                     manipulation_model->use_context(full_manipulator_context_name);
//                     manipulation_model->get_state_space()->copy_from_point(manip_initial_state);  

//                     /** SOLVE FOR LEFT HAND FIRST */
//                     manipulation_model->use_context(left_context_name);
//                     std::vector<movable_body_plant_t* > objects;
//                     manipulation_model->get_objects(objects);
//                     const space_t* object_space = objects[0]->get_state_space();
//                     PRX_DEBUG_COLOR("LEFT ARM GRABS OBJECT:" << objects[0]->get_pathname(),PRX_TEXT_RED);

//                     manipulator = manipulation_model->get_current_manipulation_info()->manipulator;                
//                     state_t* initial_state = manipulation_model->get_state_space()->alloc_point();

//                     state_t* initial_object = object_space->alloc_point();
//                     state_t* target_object = object_space->alloc_point();
//                     object_space->copy_vector_to_point(left_target_vec, target_object);
//                     manipulation_query = new manipulation_query_t();

//                     sys_clock_t pap_timer;
//                     pap_timer.reset();
//                     for (unsigned i = 0; i < potential_left_valid_poses.size(); ++i)
//                     {
//                         PRX_STATUS("LEFT PAP PROGRESS: ["<< i << "/" << potential_left_valid_poses.size() << "]...", PRX_TEXT_BROWN);
//                         object_space->copy_vector_to_point(potential_left_valid_poses[i], initial_object);
//                         manipulation_query->setup(left_context_name, manipulation_query_t::PRX_PICK_AND_PLACE, objects[0], -1, 1, retract_config, initial_state, initial_state, NULL, initial_object, target_object );
//                         planners[manipulation_task_planner_name]->link_query(manipulation_query);
//                         //std::cin >> trash;
//                         planners[manipulation_task_planner_name]->resolve_query();
//                         if (manipulation_query->found_path)
//                         {
//                             PRX_PRINT("Found solution!", PRX_TEXT_CYAN);
//                             save_plan_to_file("left", left_valid_poses.size(), &manipulation_query->plan);
//                             left_valid_poses.push_back(potential_left_valid_poses[i]);
//                             left_grasps.push_back(manipulation_query->relative_grasp_config);
//                         }
//                         manipulation_query->clear();
//                     }

//                     PRX_PRINT("Finish time: " << pap_timer.measure(), PRX_TEXT_MAGENTA);
//                     save_valid_poses_to_file("left", left_grasps, left_valid_poses);

//                     object_space->free_point(initial_object);
//                     object_space->free_point(target_object);
//                     manipulation_model->get_state_space()->free_point(initial_state);



//                     /** SOLVE FOR RIGHT HAND NEXT */
//                     manipulation_model->use_context(right_context_name);
//                     object_space = objects[0]->get_state_space();
//                     PRX_DEBUG_COLOR("RIGHT ARM GRABS OBJECT:" << objects[0]->get_pathname(),PRX_TEXT_BLUE);

//                     manipulator = manipulation_model->get_current_manipulation_info()->manipulator;                
//                     state_t* initial_state2 = manipulation_model->get_state_space()->alloc_point();

//                     state_t* initial_object2 = object_space->alloc_point();
//                     state_t* target_object2 = object_space->alloc_point();
//                     object_space->copy_vector_to_point(right_target_vec, target_object2);
//                     manipulation_query_t* query2 = new manipulation_query_t();

//                     pap_timer.reset();
//                     for (unsigned i = 0; i < potential_right_valid_poses.size(); ++i)
//                     {
//                         PRX_STATUS("RIGHT PAP PROGRESS: ["<< i << "/" << potential_right_valid_poses.size() << "]...", PRX_TEXT_BROWN);
//                         object_space->copy_vector_to_point(potential_right_valid_poses[i], initial_object2);
//                         query2->setup(right_context_name, manipulation_query_t::PRX_PICK_AND_PLACE, objects[0], -1, 1, retract_config, initial_state2, initial_state2, NULL, initial_object2, target_object2 );
//                         planners[manipulation_task_planner_name]->link_query(query2);
//                         //std::cin >> trash;
//                         planners[manipulation_task_planner_name]->resolve_query();
//                         if (query2->found_path)
//                         {
//                             PRX_PRINT("Found solution!", PRX_TEXT_CYAN);
//                             save_plan_to_file("right", right_valid_poses.size(),&query2->plan);
//                             right_valid_poses.push_back(potential_right_valid_poses[i]);
//                             right_grasps.push_back(query2->relative_grasp_config);
//                         }
//                         query2->clear();
//                     }

//                     PRX_PRINT("\nFinish time: " << pap_timer.measure(), PRX_TEXT_MAGENTA);
//                     save_valid_poses_to_file("right", right_grasps, right_valid_poses);

//                     object_space->free_point(initial_object2);
//                     object_space->free_point(target_object2);
//                     manipulation_model->get_state_space()->free_point(initial_state2);
//                 }
//                 return true;
//             }

//             bool pap_trajectory_generator_tp_t::succeeded() const
//             {
//                 return true;
//             }

//             const util::statistics_t* pap_trajectory_generator_tp_t::get_statistics()
//             {
//                 return NULL;
//             }

//             void pap_trajectory_generator_tp_t::resolve_query()
//             {
                
//                 // Directory for reading plans
//                 char* w = std::getenv("PRACSYS_PATH");
//                 std::string dir(w);
//                 dir += ("/prx_roadmaps/trajectories/");
                
//                 PRX_DEBUG_COLOR("Resolve query from pap_trajectory_generator_tp_t ...  left context:" << left_context_name,PRX_TEXT_RED);
//                 manipulation_model->use_context(full_manipulator_context_name);
//                 manipulation_model->get_state_space()->copy_from_point(manip_initial_state);  

//                 /** SOLVE FOR LEFT HAND FIRST */
//                 manipulation_model->use_context(left_context_name);

//                 plan_t left_plan; left_plan.link_control_space(manipulation_model->get_control_space()); left_plan.link_state_space(manipulation_model->get_state_space());
//                 trajectory_t left_traj; left_traj.link_space(manipulation_model->get_state_space());
//                 state_t* initial_state = manipulation_model->get_state_space()->alloc_point();

//                 for (unsigned i = 0; i < left_valid_poses.size(); ++i)
//                 {
//                     // Clear plan and trajectory
//                     left_plan.clear();
//                     left_traj.clear();
                    
//                     // Reset state of the world
//                     manipulation_model->get_state_space()->copy_from_point(initial_state);
                
//                     // Open Plan
//                     std::stringstream ss;
//                     ss << dir << "left" << i << ".plan";

//                     std::string file = ss.str();

//                     std::ifstream fin;
//                     fin.open(file.c_str());
                    
//                     left_plan.read_from_stream(fin);
                    
//                     fin.close();
                    
//                     PRX_DEBUG_COLOR ("LEFT PLAN SIZE: " << left_plan.size(), PRX_TEXT_MAGENTA);
//                     // Propagate plan and get trajectory
//                     manipulation_model->propagate_plan(initial_state, left_plan, left_traj);
                    
                
//                     // Save traj
//                     std::stringstream ss2;
//                     ss2 << dir << "left" << i << ".traj";

//                     std::string file2 = ss2.str();

//                     std::ofstream fout2;
//                     fout2.open(file2.c_str());
                    
//                     left_traj.save_to_stream(fout2);
                    
//                     fout2.close();

//                 }
                
//                 // Reset state of the world
//                 manipulation_model->get_state_space()->copy_from_point(initial_state);

//                 /** SOLVE FOR RIGHT HAND FIRST */
//                 manipulation_model->use_context(right_context_name);
                
//                 plan_t right_plan; right_plan.link_control_space(manipulation_model->get_control_space()); right_plan.link_state_space(manipulation_model->get_state_space());
//                 trajectory_t right_traj; right_traj.link_space(manipulation_model->get_state_space());
//                 state_t* initial_state2 = manipulation_model->get_state_space()->alloc_point();

//                 for (unsigned i = 0; i < right_valid_poses.size(); ++i)
//                 {
//                     // Clear plan and trajectory
//                     right_plan.clear();
//                     right_traj.clear();
                    
//                     // Reset state of the world
//                     manipulation_model->get_state_space()->copy_from_point(initial_state2);
                
//                     // Open Plan
//                     std::stringstream ss;
//                     ss << dir << "right" << i << ".plan";

//                     std::string file = ss.str();

//                     std::ifstream fin;
//                     fin.open(file.c_str());
                    
//                     right_plan.read_from_stream(fin);
                    
//                     fin.close();
//                     PRX_DEBUG_COLOR ("RIGHT PLAN SIZE: " << right_plan.size(), PRX_TEXT_MAGENTA);
                    
//                     // Propagate plan and get trajectory
//                     manipulation_model->propagate_plan(initial_state2, right_plan, right_traj);
                    
                
//                     // Save traj
//                     std::stringstream ss2;
//                     ss2 << dir << "right" << i << ".traj";

//                     std::string file2 = ss2.str();

//                     std::ofstream fout2;
//                     fout2.open(file2.c_str());
                    
//                     right_traj.save_to_stream(fout2);
                    
//                     fout2.close();

//                 }

//             }
            
            
//             void pap_trajectory_generator_tp_t::save_plan_to_file(std::string arm, unsigned object_index, sim::plan_t* plan)
//             {
//                 char* w = std::getenv("PRACSYS_PATH");
//                 std::string dir(w);
//                 dir += ("/prx_roadmaps/trajectories/");
                
//                 std::stringstream ss;
//                 ss << dir << arm << object_index << ".plan";
                
//                 std::string file = ss.str();
                
//                 std::ofstream fout;
//                 fout.open(file.c_str());
//                 if (!fout.is_open())
//                 {
//                     PRX_FATAL_S ("Could not open file!" << file);
//                 }
//                 plan->save_to_stream(fout);
                
//                 fout.close();
//             }
            
//             void pap_trajectory_generator_tp_t::save_valid_poses_to_file(std::string arm, const std::vector< util::config_t >& valid_grasps, const std::vector< std::vector<double> >& valid_poses)
//             {
//                 char* w = std::getenv("PRACSYS_PATH");
//                 std::string dir(w);
//                 dir += ("/prx_roadmaps/trajectories/");
                
//                 std::stringstream ss;
//                 ss << dir << arm <<  ".valid_poses";
                
//                 std::string file = ss.str();
                
//                 std::ofstream fout;
//                 fout.open(file.c_str());
                
//                 PRX_ASSERT(fout.is_open());
//                 for (unsigned i = 0; i < valid_poses.size(); ++i)
//                 {
//                     fout << "-\n";
//                     fout << "  grasp: " << valid_grasps[i].print() << "\n";
//                     fout << "  pose: [";
//                     for (unsigned j = 0; j < 7; ++j)
//                     {
//                         fout << valid_poses[i][j];
//                         if (j !=6)
//                             fout << ',';
//                     }
//                     fout << "]\n";
//                 }
//                 fout.close();
//             }
//         }
//     }
// }

