// /**
//  * @file face_hidden_grasping_planner.cpp
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

// #include "planning/task_planners/face_hidden_grasping_planner.hpp"

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
// #include <boost/filesystem/operations.hpp>
// #include <boost/filesystem/path.hpp>
// #include <boost/algorithm/string.hpp>


// #include <pluginlib/class_list_macros.h>

// PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::face_hidden_grasping_planner_t, prx::plan::planner_t)

// namespace fs = boost::filesystem;
// using namespace boost::algorithm;

// namespace prx
// {
//     using namespace util;
//     using namespace sim;
//     using namespace plan;

//     namespace packages
//     {
//         namespace manipulation
//         {

//             face_hidden_grasping_planner_t::face_hidden_grasping_planner_t()
//             {
//                 char* w = std::getenv("PRACSYS_PATH");
//                 std::string filename(w);
//                 pracsys_path = filename;

//                 original_object_state = NULL;
//             }

//             face_hidden_grasping_planner_t::~face_hidden_grasping_planner_t()
//             {

//             }
//             void face_hidden_grasping_planner_t::setup()
//             {
//                 const std::string directions[6] = {"x+","x-","y+","y-","z+","z-"};
//                 PRX_INFO_S("Setup grasping planner ...");
//                 std::vector<movable_body_plant_t* > objects;
//                 manipulation_model->get_objects(objects);

//                 foreach(movable_body_plant_t* object, objects)
//                 {
//                     std::string object_type =  object->get_object_type();
//                     for (std::vector<std::pair<std::string,std::string> >::iterator iter = data_folders.begin(); iter != data_folders.end(); ++iter)
//                     {
//                         std::string data_folder = iter->second;
//                         std::string hash_string = object_type+iter->first;
//                         if(face_grasps[hash_string].size()==0)
//                         {
//                             //search the entire directory for a file with this object's name, then categorize based on downward face
//                             for (int j = 0; j < 6; ++j)
//                             {
//                                 fs::path p(data_folder);
//                                 fs::directory_iterator end_iter;
//                                 for (fs::directory_iterator dir_itr(p); dir_itr != end_iter; ++dir_itr)
//                                 {
//                                     if (fs::is_regular_file(dir_itr->status()))
//                                     {
//                                         std::string file_name = dir_itr->path().string();
//                                         if(contains(file_name,object_type) && contains(file_name,directions[j]))
//                                         {
//                                             PRX_INFO_S(file_name);
//                                             //hooray, we can finally read the things
//                                             std::vector<grasp_entry_t> new_grasps;

//                                             YAML::Node doc = YAML::LoadFile(file_name);
//                                             for(unsigned i=0;i<doc.size();i++)
//                                             {
//                                                 config_t config;
//                                                 YAML::Node node = doc[i]["relative_config"];
//                                                 config.set_position(node[0].as<double>(),node[1].as<double>(),node[2].as<double>());
//                                                 config.set_xyzw_orientation(node[3].as<double>(),node[4].as<double>(),node[5].as<double>(),node[6].as<double>());
//                                                 int release_mode = 1;
//                                                 if(doc[i]["release_mode"])
//                                                     release_mode = doc[i]["release_mode"].as<int>();
//                                                 int grasp_mode = doc[i]["grasp_mode"].as<int>();
//                                                 new_grasps.push_back(grasp_entry_t(config,release_mode,grasp_mode));
//                                             }
//                                             face_grasps[hash_string].push_back(new_grasps);
//                                             PRX_INFO_S(new_grasps.size());
//                                         }

//                                     }
//                                 }
//                             }
//                         }
//                     }
//                 }
//             }       
//
//             int face_hidden_grasping_planner_t::determine_face_down(movable_body_plant_t* object)
//             {
//                 if(original_object_state == NULL)
//                     original_object_state = object->get_state_space()->alloc_point();
//                 else
//                     object->get_state_space()->copy_to_point(original_object_state);
//                 config_t config;
//                 object->get_configuration(config);
//                 quaternion_t quat;
//                 quat.zero();
//                 quat/=config.get_orientation();
//                 vector_t down_axis(0,0,-1);
//                 down_axis = quat.qv_rotation(down_axis);
//                 double max_val = fabs(down_axis[0]);
//                 int max_index = 0;
//                 if(fabs(down_axis[1]) > max_val)
//                 {
//                     max_val = fabs(down_axis[1]);
//                     max_index = 1;
//                 }
//                 if(fabs(down_axis[2]) > max_val)
//                 {
//                     max_val = fabs(down_axis[2]);
//                     max_index = 2;
//                 }
//                 // PRX_INFO_S(max_index);
//                 if(down_axis[max_index]<0)
//                 {
//                     return max_index*2+1;
//                 }
//                 else
//                 {
//                     return max_index*2;
//                 }
//             }

//             void face_hidden_grasping_planner_t::resolve_query()
//             {
//                 int grasp_database_index = determine_face_down(grasping_query->object);
//                 // PRX_INFO_S("db index: "<<grasp_database_index);
//                 std::vector<grasp_entry_t>* type_grasps = &face_grasps[grasping_query->object->get_object_type()+manipulation_model->get_current_manipulation_info()->full_arm_context_name][grasp_database_index];
//                 evaluate_the_query(type_grasps);
//             }

//             int face_hidden_grasping_planner_t::nr_grasps( std::string context_name,movable_body_plant_t* object)
//             {
//                 int grasp_database_index = determine_face_down(object);
//                 // PRX_INFO_S("db index: "<<grasp_database_index);
//                 return face_grasps[object->get_object_type()+context_name][grasp_database_index].size();
//             }
//         }
//     }
// }
