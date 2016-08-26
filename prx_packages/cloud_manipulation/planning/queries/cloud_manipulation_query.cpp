// /**
//  * @file manipulation_query.cpp
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

// #include "planning/queries/cloud_manipulation_query.hpp"

// #include "prx/utilities/parameters/parameter_reader.hpp"
// #include "prx/utilities/goals/goal.hpp"
// #include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"


// #include <pluginlib/class_list_macros.h> 
// #include <ros/ros.h>

// PLUGINLIB_EXPORT_CLASS(prx::packages::cloud_manipulation::cloud_manipulation_query_t, prx::plan::query_t)

// namespace prx
// {
//     using namespace util;
//     using namespace sim;

//     namespace packages
//     {
//         namespace cloud_manipulation
//         {

//             cloud_manipulation_query_t::cloud_manipulation_query_t()
//             {
//                 state_space = NULL;
//                 control_space = NULL;
//             }

//             cloud_manipulation_query_t::~cloud_manipulation_query_t()
//             {
//                 if( state_space == NULL )
//                     PRX_FATAL_S("Planning query does not have a linked state space!");

//                 clear();
//                 initial_poses.clear();
//                 target_poses.clear();
//             }

//             void cloud_manipulation_query_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
//             {
//                 motion_planning_query_t::init(reader, template_reader);

//                 if( parameters::has_attribute("initial_poses", reader, template_reader) )
//                 {

//                     foreach(const parameter_reader_t* s_reader, parameters::get_list("initial_poses", reader, template_reader))
//                     {

//                         initial_poses.push_back(s_reader->get_attribute_as<std::vector<double> >("pose"));
//                     }
//                 }
//                 else
//                 {
//                     PRX_WARN_S("Missing initial poses for the objects from manipulation query.");
//                 }

//                 if( parameters::has_attribute("target_poses", reader, template_reader) )
//                 {

//                     foreach(const parameter_reader_t* s_reader, parameters::get_list("target_poses", reader, template_reader))
//                     {

//                         target_poses.push_back(s_reader->get_attribute_as<std::vector<double> >("pose"));
//                     }
//                 }
//                 else
//                 {
//                     PRX_WARN_S("Missing target poses for the objects from manipulation query.");
//                 }

//                 if( parameters::has_attribute("extra_poses", reader, template_reader) )
//                 {

//                     foreach(const parameter_reader_t* s_reader, parameters::get_list("extra_poses", reader, template_reader))
//                     {

//                         extra_poses.push_back(s_reader->get_attribute_as<std::vector<double> >("pose"));
//                     }
//                 }

//             }

//             const std::vector< std::vector<double> >* cloud_manipulation_query_t::get_initial_poses() const
//             {
//                 return &initial_poses;
//             }

//             const std::vector<double>& cloud_manipulation_query_t::get_initial_pose() const
//             {
//                 return initial_poses[0];
//             }

//             const std::vector< std::vector<double> >* cloud_manipulation_query_t::get_target_poses() const
//             {
//                 return &target_poses;
//             }

//             const std::vector<double>& cloud_manipulation_query_t::get_target_pose() const
//             {
//                 return target_poses[0];
//             }

//             const std::vector< std::vector<double> >* cloud_manipulation_query_t::get_extra_poses() const
//             {
//                 return &extra_poses;
//             }

//             unsigned cloud_manipulation_query_t::total_poses() const
//             {
//                 return initial_poses.size() + target_poses.size() + extra_poses.size();
//             }

//             unsigned cloud_manipulation_query_t::important_poses() const
//             {
//                 return initial_poses.size() + target_poses.size();
//             }


//             void cloud_manipulation_query_t::set_initial_poses(std::vector< std::vector<double> > init_poses)
//             {
//                 initial_poses.clear();
//                 foreach(std::vector<double> pose, init_poses)
//                 {
//                     initial_poses.push_back(pose);
//                 }
//             }

//             void cloud_manipulation_query_t::set_target_poses(std::vector< std::vector<double> > tgt_poses)
//             {
//                 target_poses.clear();
//                 foreach(std::vector<double> pose, tgt_poses)
//                 {
//                     target_poses.push_back(pose);
//                 }
//             }
//         }
//     }
// }
