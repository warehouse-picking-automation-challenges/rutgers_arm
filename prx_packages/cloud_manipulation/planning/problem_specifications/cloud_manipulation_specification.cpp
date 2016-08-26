// /**
//  * @file manipulation_specification.cpp
//  * 
//  * @copyright Software License Agreement (BSD License)
//  * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
//  * All Rights Reserved.
//  * For a full description see the file named LICENSE.
//  * 
//  * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
//  * 
//  * Email: pracsys@googlegroups.com
//  */

// #include "planning/problem_specifications/cloud_manipulation_specification.hpp"

// #include "prx/utilities/definitions/defs.hpp"
// #include "prx/utilities/parameters/parameter_reader.hpp"
// #include "prx/utilities/distance_metrics/distance_metric.hpp"
// #include "prx/planning/modules/samplers/sampler.hpp"
// #include "prx/planning/modules/validity_checkers/validity_checker.hpp"
// #include "prx/planning/modules/local_planners/local_planner.hpp"
// #include "prx/planning/world_model.hpp"

// #include <pluginlib/class_list_macros.h> 
// #include <ros/ros.h>

// PLUGINLIB_EXPORT_CLASS(prx::packages::cloud_manipulation::cloud_manipulation_specification_t, prx::plan::specification_t)

// namespace prx
// {
//     using namespace util;
//     using namespace plan;

//     namespace packages
//     {
//         namespace cloud_manipulation
//         {

//             cloud_manipulation_specification_t::cloud_manipulation_specification_t() { }

//             cloud_manipulation_specification_t::~cloud_manipulation_specification_t() { }

//             void cloud_manipulation_specification_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
//             {
//                 specification_t::init(reader, template_reader);
//                 z_on_table = parameters::get_attribute_as<double>("z_on_table", reader, template_reader, 0);
//                 max_tries = parameters::get_attribute_as<unsigned>("max_tries", reader, template_reader, 20);
//                 max_different_grasps = parameters::get_attribute_as<unsigned>("max_different_grasps", reader, template_reader, 3);
//                 retract_distance = parameters::get_attribute_as<double>("retract_distance", reader, template_reader, 0.1);
//                 raise_distance = parameters::get_attribute_as<double>("raise_distance", reader, template_reader, 0.03);
//                 num_poses = parameters::get_attribute_as<unsigned>("num_poses", reader, template_reader, 10);

//                 if( parameters::has_attribute("safe_position", reader, template_reader) )
//                     safe_position = parameters::get_attribute_as<std::vector<double> >("safe_position", reader, template_reader);
//                 else
//                     PRX_FATAL_S("No safe position is specified for the problem!");
//             }

//             void cloud_manipulation_specification_t::setup(world_model_t * const model)
//             {
//                 specification_t::setup(model);
//             }

//             void cloud_manipulation_specification_t::clear()
//             {
//                 specification_t::clear();
//             }

//             void cloud_manipulation_specification_t::link_spaces(const util::space_t* new_state_space, const util::space_t* new_control_space)
//             {
//                 this->state_space = new_state_space;
//                 this->control_space = new_control_space;
//             }
//         }
//     }
// }