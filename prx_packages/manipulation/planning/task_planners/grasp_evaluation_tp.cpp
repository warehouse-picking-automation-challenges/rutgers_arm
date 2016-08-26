// /**
//  * @file grasp_evaluation_tp.cpp
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

// #include "planning/task_planners/grasp_evaluation_tp.hpp"

// #include "prx/utilities/definitions/string_manip.hpp"
// #include "prx/utilities/definitions/random.hpp"
// #include "prx/utilities/math/configurations/config.hpp"
// #include "prx/utilities/statistics/statistics.hpp"
// #include "prx/utilities/definitions/sys_clock.hpp"

// #include "prx/planning/communication/visualization_comm.hpp"

// #include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"

// #include <boost/range/adaptor/map.hpp>
// #include <pluginlib/class_list_macros.h>
// #include <boost/algorithm/string.hpp>

// PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::grasp_evaluation_tp_t, prx::plan::planner_t)

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

//             grasp_evaluation_tp_t::grasp_evaluation_tp_t()
//             {

//             }

//             grasp_evaluation_tp_t::~grasp_evaluation_tp_t()
//             {

//             }

//             void grasp_evaluation_tp_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
//             {
//                 PRX_INFO_S("Initializing Manipulation task planner ...");

//                 std::string template_name;
//                 planner_t::init(reader, template_reader);

//                 //Initializing the grasping planner.
//                 if(!parameters::has_attribute("grasping_planner",reader,template_reader))
//                     PRX_FATAL_S("Manipulation task planner needs a grasping planner!");

//                 const parameter_reader_t* grasping_planner_template_reader = NULL;

//                 if( parameters::has_attribute("grasping_planner/template",reader,template_reader) )
//                 {
//                     std::string template_name = parameters::get_attribute("grasping_planner/template",reader,template_reader);
//                     grasping_planner_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + template_name, global_storage);
//                 }

//                 planner_t* tmp_grasping_planner = parameters::create_from_loader<planner_t > ("prx_planning", reader, "grasping_planner", grasping_planner_template_reader, "");
//                 grasping_planner = dynamic_cast<grasping_planner_t *>(tmp_grasping_planner);
//                 grasping_planner->set_pathname(path + "/grasping_planner");
//                 parameters::initialize(grasping_planner, reader, "grasping_planner", grasping_planner_template_reader, "");

//                 if( grasping_planner_template_reader != NULL )
//                 {
//                     delete grasping_planner_template_reader;
//                     grasping_planner_template_reader = NULL;
//                 }

//                 left_context_name = parameters::get_attribute("left_context_name", reader, template_reader);
//                 right_context_name = parameters::get_attribute("right_context_name", reader, template_reader);
//             }

//             void grasp_evaluation_tp_t::setup()
//             {
//                 PRX_INFO_S("Setup manipulation tp ...");
//                 grasping_planner->setup();
//                 grasping_query = new grasping_query_t();
//             }

//             void grasp_evaluation_tp_t::reset()
//             {
                
//             }

//             void grasp_evaluation_tp_t::link_world_model(world_model_t * const model)
//             {
//                 task_planner_t::link_world_model(model);
//                 manipulation_model = dynamic_cast<manipulation_world_model_t*>(model);
//                 if(manipulation_model == NULL)
//                     PRX_FATAL_S("The manipulation task planner can work only with manipulation world model!");
//                 grasping_planner->link_world_model(model);
//             }

//             const statistics_t* grasp_evaluation_tp_t::get_statistics()
//             {
//                 PRX_WARN_S("Get statistics for manipulation task planner is not implemented!");
//                 return new statistics_t();
//             }

//             void grasp_evaluation_tp_t::link_specification(specification_t* new_spec)
//             {
//                 task_planner_t::link_specification(new_spec);
//             }

//             void grasp_evaluation_tp_t::link_query(query_t* new_query)
//             {
//                 task_planner_t::link_query(new_query);               
//             }        

//             bool grasp_evaluation_tp_t::succeeded() const
//             {
//                 return true;
//                 //return false;
//             }

//             bool grasp_evaluation_tp_t::execute()
//             {
//                 //left arm
//                 manipulation_model->use_context(left_context_name);
//                 std::vector<movable_body_plant_t* > objects;
//                 manipulation_model->get_objects(objects);
//                 movable_body_plant_t* chosen_object = objects[0];
//                 state_t* object_state = chosen_object->get_state_space()->alloc_point();
//                 int left_nr_grasps = grasping_planner->nr_grasps(left_context_name, chosen_object);

//                 config_t retract_config;
//                 retract_config.set_position(0,0,-.03);
//                 retract_config.set_orientation(0,0,0,1);
//                 std::vector<std::string> reasons;
//                 sys_clock_t sys_clock;
//                 sys_clock.reset();
//                 for (int i = 0; i < left_nr_grasps; ++i)
//                 {
//                     std::string result = resolve_grasp_query(chosen_object,i,object_state,retract_config);
//                     reasons.push_back(result);
//                 }

//                 double left_total_time = sys_clock.measure();

//                 int left_successes = 0;
//                 int ik_failure = 0;
//                 int collisions = 0;
//                 foreach(std::string& s, reasons)
//                 {
//                     if(contains(s,"failed"))
//                     {
//                         ik_failure++;
//                     }
//                     if(contains(s,"invalid"))
//                     {
//                         collisions++;
//                     }
//                     if(s=="")
//                     {
//                         left_successes++;
//                     }
//                 }


//                 PRX_INFO_S("\n Stats: (Success:IK Failures:Collisions)("<<left_successes<<":"<<ik_failure<<":"<<collisions<<")/"<<left_nr_grasps<<" Total Time: "<<left_total_time<<" Time/Grasp: "<<(left_total_time/left_nr_grasps));
//                 PRX_INFO_S("\n "<<left_successes<<","<<ik_failure<<","<<collisions<<","<<left_total_time<<"\n");
//                 return true;
//             }

//             void grasp_evaluation_tp_t::resolve_query()
//             {
//             }

//             std::string grasp_evaluation_tp_t::resolve_grasp_query(movable_body_plant_t* object, int grasping_index, state_t* object_state, config_t& retraction_config)
//             {
//                 //We store the current state of the manipulator to restore the manipulator after the grasping planner is finished.
//                 state_t* our_full_manipulator = manipulation_model->get_current_manipulation_info()->full_manipulator_state_space->alloc_point();

//                 grasping_query->setup(  manipulation_model->get_current_manipulation_info()->full_arm_state_space, 
//                                         manipulation_model->get_current_manipulation_info()->full_arm_control_space, 
//                                         object, 
//                                         grasping_index, 
//                                         object_state, 
//                                         retraction_config);
//                 grasping_planner->link_query(grasping_query);
//                 grasping_planner->resolve_query();

//                 manipulation_model->get_current_manipulation_info()->full_manipulator_state_space->copy_from_point(our_full_manipulator);             
//                 return grasping_query->reason_for_failure;
//             }


//         }
//     }
// }
