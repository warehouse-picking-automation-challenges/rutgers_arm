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

// #include "planning/task_planners/pag_evaluation_tp.hpp"

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

// PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::pag_evaluation_tp_t, prx::plan::planner_t)

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

//             pag_evaluation_tp_t::pag_evaluation_tp_t()
//             {
//                 evaluate_left_arm = evaluate_left_arm = save_left_arm = save_right_arm = false;
//                 random_rotation = true;
//                 nr_object_poses = 0;
//             }

//             pag_evaluation_tp_t::~pag_evaluation_tp_t()
//             {

//             }

//             void pag_evaluation_tp_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
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

//                 /** Entering in an empty string in the context name skips evaluate of that particular arm */
//                 left_context_name = parameters::get_attribute("left_context_name", reader, template_reader);
//                 right_context_name = parameters::get_attribute("right_context_name", reader, template_reader);
//                 /** Entering in an empty string for left/right filename skips saving the valid poses to a file */
//                 if (parameters::has_attribute("left_filename", reader, template_reader))
//                 {
//                     left_filename = parameters::get_attribute("left_filename", reader, template_reader);
//                     save_left_arm = true;
//                 }
//                 if (parameters::has_attribute("right_filename", reader, template_reader))
//                 {
//                     right_filename = parameters::get_attribute("right_filename", reader, template_reader);
//                     save_right_arm = true;
//                 }
//                 /** The initial rotation parameters for the object */
//                 if (parameters::has_attribute("initial_rotation", reader, template_reader))
//                 {
//                     initial_rotation = parameters::get_attribute_as< std::vector<double> >("initial_rotation", reader, template_reader);
//                     random_rotation = false;
//                 }
//                 else
//                 {
//                     initial_rotation.resize(3,0.0);
//                 }
//                 if (parameters::has_attribute("rotation_axis", reader, template_reader))
//                 {
//                     rotation_axis = parameters::get_attribute_as< std::vector<double> >("rotation_axis", reader, template_reader);
//                     min_rotation_range = parameters::get_attribute_as< std::vector<double> >("min_rotation_range", reader, template_reader);
//                     max_rotation_range = parameters::get_attribute_as< std::vector<double> >("max_rotation_range", reader, template_reader);
//                     random_rotation = false;
//                 }
                
//                 /** The number of poses to be sampled and evaluated per arm */
//                 nr_object_poses = parameters::get_attribute_as<int>("nr_object_poses", reader, template_reader);
                
//                 /** Set up boolean guards*/
//                 if (!left_context_name.empty())
//                     evaluate_left_arm = true;
//                 if (!right_context_name.empty())
//                     evaluate_right_arm = true;
                
//             }

//             void pag_evaluation_tp_t::setup()
//             {
//                 PRX_INFO_S("Setup manipulation tp ...");
//                 grasping_planner->setup();
//                 grasping_query = new grasping_query_t();
//             }

//             void pag_evaluation_tp_t::reset()
//             {
//                 valid_right_poses.clear();
//                 valid_left_poses.clear();
//             }

//             void pag_evaluation_tp_t::link_world_model(world_model_t * const model)
//             {
//                 task_planner_t::link_world_model(model);
//                 manipulation_model = dynamic_cast<manipulation_world_model_t*>(model);
//                 if(manipulation_model == NULL)
//                     PRX_FATAL_S("The manipulation task planner can work only with manipulation world model!");
//                 grasping_planner->link_world_model(model);
//             }

//             const statistics_t* pag_evaluation_tp_t::get_statistics()
//             {
//                 PRX_WARN_S("Get statistics for manipulation task planner is not implemented!");
//                 return new statistics_t();
//             }

//             void pag_evaluation_tp_t::link_specification(specification_t* new_spec)
//             {
//                 task_planner_t::link_specification(new_spec);
//             }

//             void pag_evaluation_tp_t::link_query(query_t* new_query)
//             {
//                 task_planner_t::link_query(new_query);               
//             }        

//             bool pag_evaluation_tp_t::succeeded() const
//             {
//                 return true;
//                 //return false;
//             }

//             bool pag_evaluation_tp_t::execute()
//             {
//                 PRX_DEBUG_COLOR("PAG EVALUATION EXECUTE", PRX_TEXT_MAGENTA);
//                 std::vector<movable_body_plant_t* > objects;
//                 manipulation_model->get_objects(objects);

//                 config_t retract_config;
//                 retract_config.set_position(0,0,-.05);
//                 retract_config.set_orientation(0,0,0,1);
                
//                 if (evaluate_left_arm)
//                 {
//                     //left arm
//                     manipulation_model->use_context(left_context_name);
//                     movable_body_plant_t* chosen_object = objects[0];
//                     state_t* object_state = chosen_object->get_state_space()->alloc_point();
//                     int left_nr_grasps = grasping_planner->nr_grasps(left_context_name, chosen_object);

//                     for (int j = 0; j < nr_object_poses; ++j)
//                     {
//                         chosen_object->get_state_space()->uniform_sample(object_state);
//                         if (!random_rotation)
//                         {
//                             quaternion_t initial_quat(initial_rotation[0], initial_rotation[1], initial_rotation[2]);
//                             double theta1 = uniform_random(min_rotation_range[0], max_rotation_range[0]) * rotation_axis[0];
//                             double theta2 = uniform_random(min_rotation_range[1], max_rotation_range[1]) * rotation_axis[1];
//                             double theta3 = uniform_random(min_rotation_range[2], max_rotation_range[2]) * rotation_axis[2];
//                             quaternion_t new_rotation(theta1, theta2, theta3);
                            
//                             quaternion_t result_quat = initial_quat*new_rotation;
//                             if(!result_quat.is_valid())
//                             {
//                                 PRX_FATAL_S ("Invalid quaternion!");
//                             }
//                             std::vector<double> new_state;
//                             chosen_object->get_state_space()->copy_point_to_vector(object_state, new_state);
//                             new_state[3] = result_quat.get_x();
//                             new_state[4] = result_quat.get_y();
//                             new_state[5] = result_quat.get_z();
//                             new_state[6] = result_quat.get_w();
//                             chosen_object->get_state_space()->copy_vector_to_point(new_state, object_state);
                            
//                         }
//                         bool valid_grasp_found = false;
//                         for (int i = 0; i < left_nr_grasps && !valid_grasp_found; ++i)
//                         {
//                             std::string result = resolve_grasp_query(chosen_object,i,object_state,retract_config);
//                             if(result=="")
//                             {
//                                 std::vector<double> point_memory;
//                                 chosen_object->get_state_space()->copy_point_to_vector(object_state, point_memory);
//                                 valid_left_poses.push_back(point_memory);
//                                 valid_grasp_found = true;
//                             }
//                         }
                        
//                         PRX_STATUS("LEFT PAG PROGRESS: ["<< j << "/" << nr_object_poses << "]...", PRX_TEXT_BROWN);
//                     }
                    
//                     chosen_object->get_state_space()->free_point(object_state);
//                 }
                
//                 PRX_PRINT ("----", PRX_TEXT_CYAN);

//                 if (evaluate_right_arm)
//                 {
//                     //right arm
//                     manipulation_model->use_context(right_context_name);
//                     movable_body_plant_t* chosen_object = objects[0];
//                     state_t* object_state = chosen_object->get_state_space()->alloc_point();
//                     int right_nr_grasps = grasping_planner->nr_grasps(right_context_name, chosen_object);

//                     for (int j = 0; j < nr_object_poses; ++j)
//                     {
//                         chosen_object->get_state_space()->uniform_sample(object_state);
//                         if (!random_rotation)
//                         {
//                             quaternion_t initial_quat(initial_rotation[0], initial_rotation[1], initial_rotation[2]);
//                             double theta1 = uniform_random(min_rotation_range[0], max_rotation_range[0]) * rotation_axis[0];
//                             double theta2 = uniform_random(min_rotation_range[1], max_rotation_range[1]) * rotation_axis[1];
//                             double theta3 = uniform_random(min_rotation_range[2], max_rotation_range[2]) * rotation_axis[2];
//                             quaternion_t new_rotation(theta1, theta2, theta3);
                            
//                             quaternion_t result_quat = initial_quat*new_rotation;
//                             if(!result_quat.is_valid())
//                             {
//                                 PRX_FATAL_S ("Invalid quaternion!");
//                             }
//                             std::vector<double> new_state;
//                             chosen_object->get_state_space()->copy_point_to_vector(object_state, new_state);
//                             new_state[3] = result_quat.get_x();
//                             new_state[4] = result_quat.get_y();
//                             new_state[5] = result_quat.get_z();
//                             new_state[6] = result_quat.get_w();
//                             chosen_object->get_state_space()->copy_vector_to_point(new_state, object_state);
                            
//                         }
//                         bool valid_grasp_found = false;
//                         for (int i = 0; i < right_nr_grasps && !valid_grasp_found; ++i)
//                         {
//                             std::string result = resolve_grasp_query(chosen_object,i,object_state,retract_config);
//                             if (result=="")
//                             {
//                                 std::vector<double> point_memory;
//                                 chosen_object->get_state_space()->copy_point_to_vector(object_state, point_memory);
//                                 valid_right_poses.push_back(point_memory);
//                                 valid_grasp_found = true;
//                             }
//                         }
                        
//                         PRX_STATUS("RIGHT PAG PROGRESS: ["<< j << "/" << nr_object_poses << "]...", PRX_TEXT_BROWN);

//                     }
                    
//                     chosen_object->get_state_space()->free_point(object_state);
//                 }



                
//                 return true;
//             }

//             void pag_evaluation_tp_t::resolve_query()
//             {
//                 PRX_DEBUG_COLOR("PAG RESOLVE QUERY", PRX_TEXT_CYAN);
//                 char* w = std::getenv("PRACSYS_PATH");
//                 std::string dir(w);
//                 dir += ("/prx_roadmaps/");
//                 if (save_left_arm)
//                 {
//                     std::string file = dir + left_filename;
//                     std::ofstream fout;
//                     fout.open(file.c_str());
//                     PRX_ASSERT(fout.is_open());
//                     for (unsigned i = 0; i < valid_left_poses.size(); ++i)
//                     {
//                         fout << "-\n";
//                         fout << "  pose: [";
//                         for (unsigned j = 0; j < 7; ++j)
//                         {
//                             fout << valid_left_poses[i][j];
//                             if (j !=6)
//                                 fout << ',';
//                         }
//                         fout << "]\n";
//                     }
//                     fout.close();
//                 }
//                 if (save_right_arm)
//                 {
                    
//                     std::string file = dir + right_filename;
//                     std::ofstream fout;
//                     fout.open(file.c_str());
//                     PRX_ASSERT(fout.is_open());
//                     for (unsigned i = 0; i < valid_right_poses.size(); ++i)
//                     {
//                         fout << "-\n";
//                         fout << "  pose: [";
//                         for (unsigned j = 0; j < 7; ++j)
//                         {
//                             fout << valid_right_poses[i][j];
//                             if (j !=6)
//                                 fout << ',';
//                         }
//                         fout << "]\n";
//                     }
//                     fout.close();
//                 }
//             }

//             std::string pag_evaluation_tp_t::resolve_grasp_query(movable_body_plant_t* object, int grasping_index, state_t* object_state, config_t& retraction_config)
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
//                 manipulation_model->get_current_manipulation_info()->full_manipulator_state_space->free_point(our_full_manipulator);
//                 return grasping_query->reason_for_failure;
//             }


//         }
//     }
// }
