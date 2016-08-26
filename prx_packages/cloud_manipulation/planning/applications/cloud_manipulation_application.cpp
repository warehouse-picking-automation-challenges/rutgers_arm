// /**
//  * @file single_query.cpp 
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

// #include "planning/applications/cloud_manipulation_application.hpp"

// #include "prx/planning/communication/planning_comm.hpp"
// #include "prx/planning/communication/simulation_comm.hpp"
// #include "prx/planning/communication/visualization_comm.hpp"
// #include "prx/planning/problem_specifications/motion_planning_specification.hpp"
// #include "prx/planning/queries/motion_planning_query.hpp"
// #include "prx/planning/world_model.hpp"
// #include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
// #include "prx/planning/task_planners/task_planner.hpp"
// #include "prx/planning/task_planners/multi_planner/multi_planner_query.hpp"
// #include "prx/planning/motion_planners/motion_planner.hpp"

// #include "prx/utilities/statistics/statistics.hpp"

// #include "prx/utilities/parameters/parameter_reader.hpp"
// #include "prx/utilities/communication/tf_broadcaster.hpp"
// #include "prx/utilities/definitions/string_manip.hpp"
// #include "planning/task_planners/cloud_manipulation_tp.hpp"

// #include <stdio.h>
// #include <cstdlib>
// #include <iostream>
// #include <string.h>
// #include <dirent.h>
// #include <time.h>
// #include <fstream>
// #include <sstream>
// #include <boost/range/adaptor/map.hpp>
// #include <pluginlib/class_list_macros.h>
// #include <unistd.h>

// PLUGINLIB_EXPORT_CLASS(prx::packages::cloud_manipulation::cloud_manipulation_application_t, prx::plan::planning_application_t)


// namespace prx
// {
//     using namespace util;
//     using namespace plan;

//     namespace plan
//     {
//         using namespace comm;
//     }

//     namespace packages
//     {
//         namespace cloud_manipulation
//         {

//             cloud_manipulation_application_t::cloud_manipulation_application_t()
//             {
//                 //            interruptable = false;                
//                 serialize_flag = false;
//                 deserialize_flag = false;
//                 graph_builder = false;
//                 graph_user = false;
//                 ready_to_deserialize = false;
//             }

//             cloud_manipulation_application_t::~cloud_manipulation_application_t() { }

//             void cloud_manipulation_application_t::init(const parameter_reader_t* reader)
//             {

//                 PRX_DEBUG_COLOR("Initializing cloud_manipulation_application_t ...", PRX_TEXT_CYAN);
//                 single_query_application_t::init(reader);
//                 PRX_DEBUG_COLOR("Done initializing single_query_application_t ...", PRX_TEXT_GREEN);
//                 sent_plan_pub = nh.advertise<std_msgs::String> ("/planning_ack", 1);
//                 changed_world_sub = nh.subscribe("/changed_world_query", 1, &cloud_manipulation_application_t::received_world_query_callback, this);
//                 waiting_for_simulation = false;
//                 toggle_ends = true;
//             }

//             void cloud_manipulation_application_t::received_world_query_callback(const prx_simulation::world_config_msg& msg)
//             {
//                 waiting_for_simulation = false;
//                 PRX_DEBUG_COLOR("\n\n\n JUST GOT BACK THE WORLD!!! ", PRX_TEXT_CYAN);

//                 foreach(prx_simulation::state_msg s_msg, msg.obstacle_configs)
//                 {

//                     config.set_position(s_msg.elements[0], s_msg.elements[1], s_msg.elements[2]);
//                     config.set_orientation(s_msg.elements[3], s_msg.elements[4], s_msg.elements[5], s_msg.elements[6]);

//                     static_cast<sim::obstacle_t*>(model->get_obstacles()[s_msg.node_name].get())->update_root_configuration(config);
//                     model->get_simulator()->update_obstacles_in_collision_checker();
//                     PRX_DEBUG_COLOR(" ROOT CONFIG " << static_cast<sim::obstacle_t*>(model->get_obstacles()[s_msg.node_name].get())->get_root_configuration().print(), PRX_TEXT_MAGENTA);

//                 }
//                 PRX_DEBUG_COLOR("\n\n\n", PRX_TEXT_RED);
//                 if( visualize )
//                 {
//                     PRX_INFO_S("Updating visualization geoms.");
//                     root_task->update_visualization();
//                     ((comm::visualization_comm_t*)comm::vis_comm)->send_geometries();
//                 }

//                 PRX_PRINT("MANIPULATION APPLICATION", PRX_TEXT_RED);


//                 //                ((prx::packages::cloud_manipulation::cloud_manipulation_tp_t*)(root_task))->manipulate(0, 1);
//                 //                viz_and_sim();
//                 //                PRX_DEBUG_POINT("First");


//                 double strt[7] = {0.1, 0.8, 2.93, 0, 0.70710678118, -0.70710678118, 0};
//                 double end[7] = {0.85, 0.52, 3.3675, 0, 0, 0, 1};
//                 //                toggle_ends = true;
//                 //                while( 1 )
//                 //                {

//                 std::vector< double > strt_pose;
//                 std::vector< double > end_pose;

//                 if( !toggle_ends )
//                 {
//                     strt_pose.assign(strt, strt + 7);
//                     end_pose.assign(end, end + 7);
//                     toggle_ends = true;
//                 }
//                 else
//                 {
//                     strt_pose.assign(end, end + 7);
//                     end_pose.assign(strt, strt + 7);
//                     toggle_ends = false;
//                 }


//                 std::string node_name = "simulator/obstacles/kiva_shelf1";
//                 sim::obstacle_t* obs = static_cast<sim::obstacle_t*>(model->get_obstacles()[node_name].get());

//                 PRX_DEBUG_COLOR("\n\n\n PLAN TESTING OBSTACLE CONFIG " << obs->get_root_configuration().print(), PRX_TEXT_GREEN);


//                 ((prx::packages::cloud_manipulation::cloud_manipulation_tp_t*)(root_task))->manipulate(strt_pose, end_pose);
//                 if( dynamic_cast<motion_planning_query_t*>(root_queries[0])->plan.size() > 0 )
//                 {



//                     std::vector<std::string> plant_names;
//                     model->get_system_graph().get_plant_names(plant_names);
//                     PRX_DEBUG_COLOR("\n\nPLANNING SIDE INFO   " << plant_names.size(), PRX_TEXT_CYAN);

//                     foreach(std::string name, plant_names)
//                     {
//                         PRX_DEBUG_COLOR(name, PRX_TEXT_CYAN);
//                     }
//                     std::stringstream out(std::stringstream::out);

//                     for( boost::unordered_map<std::string, sim::system_ptr_t, string_hash>::const_iterator val = model->get_obstacles().begin(); val != model->get_obstacles().end(); val++ )
//                     {
//                         sim::obstacle_t* obs = static_cast<sim::obstacle_t*>(val->second.get());
//                         out << "Key: " << val->first << " , " << obs->get_root_configuration().print() << std::endl;
//                     }
//                     PRX_DEBUG_COLOR(out.str(), PRX_TEXT_MAGENTA);

//                     waiting_for_simulation = true;
//                     viz_and_sim();


//                 }
//                 else
//                 {
//                     PRX_DEBUG_COLOR("\n\n\nFailed to find a solution...\n\n\n", PRX_TEXT_RED);
//                     toggle_ends = !toggle_ends;
//                     std_msgs::String msg;
//                     msg.data = "FAILED";
//                     sent_plan_pub.publish(msg);
//                 }

//             }

//             void cloud_manipulation_application_t::execute()
//             {

//                 //                PRX_DEBUG_POINT("Serialize: " << serialize_flag <<
//                 //                                "  Deserialize: " << deserialize_flag <<
//                 //                                "  grasped_file: " << grasped_graph_file_name <<
//                 //                                "  ungrapsed_file: " << ungrasped_graph_file_name <<
//                 //                                "  isBuilder:" << graph_builder <<
//                 //                                "  isUser: " << graph_user);


//                 PRX_PRINT("MANIPULATION APPLICATION", PRX_TEXT_RED);
//                 //single_query_application_t::execute();
//                 PRX_PRINT("Done with manip application execute", PRX_TEXT_RED);

//                 root_task->link_specification(root_specifications[0]);
//                 root_task->setup();
//                 root_task->link_query(root_queries[0]);
//                 root_task->execute();
//                 //                ((prx::packages::cloud_manipulation::cloud_manipulation_tp_t*)(root_task))->manipulate(0, 1);
//                 //                viz_and_sim();
//                 //                PRX_DEBUG_POINT("First");


//                 //                double strt[7] = {0.1, 0.8, 2.93, 0, 0.70710678118, -0.70710678118, 0};
//                 //                double end[7] = {0.85, 0.52, 3.3675, 0, 0, 0, 1};
//                 //                //                while( 1 )
//                 //                //                {
//                 //
//                 //                std::vector< double > strt_pose;
//                 //                std::vector< double > end_pose;
//                 //
//                 //                if( !toggle_ends )
//                 //                {
//                 //                    strt_pose.assign(strt, strt + 7);
//                 //                    end_pose.assign(end, end + 7);
//                 //                    toggle_ends = true;
//                 //                }
//                 //                else
//                 //                {
//                 //                    strt_pose.assign(end, end + 7);
//                 //                    end_pose.assign(strt, strt + 7);
//                 //                    toggle_ends = false;
//                 //                }
//                 //
//                 //
//                 //                std::string node_name = "simulator/obstacles/kiva_shelf1";
//                 //                sim::obstacle_t* obs = static_cast<sim::obstacle_t*>(model->get_obstacles()[node_name].get());
//                 //
//                 //                PRX_DEBUG_COLOR("\n\n\n PLAN TESTING OBSTACLE CONFIG " << obs->get_root_configuration().print(), PRX_TEXT_GREEN);
//                 //
//                 //
//                 //                ((prx::packages::cloud_manipulation::cloud_manipulation_tp_t*)(root_task))->manipulate(strt_pose, end_pose);
//                 //                if( dynamic_cast<motion_planning_query_t*>(root_queries[0])->plan.size() > 0 )
//                 //                {
//                 //
//                 //                    std::vector<std::string> plant_names;
//                 //                    model->get_system_graph().get_plant_names(plant_names);
//                 //                    PRX_DEBUG_COLOR("\n\nPLANNING SIDE INFO   " << plant_names.size(), PRX_TEXT_CYAN);
//                 //
//                 //                    foreach(std::string name, plant_names)
//                 //                    {
//                 //                        PRX_DEBUG_COLOR(name, PRX_TEXT_CYAN);
//                 //                    }
//                 //                    std::stringstream out(std::stringstream::out);
//                 //
//                 //                    for( boost::unordered_map<std::string, sim::system_ptr_t, string_hash>::const_iterator val = model->get_obstacles().begin(); val != model->get_obstacles().end(); val++ )
//                 //                    {
//                 //                        sim::obstacle_t* obs = static_cast<sim::obstacle_t*>(val->second.get());
//                 //                        out << "Key: " << val->first << " , " << obs->get_root_configuration().print() << std::endl;
//                 //                    }
//                 //                    PRX_DEBUG_COLOR(out.str(), PRX_TEXT_MAGENTA);
//                 //
//                 //                    waiting_for_simulation = true;
//                 //                    viz_and_sim();
//                 //
//                 //
//                 //                }
//                 //                else
//                 //                {
//                 //                    PRX_DEBUG_COLOR("\n\n\nFailed to find a solution...\n\n\n", PRX_TEXT_RED);
//                 //                    toggle_ends = !toggle_ends;
//                 //                    std_msgs::String msg;
//                 //                    msg.data = "FAILED";
//                 //                    sent_plan_pub.publish(msg);
//                 //                }
//                 int waiting=5;
//                 while(waiting>0)
//                 {
//                     sleep(1);
//                     waiting--;
//                 }
//                 std_msgs::String msg;
//                 msg.data = "START";
//                 sent_plan_pub.publish(msg);
//                 PRX_DEBUG_COLOR("Published start of planning...",PRX_TEXT_RED);
//                 while( 1 )
//                 {
//                     ros::spinOnce();
//                 }

//                 //                }
//                 //		PRX_PRINT("Done with first manipulation…",PRX_TEXT_CYAN);
//                 //                ((prx::packages::cloud_manipulation::cloud_manipulation_tp_t*)(root_task))->manipulate(1, 4);
//                 //                viz_and_sim();
//                 //                PRX_DEBUG_POINT("Second");
//                 //		PRX_PRINT("Done with second manipulation…",PRX_TEXT_CYAN);
//                 //                ((prx::packages::cloud_manipulation::cloud_manipulation_tp_t*)(root_task))->manipulate(2, 5);
//                 //                viz_and_sim();
//                 //		PRX_PRINT("Done with third manipulation…",PRX_TEXT_CYAN);
//                 //            this->root_task->execute();
//                 //                      
//                 //
//                 //            const statistics_t* stats = this->root_task->get_statistics();
//                 //
//                 //            if( stats != NULL )
//                 //            {
//                 //                PRX_INFO_S("Planner Statistics: " << stats->get_statistics());
//                 //            }
//                 //
//                 //            //If we have an initialized path smoother, attempt to smooth out the solutions we have
//                 //            if( path_smoother != NULL )
//                 //            {
//                 //                PRX_DEBUG_COLOR("Beginning path smoothing...", PRX_TEXT_CYAN);
//                 //                path_smoother->link_world_model(model);
//                 //                path_smoother->link_query(root_queries[0]);
//                 //                for( unsigned i = 0; i < 3; ++i ) //TODO : Should really try to set up the criterion for the smoother.
//                 //                {
//                 //                    path_smoother->step();
//                 //                }
//                 //                PRX_DEBUG_COLOR("Query smoothing succesfully completed!", PRX_TEXT_GREEN);
//                 //            }
//                 //





//             }

//             void cloud_manipulation_application_t::viz_and_sim()
//             {

//                 if( visualize )
//                 {
//                     PRX_INFO_S("Updating visualization geoms.");
//                     root_task->update_visualization();
//                     ((comm::visualization_comm_t*)comm::vis_comm)->send_geometries();
//                 }
//                 if( simulate )
//                 {
//                     PRX_PRINT("INSIDE SINGLE QUERY " << consumer_path, PRX_TEXT_RED);
//                     statistics_clock.reset();
//                     ((comm::planning_comm_t*)comm::plan_comm)->publish_plan(consumer_path, dynamic_cast<motion_planning_query_t*>(root_queries[0])->plan);

//                     while( 1.2 * dynamic_cast<motion_planning_query_t*>(root_queries[0])->plan.length() > statistics_clock.measure() )
//                     {
//                         sleep(1);
//                         PRX_PRINT("Slept at least once...", PRX_TEXT_BLUE);
//                     }
//                     std_msgs::String msg;
//                     msg.data = "PLANNED";
//                     sent_plan_pub.publish(msg);

//                     PRX_DEBUG_COLOR("End of application part " << dynamic_cast<motion_planning_query_t*>(root_queries[0])->plan.size(), PRX_TEXT_RED);
//                 }
//             }

//         }
//     }
// }
