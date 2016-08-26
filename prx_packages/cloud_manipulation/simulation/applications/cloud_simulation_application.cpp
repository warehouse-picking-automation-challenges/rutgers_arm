// /**
//  * @file empty_application.cpp
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


// #include "simulation/applications/cloud_simulation_application.hpp"
// #include <pluginlib/class_list_macros.h>
// #include <prx/simulation/systems/obstacle.hpp>

// PLUGINLIB_EXPORT_CLASS(prx::packages::cloud_manipulation::cloud_simulation_application_t, prx::sim::application_t)

// namespace prx
// {
//     using namespace sim;
//     using namespace util;
//     namespace packages
//     {
//         namespace cloud_manipulation
//         {

//             cloud_simulation_application_t::cloud_simulation_application_t() { }

//             cloud_simulation_application_t::~cloud_simulation_application_t() { }

//             void cloud_simulation_application_t::init(const parameter_reader_t * const reader)
//             {
//                 prx::sim::application_t::init(reader);
//                 PRX_DEBUG_COLOR("Initialized", PRX_TEXT_RED);
//                 received_plan_sub = nh.subscribe("/planning_ack", 1, &cloud_simulation_application_t::received_plan_callback, this);
//                 change_world_pub = nh.advertise<prx_simulation::world_config_msg> ("/changed_world_query", 1);
//                 s_msg.node_name = "simulator/obstacles/kiva_shelf1";
//                 PRX_DEBUG_COLOR("Subscribed to planning...",PRX_TEXT_MAGENTA);

//             }

//             void cloud_simulation_application_t::received_plan_callback(const std_msgs::String& msg)
//             {
                
//                 if(!simulator_running)
//                 {
//                     simulator_mode = 2;
//                     simulator_running = !simulator_running;
//                 }
//                 PRX_DEBUG_POINT("Received the message..." << msg.data.c_str());
//                 std::vector<std::string> plant_names;
//                 sys_graph.get_plant_names(plant_names);
//                 PRX_DEBUG_COLOR(plant_names.size(), PRX_TEXT_CYAN);

//                 foreach(std::string name, plant_names)
//                 {
//                     PRX_DEBUG_COLOR(name, PRX_TEXT_CYAN);
//                 }
//                 std::stringstream out(std::stringstream::out);

//                 for( boost::unordered_map<std::string, sim::system_ptr_t, string_hash>::const_iterator val = simulator->get_obstacles().begin(); val != simulator->get_obstacles().end(); val++ )
//                 {
//                     sim::obstacle_t* obs = static_cast<sim::obstacle_t*>(val->second.get());
//                     out << "Key: " << val->first << " , " << obs->get_root_configuration().print() << std::endl;
//                 }
//                 PRX_DEBUG_COLOR(out.str(), PRX_TEXT_MAGENTA);

//                 s_msg.node_name = "simulator/obstacles/kiva_shelf1";
//                 sim::obstacle_t* obs = static_cast<sim::obstacle_t*>(simulator->get_obstacles()[s_msg.node_name].get());
//                 conf = obs->get_root_configuration();
//                 double x = -0.1+(double) ((double)(std::rand() % 10) / (double)(50));
//                 double y = -0.1+(double) ((double)(std::rand() % 10) / (double)(50));
//                 if(std::rand()%10<6)
//                     x = 1;
// //                conf.set_position(conf.get_position()[0]+(-0.1 + ((double)((double)(std::rand() % 2) / 10))), conf.get_position()[1], conf.get_position()[2]);
//                 conf.set_position(x, y, conf.get_position()[2]);
//                 s_msg.elements.clear();
//                 s_msg.elements.push_back(conf.get_position()[0]);
//                 s_msg.elements.push_back(conf.get_position()[1]);
//                 s_msg.elements.push_back(conf.get_position()[2]);
//                 s_msg.elements.push_back(conf.get_orientation()[0]);
//                 s_msg.elements.push_back(conf.get_orientation()[1]);
//                 s_msg.elements.push_back(conf.get_orientation()[2]);
//                 s_msg.elements.push_back(conf.get_orientation()[3]);


//                 w_msg.obstacle_configs.clear();
//                 w_msg.obstacle_configs.push_back(s_msg);
                


//                 obs->update_root_configuration(conf);
//                 simulator->update_obstacles_in_collision_checker();
//                 update_obstacles_every_frame(true);
               
//                 PRX_DEBUG_COLOR("\n\nJUST PUBLISHING THE WORLD STATE MESSAGE "<<obs->get_root_configuration().print()<<"\n\n\n", PRX_TEXT_RED);

//                 change_world_pub.publish(w_msg);

//                 PRX_DEBUG_COLOR("changed_pub " << change_world_pub.getNumSubscribers(), PRX_TEXT_RED);
//             }

//         }


//     }
// }

