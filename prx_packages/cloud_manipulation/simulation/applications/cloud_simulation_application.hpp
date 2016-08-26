// /**
//  * @file empty_application.hpp
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


// #include "prx/simulation/applications/empty_application.hpp"
// #include "prx_simulation/world_config_msg.h"
// #include "prx_simulation/state_msg.h"
// #include <std_msgs/String.h>

// #ifndef PRX_CLOUD_SIMULATION_APPLICATION_HPP
// #define	PRX_CLOUD__SIMULATION_APPLICATION_HPP
// namespace prx
// {

//     namespace packages
//     {
//         namespace cloud_manipulation
//         {

//             class cloud_simulation_application_t : public sim::empty_application_t
//             {

//               public:
//                 cloud_simulation_application_t();
//                 virtual ~cloud_simulation_application_t();
//                 virtual void init(const util::parameter_reader_t * const reader);
//                 virtual void received_plan_callback(const std_msgs::String& msg);

//               private:
//                 ros::NodeHandle nh;
//                 ros::Publisher change_world_pub;      
//                 ros::Subscriber received_plan_sub;
//                 prx_simulation::world_config_msg w_msg;
//                 prx_simulation::state_msg s_msg;
//                 util::config_t conf, shelf_config;
//             };
//         }

//     }
// }
// #endif


