/**
 * @file BAXTER_SIM_application.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_BAXTER_SLAVE_APPLICATION_HPP
#define	PRX_BAXTER_SLAVE_APPLICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/applications/empty_application.hpp"
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "prx_simulation/request_ground_truth_srv.h"
#include "prx_simulation/state_msg.h"

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
            class baxter_slave_application_t : public prx::sim::empty_application_t
            {

              public:
                baxter_slave_application_t();
                virtual ~baxter_slave_application_t();
                virtual void init(const util::parameter_reader_t * const reader);
                virtual void frame_callback(const std_msgs::String& msg);
                virtual void num_slaves_callback(const std_msgs::String& msg);
                virtual void randomize_positions();
                virtual void frame(const ros::TimerEvent& event);
              private:
                ros::NodeHandle node;
                int counter;
                sim::state_t* simulator_state;
                ros::ServiceClient request_ground_truth_client;
                prx_simulation::request_ground_truth_srv request_ground_truth_server;
                std::string master;
                int num_slaves;
                int slave_index;
                ros::Subscriber frame_subscriber;
                ros::Subscriber num_slaves_subscriber;
                ros::Publisher slave_response_pub;
                unsigned int frame_number; 


            };
        }

    }
}

#endif

