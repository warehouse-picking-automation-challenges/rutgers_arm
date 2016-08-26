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

#ifndef PRX_BAXTER_MASTER_APPLICATION_HPP
#define	PRX_BAXTER_MASTER_APPLICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/applications/empty_application.hpp"
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "prx_simulation/request_ground_truth_srv.h"

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
            class baxter_master_application_t : public prx::sim::empty_application_t
            {

              public:
                baxter_master_application_t();
                virtual ~baxter_master_application_t();
                virtual void init(const util::parameter_reader_t * const reader);
                virtual void  slave_callback(const prx_simulation::state_msg& msg);
                virtual void frame(const ros::TimerEvent& event);
                bool master_ground_truth_callback(prx_simulation::request_ground_truth_srv::Request& request, prx_simulation::request_ground_truth_srv::Response& response);
              private:
                ros::NodeHandle node;
                ros::Publisher frame_pub;   
                ros::Publisher num_slaves_pub;  
                ros::ServiceServer master_ground_truth_service;  
                ros::Subscriber slave_response_sub;  
                int counter;
                sim::state_t* simulator_state;
                unsigned int frame_number;
                std::string master_name;
                int responses_received;
                int num_slaves;

                std::vector< sim::state_t* > object_states; // The states of the objects being updated in the middle of every frame
                std::vector< sim::state_t* > slave_states; // The states of the objects returned by a slave during a callback of response
                std::vector< sim::state_t* > previous_states; // The states of the objects saved from the previous frame

                bool next_frame;
                bool processing_slave;

            };
        }

    }
}

#endif

