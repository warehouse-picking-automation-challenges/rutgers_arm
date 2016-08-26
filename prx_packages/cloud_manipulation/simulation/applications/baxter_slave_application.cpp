/**
 * @file baxter_sim_application.cpp
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


#include "simulation/applications/baxter_slave_application.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx_simulation/plan_msg.h"
#include "prx_simulation/control_msg.h"
#include <pluginlib/class_list_macros.h>

#include "prx/simulation/communication/sim_base_communication.hpp"
#include "prx/simulation/communication/planning_comm.hpp"
#include "prx/simulation/communication/simulation_comm.hpp"
#include "prx/simulation/communication/visualization_comm.hpp"
#include "simulation/simulators/manipulation_simulator.hpp"
#include "simulation/systems/plants/movable_body_plant.hpp"

#include <string> 

#define MAX_MANIPULATIONS 10

PLUGINLIB_EXPORT_CLASS( prx::packages::manipulation::baxter_slave_application_t, prx::sim::application_t)

namespace prx
{
    using namespace util;
    using namespace sim::comm;
    namespace packages
    {        
        namespace manipulation
        {

            baxter_slave_application_t::baxter_slave_application_t() { }

            baxter_slave_application_t::~baxter_slave_application_t() { }

            void baxter_slave_application_t::init(const parameter_reader_t * const reader)
            {
                prx::sim::empty_application_t::init(reader);
                PRX_DEBUG_COLOR("Initialized SLAVE", PRX_TEXT_RED);
                
                
                simulator_state = simulator->get_state_space()->alloc_point();
                PRX_ERROR_S("\n\n\n SLAVE AT START "<<simulator->get_state_space()->print_point(simulator_state,4)<<"\n\n\n");
                simulator_running = true;
                counter = 0;
                frame_number = 0;
                master = reader->get_attribute_as<std::string>("application/master","simulation");
                slave_index = reader->get_attribute_as<int>("application/slave_index", 1);\
                num_slaves = reader->get_attribute_as<int>("application/num_slaves", 1);
                request_ground_truth_client = node.serviceClient<prx_simulation::request_ground_truth_srv>(master+"/master_ground_truth");
                PRX_DEBUG_COLOR("\n\n\n\n\n\nFrame topic: "<<master<<"/frame   "<<slave_index,PRX_TEXT_BROWN);
                
                frame_subscriber = node.subscribe(master+"/frame", 1, &baxter_slave_application_t::frame_callback, this);
                num_slaves_subscriber = node.subscribe(master+"/num_slaves", 1, &baxter_slave_application_t::num_slaves_callback, this);
                slave_response_pub = node.advertise<prx_simulation::state_msg> (master+"/slave_response", 1);
                
                ros::spinOnce();

            }

            void baxter_slave_application_t::num_slaves_callback(const std_msgs::String& msg)
            {
                std::stringstream ss(msg.data);
                std::string str = ss.str();
            }

            void baxter_slave_application_t::frame_callback(const std_msgs::String& msg)
            {
                PRX_ERROR_S("Frame callback ");
                std::stringstream ss(msg.data);
                std::string str = ss.str();
                frame_number = std::atoi(str.c_str());

                std::vector<double> state_vec;

                request_ground_truth_client.waitForExistence(ros::Duration(5));
                if (request_ground_truth_client.call(request_ground_truth_server))
                {
                    
                    state_vec =  request_ground_truth_server.response.full_state.elements;
                    PRX_DEBUG_COLOR("Successful ground truth call "<<state_vec.size(), PRX_TEXT_GREEN);
                    simulator->get_state_space()->set_from_vector(state_vec);// Sync the state with the master's state using the ground truth service
                }


                randomize_positions();
                PRX_WARN_S("SLAVE "<<slave_index<<" / "<<num_slaves);
                prx_simulation::state_msg slave_state;
                simulator->get_state_space()->copy_to_vector(slave_state.elements);
                PRX_ERROR_S("Publishing to slave response topic ");
                slave_response_pub.publish(slave_state); // Publish the full state of the slave after updating the objects it is controlling
            }

            void baxter_slave_application_t::randomize_positions()
            {
                std::vector<movable_body_plant_t* > objects;
                dynamic_cast<manipulation_simulator_t* >(simulator)->get_movable_objects(objects);
                std::vector<double> object_state_vec;
                for(int i=0;i<objects.size();++i)
                {
                    if(i%num_slaves!=(slave_index-1)) // Multiplexing the objects to only update the ones indicated by the slave_index
                        continue;

                    objects[i]->get_state_space()->copy_to_vector(object_state_vec);
                    object_state_vec[slave_index%2]+=0.01;
                    objects[i]->get_state_space()->set_from_vector(object_state_vec);
                    if(slave_index%2 == 0)
                        PRX_WARN_S(slave_index<<" ::: "<<i<<" : "<<objects[i]->get_state_space()->print_memory(4));
                    else
                        PRX_ERROR_S(slave_index<<" ::: "<<i<<" : "<<objects[i]->get_state_space()->print_memory(4));


                }

                simulator->get_state_space()->copy_to_point(simulator_state);
                simulator->push_state(simulator_state);
            }

            void baxter_slave_application_t::frame(const ros::TimerEvent& event)
            {
                
                //Doesn't need to do anything in this example

            }
        }


    }
}
