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


#include "simulation/applications/baxter_master_application.hpp"
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

PLUGINLIB_EXPORT_CLASS( prx::packages::manipulation::baxter_master_application_t, prx::sim::application_t)

namespace prx
{
    using namespace util;
    using namespace sim::comm;
    using namespace prx_simulation;
    namespace packages
    {        
        namespace manipulation
        {

            baxter_master_application_t::baxter_master_application_t() { }

            baxter_master_application_t::~baxter_master_application_t() { }

            void baxter_master_application_t::init(const parameter_reader_t * const reader)
            {
                prx::sim::empty_application_t::init(reader);
                PRX_DEBUG_COLOR("Initialized", PRX_TEXT_RED);
                master_name = reader->get_attribute_as<std::string>("application/name","simulation");
                num_slaves = reader->get_attribute_as<int>("application/num_slaves",1);
                slave_response_sub = node.subscribe(master_name+"/slave_response", 10, &baxter_master_application_t::slave_callback, this);
                frame_pub = node.advertise<std_msgs::String> (master_name+"/frame", 1);
                num_slaves_pub = node.advertise<std_msgs::String> (master_name+"/num_slaves", 1);
                simulator_state = simulator->get_state_space()->alloc_point();
                PRX_ERROR_S("\n\n\n MASTER AT START::: "<<simulator->get_state_space()->print_point(simulator_state,4)<<"\n\n\n");
                simulator_running = true;
                counter = 0;
                responses_received = 0;
                simulator_state = simulator->get_state_space()->alloc_point();
                master_ground_truth_service = node.advertiseService(master_name + "/master_ground_truth", &baxter_master_application_t::master_ground_truth_callback, this);
                frame_number = 0;
                next_frame = true;
                processing_slave = false;
            }


            bool baxter_master_application_t::master_ground_truth_callback(request_ground_truth_srv::Request& request, request_ground_truth_srv::Response& response)
            {
                PRX_DEBUG_COLOR("Master ground truth callback", PRX_TEXT_GREEN);
                

                simulator->get_state_space()->copy_to_point(simulator_state);
                simulator->get_state_space()->copy_point_to_vector(simulator_state, response.full_state.elements);

                return true;
            }
            void baxter_master_application_t::slave_callback(const prx_simulation::state_msg& msg)
            {
                processing_slave = true;


                PRX_INFO_S("Inside SLAVE CALLBACK");
                simulator->get_state_space()->set_from_vector(msg.elements);

                std::vector<movable_body_plant_t* > objects;
                dynamic_cast<manipulation_simulator_t* >(simulator)->get_movable_objects(objects);

                for(int i=0;i<objects.size();++i)
                {
                    objects[i]->get_state_space()->copy_to_point(slave_states[i]);
                    //Only update the states of the objects that have been changed by the slave
                    if(objects[i]->get_state_space()->distance(previous_states[i], slave_states[i])>PRX_ZERO_CHECK)
                    {
                        objects[i]->get_state_space()->copy_point(object_states[i], slave_states[i]);
                        // PRX_INFO_S("Updating "<<i<<" to "<<objects[i]->get_state_space()->print_point(object_states[i],2));
                    }
                    else
                    {
                        // PRX_ERROR_S("NOT Updating "<<i<<" to "<<objects[i]->get_state_space()->print_point(object_states[i],2));   
                    }
                }
                simulator->get_state_space()->copy_from_point(simulator_state); // Revert the state to what it was.
                processing_slave = false;
                responses_received++;

            }

            

            void baxter_master_application_t::frame(const ros::TimerEvent& event)
            {
                if(!next_frame) //FRAME MUTEX. This lets the code run in the current framework without changing the main.cpp and timer call of the frame()
                    return;


                next_frame = false;//MUTEX locked
                frame_number++;
                PRX_INFO_S("\n\n\n####################################\nFrame number "<<frame_number);
                ros::Rate rate(50);

                //Start of standard simulation frame operations
                handle_key();

                if( simulator_mode == 1 )
                {
                    if( simulator_counter > 0 )
                    {
                        simulator_running = true;
                        simulator_counter--;
                        loop_timer.reset();
                    }
                    else
                        simulator_running = false;
                }
                if( loop_timer.measure() > 1.0 )
                    loop_timer.reset();
                if( simulator_running )
                {

                    if( replays_states )
                    {
                        if( loop_counter > (int)replay_simulation_states.size() )
                            loop_counter = 0;
                        simulation_state_space->copy_from_point(replay_simulation_states[loop_counter]);
                        simulation_state_space->copy_to_point(simulation_state);
                    }
                    else
                    {
                        simulator->push_control(simulation_control);
                        simulator->propagate_and_respond();
                    }
                    if( stores_states )
                        store_simulation_states.copy_onto_back(simulation_state);

                    if( screenshot_every_simstep )
                    {
                        ((visualization_comm_t*)vis_comm)->take_screenshot(0, 1);
                    }
                    loop_total += loop_timer.measure_reset();
                    loop_counter++;
                    loop_avg = loop_total / loop_counter;

                }


                if(visualization_counter++%visualization_multiple == 0 )
                {
                    tf_broadcasting();
                }
                //End of standard simulation frame operations


                std::vector<movable_body_plant_t* > objects;
                dynamic_cast<manipulation_simulator_t* >(simulator)->get_movable_objects(objects);


                if(object_states.size()==0) // If the object point structures are empty, allocate the points
                {
                    for(int i=0;i<objects.size();++i)
                    {
                        object_states.push_back(objects[i]->get_state_space()->alloc_point());
                        slave_states.push_back(objects[i]->get_state_space()->alloc_point());
                        previous_states.push_back(objects[i]->get_state_space()->alloc_point());
                    }
                }
                else
                {
                    for(int i=0;i<objects.size();++i)
                    {
                        objects[i]->get_state_space()->copy_to_point(object_states[i]); // Copy over the previous state points by default. (Might be redundant if every slave responds correctly, however keeping it ensures consistency)
                    }

                }
                
                std::stringstream ss;  


                std_msgs::String frame;
                

                if(frame_number == 1)
                {    
                    sleep(5); // Allow time for the slave nodes to start. If time is not given for this, the first frame might be dropped.

                }



                ss.str("");
                ss<<frame_number;
                frame.data = ss.str();
                frame_pub.publish(frame);


                PRX_WARN_S("Frame "<<frame_number);
                
                int c = 0;
                while(ros::ok() && responses_received<num_slaves)// Wait for all the slaves to respond. 
                {
                    if(++c%100==0)
                        PRX_ERROR_S("Sleeping");
                    rate.sleep();
                    ros::spinOnce();
                }
                PRX_WARN_S("Ending sleep with "<<responses_received<<" versus "<<num_slaves);
                PRX_WARN_S("Frame "<<frame_number);
                for(int i=0;i<objects.size();++i)
                {
                    PRX_INFO_S(objects[i]->get_state_space()->print_point(object_states[i],2));
                    objects[i]->get_state_space()->copy_from_point(object_states[i]); // Compile all the updated object_states
                }
                simulator->get_state_space()->copy_to_point(simulator_state);
                simulator->push_state(simulator_state); // Push compiled full state, so that internal states of the manipulator simulator is updated



                for(int i=0;i<objects.size();++i)
                {
                    objects[i]->get_state_space()->copy_to_point(previous_states[i]); // Current state becomes previous state for the next frame
                }

                responses_received = 0;
                next_frame = true; // Release MUTEX


            }
        }


    }
}
