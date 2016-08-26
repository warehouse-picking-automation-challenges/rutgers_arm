/**
 * @file vo_application.cpp
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

#include "simulation/applications/GTA_application.hpp"
#include "prx/utilities/definitions/defs.hpp"

#include <pluginlib/class_list_macros.h>
//#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS( prx::packages::gta::GTA_application_t, prx::sim::application_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    
    namespace packages
    {
        namespace gta 
        {
            
            bool check_bounds(std::vector<bounds_t*>& bounds, state_t* check_state)
            {
                return (bounds[0]->is_valid(check_state->at(0)) &&  bounds[1]->is_valid(check_state->at(1)) );
            }
            
            GTA_application_t::GTA_application_t() { }
            
            
            GTA_application_t::~GTA_application_t() 
            {
                simulation_state_space->free_point(sampled_state);
                (*(consumer_controllers[0]->get_subsystems()->begin())).second->get_state_space()->free_point(sub_sampled_state);
            }
            
            void GTA_application_t::init(const parameter_reader_t * const reader)
            {
                PRX_DEBUG_COLOR ("Begin GTA application init", PRX_TEXT_RED);
                non_vo_avoidance = reader->get_attribute_as<bool>("application/non_vo_avoidance", false);
                if (non_vo_avoidance)
                    application_t::init(reader);
                else
                    VO_application_t::init(reader);
                
                timeout = reader->get_attribute_as<double>("application/timeout", 500.0);
                automatic = reader->get_attribute_as<bool>("application/automatic", false);
                wait_time = reader->get_attribute_as<double>("application/wait_time", 30);
                experiment_filename = reader->get_attribute_as<std::string>("application/experiment_filename", "results_");
                synchronized = reader->get_attribute_as<bool>("application/synchronized", false);
                random_start = reader->get_attribute_as<bool>("application/random_start", false);
                
                timeout_steps = timeout/simulation::simulation_step;
                GTA_replanning_controller_t* controller;
                system_ptr_t current_system;
                foreach(const parameter_reader_t * const body_reader, reader->get_list("application/consumer_paths"))
                {
                    std::string body_name = body_reader->get_attribute("");
                    PRX_DEBUG_COLOR("Read in controller path: " << body_name, PRX_TEXT_RED);

                    current_system = simulator->get_system(body_name);
                    controller = dynamic_cast<GTA_replanning_controller_t*>(current_system.get());
                    if( !controller )
                        PRX_FATAL_S("Pathname given for controller %s does not exist" << body_name);
                    consumer_controllers.push_back(controller);

                    delete body_reader;
                }
                sol_times.resize(consumer_controllers.size(), -1);
                path_lengths.resize(consumer_controllers.size(), -1);

                wait_timer.reset();
                timeout_timer.reset();
                simulator_running = false;
                
                
                sampled_state = simulation_state_space->alloc_point();
                sub_sampled_state = (*(consumer_controllers[0]->get_subsystems()->begin())).second->get_state_space()->alloc_point();

                if (random_start)
                {
                    find_random_starts();
                }
                                
            }
            
            void GTA_application_t::frame(const ros::TimerEvent& event)
            {
                if (automatic)
                {
                    if (loop_counter == 0 && wait_timer.measure() > wait_time)
                    {
                        PRX_DEBUG_COLOR("First step!", PRX_TEXT_BLUE);
                        simulator_running = true;
                    }
                    else if (loop_counter == 1)
                    {
                        if (!check_controllers_once())
                        {
                            simulator_running = false;
                        }
                        else
                        {
                            PRX_DEBUG_COLOR("Once", PRX_TEXT_GREEN);
                            simulator_running = true;
                        }
                    }
                    else if (loop_counter == 2)
                    {
                        if (!check_controllers_twice())
                        {
                            simulator_running = false;
                        }
                        else
                        {
                            PRX_DEBUG_COLOR("Twice", PRX_TEXT_BLUE);
                            simulator_running = true;
                        }
                    }

                }
                if (synchronized && loop_counter > 2)
                {
                    if (!check_controllers_synchronization())
                    {
                        simulator_running = false;
                    }
                    else
                    {
                        simulator_running = true;
                    }
                }
                //if (simulator_running)
                {
                    if (non_vo_avoidance)
                        application_t::frame(event);
                    else
                        VO_application_t::frame(event);
                    if (check_controllers_finished() || loop_counter > timeout_steps)
                    {
                        this->shutdown();
                        exit(0);
                    }
                }

            }
            
            void GTA_application_t::shutdown()
            {
                std::printf ("\n\n-------GTA_application shutdown--------\n\n");
                std::ofstream time_fout, path_fout;
                std::string time_filename = "INDIVIDUAL_" + ros::this_node::getName().substr(1) + "_times.txt";// + experiment_filename;
                std::string length_filename = "INDIVIDUAL_" + ros::this_node::getName().substr(1) + "_lengths.txt";// + experiment_filename;
//                time_filename.erase(time_filename.begin());
//                length_filename.erase(length_filename.begin());
                std::printf("Time filename: %s\n", time_filename.c_str());
                std::printf("Length filename: %s\n", length_filename.c_str());
                time_fout.open(time_filename.c_str(), std::fstream::app);
                
                double min_sol_time, max_sol_time;
                min_sol_time = max_sol_time = sol_times[0];
                double cumulative_sol_time = 0, avg_sol_time;
                
                // Get individual solution times
                for (size_t i = 0; i < sol_times.size(); i++)
                {
                    if (sol_times[i] > max_sol_time)
                        max_sol_time = sol_times[i];
                    if (sol_times[i] < min_sol_time)
                        min_sol_time = sol_times[i];
                    cumulative_sol_time += sol_times[i];
                    time_fout << sol_times[i] << ' ';
                }
                time_fout << std::endl;
                time_fout.close();
                
                // Output to accumulated time file
                avg_sol_time = cumulative_sol_time / sol_times.size();
                std::string total_time_filename = experiment_filename + "_times.txt";
                std::ofstream total_time_fout;
                total_time_fout.open(total_time_filename.c_str(), std::fstream::app);
                total_time_fout << avg_sol_time << " " << max_sol_time << " " << min_sol_time << std::endl;
                total_time_fout.close();
                
                
                double min_length, max_length;
                min_length = max_length = path_lengths[0];
                double cumulative_lengths = 0, avg_lengths;
                // Get individual path lengths
                path_fout.open(length_filename.c_str(), std::fstream::app);
                for (size_t i = 0; i < path_lengths.size(); i++)
                {
                    if (path_lengths[i] > max_length)
                        max_length = path_lengths[i];
                    if (path_lengths[i] < min_length)
                        min_length = path_lengths[i];
                    cumulative_lengths += path_lengths[i];
                    path_fout << path_lengths[i] << ' ';
                }
                path_fout << std::endl;
                path_fout.close();
                
                                
                // Output to accumulated length file
                avg_lengths = cumulative_lengths / path_lengths.size();
                std::string total_length_filename = experiment_filename + "_lengths.txt";
                std::ofstream total_lengths_fout;
                total_lengths_fout.open(total_length_filename.c_str(), std::fstream::app);
                total_lengths_fout << avg_lengths << " " << max_length << " " << min_length << std::endl;
                total_lengths_fout.close();
                
                for (size_t i = 0; i < consumer_controllers.size(); i++)
                {
                    ((sim::comm::simulation_comm_t*)sim::comm::sim_comm)->shutdown_node(ros::this_node::getName(), consumer_controllers[i]->get_planning_node_name());
                }
                
                std::printf("TOTAL Time filename: %s\n", total_time_filename.c_str());
                std::printf("TOAL Length filename: %s\n", total_length_filename.c_str());
                
                application_t::shutdown();
            }
            
            bool GTA_application_t::check_controllers_finished()
            {
//                PRX_ERROR_S("Check controller time!");
                bool finished = true;
                for (size_t i = 0; i < consumer_controllers.size(); i++)
                {
                    if (consumer_controllers[i]->is_finished())
                    {
                        if (sol_times[i] == -1)
                        {
                            sol_times[i] = loop_counter*simulation::simulation_step;
                            PRX_ERROR_S ("Sol time for : " << i << " is: " << sol_times[i]);
                            path_lengths[i] = consumer_controllers[i]->get_path_length();
                        }
                        
                    }
                    else
                    {
//                        PRX_ERROR_S ("Controller " << i << " finished is false!");
                        finished = false;
                    }
                }
                
                return finished;
            }
              
            bool GTA_application_t::check_controllers_once()
            {
                bool once = true;
                for (size_t i = 0; i < consumer_controllers.size(); i++)
                {
                    if (!consumer_controllers[i]->once)
                    {
//                        PRX_ERROR_S ("Controller " << i << " once is false!");
                        once = false;
                        break;
                    }
                }
                
                return once;
            }
            
            bool GTA_application_t::check_controllers_twice()
            {
                bool twice = true;
                for (size_t i = 0; i < consumer_controllers.size(); i++)
                {
                    if (!consumer_controllers[i]->twice)
                    {
//                        PRX_ERROR_S ("Controller " << i << " TWICE is false!");
                        twice = false;
                        break;
                    }
                }
                
                return twice;
            }
            
            bool GTA_application_t::check_controllers_synchronization()
            {
                bool sync = true;
                for (size_t i = 0; i < consumer_controllers.size(); i++)
                {
                    if (!consumer_controllers[i]->got_plan)
                    {
//                        PRX_ERROR_S ("Controller " << i << " TWICE is false!");
                        sync = false;
                        break;
                    }
                }
                
                return sync;
            }
            
            void GTA_application_t::initialize_sensing()
            {
                if(non_vo_avoidance)
                    application_t::initialize_sensing();
                else
                    VO_application_t::initialize_sensing();
            }
            
            void GTA_application_t::find_random_starts()
            {
//                do
//                {
//                    sampler.sample(simulation_state_space, sampled_state);
//                }
//                while (simulator->in_collision(sampled_state));
                
                // TODO: NOT GENERAL
                
                std::vector<bounds_t*> bounds_vec;
                bounds_vec.push_back(new bounds_t(-25,25));
                bounds_vec.push_back(new bounds_t(-25,25));
                
                for (size_t i = 0; i < consumer_controllers.size(); i++)
                {
                    (*(consumer_controllers[i]->get_subsystems()->begin())).second->set_active(true, "");
                    
                    do
                    {
//                        const_cast<space_t*>((*(consumer_controllers[i]->get_subsystems()->begin())).second->get_state_space())->set_bounds(cache_bounds);
                        (*(consumer_controllers[i]->get_subsystems()->begin())).second->get_state_space()->uniform_sample_near(sub_sampled_state,  consumer_controllers[i]->get_state, bounds_vec );
                        const_cast<space_t*>((*(consumer_controllers[i]->get_subsystems()->begin())).second->get_state_space())->copy_from_point(sub_sampled_state);
//                        const_cast<space_t*>((*(consumer_controllers[i]->get_subsystems()->begin())).second->get_state_space())->set_bounds(bounds_vec);
                    }
                    while (simulator->in_collision());
                    
                    consumer_controllers[i]->set_points_back();
                }
                
                simulation_state_space->copy_to_point(simulation_state);
                
                PRX_PRINT ("Done!", PRX_TEXT_BLUE);
                
                
            }
            
        }
        
    }
}
