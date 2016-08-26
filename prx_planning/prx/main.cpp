/**
 * @file main.cpp 
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

#include <stdlib.h>

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/planning/applications/planning_application.hpp"

#include <boost/function.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/bind.hpp>
#include <boost/program_options.hpp>
#include <ros/callback_queue.h>
#include <time.h>
#include <sys/types.h>
#include <unistd.h>

using namespace prx;
using namespace boost::program_options;
using namespace util;
using namespace util::parameters;
using namespace sim;
using namespace plan;

int main( int ac, char* av[] )
{   
    std::string node_name;
    std::string filename="";
    if (ac == 1)
    {
        node_name = "planning";
    }
    else
    {
        node_name = av[1];
        // This skips the first 8 characters, which by default in ros is: __name:=
        std::string test = node_name.substr(0,8);
        if (test == "__name:=")
        {
            node_name = node_name.substr(8);
        }
    }

    ros::init(ac, av, node_name);
    PRX_PRINT("Starting Planning node with name: " << node_name, PRX_TEXT_BLUE);
    ros::NodeHandle main_node_handle;
    if(ros::param::has("yaml_input"))
    {
        ros::param::get("yaml_input",filename);
    }
    global_storage = new parameter_storage_t( "" );

    // Wait for parameter setting scripts to finish.
    while (ros::param::has("prx/parameter_mutex")) {}
    
    
    // std::string init_name = "prx/initialization/";
    // init_name+=node_name;
    // ros::param::set(init_name,true);    
    // if (ros::param::has("prx/initialization/order"))
    // {
    //     parameter_reader_t init_reader("prx/initialization/");
    //     //make sure nothing comes before this
    //     std::vector<std::string> node_list = init_reader.get_attribute_as<std::vector<std::string> >("order");
    //     unsigned pos=node_list.size();
    //     for(unsigned i=0;i<pos;i++)
    //     {
    //         if(node_list[i]==node_name)
    //             pos = i;
    //     }
    //     for(unsigned i=0;i<pos;i++)
    //     {
    //         while (ros::param::has("prx/initialization/"+node_list[i])) 
    //         {
    //             sleep(1);
    //         }
    //     }
    // }
    // else
    // {
    //     //assume waiting on a node called simulation        
    //     while (ros::param::has("prx/initialization/simulation")) 
    //     {
    //         sleep(1);
    //     }
    // }
    
    parameter_reader_t reader(node_name, global_storage);
    int random_seed;
    try
    {
        
        if(reader.has_attribute("random_seed"))
        {
            random_seed = reader.get_attribute_as<int>("random_seed");
            PRX_PRINT("Planning [" << node_name << "] has random seed: " << random_seed, PRX_TEXT_LIGHTGRAY);
            init_random(random_seed);
        }
        else
        {
            pid_t pid = getpid();
            unsigned int id = pid;
            random_seed = rand_r(&id);
            PRX_PRINT("Planning [" << node_name << "] is setting a truly random seed: " << random_seed, PRX_TEXT_LIGHTGRAY);

            init_random(random_seed);
        }
    }
    catch(...)
    {
        PRX_ERROR_S ("Exception caught trying to read random seed for prx_planning.");
    }
    
    if (reader.has_attribute("print_random_seed"))
    {
       
        if (reader.get_attribute_as<bool>("print_random_seed") == true)
        {
            std::ofstream fout;
            std::string filename = ros::this_node::getName() + "_random_seed.txt";
            PRX_PRINT("Planning [" << node_name << "] is saving random seed: " << random_seed << " to file: " << filename, PRX_TEXT_MAGENTA);

            fout.open(filename.substr(1).c_str(), std::fstream::app);
            fout << random_seed << std::endl;
            fout.close();
        }
    }
    
    PRX_DEBUG_S(" Planning application type to initialize: " << reader.get_attribute_as<std::string>("type"));

    planning_application_t* app = parameters::create_from_loader<planning_application_t>("prx_planning",&reader,"",NULL,"");

    // std::string type_name = reader.get_attribute_as<std::string>("type");
    // pluginlib::ClassLoader<planning_application_t>& loader = planning_application_t::get_loader(); 
    // planning_application_t* app = loader.createUnmanagedInstance("prx_planning/" + type_name);
    PRX_ASSERT(app != NULL);
    app->init(&reader);
    
    // ros::param::del(init_name); 
    // Run until the planner or ROS wants to stop.
    ros::getGlobalCallbackQueue()->callAvailable();
    
    app->execute();
    bool persistent = true;
    
    if(reader.has_attribute("persistent"))
    {
        persistent = reader.get_attribute_as<bool>("persistent");
        if(persistent)
        {
            PRX_PRINT("Planning node [" << node_name << "] is persistent, and will continue to run.", PRX_TEXT_BLUE);
        }
    }

    // std::string command = "mv /home/zak/repos/pracsys_ws/src/pracsys/prx_output/single_shot_output/planning_planner1.txt /home/zak/repos/pracsys_ws/src/pracsys/prx_output/single_shot_output/"+int_to_str(random_seed);

    // system(command.c_str());
    // This timer calls the frame function from task planner every 1/10th of a second

    if (persistent)
    {
        ros::MultiThreadedSpinner spinner(2);
        spinner.spin();
    }
//    while( persistent && ros::ok() )
//    {
//        ros::getGlobalCallbackQueue()->callAvailable();
////        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
////        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(simulation::simulation_step));
//    }

    return 0;
}


