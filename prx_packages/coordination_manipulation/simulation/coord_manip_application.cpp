/**
 * @file coord_manip_application.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */


#include "simulation/coord_manip_application.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "../../manipulation/simulation/systems/plants/movable_body_plant.hpp"
#include "../../manipulation/simulation/simulators/manipulation_simulator.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::coordination_manipulation::coord_manip_application_t, prx::sim::application_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace packages
    {
        namespace coordination_manipulation
        {     

            coord_manip_application_t::coord_manip_application_t() 
            {
                dropped_objects = 0;
            }

            coord_manip_application_t::~coord_manip_application_t() 
            {
                object_space->free_point(left_drop);
                object_space->free_point(right_drop);
            }
            
            void coord_manip_application_t::init(const util::parameter_reader_t* reader)
            {
                application_t::init(reader);
                std::vector<double> left_arm_dropoff, right_arm_dropoff;
                left_arm_dropoff = parameters::get_attribute_as<std::vector<double> >("application/left_arm_dropoff", reader, NULL);
                right_arm_dropoff = parameters::get_attribute_as<std::vector<double> >("application/right_arm_dropoff", reader, NULL);
                report_collisions = parameters::get_attribute_as<bool>("application/report_collisions", reader, NULL, false);
                
                std::vector<plant_t*> get_plants;
                //SGC: only need plant pointers here
                sys_graph.get_plants(get_plants);
//                std::vector<sys_ptr_t> get_systems;
//                sys_graph.get_systems(get_systems);

                foreach(plant_t* plant, get_plants)
                {
                    if( dynamic_cast<movable_body_plant_t*>(plant) != NULL )
                    {
                        objects.push_back( static_cast<movable_body_plant_t*>(plant) );
                    }
                    if( dynamic_cast<manipulator_t*>(plant) != NULL )
                    {
                        manipulator = dynamic_cast<manipulator_t*>(plant);
                    }
                
                }
                
                number_objects =  parameters::get_attribute_as<unsigned> ("application/num_assigned_objects", reader, NULL, objects.size());
                
//                foreach(sys_ptr_t sys, get_systems)
//                {
//                    if (dynamic_cast<consumer_controller_t*>(sys.get()) != NULL)
//                    {
//                        PRX_WARN_S ("Found consumer!");
//                        consumer_controller = dynamic_cast<consumer_controller_t*>(sys.get());
//                    }
//                }
                
                object_space = objects.front()->get_state_space();
                object_state = object_space->alloc_point();
                left_drop = object_space->alloc_point();
                object_space->set_from_vector(left_arm_dropoff, left_drop);
                right_drop = object_space->alloc_point();
                object_space->set_from_vector(right_arm_dropoff, right_drop);
                dropped_object_state = object_space->alloc_point();
                dropped_object_state->at(0) = -10;
                dropped_object_state->at(1) = -10;
                dropped_object_state->at(2) = -10;
                
                PRX_DEBUG_COLOR("Initialized coordination manipulation application!", PRX_TEXT_BLUE);
                
            }
            
            void coord_manip_application_t::frame(const ros::TimerEvent& event)
            {
                application_t::frame(event);
                
                // TODO: Figure out how to check if an object has been placed into a box
                
               for(unsigned i = 0; i < objects.size(); i++)
               {
                   objects[i]->get_state_space()->copy_to_point(object_state);
                   double dist_left = object_space->distance(left_drop, object_state);
                   double dist_right = object_space->distance(right_drop, object_state);
                  // PRX_DEBUG_COLOR("LEFT DROP: " << object_space->print_point(left_drop) << " vs object state: " << object_space->print_point(object_state), PRX_TEXT_CYAN);
                  // PRX_DEBUG_COLOR("RIGHT DROP: " << object_space->print_point(right_drop) << " vs object state: " << object_space->print_point(object_state), PRX_TEXT_CYAN);
                  // PRX_DEBUG_COLOR("LEFT DIST: " << dist_left << "  RIGHT_DIST: " << dist_right, PRX_TEXT_LIGHTGRAY);
                   if(dist_left < .15 || dist_right < .15)
                   {
                       PRX_WARN_S ("DROPPPPPED!");
                       dropped_objects++;
                       objects[i]->push_state(dropped_object_state);
                       simulator->get_state_space()->copy_to_point(simulation_state);
                       simulator->push_state(simulation_state);
                       objects[i]->set_active(false,"");
                   }
               }
            }
            
            void coord_manip_application_t::shutdown()
            {
                std::printf("------COORDINATION Application_t shutdown-----------");
                if (report_collisions)
                {
//                    double nr_of_collisions = dynamic_cast<manipulation_simulator_t*>(simulator)->get_nr_collisions();
//                    dynamic_cast<manipulation_simulator_t*>(simulator)->
//    //                std::ofstream fout;
//    //                fout.open("collision_results.txt", std::ofstream::app);
//    //                fout << algorithm << " " << experiment << " " << nr_of_collisions << std::endl;
//    //                fout.close();
//                    std::ofstream fout;
//                    fout.open("planning_results.txt", std::ofstream::app);
//                    fout << nr_of_collisions << std::endl;
//                    fout.close();
                }
            }
            

            void coord_manip_application_t::set_selected_path(const std::string& path)
            {
                std::string name;
                std::string subpath;
                boost::tie(name, subpath) = reverse_split_path(path);
                selected_path = name;
                PRX_DEBUG_COLOR("The selected path is now: " << selected_path.c_str(),PRX_TEXT_GREEN);
            }
        }

    }
}

