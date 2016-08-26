/**
 * @file place_objects_application.cpp
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


#include "simulation/place_objects_application.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "../../manipulation/simulation/systems/plants/movable_body_plant.hpp"
#include "../../manipulation/simulation/simulators/manipulation_simulator.hpp"

#include <yaml-cpp/yaml.h>
#include <pluginlib/class_list_macros.h>
#include <fstream>

PLUGINLIB_EXPORT_CLASS(prx::packages::coordination_manipulation::place_objects_application_t, prx::sim::application_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace packages
    {
        using namespace manipulation;
        namespace coordination_manipulation
        {     

            place_objects_application_t::place_objects_application_t() 
            {
                left_assigned = right_assigned = false;
                valid_placement_counter = 0;
                failed_placement_counter = 0;
            }

            place_objects_application_t::~place_objects_application_t() 
            {
            }
            
            void place_objects_application_t::init(const util::parameter_reader_t* reader)
            {
                application_t::init(reader);
                
                results_file = parameters::get_attribute_as<std::string>("application/results_file", reader, NULL, "results"); // where to output the results of a valid placement
                max_tries = parameters::get_attribute_as<int>("application/max_tries", reader, NULL, 10); // the maximum number of tries before restarting placement
                /** Read in the different types of objects and the maximum index number of each object type*/
                object_types = parameters::get_attribute_as<std::vector< std::string > >("application/object_types", reader, NULL);
                std::vector<int> pose_limits = parameters::get_attribute_as<std::vector<int> >("application/pose_limits", reader, NULL);
                // Ensure that there is a maximum number of poses associated with each object **/
                PRX_ASSERT(object_types.size()==pose_limits.size());
                
                for(unsigned i = 0; i < object_types.size(); ++i)
                {
                    max_pose_limit[object_types[i]] = pose_limits[i];
                    left_object_assignments[object_types[i]] = 0;
                    right_object_assignments[object_types[i]] = 0;
                }
                
                /** IF each arm has a specific number of objects assigned to them */
                
                if (reader->has_attribute("left_object_assignment"))
                {
                    std::vector<unsigned> left_object_assignment = parameters::get_attribute_as<std::vector< unsigned > >("application/left_object_assignment", reader, NULL);
                    PRX_ASSERT(object_types.size()==left_object_assignment.size());
                    for(unsigned i = 0; i < object_types.size(); ++i)
                    {
                        left_object_assignments[object_types[i]] = left_object_assignment[i];
                    }
                    
                    left_assigned = true;
                }
                
                
                if (reader->has_attribute("left_object_assignment"))
                {
                    std::vector<unsigned> right_object_assignment = parameters::get_attribute_as<std::vector< unsigned > >("application/right_object_assignment", reader, NULL);
                    PRX_ASSERT(object_types.size()==right_object_assignment.size());
                    for(unsigned i = 0; i < object_types.size(); ++i)
                    {
                        right_object_assignments[object_types[i]] = right_object_assignment[i];
                    }
                    
                    right_assigned = true;
                
                }
                
                /** Points to the directory right before the object specific directory*/
                pose_directory = parameters::get_attribute("application/pose_directory", reader, NULL);
                char* w = std::getenv("PRACSYS_PATH");
                std::string dir(w);

                std::string file = dir + pose_directory; //the yaml file with the list of configurations
                
                /** Read in the poses for each object for each arm */
                for(unsigned i = 0; i < object_types.size(); ++i)
                {
                    std::string current_type = object_types[i];
                    std::string left_file = file + current_type + "/left.valid_poses";
                    YAML::Node doc1 = YAML::LoadFile(left_file);
                    
                    unsigned counter = 0;
                    for(unsigned i=0;i<doc1.size() && counter < max_pose_limit[current_type];++i)
                    {
                        std::vector<double> vec;

                        vec.resize(7);
                        vec[0]=doc1[i]["pose"][0].as<double>();
                        vec[1]=doc1[i]["pose"][1].as<double>();
                        vec[2]=doc1[i]["pose"][2].as<double>();
                        vec[3]=doc1[i]["pose"][3].as<double>();
                        vec[4]=doc1[i]["pose"][4].as<double>();
                        vec[5]=doc1[i]["pose"][5].as<double>();
                        vec[6]=doc1[i]["pose"][6].as<double>();

                        left_valid_poses[current_type].push_back(vec);
                        ++counter;
                    }

                    std::string right_file = file + current_type + "/right.valid_poses";
                    YAML::Node doc2 = YAML::LoadFile(right_file);
                    
                    counter = 0;
                    for(unsigned i=0;i<doc2.size() && counter < max_pose_limit[current_type];++i)
                    {
                        std::vector<double> vec;
                        vec.resize(7);
                        vec[0]=doc2[i]["pose"][0].as<double>();
                        vec[1]=doc2[i]["pose"][1].as<double>();
                        vec[2]=doc2[i]["pose"][2].as<double>();
                        vec[3]=doc2[i]["pose"][3].as<double>();
                        vec[4]=doc2[i]["pose"][4].as<double>();
                        vec[5]=doc2[i]["pose"][5].as<double>();
                        vec[6]=doc2[i]["pose"][6].as<double>();

                        right_valid_poses[current_type].push_back(vec);
                        ++counter;
                    }
                }
                
                std::vector<plant_t*> get_plants;
                //SGC: only need plant pointers here
                sys_graph.get_plants(get_plants);
//                std::vector<sys_ptr_t> get_systems;
//                sys_graph.get_systems(get_systems);

                foreach(plant_t* plant, get_plants)
                {
                    movable_body_plant_t* test = dynamic_cast<movable_body_plant_t*>(plant);
                    if( test != NULL )
                    {
                        objects.push_back( test );
                    }
                }
                
                for(unsigned i = 0; i < objects.size(); ++i)
                {
                    movable_body_plant_t* test = objects[i];
                    std::string object_type = test->get_object_type();
                    // Make sure no foreign movable bodies exist in the scene
                    if (left_valid_poses.find(object_type) == left_valid_poses.end())
                    {
                        PRX_FATAL_S ("Movable body of type: " << object_type << " was not declared in object_types parameter!");
                    }
                    
                    /** Construct placement struct */
                    object_placement* new_placement;
                    new_placement = new object_placement();
                    new_placement->valid = false;
                    new_placement->object_index = -1;
                    new_placement->object_type = object_type;
                    new_placement->object = test;
                    
                    /** If there is a pre-existing distribution of objects, use it */
                    if (left_assigned || right_assigned)
                    {
                        if (left_object_assignments[object_type] - 1 >= 0)
                        {
                            left_object_assignments[object_type]--;
                            current_left_assignments.push_back(new_placement);
                        }
                        else if (right_object_assignments[object_type] - 1 >= 0)
                        {
                            right_object_assignments[object_type]--;
                            current_right_assignments.push_back(new_placement);
                        }
                    } 
                    else
                    {
                        /** Otherwise evenly distribute*/
                        if(i%2 == 0)
                        {
                            current_left_assignments.push_back(new_placement);
                        }
                        else
                        {
                            current_right_assignments.push_back(new_placement);
                        }
                    }
                }
                
                object_space = objects.front()->get_state_space();
                object_state = object_space->alloc_point();
                
                PRX_DEBUG_COLOR("Initialized coordination manipulation application!", PRX_TEXT_BLUE);
                bool valid_placement = false;
                while (!valid_placement)
                {
                    bool reset_all = (failed_placement_counter < max_tries);
                    valid_placement = create_object_placement(reset_all); // create initial placement, reset all assignments to new poses
                    // TODO: Save placement
                }
                ++valid_placement_counter;
                save_placements_to_file();
            }
            
            void place_objects_application_t::frame(const ros::TimerEvent& event)
            {
                application_t::frame(event);
                
                if (simulator_running && simulator_mode == 1)
                {
                    bool valid_placement = false;
                    while (!valid_placement)
                    {
                        bool reset_all = (failed_placement_counter < max_tries);
                        valid_placement = create_object_placement(reset_all); // create initial placement, reset all assignments to new poses
                        // TODO: Save placement
                    }
                    ++valid_placement_counter;
                    save_placements_to_file();
                }
            }
            
            void place_objects_application_t::shutdown()
            {
                std::printf("------COORDINATION Application_t shutdown-----------");

            }
            

            void place_objects_application_t::set_selected_path(const std::string& path)
            {
                std::string name;
                std::string subpath;
                boost::tie(name, subpath) = reverse_split_path(path);
                selected_path = name;
                PRX_DEBUG_COLOR("The selected path is now: " << selected_path.c_str(),PRX_TEXT_GREEN);
            }
            
            bool place_objects_application_t::create_object_placement(bool reset_all)
            {
                for (unsigned i = 0; i < current_left_assignments.size(); i++)
                {
                    std::string object_type = current_left_assignments[i]->object_type;
                    if(!current_left_assignments[i]->valid || reset_all)
                    {
                        int random_index;
                        do
                        {
                            random_index = rand() % max_pose_limit[object_type];
                        }
                        while(random_index==current_left_assignments[i]->object_index);
                        
                        current_left_assignments[i]->object_index = random_index;
                        current_left_assignments[i]->pose = left_valid_poses[object_type][random_index];
                    }
                }
                for (unsigned i = 0; i < current_right_assignments.size(); i++)
                {
                    std::string object_type = current_right_assignments[i]->object_type;
                    if(!current_right_assignments[i]->valid || reset_all)
                    {
                        int random_index;
                        do
                        {
                            random_index = rand() % max_pose_limit[object_type];
                        }
                        while(random_index==current_right_assignments[i]->object_index);
                        
                        current_right_assignments[i]->object_index = random_index;
                        current_right_assignments[i]->pose = right_valid_poses[object_type][random_index];
                    }
                }
                
                place_objects_state();
                return set_object_validity();
            }
            
            void place_objects_application_t::place_objects_state()
            {
                for (unsigned i = 0; i < current_left_assignments.size(); i++)
                {
                    object_space->set_from_vector(current_left_assignments[i]->pose, object_state);
                    current_left_assignments[i]->object->push_state(object_state);
                }
                for (unsigned i = 0; i < current_right_assignments.size(); i++)
                {
                    object_space->set_from_vector(current_right_assignments[i]->pose, object_state);
                    current_right_assignments[i]->object->push_state(object_state);
                }
                simulator->get_state_space()->copy_to_point(simulation_state);
                simulator->push_state(simulation_state);
            }
            
            bool place_objects_application_t::set_object_validity()
            {
                bool no_collisions = true;
                collision_list_t* t = simulator->get_colliding_bodies();
                for (unsigned i = 0; i < current_left_assignments.size(); i++)
                {
                    if (t->is_the_system_in_the_list(current_left_assignments[i]->object->get_pathname()))
                    {
                        current_left_assignments[i]->valid = false;
                        no_collisions = false;
                    }
                    else
                        current_left_assignments[i]->valid = true;
                }
                for (unsigned i = 0; i < current_right_assignments.size(); i++)
                {
                    if (t->is_the_system_in_the_list(current_right_assignments[i]->object->get_pathname()))
                    {
                        current_right_assignments[i]->valid = false;
                        no_collisions = false;
                    }
                    else
                        current_right_assignments[i]->valid = true;
                }
                
                return no_collisions;
            }
            
            void place_objects_application_t::save_placements_to_file()
            {
                char* w = std::getenv("PRACSYS_PATH");
                std::string dir(w);
                dir += ("/prx_output/");
                
                std::stringstream ss;
                ss << dir << results_file << valid_placement_counter << ".txt";
                
                std::ofstream fout;
                fout.open(ss.str().c_str());
                
                if (!fout.is_open())
                {
                    PRX_FATAL_S ("Could not save to file: " << ss.str());
                }
                fout << current_left_assignments.size() << std::endl;
                for (unsigned i = 0; i < current_left_assignments.size(); i++)
                {
                    fout << current_left_assignments[i]->object_type << "," << current_left_assignments[i]->object_index << std::endl;
                    fout << current_left_assignments[i]->pose[0] << "," << current_left_assignments[i]->pose[1] << "," << current_left_assignments[i]->pose[2] << "," 
                            << current_left_assignments[i]->pose[3] << "," << current_left_assignments[i]->pose[4] << "," << current_left_assignments[i]->pose[5] << "," << current_left_assignments[i]->pose[6] << std::endl;
                    
                }
                fout << current_right_assignments.size() << std::endl;
                for (unsigned i = 0; i < current_right_assignments.size(); i++)
                {
                    fout << current_right_assignments[i]->object_type << "," << current_right_assignments[i]->object_index << std::endl;
                    fout << current_right_assignments[i]->pose[0] << "," << current_right_assignments[i]->pose[1] << "," << current_right_assignments[i]->pose[2] << "," 
                            << current_right_assignments[i]->pose[3] << "," << current_right_assignments[i]->pose[4] << "," << current_right_assignments[i]->pose[5] << "," << current_right_assignments[i]->pose[6] << std::endl;
                    
                }
                
                fout.close();
            }
        }

    }
}

