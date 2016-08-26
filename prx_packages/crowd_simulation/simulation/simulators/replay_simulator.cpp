/**
 * @file replay_simulator.cpp
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


#include "simulation/simulators/replay_simulator.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/simulation/systems/obstacle.hpp"
#include "prx/simulation/system_graph.hpp"

#include "boost/filesystem.hpp"
#include <boost/tuple/tuple.hpp>
#include <boost/range/adaptor/map.hpp> // boost::tie

 #include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(prx::packages::crowd::replay_simulator_t, prx::sim::simulator_t)


namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {        
        namespace crowd
        {

            replay_simulator_t::replay_simulator_t() 
            { 
                collision_response = false;
                collision_detection = false;
            }

            replay_simulator_t::~replay_simulator_t() { }

            void replay_simulator_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
            {

                geom_map_t geom_map;
                std::string obstacle_name;
                std::string template_name;
                pedestrian_initializer = NULL;
                state_pos.resize(3);

                
                if(ros::param::has("prx/common_plant"))
                {
                    pedestrian_initializer = new parameter_reader_t("prx/common_plant", global_storage);
                }
                else
                {
                    PRX_FATAL_S("No global pedestrian information for the replay simulator!");
                }

                PRX_PRINT("got the pedestrian_initializer : " << pedestrian_initializer , PRX_TEXT_GREEN);


                simulation::simulation_step = parameters::get_attribute_as<double>("simulation_step", reader, template_reader);

                if( parameters::has_attribute("pathname", reader, template_reader) )
                {
                    pathname = parameters::get_attribute("pathname", reader, template_reader);
                    PRX_PRINT("Set pathname: " << pathname, PRX_TEXT_GREEN);
                }

                parameter_reader_t::reader_map_t obstacle_map;
                bool got_obstacle_map = false;
                if(ros::param::has("prx/obstacles"))
                {
                    parameter_reader_t* obstacle_reader = new parameter_reader_t("prx/obstacles", global_storage);
                    obstacle_map = obstacle_reader->get_map("");
                    got_obstacle_map = true;
                }
                else if (parameters::has_attribute("obstacles", reader, template_reader) )
                {
                    obstacle_map = parameters::get_map("obstacles", reader, template_reader);
                    got_obstacle_map = true;
                }

                if( got_obstacle_map)
                {
                    foreach(const parameter_reader_t::reader_map_t::value_type key_value, obstacle_map)
                    {
                        const parameter_reader_t* obstacle_template_reader = NULL;

                        if( key_value.second->has_attribute("template") )
                        {
                            template_name = key_value.second->get_attribute("template");
                            // TODO: Find a way to load templates under namespaces more cleanly.
                            obstacle_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + template_name, global_storage);

                        }

                        obstacle_name = pathname + "/obstacles/" + key_value.first;
                        add_obstacle(obstacle_name, create_subsystem(obstacle_name, key_value.second, obstacle_template_reader));
                        delete obstacle_template_reader;
                    }
                }



                // if( parameters::has_attribute("obstacles", reader, template_reader) )
                // {
                //     parameter_reader_t::reader_map_t obstacle_map = parameters::get_map("obstacles", reader, template_reader);

                //     foreach(const parameter_reader_t::reader_map_t::value_type key_value, obstacle_map)
                //     {
                //         const parameter_reader_t* obstacle_template_reader = NULL;

                //         if( key_value.second->has_attribute("template") )
                //         {
                //             template_name = key_value.second->get_attribute("template");
                //             // TODO: Find a way to load templates under namespaces more cleanly.
                //             obstacle_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + template_name, global_storage);

                //         }

                //         obstacle_name = pathname + "/obstacles/" + key_value.first;
                //         add_obstacle(obstacle_name, create_subsystem(obstacle_name, key_value.second, obstacle_template_reader));
                //         delete obstacle_template_reader;
                //     }
                // }

                std::string trajectories_folder_path;
                if(parameters::has_attribute("trajectories_folder_path",reader, template_reader))
                     trajectories_folder_path = parameters::get_attribute("trajectories_folder_path", reader, template_reader);
                 else
                    PRX_FATAL_S("No trajectories folder is specified!");

                int start_time = parameters::get_attribute_as<int>("start_time", reader, template_reader, 0);
                int end_time = parameters::get_attribute_as<int>("end_time", reader, template_reader, 2000);
                max_frame_id = end_time - start_time;

                boost::filesystem::path trajectories_folder(trajectories_folder_path);
                boost::filesystem::directory_iterator end_iter;

                std::vector<double> init_pos(3,0);
                init_pos[2] = -10;
                num_pedestrians = 0;

                if(boost::filesystem::exists(trajectories_folder) && boost::filesystem::is_directory(trajectories_folder))
                    for( boost::filesystem::directory_iterator iter(trajectories_folder) ; iter != end_iter ; ++iter)
                        if(boost::filesystem::is_regular_file(iter->path()))
                            num_pedestrians++;
                        
                subsystem_names.resize(num_pedestrians);
                pedestrian_spaces.resize(num_pedestrians);
                trajectories.resize(num_pedestrians);
                int pedestrian_index = 0;
                if(boost::filesystem::exists(trajectories_folder) && boost::filesystem::is_directory(trajectories_folder))
                {
                    for( boost::filesystem::directory_iterator iter(trajectories_folder) ; iter != end_iter ; ++iter)
                    {
                        if(boost::filesystem::is_regular_file(iter->path()))
                        {
                            std::string pedestrian_name =  "pedestrian_" + int_to_str(pedestrian_index);
                            subsystems[pedestrian_name] = create_subsystem(pathname + "/" + pedestrian_name, pedestrian_initializer, NULL);
                            subsystem_names[pedestrian_index] = pedestrian_name;
                            pedestrian_spaces[pedestrian_index] = subsystems[pedestrian_name]->get_state_space();
                            pedestrian_spaces[pedestrian_index]->set_from_vector(init_pos);
                            init_pos[0] += 1;

                            std::ifstream stream(iter->path().string().c_str());
                            int i = 0;
                            std::string line;
                            //consumes the lines till the starting frame
                            while(i < start_time && !stream.eof())
                            {
                                std::getline(stream, line);
                                if( line[0] == '#')
                                {
                                    std::getline(stream, line);
                                }
                                i++;
                            }

                            //stores the points till the ending frame or the end of the file. 
                            while(i < end_time && !stream.eof())
                            {
                                std::getline(stream, line);
                                if( line[0] == '#')
                                {
                                    std::getline(stream, line);
                                }
                                string_to_pos(line, state_pos);
                                trajectories[pedestrian_index].push_back(state_pos);
                                i++;
                            }
                            max_frame_id = PRX_MINIMUM(max_frame_id, i);                            
                            PRX_STATUS(pedestrian_name << ") " <<  pedestrian_index << " / " << num_pedestrians << "      trajectory size: " << trajectories[pedestrian_index].size(), PRX_TEXT_GREEN);
                            pedestrian_index++;
                        }
                    }
                }
                // num_pedestrians = file_streams.size();
                // PRX_PRINT("Got the trajectories file and now we have " << num_pedestrians << " pedestrians to load." , PRX_TEXT_CYAN);

                // state_pos[1] = 300;
                // state_pos[2] = 2;
                // subsystem_names.resize(num_pedestrians);
                // pedestrian_spaces.resize(num_pedestrians);
                // for(int i = 0; i < num_pedestrians; ++i)
                // {
                //     std::string pedestrian_name =  "pedestrian_" + int_to_str(i);
                //     PRX_PRINT("Trace : " << pedestrian_initializer->trace(), PRX_TEXT_CYAN);
                //     subsystems[pedestrian_name] = create_subsystem(pathname + "/" + pedestrian_name, pedestrian_initializer, NULL);
                //     subsystem_names[i] = pedestrian_name;
                //     pedestrian_spaces[i] = subsystems[pedestrian_name]->get_state_space();
                //     pedestrian_spaces[i]->set_from_vector(state_pos);
                //     PRX_PRINT("Initialized pedestrian : " << pedestrian_name, PRX_TEXT_GREEN);
                //     // subsystem_names.push_back(pedestrian_name);
                //     // pedestrian_spaces.push_back(subsystems[pedestrian_name]->get_state_space());
                //     // pedestrian_spaces.back()->set_from_vector(state_pos);
                //     state_pos[0] += 1;
                // }
                
                // soft_constraint_initializer->initialize_from_loader<soft_constraints_t>("","prx_utilities");

                construct_spaces();                
                simulator_state = state_space->alloc_point();
                PRX_DEBUG_COLOR("state_space (" << state_space->get_dimension() << "): " << state_space->get_space_name(), PRX_TEXT_GREEN);
                update_phys_geoms(geom_map);
                unsigned index = 0;
                update_phys_configs(config_list, index);

                system_graph_t::directed_vertex_t v = update_system_graph(sys_graph);
                sys_graph.system_graph[v].name = "simulator";
                sys_graph.get_path_plant_hash(plants);
                foreach(plant_t* plant, plants | boost::adaptors::map_values)
                {
                    PRX_DEBUG_COLOR("Got the plant : " << plant->get_pathname(), PRX_TEXT_CYAN);
                    //find the place in the structure where this plant's state space exists
                    std::pair<unsigned, unsigned> space_bounds = state_space->get_subspace(plant->get_state_space());
                    if( space_bounds.first != space_bounds.second )
                    {
                        system_intervals[plant] = space_bounds;
                    }
                }
                
                stats_clock.reset();
                frame_id = 0;
                PRX_STATUS("Done with initializing the pedestrians  starting frame : " << start_time << " ending frame : " << end_time << "   frames to simulate:"  << max_frame_id, PRX_TEXT_CYAN);
            }

            void replay_simulator_t::propagate_and_respond()
            {
                // PRX_PRINT("==== Frame: " << frame_id << " ====" , PRX_TEXT_BLUE);
                std::string line, num;
                if(frame_id < max_frame_id)
                {
                    for(unsigned i = 0; i < num_pedestrians; ++i)
                    {
                        pedestrian_spaces[i]->set_from_vector(trajectories[i][frame_id]);
                        // std::getline(*file_streams[i], line);                    
                        // if( line[0] == '#')
                        // {
                        //     std::getline(*file_streams[i], line);                        
                        // }
                        // string_to_pos(line, state_pos);
                        // pedestrian_spaces[i]->set_from_vector(state_pos);
                    }
                }
                frame_id++;
            }

            void replay_simulator_t::add_obstacle(const std::string& name, system_ptr_t obstacle)
            {
                geom_map_t geom_map;
                config_list_t config_map;
                unsigned index = 0;
                if( obstacles[name] == NULL )
                {
                    obstacles[name] = obstacle;
                    obstacles[name]->update_phys_geoms(geom_map);
                    obstacles[name]->update_phys_configs(config_map, index);
                }
            }

            void replay_simulator_t::string_to_pos(std::string str, std::vector<double>& pos)
            {   
                std::string num;
                boost::tie(num, str) = split_path(str,',');
                pos[0] = atof(num.c_str());
                boost::tie(num, str) = split_path(str,',');
                pos[1] = atof(num.c_str());
                pos[2] = atof(str.c_str());
            }
        }
    }
}
