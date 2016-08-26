/**
 * @file manipulation_world_model.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Rahul Shome, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "planning/manipulation_world_model.hpp"
#include "simulation/simulators/manipulation_simulator.hpp"
#include "simulation/systems/plants/manipulator.hpp"
#include "simulation/systems/plants/movable_body_plant.hpp"
#include "prx/simulation/systems/obstacle.hpp"
#include "simulation/collision_checking/collidable_movable_bodies_list.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/range/adaptor/map.hpp> //adaptors

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::manipulation_world_model_t, prx::plan::world_model_t)

namespace prx
{
    namespace packages
    {
        using namespace util;
        using namespace sim;
        using namespace plan;    

        namespace manipulation
        {
            manipulation_world_model_t::manipulation_world_model_t()
            {
                collision_list = new collidable_movable_bodies_list_t();
            }

            void manipulation_world_model_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {

                world_model_t::init(reader,template_reader);

                active_manipulation_info = NULL;
                std::vector<manipulator_t*> manipulators;
                manip_simulator->get_manipulators(manipulators);
                ik_seeds = parameters::get_attribute_as<int>("num_ik_seeds",reader,template_reader,100);

                if(reader->has_attribute("IK_databases"))
                {
                    parameter_reader_t::reader_map_t ik_map = parameters::get_map("IK_databases",reader,NULL);
                    
                    PRX_PRINT("Going to build the IK databases with a map that has: " << ik_map.size() << " elements!", PRX_TEXT_CYAN);

                    foreach(const parameter_reader_t::reader_map_t::value_type key_value, ik_map)
                    {
                        IK_seed_database_t* database = new IK_seed_database_t();
                        database->init(key_value.second);
                        std::string manipulator_name = key_value.second->get_attribute("manipulator");
                        manipulator_t* manip;
                        foreach(manipulator_t* manipulator,manipulators)
                        {
                            if(reverse_split_path(manipulator->get_pathname()).second==manipulator_name)
                            {
                                manip = manipulator;
                            }
                        }

                        if(key_value.second->has_attribute("deserialize_file"))
                        {
                            char* w = std::getenv("PRACSYS_PATH");
                            std::string dir(w);
                            dir += ("/prx_roadmaps/");
                            std::string file = dir + key_value.second->get_attribute("deserialize_file");
                            std::ifstream fin;
                            fin.open(file.c_str());
                            database->deserialize(fin,manip->get_state_space());
                            fin.close();
                        }
                        else
                        {


                            state_t* full_wm_state = get_full_state_space()->alloc_point();
                            int iterations = key_value.second->get_attribute_as<int>("samples");
                            std::vector<std::string> names = manip->get_end_effector_names();
                            const space_t* manip_space = manip->get_state_space();
                            state_t* manip_state = manip_space->alloc_point();
                            config_t left_config;
                            config_t right_config;
                            std::string left_name;
                            std::string right_name;
                            if(key_value.second->has_attribute("end_effectors"))
                            {
                                names = key_value.second->get_attribute_as<std::vector<std::string> >("end_effectors"); 
                                left_name = names[0];
                                right_name = names[1];
                            }
                            else
                            {
                                left_name = reverse_split_path(names[0]).second; 
                                right_name = reverse_split_path(names[1]).second;
                            }
                            for (int i = 0; i < iterations; ++i)
                            {
                                PRX_STATUS_S("Building IK Database "<<i+1<<"/"<<iterations);
                                manip_space->uniform_sample(manip_state);
                                manip_space->copy_from_point(manip_state);
                                manip->FK_solver(left_config,left_name);
                                // double x,y,z;
                                // left_config.get_position(x,y,z);
                                // if(.7<x && x <  .9 && -.1 < y && y < .1 && .75 < z && z < .95)
                                // {
                                manip->FK_solver(right_config,right_name);
                                database->add_pair(manip_space, left_config, right_config, manip_state );
                                // }
                                // else
                                //     i--;
                            }
                            if(key_value.second->has_attribute("serialize_file"))
                            {
                                char* w = std::getenv("PRACSYS_PATH");
                                std::string dir(w);
                                dir += ("/prx_roadmaps/");
                                std::string file = dir + key_value.second->get_attribute("serialize_file");
                                std::ofstream fout;
                                fout.open(file.c_str());
                                database->serialize(fout, manip->get_state_space());
                                fout.close();

                            }
                            get_full_state_space()->copy_from_point(full_wm_state);
                            get_full_state_space()->free_point(full_wm_state);
                        }
                        ik_databases.push_back(std::make_pair(key_value.first,database));
                    }
                }

                if( parameters::has_attribute("planning_contexts",reader,template_reader))
                {
                    parameter_reader_t::reader_map_t contexts_map = parameters::get_map("planning_contexts",reader,template_reader);
                    foreach(const parameter_reader_t::reader_map_t::value_type key_value, contexts_map)
                    {                    
                        if(key_value.second->has_attribute("manipulation_context_info"))
                        {
                            world_model_t::use_context(key_value.first);
                            manipulator_t* manipulator = find_active_manipulator(manipulators);
                            if(manipulator == NULL)
                                PRX_FATAL_S("The manipulation context info " << key_value.first << " has been initialized without active manipulator!");

                            std::string full_arm_context_name = "";
                            std::string arm_context_name = "";
                            std::string end_effector_context_name = "";
                            std::string start_link = "";
                            std::string end_link = "";
                            std::string ik_database = "";
                            bool left_arm_ik = true;

                            if(parameters::has_attribute("manipulation_context_info/full_arm_context_name", key_value.second, NULL))
                                full_arm_context_name = parameters::get_attribute("manipulation_context_info/full_arm_context_name",key_value.second,NULL);
                            else
                                PRX_FATAL_S("The manipulation context info " << key_value.first << " has been initialized without full_arm_context_name");

                            if(parameters::has_attribute("manipulation_context_info/arm_context_name", key_value.second, NULL))
                                arm_context_name = parameters::get_attribute("manipulation_context_info/arm_context_name",key_value.second,NULL);
                            else
                                PRX_FATAL_S("The manipulation context info " << key_value.first << " has been initialized without arm_context_name");
                            
                            if(parameters::has_attribute("manipulation_context_info/end_effector_context_name", key_value.second, NULL))
                                end_effector_context_name = parameters::get_attribute("manipulation_context_info/end_effector_context_name",key_value.second,NULL);
                            else
                                PRX_FATAL_S("The manipulation context info " << key_value.first << " has been initialized without end_effector_context_name");
                            
                            if(parameters::has_attribute("manipulation_context_info/start_link", key_value.second, NULL))
                                start_link = parameters::get_attribute("manipulation_context_info/start_link",key_value.second,NULL);
                            else
                                PRX_FATAL_S("The manipulation context info " << key_value.first << " has been initialized without start_link");
                            
                            if(parameters::has_attribute("manipulation_context_info/end_link", key_value.second, NULL))
                                end_link = parameters::get_attribute("manipulation_context_info/end_link",key_value.second,NULL);
                            else
                                PRX_FATAL_S("The manipulation context info " << key_value.first << " has been initialized without end_link");
                            
                            bool adaptive_grasp = false;
                            int adaptive_grasping_mode = 0;
                            double adaptive_grasping_step = 0;
                            if(parameters::has_attribute("manipulation_context_info/grasp_till_collision", key_value.second, NULL))
                            {
                                adaptive_grasp = true;
                                if(parameters::has_attribute("manipulation_context_info/grasp_till_collision/grasping_mode", key_value.second, NULL))
                                    adaptive_grasping_mode = parameters::get_attribute_as<int>("manipulation_context_info/grasp_till_collision/grasping_mode",key_value.second,NULL);
                                else
                                    PRX_FATAL_S("The manipulation context info " << key_value.first << " has been initialized without a grasping mode for adaptive grasping.");

                                if(parameters::has_attribute("manipulation_context_info/grasp_till_collision/grasping_step", key_value.second, NULL))
                                    adaptive_grasping_step = parameters::get_attribute_as<double>("manipulation_context_info/grasp_till_collision/grasping_step",key_value.second,NULL);
                                else
                                    adaptive_grasping_step = -0.2;
                            }

                             
                            contexts[key_value.first].info = new manipulation_context_info_t(this,manipulator,full_arm_context_name,arm_context_name,end_effector_context_name,start_link,end_link,adaptive_grasp,adaptive_grasping_mode,adaptive_grasping_step);
                            PRX_DEBUG_COLOR("Constructed manipulation context info for planning context: " << key_value.first, PRX_TEXT_MAGENTA);

                            if(key_value.second->has_attribute("manipulation_context_info/ik_database"))
                            {
                                manipulation_context_info_t* active_info = (manipulation_context_info_t*)contexts[key_value.first].info;
                                ik_database = key_value.second->get_attribute("manipulation_context_info/ik_database");
                                left_arm_ik = key_value.second->get_attribute_as<bool>("manipulation_context_info/left_arm_ik",true);
                                active_info->ik_database = NULL;
                                for(int i=0;i<ik_databases.size();i++)
                                {
                                    if(ik_databases[i].first==ik_database)
                                    {
                                        active_info->ik_database = ik_databases[i].second;
                                    }
                                }
                                active_info->ik_left_arm = left_arm_ik;
                            }
                            else
                            {
                                manipulation_context_info_t* active_info = (manipulation_context_info_t*)contexts[key_value.first].info;
                                active_info->ik_database = NULL;
                            }

                        }
                    }
                }
            }


            void manipulation_world_model_t::init_collision_list(const parameter_reader_t * const reader)
            {
                manip_simulator = dynamic_cast<manipulation_simulator_t*>(simulator);
                if(manip_simulator == NULL)
                {
                    foreach(system_ptr_t ptr, systems | boost::adaptors::map_values)
                    {
                        if(manip_simulator==NULL)
                            manip_simulator = dynamic_cast<manipulation_simulator_t*>(ptr.get());
                    }
                    if(manip_simulator == NULL)
                    {
                        PRX_FATAL_S("Manipulation_world_model works only with manipulation_simulator_t!");
                    }
                }

                collidable_list = dynamic_cast<collidable_movable_bodies_list_t*>(collision_list);
                if ( collidable_list == NULL)
                {
                    PRX_FATAL_S ("NOPE");
                }

                manip_simulator->get_movable_objects(collidable_objects);
                collidable_list->setup_collision_list(collidable_objects);

                std::string plant_name;

                vector_collision_list_t* black_list = new vector_collision_list_t();
                hash_t<std::string, system_ptr_t> obstacles = simulator->get_obstacles();

                foreach(std::string obst, obstacles | boost::adaptors::map_keys)
                PRX_DEBUG_S("Obstacle : " << obst);

                hash_t<std::string, plant_t*> plants;
                system_graph_t sys_graph;
                simulator->update_system_graph(sys_graph);
                sys_graph.get_path_plant_hash(plants);

                foreach(std::string name1, plants | boost::adaptors::map_keys)
                PRX_DEBUG_S("Plant: " << name1);

                int index = 0;

                if( reader->has_attribute("black_list") )
                {

                    foreach(const parameter_reader_t* list_reader, reader->get_list("black_list"))
                    {
                        index++;
                        std::vector<const parameter_reader_t*> r = list_reader->get_list("");

                        if( r.size() != 2 )
                            PRX_FATAL_S("The pair "<<index<<" in black list is wrong. Has to be a system with a list of systems.");

                        plant_name = r[0]->get_attribute("");

                        foreach(const parameter_reader_t* systems_reader, r[1]->get_list(""))
                        black_list->add_pair(plant_name, systems_reader->get_attribute(""));
                    }
                }

                if( reader->has_attribute("white_list") )
                {
                    index = 0;

                    foreach(const parameter_reader_t* list_reader, reader->get_list("white_list"))
                    {
                        index++;
                        std::vector<const parameter_reader_t*> r = list_reader->get_list("");

                        if( r.size() != 2 )
                            PRX_FATAL_S("The pair "<<index<<" in white list is wrong. Has to be a system with a list of systems.");

                        plant_name = r[0]->get_attribute("");

                        foreach(const parameter_reader_t* systems_reader, r[1]->get_list(""))
                        {
                            std::string name2 = systems_reader->get_attribute("");

                            if( plants[name2] != NULL )
                            {
                                // Check if the outer plant is a movable body
                                if (dynamic_cast<movable_body_plant_t*>(plants[name2]) != NULL)
                                {
                                    // Add to the movable body specific list
                                    collision_list = collidable_list->get_target_object_collision_list(name2);
                                }
                                else
                                {
                                    // Add to the object independent list
                                    collision_list = collidable_list->get_object_independent_collision_list();
                                }
                                add_system_system_in_collision_list(plant_name, name2, black_list, plants);
                            }
                            else if( obstacles[name2] != NULL )
                            {
                                collision_list = collidable_list->get_object_independent_collision_list();
                                add_system_obstacle_in_collision_list(plant_name, name2, black_list, plants, obstacles);
                            }
                            else
                                PRX_WARN_S("The system/obstacle " << name2 << " does not exist in the simulation to pair with "<<plant_name<<".");
                        }
                    }
                }
                else
                {
                    PRX_DEBUG_COLOR("Collision list initialized with all plants", PRX_TEXT_GREEN);
                    collision_list_all_vs_all(plants, obstacles, black_list);
                }
                collision_list = collidable_list->get_object_independent_collision_list();
                simulator->link_collision_list(collision_list);
                delete black_list;
            }

            // void manipulation_world_model_t::add_system_system_in_collision_list(const std::string& system1, const std::string& system2, const vector_collision_list_t* black_list, hash_t<std::string, plant_t*>& plants)
            // {
            //     if( system1 != system2 && !black_list->pair_in_list(system1, system2) )
            //     {
            //         std::vector<std::string>* geom_names1 = plants[system1]->get_geometries_names();
            //         std::vector<std::string>* geom_names2 = plants[system2]->get_geometries_names();

            //         foreach(std::string n1, *geom_names1)
            //         {

            //             foreach(std::string n2, *geom_names2)
            //             {
            //                 collision_list->add_pair(n1, n2);
            //             }
            //         }

            //         plants[system1]->update_collision_list(collision_list);
            //         plants[system2]->update_collision_list(collision_list);
            //     }
            // }

            // void manipulation_world_model_t::add_system_obstacle_in_collision_list(const std::string& system, const std::string& obstacle, const vector_collision_list_t* black_list, hash_t<std::string, plant_t*>& plants, hash_t<std::string, system_ptr_t>& obstacles)
            // {
            //     if( !black_list->pair_in_list(system, obstacle) )
            //     {
            //         std::vector<std::string>* geom_names1 = plants[system]->get_geometries_names();
            //         std::vector<std::string>* geom_names2 = static_cast<obstacle_t*>(obstacles[obstacle].get())->get_geometries_names();

            //         foreach(std::string n1, *geom_names1)
            //         {
            //             foreach(std::string n2, *geom_names2)
            //             {
            //                 collision_list->add_pair(n1, n2);
            //             }
            //         }

            //         plants[system]->update_collision_list(collision_list);
            //     }
            // }

            void manipulation_world_model_t::collision_list_all_vs_all(hash_t<std::string, plant_t*>& plants, hash_t<std::string, system_ptr_t>& obstacles, const vector_collision_list_t* black_list)
            {
                PRX_DEBUG_COLOR("COLLISION LIST ALL VS ALL --------------\n\n\n", PRX_TEXT_CYAN);
                if(plants.size()==1)
                {
                PRX_DEBUG_COLOR("ONLY 1 PLANT?! --------------\n\n\n", PRX_TEXT_CYAN);
                    plants[plants.begin()->first]->update_collision_list(collision_list);
                }
                for( hash_t<std::string, plant_t*>::const_iterator it = plants.begin(); it != plants.end(); it++ )
                {
                    hash_t<std::string, plant_t*>::const_iterator temp_it = it;

                    // Check if the outer plant is a movable body
                    if (dynamic_cast<movable_body_plant_t*>(it->second) != NULL)
                    {
                        PRX_DEBUG_COLOR("MOVABLE BODY: " << it->first << " --------------\n\n\n", PRX_TEXT_CYAN);
                        // Add to the movable body specific list
                        collision_list = collidable_list->get_target_object_collision_list(it->first);
                        temp_it = plants.begin();
                    }
                    else
                    {
                        PRX_DEBUG_COLOR("NON-MOVABLE BODY: " << it->first << " --------------\n\n\n", PRX_TEXT_CYAN);
                        // Add to the object independent list
                        collision_list = collidable_list->get_object_independent_collision_list();
                        temp_it = it;
                        temp_it++;
                    }

                    for( hash_t<std::string, plant_t*>::const_iterator it2 = temp_it; it2 != plants.end(); it2++ )
                    {
                        add_system_system_in_collision_list(it->first, it2->first, black_list, plants);
                    }

                    foreach(std::string obst, obstacles | boost::adaptors::map_keys)
                    {
                        add_system_obstacle_in_collision_list(it->first, obst, black_list, plants, obstacles);
                    }
                }

                // TODO: Remove redundant collision pairs
            }

            void manipulation_world_model_t::update_target_objects(const std::vector<std::string>& target_object_list)
            {
                PRX_DEBUG_COLOR("Updating target object list!", PRX_TEXT_CYAN);

                collidable_list->update_target_objects(target_object_list);
                collision_list = collidable_list;

                simulator->link_collision_list(collision_list);

            }


            void manipulation_world_model_t::push_state(const sim::state_t * const source)
            {
                world_model_t::push_state(source);
            }

            void manipulation_world_model_t::use_context(std::string name)
            {
                if( active_manipulation_info == NULL || context_name != name )
                {
                    active_manipulation_info = dynamic_cast<manipulation_context_info_t*>(contexts[name].info);                    
                    // PRX_PRINT("Moving to context [" << name << "]   which has info: [" <<  contexts[name].info << "]  resulting in active info [" << active_manipulation_info << "]" ,PRX_TEXT_CYAN);
                }
                world_model_t::use_context(name);
            }

            void manipulation_world_model_t::get_objects(std::vector<movable_body_plant_t* >& objects)
            {
                manip_simulator->get_movable_objects(objects);
            }

            void manipulation_world_model_t::set_static_relative_config( bool flag)
            {
                PRX_DEBUG_COLOR("Manip model: set static relative config", PRX_TEXT_CYAN);
                //If the end effector is grasping an object and we want to maintain the relative configuration static
                if(flag && manip_simulator->is_end_effector_grasping(active_manipulation_info->manipulator, active_manipulation_info->chain.second))
                {
                    manip_simulator->set_static_relative_config(active_manipulation_info->end_effector_pathname, flag);
                }
                //If we want to stop using the relative configuration.
                else if (!flag)
                {
                    manip_simulator->set_static_relative_config(active_manipulation_info->end_effector_pathname, flag);
                }
                // else
                //     PRX_WARN_S("You tried to set static relative configuration when the end effector " << active_manipulation_info->chain.second << " does not grasp anything.");
            }
            
            void manipulation_world_model_t::engage_grasp( plan_t& plan, double grasping_mode, bool static_relative_flag)
            {
                PRX_ASSERT(selected_control_space == active_manipulation_info->full_arm_control_space);
                //PRX_ASSERT(!manip_simulator->is_end_effector_grasping(active_manipulation_info->manipulator, active_manipulation_info->chain.second));

                //If the grasping mode requires an adaptive grasp
                if(active_manipulation_info->adaptive_grasp && active_manipulation_info->adaptive_grasping_mode == grasping_mode)
                {
                    //Set the end effector to the grasping mode
                    active_manipulation_info->end_effector_control->memory[0] = grasping_mode;
                    double g_mode = grasping_mode;
                    double prev_g_mode = grasping_mode;
                    //The maximum allowed control space change is < 1
                    int max_steps = fabs((int)((double)(1/active_manipulation_info->adaptive_grasping_step)));

                    //Keep changing the end effector control by the step
                    do
                    {
                        prev_g_mode = active_manipulation_info->end_effector_control->memory[0];
                        active_manipulation_info->end_effector_control->memory[0] = g_mode;
                        active_manipulation_info->end_effector_control_space->copy_from_point(active_manipulation_info->end_effector_control);
                        active_manipulation_info->arm_control_space->zero(active_manipulation_info->arm_control);
                        active_manipulation_info->arm_control_space->copy_from_point(active_manipulation_info->arm_control);
                        active_manipulation_info->full_arm_control_space->copy_to_point(active_manipulation_info->full_arm_control);
                        active_manipulation_info->full_arm_state_space->copy_to_point(active_manipulation_info->full_arm_state);
                        //Update the state by propagating the end effector control
                        propagate_once(active_manipulation_info->full_arm_state, active_manipulation_info->full_arm_control, simulation::simulation_step, active_manipulation_info->full_arm_state);

                        // PRX_PRINT("Control: "<<active_manipulation_info->full_arm_control_space->print_point(active_manipulation_info->full_arm_control,3), PRX_TEXT_GREEN);
                        // PRX_PRINT("State: "<<active_manipulation_info->full_arm_state_space->print_point(active_manipulation_info->full_arm_state,3), PRX_TEXT_GREEN);
                        // PRX_PRINT("\nAdaptive grasp: "<<prev_g_mode<<","<<g_mode<<" : "<<max_steps, PRX_TEXT_BROWN);

                        //Change the control by the step
                        g_mode += active_manipulation_info->adaptive_grasping_step;
                    //Keep changing the control till the steps run out or the arm state with the changed control for the end effector is in collision
                    }while(--max_steps>1 && valid_state(active_manipulation_info->full_arm_state));
                    //TODO: Is there a way to avoid possibly expensive valid_state checks?

                    //Do the standard engage grasp for the last grasping control that was collision free
                    PRX_PRINT("Engaging grasp with adaptive grasp control: "<<prev_g_mode, PRX_TEXT_BROWN);
                    active_manipulation_info->end_effector_control->memory[0] = prev_g_mode;
                    active_manipulation_info->end_effector_control_space->copy_from_point(active_manipulation_info->end_effector_control);
                    active_manipulation_info->arm_control_space->zero(active_manipulation_info->arm_control);
                    active_manipulation_info->arm_control_space->copy_from_point(active_manipulation_info->arm_control);
                    active_manipulation_info->full_arm_control_space->copy_to_point(active_manipulation_info->full_arm_control);
                    active_manipulation_info->full_arm_state_space->copy_to_point(active_manipulation_info->full_arm_state);

                    propagate_once(active_manipulation_info->full_arm_state, active_manipulation_info->full_arm_control, simulation::simulation_step, active_manipulation_info->full_arm_state);
                    plan.copy_onto_back(active_manipulation_info->full_arm_control, simulation::simulation_step);
                    set_static_relative_config(static_relative_flag);
                }
                else
                {
                    //If the manipulation info does not necessitate an adaptive grasp for the current grasping mode
                    active_manipulation_info->end_effector_control->memory[0] = grasping_mode;
                    active_manipulation_info->end_effector_control_space->copy_from_point(active_manipulation_info->end_effector_control);
                    active_manipulation_info->arm_control_space->zero(active_manipulation_info->arm_control);
                    active_manipulation_info->arm_control_space->copy_from_point(active_manipulation_info->arm_control);
                    active_manipulation_info->full_arm_control_space->copy_to_point(active_manipulation_info->full_arm_control);
                    active_manipulation_info->full_arm_state_space->copy_to_point(active_manipulation_info->full_arm_state);                
                    propagate_once(active_manipulation_info->full_arm_state, active_manipulation_info->full_arm_control, simulation::simulation_step, active_manipulation_info->full_arm_state);
                    plan.copy_onto_back(active_manipulation_info->full_arm_control, simulation::simulation_step);
                    set_static_relative_config(static_relative_flag);
                }
            }

            double manipulation_world_model_t::get_current_grasping_mode()
            {
                active_manipulation_info->end_effector_state_space->copy_to_point(active_manipulation_info->end_effector_state);
                return active_manipulation_info->end_effector_state->at(0);
            }

            manipulation_context_info_t* manipulation_world_model_t::get_current_manipulation_info()
            {
                if(active_manipulation_info == NULL)
                {
                    PRX_PRINT("The current planning context does not have a manipulation info.", PRX_TEXT_RED);
                }
                return active_manipulation_info;
            }

            void manipulation_world_model_t::steering_function(plan_t& result_plan, const state_t* start, const state_t* goal)
            {
                selected_state_space->copy_from_point(goal);
                active_manipulation_info->full_manipulator_state_space->copy_to_point(active_manipulation_info->full_target_manipulator_state);

                selected_state_space->copy_from_point(start);
                active_manipulation_info->full_manipulator_state_space->copy_to_point(active_manipulation_info->full_manipulator_state);

                active_manipulation_info->manipulator_plan.clear();
                active_manipulation_info->manipulator->steering_function(active_manipulation_info->full_manipulator_state, active_manipulation_info->full_target_manipulator_state, active_manipulation_info->manipulator_plan);
                convert_plan(result_plan, selected_control_space, active_manipulation_info->manipulator_plan, active_manipulation_info->full_manipulator_control_space);                
            }

            void manipulation_world_model_t::steering_function(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& result)
            {
                // if(!IK_steer_paths)
                // {
                    world_model_t::steering_function(start,goal,result);
                // }
                // else
                // {
                //     std::string old_context = get_current_context();
                //     space_t* s_space = selected_state_space;
                //     space_t* c_space = selected_control_space;
                //     use_context(IK_steer_context);
                //     convert_spaces(active_manipulation_info->full_manipulator_state_space, active_manipulation_info->full_target_manipulator_state, s_space, goal);
                //     convert_spaces(active_manipulation_info->full_manipulator_state_space, active_manipulation_info->full_manipulator_state, s_space, start);
                //     active_manipulation_info->manipulator_plan.clear();
                //     config_t goal_config;
                //     active_manipulation_info->full_manipulator_state_space->copy_from_point(active_manipulation_info->full_manipulator_state);
                //     active_manipulation_info->manipulator->FK_solver(goal_config, active_manipulation_info->chain.second);

                //     if(active_manipulation_info->manipulator->jac_steering(active_manipulation_info->manipulator_plan, active_manipulation_info->full_manipulator_state, active_manipulation_info->full_manipulator_state, goal_config, active_manipulation_info->chain.first, active_manipulation_info->chain.second))
                //     {
                //         convert_plan(result, c_space, active_manipulation_info->manipulator_plan, active_manipulation_info->full_manipulator_control_space);
                //     }
                //     else
                //     {
                //         world_model_t::steering_function(start,goal,result);
                //     }

                //     use_context(old_context);
                // }
            }


            bool manipulation_world_model_t::IK( sim::state_t* result_state, const util::space_point_t* start_state, const util::config_t& goal_config, bool validate)
            {
                selected_state_space->copy_from_point(start_state);
                active_manipulation_info->full_manipulator_state_space->copy_to_point(active_manipulation_info->full_manipulator_state);
                if(active_manipulation_info->ik_database!=NULL)
                {
                    state_t* temp_full_point = active_manipulation_info->full_manipulator_state_space->alloc_point();
                    active_manipulation_info->end_effector_state_space->copy_to_point(active_manipulation_info->end_effector_state);
                    std::vector< state_t* > states;
                    active_manipulation_info->ik_database->get_near_neighbors(  states, goal_config, ik_seeds, active_manipulation_info->ik_left_arm );
                    for (int i = 0; i < states.size(); ++i)
                    {
                        active_manipulation_info->full_manipulator_state_space->copy_from_point(states[i]);
                        // PRX_INFO_S("IK DATABASE CONFIG: "<<goal_config);
                        // PRX_INFO_S("IK DATABASE STATE: "<<active_manipulation_info->full_manipulator_state_space->print_point(states[i]));
                        active_manipulation_info->end_effector_state_space->copy_from_point(active_manipulation_info->end_effector_state);
                        active_manipulation_info->full_manipulator_state_space->copy_to_point(states[i]);
                        if(active_manipulation_info->manipulator->IK_solver(temp_full_point, states[i], goal_config, active_manipulation_info->chain.first, active_manipulation_info->chain.second))
                        {
                            active_manipulation_info->full_manipulator_state_space->copy_from_point(temp_full_point);
                            selected_state_space->copy_to_point(result_state);
                            active_manipulation_info->full_manipulator_state_space->copy_from_point(active_manipulation_info->full_manipulator_state);
                            selected_state_space->copy_from_point(result_state);
                            if(!validate || !simulator->in_collision())
                            {
                                active_manipulation_info->full_manipulator_state_space->free_point(temp_full_point);
                                return true;
                            }
                            // else
                            // {
                            //     PRX_PRINT("Manip WM::IK:: INVALID - IK solver invalid (collision)", PRX_TEXT_BLUE);
                            // }
                        }
                        // else
                        // {
                        //     PRX_PRINT("Manip WM::IK:: INVALID - IK solver failure (no solution)", PRX_TEXT_MAGENTA);
                        // }
                    }
                    active_manipulation_info->full_manipulator_state_space->free_point(temp_full_point);
                    active_manipulation_info->full_manipulator_state_space->copy_from_point(active_manipulation_info->full_manipulator_state);
                }
                else
                {
                    state_t* temp_full_point = active_manipulation_info->full_manipulator_state_space->alloc_point();
                    state_t* temp_arm_point = active_manipulation_info->arm_state_space->alloc_point();
                    active_manipulation_info->end_effector_state_space->copy_to_point(active_manipulation_info->end_effector_state);
                    int seeds = ik_seeds;
                    if(ik_seeds==0)
                    {
                        seeds = 1;
                    }
                    for (int i = 0; i < seeds; ++i)
                    {   
                        active_manipulation_info->arm_state_space->uniform_sample(temp_arm_point);
                        active_manipulation_info->arm_state_space->copy_from_point(temp_arm_point);
                        active_manipulation_info->end_effector_state_space->copy_from_point(active_manipulation_info->end_effector_state);
                        active_manipulation_info->full_manipulator_state_space->copy_to_point(temp_full_point);
                        if(active_manipulation_info->manipulator->IK_solver(temp_full_point, temp_full_point, goal_config, active_manipulation_info->chain.first, active_manipulation_info->chain.second))
                        {
                            active_manipulation_info->full_manipulator_state_space->copy_from_point(temp_full_point);
                            selected_state_space->copy_to_point(result_state);
                            active_manipulation_info->full_manipulator_state_space->copy_from_point(active_manipulation_info->full_manipulator_state);
                            selected_state_space->copy_from_point(result_state);
                            if(!validate || !simulator->in_collision())
                            {
                                //TODO: Is there a reason we're constantly copying to and from this point?  This seems a bit... redundant?
                                selected_state_space->copy_to_point(result_state);
                                active_manipulation_info->full_manipulator_state_space->free_point(temp_full_point);
                                return true;
                            }
                            // else
                            // {
                            //     PRX_PRINT("Manip WM::IK:: INVALID - IK solver invalid (collision) " << selected_state_space->print_point( result_state, 5 ), PRX_TEXT_BLUE);
                            // }
                        }
                        // else
                        // {
                        //     PRX_PRINT("Manip WM::IK:: INVALID - IK solver failure (no solution)", PRX_TEXT_MAGENTA);
                        // }
                    }
                    active_manipulation_info->full_manipulator_state_space->free_point(temp_full_point);
                    active_manipulation_info->arm_state_space->free_point(temp_arm_point);
                    active_manipulation_info->full_manipulator_state_space->copy_from_point(active_manipulation_info->full_manipulator_state);
                }

                return false;
            }

            bool manipulation_world_model_t::jac_steering( plan_t& result_plan, workspace_trajectory_t& ee_trajectory, sim::state_t* result_state, const util::space_point_t* start_state, const util::config_t& goal_config)
            {
                // PRX_DEBUG_COLOR("Current context in jac steering: " << get_current_context(), PRX_TEXT_BROWN);
                // PRX_DEBUG_COLOR("Selected state dimension: " << selected_state_space->get_dimension() << " name: " << selected_state_space->get_space_name(), PRX_TEXT_BROWN);
                // PRX_DEBUG_COLOR("Selected control dimension: " << selected_control_space->get_dimension() << " name: " << selected_control_space->get_space_name(), PRX_TEXT_BROWN);
                // PRX_DEBUG_COLOR("Chain.first : " << active_manipulation_info->chain.first << ", chain.second: " << active_manipulation_info->chain.second, PRX_TEXT_RED);

                selected_state_space->copy_from_point(start_state);
                active_manipulation_info->full_manipulator_state_space->copy_to_point(active_manipulation_info->full_manipulator_state);

                active_manipulation_info->manipulator_plan.clear();
                if(active_manipulation_info->manipulator->jac_steering(active_manipulation_info->manipulator_plan, ee_trajectory, active_manipulation_info->full_target_manipulator_state, active_manipulation_info->full_manipulator_state, goal_config, active_manipulation_info->chain.first, active_manipulation_info->chain.second))
                {
                    convert_plan(result_plan, selected_control_space, active_manipulation_info->manipulator_plan, active_manipulation_info->full_manipulator_control_space);
                    active_manipulation_info->full_manipulator_state_space->copy_from_point(active_manipulation_info->full_target_manipulator_state);
                    selected_state_space->copy_to_point(result_state);
                    return true;
                }
                return false;
            }

            bool manipulation_world_model_t::jac_steering( plan_t& result_plan, workspace_trajectory_t& ee_trajectory, sim::state_t* result_state, const util::space_point_t* start_state, const util::space_point_t* final_state, const util::config_t& goal_config)
            {
                if(jac_steering(result_plan,ee_trajectory,result_state,start_state,goal_config))
                {
                    steering_function(result_plan,result_state,final_state);
                    selected_state_space->copy_point(result_state, final_state);
                    return true;
                }
                // selected_state_space->copy_from_point(start_state);
                // active_manipulation_info->state_space->copy_to_point(active_manipulation_info->full_manipulator_state);
                // active_manipulation_info->manipulator_plan.clear();
                // if(active_manipulation_info->manipulator->IK_steering(active_manipulation_info->manipulator_plan, active_manipulation_info->full_manipulator_state, active_manipulation_info->full_manipulator_state, goal_config, active_manipulation_info->chain.first, active_manipulation_info->chain.second))
                // {
                //     steering_function(result_state, final_state, result_plan);
                //     convert_plan(result_plan, selected_control_space, active_manipulation_info->manipulator_plan, active_manipulation_info->control_space);
                //     active_manipulation_info->state_space->copy_from_point(active_manipulation_info->full_manipulator_state);
                //     selected_state_space->copy_to_point(result_state);
                //     return true;
                // }
                return false;
            }

            unsigned manipulation_world_model_t::get_jac_steer_error_type()
            {
                return active_manipulation_info->manipulator->get_jac_steer_error_type();
            }
            
            void manipulation_world_model_t::FK(util::config_t& link)
            {
                active_manipulation_info->manipulator->FK_solver(link, active_manipulation_info->chain.second);
            }
            
            std::pair<std::string, std::string> manipulation_world_model_t::get_chain()
            {
                return active_manipulation_info->chain;
            }
            
            std::string manipulation_world_model_t::get_end_effector_context()
            {
                return active_manipulation_info->end_effector_context_name;
            }

            void manipulation_world_model_t::get_manipulators(std::vector<manipulator_t* >& manipulators)
            {
                manip_simulator->get_manipulators(manipulators);
            }

            manipulator_t* manipulation_world_model_t::find_active_manipulator(const std::vector<manipulator_t*>& manipulators)
            {
                foreach(manipulator_t* manip, manipulators)
                {
                    if(manip->is_active())
                        return manip;
                }
                return NULL;
            }

            void manipulation_world_model_t::get_end_effector_local_config(util::config_t& config)
            {
                manip_simulator->get_end_effector_configuration(config, active_manipulation_info->manipulator, active_manipulation_info->chain.second);                
            }

        }
    }
}
