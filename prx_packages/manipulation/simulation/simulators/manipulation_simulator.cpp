/**
 * @file manipulator_simulator.hpp
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

#include "simulation/simulators/manipulation_simulator.hpp"

#include "prx/simulation/collision_checking/collision_checker.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/simulation/cost_functions/default_uniform.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/tuple/tuple.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/range/adaptor/map.hpp>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::manipulation_simulator_t, prx::sim::simulator_t);
PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::manipulation_simulator_t, prx::sim::system_t);

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {

        namespace manipulation
        {

            manipulation_simulator_t::manipulation_simulator_t() : default_simulator_t()
            {
                collision_response = false;
                collision_detection = true;
                
                updated_collision_list = new vector_collision_list_t();
            }

            manipulation_simulator_t::~manipulation_simulator_t()
            {
                delete updated_collision_list;
            }

            void manipulation_simulator_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
            {
                simulator_t::init(reader, template_reader);

                util::hash_t<std::string, sim::plant_t*> plants;
                system_graph_t::directed_vertex_t v = update_system_graph(sys_graph);
                sys_graph.get_path_plant_hash(plants);

                foreach(plant_t* plant, plants | boost::adaptors::map_values)
                {
                    //Create and initialize information for all the end effectors. 
                    if(dynamic_cast<manipulator_t*>(plant) != NULL)
                    {
                        //Create end effector infos for each end effector.
                        manipulator_t* manip = dynamic_cast<manipulator_t*>(plant);
                        manipulators.push_back(manip);
                        unsigned index = 0;
                        foreach(std::string ee_name, manip->get_end_effector_names())
                        {                            
                            end_effectors.push_back(new end_effector_info_t(manip,index,ee_name));
                            index++;
                        }
                    }
                    //Create a list of all the movable bodies.
                    if(dynamic_cast<movable_body_plant_t*>(plant) != NULL)
                    {
                        movable_bodies.push_back(dynamic_cast<movable_body_plant_t*>(plant));
                    }  
                    
                }

                if(end_effectors.size() == 0)
                    PRX_FATAL_S("Manipulation simulator needs at least one end effector!");

                if(movable_bodies.size() == 0)
                    PRX_WARN_S("Manipulation simulator needs at least one movable body so the manipulator can grasp something!");


                state_memory.clear();  
                std::string space_name;
                unsigned size = end_effectors.size();
                unsigned index = 0;
                //setup collision list for end effectors and the state space of the simulator.
                foreach(end_effector_info_t* info, end_effectors)
                {
                    foreach(movable_body_plant_t* object, movable_bodies)
                    {
                        std::vector<std::string>  names = *object->get_geometries_names();
                        foreach(std::string& s, names)
                        {
                            info->grasping_check_list->add_pair(info->end_effector_name,s);
                        }
                    }

                    state_memory.push_back(info->grasped_object_id);
                    space_name += "D";
                    if(index < size-1)
                        space_name += "|";
                    index++;
                }
                controller_state_space = new space_t(space_name,state_memory);

                std::vector<bounds_t*> bounds = controller_state_space->get_bounds();

                for (int i = 0; i < bounds.size(); ++i)
                {
                    bounds[i]->set_bounds(-1,movable_bodies.size()-1);
                }

                construct_spaces();

                //Determining if we are starting with a grasp already. 
                bool collision_is_updated = false;
                foreach(end_effector_info_t* info, end_effectors)
                {
                    //if the grasp state has changed.
                    bool grasp = info->manipulator->is_end_effector_closed(info->end_effector_index);
                    //Grasp a new object.
                    if(grasp)
                    {   
                        if(!collision_is_updated)
                        {
                            update_collision_info();
                            collision_is_updated = true;
                        }
                        //We have to start a new grasp and get the correct object through collision checking.                             
                        detect_new_grasped_object(info);
                    }
                }
            }

            void manipulation_simulator_t::push_state(const sim::state_t * const state)
            {
                default_simulator_t::push_state(state);
                
                foreach(end_effector_info_t* info, end_effectors)
                {
                    bool currently_closed = info->manipulator->is_end_effector_closed( info->end_effector_index );
                    if( currently_closed && *info->grasped_object_id >= 0 )
                    {
                        //Compute relative config between end effector and the grasped object. 
                        if(!info->static_relative_configuration)
                        {
                            compute_relative_config(info);
                        }
                    }
                    else if( !currently_closed && *info->grasped_object_id >= 0 )
                    {
                        release_object(info);
                    }
                }
            }

            void manipulation_simulator_t::link_collision_list(collision_list_t* collision_list)
            {
                updated_collision_list->clear();

                PRX_DEBUG_COLOR("Number of collision pairs before: " << collision_list->size(), PRX_TEXT_LIGHTGRAY);
                foreach(collision_pair_t p, collision_list->get_body_pairs())
                {
                    bool found_ee = false;
                    for( unsigned i = 0; i < end_effectors.size() && !found_ee; ++i )
                    {
                        if( p.first == end_effectors[i]->end_effector_name || p.second == end_effectors[i]->end_effector_name )
                        {
                            found_ee = true;
                        }
                    }
                    if(!found_ee)
                        updated_collision_list->add_new_pair(p);
                }
                PRX_DEBUG_COLOR("Number of collision pairs after: " << updated_collision_list->size(), PRX_TEXT_LIGHTGRAY);

                collision_checker->link_collision_list(updated_collision_list);
            }

            bool manipulation_simulator_t::in_collision()
            {
                //Make sure all objects being grasped are in their appropriate states
                move_grasped_objects();
 
                sim::collision_list_t* in_collision_list = NULL;
                //For each end-effector
                for( unsigned i = 0; i < end_effectors.size() && in_collision_list == NULL; ++i )
                {
                    //if it is grasping an object
                    if( end_effectors[i]->object != NULL )
                    {
                        //and that end effector has a non-empty set of bodies to ignore during grasping
                        if(!end_effectors[i]->ignore_during_grasping.empty())
                        {
                            //Get all of the colliding bodies in the simulation
                            in_collision_list = default_simulator_t::get_colliding_bodies();
                        }
                    }
                }
                
                //If we didn't ever have bodies to ignore (nothing grasping, for example)
                if (in_collision_list == NULL)
                {
                    //Simply return our default answer of whether or not something is colliding
                    return default_simulator_t::in_collision();
                }
                //Otherwise, we have to filter through the pairs, looking for hand-object contacts
                else
                {
                    //So for each of those pairs
                    foreach( collision_pair_t p, in_collision_list->get_body_pairs() )
                    {
                        bool valid_collision = true;
                        //Look at every end-effector in the scene
                        for( unsigned i = 0; valid_collision == true && i < end_effectors.size(); ++i )
                        {
                            //If that end effector is grasping
                            if( end_effectors[i]->object != NULL )
                            {
                                //Look over all of the bodies we are supposed to be ignoring for this hand
                                for( unsigned j=0; valid_collision==true && j < end_effectors[i]->ignore_during_grasping.size(); ++j )
                                {
                                    //If the first body in the pair matches the ignore list
                                    if( p.first == end_effectors[i]->ignore_during_grasping[j] )
                                    {
                                        //Get all the geometries which comprise the object this arm is manipulating
                                        std::vector<std::string>* rigid_body_names = end_effectors[i]->object->get_geometries_names();
                                        //For each of those geometries
                                        for(unsigned r=0; valid_collision == true && r < rigid_body_names->size(); ++r)
                                        {
                                            //If the second half of the pair is that geometry, then this is a contact we allow
                                            if(p.second == rigid_body_names->at(r))
                                                valid_collision = false;
                                        }
                                    }
                                    //Otherwise, if the second body in the pair mathes the ignore list
                                    else if(p.second == end_effectors[i]->ignore_during_grasping[j] )
                                    {
                                        //Get all the geometries which comprise the object this arm is manipulating
                                        std::vector<std::string>* rigid_body_names = end_effectors[i]->object->get_geometries_names();
                                        //For each of those geometries
                                        for(unsigned r=0; valid_collision == true && r < rigid_body_names->size(); ++r)
                                        {
                                            //If the second half of the pair is that geometry, then this is a contact we allow
                                            if(p.first == rigid_body_names->at(r))
                                                valid_collision = false;
                                        }
                                    }
                                }
                            }
                        }

                        //If we ended up with a collision which wasn't a hand-object contact
                        if( valid_collision )
                        {
                            //Report collision
                            PRX_INFO_S("In collision: "<<p.first<<" "<<p.second);
                            return true;
                        }
                    }
                    
                    return false;
                }
                
            }

            collision_list_t* manipulation_simulator_t::get_colliding_bodies()
            {
                move_grasped_objects();
                return simulator_t::get_colliding_bodies();
            }

            void manipulation_simulator_t::propagate(const double simulation_step)
            {
                /** This commented out code computes surface normals for movable bodies, and visualizes them */
//                bool visualize_normals = true;
//                if (visualize_normals)
//                {
//                    hash_t<std::string, geometry_info_t> geoms;
//                    std::vector<std::string> names;
//                    std::vector<config_t> confs;
//                    std::vector<vector_t> colors;
//                    for (unsigned m = 0; m < movable_bodies.size(); ++m)
//                    {   
//                        /** The following code is to calculate surface normals for the movable body*/
//                        util::geom_map_t movable_body_geoms;
//                        config_t root_config;
//                        movable_bodies[m]->update_phys_geoms(movable_body_geoms);
//                        movable_bodies[m]->get_configuration(root_config);
//                        // Compute surface normals for geoms
//                        foreach(geometry_t geom, movable_body_geoms | boost::adaptors::map_values)
//                        {
//                            std::vector<vector_t> normals, centroids;
//                            std::vector<double> surface_areas;
//                            geom.get_trimesh()->get_all_normal_vectors(normals, centroids, surface_areas);
//                            for (unsigned i = 0; i < normals.size(); ++i)
//                            {
//                                PRX_DEBUG_COLOR("Normal " << i << " : " << normals[i], PRX_TEXT_MAGENTA);
//                                PRX_DEBUG_COLOR("Centroid: " << centroids[i], PRX_TEXT_GREEN);
//                                PRX_DEBUG_COLOR("Area: " << surface_areas[i], PRX_TEXT_CYAN);
//                                util::config_t start_config, end_config;
//                                start_config.set_position(centroids[i]);
//                                start_config.set_orientation(0,0,0,1);
//                                end_config.set_position((centroids[i] + normals[i]*0.1));
//                                end_config.set_orientation(0,0,0,1);
//
//                                start_config.relative_to_global(root_config);
//                                end_config.relative_to_global(root_config);
//
//                                PRX_DEBUG_COLOR ("Start: " << start_config, PRX_TEXT_BROWN);
//                                PRX_DEBUG_COLOR("End: " << end_config, PRX_TEXT_BROWN);
//
//
//                                std::vector<double> params, params2;
//
//                                PRX_DEBUG_COLOR("Next Step : ", PRX_TEXT_BROWN );
//
//                                std::string name = ros::this_node::getName() + "/" + movable_bodies[m]->get_pathname();
//                                params.push_back(0.005);
//                //                vector_t colorr(3);
//                //                colorr.set(0.5,0.2,0.8);
//                                std::stringstream goal, start, line;
//                                goal << "goal" <<i;
//                                start << "start" << i;
//                                line << "line" << i;
//
//                                geometry_info_t geom_info1(name, goal.str(), PRX_SPHERE, params, "blue");
//                                geometry_info_t geom_info2(name, start.str(), PRX_SPHERE, params, "green");
//                                params2.push_back(start_config.get_position().at(0));
//                                params2.push_back(start_config.get_position().at(1));
//                                params2.push_back(start_config.get_position().at(2));
//                                params2.push_back(end_config.get_position().at(0));
//                                params2.push_back(end_config.get_position().at(1));
//                                params2.push_back(end_config.get_position().at(2));
//                                geometry_info_t geom_info3(name, line.str(), PRX_LINES, params2, "red");
//
//
//                                geoms[ geom_info1.full_name ] = geom_info1;
//                                geoms[ geom_info2.full_name ] = geom_info2;
//                                geoms[ geom_info3.full_name ] = geom_info3;
//
//                                names.push_back(name+"/"+goal.str());
//                                names.push_back(name+"/"+start.str());
//                                names.push_back(name+"/"+line.str());
//                                confs.push_back(end_config);
//                                confs.push_back(start_config);
//                                confs.push_back(config_t());
//
//                            }
//                        }
//                    }
//                    (sim::comm::vis_comm)->send_extra_geometries(geoms);
//                    (sim::comm::vis_comm)->update_info_geoms(names, confs, colors, false);
//                }
                foreach(end_effector_info_t* info, end_effectors)
                {
                    info->prev_grasping_mode = info->manipulator->is_end_effector_closed(info->end_effector_index);
                }

                //Do the standard propagation
                simulator_t::propagate( simulation_step );

                move_grasped_objects();

                //Determining if a new grasp is need or if we need to release an object. 
                bool collision_is_updated = false;
                foreach(end_effector_info_t* info, end_effectors)
                {
                    //if the grasp state has changed.
                    bool grasp = info->manipulator->is_end_effector_closed(info->end_effector_index);
                    if(info->prev_grasping_mode != grasp)
                    {  
                        //Grasp a new object.
                        if(grasp)
                        {   
                            if(!collision_is_updated)
                            {
                                update_collision_info();
                                collision_is_updated = true;
                            }
                            //We have to start a new grasp and get the correct object through collision checking.                             
                            detect_new_grasped_object(info);
                            collision_checker->link_collision_list(updated_collision_list);
                        }
                        else
                        {
                            release_object(info);
                        }
                    }
                }
            }

            void manipulation_simulator_t::propagate_and_respond()
            {
                propagate(simulation::simulation_step);
                in_collision();
            }

            void manipulation_simulator_t::set_static_relative_config(std::string name, bool flag)
            {
                foreach(end_effector_info_t* info, end_effectors)
                {
                    if(name == info->end_effector_name)
                    {
                        if(flag && info->object == NULL)
                            PRX_FATAL_S("You cannot maintain a relative configuration without grasping an object!");
                        info->static_relative_configuration = flag;
                        return;
                    }
                }
            }

            void manipulation_simulator_t::get_movable_objects(std::vector<movable_body_plant_t* >& objects)
            {
                objects.insert(objects.end(), movable_bodies.begin(), movable_bodies.end());
            }

            void manipulation_simulator_t::get_manipulators(std::vector<manipulator_t* >& manipulators)
            {
                manipulators.insert(manipulators.end(), this->manipulators.begin(), this->manipulators.end());
            }

            bool manipulation_simulator_t::is_end_effector_grasping(manipulator_t* manipulator, std::string simple_end_effector_name)
            {
                PRX_DEBUG_COLOR("Checking end effector grasping for: " << simple_end_effector_name, PRX_TEXT_RED);
                foreach(end_effector_info_t* info, end_effectors)
                {
                    if(info->is_ee_mine(manipulator,simple_end_effector_name))
                    {
                        return *(info->grasped_object_id) != -1;
                    }
                }
                PRX_FATAL_S("You have given wrong combination of manipulator and end effector. " << manipulator->get_pathname() << " - " << simple_end_effector_name );
                return false;
            }

            void manipulation_simulator_t::get_end_effector_configuration(util::config_t& config, manipulator_t* manipulator, std::string simple_end_effector_name)
            {
                foreach(end_effector_info_t* info, end_effectors)
                {
                    if(info->is_ee_mine(manipulator,simple_end_effector_name))
                    {
                        manipulator->get_end_effector_configuration(config, info->end_effector_index);
                        return;
                    }
                }
                PRX_FATAL_S("You have given wrong combination of manipulator and end effector. " << manipulator->get_pathname() << " - " << simple_end_effector_name );
                return;
            }

            void manipulation_simulator_t::move_grasped_objects()
            {
                //For each end effector
                foreach(end_effector_info_t* info, end_effectors)
                {
                    if(info->object != NULL)
                    {
                        //PRX_DEBUG_COLOR("---RELATIVE CONFIG BEFORE: " << info->relative_config.print(), PRX_TEXT_CYAN);
                        info->manipulator->FK_solver(effector_config,info->simple_end_effector_name);
                        info->object->move_object(effector_config,info->relative_config);
                        //PRX_DEBUG_COLOR("+++RELATIVE CONFIG AFTER: " << info->relative_config.print(), PRX_TEXT_CYAN);
                        // PRX_STATUS_S("Moving grasped object");
                        // PRX_STATUS("Object [" << info->object->get_pathname() << "] moved to: " << info->object->get_state_space()->print_memory(3), PRX_TEXT_LIGHTGRAY);
                    }
                    else
                    {
                        //PRX_WARN_S ("Info->object is NULL!");
                    }
                }
            }        

            void manipulation_simulator_t::compute_relative_config(end_effector_info_t* info)
            {
                info->object = movable_bodies[*info->grasped_object_id];
                info->manipulator->FK_solver(effector_config,info->simple_end_effector_name);
                info->object->relative_config_with_manipulator(effector_config,info->relative_config);
            }

            void manipulation_simulator_t::release_object(end_effector_info_t* info)
            {
                info->object = NULL;
                *info->grasped_object_id = -1;
                info->static_relative_configuration = false;
            }

            bool manipulation_simulator_t::detect_new_grasped_object(end_effector_info_t* info)
            {
                collision_checker->link_collision_list(info->grasping_check_list);
                sim::collision_list_t* in_collision_list = default_simulator_t::get_colliding_bodies();

                PRX_DEBUG_COLOR("Collision list : " << in_collision_list->size() , PRX_TEXT_BROWN);
                PRX_ASSERT(*info->grasped_object_id == -1);
                foreach( collision_pair_t p, in_collision_list->get_body_pairs() )
                {
                    PRX_DEBUG_COLOR("New Collision : " << p.first << " - " << p.second , PRX_TEXT_RED);
                    for(unsigned i = 0; i < movable_bodies.size(); ++i)
                    {
                        std::vector<std::string>* rigid_body_names = movable_bodies[i]->get_geometries_names();
                        for(unsigned j = 0; j < rigid_body_names->size(); ++j)
                        {
                            //We need to check only the p.second because the collision list has been set to be 
                            //pairs of end effector vs movable object. 
                            if(p.second == rigid_body_names->at(j))
                            {
#ifndef NDEBUG
                                if(*info->grasped_object_id >= 0)
                                {
                                    PRX_FATAL_S("I do not know what to do! I cannot grasp two objects at the same time!!!");
                                }
#endif
                                *info->grasped_object_id = i;
                                compute_relative_config(info);
                                info->static_relative_configuration=true;
#ifndef NDEBUG
                                break;
#else
                                return true;
#endif
                            }
                        }
                    }
                }
                return false;
            }

            void manipulation_simulator_t::print_relative_config()
            {
                foreach(end_effector_info_t* info, end_effectors)
                {
                    PRX_PRINT(info->print(),PRX_TEXT_GREEN);
                }
            }
        }
    }
}
