/**
 * @file object_constraints_checker.cpp 
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


#include "planning/modules/object_constraints_checker.hpp"
#include "prx/utilities/heuristic_search/object_collision_constraints.hpp"

#include "planning/manipulation_world_model.hpp"

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::packages::manipulation::object_constraints_checker_t, prx::plan::validity_checker_t)


namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {

        namespace manipulation
        {
            
            void object_constraints_checker_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                ignore_list = NULL;
                ignore_list_validity_checker_t::init(reader, template_reader);
                map_generated = false;

                object_name = "";
            }

            void object_constraints_checker_t::setup_checker( const std::string& object_name )
            {
                this->object_name = object_name;
                //Then, if we don't have constraint names
                if( !map_generated )
                {
                    //Generate the default map
                    generate_constraint_names();
                }
            }

            bool object_constraints_checker_t::is_valid(const state_t* state)
            {
                colliding_bodies = world_model->get_colliding_bodies(state);
                
                //For each pair of colliding bodies
                foreach(collision_pair_t cp, colliding_bodies->get_body_pairs())
                {
                    //If it is not in the list of things to ignore
                    if( ignore_list == NULL || !(ignore_list->pair_in_list(cp.first, cp.second)) )
                    {
                        //First of all, if we have nothing in the list, any collision that is bad news
                        if( object_names.size() == 0 )
                        {
                            return false;
                        }
                        //Get the object names
                        std::string first_object = reverse_split_path( cp.first ).first;
                        std::string second_object = reverse_split_path( cp.second ).first;

                        //Begin by assuming this is not a valid collision
                        bool valid_collision = false;
                        //Search through our known object names
                        for( unsigned i=0; i<object_names.size() && !valid_collision; ++i )
                        {
                            //Consider only objects we are not trying to manipulate
                            if( object_names[i] != object_name )
                            {
                                //If this constraint is one of the bodies in the collision pair
                                if( object_names[i] == first_object || object_names[i] == second_object )
                                {
                                    //it is a valid collision because it involves one of our constraints
                                    valid_collision = true;   
                                }
                            }
                        }
                        //Then, if it turns out this pair was not what we would consider a valid collision
                        if( !valid_collision )
                        {
                            //Report a collision
                            return false;
                        }
                    }
                }
                //If we made it through all the collision pairs without finding an invalid collision, report success
                return true;
            }

            bool object_constraints_checker_t::validate_and_generate_constraints(constraints_t* constraints, const sim::state_t* state)
            {
                //Get a cast pointer to the constraints
                object_collision_constraints_t* ocsc = dynamic_cast<object_collision_constraints_t*>(constraints);
                colliding_bodies = world_model->get_colliding_bodies(state);
                // PRX_PRINT ("WORLD MODEL STATE: " << world_model->get_state_space()->print_memory(3), PRX_TEXT_LIGHTGRAY);
                //For each pair of colliding bodies
                foreach(collision_pair_t cp, colliding_bodies->get_body_pairs())
                {
                    // PRX_PRINT("Collision Pair: [" << cp.first << ", " << cp.second << "]  -  check in list: " << ignore_list, PRX_TEXT_BROWN);
                    //If it is not in the list of things to ignore
                    if( ignore_list == NULL || !(ignore_list->pair_in_list(cp.first, cp.second)) )
                    {
                        // PRX_WARN_S ("3");
                        //First of all, if we have nothing in the list, any collision is automatically invalid
                        if( object_names.size() == 0 )
                        {
                            ocsc->clear();
                            return false;
                        }
                        
                        //Get the object names
                        std::string first_object = reverse_split_path( cp.first ).first;
                        std::string second_object = reverse_split_path( cp.second ).first;

                        //Begin by assuming this is not a valid collision
                        bool valid_collision = false;
                        //Search through our known object names
                        for( unsigned i=0; i<object_names.size() && !valid_collision; ++i )
                        {
                            //Consider only objects we are not trying to manipulate
                            if( object_names[i] != object_name )
                            {
                                //If this constraint is one of the bodies in the collision pair
                                if( object_names[i] == first_object || object_names[i] == second_object )
                                {
                                    //it is a valid collision because it involves one of our constraints
                                    valid_collision = true;
                                    //But we also need to report that as a constraint
                                    ocsc->constraints.insert( i );
                                }
                            }
                        }
                        //Then, if it turns out this pair was not what we would consider a valid collision
                        if( !valid_collision )
                        {
                            //Clear the constraints and report a collision
                            ocsc->clear();
                            return false;
                        }
                    }
                }
                //If we made it through all the collision pairs without finding an invalid collision, report success
                return true;
            }

            void object_constraints_checker_t::collides_with(std::vector<std::string>& bodies)
            {
                foreach(collision_pair_t cp, world_model->get_colliding_bodies()->get_body_pairs())
                {                    
                    if( ignore_list == NULL || !(ignore_list->pair_in_list(cp.first, cp.second)) )
                    {
                        if( cp.first == object_name )
                            bodies.push_back(cp.second);

                        if( cp.second == object_name )
                            bodies.push_back(cp.first);
                    }
                }
            }
            
            constraints_t* object_constraints_checker_t::alloc_constraint() const
            {
                return new object_collision_constraints_t();
            }

            void object_constraints_checker_t::get_constraint_names( std::vector< std::string >& names )
            {
                if( !map_generated )
                {
                    //Generate the default map
                    generate_constraint_names();
                }
                
                //Copy over the map data
                names.assign( object_names.begin(), object_names.end() );
            }

            void object_constraints_checker_t::generate_constraint_names()
            {
                map_generated = true;
                
                system_graph_t& sys_graph = world_model->get_system_graph();
                std::vector<plant_t*> all_plants;
                sys_graph.get_plants( all_plants );
                
                foreach( plant_t* plant, all_plants )
                {
                    if( dynamic_cast< movable_body_plant_t* >(plant) != NULL )
                    {
                        object_names.push_back( plant->get_pathname() );
                    }
                }
            }

            void object_constraints_checker_t::clear_constraint_names()
            {
                object_names.clear();
                map_generated = false;
            }


        }
    }
}
