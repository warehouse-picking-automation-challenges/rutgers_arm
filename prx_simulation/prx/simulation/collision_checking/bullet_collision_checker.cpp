/**
 * @file ode_collision_checker.cpp  
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
#ifdef BULLET_FOUND
#include "prx/simulation/collision_checking/bullet_collision_checker.hpp"
#include "prx/simulation/collision_checking/vector_collision_list.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::bullet_collision_checker_t, prx::sim::collision_checker_t)

namespace prx
{
    using namespace util;

    namespace sim
    {

        bullet_collision_checker_t::bullet_collision_checker_t()
        {
            collision_list = NULL;

            current_collisions = new vector_collision_list_t();
        }

        bullet_collision_checker_t::~bullet_collision_checker_t() { }

        void bullet_collision_checker_t::set_configuration(const std::string& name, const config_t& config)
        {
            // PRX_FATAL_S("Invalid Operation \'Set Configuration\' for bullet_collision_checker_t.");
        }

        void bullet_collision_checker_t::add_body(const std::string& name, const geometry_t& geometry, system_t* plant, bool is_obstacle) {
            // all_bodies.push_back(name);
            //    PRX_LOG_WARNING("Invalid Operation \'Add Body\' for ode_collision_checker_t.");
        }

        void bullet_collision_checker_t::remove_body(const std::string& name)
        {
            PRX_FATAL_S("Invalid Operation \'Remove Body\' for ode_collision_checker_t.");
        }

        bool bullet_collision_checker_t::in_collision()
        {
            foreach(collision_pair_t pair, collision_list->get_body_pairs())
            {
                if(current_collisions->pair_in_list(pair.first,pair.second))
                {
                    current_collisions->clear();
                    return true;
                }
            }
            current_collisions->clear();
            return false;
        }

        void bullet_collision_checker_t::add_temporal_collision(const std::string& name1, const std::string& name2)
        {
            if(!current_collisions->pair_in_list(name1,name2))
            {
                // PRX_INFO_S("ADDING PAIR: "<<name1<<" "<<name2);
                collision_pair_t pair(name1,name2);
                current_collisions->add_new_pair(pair);
            }
        }

        void bullet_collision_checker_t::clear_collisions() {
            // contacts.clear();
            // if( contact_group != NULL )
            // {
            //     dJointGroupEmpty(contact_group);
            // }
        }

        collision_list_t* bullet_collision_checker_t::colliding_bodies()
        {
            //    PRX_ERROR_S ("ODE colliding bodies : " << contacts.size());
            vector_collision_list_t* col = new vector_collision_list_t();

            // for( unsigned i = 0; i < contacts.size(); ++i )
            // {
            //     std::string parent = ode_body_identifiers[ dJointGetBody(contacts[i], 0) ];
            //     std::string child = ode_body_identifiers[ dJointGetBody(contacts[i], 1) ];
            //     col->add_pair(parent, child);
            // }

            return col;
        }

        bool bullet_collision_checker_t::in_collision(const std::string& name1, const std::string& name2)
        {
            // for( unsigned i = 0; i < contacts.size(); ++i )
            // {
            //     dBodyID parent = dJointGetBody(contacts[i], 0);
            //     dBodyID child = dJointGetBody(contacts[i], 1);
            //     if( (name1 == ode_body_identifiers[parent] && name2 == ode_body_identifiers[child]) ||
            //         (name2 == ode_body_identifiers[parent] && name1 == ode_body_identifiers[child]) )
            //     {
            //         return true;
            //     }
            // }

            return false;
        }

        void bullet_collision_checker_t::link_collision_list(collision_list_t* list) 
        {
            collision_list = list;
            //check for ignored collisions
            // ignored_list.clear();
            // int count = 0;

            // foreach(std::string name_1, all_bodies)
            // {
            //     bool ignorable = true;

            //     foreach(std::string name_2, all_bodies)
            //     {
            //         if( name_1 != name_2 )
            //         {
            //             bool ret = (collision_list->pair_in_list(name_1, name_2));
            //             if( !ret )
            //             {
            //                 ignorable = false;
            //             }
            //         }
            //     }
            //     if( ignorable )
            //         ignored_list.push_back(name_1);
            //     else
            //         count++;
            // }
        }
    }
}
#endif
