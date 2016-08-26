/**
 * @file vector_collision_list.cpp 
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

#include "simulation/collision_checking/collidable_movable_bodies_list.hpp"
#include <boost/range/adaptor/map.hpp>
#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::collidable_movable_bodies_list_t, prx::sim::collision_list_t)

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {
        namespace manipulation
        {
            collidable_movable_bodies_list_t::collidable_movable_bodies_list_t()
            {
                object_independent_collision_list = new vector_collision_list_t();
            }

            collidable_movable_bodies_list_t::~collidable_movable_bodies_list_t()
            {

                foreach ( vector_collision_list_t* list, target_object_to_list | boost::adaptors::map_values )
                {
                    delete list;
                }

                delete object_independent_collision_list;
            }

            void collidable_movable_bodies_list_t::setup_collision_list(const std::vector<movable_body_plant_t*>& movable_plants)
            {
                movable_bodies = movable_plants;

                foreach(movable_body_plant_t* body, movable_bodies)
                {
                    target_object_to_list[body->get_pathname()] = new vector_collision_list_t();
                }
            }


            void collidable_movable_bodies_list_t::update_target_objects(const std::vector<std::string>&  target_objects )
            {
                // Update the collision list

                collision_pairs.clear();

                foreach(collision_pair_t pair, object_independent_collision_list->get_body_pairs())
                {
                    //PRX_DEBUG_COLOR("Added pair: " << pair.first << "," << pair.second, PRX_TEXT_MAGENTA);
                    collision_pairs.push_back(pair);
                }

                foreach(std::string object_path, target_objects)
                {
                    foreach(collision_pair_t pair, target_object_to_list[object_path]->get_body_pairs())
                    {
                        //PRX_DEBUG_COLOR("Added pair: " << pair.first << "," << pair.second, PRX_TEXT_GREEN);
                        collision_pairs.push_back(pair);
                    }
                }
            }

            sim::vector_collision_list_t* collidable_movable_bodies_list_t::get_object_independent_collision_list()
            {
                return object_independent_collision_list;
            }

            sim::vector_collision_list_t* collidable_movable_bodies_list_t::get_target_object_collision_list(const std::string& target_object)
            {
                if (target_object_to_list.find(target_object) == target_object_to_list.end())
                {
                    PRX_FATAL_S ("Target object: " << target_object << " does not have an entry in the target object list !");
                }
                else
                {
                    return target_object_to_list[target_object];
                }
            }

        }
    }
}

