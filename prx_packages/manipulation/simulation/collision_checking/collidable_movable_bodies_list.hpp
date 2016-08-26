/**
 * @file collision_list.hpp 
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
#pragma once

#ifndef PRX_COLLIDABLE_MOVABLE_BODIES_LIST_HPP
#define PRX_COLLIDABLE_MOVABLE_BODIES_LIST_HPP

#include "prx/simulation/collision_checking/vector_collision_list.hpp"
#include "simulation/systems/plants/movable_body_plant.hpp"


#include <boost/range.hpp>
#include <boost/range/any_range.hpp>
#include <pluginlib/class_loader.h>

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
            /**
             * Its a list that will maintain the pairs of systems to be checked for collisions.
             * 
             * @brief  <b> The list that maintains the pairs of systems to be checked for collisions. </b>
             * 
             * @author Andrew Kimmel
             */
            class collidable_movable_bodies_list_t : public sim::vector_collision_list_t
            {

              public:
                collidable_movable_bodies_list_t();

                virtual ~collidable_movable_bodies_list_t();

                // The collision list needs to be informed
                virtual void setup_collision_list(const std::vector<movable_body_plant_t*>& movable_plants);

                // This will change the collision_pairs list in the vector_collision_list_t class
                virtual void update_target_objects(const std::vector<std::string>&  target_objects );

                virtual sim::vector_collision_list_t* get_object_independent_collision_list();

                virtual sim::vector_collision_list_t* get_target_object_collision_list(const std::string& target_object);


            protected:

                std::vector<std::string> collidable_obstacles;

                std::vector<movable_body_plant_t*> movable_bodies;

                util::hash_t< std::string, sim::vector_collision_list_t* > target_object_to_list;

                sim::vector_collision_list_t* object_independent_collision_list;

            };
        }

    }
}

#endif
