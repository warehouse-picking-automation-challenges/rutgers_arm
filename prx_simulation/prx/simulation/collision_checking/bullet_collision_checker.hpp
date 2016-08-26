/**
 * @file ode_collision_checker.hpp  
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
#pragma once
#ifndef PRX_BULLET_COLLISION_CHECKER_HPP
#define	PRX_BULLET_COLLISION_CHECKER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/math/3d_geometry/geometry.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"
#include "prx/simulation/collision_checking/vector_collision_list.hpp"

namespace prx
{
    namespace sim
    {

        /**
         * Collision checker using  Bullet.
         * 
         * @brief <b> Collision checker for Bullet </b>
         * 
         * @author Zakary Littlefield
         * 
         */
        class bullet_collision_checker_t : public collision_checker_t
        {

          public:
            bullet_collision_checker_t();
            virtual ~bullet_collision_checker_t();

            /** 
             * Sets the collision list (white list) of pairs of systems that we want to check for collisions.
             * 
             * @brief Sets the collision list.
             * 
             * @param list Is the list that contains 
             */
            virtual void link_collision_list(collision_list_t* list);

            /**
             * @copydoc collision_checker_t::set_configuration(const std::string& name, const util::config_t& config)
             */
            void set_configuration(const std::string& name, const util::config_t& config);

            /**
             * @copydoc collision_checker_t::add_body(const std::string& name, const util::geometry_t& geometry, system_t* plant, bool is_obstacle)
             */
            void add_body(const std::string& name, const util::geometry_t& geometry, system_t* plant, bool is_obstacle = false);

            /**
             * @copydoc collision_checker_t::remove_body(const std::string& name)
             */
            void remove_body(const std::string& name);

            /**
             * @copydoc collision_checker_t::in_collision()
             */
            bool in_collision();

            /**
             * ODE specific function to reset collision information.
             * 
             * @brief ODE specific function to reset collision information.
             */
            void clear_collisions();

            /**
             * @copydoc collision_checker_t::colliding_bodies()
             */
            collision_list_t* colliding_bodies();

            void add_temporal_collision(const std::string& name1, const std::string& name2);
            
            virtual collision_checker_info_t* get_collision_info(const std::string& name){return NULL;}

          private:

            bool in_collision(const std::string& name1, const std::string& name2);

            vector_collision_list_t* current_collisions;

        };

        // std::ostream& operator<<( std::ostream& output, const bullet_collision_checker_t& checker );


    }
}

#endif
#endif

