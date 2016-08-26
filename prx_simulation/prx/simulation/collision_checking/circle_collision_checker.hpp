/**
 * @file circle_collision_checker.hpp
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

#ifndef PRX_CIRCLE_COLLISION_CHECKER_HPP
#define	PRX_CIRCLE_COLLISION_CHECKER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/math/3d_geometry/geometry.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"

namespace prx
{
    namespace sim
    {

        /**
         * An implementation of a collision checker that checks if two systems are in collision based on 
         * a maximum radius. Systems are approximated using a circle (which may have varying radii).
         * 
         * This class builds caches of paired systems to check for collisions in order to speed up
         * the collision checking process (since indexing into a hash is slower than iterating
         * over a list). The caches can be cleared without affecting the memory
         * of the systems.  Maps are used in order to provide system pathnames, and also contains
         * the memory used by the collision checking (and thus should not be cleared frivolously)
         * 
         * @brief <b> Fast collision checking based on a radius </b> 
         * 
         * @authors Andrew Kimmel
         */
        class circle_collision_checker_t : public collision_checker_t
        {

          public:

            friend std::ostream& operator<<(std::ostream& output, const circle_collision_checker_t& checker);

            circle_collision_checker_t();
            virtual ~circle_collision_checker_t();

            /** @copydoc collision_checker_t::set_configuration(const std::string&, const util::config_t&)*/
            void set_configuration(const std::string& name, const util::config_t& config);

            /** @copydoc collision_checker_t::add_body(const std::string&, const util::geometry_t&, system_t*, bool) */
            void add_body(const std::string& name, const util::geometry_t& geometry, system_t* plant, bool is_obstacle = false);

            /** @copydoc collision_checker_t::remove_body(const std::string&) 
             *  @note Not supported currently 
             */
            void remove_body(const std::string& name);

            /** @copydoc collision_checker_t::link_collision_list(collision_list_t*) 
             *  @note The caches are built from the collision list
             */
            void link_collision_list(collision_list_t* list);

            /** @copydoc collision_checker_t::in_collision() */
            bool in_collision();

            /** @copydoc collision_checker_t::colliding_bodies() */
            collision_list_t* colliding_bodies();

            virtual collision_checker_info_t* get_collision_info(const std::string& name);

          protected:

            /**
             * @brief Container class for defining non-circular obstacles
             */
            struct non_circular_obstacle
            {

                std::vector<util::vector_t> minkowski_vertices;
                util::config_t* other_conf;
                std::string obstacle_name;
                std::string other_name;
            };

            /** @brief A cache of the paired configurations to check */
            std::vector<std::pair<util::config_t*, util::config_t*> > config_cache;

            /** @brief A cache of the paired radii */
            std::vector<std::pair<double, double> > radii_cache;

            /** @brief A cache of the names of paired systems */
            std::vector<std::pair<std::string, std::string> > name_cache;

            /** @brief A cache of the non-circular obstacles to check */
            std::vector<non_circular_obstacle> non_circular_obstacle_cache;

            /** @brief Pairs together obstacle configurations with radii*/
            std::vector<std::pair<util::config_t*, double> > obstacle_configs_and_radii;

            /** @brief The pathnames of the rigid bodies */
            std::vector<std::string> body_names;
            /** @brief The pathnames of the obstacles */
            std::vector<std::string> obstacle_body_names;
            /** @brief Maps pathnames to systems */
            std::map<std::string, system_t*> body_map;
            /** @brief Maps a pathname to a rigid body name*/
            std::map<std::string, std::string> name_map;
            /** @brief Maps a pathname to a configuration */
            std::map<std::string, util::config_t> config_map;
            /** @brief Maps a pathname to a bounding  radius */
            std::map<std::string, double> radii_map;
            /** @brief Maps a pathname to a geometry */
            std::map<std::string, util::geometry_t> non_circular_obstacle_map;

          private:

            //    bool in_collision(int index1, int index2);
            //    bool in_collision(const std::string& name1, const std::string& name2);

        };

        std::ostream& operator<<(std::ostream& output, const circle_collision_checker_t& checker);


    }
}

#endif

