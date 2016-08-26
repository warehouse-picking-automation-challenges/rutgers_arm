/**
 * @file fcl_collision_checker.hpp
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

#ifndef PRX_FCL_COLLISION_CHECKER_HPP
#define	PRX_FCL_COLLISION_CHECKER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/math/3d_geometry/geometry.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"

#include <fcl/BVH/BVH_model.h>
#include <fcl/BV/BV.h>
#include <fcl/collision.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/collision_node.h>
#include <fcl/math/transform.h>
#include <fcl/traversal/traversal_node_bvhs.h>
#include <fcl/octree.h>
//#include <fcl/simple_setup.h>
#include <fcl/ccd/conservative_advancement.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>


namespace prx
{
    namespace sim
    {

        typedef fcl::CollisionGeometry FCL_Model;
        typedef fcl::BVHModel<fcl::OBBRSS> FCL_Mesh;


        /**
         * @brief <b> A struct to hold relevant information for FCL </b>
         */
        class fcl_info_t : public collision_checker_info_t
        {
        public:

            /**
             * @brief The FCL model of the system.
             */
            FCL_Model* model;

            /**
             * @brief The system that this geometry belongs to.
             */
            system_t* system;

            /**
             * @brief FCL quaternion.
             */
            fcl::Quaternion3f quaternion;

            /**
             * @brief FCL vector for translation.
             */
            fcl::Vec3f translation;

            /**
             * @brief FCL transformation.
             */
            fcl::Transform3f transform;
            
            /**
             * 
             * @brief broadphase radius
             */
            double broadphase_radius;


            /**
             * @brief Constructor
             * @param p The system pointer.
             */
            fcl_info_t(system_t * p) : pos(3)
            {
                system = p;
                model = NULL;//new FCL_Model();
                broadphase_radius = PRX_INFINITY;
            }

            ~fcl_info_t()
            {
                delete model;
            }

            /**
             * @brief Given new coordinates, translates the geometry
             * @param pos The new position.
             */
            void update_translation_matrix(const util::vector_t & pos)
            {
                update_translation_matrix(pos[0], pos[1], pos[2]);
            }

            /**
             * @brief Given new coordinates, translates the geometry
             * @param x New x position.
             * @param y New y position.
             * @param z New z position.
             */
            void update_translation_matrix(double x, double y, double z)
            {
                translation.setValue(x, y, z);
            }

            /**
             * @brief Update the orientation of the object.
             * @param quat A quaternion that represents the rotation.
             */
            void update_quaternion(const util::quaternion_t & quat)
            {
                quaternion.getW() = quat.get_w();
                quaternion.getX() = quat.get_x();
                quaternion.getY() = quat.get_y();
                quaternion.getZ() = quat.get_z();
            }

            /**
             * @brief Update the translation and rotation of the geometry.
             * @param config The translation and rotation.
             */
            virtual void update_matrices(const util::config_t & config)
            {
                config.get(pos, q);
                update_translation_matrix(pos);
                update_quaternion(q);
                transform.setTransform(quaternion, translation);
            }

            /**
             * If information is necessary, add a print in this method.
             */
            void print(){ }

          private:

            /**
             * @brief Stored position.
             */
            util::vector_t pos;

            /**
             * @brief Stored rotation.
             */
            util::quaternion_t q;

        };

        /**
         * @brief <b> A collision checker that uses FCL </b>
         * 
         * @author Zakary Littlefield
         */
        class fcl_collision_checker_t : public collision_checker_t
        {

          public:

            /**
             * @brief Output stream for the collision checker.
             */
            friend std::ostream& operator<<(std::ostream& output, const fcl_collision_checker_t& checker);

            fcl_collision_checker_t();
            virtual ~fcl_collision_checker_t();


            virtual void init(const util::parameter_reader_t * const reader)
            {
                octomap_resolution = reader->get_attribute_as<double>("octomap_resolution",.01);
            }

            /**
             * Sets the white list of pairs of systems to check for collisions.
             *
             * @brief Sets the white list of pairs of systems to check for collisions.
             *
             * @list The new white list of pairs that need to be checked for collisions.
             */
            void link_collision_list(collision_list_t* list);

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
             * @copydoc collision_checker_t::colliding_bodies()
             */
            collision_list_t* colliding_bodies();

            /**
             * @brief Create an FCL model given a name and geometry.
             * @param name The name of the rigid body.
             * @param geometry The geometry.
             */
            void update_model(const std::string& name, sensor_msgs::PointCloud2& cloud);

            virtual collision_checker_info_t* get_collision_info(const std::string& name);

            virtual bool info_in_plants()
            {
                  return true;
            }
            
          protected:

            /**
             * @brief Hash mapping from rigid body names to FCL geometry information.
             */
            util::hash_t<std::string, fcl_info_t*, util::string_hash> models_map;

            /**
             * @brief An iterator over the \ref models_map hash map.
             */
            util::hash_t<std::string, fcl_info_t*, util::string_hash>::iterator models_map_iter;

            /**
             * The list of the systems that will be checked for collisions.
             * @brief The list of the systems that will be checked for collisions.
             */
            std::vector<std::pair<fcl_info_t*, fcl_info_t*> > body_cache;


          private:
            /**
             * @brief Storage for resulting collisions.
             */
            std::vector<fcl::Contact> contacts;
            double octomap_resolution;

            /**
             * @brief Create an FCL model given a name and geometry.
             * @param name The name of the rigid body.
             * @param geometry The geometry.
             */
            void create_model(const std::string& name, const util::geometry_t& geometry);

            /**
             * @brief Asks if two rigid bodies are in collision.
             * @param name1 The name of the first rigid body.
             * @param name2 The name of the second rigid body.
             * @return True if colliding, false if not.
             */
            bool in_collision(const std::string& name1, const std::string& name2);

            /**
             * Checks if the two systems are in collision or not.
             *
             * @brief Checks if the two systems are in collision or not.
             *
             * @param name1 The fcl information for the first system.
             * @param name2 The fcl information for the second system.
             *
             * @return True if the systems are in collision. \n
             *         False if they are not in collision.
             */
            bool in_collision(fcl_info_t* name1, fcl_info_t* name2);

        };

        std::ostream& operator<<(std::ostream& output, const fcl_collision_checker_t& checker);


    }
}

#endif

