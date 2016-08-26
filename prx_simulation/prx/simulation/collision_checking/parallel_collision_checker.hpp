/**
 * @file parallel_collision_checker.hpp
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

#ifdef CUDA_COLLISION
#pragma once

#ifndef PRX_PARALLEL_COLLISION_CHECKER_HPP
#define	PRX_PARALLEL_COLLISION_CHECKER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/math/3d_geometry/geometry.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"

#include "CUDACollisionCheck.h"

#ifdef __APPLE__
#include <PQP.h>
#else
#include <PQP.h>
#endif

namespace prx
{
    namespace sim
    {

        class parallel_pqp_info_t : public collision_checker_info_t
        {
        public:
            /**
             * The PQP model.
             * @brief The PQP model.
             */
            PQP_Model* model;
            /**
             * The pointer for the \ref system_t.
             * @brief The pointer for the \ref system_t.
             */
            system_t* system;
            /**
             * The name of the system.
             * @brief The name of the system.
             */
            std::string name;

            std::vector<float > states;


            /**
             * As a basic broadphase algorithm, find that sphere that encloses the model,
             * then perform basic distance checks to determine if narrow phase collision
             * checking is needed.
             */
            double broadphase_radius;

            parallel_pqp_info_t(system_t * p) : pos(3)
            {
                system = p;
                model = new PQP_Model();
                broadphase_radius = PRX_INFINITY;
                pos.resize(3);
            }

            ~parallel_pqp_info_t()
            {
                delete model;
            }

            virtual void clear()
            {
                states.clear();
            }

            virtual void update_matrices(const util::config_t & config)
            {
                config.get(pos, q);

                states.insert(states.end(),{(float)pos[0],(float)pos[1],(float)pos[2], (float)q.get_w(),(float)q.get_x(), (float)q.get_y(), (float)q.get_z()});

            }

            std::string print() const
            {
                // std::stringstream out(std::stringstream::out);

                // out << "position: x:" << translation_matrix[0] << "  y:" << translation_matrix[1] << "  z:" << translation_matrix[2] << std::endl;
                // out << "Rotation: [" << std::endl;
                // out << rotation_matrix[0][0] << " , " << rotation_matrix[0][1] << " , " << rotation_matrix[0][2] << std::endl;
                // out << rotation_matrix[1][0] << " , " << rotation_matrix[1][1] << " , " << rotation_matrix[1][2] << std::endl;
                // out << rotation_matrix[2][0] << " , " << rotation_matrix[2][1] << " , " << rotation_matrix[2][2] << std::endl;
                // out << "]" << std::endl;

                // return out.str();
            }

          private:
            /**
             * The position of the model.
             * @brief The position of the model.
             */
            util::vector_t pos;
            /**
             * The quaternion of the model.
             * @brief The quaternion of the model.
             */
            util::quaternion_t q;

        };

        class parallel_collision_checker_t : public collision_checker_t
        {

          public:

            friend std::ostream& operator<<(std::ostream& output, const parallel_collision_checker_t& checker);

            parallel_collision_checker_t();
            virtual ~parallel_collision_checker_t();

            /**
             * Sets the new configuration of the system.
             *
             * @brief Sets the new configuration of the system.
             *
             * @param name The full slash-delimited path for the system to set the new configuration.
             * @param config The new configuration for the system.
             */
            void set_configuration(const std::string& name, const util::config_t& config);

            /**
             * Adds a body to a system to be checked for collisions.
             *
             * @brief Adds a body to a system to be checked for collisions.
             *
             * @param name The name of the rigid body to be added.
             * @param geometry The geometry of the rigid body to be added.
             * @param plant The pointer of the plant that the body belongs.
             * @param is_obstacle If the body belongs to an obstacle or not.
             */
            void add_body(const std::string& name, const util::geometry_t& geometry, system_t* plant, bool is_obstacle = false);

            /**
             * Removes a rigid body from the collision checker such as it will not spend time to
             * check for collisions for this body.
             *
             * @brief Removes a rigid body from the collision checker.
             *
             * @param name The name of the rigid body that needs to be removed from the collision list.
             */
            void remove_body(const std::string& name);

            /**
             * Sets the white list of pairs of systems to check for collisions.
             *
             * @brief Sets the white list of pairs of systems to check for collisions.
             *
             * @list The new white list of pairs that need to be checked for collisions.
             */
            void link_collision_list(collision_list_t* list);

            /**
             * Checks if there is at least one collision between the systems.
             *
             * @brief Checks if there is at least one collision between the systems.
             *
             * @return True if there is at least one collision. \n
             *         False if there is no collision during this simulation step.
             */
            bool in_collision();

            /**
             * Checks if there is at least one collision between the systems in
             * the provided collision list.
             *
             * @brief Checks for collisions in the provided collision list.
             *
             * @return True if there is a collision in the list. \n
             *         False if there is no such collision in the list.
             */
            virtual bool in_collision( collision_list_t* list );

            /**
             * Checks to make sure all of the systems are close to the obstacles of the
             * scene.
             *
             * @brief Checks if there is any near-collision.
             *
             * @param eps The clearance to use for near checking.
             * @return True if it finds a near-collision. \n
             *         False if there is no near-collision between the systems.
             */
            virtual bool near_collision( double eps );


            /**
             * Get the distance between two systems.
             *
             * @brief Get the distance between two systems.
             *
             * @return The distance
             */
            virtual double get_clearance();

            /**
             * Checks for collisions between the systems. Will return a list with all the
             * collided bodies during this simulation step.
             *
             * @brief Will return a list with all the collided bodies.
             *
             * @return The list with the pairs for all the systems that they are in collision.
             */
            collision_list_t* colliding_bodies();

            /**
             * Returns a list with all the pairs of bodes which are also in the
             * provided input list.
             */
            collision_list_t* colliding_bodies( collision_list_t* list );

            /**
             * Checks for near-collisions between the systems, returning a list of
             * all pairs of bodies which are within eps distance of each other.
             *
             * @brief Get a list of nearly-colliding bodies.
             *
             * @param eps The clearance to do near-checking with.
             * @return A list of nearly-colliding bodies.
             */
            virtual collision_list_t* near_colliding_bodies( double eps );

            virtual collision_checker_info_t* get_collision_info(const std::string& name);

            virtual bool info_in_plants();

          protected:

            /**
             * A map between the name of the systems and the pqp models.
             * @brief A map between the name of the systems and the pqp models.
             */
            std::map<std::string, parallel_pqp_info_t*> models_map;

            /**
             * The iterator for the \c models_map.
             * @brief The iterator for the \c models_map.
             */
            std::map<std::string, parallel_pqp_info_t*>::iterator models_map_iter;

            /**
             * The list of the systems that will be checked for collisions.
             * @brief The list of the systems that will be checked for collisions.
             */
            std::vector<std::pair<parallel_pqp_info_t*, parallel_pqp_info_t*> > body_cache;


          private:

            /**
             * Using information from the given parameters, creates a pqp model that is
             * mandatory for the pqp library to checkes for collisions.
             *
             * @brief Creates a pqp model given some information.
             *
             * @param name The name of the system.
             * @param geometry The geometry of the system.
             */
            void create_model(const std::string& name, const util::geometry_t& geometry);

            /**
             * Checks if the two systems are in collision or not.
             *
             * @brief Checks if the two systems are in collision or not.
             *
             * @param name1 The pqp information for the first system.
             * @param name2 The pqp information for the second system.
             *
             * @return True if the systems are in collision. \n
             *         False if they are not in collision.
             */
            bool in_collision(parallel_pqp_info_t* name1, parallel_pqp_info_t* name2);

            /**
             * Checks if the two systems are in collision or not.
             *
             * @brief Checks if the two systems are in collision or not.
             *
             * @param name1 The full slash-delimited path for the first system.
             * @param name2 The full slash-delimited path for the second system.
             *
             * @return True if the systems are in collision. \n
             *         False if they are not in collision.
             */
            bool in_collision(const std::string& name1, const std::string& name2);

            /**
             * Internal near_collision check call.
             *
             * @brief Checks if the two systems are near-collision or not.
             *
             * @param name1 The pqp information for the first system.
             * @param name2 The pqp information for the second system.
             *
             * @return True if the systems are near-collision. \n
             *         False if they are not near-collision.
             */
            bool near_collision( double eps, parallel_pqp_info_t* info1, parallel_pqp_info_t* info2 );

            /**
             * Internal distance query.
             *
             * @brief Checks the distance between two systems.
             *
             * @param min_so_far The minimum distance found so far. Used for optimization.
             * @param info1 The pqp information for the first system.
             * @param info2 The pqp information for the second system.
             *
             * @return The distance.
             */
            double get_clearance(double min_so_far, parallel_pqp_info_t* info1, parallel_pqp_info_t* info2 );

	    static collision::CUDACollisionCheck CUDACollisionChecker;
        };

        std::ostream& operator<<(std::ostream& output, const parallel_collision_checker_t& checker);


    }
}

#endif


#else

namespace prx
{
    namespace sim
    {

        class parallel_collision_checker_t : public collision_checker_t
        {
            virtual void set_configuration(const std::string& name, const util::config_t& config) {}
            virtual collision_checker_info_t* get_collision_info(const std::string& name) {}
            virtual void add_body(const std::string& name, const util::geometry_t& geometry, system_t* plant, bool is_obstacle = false) {}
            virtual void remove_body(const std::string& name) {}
            virtual bool in_collision() {}
            virtual collision_list_t* colliding_bodies() {}
        };

    }
}



#endif
