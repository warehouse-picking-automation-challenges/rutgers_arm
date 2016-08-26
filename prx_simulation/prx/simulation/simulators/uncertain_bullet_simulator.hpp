/**
 * @file uncertain_bullet_simulator.hpp 
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#ifdef BULLET_FOUND

#ifndef BT_USE_DOUBLE_PRECISION
#define BT_USE_DOUBLE_PRECISION
#endif

#pragma once

#ifndef PRX_UNCERTAIN_BULLET_SIMULATOR_HPP
#define PRX_UNCERTAIN_BULLET_SIMULATOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/simulation/simulators/simulator.hpp"
// #include "prx/simulation/collision_checking/ode_collision_checker.hpp"
#include "prx/simulation/systems/plants/bullet/bullet_plant.hpp"
#include "prx/simulation/systems/plants/bullet/bullet_obstacle.hpp"

#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"



namespace prx
{

    namespace sim
    {

        class uncertain_bullet_simulator_t : public simulator_t
        {

          public:
            uncertain_bullet_simulator_t();
            ~uncertain_bullet_simulator_t();
            void add_system(const std::string& path, system_ptr_t system);
            void init(const util::parameter_reader_t * const reader, const util::parameter_reader_t* template_reader = NULL);
            void propagate(const double simulation_step = 0);
            virtual void propagate_and_respond();
            void remove_system(const std::string& path);
            void replace_system(const std::string& path, system_ptr_t system);

            virtual void push_state(const state_t * const source);
            virtual state_t* pull_state();
            
            virtual bool in_collision();

            virtual void link_collision_list(collision_list_t* collision_list);
            virtual bool internal_state_push()
            {
                  return true;
            }

          private:
            bool heightmap;
            std::vector<double> gravity;
            int inner_iterations;
            btBroadphaseInterface* broadphase;
            btDefaultCollisionConfiguration* collisionConfiguration;
            btCollisionDispatcher* dispatcher;
            btSequentialImpulseConstraintSolver* solver;
            btDynamicsWorld* dynamicsWorld;
            // btMLCPSolver* solver;
            // btSolveProjectedGaussSeidel mlcp;

            btCollisionShape* ground_shape;
            btRigidBody* ground_plane;

            std::vector<std::pair<btCollisionShape*, btRigidBody*> > rigid_bodies;
            std::vector<std::pair< btRigidBody*, btRigidBody*> > collision_bodies_list;

            void add_bodies_from_plants();

            std::vector< bullet_plant_t* > plants;
            std::vector< plant_t* > kinematic_plants;
            std::vector< bullet_obstacle_t* > bullet_obstacles;
        };
    }
}
#endif
#endif

