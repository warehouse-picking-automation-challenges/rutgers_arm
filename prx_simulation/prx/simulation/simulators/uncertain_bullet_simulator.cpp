/**
 * @file uncertain_bullet_simulator.cpp 
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

#include "prx/simulation/simulators/uncertain_bullet_simulator.hpp"
#include "prx/simulation/systems/plants/bullet/bullet_body_info.hpp"
#include "prx/simulation/collision_checking/bullet_collision_checker.hpp"
#include <ros/param.h>

#include <pluginlib/class_list_macros.h>
#include <boost/range/adaptor/map.hpp> //adaptors
#include <stdio.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::uncertain_bullet_simulator_t, prx::sim::simulator_t)

namespace prx
{
    using namespace util;

    namespace sim
    {

        uncertain_bullet_simulator_t::uncertain_bullet_simulator_t() : simulator_t() 
        {
        }

        uncertain_bullet_simulator_t::~uncertain_bullet_simulator_t()
        {
            dynamicsWorld->removeRigidBody(ground_plane);
            delete ground_plane->getMotionState();
            delete ground_plane;
            delete ground_shape;

            for( unsigned i = 0; i < rigid_bodies.size(); i++ )
            {
                dynamicsWorld->removeRigidBody(rigid_bodies[i].second);
                delete rigid_bodies[i].second->getMotionState();
                delete rigid_bodies[i].second;
                delete rigid_bodies[i].first;
            }
            rigid_bodies.clear();

            delete dynamicsWorld;
            delete solver;
            delete dispatcher;
            delete collisionConfiguration;
            delete broadphase;
        }

        void uncertain_bullet_simulator_t::add_system(const std::string& path, system_ptr_t system)
        {
            PRX_ERROR_S("Add a system is not supported by bullet simulator!");
        }

        void uncertain_bullet_simulator_t::init(const parameter_reader_t * const reader, const parameter_reader_t * const template_reader)
        {
            simulator_t::init(reader, template_reader);
            inner_iterations = parameters::get_attribute_as<int>("iterations", reader, template_reader, 50);
            gravity = parameters::get_attribute_as<std::vector<double> >("gravity", reader, template_reader);


            rigid_bodies.clear();
            broadphase = new btAxisSweep3(btVector3(-1000,-1000,-1000),btVector3(1000,1000,1000));
            collisionConfiguration = new btDefaultCollisionConfiguration();
            dispatcher = new btCollisionDispatcher(collisionConfiguration);
            solver = new btSequentialImpulseConstraintSolver();
            // solver = new btMLCPSolver(&mlcp);
            dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
            // PRX_INFO_S("GRAVITY: "<<gravity[0]<<" "<<gravity[1]<<" "<<gravity[2]);
            dynamicsWorld->setGravity(btVector3(gravity[0], gravity[1], gravity[2]));



            heightmap = false;
            if(parameters::has_attribute("heightmap", reader, template_reader))
            {
                heightmap = parameters::get_attribute_as<bool>("heightmap", reader, template_reader, false);
            }
            if(!heightmap)
            {
                bullet_body_info_t* info = new bullet_body_info_t();
                info->name = "ground_plane";
                ground_shape = new btStaticPlaneShape(btVector3(0, 0, 1), (btScalar)1);
                ground_shape->setUserPointer(info);

                // btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,-1)));
                btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, NULL, ground_shape, btVector3(0, 0, 0));
                ground_plane = new btRigidBody(groundRigidBodyCI);

                ground_plane->setFriction(1.0);
                ground_plane->setRollingFriction(.01);
                ground_plane->setRestitution(.2);
                btTransform trans(btQuaternion(0, 0, 0, 1),
                                  btVector3(0, 0, -1));
                ground_plane->proceedToTransform(trans);
                ground_plane->setUserPointer(info);
                dynamicsWorld->addRigidBody(ground_plane);
            }


            // Now that all of the subsystems have been created, we need to get all plant pointers
            add_bodies_from_plants();
            PRX_PRINT(state_space->print_memory(), PRX_TEXT_CYAN);
        }

        void uncertain_bullet_simulator_t::propagate(const double sim_step)
        {
            broadphase->resetPool(dispatcher);
            solver->reset();
            if(!heightmap)
                dynamicsWorld->removeRigidBody(ground_plane);
            for( unsigned i = 0; i < rigid_bodies.size(); i++ )
            {
                dynamicsWorld->removeRigidBody(rigid_bodies[i].second);
            }
            // delete dynamicsWorld;
            // dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

            dynamicsWorld->getSolverInfo().m_solverMode = 0;
            // dynamicsWorld->getSolverInfo().m_globalCfm = 0.02;
            // dynamicsWorld->getSolverInfo().m_erp = 0.2;
            dynamicsWorld->getSolverInfo().m_numIterations = inner_iterations;
            dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 1;
            // dynamicsWorld->getSolverInfo().m_splitImpulse = true;
            // dynamicsWorld->getSolverInfo().m_splitImpulsePenetrationThreshold = -0.02;

            dynamicsWorld->setGravity(btVector3(gravity[0], gravity[1], gravity[2]));
            if(!heightmap)
                dynamicsWorld->addRigidBody(ground_plane);
            for( unsigned i = 0; i < rigid_bodies.size(); i++ )
            {
                rigid_bodies[i].second->clearForces();
                dynamicsWorld->addRigidBody(rigid_bodies[i].second);
            }

            // foreach(bullet_plant_t* plant, plants)
            // {
            //     plant->add_constraints(dynamicsWorld);
            // }

            simulator_t::propagate(simulation::simulation_step);

            dynamicsWorld->stepSimulation(simulation::simulation_step, 0, simulation::simulation_step);
            // collision_checker->in_collision();

            foreach(bullet_plant_t* plant, this->plants)
            {
                plant->update_from_bullet();
            }
        }

        bool uncertain_bullet_simulator_t::in_collision()
        {
            int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
            bullet_collision_checker_t* checker = dynamic_cast<bullet_collision_checker_t*>(collision_checker);
            for (int i=0;i<numManifolds;i++)
            {
                btPersistentManifold* contactManifold =  dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
                btRigidBody* obA = (btRigidBody*)const_cast<btCollisionObject*>(contactManifold->getBody0());
                btRigidBody* obB = (btRigidBody*)const_cast<btCollisionObject*>(contactManifold->getBody1());
                int iter_size = collision_bodies_list.size();
                for(int j=0;j<iter_size;j++)
                {
                    if( (collision_bodies_list[j].first == obA && collision_bodies_list[j].second == obB) ||
                        (collision_bodies_list[j].first == obB && collision_bodies_list[j].second == obA))
                        return true;
                }
            }
            return false;

            // return collision_checker->in_collision(config_list);
        }

        void uncertain_bullet_simulator_t::link_collision_list(collision_list_t* collision_list)
        {
            collision_checker->link_collision_list(collision_list);
            //try to get the pointers to the bodies instead of the string identifiers

            foreach(collision_pair_t pair, collision_list->get_body_pairs())
            {
                std::string name1 = pair.first;
                std::string name2 = pair.second;
                PRX_INFO_S(name1<<" --- "<<name2);
                unsigned i1,i2;
                for( i1 = 0; i1 < rigid_bodies.size(); i1++ )
                {   
                    std::string rb_name = ((bullet_body_info_t*)rigid_bodies[i1].first->getUserPointer())->name;
                    if(name1==rb_name)
                        break;
                }
                for( i2 = 0; i2 < rigid_bodies.size(); i2++ )
                {   
                    std::string rb_name = ((bullet_body_info_t*)rigid_bodies[i2].first->getUserPointer())->name;
                    if(name2==rb_name)
                        break;
                }
                collision_bodies_list.push_back(std::pair< btRigidBody*, btRigidBody*>(
                                            rigid_bodies[i1].second,rigid_bodies[i2].second));
            }
        }

        void uncertain_bullet_simulator_t::propagate_and_respond()
        {

            propagate(simulation::simulation_step);
        }

        void uncertain_bullet_simulator_t::remove_system(const std::string& path)
        {
            PRX_DEBUG_COLOR("DON'T EVEN TRY", PRX_TEXT_CYAN);
        }

        void uncertain_bullet_simulator_t::replace_system(const std::string& path, system_ptr_t system)
        {
            PRX_DEBUG_COLOR("DON'T EVEN TRY", PRX_TEXT_CYAN);
        }

        void uncertain_bullet_simulator_t::push_state(const state_t * const source)
        {
            state_space->copy_from_point(source);

            foreach(bullet_plant_t* plant, this->plants)
            {
                plant->update_to_bullet();
            }
        }

        state_t* uncertain_bullet_simulator_t::pull_state()
        {
            return simulator_t::pull_state();
        }

        void uncertain_bullet_simulator_t::add_bodies_from_plants()
        {
            system_graph_t sys_graph;
            update_system_graph(sys_graph);
            std::vector<plant_t*> reg_plants;
            sys_graph.get_plants(reg_plants);

            plants.clear();

            foreach(plant_t* plant, reg_plants)
            {
                plants.push_back(checked_cast< bullet_plant_t* >(plant));
                plants.back()->add_bodies(rigid_bodies);
                plants.back()->add_constraints(dynamicsWorld);
                plants.back()->update_from_bullet();

            }
            for( unsigned i = 0; i < rigid_bodies.size(); i++ )
            {
                dynamicsWorld->addRigidBody(rigid_bodies[i].second);
            }

            foreach(bullet_plant_t* plant, plants)
            {
                plant->add_constraints(dynamicsWorld);
            }

            foreach(std::string name, obstacles | boost::adaptors::map_keys)
            {
                bullet_obstacles.push_back((bullet_obstacle_t*)obstacles[name].get());
                bullet_obstacles.back()->add_bodies(rigid_bodies);
            }

        }


    }
}
#endif