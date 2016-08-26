/**
 * @file bullet_simulator.cpp 
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

#include "prx/simulation/simulators/bullet_simulator.hpp"
#include "prx/simulation/systems/plants/bullet/bullet_body_info.hpp"
#include "prx/simulation/collision_checking/bullet_collision_checker.hpp"
#include <ros/param.h>

#include <pluginlib/class_list_macros.h>
#include <boost/range/adaptor/map.hpp> //adaptors
#include <stdio.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::bullet_simulator_t, prx::sim::simulator_t)

namespace prx
{
    using namespace util;

    namespace sim
    {

        bullet_simulator_t::bullet_simulator_t() : simulator_t() 
        {
        }

        bullet_simulator_t::~bullet_simulator_t()
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

        void bullet_simulator_t::add_system(const std::string& path, system_ptr_t system)
        {
            PRX_ERROR_S("Add a system is not supported by bullet simulator!");
        }

        void bullet_simulator_t::init(const parameter_reader_t * const reader, const parameter_reader_t * const template_reader)
        {
            simulator_t::init(reader, template_reader);
            inner_iterations = parameters::get_attribute_as<int>("iterations", reader, template_reader, 50);
            gravity = parameters::get_attribute_as<std::vector<double> >("gravity", reader, template_reader);


            rigid_bodies.clear();
            broadphase = new btDbvtBroadphase();//new btAxisSweep3(btVector3(-1000,-1000,-1000),btVector3(1000,1000,1000));
            collisionConfiguration = new btDefaultCollisionConfiguration();
            dispatcher = new btCollisionDispatcher(collisionConfiguration);
            dispatcher->setDispatcherFlags(btCollisionDispatcher::CD_STATIC_STATIC_REPORTED);
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
                ground_plane->setRollingFriction(.30);
                ground_plane->setRestitution(.2);
                btTransform trans(btQuaternion(0, 0, 0, 1),
                                  btVector3(0, 0, -1));
                ground_plane->proceedToTransform(trans);
                ground_plane->setUserPointer(info);
                dynamicsWorld->addRigidBody(ground_plane);
            }


            // Now that all of the subsystems have been created, we need to get all plant pointers
            add_bodies_from_plants();
            // PRX_DEBUG_COLOR(state_space->print_memory(), PRX_TEXT_CYAN);
        }

        void bullet_simulator_t::propagate(const double sim_step)
        {
            broadphase->resetPool(dispatcher);
            solver->reset();
            if(!heightmap)
                dynamicsWorld->removeRigidBody(ground_plane);
            for( unsigned i = 0; i < rigid_bodies.size(); i++ )
            {
                dynamicsWorld->removeRigidBody(rigid_bodies[i].second);
            }
            delete dynamicsWorld;
            dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

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
            for( unsigned i = 0; i < kinematic_rigid_bodies.size(); i++ )
            {
                dynamicsWorld->addRigidBody(kinematic_rigid_bodies[i].second);
            }

            foreach(bullet_plant_t* plant, plants)
            {
                plant->add_constraints(dynamicsWorld);
            }

            simulator_t::propagate(simulation::simulation_step);
            
            unsigned index = 0;
            foreach(plant_t* plant, kinematic_plants)
            {
                plant->update_phys_configs(kinematic_configurations,index);
            }

            vector_t pos(3);
            quaternion_t orient;
            for( unsigned i = 0; i < kinematic_rigid_bodies.size(); i++ )
            {
                kinematic_configurations[i].second.get(pos,orient);
                btTransform trans( btQuaternion(orient.get_x(), orient.get_y(), orient.get_z(), orient.get_w()),
                                   btVector3(pos[0], pos[1], pos[2]));
                kinematic_rigid_bodies[i].second->setCenterOfMassTransform(trans);
            }

            dynamicsWorld->stepSimulation(simulation::simulation_step, 0, simulation::simulation_step);
            // collision_checker->in_collision();

            foreach(bullet_plant_t* plant, this->plants)
            {
                plant->update_from_bullet();
            }

            // checked_cast< ode_collision_checker_t* >(collision_checker)->clear_collisions();
        }

        struct prx_contact_t : public btCollisionWorld::ContactResultCallback
        {
            btScalar addSingleResult(btManifoldPoint& cp,
                const btCollisionObjectWrapper* colObj0Wrap,
                int partId0,
                int index0,
                const btCollisionObjectWrapper* colObj1Wrap,
                int partId1,
                int index1)
            {
                found_collision = true;
            }
            bool found_collision;
        };

        bool bullet_simulator_t::in_collision()
        {
            int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
            // bullet_collision_checker_t* checker = dynamic_cast<bullet_collision_checker_t*>(collision_checker);
            for (int i=0;i<numManifolds;i++)
            {
                btPersistentManifold* contactManifold =  dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
                btRigidBody* obA = (btRigidBody*)const_cast<btCollisionObject*>(contactManifold->getBody0());
                btRigidBody* obB = (btRigidBody*)const_cast<btCollisionObject*>(contactManifold->getBody1());
                int iter_size = collision_bodies_list.size();
                // PRX_INFO_S(((bullet_body_info_t*)obA->getUserPointer())->name<<" "<<((bullet_body_info_t*)obB->getUserPointer())->name);
                for(int j=0;j<iter_size;j++)
                {
                    if( (collision_bodies_list[j].first == obA && collision_bodies_list[j].second == obB) ||
                        (collision_bodies_list[j].first == obB && collision_bodies_list[j].second == obA))
                    {
                        return true;
                    }
                }

                // checker->add_temporal_collision(((bullet_body_info_t*)obA->getUserPointer())->name,((bullet_body_info_t*)obB->getUserPointer())->name ); 
            }

            int iter_size = manual_collision_bodies_list.size();
            prx_contact_t contact;
            contact.found_collision = false;
            for(int j=0;j<iter_size;j++)
            {
                dynamicsWorld->contactPairTest(manual_collision_bodies_list[j].first,manual_collision_bodies_list[j].second,contact);
                // PRX_INFO_S(((bullet_body_info_t*)manual_collision_bodies_list[j].first->getUserPointer())->name<<" "<<((bullet_body_info_t*)manual_collision_bodies_list[j].second->getUserPointer())->name);
                if( contact.found_collision )
                {
                    return true;
                }
            }

            return false;

            // return collision_checker->in_collision(config_list);
        }

        void bullet_simulator_t::link_collision_list(collision_list_t* collision_list)
        {
            collision_checker->link_collision_list(collision_list);
            //try to get the pointers to the bodies instead of the string identifiers

            foreach(collision_pair_t pair, collision_list->get_body_pairs())
            {
                std::string name1 = pair.first;
                std::string name2 = pair.second;
                // PRX_INFO_S(name1<<" --- "<<name2);
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
                if(rigid_bodies.size()!=0 && i1 < rigid_bodies.size() && i2 < rigid_bodies.size())
                {
                    collision_bodies_list.push_back(std::pair< btRigidBody*, btRigidBody*>(
                                                rigid_bodies[i1].second,rigid_bodies[i2].second));
                }
                else
                {
                    for( i1 = 0; i1 < kinematic_rigid_bodies.size(); i1++ )
                    {   
                        std::string rb_name = ((bullet_body_info_t*)kinematic_rigid_bodies[i1].first->getUserPointer())->name;
                        if(name1==rb_name)
                            break;
                    }
                    for( i2 = 0; i2 < kinematic_rigid_bodies.size(); i2++ )
                    {   
                        std::string rb_name = ((bullet_body_info_t*)kinematic_rigid_bodies[i2].first->getUserPointer())->name;
                        if(name2==rb_name)
                            break;
                    }
                    if(kinematic_rigid_bodies.size()!=0 && i1 < kinematic_rigid_bodies.size() && i2 < kinematic_rigid_bodies.size())
                    {
                        manual_collision_bodies_list.push_back(std::pair< btRigidBody*, btRigidBody*>(
                                                    kinematic_rigid_bodies[i1].second,kinematic_rigid_bodies[i2].second));
                    }

                }
            }
            std::vector<std::pair<btCollisionShape*, btRigidBody*> > temp_rigid_bodies;

            foreach(bullet_obstacle_t* obs, bullet_obstacles)
            {
                obs->add_bodies(temp_rigid_bodies);
            }

            for( unsigned i1 = 0; i1 < kinematic_rigid_bodies.size(); i1++ )
            {   
                    for( unsigned i2 = 0; i2 < temp_rigid_bodies.size(); i2++ )
                    {   
                        manual_collision_bodies_list.push_back(std::pair< btRigidBody*, btRigidBody*>(
                                                    kinematic_rigid_bodies[i1].second,temp_rigid_bodies[i2].second));
                    }
            }
        }

        void bullet_simulator_t::propagate_and_respond()
        {
            propagate(simulation::simulation_step);
            // if(in_collision())
            // {
            //     PRX_INFO_S("Hello");
            // }
        }

        void bullet_simulator_t::remove_system(const std::string& path)
        {
            PRX_DEBUG_COLOR("DON'T EVEN TRY", PRX_TEXT_CYAN);
        }

        void bullet_simulator_t::replace_system(const std::string& path, system_ptr_t system)
        {
            PRX_DEBUG_COLOR("DON'T EVEN TRY", PRX_TEXT_CYAN);
        }

        void bullet_simulator_t::push_state(const state_t * const source)
        {
            state_space->copy_from_point(source);

            foreach(bullet_plant_t* plant, this->plants)
            {
                plant->update_to_bullet();
            }
        }

        state_t* bullet_simulator_t::pull_state()
        {
            return simulator_t::pull_state();
        }

        void bullet_simulator_t::add_bodies_from_plants()
        {
            system_graph_t sys_graph;
            update_system_graph(sys_graph);
            std::vector<plant_t*> reg_plants;
            sys_graph.get_plants(reg_plants);

            plants.clear();
            unsigned index=0;
            foreach(plant_t* plant, reg_plants)
            {
                if(dynamic_cast<bullet_plant_t*>(plant)!=NULL)
                {
                    plants.push_back(checked_cast< bullet_plant_t* >(plant));
                    plants.back()->add_bodies(rigid_bodies);
                    plants.back()->add_constraints(dynamicsWorld);
                    plants.back()->update_from_bullet();
                }
                else
                {
                    kinematic_plants.push_back(plant);
                    plant->update_phys_configs(kinematic_configurations,index);
                    create_rigid_bodies(plant);
                }

            }
            for( unsigned i = 0; i < rigid_bodies.size(); i++ )
            {
                dynamicsWorld->addRigidBody(rigid_bodies[i].second);
            }
            for( unsigned i = 0; i < kinematic_rigid_bodies.size(); i++ )
            {
                dynamicsWorld->addRigidBody(kinematic_rigid_bodies[i].second);
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
        void bullet_simulator_t::create_rigid_bodies(plant_t* plant)
        {
            util::geom_map_t geom_map;
            plant->update_phys_geoms(geom_map);

            unsigned index=0;
            config_list_t c_list;
            plant->update_phys_configs(c_list,index);
            // std::string geom_name;

            config_t conf;
            vector_t pos(3);
            quaternion_t orient;

            foreach(const config_list_element_t& element, c_list)
            {
                std::string geom_name = element.first;
                bullet_body_info_t* info = new bullet_body_info_t();
                // PRX_INFO_S(geom_name);
                info->name = geom_name;
                info->geom = &geom_map[geom_name];
                info->plant = plant;
                element.second.get(pos,orient);

                //create the rigid body in Bullet
                btCollisionShape* collision_shape;

                if( info->geom->get_type() == PRX_SPHERE )
                {
                    double radius;
                    info->geom->get_sphere(radius);
                    collision_shape = new btSphereShape(radius);
                }
                else if( info->geom->get_type() == PRX_BOX )
                {
                    double lx, ly, lz;
                    info->geom->get_box(lx, ly, lz);
                    collision_shape = new btBoxShape(btVector3(lx / 2, ly / 2, lz / 2));
                }
                else if( info->geom->get_type() == PRX_CYLINDER )
                {
                    double rad, height;
                    info->geom->get_cylinder(rad, height);
                    collision_shape = new btCylinderShapeZ(btVector3(rad, rad, height / 2));
                }
                else if( info->geom->get_type() == PRX_CAPSULE )
                {
                    double rad, height;
                    info->geom->get_capsule(rad, height);
                    collision_shape = new btCapsuleShapeZ(rad, height);
                }
                else
                {
                    const trimesh_t* mesh = info->geom->get_trimesh();
                    int num_faces = mesh->get_faces_size();
                    int num_vertices = mesh->get_vertices_size();
                    info->faces = new int[num_faces*3];
                    info->vertex_array = new double[num_vertices*3];

                    const std::vector<face_t>& faces = mesh->get_faces();
                    const std::vector<vector_t>& vertices = mesh->get_vertices();
                    for(int i=0;i<num_faces;i++)
                    {
                        info->faces[i*3+0] = faces[i].get_index1();
                        info->faces[i*3+1] = faces[i].get_index2();
                        info->faces[i*3+2] = faces[i].get_index3();
                    }
                    for(int i=0;i<num_vertices;i++)
                    {
                        info->vertex_array[i*3+0] = vertices[i][0];
                        info->vertex_array[i*3+1] = vertices[i][1];
                        info->vertex_array[i*3+2] = vertices[i][2];
                    }
                    btTriangleIndexVertexArray* m = new btTriangleIndexVertexArray(num_faces,info->faces,3*sizeof(int),num_vertices,info->vertex_array,3*sizeof(double));
                    collision_shape = new btBvhTriangleMeshShape(m,true);
                }
                btDefaultMotionState* motion_state = new btDefaultMotionState(btTransform(
                                                              btQuaternion(orient.get_x(), orient.get_y(), orient.get_z(), orient.get_w()),
                                                              btVector3(pos[0], pos[1], pos[2])));
                collision_shape->setUserPointer(info);
                btRigidBody::btRigidBodyConstructionInfo construction_info(0, motion_state, collision_shape, btVector3(0, 0, 0));
                btRigidBody* rigid_body = new btRigidBody(construction_info);

                rigid_body->setFriction(1.0);
                rigid_body->setRollingFriction(.01);
                rigid_body->setRestitution(.2);
                rigid_body->setUserPointer(info);
                rigid_body->forceActivationState(DISABLE_DEACTIVATION);
                // rigid_body->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);
                kinematic_rigid_bodies.push_back(std::pair<btCollisionShape*, btRigidBody*>(collision_shape, rigid_body));
            }
        }


    }
}
#endif