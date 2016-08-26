/**
 * @file ntrt_simulator.cpp 
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

#include "simulation/ntrt_simulator.hpp"
#include "prx/simulation/systems/plants/bullet/bullet_body_info.hpp"
#include "prx/simulation/systems/plants/bullet/bullet_obstacle.hpp"
#include "prx/simulation/collision_checking/bullet_collision_checker.hpp"
#include "prx/utilities/definitions/random.hpp"


#include "core/terrain/tgBoxGround.h"
#include "core/terrain/tgPlaneGround.h"
#include "core/tgModel.h"
#include "core/tgWorld.h"
#include "core/tgWorldBulletPhysicsImpl.h"


#include <ros/param.h>

#include <pluginlib/class_list_macros.h>
#include <boost/range/adaptor/map.hpp> //adaptors
#include <stdio.h>
 #include <fstream>

PLUGINLIB_EXPORT_CLASS(prx::sim::ntrt_simulator_t, prx::sim::simulator_t)

namespace prx
{
    using namespace util;

    namespace sim
    {

        tgWorld* ntrt_world;


        class prxTerrain : public tgBulletGround
        {
            public:

                prxTerrain()
                {
                }

                virtual ~prxTerrain()
                {

                }

                virtual btRigidBody* getGroundRigidBody() const
                {
                
                    btDefaultMotionState* motion_state = new btDefaultMotionState(*(((bullet_obstacle_t*)obstacle.get())->transform));
                    btRigidBody::btRigidBodyConstructionInfo construction_info(0, motion_state, pGroundShape, btVector3(0, 0, 0));
                    btRigidBody* rigid_body = new btRigidBody(construction_info);
                    
                    rigid_body->setFriction(1.0);
                    rigid_body->setRollingFriction(.1);
                    rigid_body->setRestitution(.2);
                    rigid_body->forceActivationState(DISABLE_DEACTIVATION);
                    return rigid_body;
                }

                void setCollisionBody(btCollisionShape* s)
                {
                    pGroundShape = s;
                }

                system_ptr_t obstacle;
        };

        ntrt_simulator_t::ntrt_simulator_t() : simulator_t() 
        {
        }

        ntrt_simulator_t::~ntrt_simulator_t()
        {
            // dynamicsWorld->removeRigidBody(ground_plane);
            // delete ground_plane->getMotionState();
            // delete ground_plane;
            // delete ground_shape;

            // for( unsigned i = 0; i < rigid_bodies.size(); i++ )
            // {
            //     dynamicsWorld->removeRigidBody(rigid_bodies[i].second);
            //     delete rigid_bodies[i].second->getMotionState();
            //     delete rigid_bodies[i].second;
            //     delete rigid_bodies[i].first;
            // }
            // rigid_bodies.clear();

            // delete dynamicsWorld;
            // delete solver;
            // delete dispatcher;
            // delete collisionConfiguration;
            // delete broadphase;
        }

        void ntrt_simulator_t::add_system(const std::string& path, system_ptr_t system)
        {
            PRX_ERROR_S("Add a system is not supported by bullet simulator!");
        }

        void ntrt_simulator_t::init(const parameter_reader_t * const reader, const parameter_reader_t * const template_reader)
        {

            // parameter_reader_t::reader_map_t obstacle_map;
            // bool got_obstacle_map = false;
            // if (parameters::has_attribute("obstacles", reader, template_reader) )
            // {
            //     obstacle_map = parameters::get_map("obstacles", reader, template_reader);
            //     got_obstacle_map = true;
            // }

            // if( got_obstacle_map)
            // {
            //     PRX_INFO_S("Terrain");
            //     prxTerrain* terrain = new prxTerrain();
            //     foreach(const parameter_reader_t::reader_map_t::value_type key_value, obstacle_map)
            //     {
            //         const parameter_reader_t* obstacle_template_reader = NULL;

            //         if( key_value.second->has_attribute("template") )
            //         {
            //             std::string template_name = key_value.second->get_attribute("template");
            //             // TODO: Find a way to load templates under namespaces more cleanly.
            //             obstacle_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + template_name, global_storage);

            //         }

            //         std::string obstacle_name = pathname + "/obstacles/" + key_value.first;
            //         system_ptr_t obs = create_subsystem(obstacle_name, key_value.second, obstacle_template_reader);
            //         delete obstacle_template_reader;
            //         std::vector<std::pair<btCollisionShape*, btRigidBody*> > global_list;
            //         ((bullet_obstacle_t*)obs.get())->add_bodies(global_list);
            //         terrain->obstacle = obs;
            //         terrain->setCollisionBody(global_list.front().first);
            //     }
            //     ground = terrain;
            // }
            // else
            // {
                PRX_INFO_S("Plane");
                const tgPlaneGround::Config groundConfig(btVector3(0.0,1,0.0));
                ground = new tgPlaneGround(groundConfig);

            // }
            const tgWorld::Config config(98.1);
            world = new tgWorld(config, ground);
            ntrt_world = world;

            simulator_t::init(reader,template_reader);

            system_graph_t sys_graph;
            update_system_graph(sys_graph);
            std::vector<plant_t*> reg_plants;
            sys_graph.get_plants(reg_plants);

            plants.clear();


            foreach(plant_t* plant, reg_plants)
            {
                plants.push_back(checked_cast< ntrt_plant_t* >(plant));
                plant_control_space = checked_cast< ntrt_plant_t* >(plant)->get_control_space();
                plant_state_space = checked_cast< ntrt_plant_t* >(plant)->get_state_space();
            }
            prev_state = state_space->alloc_point();
            curr_state = state_space->alloc_point();
            prev_control = plant_control_space->alloc_point();
            curr_control = plant_control_space->alloc_point();

            do_simulated_particles = parameters::get_attribute_as<bool>("simulated_particles",reader,template_reader,false);
            if(do_simulated_particles)
            {
                stored_states.resize(50);
                std::ifstream fin;
                fin.open("/home/zak/repos/pracsys_ws/src/pracsys/prx_output/reduction.txt");
                for(int i=0;i<50;i++)
                {
                    stored_states[i] = plant_state_space->deserialize_point(fin);
                }
                fin.close();
                current_time=0;
                int index = 0;
                foreach(ntrt_plant_t* plant, plants)
                {
                    plant->get_state_space()->copy_from_point(stored_states[index]);
                    plant->update_to_bullet();
                    index++;
                }

            }


        }

        void ntrt_simulator_t::propagate(const double sim_step)
        {
            // plant_control_space->copy_to_point(curr_control);
            // state_space->copy_to_point(curr_state);
            // if(!plant_control_space->equal_points(prev_control,curr_control,1e-12) || !state_space->equal_points(prev_state,curr_state,1e-12) )
            // {
            //     PRX_INFO_S("Change: "<<state_space->equal_points(prev_state,curr_state,1e-12));
            
                foreach(ntrt_plant_t* plant, this->plants)
                {
                    plant->teardown();
                }
                world->reset();
                
                foreach(ntrt_plant_t* plant, this->plants)
                {
                    plant->add_bodies();
                }

                if(do_simulated_particles)
                {
                    // if(current_time==0)
                    // {
                    //     PRX_INFO_S("new start");
                    //     state_space->copy_from_point(stored_states[uniform_int_random(0,24)]);
                    //     for(int i=0;i<plant_control_space->get_dimension();i++)
                    //     {
                    //         curr_control->at(i) = stored_plan.back().control->at(i)+uniform_random(-.5,.5)*.05;
                    //     }
                    //     push_control(curr_control);
                    // }
                    // current_time+=simulation::simulation_step;
                    // if(current_time >= stored_plan.back().duration)
                    // {
                    //     current_time = 0;
                    // }
                }

                foreach(ntrt_plant_t* plant, this->plants)
                {
                    plant->update_to_bullet();
                }

            // }
            // else
            // {
            //     world->reset();
                
            //     foreach(ntrt_plant_t* plant, this->plants)
            //     {
            //         plant->add_bodies();
            //     }
            // }

            
            simulator_t::propagate(simulation::simulation_step);

            // srand(11111);
            world->step(simulation::simulation_step);

            // srand(start_seed);
            // for(unsigned i=0;i<rand_count;i++)
            // {
            //     rand();
            // }

            foreach(ntrt_plant_t* plant, this->plants)
            {
                plant->update_from_bullet();
            }
            state_space->copy_to_point(prev_state);
            plant_control_space->copy_to_point(prev_control);

        //     broadphase->resetPool(dispatcher);
        //     solver->reset();
        //     // if(!heightmap)
        //     //     dynamicsWorld->removeRigidBody(ground_plane);
        //     // for( unsigned i = 0; i < rigid_bodies.size(); i++ )
        //     // {
        //     //     dynamicsWorld->removeRigidBody(rigid_bodies[i].second);
        //     // }
        //     // delete dynamicsWorld;
        //     // dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

        //     dynamicsWorld->getSolverInfo().m_solverMode = 0;
        //     // dynamicsWorld->getSolverInfo().m_globalCfm = 0.02;
        //     // dynamicsWorld->getSolverInfo().m_erp = 0.2;
        //     dynamicsWorld->getSolverInfo().m_numIterations = inner_iterations;
        //     dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 1;
        //     // dynamicsWorld->getSolverInfo().m_splitImpulse = true;
        //     // dynamicsWorld->getSolverInfo().m_splitImpulsePenetrationThreshold = -0.02;

        //     // dynamicsWorld->setGravity(btVector3(gravity[0], gravity[1], gravity[2]));
        //     // if(!heightmap)
        //     //     dynamicsWorld->addRigidBody(ground_plane);
        //     // for( unsigned i = 0; i < rigid_bodies.size(); i++ )
        //     // {
        //     //     rigid_bodies[i].second->clearForces();
        //     //     dynamicsWorld->addRigidBody(rigid_bodies[i].second);
        //     // }

        //     // foreach(bullet_plant_t* plant, plants)
        //     // {
        //     //     plant->add_constraints(dynamicsWorld);
        //     // }



        //     dynamicsWorld->stepSimulation(simulation::simulation_step, 0, simulation::simulation_step);
        //     // collision_checker->in_collision();


        //     // checked_cast< ode_collision_checker_t* >(collision_checker)->clear_collisions();
        }

        bool ntrt_simulator_t::in_collision()
        {
            return false;
            // int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
            // bullet_collision_checker_t* checker = dynamic_cast<bullet_collision_checker_t*>(collision_checker);
            // for (int i=0;i<numManifolds;i++)
            // {
            //     btPersistentManifold* contactManifold =  dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
            //     const btCollisionObject* obA = static_cast<const btCollisionObject*>(contactManifold->getBody0());
            //     const btCollisionObject* obB = static_cast<const btCollisionObject*>(contactManifold->getBody1());
            //     checker->add_temporal_collision(((bullet_body_info_t*)obA->getUserPointer())->name,((bullet_body_info_t*)obB->getUserPointer())->name ); 
            // }
            // return checker->in_collision();

            // // return collision_checker->in_collision(config_list);
        }

        void ntrt_simulator_t::propagate_and_respond()
        {
            propagate(simulation::simulation_step);
        }

        void ntrt_simulator_t::remove_system(const std::string& path)
        {
            PRX_DEBUG_COLOR("DON'T EVEN TRY", PRX_TEXT_CYAN);
        }

        void ntrt_simulator_t::replace_system(const std::string& path, system_ptr_t system)
        {
            PRX_DEBUG_COLOR("DON'T EVEN TRY", PRX_TEXT_CYAN);
        }

        void ntrt_simulator_t::push_state(const state_t * const source)
        {
            // PRX_INFO_S("STATE: "<<state_space->print_memory());

            // PRX_INFO_S("STATE 2: "<<state_space->print_point(source));
            state_space->copy_from_point(source);

            foreach(ntrt_plant_t* plant, this->plants)
            {
                plant->update_to_bullet();
            }
        }

        state_t* ntrt_simulator_t::pull_state()
        {
            return simulator_t::pull_state();
        }

        void ntrt_simulator_t::add_bodies_from_plants()
        {
            // system_graph_t sys_graph;
            // update_system_graph(sys_graph);
            // std::vector<plant_t*> reg_plants;
            // sys_graph.get_plants(reg_plants);

            // plants.clear();

            // foreach(plant_t* plant, reg_plants)
            // {
            //     plants.push_back(checked_cast< bullet_plant_t* >(plant));
            //     plants.back()->add_bodies(rigid_bodies);
            //     plants.back()->add_constraints(dynamicsWorld);
            //     plants.back()->update_from_bullet();

            // }
            // for( unsigned i = 0; i < rigid_bodies.size(); i++ )
            // {
            //     dynamicsWorld->addRigidBody(rigid_bodies[i].second);
            // }

            // foreach(bullet_plant_t* plant, plants)
            // {
            //     plant->add_constraints(dynamicsWorld);
            // }

            // foreach(std::string name, obstacles | boost::adaptors::map_keys)
            // {
            //     bullet_obstacles.push_back((bullet_obstacle_t*)obstacles[name].get());
            //     bullet_obstacles.back()->add_bodies(rigid_bodies);
            // }

        }


    }
}
#endif