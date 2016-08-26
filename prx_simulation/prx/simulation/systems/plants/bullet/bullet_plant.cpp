/**
 * @file bullet_plant.cpp 
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

#include "prx/simulation/systems/plants/bullet/bullet_plant.hpp"
#include "prx/simulation/systems/plants/bullet/bullet_body_info.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/math/2d_geometry/angle.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/assign/list_of.hpp>
#include "prx/utilities/definitions/string_manip.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::bullet_plant_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    namespace sim
    {

        bullet_plant_t::bullet_plant_t() 
        {
            joint_state_count = 0;
        }

        bullet_plant_t::~bullet_plant_t()
        {
            rigid_bodies.clear();
            for( unsigned i = 0; i < body_spaces.size(); ++i )
            {
                delete body_spaces[i];
            }
        }

        void bullet_plant_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            //getting ready to read all the geometries
            std::vector<const parameter_reader_t*> readers = reader->get_list("geometries");
            std::string geom_name;

            config_t conf;
            vector_t pos(3);
            quaternion_t orient;

            foreach(const parameter_reader_t* r, readers)
            {
                geom_name = pathname + "/" + r->get_attribute_as< std::string > ("name");
                geometries[geom_name] = (*r->initialize_new<geometry_t > (std::string("collision_geometry")));
                if(r->has_attribute("relative_to"))
                {
                    conf = (*r->initialize_new< config_t > (std::string("config")));
                    conf.get(pos, orient);
                    orient.normalize();
                    conf.set_orientation(orient);
                    std::string other_body = pathname + "/" + r->get_attribute_as< std::string > ("relative_to");

                    for( unsigned i = 0; i < rigid_bodies.size(); ++i )
                    {
                        bullet_body_info_t* info = ((bullet_body_info_t*)(rigid_bodies[i].first->getUserPointer()));
                        std::string name = info->name;
                        if(name == other_body)
                        {
                            PRX_INFO_S(geom_name<<" "<<conf<<" relative to "<<name<<" "<<info->config);
                            conf.relative_to_global(info->config);
                            conf.get(pos, orient);
                        }
                    }
                }
                else if(r->has_attribute("between"))
                {
                    std::vector<std::string> other_bodies = r->get_attribute_as< std::vector<std::string> > ("between");
                    other_bodies[0] = pathname + "/" + other_bodies[0];
                    other_bodies[1] = pathname + "/" + other_bodies[1];
                    config_t body1;
                    config_t body2;

                    for( unsigned i = 0; i < rigid_bodies.size(); ++i )
                    {
                        bullet_body_info_t* info = ((bullet_body_info_t*)(rigid_bodies[i].first->getUserPointer()));
                        std::string name = info->name;
                        if(name == other_bodies[0])
                        {
                            PRX_INFO_S(geom_name<<" between 1: "<<name<<" "<<info->config);
                            body1 = info->config;
                        }
                        if(name == other_bodies[1])
                        {
                            PRX_INFO_S(geom_name<<" between 2: "<<name<<" "<<info->config);
                            body2 = info->config;
                        }
                    }
                    //compute the center point
                    double x1,x2,y1,y2,z1,z2;
                    body1.get_position(x1,y1,z1);
                    body2.get_position(x2,y2,z2);

                    conf.set_position((x1+x2)/2,(y1+y2)/2,(z1+z2)/2);
                    vector_t axis(3);
                    vector_t axis2(3);

                    axis[0]=0;
                    axis[1]=0;
                    axis[2]=1;
                    axis2[0]=x2-x1;
                    axis2[1]=y2-y1;
                    axis2[2]=z2-z1;
                    axis2.normalize();
                    orient.compute_rotation(axis,axis2);
                    conf.set_orientation(orient);

                    conf.get(pos, orient);
                    orient.normalize();
                    conf.set_orientation(orient);

                }
                else
                {
                    conf = (*r->initialize_new< config_t > (std::string("config")));
                    conf.get(pos, orient);
                    orient.normalize();
                    conf.set_orientation(orient);
                }

                //store in a body info class
                bullet_body_info_t* info = new bullet_body_info_t();
                PRX_INFO_S(geom_name);
                info->name = geom_name;
                info->geom = &geometries[geom_name];
                info->plant = this;
                info->config = conf;

                config_names.push_back(geom_name);


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
                    PRX_FATAL_S("Unimplemented geometry type for bullet_plant");
                }
                btScalar mass = r->get_attribute_as< double > ("mass");
                btVector3 inertia(0, 0, 0);
                if( r->has_attribute("inertia") )
                {
                    std::vector<double> inert = r->get_attribute_as<std::vector<double> >("inertia");
                    inertia.setX(inert[0]);
                    inertia.setY(inert[1]);
                    inertia.setZ(inert[2]);
                }
                else
                {
                    collision_shape->calculateLocalInertia(mass, inertia);
                }
                // btDefaultMotionState* motion_state = new btDefaultMotionState(btTransform(
                //             btQuaternion(orient.get_x(),orient.get_y(),orient.get_z(),orient.get_w()),
                //             btVector3(pos[0],pos[1],pos[2])));
                // collision_shape->calculateLocalInertia(mass,inertia);
                collision_shape->setUserPointer(info);
                btRigidBody::btRigidBodyConstructionInfo construction_info(mass, NULL, collision_shape, inertia);
                btRigidBody* rigid_body = new btRigidBody(construction_info);

                btTransform trans(btQuaternion(orient.get_x(), orient.get_y(), orient.get_z(), orient.get_w()),
                                  btVector3(pos[0], pos[1], pos[2]));
                rigid_body->proceedToTransform(trans);
                rigid_body->setAngularVelocity(btVector3(0, 0, 0));
                rigid_body->setLinearVelocity(btVector3(0, 0, 0));
                // rigid_body->setAngularFactor(0);
                // rigid_body->setDamping(0,1);

                rigid_body->setUserPointer(info);
                rigid_body->forceActivationState(ACTIVE_TAG);
                // rigid_body->setCcdMotionThreshold(.1);
                rigid_body->setFriction(1.0);
                rigid_body->setRollingFriction(.4);
                rigid_body->setRestitution(.2);
                rigid_body->setDeactivationTime(0);
                rigid_bodies.push_back(std::pair<btCollisionShape*, btRigidBody*>(collision_shape, rigid_body));

                body_state_memory.push_back(std::vector<double*>());
                for( int i = 0; i < 13; i++ )
                    body_state_memory.back().push_back(new double);
                *body_state_memory.back()[0] = pos[0];
                *body_state_memory.back()[1] = pos[1];
                *body_state_memory.back()[2] = pos[2];
                *body_state_memory.back()[3] = orient.get_x();
                *body_state_memory.back()[4] = orient.get_y();
                *body_state_memory.back()[5] = orient.get_z();
                *body_state_memory.back()[6] = orient.get_w();
                *body_state_memory.back()[7] = 0;
                *body_state_memory.back()[8] = 0;
                *body_state_memory.back()[9] = 0;
                *body_state_memory.back()[10] = 0;
                *body_state_memory.back()[11] = 0;
                *body_state_memory.back()[12] = 0;
                info->state = body_state_memory.back();

                //Finally, add the simple state space which corresponds to this space
                space_t* space = new space_t("DynamicRigidBody", body_state_memory.back());

                if( r->has_attribute("state_space") )
                {
                    space->init(r->get_child("state_space").get());
                }
                body_spaces.push_back(space);

            }

            body_control_spaces.clear();
            body_control_memory.clear();
            if( reader->has_element("controls") )
            {
                std::vector< const parameter_reader_t* > control_readers = reader->get_list("controls");

                foreach(const parameter_reader_t* r, control_readers)
                {
                    std::string type = r->get_attribute_as< std::string > ("type");
                    std::string name = r->get_attribute_as< std::string > ("name");
                    std::string object = pathname + "/" + r->get_attribute_as< std::string > ("object");

                    if( type == "body_torque" )
                    {
                        bullet_body_torque_t* bt = new bullet_body_torque_t();
                        bt->object = object;
                        std::vector<double> dir = r->get_attribute_as< std::vector<double> > ("direction");
                        bt->direction = btVector3(dir[0], dir[1], dir[2]);
                        body_controls.push_back(bt);
                    }
                    else if( type == "body_force" )
                    {
                        bullet_body_force_t* bt = new bullet_body_force_t();
                        bt->object = object;
                        std::vector<double> dir = r->get_attribute_as< std::vector<double> > ("relative_position");
                        bt->rel_pos = btVector3(dir[0], dir[1], dir[2]);
                        dir = r->get_attribute_as< std::vector<double> > ("direction");
                        bt->direction = btVector3(dir[0], dir[1], dir[2]);
                        body_controls.push_back(bt);
                    }
                    else
                    {
                        PRX_FATAL_S("Unknown type of control specified : "<<type);
                    }
                    body_controls.back()->name = name;
                    for( unsigned i = 0; i < rigid_bodies.size(); i++ )
                    {
                        if( ((bullet_body_info_t*)rigid_bodies[i].first->getUserPointer())->name == object )
                        {
                            body_controls.back()->body = rigid_bodies[i].second;
                            break;
                        }

                    }
                    if(r->has_attribute("shared_input"))
                    {
                        std::string shared_input = r->get_attribute_as< std::string > ("shared_input");
                        for( unsigned i = 0; i < body_controls.size(); i++ )
                        {
                            if( body_controls[i]->name ==shared_input )
                            {
                                body_controls.back()->val = body_controls[i]->val;
                                break;
                            }
                        }
                    }
                    else
                    {
                        std::vector<double> limits = r->get_attribute_as< std::vector<double> > ("bounds");
                        body_controls.back()->bounds.set_bounds(limits[0], limits[1]);
                        bounds_t new_bound(body_controls.back()->bounds.get_lower_bound(), body_controls.back()->bounds.get_upper_bound());
                        body_control_memory.push_back(std::vector<double*>());
                        body_control_memory.back().push_back(body_controls.back()->val);
                        body_control_spaces.push_back(new space_t("X", body_control_memory.back()));

                        ((space_t*)body_control_spaces.back())->set_bounds({&new_bound});
                    }
                }


            }
            if( reader->has_element("joints") )
            {
                joint_readers = reader->get_list("joints");

                foreach(const parameter_reader_t* r, joint_readers)
                {
                    bullet_constraint_t* constraint = new bullet_constraint_t();
                    constraint->init(r, rigid_bodies, pathname);
                    body_constraints.push_back(constraint);
                    if( constraint->controlled )
                    {
                        bounds_t new_bound(body_constraints.back()->bounds.get_lower_bound(), body_constraints.back()->bounds.get_upper_bound());
                        body_control_memory.push_back(std::vector<double*>());
                        body_control_memory.back().push_back(body_constraints.back()->val);
                        body_control_spaces.push_back(new space_t("X", body_control_memory.back()));
                        ((space_t*)body_control_spaces.back())->set_bounds({&new_bound});
                    }
                    if(constraint->type=="spring_cable")
                    {
                        std::vector<double*> joint_state;
                        joint_state.push_back(&(constraint->m_prevLength));
                        body_spaces.push_back(new space_t("X", joint_state));       
                        joint_state_count++;                 
                    }
                }
            }


            if( body_control_spaces.size() != 0 )
            {
                input_control_space = new space_t(body_control_spaces);

                foreach(std::vector<double*> inner, body_control_memory)
                {
                    control_memory.push_back(inner.back());
                }

                // foreach(double* val, control_memory)
                // {
                //     *val = 0;
                // }
            }
            else
            {
                input_control_space = new space_t("EMPTY",control_memory);
            }

            //Now we can finally set up the state space!
            state_space = new space_t(body_spaces);

            //Finally, allocate our state

            foreach(std::vector<double*> inner, body_state_memory)
            {

                foreach(double* val, inner)
                {
                    state_memory.push_back(val);
                }
            }
            update_to_bullet();
            update_from_bullet();
        }

        void bullet_plant_t::update_phys_configs(config_list_t& configs, unsigned& index) const
        {

            for( unsigned i = 0; i < rigid_bodies.size(); ++i )
            {
                const btTransform& trans = rigid_bodies[i].second->getCenterOfMassTransform();
                std::string name = ((bullet_body_info_t*)(rigid_bodies[i].first->getUserPointer()))->name;
                augment_config_list(configs,index);
                configs[index].first = name;
                configs[index].second.set_position(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
                configs[index].second.set_orientation(trans.getRotation().getX(), trans.getRotation().getY(), trans.getRotation().getZ(), trans.getRotation().getW());
                index++;
            }
        }

        void bullet_plant_t::verify() const { }

        void bullet_plant_t::propagate(const double simulation_step)
        {
            if( input_control_space != NULL )
            {
                input_control_space->enforce_bounds();
                for( unsigned i = 0; i < body_controls.size(); i++ )
                {
                    body_controls[i]->apply();
                }
                for( unsigned i = 0; i < body_constraints.size(); i++ )
                {
                    body_constraints[i]->apply();
                }
            }
        }

        void bullet_plant_t::set_active(bool in_active, const std::string& path)
        {
            active = in_active;
        }

        void bullet_plant_t::add_bodies(std::vector<std::pair<btCollisionShape*, btRigidBody*> >& global_list)
        {
            //add to the global list
            for( unsigned i = 0; i < rigid_bodies.size(); i++ )
            {
                global_list.push_back(rigid_bodies[i]);
            }
        }

        void bullet_plant_t::add_constraints(btDynamicsWorld* world)
        {
            for( unsigned i = 0; i < body_constraints.size(); i++ )
            {
                if(body_constraints[i]->constraint!=NULL)
                    world->addConstraint(body_constraints[i]->constraint, true);
            }
        }

        void bullet_plant_t::update_to_bullet()
        {
            unsigned i;
            for( i = 0; i < rigid_bodies.size(); ++i )
            {
                btVector3 translate(*state_memory[ 13 * i], *state_memory[ 13 * i + 1], *state_memory[ 13 * i + 2]);
                btQuaternion quat(*state_memory[ 13 * i + 3 ], *state_memory[ 13 * i + 4 ], *state_memory[ 13 * i + 5 ], *state_memory[ 13 * i + 6 ]);
                btVector3 lin(*state_memory[ 13 * i + 7 ], *state_memory[ 13 * i + 8 ], *state_memory[ 13 * i + 9 ]);
                btVector3 ang(*state_memory[ 13 * i + 10 ], *state_memory[ 13 * i + 11 ], *state_memory[ 13 * i + 12 ]);
                btTransform trans(quat, translate);
                rigid_bodies[i].second->setCenterOfMassTransform(trans);
                rigid_bodies[i].second->setLinearVelocity(lin);
                rigid_bodies[i].second->setAngularVelocity(ang);
            }
        }

        void bullet_plant_t::update_from_bullet()
        {
            for( unsigned i = 0; i < rigid_bodies.size(); ++i )
            {
                rigid_bodies[i].second->forceActivationState(ACTIVE_TAG);
                rigid_bodies[i].second->setDeactivationTime(0);
                const btTransform& trans = rigid_bodies[i].second->getCenterOfMassTransform();
                const btVector3& lin = rigid_bodies[i].second->getLinearVelocity();
                const btVector3& ang = rigid_bodies[i].second->getAngularVelocity();
                // rigid_bodies[i].second->getMotionState()->getWorldTransform(trans);
                btQuaternion quat = trans.getRotation();
                *state_memory[ 13 * i ] = trans.getOrigin().getX();
                *state_memory[ 13 * i + 1 ] = trans.getOrigin().getY();
                *state_memory[ 13 * i + 2 ] = trans.getOrigin().getZ();
                *state_memory[ 13 * i + 3 ] = quat.getX();
                *state_memory[ 13 * i + 4 ] = quat.getY();
                *state_memory[ 13 * i + 5 ] = quat.getZ();
                *state_memory[ 13 * i + 6 ] = quat.getW();
                *state_memory[ 13 * i + 7 ] = lin.getX();
                *state_memory[ 13 * i + 8 ] = lin.getY();
                *state_memory[ 13 * i + 9 ] = lin.getZ();
                *state_memory[ 13 * i + 10 ] = ang.getX();
                *state_memory[ 13 * i + 11 ] = ang.getY();
                *state_memory[ 13 * i + 12 ] = ang.getZ();
            }
            state_space->enforce_bounds();

        }

        void bullet_plant_t::recreate_bodies()
        {
            for( unsigned i = 0; i < rigid_bodies.size(); ++i )
            {
                const btTransform& trans = rigid_bodies[i].second->getCenterOfMassTransform();
                const btVector3& lin = rigid_bodies[i].second->getLinearVelocity();
                const btVector3& ang = rigid_bodies[i].second->getAngularVelocity();
                btScalar mass = ((bullet_body_info_t*)(rigid_bodies[i].second->getUserPointer()))->mass;
                btVector3 inertia(0, 0, 0);
                rigid_bodies[i].first->calculateLocalInertia(mass, inertia);
                btRigidBody::btRigidBodyConstructionInfo construction_info(mass, NULL, rigid_bodies[i].first, inertia);
                btRigidBody* rigid_body = new btRigidBody(construction_info);
                rigid_body->proceedToTransform(trans);
                rigid_body->setLinearVelocity(lin);
                rigid_body->setAngularVelocity(ang);
                rigid_body->setUserPointer(((bullet_body_info_t*)(rigid_bodies[i].second->getUserPointer())));
                rigid_body->forceActivationState(DISABLE_DEACTIVATION);
                delete rigid_bodies[i].second;
                rigid_bodies[i].second = rigid_body;

            }

            for( unsigned j = 0; j < body_controls.size(); j++ )
            {

                for( unsigned i = 0; i < rigid_bodies.size(); i++ )
                {
                    if( ((bullet_body_info_t*)rigid_bodies[i].first->getUserPointer())->name == body_controls[j]->object )
                    {
                        body_controls[j]->body = rigid_bodies[i].second;
                        break;
                    }

                }
            }
            foreach(bullet_constraint_t* c, body_constraints)
                    delete c;
            body_constraints.clear();

            foreach(const parameter_reader_t* r, joint_readers)
            {
                bullet_constraint_t* constraint = new bullet_constraint_t();
                constraint->init(r, rigid_bodies, pathname);
                body_constraints.push_back(constraint);
            }
            update_to_bullet();
            update_from_bullet();
        }

        void bullet_plant_t::update_vis_info() const
        {
            // for( unsigned i = 0; i < body_constraints.size(); i++ )
            // {
            //     body_constraints[i]->visualize();
            // }
            // ((comm::visualization_comm_t*)comm::vis_comm)->send_geometries();
        }
    }
}
#endif

