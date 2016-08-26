/**
 * @file bullet_body_info.hpp  
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
#pragma once



#ifndef PRX_BULLET_BODY_INFO_HPP
#define PRX_BULLET_BODY_INFO_HPP


#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/simulation/systems/plants/bullet/bullet_plant.hpp"
#include "prx/simulation/communication/visualization_comm.hpp"

namespace prx
{
    namespace util
    {
        class geometry_t;
    }
    
    namespace sim
    {

        /**
         * Descriptors of rigid bodies for use in Bullet.
         * 
         * @brief <b> Rigid body information in Bullet. </b>
         * 
         * @author Zakary Littlefield
         * 
         */
        class bullet_body_info_t
        {

          public:
            bullet_body_info_t() {};
            ~bullet_body_info_t() {};

            /** @brief The linked geometry for this rigid body. */
            util::geometry_t* geom;
            /** @brief The string identifier for this rigid body. */
            std::string name;          
            /** @brief The plant that this body belongs to*/
            system_t* plant;
            /** @brief The configuration of the rigid body. */
            util::config_t config;

            std::vector<double*> state;

            double mass;

            int* faces;
            double* vertex_array;
        };


        class bullet_control_t
        {
            public:
                bullet_control_t()
                {
                    val = new double;
                    *val = 0;
                }
                ~bullet_control_t()
                {
                    delete val;
                }

                virtual void apply() = 0;

                std::string object;

                std::string name;
                /** @brief The control bounds for this control. */
                util::bounds_t bounds;
                double* val;
                btRigidBody* body;
        };

        class bullet_body_torque_t : public bullet_control_t
        {
        public:
                bullet_body_torque_t() {};
                ~bullet_body_torque_t() {};

                void apply()
                {
                    // btTransform trans = body->getWorldTransform();
                    // trans.setOrigin(btVector3(0,0,0));
                    // torque = (*val)*(trans(direction));
                    torque = (*val)*((direction));
                    body->applyTorque(torque);
                }

                btVector3 direction;
                btVector3 torque;
        };
        class bullet_body_force_t : public bullet_control_t
        {
        public:
                bullet_body_force_t() {};
                ~bullet_body_force_t() {};

                void apply()
                {
                    btTransform trans = body->getWorldTransform();
                    trans.setOrigin(btVector3(0,0,0));
                    force = (*val)*(trans(direction));
                    body->applyForce(force,rel_pos);
                }

                btVector3 rel_pos;
                btVector3 direction;
                btVector3 force;
        };

        class bullet_constraint_t: public bullet_control_t
        {
        public:
                bullet_constraint_t() : bullet_control_t()
                {
                    constraint = NULL;
                    controlled = false;
                }
                ~bullet_constraint_t() {delete constraint;};

                void init(const util::parameter_reader_t* reader, std::vector<std::pair<btCollisionShape*,btRigidBody*> >& rigid_bodies, std::string plant_name)
                {
                    //read the type of constraint from reader
                    type = reader->get_attribute("type");
                    if(type=="hinge2")
                    {
                        std::string root_body_name = plant_name+"/"+reader->get_attribute("root_body");
                        std::string child_body_name = plant_name+"/"+reader->get_attribute("child_body");

                        for(unsigned i=0;i<rigid_bodies.size();i++)
                        {
                            if( ((bullet_body_info_t*)rigid_bodies[i].first->getUserPointer())->name == root_body_name )
                            {
                                root_body = rigid_bodies[i].second;
                                break;
                            }
                        }
                        for(unsigned i=0;i<rigid_bodies.size();i++)
                        {
                            if( ((bullet_body_info_t*)rigid_bodies[i].first->getUserPointer())->name == child_body_name )
                            {
                                child_body = rigid_bodies[i].second;
                                break;
                            }
                        }
                        std::vector<double> anchor_values = reader->get_attribute_as<std::vector<double> >("anchor");
                        std::vector<double> axis1_values = reader->get_attribute_as<std::vector<double> >("axis1");
                        std::vector<double> axis2_values = reader->get_attribute_as<std::vector<double> >("axis2");
                        btVector3 anchor(anchor_values[0],anchor_values[1],anchor_values[2]);
                        btVector3 axis1(axis1_values[0],axis1_values[1],axis1_values[2]);
                        btVector3 axis2(axis2_values[0],axis2_values[1],axis2_values[2]);
                        anchor+=root_body->getWorldTransform().getOrigin(); 
                        btTransform trans(root_body->getOrientation());
                        axis1 = trans(axis1);
                        axis2 = trans(axis2);
                        btHinge2Constraint* hinge = new btHinge2Constraint (*root_body, *child_body, anchor, axis1, axis2);
                        hinge->setLowerLimit(0);
                        hinge->setUpperLimit(0);
                        hinge->enableSpring(2,true);
                        hinge->enableSpring(1,false);
                        hinge->enableSpring(0,false);
                        hinge->setLinearLowerLimit(btVector3(0,0,0));
                        hinge->setLinearUpperLimit(btVector3(0,0,0));
                        hinge->setAngularLowerLimit(btVector3(0,1,0));
                        hinge->setAngularUpperLimit(btVector3(0,-1,0));

                        constraint = hinge;

                    }
                    else if(type=="generic")
                    {
                        std::string root_body_name = plant_name+"/"+reader->get_attribute("root_body");
                        std::string child_body_name = plant_name+"/"+reader->get_attribute("child_body");

                        if(root_body_name!=plant_name+"/world")
                        {
                            for(unsigned i=0;i<rigid_bodies.size();i++)
                            {
                                if( ((bullet_body_info_t*)rigid_bodies[i].first->getUserPointer())->name == root_body_name )
                                {
                                    root_body = rigid_bodies[i].second;
                                    break;
                                }
                            }
                            for(unsigned i=0;i<rigid_bodies.size();i++)
                            {
                                if( ((bullet_body_info_t*)rigid_bodies[i].first->getUserPointer())->name == child_body_name )
                                {
                                    child_body = rigid_bodies[i].second;
                                    break;
                                }
                            }
                            btTransform trans;
                            trans.setIdentity();
                            btTransform placement = root_body->getWorldTransform().inverseTimes(child_body->getWorldTransform());
                            btTransform inv = placement.inverse();
                            inv.setOrigin(btVector3(0,0,0));

                            btGeneric6DofConstraint* generic = new btGeneric6DofConstraint(*root_body,*child_body,placement,trans,false);
                            std::vector<double> linear_lower = reader->get_attribute_as<std::vector<double> >("linear_lower_limits");
                            std::vector<double> linear_upper = reader->get_attribute_as<std::vector<double> >("linear_upper_limits");
                            std::vector<double> angular_lower = reader->get_attribute_as<std::vector<double> >("angular_lower_limits");
                            std::vector<double> angular_upper = reader->get_attribute_as<std::vector<double> >("angular_upper_limits");
                            generic->setLinearLowerLimit(btVector3(linear_lower[0],linear_lower[1],linear_lower[2]));
                            generic->setLinearUpperLimit(btVector3(linear_upper[0],linear_upper[1],linear_upper[2]));
                            generic->setAngularLowerLimit(inv(btVector3(angular_lower[0],angular_lower[1],angular_lower[2])));
                            generic->setAngularUpperLimit(inv(btVector3(angular_upper[0],angular_upper[1],angular_upper[2])));
                            constraint = generic;
                        }
                        else
                        {

                            for(unsigned i=0;i<rigid_bodies.size();i++)
                            {
                                if( ((bullet_body_info_t*)rigid_bodies[i].first->getUserPointer())->name == child_body_name )
                                {
                                    child_body = rigid_bodies[i].second;
                                    break;
                                }
                            }
                            btTransform trans;
                            trans.setIdentity();
                            btTransform placement = child_body->getWorldTransform();
                            btTransform inv = placement.inverse();
                            inv.setOrigin(btVector3(0,0,0));

                            btGeneric6DofConstraint* generic = new btGeneric6DofConstraint(*child_body,trans,true);
                            std::vector<double> linear_lower = reader->get_attribute_as<std::vector<double> >("linear_lower_limits");
                            std::vector<double> linear_upper = reader->get_attribute_as<std::vector<double> >("linear_upper_limits");
                            std::vector<double> angular_lower = reader->get_attribute_as<std::vector<double> >("angular_lower_limits");
                            std::vector<double> angular_upper = reader->get_attribute_as<std::vector<double> >("angular_upper_limits");
                            generic->setLinearLowerLimit(btVector3(linear_lower[0],linear_lower[1],linear_lower[2]));
                            generic->setLinearUpperLimit(btVector3(linear_upper[0],linear_upper[1],linear_upper[2]));
                            generic->setAngularLowerLimit(inv(btVector3(angular_lower[0],angular_lower[1],angular_lower[2])));
                            generic->setAngularUpperLimit(inv(btVector3(angular_upper[0],angular_upper[1],angular_upper[2])));
                            constraint = generic;
                        }

                    }
                    else if(type=="hinge")
                    {
                        std::string root_body_name = plant_name+"/"+reader->get_attribute("root_body");
                        std::string child_body_name = plant_name+"/"+reader->get_attribute("child_body");

                        if(root_body_name!=plant_name+"/world")
                        {
                            for(unsigned i=0;i<rigid_bodies.size();i++)
                            {
                                if( ((bullet_body_info_t*)rigid_bodies[i].first->getUserPointer())->name == root_body_name )
                                {
                                    root_body = rigid_bodies[i].second;
                                    break;
                                }
                            }
                            for(unsigned i=0;i<rigid_bodies.size();i++)
                            {
                                if( ((bullet_body_info_t*)rigid_bodies[i].first->getUserPointer())->name == child_body_name )
                                {
                                    child_body = rigid_bodies[i].second;
                                    break;
                                }
                            }


                            std::vector<double> origin_in = reader->get_attribute_as<std::vector<double> >("origin");
                            std::vector<double> rpy_in;
                            rpy_in.push_back(0);
                            rpy_in.push_back(0);
                            rpy_in.push_back(0);
                            if(reader->has_attribute("rpy"))
                                rpy_in = reader->get_attribute_as<std::vector<double> >("rpy");
                            std::vector<double> axis_in  = reader->get_attribute_as<std::vector<double> >("axis" );
                            std::vector<double> limit = reader->get_attribute_as<std::vector<double> >("limit");
                            btVector3 axis(axis_in[0],axis_in[1],axis_in[2]);
                            btVector3 origin(origin_in[0],origin_in[1],origin_in[2]);
                            btQuaternion quat(rpy_in[2],rpy_in[1],rpy_in[0]);

                            btTransform parent_trans = root_body->getWorldTransform();
                            btTransform child_trans = child_body->getWorldTransform();

                            btVector3 pivotParent = parent_trans.inverse()*child_trans*origin;
                            btVector3 pivotChild = origin;

                            parent_trans.setOrigin(btVector3(0,0,0));
                            child_trans.setOrigin(btVector3(0,0,0));
                            btVector3 axisParent = child_trans.inverse()*axis;
                            btVector3 axisChild = axis;
                            PRX_INFO_S("Pivots: Parent: "<<pivotParent.getX()<<" "<<pivotParent.getY()<<" "<<pivotParent.getZ()<<" Child: "
                                        <<pivotChild.getX()<<" "<<pivotChild.getY()<<" "<<pivotChild.getZ());

                            PRX_INFO_S("Hinge Axes: Parent: "<<axisParent.getX()<<" "<<axisParent.getY()<<" "<<axisParent.getZ()<<" Child: "
                                        <<axisChild.getX()<<" "<<axisChild.getY()<<" "<<axisChild.getZ());

                            btHingeConstraint* hinge = new btHingeConstraint(*root_body,*child_body,pivotParent,pivotChild,axisParent,axisChild,false);
                            hinge->setLimit(limit[0],limit[1]);
                            constraint = hinge;
                            hinge->enableMotor(true);
                            controlled = true;
                            bounds.set_bounds(limit[0],limit[1]);
                            *val = hinge->getHingeAngle();
                        }
                        else
                        {
                            for(unsigned i=0;i<rigid_bodies.size();i++)
                            {
                                if( ((bullet_body_info_t*)rigid_bodies[i].first->getUserPointer())->name == child_body_name )
                                {
                                    child_body = rigid_bodies[i].second;
                                    break;
                                }
                            }

                            std::vector<double> origin_in = reader->get_attribute_as<std::vector<double> >("origin");
                            std::vector<double> axis_in  = reader->get_attribute_as<std::vector<double> >("axis" );
                            std::vector<double> limit = reader->get_attribute_as<std::vector<double> >("limit");
                            btVector3 axis(axis_in[0],axis_in[1],axis_in[2]);
                            btVector3 origin(origin_in[0],origin_in[1],origin_in[2]);

                            btTransform child_trans = child_body->getWorldTransform();
                            child_trans.setOrigin(btVector3(0,0,0));

                            btVector3 pivotChild = origin;//child_body->getWorldTransform().inverse()*(root_body->getWorldTransform()*pivotParent);
                            btVector3 axisChild = axis;//child_trans.inverse()*parent_trans*axis;

                            btHingeConstraint* hinge = new btHingeConstraint(*child_body,pivotChild,axisChild,false);
                            hinge->setLimit(limit[0],limit[1]);
                            constraint = hinge;
                            hinge->enableMotor(true);
                            hinge->setMaxMotorImpulse(1.0);
                            controlled = true;
                            bounds.set_lower_bound(limit[0]);
                            bounds.set_upper_bound(limit[1]);
                            *val = hinge->getHingeAngle();
                        }

                    }
                    //used for spring forces (derived from NTRT 1.1 3/4/15)
                    else if(type=="spring_cable")
                    {
                        controlled = true;
                        std::string root_body_name = plant_name+"/"+reader->get_attribute("root_body");
                        std::string child_body_name = plant_name+"/"+reader->get_attribute("child_body");
                        node1 = reader->get_attribute("root_body");
                        node2 = reader->get_attribute("child_body");
                        std::string root_link_name = plant_name+"/"+reader->get_attribute("root_link");
                        std::string child_link_name = plant_name+"/"+reader->get_attribute("child_link");
                        std::vector<double> limit = reader->get_attribute_as<std::vector<double> >("limit");
                        bounds.set_bounds(limit[0],limit[1]);

                        m_dampingCoefficient = reader->get_attribute_as<double>("damping_coefficient");
                        m_K = reader->get_attribute_as<double>("stiffness");
                        double pretension = reader->get_attribute_as<double>("pretension");
                        m_damping = 0;
                        m_velocity = 0;
                        for(unsigned i=0;i<rigid_bodies.size();i++)
                        {
                            if( ((bullet_body_info_t*)rigid_bodies[i].first->getUserPointer())->name == root_body_name )
                            {
                                root_body = rigid_bodies[i].second;
                                break;
                            }
                        }
                        for(unsigned i=0;i<rigid_bodies.size();i++)
                        {
                            if( ((bullet_body_info_t*)rigid_bodies[i].first->getUserPointer())->name == child_body_name )
                            {
                                child_body = rigid_bodies[i].second;
                                break;
                            }
                        }
                        for(unsigned i=0;i<rigid_bodies.size();i++)
                        {
                            if( ((bullet_body_info_t*)rigid_bodies[i].first->getUserPointer())->name == root_link_name )
                            {
                                root_link = rigid_bodies[i].second;
                                break;
                            }
                        }
                        for(unsigned i=0;i<rigid_bodies.size();i++)
                        {
                            if( ((bullet_body_info_t*)rigid_bodies[i].first->getUserPointer())->name == child_link_name )
                            {
                                child_link = rigid_bodies[i].second;
                                break;
                            }
                        }
                        btTransform root_trans = root_body->getWorldTransform();
                        btTransform child_trans = child_body->getWorldTransform();
                        btTransform child_link_trans = child_link->getWorldTransform();
                        btTransform root_link_trans = root_link->getWorldTransform();
                        btVector3 zero(0,0,0);

                        //calculate anchor positions
                        {
                            btVector3 to = root_trans(zero);
                            btVector3 center = root_link_trans(zero);
                            const btVector3 cylinderAxis = (to - center).normalize();
                            const btVector3 cylinderAxis2 = (to - center).normalize();
                            btVector3 referencePoint = root_trans(zero);
                            btVector3 destinationPoint = child_trans(zero);
                            const btVector3 refToDest = (referencePoint - destinationPoint).normalize();
                            btVector3 rotationAxis = cylinderAxis.cross(refToDest);
                            const btVector3 directional = cylinderAxis.rotate(rotationAxis, -M_PI / 2.0).normalize();
                            const btVector3 surfaceVector = directional * .31;
                            root_anchor = root_link_trans.inverse()((referencePoint+surfaceVector));
                        }
                        {
                            btVector3 to = child_trans(zero);
                            btVector3 center = child_link_trans(zero);
                            const btVector3 cylinderAxis = (to - center).normalize();
                            const btVector3 cylinderAxis2 = (to - center).normalize();
                            btVector3 referencePoint = child_trans(zero);
                            btVector3 destinationPoint = root_trans(zero);
                            const btVector3 refToDest = (referencePoint - destinationPoint).normalize();
                            btVector3 rotationAxis = cylinderAxis.cross(refToDest);
                            const btVector3 directional = cylinderAxis.rotate(rotationAxis, -M_PI / 2.0).normalize();
                            const btVector3 surfaceVector = directional * .31;
                            child_anchor = child_link_trans.inverse()((referencePoint+surfaceVector));
                        }
                        // {
                        //     root_anchor = root_trans(zero);
                        //     child_anchor = child_trans(zero);
                        //     btVector3 dist = root_anchor-child_anchor;
                        //     double curr_length = dist.length();
                        //     double scaling = (curr_length-.2)/curr_length;
                        //     btVector3 temp_root = scaling*root_anchor + (1-scaling)*child_anchor;
                        //     btVector3 temp_child = (1-scaling)*root_anchor + scaling*child_anchor;
                        //     root_anchor = root_link_trans.inverse()(temp_root);
                        //     child_anchor = child_link_trans.inverse()(temp_child);
                        // }
                        // PRX_INFO_S(root_link_name<<" "<<root_anchor.getX()<<" "<<root_anchor.getY()<<" "<<root_anchor.getZ());
                        // PRX_INFO_S(child_link_name<<" "<<child_anchor.getX()<<" "<<child_anchor.getY()<<" "<<child_anchor.getZ());
                        
                        btVector3 root_pos = root_link_trans(root_anchor);
                        btVector3 child_pos = child_link_trans(child_anchor);
                        m_restLength = root_pos.distance(child_pos) - pretension / m_K;
                        // PRX_INFO_S(m_restLength);
                        m_prevLength = m_restLength;
                    }

                }
                void apply()
                {
                    if(controlled && type=="hinge")
                    {
                        PRX_INFO_S("NAME: "<<((bullet_body_info_t*)child_body->getUserPointer())->name<<" "<< ((btHingeConstraint*)(constraint))->getHingeAngle()<<" "<<*val);
                        double diff= ((btHingeConstraint*)(constraint))->getHingeAngle() - *val;
                        double applied = ((btHingeConstraint*)(constraint))->getHingeAngle();
                        double motorspeed = 1;
                        if(diff < -motorspeed * simulation::simulation_step)
                            applied += motorspeed * simulation::simulation_step;
                        else if(diff > motorspeed * simulation::simulation_step)
                            applied -= motorspeed * simulation::simulation_step;
                        else
                            applied = *val;

                        ((btHingeConstraint*)(constraint))->setMotorTarget(*val,simulation::simulation_step);
                    }
                    if(type=="spring_cable")
                    {
                        //from moveMotor
                        double step_size = 10000*simulation::simulation_step;

                        //from bulletSpring calculateAndApplyForce
                        btVector3 force(0, 0, 0);
                        btTransform root_link_trans = root_link->getWorldTransform();
                        btTransform child_link_trans = child_link->getWorldTransform();
                        // btTransform root_trans = root_body->getWorldTransform();
                        // btTransform child_trans = child_body->getWorldTransform();


                        // btVector3 zero(0,0,0);
                        btVector3 root_pos = root_link_trans(root_anchor);
                        btVector3 child_pos = child_link_trans(child_anchor);

                        btVector3 dist = root_pos-child_pos;
                        double curr_length = dist.length();
                        btVector3 unit_vector = dist/curr_length;
                        double stretch = curr_length - *val;
                        double magnitude = m_K*stretch;

                        if(magnitude > 100000)
                        {
                            *val = curr_length - 100000/m_K;
                        }
                        double diff = *val - m_restLength;
                        const double fabsDiff = fabs(diff);
                        if(fabsDiff > step_size)
                        {
                            m_restLength += (diff/fabsDiff)*step_size;
                        }
                        else
                        {
                            m_restLength += diff;
                        }

                        stretch = curr_length - m_restLength;
                        magnitude = m_K*stretch;

                        double delta_stretch = curr_length - m_prevLength;
                        m_velocity = delta_stretch/simulation::simulation_step;
                        m_damping = m_dampingCoefficient*m_velocity;
                        if (fabs(magnitude) * 1.0 < fabs(m_damping))
                        {
                            m_damping =
                              (m_damping > 0.0 ? magnitude * 1.0 : -magnitude * 1.0);
                        }
                        // PRX_INFO_S("---- "<<magnitude);
                        magnitude+=m_damping;
                        // PRX_INFO_S("++++ "<<magnitude);
                        if (curr_length > m_restLength)
                        {   
                            force = unit_vector * magnitude; 
                        }
                        else
                        {
                            // force = -unit_vector * magnitude; 
                        }
                        m_prevLength = curr_length;
                        // PRX_INFO_S("-spring: "<<force.getX()<<" "<<force.getY()<<" "<<force.getZ());
                        // root_link->activate();
                        // child_link->activate();
                        btVector3 point1 = root_pos - root_link->getCenterOfMassPosition();
                        btVector3 point2 = child_pos - child_link->getCenterOfMassPosition();
                        // PRX_INFO_S("-point: "<<point1.getX()<<" "<<point1.getY()<<" "<<point1.getZ());
                        // PRX_INFO_S("-anchor: "<<root_anchor.getX()<<" "<<root_anchor.getY()<<" "<<root_anchor.getZ());
                        root_link->applyImpulse(-force*simulation::simulation_step,point1);
                        child_link->applyImpulse(force*simulation::simulation_step,point2);
                        // root_link->applyForce(-force,root_anchor);
                        // child_link->applyForce(force,child_anchor);
                    }
                }
                void visualize()
                {
                    if(type=="spring_cable" && ((comm::visualization_comm_t*)comm::vis_comm)!=NULL)
                    {
                        btTransform root_link_trans = root_link->getWorldTransform();
                        btTransform child_link_trans = child_link->getWorldTransform();
                        std::vector<util::geometry_info_t> geoms;
                        std::vector<util::config_t> configs;
                        std::vector<double> params;
                        btVector3 root_link_pos = root_link_trans(root_anchor);
                        btVector3 child_link_pos = child_link_trans(child_anchor);

                        params.push_back(root_link_pos.getX());
                        params.push_back(root_link_pos.getY());
                        params.push_back(root_link_pos.getZ());
                        params.push_back(child_link_pos.getX());
                        params.push_back(child_link_pos.getY());
                        params.push_back(child_link_pos.getZ());
                        geoms.push_back(util::geometry_info_t("tensegrity", "/"+node1+"_"+node2, util::PRX_LINESTRIP, params, "teal"));
                        configs.push_back( util::config_t() );
                        params.clear();
                        ((comm::visualization_comm_t*)comm::vis_comm)->visualization_geom_map["tensegrity/"+node1+"_"+node2] = geoms;
                        ((comm::visualization_comm_t*)comm::vis_comm)->visualization_configs_map["tensegrity/"+node1+"_"+node2] = configs;
                    }
                }

                btTypedConstraint* constraint;
                btRigidBody* root_body;
                btRigidBody* child_body;
                btRigidBody* root_link;
                btRigidBody* child_link;
                btVector3 root_anchor;
                btVector3 child_anchor;
                std::string type;
                bool controlled;

                std::string node1;
                std::string node2;
                
                //used for spring forces (derived from NTRT 1.1 3/4/15)
                double m_damping;
                double m_velocity;
                double m_K;
                double m_dampingCoefficient;
                double m_restLength;
                double m_prevLength;

        };

    }
}


#endif
#endif


