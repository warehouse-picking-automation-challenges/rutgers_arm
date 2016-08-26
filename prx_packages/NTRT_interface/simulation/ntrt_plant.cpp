/**
 * @file ntrt_plant.cpp 
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

#include "simulation/ntrt_plant.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/math/2d_geometry/angle.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

#include "dev/kcaluwae/zeromq_superball/T6Model.h"
#include "core/tgKinematicActuator.h"
#include "core/tgBasicActuator.h"
#include "core/tgRod.h"

#include "core/tgSpringCable.h"
#include "core/tgSpringCableAnchor.h"
#include "dev/kcaluwae/zeromq_superball/controllers/T6PIDController.h"


#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/assign/list_of.hpp>
#include "prx/utilities/definitions/string_manip.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::ntrt_plant_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    namespace sim
    {
        ntrt_plant_t::ntrt_plant_t() 
        {
            // joint_state_count = 0;
        }

        ntrt_plant_t::~ntrt_plant_t()
        {
            rigid_bodies.clear();
            for( unsigned i = 0; i < body_spaces.size(); ++i )
            {
                delete body_spaces[i];
            }
        }

        void ntrt_plant_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            quaternion_t rot;
          rot.set(vector_t(1,0,0),PRX_PI/2);
            rotation.zero();
            rotation.set_orientation(rot);
            myModel = new T6Model();

            T6PIDController::ControlMode control_mode = T6PIDController::POSITION;
            pTC = new T6PIDController(control_mode,100,0,0);
            myModel->attach(pTC);
            built_world = ntrt_world;
            myModel->setup(*built_world);

            std::vector<tgRod *> rods = myModel->find<tgRod>("rod");
            int index = 0;
            rigid_bodies.resize(rods.size());
            body_control_memory.clear();
            foreach(tgRod* rod, rods)
            {
                  std::stringstream ss;
                  ss << index;
                  rigid_bodies[index].first = pathname + "/rod" +ss.str();
                  PRX_INFO_S(rod->length()<<" "<<rigid_bodies[index].first );
                  rigid_bodies[index].second = rod->getPRigidBody();

                  rigid_bodies[index].second->forceActivationState(ACTIVE_TAG);

                  body_state_memory.push_back(std::vector<double*>());
                  for( int i = 0; i < 13; i++ )
                        body_state_memory.back().push_back(new double);

                  const btTransform& trans = rigid_bodies[index].second->getCenterOfMassTransform();
                  const btVector3& lin = rigid_bodies[index].second->getLinearVelocity();
                  const btVector3& ang = rigid_bodies[index].second->getAngularVelocity();
                  btQuaternion quat = trans.getRotation();
                  *body_state_memory.back()[ 0 ] = trans.getOrigin().getX();
                  *body_state_memory.back()[ 0 + 1 ] = trans.getOrigin().getY();
                  *body_state_memory.back()[ 0 + 2 ] = trans.getOrigin().getZ();
                  *body_state_memory.back()[ 0 + 3 ] = quat.getX();
                  *body_state_memory.back()[ 0 + 4 ] = quat.getY();
                  *body_state_memory.back()[ 0 + 5 ] = quat.getZ();
                  *body_state_memory.back()[ 0 + 6 ] = quat.getW();
                  *body_state_memory.back()[ 0 + 7 ] = lin.getX();
                  *body_state_memory.back()[ 0 + 8 ] = lin.getY();
                  *body_state_memory.back()[ 0 + 9 ] = lin.getZ();
                  *body_state_memory.back()[ 0 + 10 ] = ang.getX();
                  *body_state_memory.back()[ 0 + 11 ] = ang.getY();
                  *body_state_memory.back()[ 0 + 12 ] = ang.getZ();

                  space_t* space = new space_t("DynamicRigidBody", body_state_memory.back());
                  body_spaces.push_back(space);
                  index++;
            }


            muscles = myModel->getAllActuators();

            for (unsigned i = 0; i < 12; ++i)
            {
                  bounds_t new_bound(6,13);//TODO: Get the bounds from the actuator somehow
                  body_control_memory.push_back(std::vector<double*>());
                  body_control_memory.back().push_back(&motor_targets[i]);
                  *body_control_memory.back()[0] = muscles[i]->getRestLength();
                  body_control_spaces.push_back(new space_t("X", body_control_memory.back()));
                  ((space_t*)body_control_spaces.back())->set_bounds({&new_bound});

                  body_state_memory.push_back(std::vector<double*>());
                  body_state_memory.back().push_back(new double);
                  body_state_memory.back().push_back(new double);
                  body_state_memory.back().push_back(new double);
                  *body_state_memory.back()[0] = muscles[i]->getRestLength();
                  *body_state_memory.back()[1] = muscles[i]->getPreviousLength();
                  *body_state_memory.back()[2] = muscles[i]->getMotorVel();
                  space_t* space = new space_t("XYZ", body_state_memory.back());
                  body_spaces.push_back(space);

            }

            passive = myModel->find<tgBasicActuator> ("passive");

            for (unsigned i = 0; i < 12; ++i)
            {
                  body_state_memory.push_back(std::vector<double*>());
                  body_state_memory.back().push_back(new double);
                  body_state_memory.back().push_back(new double);
                  *body_state_memory.back()[0] = passive[i]->getPreviousLength();
                  *body_state_memory.back()[1] = passive[i]->getRestLength();
                  space_t* space = new space_t("XY", body_state_memory.back());
                  body_spaces.push_back(space);

            }

            //Now we can finally set up the state space!
            state_space = new space_t(body_spaces);

            foreach(std::vector<double*> inner, body_state_memory)
            {
                  foreach(double* val, inner)
                  {
                        state_memory.push_back(val);
                  }
            }

            input_control_space = new space_t(body_control_spaces);

            foreach(std::vector<double*> inner, body_control_memory)
            {
                  control_memory.push_back(inner.back());
            }
            update_to_bullet();

            PRX_INFO_S((*state_space));
            PRX_INFO_S((*input_control_space));

        }

        void ntrt_plant_t::update_phys_configs(config_list_t& configs, unsigned& index) const
        {
            for( unsigned i = 0; i < rigid_bodies.size(); ++i )
            {
                const btTransform& trans = rigid_bodies[i].second->getCenterOfMassTransform();
                augment_config_list(configs,index);
                configs[index].first = rigid_bodies[i].first;
                configs[index].second.set_position(trans.getOrigin().getX(), trans.getOrigin().getY(),trans.getOrigin().getZ());
                configs[index].second.set_orientation(trans.getRotation().getX(), trans.getRotation().getY(), trans.getRotation().getZ(), trans.getRotation().getW());
                configs[index].second.relative_to_global(rotation);
                index++;
            }
        }

        void ntrt_plant_t::verify() const { }

        void ntrt_plant_t::propagate(const double simulation_step)
        {
            // int index =0;
            // foreach(tgKinematicActuator* act, muscles)
            // {
            //   // double val = act->getRestLength();
            //   double input = *control_memory[index];
            //   // input = val+(input - val)*.2;
            //   // if(val - 1.5 > input)
            //   //   input= val-1.5;
            //   // else if(val + 1.5 < input)
            //   //   input= val+1.5;
            //   // if(val!=0)
            //   // {
            //   //   PRX_INFO_S(val);
            //   //   exit(0);
            //   // }
            //   // val+=PRX_SIGN(*control_memory[index])*.25;
            //   // if(val < 3.0)
            //   //   val = 3.0;
            //   // if(val > 9.0)
            //   //   val = 9.0;
            //   act->setControlInput(input);
            //   index++;
            // }
            // for (int i = 0; i < 12; ++i)
            // {
            //   std::cout<<motor_targets[i]<<" ";
            // }
            // std::cout<<std::endl;
            pTC->setTarget(motor_targets);
            myModel->step(simulation_step);
        }

        void ntrt_plant_t::set_active(bool in_active, const std::string& path)
        {
            active = in_active;
        }

        void ntrt_plant_t::add_bodies()
        {
          // myModel->attach(pTC);
          myModel->setup(*built_world);
          std::vector<tgRod *> rods = myModel->find<tgRod>("rod");
          muscles = myModel->getAllActuators();
          passive = myModel->find<tgBasicActuator> ("passive");
          int index = 0;
          foreach(tgRod* rod, rods)
          {
            rigid_bodies[index].second = rod->getPRigidBody();
            index++;
          }
        }
        void ntrt_plant_t::teardown()
        {
          myModel->teardown();
        }

        void ntrt_plant_t::update_to_bullet()
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

            int index = i*13;
            foreach(tgKinematicActuator* act, muscles)
            {
                  act->setRestLength(*state_memory[index]);
                  index++;
                  act->setPreviousLength(*state_memory[index]);
                  index++;
                  act->setMotorVel(*state_memory[index]);
                  index++;
            }
            foreach(tgBasicActuator* act, passive)
            {
                  act->setPreviousLength(*state_memory[index]);
                  index++;
                  act->setRestLength(*state_memory[index]);
                  index++;
            }
        }

        void ntrt_plant_t::update_from_bullet()
        {
            // PRX_INFO_S(state_space->print_memory(4));
            for( unsigned i = 0; i < rigid_bodies.size(); ++i )
            {
                // rigid_bodies[i].second->forceActivationState(ACTIVE_TAG);
                // rigid_bodies[i].second->setDeactivationTime(0);
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

            int index = rigid_bodies.size()*13;
            foreach(tgKinematicActuator* act, muscles)
            {
                  // PRX_INFO_S(act->getRestLength());getPreviousLength
                  *state_memory[index] = act->getRestLength();
                  index++;
                  *state_memory[index] = act->getPreviousLength();
                  index++;
                  *state_memory[index] = act->getMotorVel();

                  index++;
            }
            foreach(tgBasicActuator* act, passive)
            {
                  // PRX_INFO_S(act->getRestLength());getPreviousLength
                  *state_memory[index] = act->getPreviousLength();
                  index++;
                  *state_memory[index] = act->getRestLength();
                  index++;
            }

            // if(((comm::visualization_comm_t*)comm::vis_comm)!=NULL)
            // {
            //   com.resize(com.size()+1);
            //   for(unsigned i=0;i<3;i++)
            //   {
            //       double val = 0;
            //       for(unsigned j=0;j<6;j++)
            //       {
            //           val += *state_memory[j*13+i];
            //       }
            //       val/=6;
            //       com.back().push_back(val);
            //   }
            //   double temp = com.back()[1];
            //   com.back()[1] = -com.back()[2];
            //   com.back()[2] = temp;
            // }


        }


        void ntrt_plant_t::update_vis_info() const
        {
            int index = 0;
            foreach(tgKinematicActuator* act, muscles)
            {
              std::vector<util::geometry_info_t> geoms;
              std::vector<util::config_t> configs;
              std::vector<double> params;
              btVector3 root_link_pos = act->getSpringCable()->getAnchors()[0]->getWorldPosition();
              btVector3 child_link_pos = act->getSpringCable()->getAnchors()[1]->getWorldPosition();

              params.push_back(root_link_pos.getX());
              params.push_back(-root_link_pos.getZ());
              params.push_back(root_link_pos.getY());
              params.push_back(child_link_pos.getX());
              params.push_back(-child_link_pos.getZ());
              params.push_back(child_link_pos.getY());
              geoms.push_back(util::geometry_info_t("tensegrity", "/"+int_to_str(index), util::PRX_LINESTRIP, params, "white"));
              configs.push_back( util::config_t() );
              params.clear();
              ((comm::visualization_comm_t*)comm::vis_comm)->visualization_geom_map["tensegrity/"+int_to_str(index)] = geoms;
              ((comm::visualization_comm_t*)comm::vis_comm)->visualization_configs_map["tensegrity/"+int_to_str(index)] = configs;
              index++;
            }
            foreach(tgBasicActuator* act, passive)
            {
              std::vector<util::geometry_info_t> geoms;
              std::vector<util::config_t> configs;
              std::vector<double> params;
              btVector3 root_link_pos = act->getSpringCable()->getAnchors()[0]->getWorldPosition();
              btVector3 child_link_pos = act->getSpringCable()->getAnchors()[1]->getWorldPosition();

              params.push_back(root_link_pos.getX());
              params.push_back(-root_link_pos.getZ());
              params.push_back(root_link_pos.getY());
              params.push_back(child_link_pos.getX());
              params.push_back(-child_link_pos.getZ());
              params.push_back(child_link_pos.getY());
              geoms.push_back(util::geometry_info_t("tensegrity", "/"+int_to_str(index), util::PRX_LINESTRIP, params, "grey"));
              configs.push_back( util::config_t() );
              params.clear();
              ((comm::visualization_comm_t*)comm::vis_comm)->visualization_geom_map["tensegrity/"+int_to_str(index)] = geoms;
              ((comm::visualization_comm_t*)comm::vis_comm)->visualization_configs_map["tensegrity/"+int_to_str(index)] = configs;
              index++;
            }


            // std::vector<util::geometry_info_t> geoms;
            // std::vector<util::config_t> configs;
            // std::vector<double> params;
            // for (std::vector<std::vector<double> >::const_iterator iter = com.begin(); iter != com.end(); ++iter)
            // {
            //   params.push_back((*iter)[0]);
            //   params.push_back((*iter)[1]);
            //   params.push_back((*iter)[2]);
            // }
            // geoms.push_back(util::geometry_info_t("tensegrity", "/com", util::PRX_LINESTRIP, params, "green"));
            // configs.push_back( util::config_t() );
            // ((comm::visualization_comm_t*)comm::vis_comm)->visualization_geom_map["tensegrity/com"] = geoms;
            // ((comm::visualization_comm_t*)comm::vis_comm)->visualization_configs_map["tensegrity/com"] = configs;


            // ((comm::visualization_comm_t*)comm::vis_comm)->send_geometries();
        }
    }
}
#endif

