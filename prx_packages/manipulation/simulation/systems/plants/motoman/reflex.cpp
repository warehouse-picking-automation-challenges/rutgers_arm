/**
 * @file reflex.cpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2016, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "simulation/systems/plants/motoman/reflex.hpp"

#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/simulation/simulators/simulator.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"

#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/assign/list_of.hpp>
#include <boost/range/adaptor/map.hpp> //adaptors
#include <pluginlib/class_list_macros.h>
#include <urdf/model.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <sys/param.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::reflex_t, prx::sim::system_t)

namespace prx
{
    using namespace sim;
    using namespace util;
    namespace packages
    {
        namespace manipulation
        {
            reflex_t::reflex_t()
            {
            }

            void reflex_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                manipulator_t::init(reader, template_reader);   

                std::vector<std::string> reflex_gripper_names = {"swivel_1","proximal_1","distal_1","swivel_2","proximal_2","distal_2","proximal_3","distal_3"};
                foreach(std::string name, reflex_gripper_names)
                {
                    std::deque<manip_node_t*> queue;
                    queue.push_back(manip_tree);
                    bool not_found = true;
                    while(queue.size()!=0 && not_found)
                    {
                        manip_node_t* popped = queue.front();
                        queue.pop_front();
                        if(popped->kdl_node->first == name)
                        {
                            not_found = false;
                            reflex_gripper_angles.push_back(popped->memory);
                        }
                        else
                        {
                            queue.insert(queue.end(),popped->children.begin(),popped->children.end());
                        }
                    }
                }

                KDL::Chain chain1;
                kdl_tree->getChain(kdl_tree->getRootSegment()->first, reverse_split_path(effector_names[0]).second, chain1);          
                KDL::JntArray q(chain1.getNrOfJoints());  

                KDL::ChainFkSolverPos_recursive fk_solver(chain1);
                fk_solver.JntToCart(q,static_transform); 

            }


            void reflex_t::update_hand_geometries() const
            {
                switch( ((int)end_effector_states[0]))
                {
                    case 2:
                    case 3:
                        *reflex_gripper_angles[0] = 0;
                        *reflex_gripper_angles[1] = 1.05;
                        *reflex_gripper_angles[2] = 0.7;
                        *reflex_gripper_angles[3] = 0;
                        *reflex_gripper_angles[4] = 1.05;
                        *reflex_gripper_angles[5] = 0.7;
                        *reflex_gripper_angles[6] = 1.05;
                        *reflex_gripper_angles[7] = 0.7;
                        break;
                    case 4:
                    case 5:
                        *reflex_gripper_angles[0] = 1.5;
                        *reflex_gripper_angles[1] = 1.05;
                        *reflex_gripper_angles[2] = 0.7;

                        *reflex_gripper_angles[3] = -1.5;
                        *reflex_gripper_angles[4] = 1.05;
                        *reflex_gripper_angles[5] = 0.7;

                        *reflex_gripper_angles[6] = 2.8;
                        *reflex_gripper_angles[7] = 0.7;
                        break;
                    default:
                        *reflex_gripper_angles[0] = 0;
                        *reflex_gripper_angles[1] = 0;
                        *reflex_gripper_angles[2] = 0;
                        *reflex_gripper_angles[3] = 0;
                        *reflex_gripper_angles[4] = 0;
                        *reflex_gripper_angles[5] = 0;
                        *reflex_gripper_angles[6] = 0;
                        *reflex_gripper_angles[7] = 0;
                        break;

                }             
            }

            void reflex_t::update_collision_info()
            {
                util::quaternion_t manip_orientation;
                manip_orientation.set_from_euler(_roll, _pitch, _yaw);
                double _qx, _qy, _qz, _qw;
                manip_orientation.get(_qx, _qy, _qz, _qw);
                root_frame = KDL::Frame(KDL::Rotation::Quaternion(_qx,_qy,_qz,_qw),KDL::Vector(_x, _y, _z));
                update_hand_geometries();
                manipulator_t::update_collision_info();     
            }

            void reflex_t::update_phys_configs(config_list_t& configs, unsigned& index) const
            {
                util::quaternion_t manip_orientation;
                manip_orientation.set_from_euler(_roll, _pitch, _yaw);
                double _qx, _qy, _qz, _qw;
                manip_orientation.get(_qx, _qy, _qz, _qw);
                root_frame = KDL::Frame(KDL::Rotation::Quaternion(_qx,_qy,_qz,_qw),KDL::Vector(_x, _y, _z));
                update_hand_geometries();
                manipulator_t::update_phys_configs(configs,index);    
            }

            void reflex_t::FK_solver( config_t& goal_config, std::string link_name)
            {
                util::quaternion_t manip_orientation;
                manip_orientation.set_from_euler(_roll, _pitch, _yaw);
                double _qx, _qy, _qz, _qw;
                manip_orientation.get(_qx, _qy, _qz, _qw);
                root_frame = KDL::Frame(KDL::Rotation::Quaternion(_qx,_qy,_qz,_qw),KDL::Vector(_x, _y, _z));
                update_hand_geometries();
                manipulator_t::FK_solver( goal_config, link_name);
            }

        }
    }
}
