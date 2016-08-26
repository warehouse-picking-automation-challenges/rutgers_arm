/**
 * @file motoman.cpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "simulation/systems/plants/motoman/motoman.hpp"

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
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::motoman_t, prx::sim::system_t)

namespace prx
{
    using namespace sim;
    using namespace util;
    namespace packages
    {
        namespace manipulation
        {
            motoman_t::motoman_t()
            {
                input_path = pracsys_path + "/prx_packages/manipulation/input/motoman/";
            }

            void motoman_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
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
            }

            void motoman_t::create_spaces()
            {
                state_space = new space_t("Motoman", state_memory);
                input_control_space = new space_t("Motoman", control_memory);
            }

            void motoman_t::update_collision_info()
            {
                update_right_hand();
                manipulator_t::update_collision_info();     
            }

            void motoman_t::update_phys_configs(config_list_t& configs, unsigned& index) const
            {
                update_right_hand();
                manipulator_t::update_phys_configs(configs,index);    
            }

            bool motoman_t::is_end_effector_closed(int index) const
            {
                if(index==0)
                {
                    return (end_effector_states[index] == 2);
                }
                else if(index==1)
                {
                    return (end_effector_states[index] == 2) || (end_effector_states[index] == 5);
                }
                return false;

            }

            void motoman_t::update_right_hand() const
            {
                switch( ((int)end_effector_states[1]))
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

        }
    }
}
