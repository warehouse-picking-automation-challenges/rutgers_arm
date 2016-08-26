/**
 * @file motoman_parallel_unigripper.cpp 
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

#include "simulation/systems/plants/motoman/motoman_parallel_unigripper.hpp"

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

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::motoman_parallel_unigripper_t, prx::sim::system_t)

namespace prx
{
    using namespace sim;
    using namespace util;
    namespace packages
    {
        namespace manipulation
        {
            motoman_parallel_unigripper_t::motoman_parallel_unigripper_t()
            {
                input_path = pracsys_path + "/prx_packages/manipulation/input/motoman/";
            }

            void motoman_parallel_unigripper_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                manipulator_t::init(reader, template_reader);   

                std::vector<std::string> unigripper_names = {"right_movable_pipe","left_movable_pipe"};
                foreach(std::string name, unigripper_names)
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
                            parallel_offsets.push_back(popped->memory);
                        }
                        else
                        {
                            queue.insert(queue.end(),popped->children.begin(),popped->children.end());
                        }
                    }
                }
            }

            void motoman_parallel_unigripper_t::create_spaces()
            {
                state_space = new space_t("Motoman_Parallel_Unigripper", state_memory);
                input_control_space = new space_t("Motoman_Parallel_Unigripper", control_memory);
            }

            void motoman_parallel_unigripper_t::update_collision_info()
            {
                update_right_hand();
                manipulator_t::update_collision_info();     
            }

            void motoman_parallel_unigripper_t::update_phys_configs(config_list_t& configs, unsigned& index) const
            {
                update_right_hand();
                manipulator_t::update_phys_configs(configs,index);    
            }

            // EE0: Head Sponge, 1 not grasping, 2 grasping
            // EE1: Front Vacuum, 1 not grasping, 2 grasping
            // EE2: Upper Vacuum, 1 not grasping, 2 grasping
            // EE3: Parallel Gripper, 1 not grasping and fingers are completely closed
            //                        1-2 grasping and fingers partially closed
            //                        2 grasping and fingers completely open
            //                        3 not grasping and fingers completely open



            bool motoman_parallel_unigripper_t::is_end_effector_closed(int index) const
            {
                if(index==0 || index == 1 || index == 2)
                {
                    return (end_effector_states[index] == 2);
                }
                else if(index==3)
                {
                    //The parallel gripper is only in a grasping state for values between 1 and 2
                    if (end_effector_states[index] > 1.001 && end_effector_states[index] < 2.001)
                    {
                        return true;
                    }
                }
                return false;

            }

            void motoman_parallel_unigripper_t::update_right_hand() const
            {
                // if ( ((int)end_effector_states[3]) == 2 ||  ((int)end_effector_states[3]) == 3 )
                //If parallel gripper state is 2 keep it completely open
                if ( fabs(end_effector_states[3] - 2.0) < 0.001 )
                {
                    // PRX_STATUS_S("Unigripper thinks it is open...");
                    *parallel_offsets[0] = 0.003;
                    *parallel_offsets[1] = 0.0;                    
                }
                //If parallel gripper state is 3, keep it completely open
                else if( fabs(end_effector_states[3] - 3.0) < 0.001 )
                {
                    *parallel_offsets[0] = 0.003;
                    *parallel_offsets[1] = 0.0; 
                }
                //If parallel gripper state is between 1 and 2, partially close the fingers
                else if( end_effector_states[3] > 1.001 && end_effector_states[3] < 2.0 )
                {
                    double delta = 2 - end_effector_states[3];

                    *parallel_offsets[0] = 0.003 + (delta * 0.054);
                    *parallel_offsets[1] = 0.0 + (delta * 0.060); 
                    // PRX_PRINT("Parallel geoms: "<<(*parallel_offsets[0])<<", "<<(*parallel_offsets[0]), PRX_TEXT_GREEN);
                }
                //If the parallel gripper state is in any other range, keep the fingers closed
                else
                {
                    // PRX_STATUS_S("Unigripper thinks it is closed...");
                    *parallel_offsets[0] = 0.057;
                    *parallel_offsets[1] = 0.06;
                }            
            }

            void motoman_parallel_unigripper_t::propagate(const double simulation_step)
            {
                manipulator_t::propagate(simulation_step);
                if(input_control_space->at(19)!=0)
                {
                    end_effector_states[3] = input_control_space->at(19);
                }
            }

        }
    }
}
