/**
 * @file application.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/simulation/systems/plants/plant.hpp"
#include "prx/simulation/collision_checking/vector_collision_list.hpp"
#include "prx/simulation/systems/obstacle.hpp"

#include "prx/simulation/communication/sim_base_communication.hpp"
#include "prx/simulation/communication/planning_comm.hpp"
#include "prx/simulation/communication/simulation_comm.hpp"
#include "prx/simulation/communication/visualization_comm.hpp"
#include "prx/utilities/communication/tf_broadcaster.hpp"

#include <boost/range/adaptor/map.hpp> //adaptors
#include <fstream>
#include "prx/utilities/definitions/sys_clock.hpp"

#include "simulation/apc_generate_bin_traj.hpp"
#include <pluginlib/class_list_macros.h>
#include <map>

#ifdef PRX_SENSING_FOUND
#include "prx_sensing/UpdateObjectList.h"
#include "prx_sensing/PublishObjectList.h"
#include "prx_sensing/UpdateShelfPosition.h"
#endif

#include "prx_simulation/gripper_change_srv.h"
#include "prx_simulation/UnigripperVacuumOn.h"

 //Json Parser
#include <simulation/json/read_file.h>




PLUGINLIB_EXPORT_CLASS(prx::packages::apc::apc_generate_bin_traj_t, prx::sim::application_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace sim::comm;

    namespace packages
    {
        using namespace manipulation;
        namespace apc
        {
            const std::string dof_names[16] = {"torso_joint_b1", "arm_left_joint_1_s", "arm_left_joint_2_l", "arm_left_joint_3_e", "arm_left_joint_4_u", "arm_left_joint_5_r", "arm_left_joint_6_b", "arm_left_joint_7_t", "arm_right_joint_1_s", "arm_right_joint_2_l", "arm_right_joint_3_e", "arm_right_joint_4_u", "arm_right_joint_5_r", "arm_right_joint_6_b", "arm_right_joint_7_t", "torso_joint_b2"};
            apc_generate_bin_traj_t::apc_generate_bin_traj_t()
            {
                tf_broadcaster = NULL;
                simulator_running = false;
                simulator_counter = 0;
                simulator_mode = 0;
                loop_counter = 0;
                loop_total = 0.0;
                new_geometries_info = false;
                selected_point.resize(3);
                selected_path.clear();
                collision_list = new vector_collision_list_t();

                stores_states = replays_states = false;


            }

            apc_generate_bin_traj_t::~apc_generate_bin_traj_t()
            {
                delete tf_broadcaster;
                delete plan_comm;
                delete sim_comm;
                delete vis_comm;
            }

            void apc_generate_bin_traj_t::init(const parameter_reader_t * const reader)
            {
                Init_Finished = false;
                cur_hand = "left";
                PRX_INFO_S("Initializing APC Simulation Application");
                current_item_index = 0;
                number_of_orders = 0;
                current_bin_name = 0;
                Got_Item = false;
                //getting the subsystems
                automaton_state = START;
                displayed_state = false;
                execution_time =0.0;
                plan_duration = 0.0;
                currently_in_home_position = false;
                executing_trajectory = false;
                counter = 0;
                planning_app_query = new planning_app_query_client("prx/apc_action",false);
                controller_query = new send_traj_client("prx/execute_plan_action",false);

                simulation::update_all_configs = true;
                std::string plugin_type_name;
                plugin_type_name = reader->get_attribute("sim_to_plan_comm", "planning_comm");
                comm::plan_comm = sim_base_communication_t::get_loader().createUnmanagedInstance("prx_simulation/" + plugin_type_name);
                plan_comm->link_application(this);

                plugin_type_name = reader->get_attribute("sim_comm", "simulation_comm");
                sim_comm = sim_base_communication_t::get_loader().createUnmanagedInstance("prx_simulation/" + plugin_type_name);
                sim_comm->link_application(this);


                PRX_PRINT("Initializing the application", PRX_TEXT_GREEN);
                simulator = reader->initialize_from_loader<simulator_t > ("simulator", "prx_simulation");
                simulator->update_system_graph(sys_graph);
                sys_graph.get_name_system_hash(subsystems);

                foreach(system_ptr_t plant, subsystems | boost::adaptors::map_values)
                {
                    // PRX_PRINT("plants iteration",PRX_TEXT_GREEN);
                    //Create and initialize information for all the end effectors. 
                    if(dynamic_cast<manipulator_t*>(plant.get()) != NULL)
                    {
                        //Create end effector infos for each end effector.
                        manipulator = dynamic_cast<manipulator_t*>(plant.get());
                    }

                    //Create a list of all the movable bodies.
                    if(dynamic_cast<movable_body_plant_t*>(plant.get()) != NULL)
                    {
                        movable_bodies.push_back(dynamic_cast<movable_body_plant_t*>(plant.get()));
                    }

                    //Create a controller
                    if ( dynamic_cast<apc_controller_t*> (plant.get()) != NULL )
                    {
                       PRX_INFO_S(" Found a controller!");
                       controller = dynamic_cast<apc_controller_t*> (plant.get());
                    }
                }
                visualize_plans = reader->get_attribute_as<bool>("application/visualize_plans", false);
                has_dynamic_obstacles = reader->get_attribute_as<bool>("application/has_dynamic_obstacles", true);
                visualize = reader->get_attribute_as<bool>("application/visualize", true);
                screenshot_every_simstep = reader->get_attribute_as<bool>("application/screenshot_every_simstep", false);
                simulator_running = reader->get_attribute_as<bool>("application/start_simulation",false);
                jk_prm_star = reader->get_attribute_as<bool>("application/jk_prm_star",false);

                simulation_control_space = simulator->get_control_space();
                simulation_state_space = simulator->get_state_space();
                simulation_state = simulation_state_space->alloc_point();
                simulation_control = simulation_control_space->alloc_point();

                deserialize_sim_file = reader->get_attribute_as<std::string > ("application/deserialize_sim_file", "");
                if( !deserialize_sim_file.empty() )
                {
                    this->replays_states = true;
                    replay_simulation_states.link_space(simulation_state_space);
                    deserialize_simulation_states(deserialize_sim_file, replay_simulation_states);
                }

                serialize_sim_file = reader->get_attribute_as<std::string > ("application/serialize_sim_file", "");
                if( !serialize_sim_file.empty() )
                {
                    store_simulation_states.link_space(simulation_state_space);
                    this->stores_states = true;
                }

                PRX_ASSERT(simulator != NULL);
                //get obstacles
                obstacles_hash = simulator->get_obstacles();
                foreach(system_ptr_t obstacle, obstacles_hash | boost::adaptors::map_values)
                {
                    if(dynamic_cast<sim::obstacle_t*>(obstacle.get()) != NULL)
                    {
                        obstacles.push_back(dynamic_cast<sim::obstacle_t*>(obstacle.get()));
                    }
                }
                tf_broadcaster = new tf_broadcaster_t;

                sim_key_name = "prx/input_keys/" + int_to_str(PRX_KEY_RETURN);
                if( !ros::param::has(sim_key_name) )
                    ros::param::set(sim_key_name, false);

                simulator->update_system_graph(sys_graph);
                
                sys_graph.get_plant_paths(plant_paths);
                if( visualize )
                {
                    plugin_type_name = reader->get_attribute("sim_to_vis_comm", "visualization_comm");
                    vis_comm = sim_base_communication_t::get_loader().createUnmanagedInstance("prx_simulation/" + plugin_type_name);
                    vis_comm->link_application(this);
                    vis_comm->send_plants(ros::this_node::getName(), plant_paths);
                    ((visualization_comm_t*)vis_comm)->send_rigid_bodies();
                }

                init_collision_list(reader->get_child("application").get());

                PRX_DEBUG_COLOR(" collision_list has been initialized ", PRX_TEXT_GREEN);
                
                hash_t<std::string, int> comm_systems;

                std::vector<plant_t*> get_plants;
                //SGC: only need plant pointers here
                sys_graph.get_plants(get_plants);

                foreach(plant_t* sys, get_plants)
                {
                    // PRX_DEBUG_COLOR("Plant: " << sys->get_pathname(), PRX_TEXT_GREEN);
                    if( !sys->is_active() && visualize )
                    {
                        vis_comm->visualize_plant(sys->get_pathname(), PRX_VIS_TEMPORARY_REMOVE);
                    }
                }
                
                if (simulator->get_sensing_model() != NULL)
                {
                    PRX_WARN_S ("Sensing model not null, proceeding with initialization of sensors.");
                    initialize_sensing();
                }
                else
                {
                    PRX_ERROR_S ("Sensing model is null, cannot initialize sensors.");
                }

                if( visualize )
                {
                    vis_comm->publish_markers();
                }

                visualization_multiple = ceil(.02/simulation::simulation_step);
                visualization_counter = 0;

                object_publisher = n.advertise<prx_simulation::object_msg>("prx/object_state", 100);

                bin_names.push_back("A");
                bin_names.push_back("B");
                bin_names.push_back("C");
                bin_names.push_back("D");
                bin_names.push_back("E");
                bin_names.push_back("F");
                bin_names.push_back("G");
                bin_names.push_back("H");
                bin_names.push_back("I");
                bin_names.push_back("J");
                bin_names.push_back("K");
                bin_names.push_back("L");
                

                child_state_space = manipulator->get_state_space();
                real_robot = reader->get_attribute_as<bool>("application/real_robot",false);
                if(real_robot)
                {
                    real_robot_state = n.subscribe("/joint_states",4,&apc_generate_bin_traj_t::real_robot_state_callback,this);
                    robot_state = child_state_space->alloc_point();
                
                    name_index_map["torso_joint_b1"] = 0;
                    name_index_map["torso_joint_b2"] = 0;

                    name_index_map["arm_left_joint_1_s"] = 1;
                    name_index_map["arm_left_joint_2_l"] = 2;
                    name_index_map["arm_left_joint_3_e"] = 3;
                    name_index_map["arm_left_joint_4_u"] = 4;
                    name_index_map["arm_left_joint_5_r"] = 5;
                    name_index_map["arm_left_joint_6_b"] = 6;
                    name_index_map["arm_left_joint_7_t"] = 7;
                    name_index_map["head_hinge"] = 8;

                    name_index_map["arm_right_joint_1_s"] = 9;
                    name_index_map["arm_right_joint_2_l"] = 10;
                    name_index_map["arm_right_joint_3_e"] = 11;
                    name_index_map["arm_right_joint_4_u"] = 12;
                    name_index_map["arm_right_joint_5_r"] = 13;
                    name_index_map["arm_right_joint_6_b"] = 14;
                    name_index_map["arm_right_joint_7_t"] = 15;
                    robot_ac = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("joint_trajectory_action", true);
                
                    unigripper_ac = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("unigripper/unigripper_joint_trajectory_action", true);

                    unigripper_command.trajectory.joint_names.push_back("head_hinge");
                    for( int i = 0; i < 16; i++ )
                        robot_command.trajectory.joint_names.push_back(dof_names[i]);
                
                }
                controller->real_robot = real_robot;
                output_control_space = controller->get_output_control_space();
                state_publisher = n.advertise<prx_simulation::state_msg>("prx/manipulator_state", 1);
                Init_Finished = true;
            }

            void apc_generate_bin_traj_t::real_robot_state_callback(const sensor_msgs::JointState& stateMsg)
            {
                child_state_space->copy_to_point(robot_state);
                sensor_msgs::JointState start_state = stateMsg;
                for( unsigned i = 0; i < start_state.name.size(); i++ )
                {
                    robot_state->at(name_index_map[start_state.name[i]]) = start_state.position[i];
                }
                child_state_space->copy_from_point(robot_state);
                publish_state();
               
                
            }

            void apc_generate_bin_traj_t::publish_state()
            {
                prx_simulation::state_msg msg;
                for(unsigned i=0;i<child_state_space->get_dimension();++i)
                {
                    msg.elements.push_back(child_state_space->at(i));
                }
                state_publisher.publish(msg);
            }

            void apc_generate_bin_traj_t::frame(const ros::TimerEvent& event)
            {
                handle_key();

                if( simulator_mode == 1 )
                {
                    if( simulator_counter > 0 )
                    {
                        simulator_running = true;
                        simulator_counter--;
                        loop_timer.reset();
                    }
                    else
                        simulator_running = false;
                }
                if( loop_timer.measure() > 1.0 )
                    loop_timer.reset();
                if( simulator_running & Init_Finished)
                {
                    prx_planning::apc_queryGoal command;
                    prx_planning::apc_queryResult result;
                    prx_simulation::send_trajectoryGoal execute_command;

                    if(executing_trajectory == true && !real_robot)
                    {
                        if(execution_time<plan_duration*0.6)
                        {
                            PRX_STATUS("Executing Trajectory"<<execution_time <<"/"<<plan_duration,PRX_TEXT_CYAN);
                            execution_time+=simulation::simulation_step;
                            // update_object_poses(command.object);
                            update_simulation();
                        }
                        else
                        {
                            plan_duration = 0.0;
                            execution_time = 0.0;
                            executing_trajectory = false;
                            PRX_PRINT("Trajectory executed",PRX_TEXT_BROWN);
                        }
                    }
                    else if( automaton_state == START)
                    {
                        reset_object_pose();
                        PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state),PRX_TEXT_CYAN);

                        if(!currently_in_home_position)
                        {
                            automaton_state = MOVE_TO_HOME;
                        }
                        else
                        {
                            // automaton_state = PLAN_BIN_TRAJECTORIES;
                            automaton_state = PLAN_BIN_TRAJECTORIES_3_STAGES_FIRST;
                        }
                    } 
                    else if( automaton_state == MOVE_TO_HOME)
                    {
                        PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state),PRX_TEXT_CYAN);
                        // reset_object_pose();
                        std::vector<double> home_position={0.00000,1.57000,0.00000,0.00000,-1.70000,0.00000,0.00000,0.00000,0.00000,1.57000,0.00000,0.00000,-1.70000,0.00000,0.00000,0.00000,1.00000,1.00000,1.00000,1.00000};
                        command.goal_state = home_position;
                        //0 means MOVE (refer to apc_query.action)
                        command.stage = 0;
                        // PRX_PRINT(command.goal_state,PRX_TEXT_BROWN);
                        planning_app_query->sendGoal(command);
                        bool planning_success = false;
                        planning_success = planning_app_query->waitForResult(ros::Duration(0));
                        actionlib::SimpleClientGoalState state = planning_app_query->getState();
                        result = *planning_app_query->getResult();
                        if(state.toString() == "SUCCEEDED" && result.plan.size()>0)
                        {
                            PRX_PRINT("Received Plan Successfully...",PRX_TEXT_CYAN);
                            execute_command.plan=result.plan;
                            controller->convert_and_copy_to_robot_plan(execute_command.plan);
                            controller->set_robot_plan();
                            //ZPP
                            if(real_robot)
                            {
                                create_and_send_robot_command();
                            }
                            


                            plan_duration = result.duration;
                            executing_trajectory = true;
                            PRX_PRINT("Executing Plan...",PRX_TEXT_CYAN);
                            currently_in_home_position = true;
                        }
                        else if (state.toString() == "ABORTED")
                        {
                            PRX_PRINT("Planning failed to provide a plan!",PRX_TEXT_RED);
                        }
                        if(result.plan.size()==0 & state.toString() == "SUCCEEDED")
                        {
                            PRX_PRINT("Already in queried position",PRX_TEXT_CYAN);
                            currently_in_home_position = true;
                        }
                        // planning srv
                        automaton_state = START;
                    }
                    else if( automaton_state == END)
                    {
                        if(cur_hand=="left")
                        {
                            cur_hand="right";
                            current_bin_name = 0;
                            automaton_state = START;
                        }
                        else
                        {
                            PRX_PRINT("Finished all bins", PRX_TEXT_GREEN);
                        }
                    }
                    else if(automaton_state == PLAN_BIN_TRAJECTORIES)
                    {
                        PRX_PRINT("current_bin_name : "<<current_bin_name,PRX_TEXT_RED);
                        for(int i = current_bin_name; i<= bin_names.size(); i++){
                            PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state)<<"BIN: "<<bin_names.at(i),PRX_TEXT_CYAN);
                            PRX_PRINT("@SIMULATION: Initial: "<<simulator->get_state_space()->print_point(simulation_state,2),PRX_TEXT_CYAN);
                            command.stage = 1;
                            command.bin = bin_names.at(i);
                            command.hand = cur_hand;
                            PRX_PRINT(command.saved_plan_path,PRX_TEXT_BROWN);
                            double bin_plan_duration = 100;
                            jk_prm_star ? thresh = 1000 : thresh = 6;
                            bool planning_success = false;
                            int tries_counter = 0;
                            while(bin_plan_duration>thresh)
                            {
                                bin_plan_duration = 10000;
                                planning_app_query->sendGoal(command);
                                planning_success = planning_app_query->waitForResult(ros::Duration(0));
                                result = *planning_app_query->getResult();
                                tries_counter++;
                                if(tries_counter%4==0)
                                {
                                    thresh+=.5;
                                }
                                if(planning_success && result.plan.size()>0)
                                {
                                    bin_plan_duration = result.duration;
                                    PRX_PRINT("Plan Duration: "<<bin_plan_duration<<"("<<thresh<<")"<<"   Bin_Hand: "<<cur_hand<<"_"<<bin_names.at(i),PRX_TEXT_GREEN);
                                }
                            }
                            if(planning_success && result.plan.size()>0)
                            {
                                jk_prm_star ? thresh = 1000 : thresh = 6;
                                PRX_PRINT("Received Plan Successfully...",PRX_TEXT_CYAN);
                                execute_command.plan=result.plan;
                        

                                controller->convert_and_copy_to_robot_plan(execute_command.plan);
                                controller->set_robot_plan();
                                
                                //ZPP
                                if(real_robot)
                                {
                                    create_and_send_robot_command();
                                }

                                plan_duration = result.duration;
                                executing_trajectory = true;
                                PRX_PRINT("Executing Plan...",PRX_TEXT_CYAN);
                                currently_in_home_position = false;
                            }
                            automaton_state = START;
                            current_bin_name ++;
                            if(i == bin_names.size()-1 )
                            {   
                                automaton_state = END;
                            }
                            break;
                        }


                    }
                    else if(automaton_state == PLAN_BIN_TRAJECTORIES_3_STAGES_FIRST)
                    {
                        PRX_PRINT("current_bin_name : "<<current_bin_name,PRX_TEXT_RED);
                        for(int i = current_bin_name; i<= bin_names.size(); i++)
                        {
                            PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state)<<"BIN: "<<bin_names.at(i),PRX_TEXT_CYAN);
                            PRX_PRINT("@SIMULATION: Initial: "<<simulator->get_state_space()->print_point(simulation_state,2),PRX_TEXT_CYAN);
                            command.stage = 1;
                            command.bin = bin_names.at(i);
                            command.hand = cur_hand;
                            PRX_PRINT(command.saved_plan_path,PRX_TEXT_BROWN);
                            double bin_plan_duration = 10000;
                            jk_prm_star ? thresh = 1000 : thresh = 6;
                            bool planning_success = false;
                            int tries_counter = 0;
                            while(bin_plan_duration>thresh)
                            {
                                bin_plan_duration = 10000;
                                planning_app_query->sendGoal(command);
                                planning_success = planning_app_query->waitForResult(ros::Duration(0));
                                result = *planning_app_query->getResult();
                                tries_counter++;
                                if(tries_counter%4==0)
                                {
                                    thresh+=.5;
                                }
                                if(planning_success && result.plan.size()>0)
                                {
                                    bin_plan_duration = result.duration;
                                    PRX_PRINT("Plan Duration: "<<bin_plan_duration<<"("<<thresh<<")"<<"   Bin_Hand: "<<cur_hand<<"_"<<bin_names.at(i),PRX_TEXT_GREEN);
                                }
                            }
                            if(planning_success && result.plan.size()>0)
                            {
                                jk_prm_star ? thresh = 1000 : thresh = 6;
                                PRX_PRINT("Received Plan Successfully...",PRX_TEXT_CYAN);
                                execute_command.plan=result.plan;
                        

                                controller->convert_and_copy_to_robot_plan(execute_command.plan);
                                controller->set_robot_plan();
                                
                                //ZPP
                                if(real_robot)
                                {
                                    create_and_send_robot_command();
                                }

                                plan_duration = result.duration;
                                executing_trajectory = true;
                                PRX_PRINT("Executing Plan...",PRX_TEXT_CYAN);
                                currently_in_home_position = false;
                            }
                            //3-stage
                            automaton_state = PLAN_BIN_TRAJECTORIES_3_STAGES_SECOND;
                            //2-stage
                            automaton_state = PLAN_BIN_TRAJECTORIES_3_STAGES_THIRD;
                            break;
                        }


                    }
                    else if(automaton_state == PLAN_BIN_TRAJECTORIES_3_STAGES_SECOND)
                    {
                        PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state)<<"BIN: "<<bin_names.at(current_bin_name),PRX_TEXT_CYAN);
                        //THREE_STAGE_TRAJECTORY_SECOND == 9
                        command.stage = 9;
                        command.bin = bin_names.at(current_bin_name);
                        command.hand = cur_hand;

                        bool planning_success = false;
                        planning_app_query->sendGoal(command);
                        planning_success = planning_app_query->waitForResult(ros::Duration(0));
                        result = *planning_app_query->getResult();
                        if(planning_success && result.plan.size()>0)
                        {
                            double bin_plan_duration = result.duration;
                            PRX_PRINT("Plan Duration: "<<bin_plan_duration<<"("<<thresh<<")"<<"   Bin_Hand: "<<cur_hand<<"_"<<bin_names.at(current_bin_name),PRX_TEXT_GREEN);
                        }
                        if(planning_success && result.plan.size()>0)
                        {
                            jk_prm_star ? thresh = 1000 : thresh = 6;
                            PRX_PRINT("Received Plan Successfully...",PRX_TEXT_CYAN);
                            execute_command.plan=result.plan;
                    

                            controller->convert_and_copy_to_robot_plan(execute_command.plan);
                            controller->set_robot_plan();
                            
                            //ZPP
                            if(real_robot)
                            {
                                create_and_send_robot_command();
                            }

                            plan_duration = result.duration;
                            executing_trajectory = true;
                            PRX_PRINT("Executing Plan...",PRX_TEXT_CYAN);
                            currently_in_home_position = false;
                        }
                        automaton_state = PLAN_BIN_TRAJECTORIES_3_STAGES_THIRD;

                    }
                    else if(automaton_state == PLAN_BIN_TRAJECTORIES_3_STAGES_THIRD)
                    {
                        PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state)<<"BIN: "<<bin_names.at(current_bin_name),PRX_TEXT_CYAN);
                        //THREE_STAGE_TRAJECTORY_THIRD == 10
                        command.stage = 10;
                        command.bin = bin_names.at(current_bin_name);
                        command.hand = cur_hand;

                        bool planning_success = false;
                        planning_app_query->sendGoal(command);
                        planning_success = planning_app_query->waitForResult(ros::Duration(0));
                        result = *planning_app_query->getResult();
                        if(planning_success && result.plan.size()>0)
                        {
                            double bin_plan_duration = result.duration;
                            PRX_PRINT("Plan Duration: "<<bin_plan_duration<<"("<<thresh<<")"<<"   Bin_Hand: "<<cur_hand<<"_"<<bin_names.at(current_bin_name),PRX_TEXT_GREEN);
                        }
                        if(planning_success && result.plan.size()>0)
                        {
                            jk_prm_star ? thresh = 1000 : thresh = 6;
                            PRX_PRINT("Received Plan Successfully...",PRX_TEXT_CYAN);
                            execute_command.plan=result.plan;
                    

                            controller->convert_and_copy_to_robot_plan(execute_command.plan);
                            controller->set_robot_plan();
                            
                            //ZPP
                            if(real_robot)
                            {
                                create_and_send_robot_command();
                            }

                            plan_duration = result.duration;
                            executing_trajectory = true;
                            PRX_PRINT("Executing Plan...",PRX_TEXT_CYAN);
                            currently_in_home_position = false;
                        }
                        automaton_state = START;
                        current_bin_name ++;
                        if(current_bin_name == bin_names.size() )
                        {   
                            automaton_state = END;
                        }

                    }
                    else{
                        PRX_FATAL_S("Unknown automaton state!");
                    }

                    update_simulation();
                }

                tf_broadcasting();

            }
            void apc_generate_bin_traj_t::update_simulation()
            {
                if(!real_robot)
                {
                    // PRX_PRINT("Updating Simulation",PRX_TEXT_CYAN);
                    simulator->push_control(simulation_control);
                    simulator->propagate_and_respond();
                    simulation_state_space->copy_to_point(simulation_state);
                    loop_total += loop_timer.measure_reset();
                    loop_counter++;
                    loop_avg = loop_total / loop_counter;
                    publish_state();
                }
            }


            void apc_generate_bin_traj_t::place_object_callback(const prx_simulation::object_msg& objectMsg)
            {
                config_t new_config;
                foreach(movable_body_plant_t* plant, movable_bodies)
                {
                    // PRX_INFO_S("Try Setting "<<objectMsg.name);
                    if(plant->get_object_type() == objectMsg.name)
                    {
                        PRX_INFO_S("Setting "<<plant->get_object_type());
                        moved = plant;
                        new_config.set_position(objectMsg.elements[0],objectMsg.elements[1],objectMsg.elements[2]);
                        new_config.set_orientation(objectMsg.elements[3],objectMsg.elements[4],objectMsg.elements[5],objectMsg.elements[6]);
                        moved->set_configuration(new_config);
                        moved = NULL;
                    }
                }
            }

            void apc_generate_bin_traj_t::reset_object_pose()
            {
                // ros::ServiceClient prx_sensing_publish_cl = n.serviceClient<prx_sensing::UpdateObjectList>("prx/sensing/update_object_list");

                // prx_sensing::UpdateObjectList srv;
                // // srv.request.publish_objects = true;
                // prx_sensing_publish_cl.call(srv);

                std::vector<double> dummy_state;
                dummy_state.push_back(3);
                dummy_state.push_back(3);
                dummy_state.push_back(3);
                dummy_state.push_back(0);
                dummy_state.push_back(0);
                dummy_state.push_back(0);
                dummy_state.push_back(1);

                foreach(movable_body_plant_t* obj, movable_bodies)
                {
                    obj->get_state_space()->set_from_vector(dummy_state);
                    //send object poses to planning
                    objectMsg.name = obj->get_object_type();
                    objectMsg.elements = dummy_state;
                    object_publisher.publish(objectMsg);
                    // obj->print_configuration();
                    dummy_state[0]+=2;
                    dummy_state[1]+=2;
                    dummy_state[2]+=2;  
                    // PRX_INFO_S("*SIMULATION*: Resetting "<<obj->get_object_type());
                }
            }

            void apc_generate_bin_traj_t::create_and_send_robot_command()
            {
                std::vector<trajectory_t*> robot_trajs;
                std::vector<plan_t*> robot_plans;
                std::vector<bool> grasp_plan;

                controller->get_robot_plans(robot_trajs, robot_plans, grasp_plan);
                for(unsigned index = 0;index<robot_trajs.size();index++)
                {
                    double duration = create_robot_message(robot_trajs, robot_plans, grasp_plan, index);
                    if(duration>0){
                        send_robot_message(duration);
                    }
                }
                PRX_INFO_S("Done");
                has_goal = false;
                controller->clear_robot_plan();
                controller->set_completed();
            }

            double apc_generate_bin_traj_t::create_robot_message(std::vector<trajectory_t*>& robot_trajs, std::vector<plan_t*>& robot_plans, std::vector<bool>& grasp_plan, int index)
            {
                state_t* local_state = child_state_space->alloc_point();
                double duration=0;
                if(!grasp_plan[index] && robot_plans[index]->length()>0)
                {

                    child_state_space->copy_to_point(local_state);
                    robot_trajs[index]->copy_onto_back(local_state);

                    foreach(plan_step_t step, *robot_plans[index])
                    {
                        int steps = (int)((step.duration / simulation::simulation_step) + .1);
                        output_control_space->copy_from_point(step.control);
                        for(int i=0;i<steps;i++)
                        {
                            controller->propagate(simulation::simulation_step);
                            child_state_space->copy_to_point(local_state);
                            robot_trajs[index]->copy_onto_back(local_state);
                        }
                    }

                    robot_command.trajectory.points.clear();
                    unigripper_command.trajectory.points.clear();
                    //create message and send to robot
                    
                    for(unsigned i=0;i<robot_trajs[index]->size()-1;i++)
                    {
                        trajectory_msgs::JointTrajectoryPoint point;
                        trajectory_msgs::JointTrajectoryPoint point_uni;
                        point.time_from_start = ros::Duration(duration);
                        point_uni.time_from_start = ros::Duration(duration);
                        control_t* control = robot_plans[index]->get_control_at(duration);
                        //add the state to the trajectory point
                        for( unsigned j = 0; j < 16; j++ )
                        {
                            point.positions.push_back((*robot_trajs[index])[i]->at(name_index_map[dof_names[j]]));
                            point.velocities.push_back(control->at(name_index_map[dof_names[j]]));
                            point.accelerations.push_back(0);
                        }
                        duration+=simulation::simulation_step;
                        robot_command.trajectory.points.push_back(point);

                        point_uni.positions.push_back((*robot_trajs[index])[i]->at(8));
                        point_uni.positions.push_back(control->at(8));
                        point_uni.accelerations.push_back(0);

                        unigripper_command.trajectory.points.push_back(point_uni);

                    }
                }
                else if(grasp_plan[index])
                {
                    PRX_INFO_S("Plan: \n"<<robot_plans[index]->print(2));
                    //trigger a grasp
                    control_t* control = robot_plans[index]->get_control_at(0);
                    if(control->at(16)>=1)
                    {
                        PRX_INFO_S("Sending UniGripper command");
                        ros::ServiceClient uni_client = n.serviceClient<prx_simulation::UnigripperVacuumOn>("unigripper_vacuum");
                        prx_simulation::UnigripperVacuumOn uni_srv;
                        if(control->at(16) == 1)//Vacuum OFF
                        {
                            uni_srv.request.TurnVacuumOn = false;
                        }
                        else if(control->at(16) == 2)//Vacuum ON
                        {
                            uni_srv.request.TurnVacuumOn = true;
                        }
                        else
                        {
                            PRX_ERROR_S("Wrong UniGripper Control!");
                        }

                        uni_client.waitForExistence();
                        uni_client.call(uni_srv);
                        sleep(2);

                    }
                    if(control->at(17)>=1)
                    {
                        PRX_INFO_S("Sending gripper command");
                        ros::ServiceClient client = n.serviceClient<prx_simulation::gripper_change_srv>("/prx/reflex_grasp");
                        prx_simulation::gripper_change_srv srv;
                        srv.request.gripper_state = control->at(17);

                        client.waitForExistence();
                        client.call(srv);
                    }

                }
                delete robot_trajs[index];
                delete robot_plans[index];
                child_state_space->free_point(local_state);
                return duration;
            }

            void apc_generate_bin_traj_t::send_robot_message(double duration)
            {
                PRX_INFO_S("Searching for the ac_server");
                bool found_ac_server = false;
                while( !found_ac_server )
                    found_ac_server = robot_ac->waitForServer(ros::Duration(0)) && unigripper_ac->waitForServer(ros::Duration(0));
                PRX_INFO_S("Found the ac_server!");

                // PRX_INFO_S("Plan: \n"<<robot_plans[index]->print(5));
                //PRX_INFO_S("Traj: \n"<<robot_trajs[index]->print(2));
                if(duration>0)
                {
                    robot_command.trajectory.header.stamp = ros::Time::now();
                    unigripper_command.trajectory.header.stamp = ros::Time::now();
                    PRX_INFO_S("Sending Robot command");
                    robot_ac->sendGoal(robot_command);
                    unigripper_ac->sendGoal(unigripper_command);
                }
                bool finished_before_timeout=false;
                double elapsed = 0.0;
                // PRX_WARN_S(ros::Duration(robot_plans[index]->length()+1.0));
                while(!finished_before_timeout && elapsed<duration+2){
                    elapsed+=.1;
                    finished_before_timeout = robot_ac->waitForResult(ros::Duration(.1));
                    ros::spinOnce();
                    tf_broadcasting();
                }
                if(!finished_before_timeout)
                {
                    // PRX_INFO_S("Didn't finish before timeout: "<<robot_plans[index]->length()+5.0);
                    robot_ac->cancelGoal();
                    unigripper_ac->cancelGoal();
                }
                else
                {
                    actionlib::SimpleClientGoalState state = robot_ac->getState();
                    ROS_INFO("Action finished: %s",state.toString().c_str());
                }
                ros::Rate rate(10);
                int counter =0;
                has_goal = false;
                while(ros::ok() && counter++<11)
                {
                    rate.sleep();
                }
                has_goal = true;
            }
            void apc_generate_bin_traj_t::set_selected_path(const std::string& path){}

            std::string apc_generate_bin_traj_t::getStringFromEnum(automaton_states a_s)
            {
              switch (a_s)
              {
              case START: return "START";
              case SHELF_DETECTION: return "SHELF_DETECTION";
              case TARGET_SELECTION: return "TARGET_SELECTION";
              case MOVE_AND_SENSE: return "MOVE_AND_SENSE";
              case POSE_ESTIMATION: return "POSE_ESTIMATION";
              case GRASP_PLANNING: return "GRASP_PLANNING";
              case BLOCKING_ITEM_SELECTION: return "BLOCKING_ITEM_SELECTION";
              case EXECUTE_REACHING: return "EXECUTE_REACHING";
              case EVALUATE_GRASP: return "EVALUATE_GRASP";
              case ADJUST_EE: return "ADJUST_EE";
              case EXECUTE_GRASP: return "EXECUTE_GRASP";
              case DISENGAGE_EE: return "DISENGAGE_EE";
              case PLAN_FOR_BLOCKING_ITEM: return "PLAN_FOR_BLOCKING_ITEM";
              case PLAN_FOR_TARGET_ITEM: return "PLAN_FOR_TARGET_ITEM";
              case EXECUTE_PLACING: return "EXECUTE_PLACING";
              case PLAN_FOR_RETRACTION: return "PLAN_FOR_RETRACTION";
              case STOP_ROBOT: return "STOP_ROBOT";
              case TURN_OFF_SENSING: return "TURN_OFF_SENSING";
              case MOVE_TO_HOME: return "MOVE_TO_HOME";
              case PLAN_BIN_TRAJECTORIES: return "PLAN_BIN_TRAJECTORIES";
              case PLAN_BIN_TRAJECTORIES_3_STAGES_FIRST: return "PLAN_BIN_TRAJECTORIES_3_STAGES_FIRST";
              case PLAN_BIN_TRAJECTORIES_3_STAGES_SECOND: return "PLAN_BIN_TRAJECTORIES_3_STAGES_SECOND";
              case PLAN_BIN_TRAJECTORIES_3_STAGES_THIRD: return "PLAN_BIN_TRAJECTORIES_3_STAGES_THIRD";
              case END: return "END";
              default: return "!!!Bad enum!!!";
              }
            }
        }
    }
}