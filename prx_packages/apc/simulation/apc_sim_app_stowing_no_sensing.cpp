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

#include "simulation/apc_sim_app_stowing_no_sensing.hpp"
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


PLUGINLIB_EXPORT_CLASS(prx::packages::apc::apc_sim_app_stowing_no_sensing_t, prx::sim::application_t)



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
            apc_sim_app_stowing_no_sensing_t::apc_sim_app_stowing_no_sensing_t()
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

            apc_sim_app_stowing_no_sensing_t::~apc_sim_app_stowing_no_sensing_t()
            {
                delete tf_broadcaster;
                delete plan_comm;
                delete sim_comm;
                delete vis_comm;
            }

            void apc_sim_app_stowing_no_sensing_t::init(const parameter_reader_t * const reader)
            {
                Init_Finished = false;
                PRX_INFO_S("Initializing APC Simulation Application");
                current_item_index = 0;
                number_of_orders = 0;
                current_bin_name = 0;
                Got_Item = false;
                rearrangement_flag = false;
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

                has_dynamic_obstacles = reader->get_attribute_as<bool>("application/has_dynamic_obstacles", true);
                visualize = reader->get_attribute_as<bool>("application/visualize", true);
                screenshot_every_simstep = reader->get_attribute_as<bool>("application/screenshot_every_simstep", false);
                simulator_running = reader->get_attribute_as<bool>("application/start_simulation",false);

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
                    
                    //NEW SHELF VISUALIZATION
                    ((visualization_comm_t*)vis_comm)->send_rigid_bodies();
                    //  if(ros::param::has("prx/obstacles"))
                    //     vis_comm->visualize_obstacles("prx");    
                    // else
                    //     vis_comm->visualize_obstacles(ros::this_node::getName().substr(1,ros::this_node::getName().length())+"/"+ simulator->get_pathname());
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

                // Object priorities

                const parameter_reader_t* child_template_reader = NULL;

                if( parameters::has_attribute("template", reader, NULL) )
                {
                    std::string template_name = parameters::get_attribute("template", reader, NULL);
                    child_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + template_name, global_storage);
                }

                if (parameters::has_attribute("object_to_prioritized_end_effector_context", reader, child_template_reader))
                {
                    PRX_DEBUG_COLOR ("\n\n\nCHECK\n\n\n", PRX_TEXT_GREEN);


                    parameter_reader_t::reader_map_t map = parameters::get_map("object_to_prioritized_end_effector_context",reader,child_template_reader);
                    foreach(const parameter_reader_t::reader_map_t::value_type key_value, map)
                    {                    
                        std::vector<std::string> prioritized_context = key_value.second->get_attribute_as< std::vector<std::string> >("");
                        object_to_prioritized_end_effector_context[key_value.first] = prioritized_context;

                        // DEBUG PRINT
                        PRX_DEBUG_COLOR("Object name: " << key_value.first, PRX_TEXT_MAGENTA);
                        for (auto name : object_to_prioritized_end_effector_context[key_value.first] )
                        {
                            PRX_DEBUG_COLOR("EE Context name: " << name, PRX_TEXT_CYAN);
                        } 
                    }
                }
                else
                {
                    PRX_FATAL_S ("Need to define the end effector contexts to use for each object!");
                }
                if(real_robot)
                {
                    real_robot_state = n.subscribe("/joint_states",4,&apc_sim_app_stowing_no_sensing_t::real_robot_state_callback,this);
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
                //Object pose
                std::vector<double> dummy = {0.8,0,1.3,0,0,0,1};
                object_pose = reader->get_attribute_as<std::vector<double> >("application/object_pose",dummy);
                object_new_pose = reader->get_attribute_as<std::vector<double> >("application/object_new_pose",dummy);
                change_hand = false;
                Init_Finished = true;
            }

            void apc_sim_app_stowing_no_sensing_t::real_robot_state_callback(const sensor_msgs::JointState& stateMsg)
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

            void apc_sim_app_stowing_no_sensing_t::publish_state()
            {
                prx_simulation::state_msg msg;
                for(unsigned i=0;i<child_state_space->get_dimension();++i)
                {
                    msg.elements.push_back(child_state_space->at(i));
                }
                // PRX_PRINT("child_state_space: "<< child_state_space->print_memory(3), PRX_TEXT_BROWN);
                state_publisher.publish(msg);
            }

            void apc_sim_app_stowing_no_sensing_t::frame(const ros::TimerEvent& event)
            {
                handle_key();
                // update_simulation();
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
                    // PRX_PRINT(simulator->get_state_space()->print_memory(),PRX_TEXT_BROWN);
                    // PRX_PRINT(simulator->get_state_space()->print_point(simulation_state),PRX_TEXT_CYAN);
                    // simulator->push_state(simulation_state);
                    // PRX_PRINT(simulator->get_state_space()->print_memory(),PRX_TEXT_BROWN);
                    // PRX_PRINT("-----",PRX_TEXT_RED);
                    prx_planning::apc_queryGoal command;
                    prx_planning::apc_queryResult result;
                    prx_simulation::send_trajectoryGoal execute_command;


                    if(executing_trajectory == true && !real_robot)
                    {
                        // PRX_PRINT("executing_trajectory", PRX_TEXT_GREEN);
                        // PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state),PRX_TEXT_CYAN);
                        // if(execution_time>plan_duration*0.4 && automaton_state == EVALUATE_GRASP && start_servoing())
                        // {
                        //     controller->cancel_goal();
                        //     //LOGIC
                        //     automaton_state = GRASP_PLANNING;
                        //     plan_duration = 0.0;
                        //     execution_time = 0.0;
                        //     executing_trajectory = false;
                        //     update_simulation();
                        //     PRX_PRINT("target_obj_pose changed, will replan",PRX_TEXT_BROWN);
                        // }
                        if(execution_time<plan_duration*.65)
                        {
                            PRX_STATUS("Executing Trajectory"<<execution_time <<"/"<<plan_duration,PRX_TEXT_CYAN);
                            execution_time+=simulation::simulation_step;
                            // update_object_poses(command.object);
                            update_simulation();
                        }
                        else
                        {
                            update_simulation();
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

                        // std::string work_order_json_file_name("/home/psarakisz89/PRACSYS/src/prx_packages/apc/simulation/apc_pick_task.json");

                        //ZPTODO : We need to parse the JSON file 
                        char* w = std::getenv("PRACSYS_PATH");
                        std::string dir(w);
                        dir += ("/prx_packages/apc/simulation/apc_pick_task.json");
 
                        std::string work_order_json_file_name(dir);
                        number_of_orders = read_file(work_order_json_file_name, work_order, bin_contents);

                        PRX_PRINT("number_of_orders: "<<number_of_orders, PRX_TEXT_GREEN);

                        if(!currently_in_home_position)
                        {
                            automaton_state = MOVE_TO_HOME;
                        }
                        else
                        {
                            automaton_state = SHELF_DETECTION;

                        }
                    } 
                    else if( automaton_state == SHELF_DETECTION)
                    {
                        PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state),PRX_TEXT_CYAN);
                        // estimate_shelf_position();
                        config_t new_config;
                        foreach(sim::obstacle_t* obst, obstacles)
                        {
                            // PRX_PRINT(obstacles.get_pathname(),PRX_TEXT_RED);
                            PRX_PRINT("SIM APP: Obstacle = "<<obst->get_pathname()<<", config: " <<obst->get_root_configuration().print(),PRX_TEXT_GREEN);
                            if(obst->get_pathname().find("shelf")!=std::string::npos) 
                            {
                                shelf = obst;
                                PRX_PRINT("Found the Shelf"<<shelf->get_root_configuration().print(),PRX_TEXT_RED);
                                double x,y,z,qx,qy,qz,qw;
                                x = 1.22;
                                y = 0;
                                z = 1.11;
                                qx = 0;
                                qy = 0;
                                qz = 0;
                                qw = 1;
                                new_config.set_position(x,y,z);
                                new_config.set_orientation(qx,qy,qz,qw);
                                shelf->update_root_configuration(new_config);
                                shelf->update_collision_info();

                                //send shelf pose to planning
                                objectMsg.name = "shelf";
                                std::vector<double> shelf_pose_vec;
                                new_config.get_position(shelf_pose_vec);
                                shelf_pose_vec.push_back(qx);
                                shelf_pose_vec.push_back(qy);
                                shelf_pose_vec.push_back(qz);
                                shelf_pose_vec.push_back(qw);
                                objectMsg.elements = shelf_pose_vec;
                                object_publisher.publish(objectMsg);
                                PRX_PRINT("Updated the shelf's position: "<<shelf->get_root_configuration().print(),PRX_TEXT_RED);
                                shelf = NULL;
                            }

                        }
                        //sensing srv prx/sensing/update_shelf_pose
                        automaton_state = BIN_SELECTION;
                        // automaton_state = PLAN_BIN_TRAJECTORIES;
                        // automaton_state = SHELF_DETECTION;
                    }
                    else if( automaton_state == BIN_SELECTION)
                    {
                        PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state),PRX_TEXT_CYAN);
                        //logic
                        //else if no more target, automaton_state = END
                        //ZPTODO: We need to devise a way to choose a bin depending on how many items are inside the bin
                        
                        CurrentBin = "B";
                        CurrentArm = "left";
                        CurrentTarget = "kleenex_tissue_box";
                        // current_item_index = i;                                
                        PRX_PRINT("CurrentBin: "<<CurrentBin, PRX_TEXT_GREEN);
                        // PRX_PRINT("CurrentTarget: "<<CurrentTarget, PRX_TEXT_GREEN);
                        PRX_PRINT("current_item_index: "<<current_item_index, PRX_TEXT_GREEN);
                        automaton_state = MOVE_AND_SENSE;
                        if (automaton_state != MOVE_AND_SENSE)
                        {
                            automaton_state = END;
                        }
                    }
                    else if( automaton_state == MOVE_AND_SENSE)
                    {
                        PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state),PRX_TEXT_CYAN);
                        //play saved traj   
                        //5 means EXECUTE_SAVED_TRAJ (refer to apc_query.action)
                        // std::stringstream path_name;
                        // path_name << CurrentArm << "_" << CurrentBin;
                        // PRX_PRINT(path_name.str(),PRX_TEXT_BROWN);
                        // command.stage = 5;
                        // command.hand = CurrentArm;
                        // command.saved_plan_path = path_name.str();
                        
                        // PRX_PRINT(command.saved_plan_path,PRX_TEXT_BROWN);
                        // planning_app_query->sendGoal(command);

                        PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state),PRX_TEXT_CYAN);
                        reset_object_pose();
                        //int32 MOVE_AND_DETECT_TOTE=11
                        command.stage = 11;
                        command.bin = "T";
                        command.hand = "right";
                        // PRX_PRINT(command.goal_state,PRX_TEXT_BROWN);
                        planning_app_query->sendGoal(command);

                        bool planning_success = false;
                        planning_success = planning_app_query->waitForResult(ros::Duration(0));
                        result = *planning_app_query->getResult();
                        if(planning_success && result.plan.size()>0)
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
                            currently_in_home_position = false;
                            clear_plan();
                            automaton_state = POSE_ESTIMATION;
                            // automaton_state = GRASP_PLANNING;
                            
                        }
                    }
                    else if( automaton_state == POSE_ESTIMATION)
                    {
                        PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state),PRX_TEXT_CYAN);
                        double counter = 0;
                        z_axis_offset = 0.0;
                        foreach(movable_body_plant_t* plant, movable_bodies)
                        {
                            if(plant->get_object_type() == CurrentTarget)
                            {
                                PRX_PRINT("CurrentTarget: "<<CurrentTarget,PRX_TEXT_CYAN);

                                target_index = counter;
                                PRX_PRINT("target_index: "<<target_index, PRX_TEXT_GREEN);

                                std::vector<double> obj_pose_vector;
                                config_t obj_pose;
                                //hardcode here some position

                                obj_pose.set_position(object_pose.at(0),object_pose.at(1),object_pose.at(2));
                                obj_pose.set_orientation(object_pose.at(3),object_pose.at(4),object_pose.at(5));

                                plant->set_configuration(obj_pose);
                                obj_pose.copy_to_vector(target_obj_pose);
                                PRX_PRINT("target_obj_pose: " << target_obj_pose, PRX_TEXT_GREEN);
                                plant->update_collision_info();

                                check_object_collision(plant);

                                objectMsg.name = CurrentTarget;
                                // std::vector<double> pose_vec;
                                // double quat[4];
                                // obj_pose.get_position(pose_vec);
                                // target_obj_pose.get_xyzw_orientation(quat);
                                // pose_vec.push_back(quat[0]);
                                // pose_vec.push_back(quat[1]);
                                // pose_vec.push_back(quat[2]);
                                // pose_vec.push_back(quat[3]);
                                objectMsg.elements = target_obj_pose;
                                object_publisher.publish(objectMsg);
                                plant = NULL;

                                //test rearrangement
                                // rearrangement_flag = true;
                                if (rearrangement_flag == true)
                                {
                                    automaton_state = PLAN_FOR_BLOCKING_ITEM; 
                                }
                                else{
                                    CurrentArm = ((object_to_prioritized_end_effector_context[CurrentTarget].front().find("left") != std::string::npos ) ? "left" : "right");
                                    if(CurrentArm == "left")
                                    {
                                        change_hand = true;
                                        automaton_state = MOVE_TO_HOME;
                                    }
                                    else
                                    {
                                        automaton_state = GRASP_PLANNING;                                         
                                    }
                                }                                                             
                            }
                            counter++;
                        }
                    }
                    else if( automaton_state == GRASP_PLANNING)
                    {
                        PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state),PRX_TEXT_CYAN);

                        PRX_PRINT("Beginning of GRASP PLANNING: simulation_state: "<<simulation_state_space->print_point(simulation_state, 4), PRX_TEXT_RED);

                        //12 means REMOVE_FROM_TOTE (bring item in front of head camera) (refer to apc_query.action)
                        command.stage = 12;
                        command.object = CurrentTarget;
                        command.hand = CurrentArm;
                        command.bin = CurrentBin;
                        PRX_PRINT("\n\n\n\n CurrentArm: "<<CurrentArm, PRX_TEXT_CYAN);
                        foreach(movable_body_plant_t* plant, movable_bodies)
                        {
                            if(plant->get_object_type() == command.object)
                            {
                                std::vector<double> obj_pose_vector;
                                config_t obj_pose;
                                plant->get_configuration(obj_pose);
                                obj_pose.copy_to_vector(obj_pose_vector);
                                command.object_state = obj_pose_vector;
                            }
                        }
                        
                        PRX_PRINT(command.object_state,PRX_TEXT_BROWN);
                        planning_app_query->sendGoal(command);
                        bool planning_success = false;
                        
                        double before_planning =ros::Time::now().toSec();

                        planning_app_query->waitForResult(ros::Duration(0));
                        result = *planning_app_query->getResult();

                        double after_planning =ros::Time::now().toSec();
                        PRX_PRINT("Grasp planning takes: "<< after_planning-before_planning << "secs", PRX_TEXT_CYAN);

                        actionlib::SimpleClientGoalState state = planning_app_query->getState();
                        PRX_PRINT("Planning Succeded: "<<state.toString(),PRX_TEXT_RED);
                        // PRX_PRINT("Executing Plan..."<<result.duration,PRX_TEXT_CYAN);

                        nr_execution_failures = 0;
                        nr_grasping_failures = 0;
                        if (state.toString()=="SUCCEEDED")
                        {
                            current_result = result;

                            //execute the whole plan until we have resensing capability
                            automaton_state = EXECUTE_PLACING;
                            // automaton_state = EXECUTE_REACHING;
                        }
                        else if (rearrangement_flag == true)
                        {
                            PRX_PRINT("Engaging Rearrangement Mode...",PRX_TEXT_CYAN);
                            automaton_state = BLOCKING_ITEM_SELECTION;
                        }
                        else if (state.toString()=="ABORTED")
                        {
                            PRX_PRINT("Grasp Planning Failed...",PRX_TEXT_CYAN);
                            automaton_state = STOP_ROBOT;
                        }
                        // planning srv; (may set rearrangement_flag = true else if palanning failed)
                    }
                    else if( automaton_state == BLOCKING_ITEM_SELECTION)
                    {
                        PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state),PRX_TEXT_CYAN);
                        automaton_state = GRASP_PLANNING;
                        //logic
                    }
                    else if( automaton_state == EXECUTE_REACHING)
                    {
                        PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state),PRX_TEXT_CYAN);
                        // subscribe topic: prx/sensing/publish_object_list, potetially RGB cam as well
                        bool pose_change = false;
                        bool replan = true;
                        
                        has_goal=true;
                        execute_command.plan=current_result.reaching;

                        // for (int i = 0; i < current_result.reaching.size(); ++i)
                        // {
                        //    PRX_PRINT("reaching plan: "<<execute_command.plan[i].control, PRX_TEXT_CYAN);
                        // }

                        controller->convert_and_copy_to_robot_plan(execute_command.plan);
                        controller->set_robot_plan();
                        //need to create variables to save the robot messages
                        if(real_robot)
                        {
                            create_and_send_robot_command();
                        }

                        plan_duration = current_result.reaching_duration;

                        executing_trajectory = true;
                        PRX_PRINT("Executing Plan..."<<plan_duration,PRX_TEXT_CYAN);
                        currently_in_home_position = false;
                        clear_plan();

                        if (pose_change)
                        {
                            if (replan && nr_execution_failures << 3)
                            {
                                nr_execution_failures++;
                                automaton_state = GRASP_PLANNING;
                            }
                            else{
                                automaton_state = STOP_ROBOT;
                            }
                        }
                        else{
                            automaton_state = EVALUATE_GRASP;
                        }
                    }
                    else if( automaton_state == EVALUATE_GRASP)
                    {
                        PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state),PRX_TEXT_CYAN);
                        bool good_grasp = true;
                        if (good_grasp)// && start_servoing())
                        {                            
                            // foreach(movable_body_plant_t* plant, movable_bodies)
                            // {
                            //     if(plant->get_object_type() == CurrentTarget)
                            //     {
                            //         std::vector<double> obj_pose_vector;
                            //         config_t obj_pose;
                            //         obj_pose.set_position(object_new_pose.at(0),object_new_pose.at(1),object_new_pose.at(2));
                            //         obj_pose.set_orientation(object_new_pose.at(3),object_new_pose.at(4),object_new_pose.at(5));
                            //         obj_pose.copy_to_vector(obj_pose_vector);
                            //         command.object_state = obj_pose_vector;

                            //         objectMsg.name = CurrentTarget;
                            //         std::vector<double> pose_vec;
                            //         double quat[4];
                            //         obj_pose.get_position(pose_vec);
                            //         obj_pose.get_xyzw_orientation(quat);
                            //         pose_vec.push_back(quat[0]);
                            //         pose_vec.push_back(quat[1]);
                            //         pose_vec.push_back(quat[2]);
                            //         pose_vec.push_back(quat[3]);
                            //         objectMsg.elements = pose_vec;
                            //         object_publisher.publish(objectMsg);

                            //         PRX_PRINT("Update object pose", PRX_TEXT_GREEN);
                            //         plant = NULL;

                            //         target_obj_pose = obj_pose_vector;

                            //         update_simulation();
                            //     }
                            // }
                            // automaton_state = ADJUST_EE;
                            automaton_state = EXECUTE_GRASP;
                        }
                        else if(good_grasp)
                        {
                            automaton_state = EXECUTE_GRASP;
                        }
                        else{
                            automaton_state = STOP_ROBOT;
                        }
                        // Shaojun TODO, sensing from RGB
                    }
                    else if( automaton_state == ADJUST_EE)
                    {
                        PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state),PRX_TEXT_CYAN);
                        bool adjustable = false;

                        //6 means ADJUST_EE (refer to apc_query.action)
                        
                        command.stage = 6;
                        command.object = CurrentTarget;
                        command.hand = CurrentArm;
                        command.bin = CurrentBin;

                        foreach(movable_body_plant_t* plant, movable_bodies)
                        {
                            if(plant->get_object_type() == command.object)
                            {
                                std::vector<double> obj_pose_vector;
                                config_t obj_pose;
                                plant->get_configuration(obj_pose);
                                obj_pose.copy_to_vector(obj_pose_vector);
                                command.object_state = obj_pose_vector;
                            }
                        }
                        
                        PRX_PRINT(command.object_state,PRX_TEXT_BROWN);
                        planning_app_query->sendGoal(command);
                        bool planning_success = false;
                        planning_app_query->waitForResult(ros::Duration(0));
                        result = *planning_app_query->getResult();
                        actionlib::SimpleClientGoalState state = planning_app_query->getState();
                        PRX_PRINT("Planning Succeded: "<<state.toString(),PRX_TEXT_RED);
                        // PRX_PRINT("Executing Plan..."<<result.duration,PRX_TEXT_CYAN);

                        nr_execution_failures = 0;
                        nr_grasping_failures = 0;
                        if (state.toString()=="SUCCEEDED")
                        {
                            PRX_PRINT("Have a new plan for the new obj pose", PRX_TEXT_GREEN);
                            current_result = result;
                            adjustable = true;
                        }

                        ///////////////////////////////////////////
                        // sim::plan_t manipulator_plan;
                        // workspace_trajectory_t ee_trajectory;
                        // const util::space_t* result_state;

                        // if(manipulator->jac_steering(manipulator_plan, ee_trajectory, result_state, child_state_space, goal_config, active_manipulation_info->chain.first, active_manipulation_info->chain.second))
                        // {
                        //     convert_plan(result_plan, selected_control_space, active_manipulation_info->manipulator_plan, active_manipulation_info->full_manipulator_control_space);
                        //     active_manipulation_info->full_manipulator_state_space->copy_from_point(active_manipulation_info->full_target_manipulator_state);
                        //     selected_state_space->copy_to_point(result_state);
                        //     return true;
                        // }
                        //////////////////////////////////////////////////
                        if (adjustable)
                        {
                            automaton_state = EXECUTE_REACHING;
                        }
                        else{
                            automaton_state = STOP_ROBOT;
                        }
                        // Shaojun TODO, Jacobian steering

                    }
                    else if( automaton_state == EXECUTE_GRASP)
                    {
                        PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state),PRX_TEXT_CYAN);
                        bool grasp_success = true;
                        bool retry_grasp = true;

                        execute_command.plan=current_result.grasping;
                        for (int i = 0; i < current_result.grasping.size(); ++i)
                        {
                           PRX_PRINT("grasping plan: "<<execute_command.plan[i].control, PRX_TEXT_CYAN);
                        }
 
                        controller->convert_and_copy_to_robot_plan(execute_command.plan);
                        controller->set_robot_plan();
                        
                        //ZPP
                        if(real_robot)
                        {
                            create_and_send_robot_command();
                        }

                        plan_duration = current_result.grasping_duration;

                        executing_trajectory = true;
                        PRX_PRINT("Executing Plan..."<<plan_duration,PRX_TEXT_CYAN);
                        currently_in_home_position = false;
                        clear_plan();


                        if (grasp_success)
                        {
                            if (rearrangement_flag == true)
                            {
                                automaton_state = PLAN_FOR_BLOCKING_ITEM;
                                // automaton_state = EXECUTE_PLACING;
                            }
                            else{
                                // automaton_state = PLAN_FOR_TARGET_ITEM;
                                // automaton_state = PLAN_MOVE_TARGET_OUTSIDE_BIN;
                                automaton_state = EXECUTE_PLACING;
                            }
                        }
                        else{
                            if (retry_grasp & nr_grasping_failures < 3)
                            {
                                nr_grasping_failures++;
                                automaton_state = DISENGAGE_EE;
                            }

                        }
                        // sensing srv, tactile etc.
                    }
                    else if( automaton_state == DISENGAGE_EE)
                    {
                        PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state),PRX_TEXT_CYAN);
                        // EE control
                        std::vector<double> control(18, 0.0);
                        float duration = 0.1;

                        PRX_PRINT("control: "<< control, PRX_TEXT_GREEN);
                        PRX_PRINT("duration: "<< duration, PRX_TEXT_GREEN);
                        current_result.grasping[0].control = control;
                        current_result.grasping[0].duration = duration;
                        PRX_PRINT("current_result.plan.size: "<< current_result.grasping.size(), PRX_TEXT_GREEN);
                        for (int i = 1; i < current_result.grasping.size(); ++i)
                        {
                            current_result.grasping[i].control = control;
                            current_result.grasping[i].duration = 0;
                        }

                        execute_command.plan = current_result.grasping;//current_result.plan[0];
                        
                        // PRX_PRINT("control: "<< execute_command.plan[0].control, PRX_TEXT_GREEN);
                        // PRX_PRINT("duration: "<< execute_command.plan[0].duration, PRX_TEXT_GREEN);
                        controller->convert_and_copy_to_robot_plan(execute_command.plan);
                        controller->set_robot_plan();
                        
                        //ZPP
                        if(real_robot)
                        {
                            create_and_send_robot_command();
                        }
                        clear_plan();
                        automaton_state = MOVE_TO_HOME;
                    }
                    else if( automaton_state == PLAN_FOR_BLOCKING_ITEM)
                    {
                        PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state),PRX_TEXT_CYAN);
                        // logic: decide which bin to put; update Json file; planning srv
                        command.stage = 7;
                        command.hand = CurrentArm;
                        command.object = CurrentTarget;

                        // PRX_PRINT("CurrentTarget: "<<CurrentTarget, PRX_TEXT_GREEN);
                        command.object = CurrentTarget;
                        foreach(movable_body_plant_t* plant, movable_bodies)
                        {
                            if(plant->get_object_type() == command.object)
                            {
                                std::vector<double> obj_pose_vector;
                                config_t obj_pose;
                                plant->get_configuration(obj_pose);
                                obj_pose.copy_to_vector(obj_pose_vector);
                                command.object_state = obj_pose_vector;
                            }
                        }
                        
                        std::vector<double> final_obj_state = target_obj_pose;
                        final_obj_state[0] = 0.97;
                        final_obj_state[1] = 0;
                        final_obj_state[2] = 1.4;
                        command.final_obj_state = final_obj_state;
                        planning_app_query->sendGoal(command);
                        bool planning_success = false;
                        planning_success = planning_app_query->waitForResult(ros::Duration(0));
                        result = *planning_app_query->getResult();
                        actionlib::SimpleClientGoalState state = planning_app_query->getState();
                        PRX_PRINT("Planning Succeded: "<<state.toString(),PRX_TEXT_RED);
                        PRX_PRINT("Executing Plan..."<<result.duration,PRX_TEXT_CYAN);

                        if (state.toString()=="SUCCEEDED")
                        {
                            current_result = result;
                            automaton_state = EXECUTE_REACHING;
                        }
                    }
                    else if( automaton_state == PLAN_MOVE_TARGET_OUTSIDE_BIN)
                    {
                        PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state),PRX_TEXT_CYAN);
                        // planning srv
                        //3 means MOVE_OUTSIDE_BIN (refer to apc_query.action)
                        command.stage = 8;
                        command.hand = CurrentArm;
                        command.object = CurrentTarget;
                        PRX_PRINT("command.object: "<<command.object, PRX_TEXT_GREEN);

                        foreach(movable_body_plant_t* plant, movable_bodies)
                        {
                            if(plant->get_object_type() == command.object)
                            {
                                std::vector<double> obj_pose_vector;
                                config_t obj_pose;
                                plant->get_configuration(obj_pose);
                                obj_pose.copy_to_vector(obj_pose_vector);
                                command.object_state = obj_pose_vector;
                            }
                        }
                        
                        std::vector<double> final_obj_state = target_obj_pose;
                        final_obj_state[0] -= 0.1;
                        final_obj_state[2] += 0.03;
                        PRX_PRINT("final_obj_state: "<<final_obj_state, PRX_TEXT_GREEN);
                        command.final_obj_state = final_obj_state;

                        planning_app_query->sendGoal(command);
                        bool planning_success = false;
                        planning_app_query->waitForResult(ros::Duration(0));
                        result = *planning_app_query->getResult();
                        actionlib::SimpleClientGoalState state = planning_app_query->getState();
                        PRX_PRINT("Planning Succeded: "<<state.toString(),PRX_TEXT_RED);
                        PRX_PRINT("Executing Plan..."<<result.duration,PRX_TEXT_CYAN);

                        if (state.toString()=="SUCCEEDED")
                        {
                            current_result = result;
                            automaton_state = EXECUTE_PLACING;
                        }
                        else
                        {
                            PRX_PRINT("Cannot find a plan to the tote, disengage EE ...",PRX_TEXT_CYAN);
                            automaton_state = DISENGAGE_EE;
                        }
                    }

                    else if( automaton_state == PLAN_FOR_TARGET_ITEM)
                    {
                        PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state),PRX_TEXT_CYAN);
                        // planning srv

                        //3 means MOVE_TO_ORDER_BIN (refer to apc_query.action)
                        command.stage = 3;
                        command.hand = CurrentArm;
                        command.object = CurrentTarget;
                        PRX_PRINT("command.object: "<<command.object, PRX_TEXT_GREEN);

                        std::vector<double> final_obj_state = target_obj_pose;
                        // final_obj_state[0] = 0.97;
                        final_obj_state[0] -= 0.1;
                        final_obj_state[2] += 0.03;
                        PRX_PRINT("final_obj_state: "<<final_obj_state, PRX_TEXT_GREEN);
                        command.final_obj_state = final_obj_state;

                        planning_app_query->sendGoal(command);
                        bool planning_success = false;
                        planning_app_query->waitForResult(ros::Duration(0));
                        result = *planning_app_query->getResult();
                        actionlib::SimpleClientGoalState state = planning_app_query->getState();
                        PRX_PRINT("Planning Succeded: "<<state.toString(),PRX_TEXT_RED);
                        PRX_PRINT("Executing Plan..."<<result.duration,PRX_TEXT_CYAN);

                        if (state.toString()=="SUCCEEDED")
                        {
                            current_result = result;
                            automaton_state = EXECUTE_PLACING;
                        }
                        else
                        {
                            PRX_PRINT("Cannot find a plan to the tote, disengage EE ...",PRX_TEXT_CYAN);
                            automaton_state = DISENGAGE_EE;
                        }
                    }
                    else if( automaton_state == EXECUTE_PLACING)
                    {
                        PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state),PRX_TEXT_CYAN);
                        // send cammand to robot

                        // execute_command.plan=current_result.retracting;
                        execute_command.plan=current_result.plan;                        

                        controller->convert_and_copy_to_robot_plan(execute_command.plan);
                        controller->set_robot_plan();
                        
                        //ZPP
                        if(real_robot)
                        {
                            create_and_send_robot_command();
                        }

                        // plan_duration = current_result.retracting_duration;
                        plan_duration = current_result.duration;

                        executing_trajectory = true;
                        PRX_PRINT("Executing Plan..."<<plan_duration,PRX_TEXT_CYAN);
                        currently_in_home_position = false;
                        work_order.done[current_item_index] = true;

                        clear_plan();
                        automaton_state = MOVE_INSIDE_BIN;
                    }
                    else if(automaton_state == MOVE_INSIDE_BIN)
                    {
                        PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state),PRX_TEXT_CYAN);
                        // send cammand to robot

                        // execute_command.plan=current_result.retracting;
                        execute_command.plan=current_result.plan;                        

                        controller->convert_and_copy_to_robot_plan(execute_command.plan);
                        controller->set_robot_plan();
                        
                        //ZPP
                        if(real_robot)
                        {
                            create_and_send_robot_command();
                        }

                        // plan_duration = current_result.retracting_duration;
                        plan_duration = current_result.duration;

                        executing_trajectory = true;
                        PRX_PRINT("Executing Plan..."<<plan_duration,PRX_TEXT_CYAN);
                        currently_in_home_position = false;
                        work_order.done[current_item_index] = true;

                        clear_plan();
                        automaton_state = TURN_OFF_SENSING;
                    }
                    else if( automaton_state == PLAN_FOR_RETRACTION)
                    {
                        PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state),PRX_TEXT_CYAN);
                        // disengage ee and planning srv w/o object
                        automaton_state = STOP_ROBOT;
                    }
                    else if( automaton_state == STOP_ROBOT)
                    {
                        PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state),PRX_TEXT_CYAN);                    
                        // decrease object/bin priority
                        //MOVE EVERYTHING FROM THE SHELF BECAUSE OF POSSIBLE COLLISION
                        reset_object_pose();
                        automaton_state = TURN_OFF_SENSING;
                    }
                    else if( automaton_state == TURN_OFF_SENSING)
                    {
                        PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state),PRX_TEXT_CYAN);
                        reset_object_pose();
                        automaton_state = MOVE_TO_HOME;
                    }
                    else if( automaton_state == MOVE_TO_HOME)
                    {
                        PRX_PRINT("Current State: "<<getStringFromEnum(automaton_state),PRX_TEXT_CYAN);
                        if(!change_hand)
                        {
                            reset_object_pose();
                        }
                        std::vector<double> home_position={0.00000,1.57000,0.00000,0.00000,-1.70000,0.00000,0.00000,0.00000,0.00000,1.57000,0.00000,0.00000,-1.70000,0.00000,0.00000,0.00000,1.00000,1.00000, 1.0, 1.0};
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
                        clear_plan();
                        if(change_hand)
                        {
                            change_hand = false;
                            automaton_state = GRASP_PLANNING;
                        }
                        else
                        {
                            automaton_state = START;
                        }
                    }
                    else if( automaton_state == END)
                    {
                        PRX_PRINT("Finished every item", PRX_TEXT_GREEN);
                    }                    
                    else{
                        PRX_FATAL_S("Unknown automaton state!");
                    }

                    update_simulation();
                }


                /**
                 * WARNING: This needs to be called at some point for visualization to work. If you override this frame function, you must call this.
                */
                if(visualization_counter++%visualization_multiple == 0)
                {
                    tf_broadcasting();
                }


                //    if ( ground_truth_timer.measure() > ground_truth_time_limit )
                //    {
                //        sim_comm->publish_system_states();
                //        ground_truth_timer.reset();
                //    }

            }
            void apc_sim_app_stowing_no_sensing_t::update_simulation()
            {
                if(!real_robot)
                {
                    // PRX_PRINT("Updating Simulation",PRX_TEXT_CYAN);
                    // PRX_PRINT("simulation_control: "<< simulator->get_control_space()->print_point(simulation_control), PRX_TEXT_RED);
                    simulator->push_control(simulation_control);
                    simulator->propagate_and_respond();
                    simulation_state_space->copy_to_point(simulation_state);
                    loop_total += loop_timer.measure_reset();
                    loop_counter++;
                    loop_avg = loop_total / loop_counter;
                    publish_state();
                    // PRX_PRINT("finished updating Simulation",PRX_TEXT_CYAN);
                }
            }

            void apc_sim_app_stowing_no_sensing_t::place_object_callback(const prx_simulation::object_msg& objectMsg)
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

            void apc_sim_app_stowing_no_sensing_t::check_object_collision(movable_body_plant_t* plant)
            {
                if (z_axis_offset >=0.02)
                {
                    return;
                }
                foreach(collision_pair_t cp, simulator->get_colliding_bodies()->get_body_pairs())
                {
                    bool collision_with_bin_bottom = ( cp.first.find(CurrentTarget)!=std::string::npos) && (cp.second.find("top_shelf")!=std::string::npos \
                        || cp.second.find("middle_shelf")!=std::string::npos || cp.second.find("bottom_shelf")!=std::string::npos || cp.second.find("bottom")!=std::string::npos ) \
                        || ( cp.second.find(CurrentTarget)!=std::string::npos) && (cp.first.find("top_shelf")!=std::string::npos \
                        || cp.first.find("middle_shelf")!=std::string::npos || cp.first.find("bottom_shelf")!=std::string::npos || cp.first.find("bottom")!=std::string::npos );

                    if (CurrentBin == "A" || CurrentBin == "D" || CurrentBin == "G" || CurrentBin == "J")
                    {
                        bool collision_with_right_side = ( cp.first.find(CurrentTarget)!=std::string::npos) && (cp.second.find("left_divider")!=std::string::npos );
                    }
                    if (CurrentBin == "B" || CurrentBin == "E" || CurrentBin == "H" || CurrentBin == "K")
                    {
                        bool collision_with_right_side = ( cp.first.find(CurrentTarget)!=std::string::npos) && (cp.second.find("right_divider")!=std::string::npos );
                    }
                    if (CurrentBin == "C" || CurrentBin == "F" || CurrentBin == "I" || CurrentBin == "L")
                    {
                        bool collision_with_right_side = ( cp.first.find(CurrentTarget)!=std::string::npos) && (cp.second.find("right_side")!=std::string::npos );
                    }
                    
                    
                    // bool collision_with_left_side = 



                    if( collision_with_bin_bottom )
                    {
                        PRX_PRINT("Collision Pair: " << cp.first <<", " << cp.second, PRX_TEXT_BROWN);
                        config_t obj_pose;
                        z_axis_offset += 0.005; 
                        target_obj_pose[2] += z_axis_offset;
                        obj_pose.set_position(target_obj_pose[0],target_obj_pose[1],target_obj_pose[2]);
                        obj_pose.set_orientation(target_obj_pose[3],target_obj_pose[4],target_obj_pose[5],target_obj_pose[6]);
                        // target_obj_pose;
                        plant->set_configuration(obj_pose);
                        plant->update_collision_info();
                        update_simulation();
                        check_object_collision(plant);

                            // do
                            // {
                            //     final_target_object_scale -= scale_increment;
                            //     foreach(std::string name, map | boost::adaptors::map_keys)
                            //     {
                            //         PRX_PRINT ("Scaling down geom: " << name << " with : " << final_target_object_scale, PRX_TEXT_MAGENTA);
                            //         pqp_info_t* pqp_info = dynamic_cast<pqp_info_t*>(pqp_checker->get_collision_info(name));
                            //         pqp_info->model->ScaleModel(final_target_object_scale/temp_scale);
                            //     }
                            //     temp_scale = final_target_object_scale;
                            // }
                            // while (!manipulation_model->valid_state(actual_start_state, true) && final_target_object_scale >= 0.55);

                    }
                }
            }

            void apc_sim_app_stowing_no_sensing_t::reset_object_pose()
            {
                // ros::ServiceClient prx_sensing_publish_cl = n.serviceClient<prx_sensing::UpdateObjectList>("prx/sensing/update_object_list");

                // prx_sensing::UpdateObjectList srv;
                // srv.request.publish_objects = true;
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

            void apc_sim_app_stowing_no_sensing_t::set_selected_path(const std::string& path){}

            void apc_sim_app_stowing_no_sensing_t::create_and_send_robot_command()
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
                clear_plan();
            }

            void apc_sim_app_stowing_no_sensing_t::clear_plan()
            {
                has_goal = false;
                controller->clear_robot_plan();
                controller->set_completed();
            }

            double apc_sim_app_stowing_no_sensing_t::create_robot_message(std::vector<trajectory_t*>& robot_trajs, std::vector<plan_t*>& robot_plans, std::vector<bool>& grasp_plan, int index)
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
                    if(control->at(18)>=1)
                    {
                        PRX_INFO_S("Sending UniGripper command");
                        ros::ServiceClient uni_client = n.serviceClient<prx_simulation::UnigripperVacuumOn>("unigripper_vacuum");
                        prx_simulation::UnigripperVacuumOn uni_srv;
                        if(control->at(18) == 1)//Vacuum OFF
                        {
                            uni_srv.request.TurnVacuumOn = false;
                        }
                        else if(control->at(18) == 2)//Vacuum ON
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
                    if(control->at(18)>=1)
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

            double apc_sim_app_stowing_no_sensing_t::l2_norm(std::vector<double> const& u) 
            {
                double accum = 0.;
                for (double x : u) {
                    accum += x * x;
                }
                return sqrt(accum);
            }

            bool apc_sim_app_stowing_no_sensing_t::start_servoing()
            {
                //LOGIC
                PRX_PRINT("Monitoring target obj pose", PRX_TEXT_GREEN);
                PRX_PRINT("target_obj_pose: " << target_obj_pose, PRX_TEXT_GREEN);
                foreach(movable_body_plant_t* plant, movable_bodies)
                {
                    
                    if(plant->get_object_type() == CurrentTarget)
                    {
                        // PRX_PRINT("CurrentTarget: "<<CurrentTarget,PRX_TEXT_CYAN);
                        std::vector<double> obj_pose_vector, pose_diff_vector;
                        config_t obj_pose;
                        //hardcode here some position
                        obj_pose.set_position(object_new_pose.at(0),object_new_pose.at(1),object_new_pose.at(2));
                        obj_pose.set_orientation(object_new_pose.at(3),object_new_pose.at(4),object_new_pose.at(5));
                        // obj_pose.set_position(1.0,0.25,1.4);
                        // obj_pose.set_orientation(0,0,0,1);
                        plant->set_configuration(obj_pose);
                        obj_pose.copy_to_vector(obj_pose_vector);     

                        std::transform(obj_pose_vector.begin(), obj_pose_vector.end(), target_obj_pose.begin(),
                                             std::back_inserter(pose_diff_vector), [&](double l, double r)
                        {
                            return std::abs(l - r);
                        });
                        // for ( std::vector<double>::size_type i = 0; i < obj_pose_vector.size(); i++ )
                        // {
                        //    pose_diff_vector[i] = std::abs( obj_pose_vector[i] - target_obj_pose[i] );
                        // }

                        // PRX_PRINT("obj_pose: " << obj_pose, PRX_TEXT_GREEN);
                        // PRX_PRINT("target_obj_pose: " << target_obj_pose, PRX_TEXT_GREEN);
                        // PRX_PRINT("pose_diff: " << pose_diff_vector, PRX_TEXT_GREEN);
                        
                        double pose_diff_norm = 0;
                        pose_diff_norm = l2_norm(pose_diff_vector);
                        PRX_PRINT("pose_diff_norm: "<< pose_diff_norm, PRX_TEXT_GREEN);
                        if (pose_diff_norm > POSE_CAHNGE_THRESHOLD)
                        {
                            plant->update_collision_info();
                            objectMsg.name = CurrentTarget;
                            std::vector<double> pose_vec;
                            double quat[4];
                            obj_pose.get_position(pose_vec);
                            obj_pose.get_xyzw_orientation(quat);
                            pose_vec.push_back(quat[0]);
                            pose_vec.push_back(quat[1]);
                            pose_vec.push_back(quat[2]);
                            pose_vec.push_back(quat[3]);
                            objectMsg.elements = pose_vec;
                            object_publisher.publish(objectMsg);
                            plant = NULL;

                            target_obj_pose = obj_pose_vector;

                            publish_state();
                            return true;
                        }                             
                    }
                }
                return false;
            }

            void apc_sim_app_stowing_no_sensing_t::send_robot_message(double duration)
            {
                PRX_INFO_S("Searching for the ac_server");
                bool found_ac_server = false;
                bool pose_change = false;

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
                    finished_before_timeout = robot_ac->waitForResult(ros::Duration(.1)); //ros::Duration(robot_plans[index]->length()+
                    
                    if(start_servoing())
                    {
                        controller->cancel_goal();
                        //LOGIC
                    }
                    // bool sensing_success;
                    // sensing_success = estimate_objects_pose();
                    // if (sensing_success == true)
                    // {
                    //     foreach(movable_body_plant_t* plant, movable_bodies)
                    //     {
                    //         if(plant->get_object_type() == CurrentTarget)
                    //         {
                    //             std::vector<double> pose_diff_vector;
                    //             config_t obj_pose, pose_diff;
                    //             plant->get_configuration(obj_pose);
                    //             pose_diff = obj_pose - target_obj_pose;
                    //             pose_diff.copy_to_vector(pose_diff_vector);
                    //             double pose_diff_norm;
                    //             pose_diff_norm = l2_norm(pose_diff_vector);
                    //             if (pose_diff_norm > POSE_CAHNGE_THRESHOLD)
                    //             {
                    //                 pose_change = true;
                    //             }
                    //         }
                    //     }
                    // }


                    // SURVOING LOGIC
                    // if(survoing)
                    // {
                    //     robot_ac->cancelGoal();
                    //     unigripper_ac->cancelGoal();
                    //     PRX_INFO_S("Done");
                    //     has_goal = false;
                    //     controller->clear_robot_plan();
                    //     controller->set_completed();
                    //     call_planning
                    //     controller->convert_and_copy_to_robot_plan(execute_command.plan);
                    //     controller->set_robot_plan();
                        
                    //     //ZPP
                    //     if(real_robot)
                    //     {
                    //         create_and_send_robot_command();
                    //     }
                    // }

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


            std::string apc_sim_app_stowing_no_sensing_t::getStringFromEnum(automaton_states a_s)
            {
              switch (a_s)
              {
              case START: return "START";
              case SHELF_DETECTION: return "SHELF_DETECTION";
              case BIN_SELECTION: return "BIN_SELECTION";
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
              case END: return "END";
              case PLAN_MOVE_TARGET_OUTSIDE_BIN: return "PLAN_MOVE_TARGET_OUTSIDE_BIN";
              case MOVE_INSIDE_BIN: return "MOVE_INSIDE_BIN";
              default: return "!!!Bad enum!!!";
              }
            }
        }
    }
}