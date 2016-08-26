/**
 * @file apc_planning_application.cpp 
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

#include "prx/planning/task_planners/task_planner.hpp"
#include "prx/planning/communication/planning_comm.hpp"
#include "planning/apc_planning_application.hpp"

#include "prx_simulation/object_msg.h"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include <pluginlib/class_list_macros.h>

#include <boost/range/adaptor/map.hpp> //adaptors

#include <sstream>

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        using namespace manipulation;
        namespace apc
        {

            PLUGINLIB_EXPORT_CLASS( prx::packages::apc::apc_planning_application_t, prx::plan::planning_application_t)

            apc_planning_application_t::apc_planning_application_t()
            {
                shelf_update_counter = 0;
                total_resolves = 0;
                validity_checker = NULL;
            }

            apc_planning_application_t::~apc_planning_application_t() { }

            void apc_planning_application_t::init(const parameter_reader_t* reader)
            {
                PRX_PRINT("Starting APC planning application init()", PRX_TEXT_GREEN);
                planning_application_t::init(reader);

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
                planning_budget_time = parameters::get_attribute_as<double>("planning_budget_time", reader, child_template_reader);

                full_manipulator_context_name = parameters::get_attribute("full_manipulator_context_name", reader,NULL);

                if( parameters::has_attribute("validity_checker", reader, NULL) )
                {
                    validity_checker = parameters::initialize_from_loader<validity_checker_t > ("prx_planning", reader, "validity_checker", NULL, "validity_checker");
                }
                else
                {
                    PRX_FATAL_S("Missing validity_checker attribute in planning specification!");
                }
                record_plans = reader->get_attribute_as<bool>("record_plans", false);
                manipulation_model = dynamic_cast<manipulation_world_model_t*>(model);
                manipulation_model->use_context(full_manipulator_context_name);

                validity_checker->link_model(manipulation_model);

                output_query = new apc_task_query_t(manipulation_model->get_control_space());
                current_manipulator_state = manipulation_model->get_state_space()->alloc_point();
                propagated_manipulator_state = manipulation_model->get_state_space()->clone_point(current_manipulator_state);
                plan_counter = 0;
                object_subscriber = n.subscribe("prx/object_state", 100, &apc_planning_application_t::update_object_callback,this);
                cur_hand="init";
                prev_hand="none";
                stage_counter = 0;
                cur_bin="init";
                PRX_PRINT("COMPLETED: APC planning application init()", PRX_TEXT_BLUE);

                if (child_template_reader != NULL)
                    delete child_template_reader;
            }

            void apc_planning_application_t::execute()
            {
                manipulation_model->use_context(full_manipulator_context_name);
                prx_simulation::send_trajectoryGoal command;

                this->root_task->link_specification(root_specifications[0]);
                this->root_task->setup();
                this->root_task->execute();
                output_query->clear();

                state_subscriber = n.subscribe("prx/manipulator_state", 1, &apc_planning_application_t::get_state_callback,this);
                execute_client = new send_traj_client("prx/execute_plan_action",false);
                q_server = new query_server(n,"prx/apc_action", boost::bind(&apc_planning_application_t::resolve_query, this, _1), false);
                q_server->start();

                //get all the obtacles
                obstacles_hash = manipulation_model->get_obstacles();
                foreach(system_ptr_t obstacle, obstacles_hash | boost::adaptors::map_values)
                {
                    if(dynamic_cast<sim::obstacle_t*>(obstacle.get()) != NULL)
                    {
                        obstacles.push_back(dynamic_cast<sim::obstacle_t*>(obstacle.get()));
                    }
                }
                // get all the objects
                manipulation_model->get_objects(objects);
                initialize_objects_pose();
            }

            void apc_planning_application_t::resolve_query(const prx_planning::apc_queryGoalConstPtr& req)
            {
                // PRX_INFO_S("\n\n\n\n\n TOTAL COMPLETED: "<<total_resolves++<<"\n\n\n\n\n\n\n\n");
                // PRX_PRINT("\nStart: "<<manipulation_model->get_full_state_space()->print_memory(3),PRX_TEXT_RED);
                prx_simulation::send_trajectoryGoal command;

                bool found_ac_server = false;
                while( !found_ac_server )
                {
                    found_ac_server = execute_client->waitForServer(ros::Duration(1));
                }
                manipulation_model->use_context(full_manipulator_context_name);
                manipulation_model->get_state_space()->copy_from_point(current_manipulator_state);

                // PRX_PRINT("current_manipulator_state: "<<manipulation_model->get_state_space()->print_point(current_manipulator_state), PRX_TEXT_CYAN);
                // bool collided = manipulation_model->valid_state(current_manipulator_state, true);
                // PRX_PRINT("current_manipulator_state, collided: "<<collided, PRX_TEXT_GREEN);

                manipulation_model->get_state_space()->copy_to_point(propagated_manipulator_state);
                manipulation_model->get_state_space()->copy_from_point(propagated_manipulator_state);
                //PRX_PRINT("\nStart: "<<manipulation_model->get_full_state_space()->print_memory(3),PRX_TEXT_RED);
                PRX_PRINT ("BEGIN RESOLVE QUERY", PRX_TEXT_BLUE);

                if(req->stage == apc_task_query_t::MOVE)
                {
                    state_t* goal_state = manipulation_model->get_state_space()->alloc_point();
                    manipulation_model->get_state_space()->set_from_vector(req->goal_state,goal_state);
                    //move only, bin, hand, and object don't matter
                    output_query->setup("",NULL,apc_task_query_t::MOVE,'A',goal_state);
                    this->root_task->link_query(output_query);
                    this->root_task->resolve_query();
                    // PRX_PRINT("\nStar2: "<<manipulation_model->get_full_state_space()->print_memory(3),PRX_TEXT_RED);

                    if(output_query->found_solution)
                    {
                        cur_hand=req->hand;
                        cur_bin=req->bin[0];
                        PRX_PRINT("current_manipulator_context_name: "<<manipulation_model->get_current_context(), PRX_TEXT_GREEN);
                        PRX_PRINT(" manipulation_model->get_state_space(): "<< manipulation_model->get_state_space()->print_memory(4), PRX_TEXT_GREEN);
                        convert_to_plan_msg(output_query->move_plan,command);
                        send_command(command,output_query->move_plan.length());

                        PRX_PRINT("Sent plan to Simulation: "<<output_query->move_plan.print(2), PRX_TEXT_MAGENTA);
                        manipulation_model->use_context(full_manipulator_context_name);
                        manipulation_model->get_state_space()->free_point(goal_state);
                        
                    }
                    else
                    {
                        q_server->setAborted();
                    }
                }
                else if(req->stage == apc_task_query_t::MOVE_AND_DETECT)
                {
                    //move and detect
                    output_query->setup(req->hand,NULL,apc_task_query_t::MOVE_AND_DETECT,req->bin[0],NULL);
                    this->root_task->link_query(output_query);
                    this->root_task->resolve_query();

                    if(output_query->found_solution)
                    {
                        cur_hand=req->hand;
                        cur_bin=req->bin[0];
                        convert_to_plan_msg(output_query->move_gripper_to_bin,command);
                        send_command(command,output_query->move_gripper_to_bin.length());
                        PRX_PRINT("*PLANNING* Sent plan to Simulation: "<<output_query->move_plan.print(2), PRX_TEXT_GREEN);
                    //     q_server->setSucceeded();
                    }
                    else
                    {
                        PRX_PRINT("*PLANNING* Move and Detect Failed!",PRX_TEXT_RED);
                        q_server->setAborted();
                    }
                }
                else if(req->stage == apc_task_query_t::MOVE_AND_DETECT_TOTE)
                {
                    //move and detect
                    output_query->setup(req->hand,NULL,apc_task_query_t::MOVE_AND_DETECT_TOTE,req->bin[0],NULL);
                    this->root_task->link_query(output_query);
                    this->root_task->resolve_query();

                    if(output_query->found_solution)
                    {
                        cur_hand=req->hand;
                        cur_bin=req->bin[0];
                        convert_to_plan_msg(output_query->move_gripper_to_bin,command);
                        send_command(command,output_query->move_gripper_to_bin.length());
                        PRX_PRINT("*PLANNING* Sent plan to Simulation: "<<output_query->move_plan.print(2), PRX_TEXT_GREEN);
                    //     q_server->setSucceeded();
                    }
                    else
                    {
                        PRX_PRINT("*PLANNING* Move and Detect Failed!",PRX_TEXT_RED);
                        q_server->setAborted();
                    }
                }
                else if(req->stage == apc_task_query_t::PERFORM_GRASP)
                {
                    // PRX_PRINT("Beginning of Perform grasp in planning application: FULL state: "<<manipulation_model->get_full_state_space()->print_memory(4), PRX_TEXT_GREEN);
                    //perform grasp
                    movable_body_plant_t* chosen_object = NULL;
                    chosen_object = update_manipulated_object(req);
                    state_t* object_initial_state = chosen_object->get_state_space()->alloc_point();

                    planning_clock.reset();

                    if (object_to_prioritized_end_effector_context.find(chosen_object->get_object_type()) == object_to_prioritized_end_effector_context.end())
                    {
                        PRX_FATAL_S ("Could not find object: " << chosen_object->get_object_type() << " in the e.e. priority context list");
                    }

                    // TODO: If the prioritized arm fails, 
                    // 1. Pretend both arms are in the home state
                    // 2. Attempt to solve the grasp planning 
                    // 3. If success, plan to move the other arm to home
                    // 4. Prepend the home plan to the grasp plan, send to simulation
                    // 5. If failed, then continue with the list (given things)
                    std::vector<std::string> ee_contexts = object_to_prioritized_end_effector_context[chosen_object->get_object_type()];
                    unsigned context_counter = 0;
                    bool is_left_arm = (ee_contexts[context_counter].find("left") != std::string::npos);
                    manipulation_model->use_context(full_manipulator_context_name);
                    const space_t* full_manipulator_state_space = manipulation_model->get_state_space();
                    bool check, move_arm_to_home = false;
                    do
                    {
                        PRX_PRINT("Attempting to PERFORM GRASP for object : " << chosen_object->get_object_type() << " with context: " << ee_contexts[context_counter], PRX_TEXT_BLUE);
                        check = (ee_contexts[context_counter].find("left") != std::string::npos);
                        if (is_left_arm != check)
                        {
                            // Set both arms to home position here
                            full_manipulator_state_space->set_from_vector(home_position);
                            move_arm_to_home = true;
                        }
                        else
                        {
                            full_manipulator_state_space->copy_from_point(current_manipulator_state);
                            move_arm_to_home = false;
                        }

                        chosen_object->get_state_space()->copy_from_point(object_initial_state);
                        output_query->setup(ee_contexts[context_counter],chosen_object,apc_task_query_t::PERFORM_GRASP,req->bin[0],NULL);
                        
                        this->root_task->link_query(output_query);
                        this->root_task->resolve_query();
                        current_end_effector_index = manipulation_model->get_current_manipulation_info()->end_effector_index;
                        context_counter++;
                    }
                    while (!output_query->found_solution && planning_clock.measure() < planning_budget_time && context_counter < ee_contexts.size());

                    PRX_PRINT("*PLANNING* retrieve_object1: "<<output_query->retrieve_object.length(), PRX_TEXT_GREEN);
                    PRX_DEBUG_COLOR("retrieve_object plan: "<<output_query->retrieve_object.print(2), PRX_TEXT_MAGENTA);

                    chosen_object->get_state_space()->free_point(object_initial_state);
                    if(output_query->found_solution)
                    {
                        current_planning_context_name = ee_contexts[context_counter-1];
                        if (move_arm_to_home)
                        {   
                            manipulation_model->use_context(full_manipulator_context_name);
                            // Set the goal state to home position and reset manipulator to its initial state
                            state_t* goal_state = full_manipulator_state_space->alloc_point();
                            full_manipulator_state_space->set_from_vector(home_position, goal_state);
                            full_manipulator_state_space->copy_from_point(current_manipulator_state);

                            PRX_PRINT ("APC PLANNING APPLICATION FULL STATE: " << manipulation_model->get_full_state_space()->print_memory(), PRX_TEXT_CYAN);
                            
                            // Reset object back to its initial state
                            chosen_object->get_state_space()->copy_from_point(object_initial_state);

                            output_query->setup("",NULL,apc_task_query_t::MOVE,'A',goal_state);
                            this->root_task->link_query(output_query);
                            this->root_task->resolve_query();
                            full_manipulator_state_space->free_point(goal_state);
                            PRX_PRINT("*PLANNING* move_plan: "<<output_query->move_plan.length(), PRX_TEXT_GREEN);
                            PRX_DEBUG_COLOR("move_plan plan: "<<output_query->move_plan.print(2), PRX_TEXT_MAGENTA);

                            if (!output_query->found_solution)
                            {
                                PRX_FATAL_S ("Failed to MOVE TO HOME");
                            }
                            
                            output_query->move_plan += output_query->retrieve_object;
                            output_query->retrieve_object = output_query->move_plan;

                            PRX_PRINT("*PLANNING* retrieve_object2: "<<output_query->retrieve_object.length(), PRX_TEXT_GREEN);
                            PRX_DEBUG_COLOR("retrieve_object plan: "<<output_query->retrieve_object.print(2), PRX_TEXT_MAGENTA);

                            // if (is_left_arm)
                            // {
                            //     // Move left arm to home
                            // }
                            // else
                            // {
                            //     // Move right arm to home
                            // }
                        }
                        cur_hand=req->hand;
                        cur_bin=req->bin[0];
                        // PRX_PRINT("*PLANNING* Sent plan to Simulation: "<<output_query->retrieve_object.print(2), PRX_TEXT_GREEN);
                        // PRX_PRINT("*PLANNING* move_gripper_to_bin: "<<output_query->move_gripper_to_bin.print(2), PRX_TEXT_GREEN);
                        // PRX_PRINT("*PLANNING* approach_object: "<<output_query->approach_object.print(2), PRX_TEXT_GREEN);
                        // PRX_PRINT("*PLANNING* retrieve_object: "<<output_query->retrieve_object.print(2), PRX_TEXT_GREEN);
                        PRX_PRINT("*PLANNING* retrieve_object: "<<output_query->retrieve_object.length(), PRX_TEXT_GREEN);
                        // PRX_PRINT("*PLANNING* move_to_order_bin: "<<output_query->move_to_order_bin.print(2), PRX_TEXT_GREEN);
                        convert_to_plan_msg(output_query->retrieve_object,command);
                        send_command(command,output_query->retrieve_object.length());

                        // PRX_PRINT("Sent plan to Simulation: "<<output_query->approach_object.print(2), PRX_TEXT_MAGENTA);
                        // PRX_PRINT("Sent plan to Simulation: "<<output_query->retrieve_object.print(2), PRX_TEXT_MAGENTA);

                        // convert_to_plan_msg(output_query->approach_object,command);
                        // send_command(command,output_query->approach_object.length());

                        
                        // convert_to_plan_msg(output_query->retrieve_object,command);
                        // send_command(command,output_query->retrieve_object.length());

                    }
                    else
                    {
                        PRX_PRINT("*PLANNING* Grasping Planning Failed!",PRX_TEXT_RED);
                        q_server->setAborted();
                    }
                }

                else if(req->stage == apc_task_query_t::LIFT)
                {
                    PRX_PRINT("Beginning of LIFT in planning application", PRX_TEXT_GREEN);
                    //perform grasp
                    movable_body_plant_t* chosen_object = NULL;
                    chosen_object = update_manipulated_object(req);

                    planning_clock.reset();

                    if (object_to_prioritized_end_effector_context.find(chosen_object->get_object_type()) == object_to_prioritized_end_effector_context.end())
                    {
                        PRX_FATAL_S ("Could not find object: " << chosen_object->get_object_type() << " in the e.e. priority context list");
                    }

                    // TODO: If the prioritized arm fails, 
                    // 1. Pretend both arms are in the home state
                    // 2. Attempt to solve the grasp planning 
                    // 3. If success, plan to move the other arm to home
                    // 4. Prepend the home plan to the grasp plan, send to simulation
                    // 5. If failed, then continue with the list (given things)
                    std::vector<std::string> ee_contexts = object_to_prioritized_end_effector_context[chosen_object->get_object_type()];
                    unsigned context_counter = 0;
                    bool is_left_arm = (ee_contexts[context_counter].find("left") != std::string::npos);
                    
                    const space_t* full_manipulator_state_space = manipulation_model->get_state_space();
                    bool check, move_arm_to_home = false;

/////////////////////////////////////
                    PRX_PRINT("current_planning_context_name"<<current_planning_context_name, PRX_TEXT_GREEN);
                    manipulation_model->use_context(current_planning_context_name);

                    config_t lift_offset(vector_t(0,0,-0.02), quaternion_t(0,0,0,1));
                    config_t current_ee_config, lifted_config;
                    manipulation_model->FK(current_ee_config);

                    lifted_config = lift_offset;
                    lifted_config.relative_to_global(current_ee_config);

                    space_t* current_manipulator_control_space = manipulation_model->get_control_space();
                    state_t* initial_state = manipulation_model->get_state_space()->alloc_point();
                    state_t* temp_initial_state = manipulation_model->get_state_space()->alloc_point();
                    state_t* lifted_state  = manipulation_model->get_state_space()->alloc_point();
                    state_t* returned_state  = manipulation_model->get_state_space()->alloc_point();
                    manipulation_context_info_t* current_manipulation_context_info;
                    current_manipulation_context_info = manipulation_model->get_current_manipulation_info();
                    
                    workspace_trajectory_t ee_traj;
                    sim::plan_t plan1(manipulation_model->get_control_space());
                    // sim::plan_t plan2(manipulation_model->get_control_space());

                    // current_manipulation_context_info->manipulator->scale_control_bounds(0.05);
                    manipulation_model->jac_steering( plan1, ee_traj, lifted_state, initial_state, lifted_config);

                    // HACK TO GET THINGS TO WORK. THIS NEEDS THE ACTUAL GRASPING MODE...
                    // manipulation_model->engage_grasp( plan1, 1.0 , false);
                    // manipulation_model->jac_steering( plan2, ee_traj, returned_state, pushed_state, initial_state, current_ee_config);
                    // current_manipulation_context_info->manipulator->scale_control_bounds(20);
                    // manip_model->jac_steering(plan1, pushed_state, arm_state, pushed_config)

                    // manip_model->jac_steering(plan2, returned_state, pushed_state, global_fk, arm_state)
                    // speedn things up
                    // manipulation_model->get_state_space()->free_point(initial_state);
                    // manipulation_model->get_state_space()->free_point(temp_initial_state);
                    // manipulation_model->get_state_space()->free_point(final_state);
                    // PRX_PRINT("*PLANNING* RETRY_GRASP plan1: "<<plan1.print(), PRX_TEXT_GREEN);
                    // PRX_DEBUG_COLOR("retry_grasp plan1: "<<plan1.print(2), PRX_TEXT_MAGENTA);
                    // PRX_PRINT("*PLANNING* RETRY_GRASP plan2: "<<plan2.print(), PRX_TEXT_GREEN);
                    // PRX_DEBUG_COLOR("retry_grasp plan2: "<<plan2.print(2), PRX_TEXT_MAGENTA);
                    // plan1 += plan2;
                    // PRX_PRINT("*PLANNING* RETRY_GRASP plan1: "<<plan1.print(), PRX_TEXT_GREEN);
                    // PRX_DEBUG_COLOR("retry_grasp plan1: "<<plan1.print(2), PRX_TEXT_MAGENTA);

                    // PRX_DEBUG_COLOR("current_manipulator_control_space"<<);
                    manipulation_model->convert_plan(output_query->retrieve_object, current_manipulation_context_info->full_manipulator_control_space, plan1, current_manipulator_control_space);
                    
                    PRX_PRINT("*PLANNING* RETRY_GRASP retrieve_object: "<<output_query->retrieve_object.length(), PRX_TEXT_GREEN);
                    // PRX_DEBUG_COLOR("RETRY_GRASP retrieve_object plan: "<<output_query->retrieve_object.print(2), PRX_TEXT_MAGENTA);


                    convert_to_plan_msg(output_query->retrieve_object,command);
                    send_command(command,output_query->retrieve_object.length());
                }

                else if(req->stage == apc_task_query_t::RETRY_GRASP)
                {
                    PRX_PRINT("Beginning of RETRY_GRASP in planning application", PRX_TEXT_GREEN);
                    //PRX_PRINT("Beginning of Perform grasp in planning application: FULL state: "<<manipulation_model->get_full_state_space()->print_memory(4), PRX_TEXT_GREEN);
                    //perform grasp
                    movable_body_plant_t* chosen_object = NULL;
                    chosen_object = update_manipulated_object(req);

                    planning_clock.reset();

                    if (object_to_prioritized_end_effector_context.find(chosen_object->get_object_type()) == object_to_prioritized_end_effector_context.end())
                    {
                        PRX_FATAL_S ("Could not find object: " << chosen_object->get_object_type() << " in the e.e. priority context list");
                    }

                    // TODO: If the prioritized arm fails, 
                    // 1. Pretend both arms are in the home state
                    // 2. Attempt to solve the grasp planning 
                    // 3. If success, plan to move the other arm to home
                    // 4. Prepend the home plan to the grasp plan, send to simulation
                    // 5. If failed, then continue with the list (given things)
                    std::vector<std::string> ee_contexts = object_to_prioritized_end_effector_context[chosen_object->get_object_type()];
                    unsigned context_counter = 0;
                    bool is_left_arm = (ee_contexts[context_counter].find("left") != std::string::npos);
                    
                    const space_t* full_manipulator_state_space = manipulation_model->get_state_space();
                    bool check, move_arm_to_home = false;

/////////////////////////////////////
                    PRX_DEBUG_COLOR("current_planning_context_name"<<current_planning_context_name, PRX_TEXT_GREEN);
                    manipulation_model->use_context(current_planning_context_name);

                    config_t push_offset(vector_t(0,0,0.02), quaternion_t(0,0,0,1));
                    config_t current_ee_config, pushed_config;
                    manipulation_model->FK(current_ee_config);

                    pushed_config = push_offset;
                    pushed_config.relative_to_global(current_ee_config);

                    space_t* current_manipulator_control_space = manipulation_model->get_control_space();
                    state_t* initial_state = manipulation_model->get_state_space()->alloc_point();
                    state_t* temp_initial_state = manipulation_model->get_state_space()->alloc_point();
                    state_t* pushed_state  = manipulation_model->get_state_space()->alloc_point();
                    state_t* returned_state  = manipulation_model->get_state_space()->alloc_point();
                    manipulation_context_info_t* current_manipulation_context_info;
                    current_manipulation_context_info = manipulation_model->get_current_manipulation_info();


                    
                    workspace_trajectory_t ee_traj;
                    sim::plan_t plan1(manipulation_model->get_control_space());
                    sim::plan_t plan2(manipulation_model->get_control_space());

//                    current_manipulation_context_info->manipulator->scale_control_bounds(0.05);
                    manipulation_model->jac_steering( plan1, ee_traj, pushed_state, initial_state, pushed_config);

                    // HACK TO GET THINGS TO WORK. THIS NEEDS THE ACTUAL GRASPING MODE...
                    manipulation_model->engage_grasp( plan1, 2.0 , false);
                    manipulation_model->jac_steering( plan2, ee_traj, returned_state, pushed_state, initial_state, current_ee_config);
                   // current_manipulation_context_info->manipulator->scale_control_bounds(20);
                    // manip_model->jac_steering(plan1, pushed_state, arm_state, pushed_config)

                    // manip_model->jac_steering(plan2, returned_state, pushed_state, global_fk, arm_state)
                    // speedn things up
                    // manipulation_model->get_state_space()->free_point(initial_state);
                    // manipulation_model->get_state_space()->free_point(temp_initial_state);
                    // manipulation_model->get_state_space()->free_point(final_state);
                    // PRX_PRINT("*PLANNING* RETRY_GRASP plan1: "<<plan1.print(), PRX_TEXT_GREEN);
                    // PRX_DEBUG_COLOR("retry_grasp plan1: "<<plan1.print(2), PRX_TEXT_MAGENTA);
                    // PRX_PRINT("*PLANNING* RETRY_GRASP plan2: "<<plan2.print(), PRX_TEXT_GREEN);
                    // PRX_DEBUG_COLOR("retry_grasp plan2: "<<plan2.print(2), PRX_TEXT_MAGENTA);

                    // comment out to just push without retract
                    // plan1 += plan2;
                    
                    // PRX_PRINT("*PLANNING* RETRY_GRASP plan1: "<<plan1.print(), PRX_TEXT_GREEN);
                    // PRX_DEBUG_COLOR("retry_grasp plan1: "<<plan1.print(2), PRX_TEXT_MAGENTA);

                    // PRX_DEBUG_COLOR("current_manipulator_control_space"<<);
                    manipulation_model->convert_plan(output_query->retrieve_object, current_manipulation_context_info->full_manipulator_control_space, plan1, current_manipulator_control_space);
                    
                    PRX_PRINT("*PLANNING* RETRY_GRASP retrieve_object: "<<output_query->retrieve_object.length(), PRX_TEXT_GREEN);
                    // PRX_DEBUG_COLOR("RETRY_GRASP retrieve_object plan: "<<output_query->retrieve_object.print(2), PRX_TEXT_MAGENTA);


                    convert_to_plan_msg(output_query->retrieve_object,command);
                    send_command(command,output_query->retrieve_object.length());
            

                    // do
                    // {
                    //     check = (ee_contexts[context_counter].find("left") != std::string::npos);
                    //     if (is_left_arm != check)
                    //     {
                    //         // Set both arms to home position here
                    //         full_manipulator_state_space->set_from_vector(home_position);
                    //         move_arm_to_home = true;
                    //     }
                    //     else
                    //     {
                    //         full_manipulator_state_space->copy_from_point(current_full_state);
                    //         move_arm_to_home = false;
                    //     }
                    //     PRX_DEBUG_COLOR("Attempting to PERFORM GRASP for object : " << chosen_object->get_object_type() << " with context: " << ee_contexts[context_counter], PRX_TEXT_BLUE);
                    //     output_query->setup(ee_contexts[context_counter],chosen_object,apc_task_query_t::PERFORM_GRASP,req->bin[0],NULL);
                        
                    //     this->root_task->link_query(output_query);
                    //     this->root_task->resolve_query();
                    //     current_end_effector_index = manipulation_model->get_current_manipulation_info()->end_effector_index;
                    //     context_counter++;
                    // }
                    // while (!output_query->found_solution && planning_clock.measure() < planning_budget_time && context_counter < ee_contexts.size());

                    // PRX_PRINT("*PLANNING* retrieve_object1: "<<output_query->retrieve_object.length(), PRX_TEXT_GREEN);
                    // PRX_DEBUG_COLOR("retrieve_object plan: "<<output_query->retrieve_object.print(2), PRX_TEXT_MAGENTA);


                    // if(output_query->found_solution)
                    // {
                    //     if (move_arm_to_home)
                    //     {
                    //         state_t* goal_state = full_manipulator_state_space->alloc_point();
                    //         full_manipulator_state_space->set_from_vector(home_position, goal_state);
                    //         full_manipulator_state_space->copy_from_point(current_full_state);
                    //         output_query->setup("",NULL,apc_task_query_t::MOVE,'A',goal_state);
                    //         this->root_task->link_query(output_query);
                    //         this->root_task->resolve_query();
                    //         full_manipulator_state_space->free_point(goal_state);

                    //         PRX_PRINT("*PLANNING* move_plan: "<<output_query->move_plan.length(), PRX_TEXT_GREEN);
                    //         PRX_DEBUG_COLOR("move_plan plan: "<<output_query->move_plan.print(2), PRX_TEXT_MAGENTA);

                    //         if (!output_query->found_solution)
                    //         {
                    //             PRX_FATAL_S ("Failed to MOVE TO HOME");
                    //         }
                            
                    //         output_query->move_plan += output_query->retrieve_object;
                    //         output_query->retrieve_object = output_query->move_plan;

                    //         PRX_PRINT("*PLANNING* retrieve_object2: "<<output_query->retrieve_object.length(), PRX_TEXT_GREEN);
                    //         PRX_DEBUG_COLOR("retrieve_object plan: "<<output_query->retrieve_object.print(2), PRX_TEXT_MAGENTA);

                    //         // if (is_left_arm)
                    //         // {
                    //         //     // Move left arm to home
                    //         // }
                    //         // else
                    //         // {
                    //         //     // Move right arm to home
                    //         // }
                    //     }
                    //     full_manipulator_state_space->free_point(current_full_state);
                    //     cur_hand=req->hand;
                    //     cur_bin=req->bin[0];
                    //     // PRX_PRINT("*PLANNING* Sent plan to Simulation: "<<output_query->retrieve_object.print(2), PRX_TEXT_GREEN);
                    //     // PRX_PRINT("*PLANNING* move_gripper_to_bin: "<<output_query->move_gripper_to_bin.print(2), PRX_TEXT_GREEN);
                    //     // PRX_PRINT("*PLANNING* approach_object: "<<output_query->approach_object.print(2), PRX_TEXT_GREEN);
                    //     // PRX_PRINT("*PLANNING* retrieve_object: "<<output_query->retrieve_object.print(2), PRX_TEXT_GREEN);
                    //     PRX_PRINT("*PLANNING* retrieve_object: "<<output_query->retrieve_object.length(), PRX_TEXT_GREEN);
                    //     // PRX_PRINT("*PLANNING* move_to_order_bin: "<<output_query->move_to_order_bin.print(2), PRX_TEXT_GREEN);
                    //     convert_to_plan_msg(output_query->retrieve_object,command);
                    //     send_command(command,output_query->retrieve_object.length());

                    //     // PRX_PRINT("Sent plan to Simulation: "<<output_query->approach_object.print(2), PRX_TEXT_MAGENTA);
                    //     // PRX_PRINT("Sent plan to Simulation: "<<output_query->retrieve_object.print(2), PRX_TEXT_MAGENTA);

                    //     // convert_to_plan_msg(output_query->approach_object,command);
                    //     // send_command(command,output_query->approach_object.length());

                        
                    //     // convert_to_plan_msg(output_query->retrieve_object,command);
                    //     // send_command(command,output_query->retrieve_object.length());

                    // }
                    // else
                    // {
                    //     PRX_PRINT("*PLANNING* Grasping Planning Failed!",PRX_TEXT_RED);
                    //     q_server->setAborted();
                    // }
                }

                // else if(req->stage == apc_task_query_t::RETRY_GRASP)
                // {
                //     // PRX_PRINT("Beginning of Perform grasp in planning application: FULL state: "<<manipulation_model->get_full_state_space()->print_memory(4), PRX_TEXT_GREEN);
                //     //perform grasp
                //     movable_body_plant_t* chosen_object = NULL;
                //     chosen_object = update_manipulated_object(req);

                //     planning_clock.reset();

                //     if (object_to_prioritized_end_effector_context.find(chosen_object->get_object_type()) == object_to_prioritized_end_effector_context.end())
                //     {
                //         PRX_FATAL_S ("Could not find object: " << chosen_object->get_object_type() << " in the e.e. priority context list");
                //     }

                //     // TODO: If the prioritized arm fails, 
                //     // 1. Pretend both arms are in the home state
                //     // 2. Attempt to solve the grasp planning 
                //     // 3. If success, plan to move the other arm to home
                //     // 4. Prepend the home plan to the grasp plan, send to simulation
                //     // 5. If failed, then continue with the list (given things)
                //     std::vector<std::string> ee_contexts = object_to_prioritized_end_effector_context[chosen_object->get_object_type()];
                //     unsigned context_counter = 0;
                //     bool is_left_arm = (ee_contexts[context_counter].find("left") != std::string::npos);
                //     manipulation_model->use_context(full_manipulator_context_name);
                //     const space_t* full_manipulator_state_space = manipulation_model->get_state_space();
                //     state_t* current_full_state = full_manipulator_state_space->alloc_point();
                //     bool check, move_arm_to_home = false;
                //     do
                //     {
                //         check = (ee_contexts[context_counter].find("left") != std::string::npos);
                //         if (is_left_arm != check)
                //         {
                //             // Set both arms to home position here
                //             full_manipulator_state_space->set_from_vector(home_position);
                //             move_arm_to_home = true;
                //         }
                //         else
                //         {
                //             full_manipulator_state_space->copy_from_point(current_full_state);
                //             move_arm_to_home = false;
                //         }
                //         PRX_DEBUG_COLOR("Attempting to PERFORM GRASP for object : " << chosen_object->get_object_type() << " with context: " << ee_contexts[context_counter], PRX_TEXT_BLUE);
                //         output_query->setup(ee_contexts[context_counter],chosen_object,apc_task_query_t::PERFORM_GRASP,req->bin[0],NULL);
                        
                //         this->root_task->link_query(output_query);
                //         this->root_task->resolve_query();
                //         current_end_effector_index = manipulation_model->get_current_manipulation_info()->end_effector_index;
                //         context_counter++;
                //     }
                //     while (!output_query->found_solution && planning_clock.measure() < planning_budget_time && context_counter < ee_contexts.size());

                //     PRX_PRINT("*PLANNING* retrieve_object1: "<<output_query->retrieve_object.length(), PRX_TEXT_GREEN);
                //     PRX_DEBUG_COLOR("retrieve_object plan: "<<output_query->retrieve_object.print(2), PRX_TEXT_MAGENTA);


                //     if(output_query->found_solution)
                //     {
                //         if (move_arm_to_home)
                //         {
                //             state_t* goal_state = full_manipulator_state_space->alloc_point();
                //             full_manipulator_state_space->set_from_vector(home_position, goal_state);
                //             full_manipulator_state_space->copy_from_point(current_full_state);
                //             output_query->setup("",NULL,apc_task_query_t::MOVE,'A',goal_state);
                //             this->root_task->link_query(output_query);
                //             this->root_task->resolve_query();
                //             full_manipulator_state_space->free_point(goal_state);

                //             PRX_PRINT("*PLANNING* move_plan: "<<output_query->move_plan.length(), PRX_TEXT_GREEN);
                //             PRX_DEBUG_COLOR("move_plan plan: "<<output_query->move_plan.print(2), PRX_TEXT_MAGENTA);

                //             if (!output_query->found_solution)
                //             {
                //                 PRX_FATAL_S ("Failed to MOVE TO HOME");
                //             }
                            
                //             output_query->move_plan += output_query->retrieve_object;
                //             output_query->retrieve_object = output_query->move_plan;

                //             PRX_PRINT("*PLANNING* retrieve_object2: "<<output_query->retrieve_object.length(), PRX_TEXT_GREEN);
                //             PRX_DEBUG_COLOR("retrieve_object plan: "<<output_query->retrieve_object.print(2), PRX_TEXT_MAGENTA);

                //             // if (is_left_arm)
                //             // {
                //             //     // Move left arm to home
                //             // }
                //             // else
                //             // {
                //             //     // Move right arm to home
                //             // }
                //         }
                //         full_manipulator_state_space->free_point(current_full_state);
                //         cur_hand=req->hand;
                //         cur_bin=req->bin[0];
                //         // PRX_PRINT("*PLANNING* Sent plan to Simulation: "<<output_query->retrieve_object.print(2), PRX_TEXT_GREEN);
                //         // PRX_PRINT("*PLANNING* move_gripper_to_bin: "<<output_query->move_gripper_to_bin.print(2), PRX_TEXT_GREEN);
                //         // PRX_PRINT("*PLANNING* approach_object: "<<output_query->approach_object.print(2), PRX_TEXT_GREEN);
                //         // PRX_PRINT("*PLANNING* retrieve_object: "<<output_query->retrieve_object.print(2), PRX_TEXT_GREEN);
                //         PRX_PRINT("*PLANNING* retrieve_object: "<<output_query->retrieve_object.length(), PRX_TEXT_GREEN);
                //         // PRX_PRINT("*PLANNING* move_to_order_bin: "<<output_query->move_to_order_bin.print(2), PRX_TEXT_GREEN);
                //         convert_to_plan_msg(output_query->retrieve_object,command);
                //         send_command(command,output_query->retrieve_object.length());

                //         // PRX_PRINT("Sent plan to Simulation: "<<output_query->approach_object.print(2), PRX_TEXT_MAGENTA);
                //         // PRX_PRINT("Sent plan to Simulation: "<<output_query->retrieve_object.print(2), PRX_TEXT_MAGENTA);

                //         // convert_to_plan_msg(output_query->approach_object,command);
                //         // send_command(command,output_query->approach_object.length());

                        
                //         // convert_to_plan_msg(output_query->retrieve_object,command);
                //         // send_command(command,output_query->retrieve_object.length());

                //     }
                //     else
                //     {
                //         PRX_PRINT("*PLANNING* Grasping Planning Failed!",PRX_TEXT_RED);
                //         q_server->setAborted();
                //     }
                // }

                else if(req->stage == apc_task_query_t::REMOVE_FROM_TOTE)
                {
                    // PRX_PRINT("Beginning of Perform grasp in planning application: FULL state: "<<manipulation_model->get_full_state_space()->print_memory(4), PRX_TEXT_GREEN);
                    //perform grasp
                    movable_body_plant_t* chosen_object = NULL;
                    chosen_object = update_manipulated_object(req);

                    planning_clock.reset();

                    if (object_to_prioritized_end_effector_context.find(chosen_object->get_object_type()) == object_to_prioritized_end_effector_context.end())
                    {
                        PRX_FATAL_S ("Could not find object: " << chosen_object->get_object_type() << " in the e.e. priority context list");
                    }

                    // TODO: If the prioritized arm fails, 
                    // 1. Pretend both arms are in the home state
                    // 2. Attempt to solve the grasp planning 
                    // 3. If success, plan to move the other arm to home
                    // 4. Prepend the home plan to the grasp plan, send to simulation
                    // 5. If failed, then continue with the list (given things)
                    std::vector<std::string> ee_contexts = object_to_prioritized_end_effector_context[chosen_object->get_object_type()];
                    unsigned context_counter = 0;
                    do
                    {
                        PRX_DEBUG_COLOR("Attempting to PERFORM GRASP for object : " << chosen_object->get_object_type() << " with context: " << ee_contexts[context_counter], PRX_TEXT_BLUE);
                        output_query->setup(ee_contexts[context_counter],chosen_object,apc_task_query_t::REMOVE_FROM_TOTE,req->bin[0],NULL);
                        this->root_task->link_query(output_query);
                        this->root_task->resolve_query();
                        context_counter++;
                    }
                    while (!output_query->found_solution && planning_clock.measure() < planning_budget_time && context_counter < ee_contexts.size());

                    if(output_query->found_solution)
                    {
                        cur_hand=req->hand;
                        cur_bin=req->bin[0];
                        // PRX_PRINT("*PLANNING* Sent plan to Simulation: "<<output_query->retrieve_object.print(2), PRX_TEXT_GREEN);
                        // PRX_PRINT("*PLANNING* move_gripper_to_bin: "<<output_query->move_gripper_to_bin.print(2), PRX_TEXT_GREEN);
                        // PRX_PRINT("*PLANNING* approach_object: "<<output_query->approach_object.print(2), PRX_TEXT_GREEN);
                        // PRX_PRINT("*PLANNING* retrieve_object: "<<output_query->retrieve_object.print(2), PRX_TEXT_GREEN);
                        PRX_PRINT("*PLANNING* retrieve_object: "<<output_query->retrieve_object.length(), PRX_TEXT_GREEN);
                        // PRX_PRINT("*PLANNING* move_to_order_bin: "<<output_query->move_to_order_bin.print(2), PRX_TEXT_GREEN);
                        convert_to_plan_msg(output_query->retrieve_object,command);
                        send_command(command,output_query->retrieve_object.length());

                        // PRX_PRINT("Sent plan to Simulation: "<<output_query->approach_object.print(2), PRX_TEXT_MAGENTA);
                        // PRX_PRINT("Sent plan to Simulation: "<<output_query->retrieve_object.print(2), PRX_TEXT_MAGENTA);

                        // convert_to_plan_msg(output_query->approach_object,command);
                        // send_command(command,output_query->approach_object.length());

                        
                        // convert_to_plan_msg(output_query->retrieve_object,command);
                        // send_command(command,output_query->retrieve_object.length());

                    }
                    else
                    {
                        PRX_PRINT("*PLANNING* Grasping Planning Failed!",PRX_TEXT_RED);
                        q_server->setAborted();
                    }
                }
                else if(req->stage == apc_task_query_t::MOVE_TO_OTHER_BIN)
                {
                    cur_hand=req->hand;
                    cur_bin=req->bin[0];
                    //perform grasp
                    movable_body_plant_t* chosen_object = NULL;
                    chosen_object = update_manipulated_object(req);
                    output_query->setup(req->hand,chosen_object,apc_task_query_t::MOVE_TO_OTHER_BIN,req->bin[0],NULL,req->final_obj_state);
                    PRX_PRINT("Query Setup", PRX_TEXT_GREEN);
                    this->root_task->link_query(output_query);
                    this->root_task->resolve_query();
                    PRX_PRINT("Query Resolved", PRX_TEXT_GREEN);
                    if(output_query->found_solution)
                    {
                        // PRX_PRINT("*PLANNING* Sent plan to Simulation: "<<output_query->retrieve_object.print(2), PRX_TEXT_GREEN);
                        convert_to_plan_msg(output_query->retrieve_object,command);
                        send_command(command,output_query->retrieve_object.length());

                        // PRX_PRINT("Sent plan to Simulation: "<<output_query->approach_object.print(2), PRX_TEXT_MAGENTA);
                        // PRX_PRINT("Sent plan to Simulation: "<<output_query->retrieve_object.print(2), PRX_TEXT_MAGENTA);

                        // convert_to_plan_msg(output_query->approach_object,command);
                        // send_command(command,output_query->approach_object.length());

                        
                        // convert_to_plan_msg(output_query->retrieve_object,command);
                        // send_command(command,output_query->retrieve_object.length());

                    }
                    else
                    {
                        PRX_PRINT("*PLANNING* Grasping Planning Failed!",PRX_TEXT_RED);
                        q_server->setAborted();
                    }
                }
                else if(req->stage == apc_task_query_t::MOVE_OUTSIDE_BIN)
                {
                    cur_hand=req->hand;
                    cur_bin=req->bin[0];
                    //MOVE_OUTSIDE_BIN
                    PRX_INFO_S("Setting up query: MOVE_OUTSIDE_BIN");
                    movable_body_plant_t* chosen_object = NULL;
                    chosen_object = update_manipulated_object(req);
                    output_query->setup(req->hand,chosen_object,apc_task_query_t::MOVE_OUTSIDE_BIN,req->bin[0],NULL,req->final_obj_state);
                    PRX_PRINT("Query Setup", PRX_TEXT_GREEN);
                    this->root_task->link_query(output_query);
                    this->root_task->resolve_query();
                    PRX_PRINT("Query Resolved", PRX_TEXT_GREEN);
                    if(output_query->found_solution)
                    {
                        PRX_PRINT("*PLANNING* Sent plan to Simulation: "<<output_query->retrieve_object.print(2), PRX_TEXT_GREEN);
                        convert_to_plan_msg(output_query->retrieve_object,command);
                        send_command(command,output_query->retrieve_object.length());
                    }
                    else
                    {
                        PRX_PRINT("*PLANNING* MOVE_OUTSIDE_BIN Failed!",PRX_TEXT_RED);
                        q_server->setAborted();
                    }
                }
                else if(req->stage == apc_task_query_t::ADJUST_EE)
                {
                    cur_hand=req->hand;
                    cur_bin=req->bin[0];
                    //perform grasp
                    movable_body_plant_t* chosen_object = NULL;
                    chosen_object = update_manipulated_object(req);
                    output_query->setup(req->hand,chosen_object,apc_task_query_t::ADJUST_EE,req->bin[0],NULL);
                    this->root_task->link_query(output_query);
                    this->root_task->resolve_query();
                    if(output_query->found_solution)
                    {
                        PRX_PRINT("*PLANNING* Sent plan to Simulation: "<<output_query->retrieve_object.print(2), PRX_TEXT_GREEN);
                        convert_to_plan_msg(output_query->retrieve_object,command);
                        send_command(command,output_query->retrieve_object.length());

                        // PRX_PRINT("Sent plan to Simulation: "<<output_query->approach_object.print(2), PRX_TEXT_MAGENTA);
                        // PRX_PRINT("Sent plan to Simulation: "<<output_query->retrieve_object.print(2), PRX_TEXT_MAGENTA);

                        // convert_to_plan_msg(output_query->approach_object,command);
                        // send_command(command,output_query->approach_object.length());

                        
                        // convert_to_plan_msg(output_query->retrieve_object,command);
                        // send_command(command,output_query->retrieve_object.length());

                    }
                    else
                    {
                        PRX_PRINT("*PLANNING* Grasping Planning Failed!",PRX_TEXT_RED);
                        q_server->setAborted();
                    }
                }
                else if(req->stage == apc_task_query_t::MOVE_TO_ORDER_BIN)
                {
                    cur_hand=req->hand;
                    cur_bin=req->bin[0];
                    // PRX_PRINT("FULL state: "<<manipulation_model->get_full_state_space()->print_memory(), PRX_TEXT_GREEN);
                    // state_t* full_state = manipulation_model->get_full_state_space()->alloc_point();
                    //move to order bin
                    output_query->setup(req->hand,NULL,apc_task_query_t::MOVE_TO_ORDER_BIN,'A',NULL);
                    this->root_task->link_query(output_query);
                    this->root_task->resolve_query();

                    if(output_query->found_solution)
                    {
                        convert_to_plan_msg(output_query->move_to_order_bin,command);
                        send_command(command,output_query->move_to_order_bin.length());

                        PRX_PRINT("Sent plan to Simulation: "<<output_query->move_to_order_bin.print(2), PRX_TEXT_MAGENTA);
                        manipulation_model->use_context(full_manipulator_context_name);

                        
                        // trajectory_t output_trajectory(manipulation_model->get_full_state_space()); 
                        // manipulation_model->propagate_plan(full_state, output_query->move_to_order_bin, output_trajectory);
                        // PRX_PRINT ("Successful move trajectory: " << output_trajectory.print(), PRX_TEXT_GREEN);

                        // q_server->setSucceeded();
                    }
                    else
                    {
                        q_server->setAborted();
                    }
                }
                else if(req->stage == apc_task_query_t::EXECUTE_SAVED_TRAJ)
                {
                    cur_hand=req->hand;
                    cur_bin=req->bin[0];
                    /////////////EXECUTE SAVED PLANS ONLY/////////////////////////////////
                    std::ostringstream file_name;
                    
                    file_name << "/bin_trajectories/"<<req->saved_plan_path;
                    PRX_PRINT("Executing plan in file: "<<file_name.str(),PRX_TEXT_RED);
                    execute_plan_from_file(file_name.str().c_str());
                    
                    // q_server->setSucceeded();
                    /////////////////////////////////////////////////////////////////////
                }
                else if(req->stage == apc_task_query_t::THREE_STAGE_TRAJECTORY_SECOND)
                {
                    //three stage traj
                    PRX_PRINT("@PLANNING APP: Initializing 3 Stage Trajectory...", PRX_TEXT_BLUE);
                    output_query->setup(req->hand,NULL,apc_task_query_t::THREE_STAGE_TRAJECTORY_SECOND,req->bin[0],NULL);
                    this->root_task->link_query(output_query);
                    this->root_task->resolve_query();

                    if(output_query->found_solution)
                    {
                        cur_hand=req->hand;
                        cur_bin=req->bin[0];
                        convert_to_plan_msg(output_query->move_gripper_to_bin,command);
                        send_command(command,output_query->move_gripper_to_bin.length());
                        PRX_PRINT("*PLANNING* Sent plan to Simulation: "<<output_query->move_plan.print(2), PRX_TEXT_GREEN);
                    //     q_server->setSucceeded();
                    }
                    else
                    {
                        PRX_PRINT("*PLANNING* Move and Detect Failed!",PRX_TEXT_RED);
                        q_server->setAborted();
                    }
                }
                else if(req->stage == apc_task_query_t::THREE_STAGE_TRAJECTORY_THIRD)
                {
                    //three stage traj
                    PRX_PRINT("@PLANNING APP: Initializing 3 Stage Trajectory...", PRX_TEXT_BLUE);
                    output_query->setup(req->hand,NULL,apc_task_query_t::THREE_STAGE_TRAJECTORY_THIRD,req->bin[0],NULL);
                    this->root_task->link_query(output_query);
                    this->root_task->resolve_query();

                    if(output_query->found_solution)
                    {
                        cur_hand=req->hand;
                        cur_bin=req->bin[0];
                        convert_to_plan_msg(output_query->move_gripper_to_bin,command);
                        send_command(command,output_query->move_gripper_to_bin.length());
                        PRX_PRINT("*PLANNING* Sent plan to Simulation: "<<output_query->move_plan.print(2), PRX_TEXT_GREEN);
                    //     q_server->setSucceeded();
                    }
                    else
                    {
                        PRX_PRINT("*PLANNING* Move and Detect Failed!",PRX_TEXT_RED);
                        q_server->setAborted();
                    }
                }
                else
                {
                    PRX_WARN_S ("Invalid state received");
                }
                output_query->clear();

            }

            void apc_planning_application_t::send_command(prx_simulation::send_trajectoryGoal& goal,double timeout)
            {
                prx_planning::apc_queryResult res;
                res.plan = goal.plan;
                res.reaching = goal.reaching;
                res.reaching_duration = goal.reaching_plan_time;
                PRX_PRINT("reaching_duration: "<<res.reaching_duration, PRX_TEXT_GREEN);
                res.grasping = goal.grasping;
                res.grasping_duration = goal.grasping_plan_time;
                PRX_PRINT("grasping_duration: "<<res.grasping_duration, PRX_TEXT_GREEN);
                res.retracting = goal.retracting;
                res.retracting_duration = goal.retracting_plan_time;
                PRX_PRINT("retracting_duration: "<<res.retracting_duration, PRX_TEXT_GREEN);
                res.duration = timeout;
                
                q_server->setSucceeded(res);

                // execute_client->sendGoal(goal);
                // bool finished = false;

                // finished = execute_client->waitForResult(ros::Duration(0));
                // PRX_PRINT("finished = "<<finished,PRX_TEXT_GREEN);
                // sleep(2);

            }

            void apc_planning_application_t::convert_to_plan_msg(sim::plan_t& in_plan, prx_simulation::send_trajectoryGoal& goal)
            {
                //////////////////SAVE PLANS TO FILE////////////////////////
                if(record_plans){
                    if(prev_hand != cur_hand)
                    {
                        stage_counter = 0;
                    }
                    char* w = std::getenv("PRACSYS_PATH");
                    std::string dir(w);
                    dir += ("/prx_input/bin_trajectories/");
                    std::ostringstream file_name;
                    if(in_plan.size()>15){
                        stage_counter++;
                    }
                    file_name << dir << cur_hand<<"_"<<cur_bin<<"_"<<stage_counter;                    
                    PRX_PRINT("file_name:"<<file_name.str(),PRX_TEXT_GREEN);
                    in_plan.save_to_file(file_name.str().c_str());
                    prev_hand = cur_hand;
                }
                // cur_hand = "home";
                ////////////////////////////////////////////////////////////

                // manipulation_model->use_context(full_manipulator_context_name);
                // PRX_PRINT("cur_hand: "<<cur_hand, PRX_TEXT_CYAN);
                // state_t* reached_connection_state = manipulation_model->get_state_space()->alloc_point();
                // PRX_PRINT("Beginning of Perform grasp in planning application: FULL state: "<<manipulation_model->get_state_space()->print_memory(4), PRX_TEXT_GREEN);

                // trajectory_t reaching_path( manipulation_model->get_state_space() );
                // reaching_path.clear();
                // //Repropagate the reaching plan to get an updated/refined/true reaching trajectory

                // manipulation_model->propagate_plan( reached_connection_state, in_plan, reaching_path );
                // util::constraints_t* tmp_constraint;
                // tmp_constraint = validity_checker->alloc_constraint();

                // PRX_PRINT("current_manipulator_context_name: "<<manipulation_model->get_current_context(), PRX_TEXT_GREEN);
                // PRX_PRINT(" manipulation_model->get_state_space(): "<< manipulation_model->get_state_space()->print_memory(4), PRX_TEXT_GREEN);
                      
                // bool result = validity_checker->validate_and_generate_constraints( tmp_constraint, reaching_path );
                // PRX_PRINT("validity_checker result: "<<result, PRX_TEXT_GREEN);

                PRX_PRINT("current_end_effector_index: "<<current_end_effector_index, PRX_TEXT_RED);
                int gripper_control_index = 16 + current_end_effector_index;

                manipulation_model->use_context(full_manipulator_context_name);
                space_t* control_space = manipulation_model->get_control_space();
                space_t* state_space = manipulation_model->get_state_space();
                sim::plan_t gripper_on_plan;
                gripper_on_plan.link_control_space(control_space);
                gripper_on_plan.clear();

                goal.plan.clear();
                goal.traj.clear();
                goal.reaching.clear();
                goal.grasping.clear();
                goal.retracting.clear();

                int counter = 0;
                int early_grasp = 0;
                int reaching_end = 0;
                int grasping_end = 0;
                int retracting_end = 0;
                double duration_threshold = 3.0;
                bool has_early_grasp = false;

                // PRX_DEBUG_COLOR("Plan Before: "<<in_plan.print(2), PRX_TEXT_RED);

                // if (cur_hand == "left")
                // {
                //     gripper_control_index = 16;
                // }
                // else
                // {
                //     gripper_control_index = 18;
                // }



                foreach(plan_step_t step, in_plan)
                {                    
                    // double before_grasp_offset = 10;
                    // double after_grasp_offset = 100;
                    
                    // if(counter>0 && step.control->at(gripper_control_index)==2)
                    if(step.control->at(gripper_control_index) > 1) 
                    {
                        PRX_PRINT("Found early grasp at index: "<<counter, PRX_TEXT_CYAN); 
                        /*
                          Reaching -> [0,reaching_end]
                          Grasping -> [reaching_end+1, grasping_end]
                          Retracting->[grasping_end+1, retracting_end]
                        */
                        // early_grasp = counter - before_grasp_offset;
                        has_early_grasp = true;
                        int _i = counter;
                        for( double duration = 0; duration<duration_threshold && _i >= 0; duration+=in_plan[_i].duration, _i--)
                        {
                            // PRX_PRINT("Figuring out early grasp... "<<_i<<", duration: "<<duration<<", counter... "<<counter, PRX_TEXT_GREEN);
                        }
                        early_grasp = _i+1;                        
                        reaching_end = early_grasp;

                        // _i = counter;
                        // for( double duration = 0; duration<duration_threshold && _i >= 0; duration+=in_plan[_i].duration, _i++)
                        // {
                        //     // PRX_PRINT("Figuring out early grasp... "<<_i<<", duration: "<<duration<<", counter... "<<counter, PRX_TEXT_GREEN);
                        // }
                        grasping_end = counter;//_i-1;

                        break;
                        // PRX_PRINT("step.control->at(16) = "<<step.control->at(16)<<"  duration = "<<step.duration,PRX_TEXT_CYAN);
                    }
                    counter ++;
                }

                PRX_PRINT(" early_grasp: "<<early_grasp<<" reaching_end: "<<reaching_end<<" grasping_end: "<<grasping_end,PRX_TEXT_CYAN);
                

                counter = 0;
                foreach(plan_step_t step, in_plan)
                {
                    
                    if(has_early_grasp && counter == reaching_end+1){
                        control_t* gripper_control = control_space->alloc_point();
                        control_space->zero(gripper_control);

                        // early vaccum on
                        //adding a control of 3 so we distinguish between an early turn on of the vacuum
                        PRX_PRINT("Adding early grasp!!! ", PRX_TEXT_CYAN); 
                        gripper_control->at(gripper_control_index)=3;
                        gripper_on_plan.copy_onto_back(gripper_control, 0.02);  
                    }
                    gripper_on_plan.copy_onto_back(step.control,step.duration);
                    counter++;
                }
                // PRX_PRINT("Plan After adding  early grasp: "<<gripper_on_plan.print(2), PRX_TEXT_RED);
                retracting_end = counter+2;
            

                control_t* release_object = control_space->alloc_point();
                control_space->zero(release_object);
                release_object->at(gripper_control_index)=1;
                gripper_on_plan.copy_onto_back(release_object, 0.02);

                // PRX_PRINT("Plan After adding release_object: "<<gripper_on_plan.print(2), PRX_TEXT_RED);

                foreach(plan_step_t step, gripper_on_plan)
                {
                    goal.plan.push_back(prx_simulation::control_msg());
                    goal.plan.back().duration = step.duration;
                    int i=0;
                    for(i=0;i<control_space->get_dimension();++i)
                    {
                        goal.plan.back().control.push_back(step.control->at(i));
                    }
                }


                //devide the plan into 3 parts: reaching, grasping, retracting    
                counter = 0;
                float reaching_time=0.0;
                float grasping_time=0.0;
                float retracting_time=0.0;
                if (reaching_end == 0 && grasping_end == 0)
                {
                    PRX_WARN_S ("Empty plan");
                }
                else
                {

                    foreach(plan_step_t step, gripper_on_plan)
                    {                  
                        
                        if(counter < reaching_end){
                            goal.reaching.push_back(prx_simulation::control_msg());
                            goal.reaching.back().duration = step.duration;
                            reaching_time+=step.duration;
                            int i=0;
                            for(i=0;i<control_space->get_dimension();++i)
                            {
                                goal.reaching.back().control.push_back(step.control->at(i));
                            }
                            // PRX_PRINT("Reaching Control@16: "<<step.control->at(16),PRX_TEXT_RED);
                        }
                        if(counter >= reaching_end && counter <= grasping_end){
                            goal.grasping.push_back(prx_simulation::control_msg());
                            goal.grasping.back().duration = step.duration;
                            grasping_time+=step.duration;
                            int i=0;
                            for(i=0;i<control_space->get_dimension();++i)
                            {
                                goal.grasping.back().control.push_back(step.control->at(i));
                            }
                            // PRX_PRINT("Control@16: "<<step.control->at(16),PRX_TEXT_GREEN);
                        }
                        if(counter > grasping_end && counter <= retracting_end){
                            goal.retracting.push_back(prx_simulation::control_msg());
                            goal.retracting.back().duration = step.duration;
                            retracting_time+=step.duration;
                            int i=0;
                            for(i=0;i<control_space->get_dimension();++i)
                            {
                                goal.retracting.back().control.push_back(step.control->at(i));
                            }
                            // PRX_PRINT("Retracting Control@16: "<<step.control->at(16),PRX_TEXT_BLUE);
                        }
                        counter++;
                    }
                }
                goal.reaching_plan_time = reaching_time;
                goal.grasping_plan_time = grasping_time;
                goal.retracting_plan_time = retracting_time;

            }

            void apc_planning_application_t::get_state_callback(const prx_simulation::state_msg& stateMsg)
            {
                for(unsigned i=0;i<stateMsg.elements.size();i++)
                {
                    current_manipulator_state->at(i) = stateMsg.elements[i];
                }
            }

            void apc_planning_application_t::execute_plan_from_file(std::string file_name)
            {
                prx_simulation::send_trajectoryGoal goal;
                PRX_PRINT("Reading plan from file: "<<file_name,PRX_TEXT_CYAN);
                manipulation_model->use_context(full_manipulator_context_name);
                space_t* control_space = manipulation_model->get_control_space();
                saved_plan.link_control_space(control_space);
                saved_plan.clear();
                saved_plan.read_from_file(file_name.c_str());
                PRX_PRINT("Executing plan of length: "<<saved_plan.length(), PRX_TEXT_MAGENTA);
                // convert_to_plan_msg(saved_plan, goal);
                // send_command(goal,saved_plan.length());
                prx_simulation::send_trajectoryGoal command;
                convert_to_plan_msg(saved_plan,command);
                send_command(command,saved_plan.length());     
            }

            void apc_planning_application_t::initialize_objects_pose()
            {
                std::vector<double> dummy_state;
                dummy_state.push_back(3);
                dummy_state.push_back(3);
                dummy_state.push_back(3);
                dummy_state.push_back(0);
                dummy_state.push_back(0);
                dummy_state.push_back(0);
                dummy_state.push_back(1);
                foreach(movable_body_plant_t* manip_obj, objects)
                {
                    manip_obj->get_state_space()->set_from_vector(dummy_state);
                    prx_simulation::object_msg object_placement;
                    object_placement.name = manip_obj->get_object_type();
                    PRX_INFO_S("Set up "<<object_placement.name);
                    dummy_state[0]+=2;
                    dummy_state[1]+=2;
                    dummy_state[2]+=2;
                }
            }
            void apc_planning_application_t::update_object_callback(const prx_simulation::object_msg& objectMsg)
            {
                config_t new_config;
                new_config.zero();
                std::vector<double> new_config_vec;
                foreach(movable_body_plant_t* plant, objects)
                {
                   // PRX_PRINT("Try Setting "<<objectMsg.name, PRX_TEXT_RED);
                    if(plant->get_object_type() == objectMsg.name)
                    {
                        moved = plant;
                        // moved->get_state_space()->set_from_vector(objectMsg.elements);
                        new_config.set_position(objectMsg.elements[0],objectMsg.elements[1],objectMsg.elements[2]);
                        new_config.set_orientation(objectMsg.elements[3],objectMsg.elements[4],objectMsg.elements[5],objectMsg.elements[6]);
                        moved->set_configuration(new_config);
                        moved->update_collision_info();
                        //PRX_PRINT("*PLANNING*: Updated "<<moved->get_object_type(), PRX_TEXT_RED);
                        // moved->print_configuration();
                        moved = NULL;
                    }
                }

                //UPDATE SHELF POSITION
                new_config.zero();
                if (shelf_update_counter < 2)
                {
                    foreach(sim::obstacle_t* obst, obstacles)
                    {
                        // PRX_PRINT("PLANNING APP: Obstacle = "<<obst->get_pathname()<<", config: " <<obst->get_root_configuration().print(),PRX_TEXT_GREEN);
                        if(obst->get_pathname().find(objectMsg.name)!=std::string::npos) 
                        {
                            shelf = obst;
                            new_config.set_position(objectMsg.elements[0],objectMsg.elements[1],objectMsg.elements[2]);
                            new_config.set_orientation(objectMsg.elements[3],objectMsg.elements[4],objectMsg.elements[5],objectMsg.elements[6]);
                            shelf->update_root_configuration(new_config);
                            shelf->update_collision_info();
                            //PRX_PRINT("*PLANNING*: Updated the shelf's position: "<<shelf->get_root_configuration().print(),PRX_TEXT_RED);
                            shelf = NULL;
                        } 
                    }
                    shelf_update_counter++;
                }
            }

            movable_body_plant_t* apc_planning_application_t::update_manipulated_object(const prx_planning::apc_queryGoalConstPtr& req)
            {
                std::vector<movable_body_plant_t* > objects;
                manipulation_model->get_objects(objects);

                movable_body_plant_t* chosen_object = NULL;
                int count=0;
                std::vector<double> dummy_state;
                dummy_state.push_back(3);
                dummy_state.push_back(3);
                dummy_state.push_back(3);
                dummy_state.push_back(0);
                dummy_state.push_back(0);
                dummy_state.push_back(0);
                dummy_state.push_back(1);

                foreach(movable_body_plant_t* manip_obj, objects)
                {
                    if(manip_obj->get_object_type()==req->object)
                    {
                        chosen_object = manip_obj;
                        chosen_object->get_state_space()->set_from_vector(req->object_state);
                        chosen_object->update_collision_info();
                        prx_simulation::object_msg object_placement;
                        object_placement.name = chosen_object->get_object_type();
                        for(unsigned i=0;i<7;i++)
                        {
                            object_placement.elements.push_back(req->object_state[i]);
                        }
                    }
                    // else
                    // {
                    //     manip_obj->get_state_space()->set_from_vector(dummy_state);
                    //     prx_simulation::object_msg object_placement;
                    //     object_placement.name = manip_obj->get_object_type();
                    //     for(unsigned i=0;i<7;i++)
                    //     {
                    //         object_placement.elements.push_back(dummy_state[i]);
                    //     }
                    //     dummy_state[0]+=2;
                    //     dummy_state[1]+=2;
                    //     dummy_state[2]+=2;     
                    //     PRX_INFO_S("*PLANNING*: Setting "<<manip_obj->get_object_type());                   
                    // }

                }
                return chosen_object;
            }
        }
    }
}
