/**
 * @file manipulation_world_model.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#pragma once

#ifndef PRACSYS_MANIPULATION_WORLD_MODEL_HPP
#define PRACSYS_MANIPULATION_WORLD_MODEL_HPP

#include "prx/planning/world_model.hpp"
#include "simulation/systems/plants/manipulator.hpp"
#include "simulation/simulators/manipulation_simulator.hpp"
#include "simulation/workspace_trajectory.hpp"
#include "planning/modules/IK_database.hpp"

#include <pluginlib/class_loader.h>

namespace prx
{
    namespace packages
    {

        namespace manipulation
        {
            class manipulation_simulator_t;
            class movable_body_plant_t;
            class manipulator_t;
            class collidable_movable_bodies_list_t;

            class manipulation_context_info_t : public plan::planning_context_info_t
            {
                public:
                    std::string full_arm_context_name; //with the end effector
                    std::string arm_context_name; //without the end effector
                    std::string end_effector_context_name;

                    std::string end_effector_pathname;
                    std::string simple_end_effector_name;
                    int end_effector_index;
                    
                    const util::space_t* full_manipulator_state_space;
                    const util::space_t* full_manipulator_control_space;
                    const util::space_t* full_arm_state_space;
                    const util::space_t* full_arm_control_space;
                    const util::space_t* arm_state_space;
                    const util::space_t* arm_control_space;
                    const util::space_t* end_effector_state_space;
                    const util::space_t* end_effector_control_space;

                    sim::state_t* full_manipulator_state;
                    sim::state_t* full_target_manipulator_state;
                    sim::state_t* full_arm_state;
                    sim::state_t* arm_state;
                    sim::state_t* end_effector_state;
                    sim::control_t* full_manipulator_control;
                    sim::control_t* full_arm_control;
                    sim::control_t* arm_control;
                    sim::control_t* end_effector_control;

                    manipulator_t* manipulator;
                    std::pair<std::string, std::string> chain;

                    sim::plan_t manipulator_plan;

                    IK_seed_database_t* ik_database;
                    bool ik_left_arm;

                    //Adaptive grasps will collision check the geometries for determining the grasping control one step before collision
                    bool adaptive_grasp;
                    //Mode at which adaptive grasp checks will occur
                    int adaptive_grasping_mode;
                    //Steps at which the controls need to be changed to check for collisions for the adaptive grasp
                    double adaptive_grasping_step;

                public:
                    manipulation_context_info_t(plan::world_model_t* model, manipulator_t* manipulator, std::string full_arm_context_name, std::string arm_context_name, std::string end_effector_context_name, std::string base_link, std::string end_link, bool in_adaptive_flag, int in_adaptive_mode, double in_adaptive_step)
                    {
                        this->full_arm_context_name = full_arm_context_name;
                        this->arm_context_name = arm_context_name;
                        this->end_effector_context_name = end_effector_context_name;
                        chain = std::make_pair(base_link, end_link);

                        std::string old_context = model->get_current_context();

                        model->use_context(full_arm_context_name);
                        full_arm_state_space = model->get_state_space(); 
                        full_arm_control_space = model->get_control_space();

                        full_arm_state = full_arm_state_space->alloc_point();
                        full_arm_control = full_arm_control_space->alloc_point();

                        model->use_context(arm_context_name);
                        arm_state_space = model->get_state_space(); 
                        arm_control_space = model->get_control_space();

                        arm_state = arm_state_space->alloc_point();
                        arm_control = arm_control_space->alloc_point();

                        model->use_context(end_effector_context_name);
                        end_effector_state_space = model->get_state_space(); 
                        end_effector_control_space = model->get_control_space();
                        
                        end_effector_state = end_effector_state_space->alloc_point();
                        end_effector_control = end_effector_control_space->alloc_point();

                        model->use_context(old_context);

                        this->manipulator = manipulator;
                        full_manipulator_state_space = manipulator->get_state_space();
                        full_manipulator_control_space = manipulator->get_control_space();  
                        full_manipulator_state = full_manipulator_state_space->alloc_point();
                        full_target_manipulator_state = full_manipulator_state_space->alloc_point();
                        full_manipulator_control = full_manipulator_control_space->alloc_point();

                        end_effector_pathname = manipulator->get_pathname() + "/" + end_link;
                        end_effector_index = manipulator->get_end_effector_index(end_effector_pathname);

                        manipulator_plan.link_control_space(full_manipulator_control_space);

                        adaptive_grasp = in_adaptive_flag;
                        adaptive_grasping_mode = in_adaptive_mode;
                        adaptive_grasping_step = in_adaptive_step;
                    }

                    virtual ~manipulation_context_info_t()
                    {
                        full_arm_state_space->free_point(full_arm_state);
                        full_arm_control_space->free_point(full_arm_control);
                        arm_state_space->free_point(arm_state);
                        arm_control_space->free_point(arm_control);
                        end_effector_state_space->free_point(end_effector_state);
                        end_effector_control_space->free_point(end_effector_control);
                        full_manipulator_state_space->free_point(full_manipulator_state);
                        full_manipulator_state_space->free_point(full_target_manipulator_state);                        
                        full_manipulator_control_space->free_point(full_manipulator_control);

                    }

                    virtual void convert_plan(sim::plan_t& plan, const util::space_t* output_space)
                    {
                        sim::control_t* ctrl = output_space->alloc_point();
                        foreach(sim::plan_step_t step, manipulator_plan)
                        {
                            full_manipulator_control_space->copy_from_point(step.control);
                            output_space->copy_to_point(ctrl);
                            plan.copy_onto_back(ctrl,step.duration);
                        }
                        output_space->free_point(ctrl);
                    }

                    virtual std::string get_end_link_name()
                    {
                        return chain.second;
                    }

                    virtual bool is_end_effector_closed()
                    {
                        return manipulator->is_end_effector_closed(end_effector_index);
                    }
            };

            /**
             * @anchor manipulation_world_model_t
             *
             *
             * This class represents a planner's view of the world, and is responsible for
             * reporting state information. The world model works closely with planning
             * modules for proper operations in manipulation.
             *
             * @brief <b> Abstract world representation class for planners used in manipulation problems. </b>
             *
             * @author Zakary Littlefield, Athanasios Krontiris
             */
            class manipulation_world_model_t : public plan::world_model_t
            {

              public:
                manipulation_world_model_t();

                virtual ~manipulation_world_model_t(){ }

                /**
                 * Initialize the world_model from the given parameters. Also performs a massive initialization
                 * procedure that involved subdividing the world state space into an embedded space for planning.
                 * This embedded space can perform the task of hiding certain systems from the planner for dynamic
                 * obstacle planning, or for planning for multiple systems in a decentralized way. This can also
                 * be used to create a task space, a typically lower dimensional space that captures the task that
                 * is being achieved.
                 *
                 * These things are accomplished through the use of the embedded space class.
                 *
                 * @param reader A \ref util::parameter_reader_t with a dictionary of parameters for this system.
                 * @param template_reader A secondary \ref util::parameter_reader_t with a dictionary of default parameters.
                 */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);


                /**
                 * Copy the given state over the world current state.
                 *
                 * The state must be compatible with the system's state space.
                 * All the implementations of systems have to enforce the bounds
                 * of the state space on the state that they just receive \c source.
                 *
                 * @param source A point from the simulator's state space.
                 */
                virtual void push_state(const sim::state_t * const source);

                /**
                 * @brief Space context switch function.
                 * 
                 * @details This function switches the context of the planning to work over a new
                 * space. This function loads a new \ref space_descriptor_t for planning. Also, uses 
                 * the correct manipulator and the correct end effector. 
                 * 
                 * @param name The name of the space which we want to plan for.
                 */
                virtual void use_context(std::string name);

                /**
                 * @brief Returns the list with all the objects that you can manipulate. 
                 * 
                 * @details Returns the list with all the objects that you can manipulate. 
                 * These objects have an identifier type that the grasping module knows for computing the correct relative configurations for this object.
                 */
                virtual void get_objects(std::vector<movable_body_plant_t* >& objects);

                /**
                 * @brief  This function can set the relative configuration to be static between the manipulator and the grasped object.
                 * 
                 * @details This function can set the relative configuration to be static between the manipulator and the grasped object. 
                 * You have to make sure when you call this function that the object is actually grasped by the corresponding end-effector. 
                 * Otherwise, the object will be flying together as the arm moves without being grasped. 
                 * We don't have to specify the manipulator because this is already specified by the manipulation context.
                 * 
                 * @param flag True if we want to keep the relative configuration static (e.g. when we are moving the manipulator with an object grasped). 
                 * Be CAREFUL: You have to be sure that the manipulator is grasping the object, otherwise the relative configuration will be off.
                 */
                virtual void set_static_relative_config( bool flag);
                
                /**
                 * @brief this function uses the end-effector context in order to generate the control 
                 * which results in grasping.
                 * 
                 * @details this function uses the end-effector context in order to generate the control 
                 * which results in grasping. It appends to the input plan the control pulse 
                 * that performs the grasp.
                 * 
                 * @param plan The plan with the controls we need in order to achieve the grasping mode.
                 * @param grasping_mode The mode that we want the end effector to be (e.g. 1 for open end effector, 2 for closed end effector). 
                 * We could have multiple modes for grasping and open end effectors.
                 * @ static_relative_flag Say to simulator to maintain the relative configuration static and update the object based on 
                 * the position of the manipulator.
                 */
                virtual void engage_grasp( sim::plan_t& plan, double grasping_mode, bool static_relative_flag );

                /**
                 * @brief Returns 
                 * @details [long description]
                 * @return [description]
                 */
                virtual double get_current_grasping_mode();

                /**
                 * @brief Returns the information for the current active manipulation info.
                 * @details Returns the information for the current active manipulation info. The manipulation info
                 * holds information about the full arm, just the arm and the end effector context. Along with, the spaces for 
                 * each of these different cases.
                 * 
                 * @return The active manipulation info. 
                 */
                virtual manipulation_context_info_t* get_current_manipulation_info();                

                /**
                 * @brief Steers between two states.
                 * @details Steers between two states. It will return the plan for moving the active manipulator from the start 
                 * state to the goal state.
                 * 
                 * @param result_plan The plan to steer between the two states.
                 * @param start The initial state.
                 * @param goal The goal state.
                 */
                void steering_function(sim::plan_t& result_plan, const sim::state_t* start, const sim::state_t* goal);


                /**
                 * @brief Use a steering function to drive toward a desired state.
                 *
                 * This function attempts to solve the boundary value problem, where, given
                 * an initial and goal state, the world model attempts to drive the simulation
                 * from the start state to the goal state as exactly as possible.
                 *
                 * @param start The state from which to start propagating.
                 * @param goal The desired state of the simulation.
                 * @param result The resulting plan which drives the system toward goal.
                 */
                virtual void steering_function(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& result);
                
                /**
                 * @brief This function calls IK for the specific manipulation context.
                 * 
                 * @details This function calls IK for the specific manipulation context.
                 * 
                 * @param result_state The resulting state of the IK solution. 
                 * @param start_state The initial state of the manipulator. From this state we can retrieve the starting configuration of the link 
                 * that we are interesting in solving IK for and the mode of the manipulator if it is grasping or not.  
                 * @param goal_config The goal configuration that we need to achieve.
                 * @param validate Only return successful IK if the state is collision-free. 
                 * 
                 * @return True if the IK_solver found a solution, otherwise false.
                 */
                virtual bool IK( sim::state_t* result_state, const util::space_point_t* start_state, const util::config_t& goal_config, bool validate = false);

                
                /**
                 * @brief This function calls IK_steering for the manipulation context.
                 * @details This function calls IK_steering for the manipulation context.
                 * 
                 * @param result_plan The plan that will bring the manipulator from the start state to the goal state.
                 * @param result_state The resulting state of the IK solution. 
                 * @param start_state The initial state of the manipulator. From this state we can retrieve the starting configuration of the link 
                 * that we are interesting in solving IK for and the mode of the manipulator if it is grasping or not.  
                 * @param goal_config The goal configuration that we need to achieve.
                 * @return True if the IK_steering found a solution, otherwise false.
                 */
                virtual bool jac_steering( sim::plan_t& result_plan, workspace_trajectory_t& ee_trajectory, sim::state_t* result_state, const util::space_point_t* start_state, const util::config_t& goal_config);

                /**
                 * @brief This function calls IK_steering for the manipulation context.
                 * @details This function calls IK_steering for the manipulation context. At the end will also call 
                 * the manipulator's steering function in order to get the manipulator to the final desired state.
                 * 
                 * @param result_plan The plan that will bring the manipulator from the start state to the goal state.
                 * @param result_state The resulting state of the IK solution. 
                 * @param start_state The initial state of the manipulator. From this state we can retrieve the starting configuration of the link 
                 * that we are interesting in solving IK for and the mode of the manipulator if it is grasping or not.  
                 * @param final_state The desired final state for the manipulator. The IK_steering could return a different state from the one that 
                 * we desire given the goal configuration. 
                 * @param goal_config The goal configuration that we need to achieve.
                 * @return True if the IK_steering found a solution, otherwise false.
                 */
                virtual bool jac_steering( sim::plan_t& result_plan, workspace_trajectory_t& ee_trajectory, sim::state_t* result_state, const util::space_point_t* start_state, const util::space_point_t* final_state, const util::config_t& goal_config);
                
                /**
                 * @brief Get the Jacobian steering error code from the current active manipulator.
                 */
                unsigned get_jac_steer_error_type();

                /**
                 * @brief It will return the configuration of the last link along the active chain per the manipulation context.
                 * @details It will return the configuration of the last link along the active chain per the manipulation context.
                 * 
                 * @param link The result configuration for the last link along the chain. 
                 */
                virtual void FK(util::config_t& link);
                
                /**
                 * @brief Returns the first link and the end link for the chain that the specific manipulation context is working on. 
                 * 
                 * @details Returns the first link and the end link for the chain that the specific manipulation context is working on. 
                 * Hopefully, it will not be necessary to reason over chains above the world model. 
                 * But it may end up being necessary for certain projects.
                 * 
                 * @return Returns the first link and the end link for the chain that the specific manipulation context is working on. 
                 */
                virtual std::pair<std::string, std::string> get_chain();
                
                /**
                 * @brief Returns the end effector context that only the specific end effector is active.
                 * 
                 * @details Returns the end effector context that only the specific end effector is active. 
                 * Hopefully, it will not be necessary to reason over end effector contexts above the world model. 
                 * But it may end up being necessary for certain projects.
                 * 
                 * @return Returns the end effector context that only the specific end effector is active.
                 */
                virtual std::string get_end_effector_context();

                
                /**
                 * @brief You can get the manipulators but you most probably won't need that in most cases.
                 * @details You can get the manipulators but you most probably won't need that in most cases.
                 * 
                 * @param manipulators Appends to this vector all the manipulators that are in the world.
                 */
                virtual void get_manipulators(std::vector<manipulator_t* >& manipulators);

                /**
                 * @brief Returns the configuration of the active end effector.
                 * 
                 * @details Returns the configuration of the active end effector. Each end effector has a local configuration. 
                 * This function will return the local configuration for the active end effector of the active manipulator. 
                 * 
                 * @param config The local configuraton of the end effector.
                 */
                virtual void get_end_effector_local_config(util::config_t& config);


                /**
                 * @brief This function initializes the simulator's collision list.
                 *
                 * @param reader A \ref util::parameter_reader_t from which to read the collision list.
                 */
                virtual void init_collision_list(const util::parameter_reader_t * const reader);

                /**
                 * @brief Fills a collision list with all possible collision pairs.
                 *
                 * @param plants A map containing the plants in the simulation.
                 * @param obstacles A map containing the obstacles in the simulation.
                 * @param black_list The collision list to populate.
                 */
                virtual void collision_list_all_vs_all(util::hash_t<std::string, sim::plant_t*>& plants, util::hash_t<std::string, sim::system_ptr_t>& obstacles, const sim::vector_collision_list_t* black_list);

                virtual void update_target_objects(const std::vector<std::string>& target_object_list);
            protected:

                std::vector<std::pair<std::string, IK_seed_database_t*> > ik_databases;
                int ik_seeds;

                std::string IK_steer_context;

                manipulation_simulator_t* manip_simulator;

                manipulation_context_info_t* active_manipulation_info;

                collidable_movable_bodies_list_t* collidable_list;

                virtual manipulator_t* find_active_manipulator(const std::vector<manipulator_t*>& manipulators);
                std::vector<movable_body_plant_t* > collidable_objects;

            };
        }
    }
}

#endif
