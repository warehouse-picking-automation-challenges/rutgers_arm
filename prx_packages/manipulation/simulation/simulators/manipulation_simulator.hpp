/**
 * @file manipulator_simulator.hpp
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

#pragma once

#ifndef PRACSYS_MANIPULATION_SIMULATOR_HPP
#define PRACSYS_MANIPULATION_SIMULATOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/definitions/string_manip.hpp" 
#include "prx/utilities/boost/hash.hpp"

#include "prx/simulation/simulators/default_simulator.hpp"
#include "prx/simulation/collision_checking/vector_collision_list.hpp"
#include "prx/simulation/collision_checking/collision_list.hpp"

#include "simulation/systems/plants/manipulator.hpp"
#include "simulation/systems/plants/movable_body_plant.hpp"

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {

            class end_effector_info_t
            {

              public:
                /** @brief The manipulator that the end effector belongs */
                manipulator_t* manipulator;
                /** @brief The name of the end effector */
                std::string end_effector_name;
                /** @brief The name of the link of the end effector, without the path */
                std::string simple_end_effector_name;
                /** @brief The index of the end effector inside the manipulator. */
                unsigned end_effector_index;
                /** The relative configuration to the object that the manipulator is grasping */
                util::config_t relative_config;
                /**
                 * @brief A pointer to the movable object that the end effector is grasping. 
                 * If the end effector is empty then the pointer is NULL.
                 */
                movable_body_plant_t* object;
                /** @brief The memory variable for the simulation state for the object that the end effector is grasping */
                double* grasped_object_id;
                /** @brief The collision list of the end effector with all the movable objects */
                sim::vector_collision_list_t* grasping_check_list;
                /** @brief The list of geoms that have to be ignored while the end effector is grasping something */
                std::vector<std::string>  ignore_during_grasping;
                /** @brief The previous grasping mode if the end effector is closed (true) or open (false) */
                bool prev_grasping_mode;
                /** @brief Keeps the relative configuration static while push state is being called. */
                bool static_relative_configuration;

                end_effector_info_t(manipulator_t* manipulator, unsigned end_effector_index, std::string end_effector_name)
                {
                    this->manipulator = manipulator;
                    this->end_effector_index = end_effector_index;
                    this->end_effector_name = end_effector_name;
                    simple_end_effector_name = util::reverse_split_path(end_effector_name).second;
                    grasped_object_id = new double;
                    *grasped_object_id = -1;
                    object = NULL;
                    ignore_during_grasping = manipulator->get_ignored_grasping_bodies(end_effector_index);
                    grasping_check_list = new sim::vector_collision_list_t();
                    prev_grasping_mode = false;
                    static_relative_configuration = false;
                }

                ~end_effector_info_t()
                {
                    delete grasped_object_id;
                    delete grasping_check_list;
                }

                /**
                 * @brief Checks if the end effector is in this manipulator.
                 * @details Checks if the end effector is in this manipulator. First checks if the manipulator
                 * is the correct one and then checks if the simple name of the end effector is the same with 
                 * this specific end_effector_info_t. 
                 * 
                 * @param manipulator The manipulator.
                 * @param end_effector_name The simple name of the end effector that we are looking for.
                 * It is the simple name for the end effector without the full path from the manipulator.
                 * 
                 * @return [description]
                 */
                bool is_ee_mine(manipulator_t* manipulator, std::string end_effector_name)
                {
                    if(this->manipulator == manipulator)
                        return util::reverse_string_compare(simple_end_effector_name, end_effector_name);
                    return false;
                }

                std::string print()
                {
                    std::stringstream out(std::stringstream::out);

                    out << end_effector_name;
                    if(object != NULL)
                        out << " relative config: " << relative_config;
                    else
                        out << " does not grasp anything.";
                    
                    return out.str();
                }


            };

            /**
             * This is a variation of the simulator for manipulation problems.
             *
             * @brief <b> This is a variation of the simulator for manipulation problems. </b>
             *
             * @author Athanasios Krontiris, Zakary Littlefield
             *
             */
            class manipulation_simulator_t : public sim::default_simulator_t
            {

              public:

                manipulation_simulator_t();
                ~manipulation_simulator_t();

                /** @copydoc simulator_t::init( const util::parameter_reader_t* const) */
                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

                /**
                 * @copydoc simulator_t::push_state(constsim::state_t* ) 
                 *
                 * Push state resets the world. The state has to be properly constructed. If the manipulator is grasping something 
                 * then the state variable for the end effector must have a value >= 0. This function will NOT check for new grasps. 
                 * Propagate is going to setup any new grasps. If the object state variable is -1 then the end effector will release 
                 * the object. 
                 * 
                 * WARNING: If the object state variable is >= 0 then the object must be in the correct place, e.g. inside the end effector 
                 *          because the compute relative configuration will be called.  
                 * 
                 */
                virtual void push_state(const sim::state_t * const state);

                virtual void link_collision_list(sim::collision_list_t* collision_list);

                /** 
                 * @copydoc simulator_t::in_collision( ) 
                 *
                 * First will update the ground truth and then will check if there is at least one collision. 
                 */
                virtual bool in_collision();

                /**
                 * @copydoc simulator_t::get_colliding_bodies( ) 
                 * 
                 * First will move the grasped objects and then will call the \c get_colliding_bodies()
                 */
                virtual sim::collision_list_t* get_colliding_bodies();

                /** @copydoc simulator_t::propagate(const double ) */
                virtual void propagate(const double simulation_step);

                /** @copydoc simulator_t::propagate_and_respond() */
                virtual void propagate_and_respond();

                /**
                 * @brief Sets the relative configuration between an end effector and an object static.                  
                 * 
                 * @details Sets the relative configuration between an end effector and an object static. 
                 * This way the push state of the simulator will not change the relative configuration between
                 * this end effector and the object that is grasping.
                 * 
                 * @param name The name of the end effector with the full path.
                 * @param flag True if we want to maintain the relative configuration, otherwise false. 
                 */
                virtual void set_static_relative_config(std::string name, bool flag);

                /**
                 * @brief Returns the list with all the objects that you can manipulate. 
                 * 
                 * @details Returns the list with all the objects that you can manipulate. 
                 * These objects have an identifier type that the grasping module knows for computing the correct relative configurations for this object.
                 */
                virtual void get_movable_objects(std::vector<movable_body_plant_t* >& objects);

                /**
                 * @brief You can get the manipulators but you most probably won't need that in most cases.
                 * @details You can get the manipulators but you most probably won't need that in most cases.
                 * 
                 * @param manipulators Appends to this vector all the manipulators that are in the world.
                 */
                virtual void get_manipulators(std::vector<manipulator_t* >& manipulators);

                /**
                 * @brief Checks if the end effector of a manipulator is grasping an object or not. 
                 * @details Checks if the end effector of a manipulator is grasping an object or not. 
                 * It will check that through the extra state variable that the simulator has for the 
                 * object id for each end effector. If the variable is -1 then the manipulator is not 
                 * grasping an object.
                 * 
                 * @param manipulator The manipulator that should have this end effector.
                 * @param simple_end_effector_name The simple name of the end effector that we are looking for.
                 * It is the simple name for the end effector without the full path from the manipulator.
                 * 
                 * @return True if the end effector is currently grasping something, otherwise false.
                 */
                virtual bool is_end_effector_grasping(manipulator_t* manipulator, std::string simple_end_effector_name);

                /**
                 * @brief Returns the configuration of the active end effector.
                 * 
                 * @details Returns the configuration of the active end effector. Each end effector has a local configuration. 
                 * This function will return the local configuration for the active end effector of the active manipulator. 
                 * 
                 * @param config The local configuraton of the end effector.
                 */
                virtual void get_end_effector_configuration(util::config_t& config, manipulator_t* manipulator, std::string simple_end_effector_name);

                virtual bool internal_state_push()
                {
                    return true;
                }

              protected:

                void move_grasped_objects();

                void compute_relative_config(end_effector_info_t* info);

                void release_object(end_effector_info_t* info);

                bool detect_new_grasped_object(end_effector_info_t* info);

                void print_relative_config();



                sim::vector_collision_list_t* updated_collision_list;

                std::vector<manipulator_t*> manipulators;
                std::vector<end_effector_info_t*> end_effectors;
                std::vector<movable_body_plant_t*> movable_bodies;

                util::config_t effector_config;
            };
        }
    }
}

#endif
