/**
 * @file rearrangement_primitive.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#pragma once

#ifndef PRX_REARRANGEMENT_MANIPULATION_TP_HPP
#define PRX_REARRANGEMENT_MANIPULATION_TP_HPP


#include "planning/modules/path_part.hpp"
#include "planning/queries/rearrangement_query.hpp"
#include "planning/problem_specifications/rearrangement_primitive_specification.hpp"
#include "planning/statistics/rearrangement_primitive_statistics.hpp"

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"

#include "prx/planning/task_planners/task_planner.hpp"

//From different packages
#include "../../../baxter/simulation/plants/manipulator.hpp"
#include "../../../manipulation/simulation/plants/movable_body_plant.hpp"
#include "../../../manipulation/planning/modules/pose.hpp"
#include "../../../rearrangement_manipulation/planning/queries/manipulator_query.hpp"
#include "../../../rearrangement_manipulation/planning/task_planners/manipulator_tp.hpp"


#include <list>
#include <sstream>

namespace prx
{

    namespace util
    {
        class bounds_t;
        class multiple_goal_states_t;
        class statistics_t;
    }

    namespace packages
    {
        namespace manipulation
        {
            class manip_sampler_t;
        }

        namespace rearrangement_manipulation
        {
            class manipulator_specification_t;
            class manipulation_mp_specification_t;
            class obstacle_aware_astar_t;
            class system_name_validity_checker_t;
        }

        namespace labeled_rearrangement_manipulation
        {
            using namespace baxter;
            using namespace manipulation;

            /**
             * Manipulation task planner. Computes the path for moving an object from an 
             * initial to a target position.             
             * 
             * @autors Athanasios Krontiris
             */
            class rearrangement_primitive_t : public plan::task_planner_t
            {

              public:

                rearrangement_primitive_t();
                virtual ~rearrangement_primitive_t();

                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                /**
                 * @copydoc task_planner_t::get_statistics()
                 */
                virtual const util::statistics_t* get_statistics();

                /** @copydoc task_planner_t::link_specification(specification_t*) */
                virtual void link_specification(plan::specification_t* new_spec);

                /** @copydoc task_planner_t::get_statistics()*/
                virtual void link_query(plan::query_t* new_query);

                /** @copydoc motion_planner_t::resolve_query() */
                virtual void resolve_query();

                /** @copydoc task_planner_t::setup() */
                virtual void setup();

                /** @copydoc task_planner_t::execute() */
                virtual bool execute();

                /** @copydoc task_planner_t::succeeded() const */
                virtual bool succeeded() const;

              protected:
                
                /**
                 * This function will be used from the resolved query in order to solve the problem. 
                 * 
                 * @return True if there is solution. False otherwise. 
                 */
                virtual bool solve() = 0;

                /**
                 * Detects the manipulator and one cup in the full state space.
                 * 
                 * @brief Detects the manipulator and one cup in the full state space.
                 */
                virtual bool detect_plants();

                //===================================================//
                // Communication with rearrangement search algorithm //
                //===================================================//
                rearrangement_query_t* in_query;
                rearrangement_primitive_specification_t* in_specs;
                unsigned k_objects;

                //===============//
                //   For Poses   //
                //===============//
                std::vector<pose_t> poses_set;
                unsigned poses_set_length;

                /** @brief All the poses that will inform the graph. Id for the pose and the state*/
                std::vector< std::pair<unsigned, sim::state_t*> > seed_poses;
                /** @brief The new poses that correspond to the initial and final positions.*/
                std::vector< std::pair<unsigned, sim::state_t*> > query_poses;


                //========================================//
                //   Communication with Manipulation tp   //
                //========================================//
                std::string manipulation_tp_name;
                rearrangement_manipulation::manipulator_tp_t* manip_tp;
                rearrangement_manipulation::manipulator_specification_t* manip_specs;

                rearrangement_manipulation::manipulator_query_t* transfer_query;
                rearrangement_manipulation::manipulator_query_t* transit_query;

                util::multiple_goal_states_t* transit_query_goal;
                util::multiple_goal_states_t* transfer_query_goal;

                /** @brief An A* module that computes minimum conflict paths using tree search A* */
                rearrangement_manipulation::obstacle_aware_astar_t* transit_astar;
                rearrangement_manipulation::obstacle_aware_astar_t* transfer_astar;

                //The constraints for the A*
                std::set<unsigned> transit_constraints;
                std::set<unsigned> transfer_constraints;
                //The constraints for the minimum conflict A*
                std::set<unsigned> transit_obstacles;
                std::set<unsigned> transfer_obstacles;
                std::set<unsigned> transit_avoid;
                std::set<unsigned> transfer_avoid;

                //Because we need to keep a pointer for transit/transfer A* we are going to build the specification for the motion
                //planners at this level and we will pass it down to the manipulation task planner. 
                rearrangement_manipulation::manipulation_mp_specification_t* transit_specification;
                rearrangement_manipulation::manipulation_mp_specification_t* transfer_specification;

                //For implementing the multiple start points for the A*
                std::vector<util::space_point_t*> transit_extra_start_states;
                std::vector<util::space_point_t*> transfer_extra_start_states;

                //======================//
                // For De/Serialization //
                //======================//
                std::string prx_output_dir;
                std::string prx_input_dir;

                //=======================//
                //   Helping Variables   //
                //=======================//
                manipulator_plant_t* _manipulator;
                movable_body_plant_t* _object;

                /** @brief Reads from input the name of the planning context that is for the manipulator only.*/
                std::string pc_name_manipulator_only;
                /** @brief Reads from input the name of the planning context that is for the real world objects.*/
                std::string pc_name_real_world;
                /** @brief Reads from input the name of the planning context that is for the object.*/
                std::string pc_name_object_only;
                /** @brief Reads from input the name of the planning context that is for the manipulator and the object.*/
                std::string pc_name_manipulator_with_object;
                /** @brief Reads from input the name of the planning context that is for the manipulator and an active for collision object.*/
                std::string pc_name_manipulator_with_active_object;
                /** @brief The planning context that is for the transit state (The manipulator and active object).*/
                std::string pc_name_transit_inform;
                /** @brief The planning context that is for the transfer state (The manipulator with plannable object and one more active object.*/
                std::string pc_name_transfer_inform;
                /** @brief Reads from input the name of the planning context that is for the real world.*/
                std::string pc_name_all_objects;
                /** @brief Reads from input the name of the planning context that is for the manipulator and the 
                 * object while all the other objects are active.*/
                std::string pc_name_grasp_planning;
                /** @brief Reads from input the name of the planning context that is for collision checks between poses in MPG*/
                std::string pc_name_collision_check;

                /** @brief The state space over the manipulator that the manipulation task planner is using. */
                const util::space_t* manip_state_space;
                /** @brief The control space over the manipulator that the manipulation task planner is using. */
                const util::space_t* manip_control_space;
                /** @brief The state space over the objects that the task planner is using*/
                const util::space_t* object_state_space;
                /** @brief The state space over the manipulator and the object that the task planner is using*/
                const util::space_t* mo_space;
                /** @brief The state space for the real objects*/
                const util::space_t* all_object_space;
                /** @brief The state space for all the other cup except the one that we control*/
                const util::space_t* other_objects_space;

                //Helping variables for state points.
                sim::state_t* manip_state;
                sim::state_t* object_state;
                /** @brief The safe state for the manipulator.*/
                sim::state_t* safe_state;
                /** @brief Real object initial state point.*/
                sim::state_t* real_initial_state;

                //Helping variables for control points
                sim::control_t* manip_ctrl;
                /** @brief The control that will bring the manipulator to the safe state. Because the manipulator is a rigid body the safe_control = safe_state.*/
                sim::control_t* safe_control;

                //Helping vector variables.
                std::vector<double> manip_control_vec;


                //=======//
                // Tools //
                //=======//

                /** @brief Random manipulation sampling module. Samples both manipulator's state and object's state */
                manipulation::manip_sampler_t* manip_sampler;

                /** 
                 * @brief A validity checker that is aware of the objects and can detect important collisions. 
                 * Will be used by the Obstacle aware A*
                 */
                rearrangement_manipulation::system_name_validity_checker_t* system_name_validity_checker;

                //================//
                // For Statistics //
                //================//
                util::sys_clock_t statistics_clock;
                rearrangement_primitive_statistics_t* stats;

                bool time_ends;


                virtual std::string print(const std::vector<unsigned>& arrangement);
                virtual std::string print(const std::vector<double>& arrangement);
                virtual std::string print(const std::deque<unsigned>& arrangement);
                virtual std::string print(const std::set<unsigned>& constraints);
                virtual std::string print(const std::vector<pose_t*>& free_poses);
            };
        }
    }
}


#endif	
