/**
 * @file multi_pap_tp.cpp
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

#include "planning/task_planners/multi_pap_tp.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/statistics/statistics.hpp"

#include <boost/range/adaptor/map.hpp>
#include <pluginlib/class_list_macros.h>

#include <iostream>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::multi_pap_tp_t ,prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace manipulation
        {

            multi_pap_tp_t::multi_pap_tp_t()
            {

            }

            multi_pap_tp_t::~multi_pap_tp_t()
            {

            }

            void multi_pap_tp_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                PRX_INFO_S("Initializing multi_pap_tp_t task planner ...");
                task_planner_t::init(reader,template_reader);
                full_manipulator_context_name = parameters::get_attribute("full_manipulator_context_name", reader, template_reader);
                left_context_name = parameters::get_attribute("left_context_name", reader, template_reader);
                right_context_name = parameters::get_attribute("right_context_name", reader, template_reader);
                left_target_vec = parameters::get_attribute_as<std::vector<double> >("left_target", reader, template_reader);
                right_target_vec = parameters::get_attribute_as<std::vector<double> >("right_target", reader, template_reader);
                manipulation_task_planner_name = parameters::get_attribute("manipulation_task_planner_name", reader, template_reader);
            }

            void multi_pap_tp_t::link_world_model(world_model_t * const model)
            {
                task_planner_t::link_world_model(model);
                manipulation_model = dynamic_cast<manipulation_world_model_t*>(model);
                if(manipulation_model == NULL)
                    PRX_FATAL_S("The manipulation task planner can work only with manipulation world model!");
            }

            void multi_pap_tp_t::link_query(query_t* new_query)
            {
                task_planner_t::link_query(new_query);
                in_query = static_cast<motion_planning_query_t*>(new_query);
            }

            void multi_pap_tp_t::setup()
            {
                manipulation_model->use_context(full_manipulator_context_name);
                manip_initial_state = manipulation_model->get_state_space()->alloc_point();
                full_state_space = manipulation_model->get_state_space();
                full_control_space = manipulation_model->get_control_space();
                
                manipulation_model->use_context(left_context_name);
                left_state_space = manipulation_model->get_current_manipulation_info()->full_arm_state_space;
                left_control_space = manipulation_model->get_current_manipulation_info()->full_arm_control_space;
                
                manipulation_model->use_context(right_context_name);
                right_state_space = manipulation_model->get_current_manipulation_info()->full_arm_state_space;
                right_control_space = manipulation_model->get_current_manipulation_info()->full_arm_control_space;
                
                manipulation_model->use_context(full_manipulator_context_name);

                foreach(planner_t* planner, planners | boost::adaptors::map_values)
                {
                    PRX_INFO_S("setup planner: " << planner->get_name());
                    planner->setup();
                }
            }

            bool multi_pap_tp_t::execute()
            {
                foreach(planner_t* planner, planners | boost::adaptors::map_values)
                {                    
                    PRX_INFO_S("execute planner: " << planner->get_name());
                    planner->execute();
                }
                return true;
            }

            bool multi_pap_tp_t::succeeded() const
            {
                return true;
            }

            const util::statistics_t* multi_pap_tp_t::get_statistics()
            {
                return NULL;
            }

            void multi_pap_tp_t::resolve_query()
            {
                config_t retract_config;
                retract_config.set_position(0,0,-.05);
                retract_config.set_orientation(0,0,0,1);
                PRX_DEBUG_COLOR("Resolve query from multi_pap_tp ...  left context:" << left_context_name,PRX_TEXT_RED);
                manipulation_model->use_context(full_manipulator_context_name);
                manipulation_model->get_state_space()->copy_from_point(manip_initial_state);  

                /** SOLVE FOR LEFT HAND FIRST */
                manipulation_model->use_context(left_context_name);
                std::vector<movable_body_plant_t* > objects;
                manipulation_model->get_objects(objects);
                const space_t* object_space = objects[1]->get_state_space();
                PRX_DEBUG_COLOR("LEFT ARM GRABS OBJECT:" << objects[1]->get_pathname(),PRX_TEXT_RED);

                manipulator = manipulation_model->get_current_manipulation_info()->manipulator;                
                state_t* initial_state = manipulation_model->get_state_space()->alloc_point();

                state_t* initial_object = object_space->alloc_point();
                state_t* target_object = object_space->alloc_point();
                object_space->copy_vector_to_point(left_target_vec, target_object);

                manipulation_query;// = new manipulation_query_t(left_context_name, TASK_PICK_AND_PLACE, GRASP_GREEDY, objects[1], 1, retract_config, initial_state, initial_state, NULL, initial_object, target_object );
                planners[manipulation_task_planner_name]->link_query(manipulation_query);
                //std::cin >> trash;
                planners[manipulation_task_planner_name]->resolve_query();
                
                object_space->free_point(initial_object);
                object_space->free_point(target_object);
                manipulation_model->get_state_space()->free_point(initial_state);
                
                /** SOLVE FOR RIGHT HAND NEXT */
                PRX_DEBUG_COLOR("Resolve query from multi_pap_tp ...  right context:" << right_context_name,PRX_TEXT_BLUE);
                manipulation_model->use_context(right_context_name);
                object_space = objects[0]->get_state_space();
                PRX_DEBUG_COLOR("RIGHT ARM GRABS OBJECT:" << objects[0]->get_pathname(),PRX_TEXT_BLUE);

                manipulator = manipulation_model->get_current_manipulation_info()->manipulator;                
                state_t* initial_state2 = manipulation_model->get_state_space()->alloc_point();

                state_t* initial_object2 = object_space->alloc_point();
                state_t* target_object2 = object_space->alloc_point();
                object_space->copy_vector_to_point(right_target_vec, target_object2);

                manipulation_query_t* query2;// = new manipulation_query_t(right_context_name, TASK_PICK_AND_PLACE, GRASP_GREEDY, objects[0], 1, retract_config, initial_state2, initial_state2, NULL, initial_object2, target_object2 );
                planners[manipulation_task_planner_name]->link_query(query2);
                //std::cin >> trash;
                planners[manipulation_task_planner_name]->resolve_query();
                
                //manipulation_model->use_context(full_manipulator_context_name);
                in_query->link_spaces(full_state_space, full_control_space);
                manipulation_model->convert_plan(in_query->plan, full_control_space, manipulation_query->plan, left_control_space);
                manipulation_model->convert_plan(in_query->plan, full_control_space, query2->plan, right_control_space);

                manipulation_model->use_context(full_manipulator_context_name);
                
                object_space->free_point(initial_object2);
                object_space->free_point(target_object2);
                manipulation_model->get_state_space()->free_point(initial_state2);
            }
        }
    }
}

