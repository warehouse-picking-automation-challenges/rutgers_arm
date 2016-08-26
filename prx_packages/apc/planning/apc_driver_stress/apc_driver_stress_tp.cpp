/**
 * @file simple_pap_tp.cpp
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

#include "apc_driver_stress_tp.hpp"
#include "apc_driver_stress_specification.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include <boost/range/adaptor/map.hpp>
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::apc::apc_driver_stress_tp_t ,prx::plan::planner_t)

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

            apc_driver_stress_tp_t::apc_driver_stress_tp_t()
            {

            }

            apc_driver_stress_tp_t::~apc_driver_stress_tp_t()
            {

            }

            void apc_driver_stress_tp_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                PRX_INFO_S("Initializing apc_task_planner_t task planner ...");
                task_planner_t::init(reader,template_reader);
                left_context_name = parameters::get_attribute("left_context_name", reader, template_reader);
                right_context_name = parameters::get_attribute("right_context_name", reader, template_reader);
                manipulation_task_planner_name = parameters::get_attribute("manipulation_task_planner_name", reader, template_reader);

                manip_planner = dynamic_cast<manipulation_tp_t*>(planners[manipulation_task_planner_name]);

                examination_profile_t* profile = new examination_profile_t();
                profile->focus.push_back(.8);
                profile->focus.push_back(0);
                profile->focus.push_back(1.45);
                profile->base_viewpoint.set(0,1,0,1);
                profile->base_viewpoint.normalize();
                profile->offsets.push_back(profile->base_viewpoint);
                profile->offsets.push_back(profile->base_viewpoint);
                profile->distance = .3;
                camera_positions['B'] = profile;
                profile = new examination_profile_t();
                profile->focus.push_back(.8);
                profile->focus.push_back(0);
                profile->focus.push_back(1.15);
                profile->base_viewpoint.set(0,1,0,1);
                profile->base_viewpoint.normalize();
                profile->offsets.push_back(profile->base_viewpoint);
                profile->offsets.push_back(profile->base_viewpoint);
                profile->distance = .3;
                camera_positions['E'] = profile;

                left_arm_order_bin  = {-1.57,1.661659,0.677508,0,-1.120185,0,-0.165669,0,0};
                right_arm_order_bin = { 1.57,1.661659,0.677508,0,-1.120185,0,-0.165669,0};
                PRX_WARN_S("Finished Init...");
            }

            void apc_driver_stress_tp_t::link_world_model(world_model_t * const model)
            {
                PRX_WARN_S("Linking world model...");
                task_planner_t::link_world_model(model);
                manipulation_model = dynamic_cast<manipulation_world_model_t*>(model);
                if(manipulation_model == NULL)
                    PRX_FATAL_S("The apc_driver_stress task planner can work only with manipulation world model!");
                PRX_WARN_S("Finished linking world model...");
            }

            void apc_driver_stress_tp_t::setup()
            {
                foreach(planner_t* planner, planners | boost::adaptors::map_values)
                {
                    PRX_INFO_S("setup planner: " << planner->get_name());
                    planner->setup();
                }
            }

            bool apc_driver_stress_tp_t::execute()
            {
                foreach(planner_t* planner, planners | boost::adaptors::map_values)
                {                    
                    PRX_INFO_S("execute planner: " << planner->get_name());
                    planner->execute();
                }
                left_manipulation_query = new manipulation_query_t();
                right_manipulation_query = new manipulation_query_t();

                manipulation_model->use_context(left_context_name);
                manipulator = manipulation_model->get_current_manipulation_info()->manipulator;
                full_manipulator_state_space = manipulation_model->get_current_manipulation_info()->full_manipulator_state_space;
                full_manipulator_control_space = manipulation_model->get_current_manipulation_info()->full_manipulator_control_space;

                return true;
            }

            bool apc_driver_stress_tp_t::succeeded() const
            {
                return true;
            }

            const util::statistics_t* apc_driver_stress_tp_t::get_statistics()
            {
                return NULL;
            }
            
            void apc_driver_stress_tp_t::link_query(query_t* new_query)
            {
                task_planner_t::link_query(new_query);
                in_query = static_cast<apc_task_query_t*>(new_query);
                PRX_WARN_S("Finished linking query...");
            }
            
            void apc_driver_stress_tp_t::resolve_query()
            {
                in_query->found_solution = move();
            }

            bool apc_driver_stress_tp_t::move(){
                //taking a full manipulator state, move both arms in sequence to that position
                state_t* full_init = full_manipulator_state_space->alloc_point();
                manipulation_model->use_context(left_context_name);
                space_t* left_space =  manipulation_model->get_state_space();
                manipulation_model->use_context(right_context_name);
                space_t* right_space =  manipulation_model->get_state_space();

                state_t* initial_state_left = left_space->alloc_point();
                state_t* left_goal = left_space->alloc_point();
                state_t* initial_state_right = right_space->alloc_point();
                state_t* right_goal = right_space->alloc_point();
                // PRX_INFO_S("left goal: "<<left_space->print_point(left_goal,5));
                ((apc_driver_stress_specification_t*)input_specification)->sampler->sample(full_manipulator_state_space,in_query->goal_state);
                // PRX_INFO_S("left goal: "<<left_space->print_point(left_goal,5));
                //PRX_DEBUG_S("right goal: "<<left_space->print_point(left_goal,5));

                manipulation_model->convert_spaces(left_space,left_goal,full_manipulator_state_space,in_query->goal_state);
                //in order to make sure the right movement takes into account any movement on the left, this update is needed
                full_manipulator_state_space->copy_from_point(full_init);
                manipulation_model->convert_spaces(right_space,initial_state_right,left_space,left_goal);
                manipulation_model->convert_spaces(right_space,right_goal,full_manipulator_state_space,in_query->goal_state);
                full_manipulator_state_space->free_point(full_init);
                //left hand
                manipulation_model->use_context(left_context_name);
                left_manipulation_query->setup_move(manipulation_model->get_current_context(),initial_state_left,left_goal);
                left_manipulation_query->plan.clear();
                manip_planner->link_query(left_manipulation_query);
                manip_planner->resolve_query();
                in_query->move_plan.clear();
                if(left_manipulation_query->found_path)
                {
                    manipulation_model->convert_plan(in_query->move_plan, manipulator->get_control_space(), left_manipulation_query->plan, manipulation_model->get_control_space());
                    manipulation_model->get_state_space()->free_point(initial_state_left);
                    manipulation_model->get_state_space()->free_point(left_goal);
                    plan_t plan(manipulation_model->get_control_space());
                    manipulation_model->engage_grasp(plan,manipulation_model->get_current_manipulation_info()->end_effector_state_space->at(0),manipulation_model->get_current_manipulation_info()->is_end_effector_closed());
                }
                else
                {
                    manipulation_model->get_state_space()->free_point(initial_state_left);
                    manipulation_model->get_state_space()->free_point(left_goal);
                    manipulation_model->get_state_space()->free_point(initial_state_right);
                    manipulation_model->get_state_space()->free_point(right_goal);
                    return false;

                }


                //right hand
                manipulation_model->use_context(right_context_name);
                right_manipulation_query->setup_move(manipulation_model->get_current_context(),initial_state_right,right_goal);
                right_manipulation_query->plan.clear();
                manip_planner->link_query(right_manipulation_query);
                manip_planner->resolve_query();
                if(right_manipulation_query->found_path)
                {
                    manipulation_model->convert_plan(in_query->move_plan, manipulator->get_control_space(), right_manipulation_query->plan, manipulation_model->get_control_space());
                    plan_t plan(manipulation_model->get_control_space());
                    manipulation_model->engage_grasp(plan,manipulation_model->get_current_manipulation_info()->end_effector_state_space->at(0),manipulation_model->get_current_manipulation_info()->is_end_effector_closed());

                    manipulation_model->get_state_space()->free_point(initial_state_right);
                    manipulation_model->get_state_space()->free_point(right_goal);
                    return true;
                }
                else
                {
                    manipulation_model->get_state_space()->free_point(initial_state_right);
                    manipulation_model->get_state_space()->free_point(right_goal);
                    return false;
                }

                PRX_WARN_S("Finished resolving query...");
            }
        }
    }
}

