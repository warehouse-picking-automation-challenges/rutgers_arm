/**
 * @file sliding_rigid_body.cpp 
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


#include "prx/simulation/systems/plants/sliding_rigid_body.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/simulation/plan.hpp"

#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::sliding_rigid_body_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    namespace sim
    {

        const unsigned sliding_rigid_body_t::STATE_X = 0;
        const unsigned sliding_rigid_body_t::STATE_Y = 1;

        sliding_rigid_body_t::sliding_rigid_body_t() : plant_t()
        {
            state_memory = {&_x,&_y};
            control_memory = {&_Cx,&_Cy};

            state_space = new space_t("XY", state_memory);
            input_control_space = new space_t("XY", control_memory);

            reset = false;
            _z = 0;
        }

        void sliding_rigid_body_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            //    PRX_WARN_S("Init for rigid body ");
            plant_t::init(reader, template_reader);

            max_step = parameters::get_attribute_as<double>("max_step", reader, template_reader);
            _z = parameters::get_attribute_as<double>("z", reader, template_reader, 1.5);
            //    set_interpolation_step

            initial_state = state_space->alloc_point();
            state = state_space->alloc_point();
            prior_state = state_space->alloc_point();
            hold_state = state_space->alloc_point();

            control = input_control_space->alloc_point();
            prior_control = input_control_space->alloc_point();

            dist = 2;
            interpolation_step = 0;
        }

        void sliding_rigid_body_t::propagate(const double simulation_step)
        {
            state_space->copy_to_point(state);
            input_control_space->copy_to_point(control);

            if( state_space->equal_points(state, hold_state, PRX_ZERO_CHECK) )
                interpolation_step = 0;
            else
                reset = true;

            if( (!input_control_space->equal_points(control, prior_control, PRX_ZERO_CHECK)) || reset )
            {
                input_control_space->copy_point(prior_control, control);

                state_space->copy_point(initial_state, state);

                // Comment in this chunk of code for equidistant intervals in the interpolation
                //        double total_distance = state_space->distance(initial_state, control);
                //        double interval = std::ceil(total_distance / max_step);
                //        double actual_max_step = total_distance / interval;
                //        interpolation_step = actual_max_step / state_space->distance(initial_state, control);

                interpolation_step = max_step / state_space->distance(initial_state, control);
                //        PRX_DEBUG_S (max_step);
                dist = 0;
                reset = false;
            }

            dist += interpolation_step;

            if( dist <= 1 )
            {
                state_space->copy_to_point(hold_state);
                state_space->interpolate(initial_state, control, dist, state);
                //        PRX_WARN_S ("Dist: " << dist << ", and Distance between states: " << state_space->distance(initial_state, state));
                state_space->copy_from_point(state);
            }
            else
            {
                state_space->copy_from_point(control);
            }

            //Store previous control and the now current state.
            state_space->copy_point(prior_state, state);
            input_control_space->copy_point(prior_control, control);
        }

        void sliding_rigid_body_t::update_phys_configs(config_list_t& configs, unsigned& index) const
        {
            root_config.set_position(_x, _y, _z);
            plant_t::update_phys_configs(configs, index);
        }

        void sliding_rigid_body_t::update_collision_info()
        {
            root_config.set_position(_x, _y, _z);
            plant_t::update_collision_info();
        }

        void sliding_rigid_body_t::steering_function(const state_t* start, const state_t* goal, plan_t& result_plan)
        {
            //    PRX_ERROR_S ("Received start: " << state_space->print_point(start) << " and goal: " << state_space->print_point(goal));
            //    PRX_INFO_S ("Max step: " << max_step << " distance: " << state_space->distance(start,goal));
            double time = std::ceil(state_space->distance(start, goal) / max_step);
            time *= simulation::simulation_step;
            //    if (!result_plan.steps.empty())
            //        time += result_plan.steps.back().duration;
            //    PRX_WARN_S ("TYIME: " << time);
            result_plan.copy_onto_back(goal, time);
            //    PRX_WARN_S ("Pushed point: " << )

        }

        void sliding_rigid_body_t::append_contingency(plan_t& result_plan, double duration)
        {
            double difference = duration - result_plan.length();
            //    PRX_DEBUG_S("Difference in append: "<<difference);
            //    PRX_ASSERT(difference >= 0);
            PRX_DEBUG_S("Result plan size is: " << result_plan.size());
            if( result_plan.size() == 0 )
            {
                state_t* temp_state = state_space->alloc_point();
                result_plan.copy_onto_back(temp_state, 0.0);
                state_space->free_point(temp_state);
                result_plan.back().duration += duration;
            }
            else
            {
                PRX_DEBUG_S("Difference is: " << difference);
                PRX_ASSERT(difference >= 0);
                result_plan.back().duration += difference;
            }
        }


    }
}

