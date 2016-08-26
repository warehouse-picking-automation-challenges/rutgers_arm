/**
 * @file double_integrator_2d.cpp
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

#include "prx/simulation/systems/plants/double_integrator_2d.hpp"
#include "prx/utilities/math/2d_geometry/angle.hpp"

#include "prx/utilities/spaces/space.hpp"

#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::double_integrator_2d_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    namespace sim
    {

        const unsigned double_integrator_2d_t::STATE_X = 0;
        const unsigned double_integrator_2d_t::STATE_Y = 1;
        const unsigned double_integrator_2d_t::STATE_X_DOT = 2;
        const unsigned double_integrator_2d_t::STATE_Y_DOT = 3;
        const unsigned double_integrator_2d_t::CONTROL_A_X = 0;
        const unsigned double_integrator_2d_t::CONTROL_A_Y = 1;

        double_integrator_2d_t::double_integrator_2d_t()
        {
            state_memory = {&_x,&_y,&_xdot,&_ydot};
            control_memory = {&_ax,&_ay};

            state_space = new space_t("TWO_D_BODY", state_memory);
            input_control_space = new space_t("XddYdd", control_memory);

            _z = 0;
        }

        double_integrator_2d_t::~double_integrator_2d_t() { }

        void double_integrator_2d_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            integration_plant_t::init(reader, template_reader);
            _z = parameters::get_attribute_as<double>("z", reader, template_reader, 2.5);
        }

        void double_integrator_2d_t::propagate(const double simulation_step)
        {
            integration_plant_t::propagate(simulation_step);
        }

        void double_integrator_2d_t::update_phys_configs(config_list_t& configs, unsigned& index) const
        {
            root_config.set_position(_x, _y, _z);
            root_config.set_xyzw_orientation(0.0, 0.0, 0.0, 1.0);
            plant_t::update_phys_configs(configs, index);
        }

        void double_integrator_2d_t::update_derivative(state_t * const result)
        {
            result->memory[STATE_X] = _xdot;
            result->memory[STATE_Y] = _ydot;
            result->memory[STATE_X_DOT] = _ax;
            result->memory[STATE_Y_DOT] = _ay;
        }

        bool compute_switch_times(double start_x, double start_v, double goal_x, double goal_v, double max_acc, double& t1, double& t2)
        {
            double const_s;
            double const_g;
            double target_x;
            double target_vel;
            if( start_x < goal_x )
            {
                const_s = start_x - .5 * (1 / max_acc) * start_v*start_v;
                const_g = goal_x + .5 * (1 / max_acc) * goal_v*goal_v;
                target_x = (const_s + const_g)*.5;
                target_vel = sqrt((const_g - const_s) * max_acc);
                t1 = fabs((start_v - target_vel) / max_acc);
                t2 = fabs((target_vel - goal_v) / max_acc);
                t1 = ((int)(t1 / simulation::simulation_step)) * simulation::simulation_step;
                t2 = ((int)(t2 / simulation::simulation_step)) * simulation::simulation_step;
                return true;
            }
            else
            {
                const_s = start_x + .5 * (1 / max_acc) * start_v*start_v;
                const_g = goal_x - .5 * (1 / max_acc) * goal_v*goal_v;
                target_x = (const_s + const_g)*.5;
                target_vel = sqrt((const_s - const_g) * max_acc);
                t1 = fabs((start_v - target_vel) / max_acc);
                t2 = fabs((target_vel - goal_v) / max_acc);
                t1 = ((int)(t1 / simulation::simulation_step)) * simulation::simulation_step;
                t2 = ((int)(t2 / simulation::simulation_step)) * simulation::simulation_step;
                return false;
            }
        }

        void double_integrator_2d_t::steering_function(const state_t* start, const state_t* goal, plan_t& result_plan)
        {
            double start_x1 = start->memory[STATE_X];
            double start_x2 = start->memory[STATE_Y];
            double start_v1 = start->memory[STATE_X_DOT];
            double start_v2 = start->memory[STATE_Y_DOT];

            double goal_x1 = goal->memory[STATE_X];
            double goal_x2 = goal->memory[STATE_Y];
            double goal_v1 = goal->memory[STATE_X_DOT];
            double goal_v2 = goal->memory[STATE_Y_DOT];

            PRX_INFO_S(state_space->print_point(start));
            PRX_INFO_S(state_space->print_point(goal));

            double t_1_s, t_1_g;
            double total_t1;

            double t_2_s, t_2_g;
            double total_t2;

            double acc1, acc2;

            bool bang1 = compute_switch_times(start_x1, start_v1, goal_x1, goal_v1, 1, t_1_s, t_1_g);
            bool bang2 = compute_switch_times(start_x2, start_v2, goal_x2, goal_v2, 1, t_2_s, t_2_g);

            total_t1 = t_1_s + t_1_g;
            total_t2 = t_2_s + t_2_g;
            PRX_INFO_S(t_1_s << " " << t_1_g);
            PRX_INFO_S(t_2_s << " " << t_2_g);
            PRX_INFO_S(total_t1 << " " << total_t2);


            if( total_t1 < total_t2 )
            {
                acc2 = 1;
                double min_acc = 0;
                double max_acc = 1;
                //loop over accelerations that can work for t1
                while( fabs(total_t1 - total_t2) >= simulation::simulation_step / 2 )
                {
                    acc1 = (max_acc + min_acc) / 2.0;
                    compute_switch_times(start_x1, start_v1, goal_x1, goal_v1, acc1, t_1_s, t_1_g);
                    total_t1 = t_1_s + t_1_g;
                    if( total_t1 < total_t2 )
                        max_acc = acc1;
                    else
                        min_acc = acc1;
                }
            }
            else
            {
                acc1 = 1;
                double min_acc = 0;
                double max_acc = 1;
                //loop over accelerations that can work for t1
                while( fabs(total_t1 - total_t2) >= simulation::simulation_step / 2 )
                {
                    acc2 = (max_acc + min_acc) / 2.0;
                    compute_switch_times(start_x2, start_v2, goal_x2, goal_v2, acc2, t_2_s, t_2_g);
                    total_t2 = t_2_s + t_2_g;
                    if( total_t2 < total_t1 )
                        max_acc = acc2;
                    else
                        min_acc = acc2;
                }
            }

            PRX_INFO_S(t_1_s << " " << t_1_g);
            PRX_INFO_S(t_2_s << " " << t_2_g);
            PRX_INFO_S(total_t1 << " " << total_t2);

            result_plan.append_onto_back(std::min(t_1_s, t_2_s));
            result_plan.append_onto_back(std::max(t_1_s, t_2_s) - std::min(t_1_s, t_2_s));
            result_plan.append_onto_back(total_t1 - std::max(t_1_s, t_2_s));

            result_plan[0].control->at(0) = (bang1 ? acc1 : -acc1);
            result_plan[0].control->at(1) = (bang2 ? acc2 : -acc2);
            if( t_1_s < t_2_s )
            {
                result_plan[1].control->at(0) = (bang1 ? -acc1 : acc1);
                result_plan[1].control->at(1) = (bang2 ? acc2 : -acc2);
            }
            else
            {
                result_plan[1].control->at(0) = (bang1 ? acc1 : -acc1);
                result_plan[1].control->at(1) = (bang2 ? -acc2 : acc2);
            }

            result_plan[2].control->at(0) = (bang1 ? -acc1 : acc1);
            result_plan[2].control->at(1) = (bang2 ? -acc2 : acc2);

            PRX_DEBUG_COLOR(result_plan.print(),PRX_TEXT_CYAN);

        }


    }
}

