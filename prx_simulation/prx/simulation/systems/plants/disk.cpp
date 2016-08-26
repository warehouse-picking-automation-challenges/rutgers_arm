/**
 * @file disk.cpp 
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

#include "prx/simulation/systems/plants/disk.hpp"
#include "prx/utilities/math/2d_geometry/angle.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::disk_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    namespace sim
    {
        const unsigned disk_t::STATE_X = 0;
        const unsigned disk_t::STATE_Y = 1;

        const unsigned int disk_t::CONTROL_V = 0;
        const unsigned int disk_t::CONTROL_THETA = 1;

        disk_t::disk_t() : integration_plant_t()
        {
            state_memory = {&_x,&_y};
            control_memory = {&_v,&_theta};

            state_space = new space_t("XY", state_memory);
            input_control_space = new space_t("Vector", control_memory);
            //    previous_state = state_space->alloc_point();
            _z = 0;
        }

        disk_t::~disk_t() { }

        void disk_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            PRX_DEBUG_S("@@@Init disk_t : " << pathname);
            integration_plant_t::init(reader, template_reader);
            _z = parameters::get_attribute_as<double>("z", reader, template_reader, 1.5);
            max_steps = parameters::get_attribute_as<double>("max_steps", reader, template_reader, PRX_INFINITY);
        }

        void disk_t::propagate(const double simulation_step)
        {
            //    PRX_DEBUG_S ("V: " << (*input_control_space)[0] << " Theta: " << (*input_control_space)[1]);

            //    state_space->copy_to_point(previous_state);
            //    PRX_WARN_S ("Propagation step : " << simulation_step);
            integration_plant_t::propagate(simulation_step);
            //    state_t* current_state = state_space->alloc_point();
            //    PRX_DEBUG_S("Distance to new state:  " << state_space->distance(previous_state,current_state));
        }

        void disk_t::update_phys_configs(config_list_t& configs, unsigned& index) const
        {
            if( ros::this_node::getName() == "planning" )
            {
                PRX_DEBUG_COLOR(config_names[0],PRX_TEXT_CYAN);
            }
            root_config.set_position(_x, _y, _z);
            root_config.set_xyzw_orientation(0.0, 0.0, sin(_theta / 2.0), cos(_theta / 2.0));
            plant_t::update_phys_configs(configs, index);
        }
        void disk_t::update_collision_info()
        {
            root_config.set_position(_x, _y, _z);
            root_config.set_xyzw_orientation(0.0, 0.0, sin(_theta / 2.0), cos(_theta / 2.0));
            plant_t::update_collision_info();
        }

        void disk_t::update_derivative(state_t * const result)
        {
            //                PRX_WARN_S ("V : " << _v << " , Theta: " << _theta);
            result->memory[STATE_X] = _v * cos(_theta);
            result->memory[STATE_Y] = _v * sin(_theta);
        }

        void disk_t::steering_function(const state_t* start, const state_t* goal, plan_t& result_plan)
        {
            double difx = goal->at(0) - start->at(0);
            double dify = goal->at(1) - start->at(1);
            double magn = sqrt(difx * difx + dify * dify);
            double max_vel = input_control_space->get_bounds()[0]->get_upper_bound();

            //    difx = (difx * max_vel) / magn;
            //    dify = (dify * max_vel) / magn;
            double min_time = magn / max_vel;
            double steps = std::ceil(min_time / simulation::simulation_step);
            magn = magn / (steps * simulation::simulation_step);

            std::vector<double> new_control;
            new_control.push_back(magn);
            //    PRX_DEBUG_S ("Magn: " << magn);
            new_control.push_back(atan2(dify, difx));
            if( steps > max_steps )
            {
                steps = max_steps;
            }
            control_t* temp_control = input_control_space->alloc_point();
            result_plan.copy_onto_back(temp_control, steps * simulation::simulation_step);
            input_control_space->free_point(temp_control);
            input_control_space->set_from_vector(new_control, result_plan.back().control);
            if( !input_control_space->satisfies_bounds(result_plan.back().control) )
            {
                result_plan.clear();
            }
        }

        void disk_t::set_param(const std::string& parameter_name, const boost::any& value)
        {
            if( std::strcmp(parameter_name.c_str(), "z") == 0 )
            {
                _z = boost::any_cast<double>(value);
            }
        }



    }
}

