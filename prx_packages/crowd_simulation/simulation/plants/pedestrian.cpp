/**
 * @file pedestrian.cpp 
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

#include "simulation/structures/world_structure.hpp"
#include "simulation/plants/pedestrian.hpp"

#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>

#include "prx/utilities/math/2d_geometry/angle.hpp"

PLUGINLIB_EXPORT_CLASS(prx::packages::crowd::pedestrian_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    
    namespace packages
    {
        namespace crowd
        {

            const unsigned pedestrian_t::STATE_X = 0;
            const unsigned pedestrian_t::STATE_Y = 1;
            const unsigned pedestrian_t::STATE_Z = 2;

            const unsigned int pedestrian_t::CONTROL_V = 0;
            const unsigned int pedestrian_t::CONTROL_THETA = 1;

            pedestrian_t::pedestrian_t() : integration_plant_t()
            {
                state_memory = {&_x,&_y,&_z};
                control_memory = {&_v,&_theta};

                state_space = new space_t("XYZ", state_memory);
                state = state_space->alloc_point();
                input_control_space = new space_t("Vector", control_memory);
                
                path_follower = NULL;
            }

            pedestrian_t::~pedestrian_t()
            {
                state_space->free_point( state );
            }

            void pedestrian_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
            {
                integration_plant_t::init(reader, template_reader);
                _z = parameters::get_attribute_as<double>("z", reader, template_reader, 1.5);
                max_steps = parameters::get_attribute_as<double>("max_steps", reader, template_reader, PRX_INFINITY);
                double turn_speed = parameters::get_attribute_as<double>("turn_speed", reader, template_reader, 2.0);
                _turn_rate = turn_speed * simulation::simulation_step;
                
                _prior_theta = _theta;
            }

            void pedestrian_t::set_world_structure( world_structure_t* input_structure )
            {
                world_structure = input_structure;
            }

            void pedestrian_t::propagate(const double simulation_step)
            {
                //Perhaps I need to constrain how far they can turn in any one direction
                integration_plant_t::propagate(simulation_step);

                state_space->copy_to_point( state );
                world_structure->fix_state( state, path_follower );
                state_space->copy_from_point( state );
            }

            void pedestrian_t::update_phys_configs(config_list_t& configs, unsigned& index) const
            {
                double used_theta = norm_angle_pi(_prior_theta);
                root_config.set_position(_x, _y, _z);
                root_config.set_xyzw_orientation(0.0, 0.0, sin(used_theta / 2.0), cos(used_theta / 2.0));
                plant_t::update_phys_configs(configs, index);
            }
            void pedestrian_t::update_collision_info()
            {
                root_config.set_position(_x, _y, _z);
                root_config.set_xyzw_orientation(0.0, 0.0, sin(_theta / 2.0), cos(_theta / 2.0));
                plant_t::update_collision_info();
            }

            void pedestrian_t::update_derivative(state_t * const result)
            {
                //Figure out the theta difference
                double theta_diff = minimum_angle_with_orientation( _prior_theta, _theta );
                double vel = _v;

                //If the theta difference is too large, we just kinda have to stop moving or we'll run into things
                // if( fabs( theta_diff ) > 5 * _turn_rate )
                // {
                //     vel = 0;
                // }

                if( fabs( theta_diff ) > _turn_rate )
                {
                    //The best control heading is from the current heading, moving as far as we can towards the new one
                    double sign = theta_diff/fabs(theta_diff);
                    _prior_theta = _prior_theta + sign * _turn_rate;
                }
                else
                {
                    _prior_theta = _theta;
                }
                
                result->memory[STATE_X] = vel * cos(_prior_theta);
                result->memory[STATE_Y] = vel * sin(_prior_theta);
                result->memory[STATE_Z] = 0;
            }

            void pedestrian_t::steering_function(const state_t* start, const state_t* goal, plan_t& result_plan)
            {
                double difx = goal->at(0) - start->at(0);
                double dify = goal->at(1) - start->at(1);
                double magn = sqrt(difx * difx + dify * dify);
                double max_vel = input_control_space->get_bounds()[0]->get_upper_bound();

                double min_time = magn / max_vel;
                double steps = std::ceil(min_time / simulation::simulation_step);
                magn = magn / (steps * simulation::simulation_step);

                std::vector<double> new_control;
                new_control.push_back(magn);

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

            void pedestrian_t::get_current_position(std::vector<double>& pos)
            {
                PRX_ASSERT(pos.size() == 3);
                pos[0] = _x;
                pos[1] = _y;
                pos[2] = _z;
            }

            void pedestrian_t::link_path_follower( path_follow_controller_t* cont )
            {
                path_follower = cont;
            }
            
            const path_follow_controller_t* pedestrian_t::get_path_follower()
            {
                return path_follower;
            }
            

            void pedestrian_t::set_param(const std::string& parameter_name, const boost::any& value)
            {
                if( std::strcmp(parameter_name.c_str(), "z") == 0 )
                {
                    _z = boost::any_cast<double>(value);
                }
            }

            void pedestrian_t::get_state( std::vector<double>& curr_state)
            {
                state_space->copy_to_vector(curr_state);
            }

        }
    }
}

