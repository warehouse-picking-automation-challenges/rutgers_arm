/**
 * @file 3d_plant.cpp 
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

#include "simulation/3d_plant.hpp"
#include "prx/utilities/math/2d_geometry/angle.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::conformant::threed_plant_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace packages
    {
        namespace conformant
        {
            const unsigned threed_plant_t::STATE_X = 0;
            const unsigned threed_plant_t::STATE_Y = 1;
            const unsigned threed_plant_t::STATE_Z = 2;
            const unsigned threed_plant_t::STATE_ROLL = 3;
            const unsigned threed_plant_t::STATE_PITCH = 4;
            const unsigned threed_plant_t::STATE_YAW = 5;

            threed_plant_t::threed_plant_t() : integration_plant_t()
            {
                state_memory = {&_x,&_y,&_z,&_roll,&_pitch,&_yaw};
                control_memory = {&_vx,&_vy,&_vz,&_vroll,&_vpitch,&_vyaw};

                state_space = new space_t("PointRollPitchYaw", state_memory);
                input_control_space = new space_t("XXXXXX", control_memory);
            }

            threed_plant_t::~threed_plant_t() { }

            void threed_plant_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
            {
                PRX_DEBUG_S("@@@Init threed_plant_t : " << pathname);
                integration_plant_t::init(reader, template_reader);
            }

            void threed_plant_t::propagate(const double simulation_step)
            {
                integration_plant_t::propagate(simulation_step);
            }

            void threed_plant_t::update_phys_configs(config_list_t& configs, unsigned& index) const
            {
                root_config.set_position(_x, _y, _z);
                root_config.set_orientation(_roll, _pitch, _yaw);
                plant_t::update_phys_configs(configs, index);
            }

            void threed_plant_t::update_collision_info()
            {
                root_config.set_position(_x, _y, _z);
                root_config.set_orientation(_roll, _pitch, _yaw);
                plant_t::update_collision_info();
            }
            void threed_plant_t::update_derivative(state_t * const result)
            {
                //                PRX_WARN_S ("V : " << _v << " , Theta: " << _theta);
                result->memory[STATE_X] = _vx;
                result->memory[STATE_Y] = _vy;
                result->memory[STATE_Z] = _vz;
                result->memory[STATE_ROLL] = _vx;
                result->memory[STATE_PITCH] = _vy;
                result->memory[STATE_YAW] = _vz;
            }
            void threed_plant_t::steering_function(const state_t* start, const state_t* goal, plan_t& result_plan)
            {


                //each joint is independent
                double max_time = 0;
                for(unsigned i=0;i<6; i++)
                {
                    double dist = goal->memory[i] - start->memory[i];
                    if(i>=3)
                    {
                        if(dist > PRX_PI)
                        {
                            dist -= 2*PRX_PI;
                        }
                        else if(dist < -PRX_PI)
                        {
                            dist += 2*PRX_PI;
                        }
                    }
                    double vel = input_control_space->get_bounds()[i]->get_upper_bound();
                    double test_time = fabs(dist)/vel;
                    if(vel <= 0.0000001)
                        test_time = 0;
                    if(test_time>max_time)
                        max_time = test_time;
                }
                double steps = std::ceil(max_time / simulation::simulation_step);
                max_time = steps * simulation::simulation_step;
                result_plan.append_onto_back(max_time);
                std::vector<double> new_control;
                for(unsigned i=0;i<6; i++)
                {
                    double dist = goal->memory[i] - start->memory[i];
                    if(i>=3)
                    {
                        if(dist > PRX_PI)
                        {
                            dist -= 2*PRX_PI;
                        }
                        else if(dist < -PRX_PI)
                        {
                            dist += 2*PRX_PI;
                        }
                    }
                    new_control.push_back(dist/max_time);
                }
                input_control_space->set_from_vector(new_control, result_plan.back().control);
                if(max_time<simulation::simulation_step)
                {
                    result_plan.clear();
                }
                // else
                //     PRX_INFO_S(input_control_space->print_point(result_plan.back().control));




                // double difx = goal->at(0) - start->at(0);
                // double dify = goal->at(1) - start->at(1);
                // double difz = goal->at(2) - start->at(2);
                // double difang1 = goal->at(3) - start->at(3);
                // double difang2 = goal->at(4) - start->at(4);
                // double difang3 = goal->at(5) - start->at(5);
                // if(difang1 > PRX_PI)
                // {
                //     difang1 -= 2*PRX_PI;
                // }
                // else if(difang1 < -PRX_PI)
                // {
                //     difang1 += 2*PRX_PI;
                // }
                // if(difang2 > PRX_PI)
                // {
                //     difang2 -= 2*PRX_PI;
                // }
                // else if(difang2 < -PRX_PI)
                // {
                //     difang2 += 2*PRX_PI;
                // }
                // if(difang3 > PRX_PI)
                // {
                //     difang3 -= 2*PRX_PI;
                // }
                // else if(difang3 < -PRX_PI)
                // {
                //     difang3 += 2*PRX_PI;
                // }
                // double max_velx = input_control_space->get_bounds()[0]->get_upper_bound();
                // double max_vely = input_control_space->get_bounds()[1]->get_upper_bound();
                // double max_velz = input_control_space->get_bounds()[2]->get_upper_bound();
                // double max_velang1 = input_control_space->get_bounds()[3]->get_upper_bound();
                // double max_velang2 = input_control_space->get_bounds()[4]->get_upper_bound();
                // double max_velang3 = input_control_space->get_bounds()[5]->get_upper_bound();
                // double min_time_x = fabs(difx / max_velx);
                // double min_time_y = fabs(dify / max_vely);
                // double min_time_z = fabs(difz / max_velz);
                // double min_time_ang1 = fabs(difang1 / max_velang1);
                // double min_time_ang2 = fabs(difang2 / max_velang2);
                // double min_time_ang3 = fabs(difang3 / max_velang3);

                // double max_time = PRX_MAXIMUM(PRX_MAXIMUM(PRX_MAXIMUM(PRX_MAXIMUM(PRX_MAXIMUM(min_time_ang3,min_time_ang2),min_time_ang1),min_time_z),min_time_y),min_time_x);

                // double steps = std::ceil(max_time / simulation::simulation_step);
                // max_time = (steps * simulation::simulation_step);

                // std::vector<double> new_control;
                // new_control.push_back(difx/max_time);
                // new_control.push_back(dify/max_time);
                // new_control.push_back(difz/max_time);
                // new_control.push_back(difang1/max_time);
                // new_control.push_back(difang2/max_time);
                // new_control.push_back(difang3/max_time);

                // control_t* temp_control = input_control_space->alloc_point();
                // result_plan.copy_onto_back(temp_control, steps * simulation::simulation_step);
                // input_control_space->free_point(temp_control);
                // input_control_space->set_from_vector(new_control, result_plan.back().control);
                // if( !input_control_space->satisfies_bounds(result_plan.back().control) )
                // {
                //     result_plan.clear();
                // }
            }

        }



    }
}

