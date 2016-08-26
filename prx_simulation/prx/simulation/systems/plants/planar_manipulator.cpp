/**
 * @file planar_manipulator.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "planar_manipulator.hpp"
#include "prx/utilities/spaces/space.hpp"
#include <pluginlib/class_loader.h>
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(prx::sim::planar_manipulator_t, prx::sim::system_t)


namespace prx
{
    using namespace util;
    
    namespace sim
    {
        const unsigned planar_manipulator_t::STATE_X = 0;
        const unsigned planar_manipulator_t::STATE_Y = 1;
        const unsigned planar_manipulator_t::STATE_THETA1 = 2;
        const unsigned planar_manipulator_t::STATE_THETA2 = 3;
        const unsigned planar_manipulator_t::STATE_THETA3 = 4;
        
        
        
        planar_manipulator_t::planar_manipulator_t() : integration_plant_t()
        {
            state_memory   = { &s_x , &s_y , &s_theta[0] , &s_theta[1] , &s_theta[2] };
            control_memory = { &c_vx , &c_vy , &c_dtheta[0] , &c_dtheta[1] , &c_dtheta[2] };
            
            state_space = new space_t("XY|R|R|R", state_memory);
            input_control_space = new space_t("XdYd|Rd|Rd|Rd", control_memory);
        }
        
        
        
        planar_manipulator_t::~planar_manipulator_t()
        {
        }
        
        
        
        void planar_manipulator_t::init(const parameter_reader_t *reader, 
            const parameter_reader_t *template_reader)
        {
            integration_plant_t::init(reader, template_reader);
            
            p_z = parameters::get_attribute_as<double>("z", reader, template_reader, 0.0);
            
            double ly, lz;
            for(int i = 0; i < 3; i++)
            {
                geometries[config_names[i + 1]].get_box(lengths[i], ly, lz);
            }
        }
        
        
        
        void planar_manipulator_t::update_phys_configs(config_list_t& configs, unsigned& index) const
        {
            double linkx[3], linky[3], theta[3];
            
            root_config.set_position(s_x, s_y, p_z);
            root_config.set_xyzw_orientation(0.0, 0.0, 0.0, 1.0);
            
            augment_config_list(configs, index);
            configs[index].first = root_geom;
            configs[index].second.set_position(s_x, s_y, p_z);
            configs[index].second.set_xyzw_orientation(0.0, 0.0, 0.0, 1.0);
            ++index;
            
            for(int i = 0; i < 3; i++)
            {
                theta[i] = s_theta[i];
                if(i > 0)
                    theta[i] += theta[i - 1];
                
                linkx[i] = 0.5*lengths[i]*cos(theta[i]);
                linky[i] = 0.5*lengths[i]*sin(theta[i]);
                
                if(i > 0)
                {
                    linkx[i] += linkx[i - 1] + 0.5*lengths[i - 1]*cos(theta[i - 1]);
                    linky[i] += linky[i - 1] + 0.5*lengths[i - 1]*sin(theta[i - 1]);
                }
                else
                {
                    linkx[i] += s_x;
                    linky[i] += s_y;
                }
                
                augment_config_list(configs, index);
                configs[index].first = config_names[i + 1];
                configs[index].second.set_position(linkx[i], linky[i], p_z);
                configs[index].second.set_xyzw_orientation(0.0, 0.0, sin(theta[i] / 2.0), cos(theta[i] / 2));
                ++index;
            }//for
        }//update_phys_configs()
        
        
        
        void planar_manipulator_t::update_derivative(state_t * const result)
        {
            result->at(STATE_X) = c_vx;
            result->at(STATE_Y) = c_vy;
            result->at(STATE_THETA1) = c_dtheta[0];
            result->at(STATE_THETA2) = c_dtheta[1];
            result->at(STATE_THETA3) = c_dtheta[2];
        }
        
        
        
        void planar_manipulator_t::steering_function(const sim::state_t *start, const sim::state_t *goal, sim::plan_t& plan)
        {
            unsigned int dim = get_control_space()->get_dimension();
            std::vector<bounds_t*> bounds = get_control_space()->get_bounds();
            double distances[dim], durations[dim], maxTime;
            unsigned int i;
            
            maxTime = 0;
            plan.clear();
            
            for(i = 0; i < dim; i++)
            {
                distances[i] = goal->at(i) - start->at(i);
                
                if(distances[i] < 0.0 && bounds[i]->get_lower_bound() < 0.0)
                    durations[i] = distances[i] / bounds[i]->get_lower_bound();
                else if(distances[i] > 0.0 && bounds[i]->get_upper_bound() > 0.0)
                    durations[i] = distances[i] / bounds[i]->get_upper_bound();
                else if(distances[i] == 0.0)
                    ; //Nothing to do
                else
                {
                    PRX_ERROR_S("Controls will not allow steering function to go from start to goal.");
                    return;
                }
                
                if(durations[i] > maxTime)
                    maxTime = durations[i];
            }//for
            
            maxTime = sim::simulation::simulation_step * std::ceil(maxTime / sim::simulation::simulation_step);
            
            plan.append_onto_back(maxTime);
            
            for(i = 0; i < dim; i++)
            {
                plan.at(0).control->at(i) = distances[i] / maxTime;
            }
        }//steering_function()
    }//namespace sim
}//namespace prx
