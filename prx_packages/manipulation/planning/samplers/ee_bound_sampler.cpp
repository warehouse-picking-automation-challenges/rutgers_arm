/**
 * @file ee_bound_sampler.cpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zacharias Psarakis, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "planning/samplers/ee_bound_sampler.hpp"
#include "planning/manipulation_world_model.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/math/configurations/config.hpp"

#include <pluginlib/class_list_macros.h> 

#define SAMPLING_RANGE 0.001

PLUGINLIB_EXPORT_CLASS( prx::packages::manipulation::ee_bound_sampler_t, prx::plan::sampler_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;
    namespace packages
    {        
        namespace manipulation
        {
            void ee_bound_sampler_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
            {
                rejection_sampling = parameters::get_attribute_as<bool>("rejection_sampling", reader, template_reader, false);
                if(rejection_sampling)
                    PRX_PRINT("Bounded Rejection Sampling...", PRX_TEXT_GREEN);
                const std::vector<double> initial_sample = parameters::get_attribute_as<std::vector<double> >("initial_sample", reader, template_reader);
                const std::vector<double> min = parameters::get_attribute_as<std::vector<double> >("min", reader, template_reader);
                const std::vector<double> max = parameters::get_attribute_as<std::vector<double> >("max", reader, template_reader);
                const std::vector<double> euler_min = parameters::get_attribute_as<std::vector<double> >("euler_min", reader, template_reader);
                const std::vector<double> euler_max = parameters::get_attribute_as<std::vector<double> >("euler_max", reader, template_reader);
                manipulation_context = parameters::get_attribute_as<std::string>("manipulation_context", reader, template_reader);
                tries = 0;
                first_sample_tries = 0;

                unsigned dimension = min.size();

                for( unsigned i = 0; i < dimension; i++ )
                {
                    bounds.push_back(new bounds_t());
                    initial_sample_bounds.push_back(new bounds_t());
                    euler_bounds.push_back(new bounds_t());
                }
                bounds::set_bounds(bounds, min, max);
                bounds::set_bounds(euler_bounds, euler_min, euler_max);
                bounds::verify(bounds);
                bounds::verify(euler_bounds);

                //define a small area around the initial sample
                std::vector<double> min_init_sample;
                std::vector<double> max_init_sample;
                for(int i=0; i<initial_sample.size(); i++)
                {
                    min_init_sample.push_back(initial_sample[i]-SAMPLING_RANGE);
                    max_init_sample.push_back(initial_sample[i]+SAMPLING_RANGE);
                }
                bounds::set_bounds(initial_sample_bounds, min_init_sample, max_init_sample);
                bounds::verify(initial_sample_bounds);

                std::vector<double> orientation = parameters::get_attribute_as<std::vector<double> >("default_orientation", reader, template_reader);
                quat.set(orientation[0],orientation[1],orientation[2],orientation[3]);
                quat.normalize();
            }
            
            
            void ee_bound_sampler_t::sample(const space_t* space, space_point_t* point)
            {
                vector_t euler_angles(3);
                PRX_ASSERT(space->get_space_name()==manip_model->get_state_space()->get_space_name());
                util::quaternion_t quatx;
                util::quaternion_t quaty;
                util::quaternion_t quatz;
                if(!rejection_sampling)
                {
                    if(uniform_random()<0.5)//Bias the sampling towards the bounded area
                    {
                        tries=0;
                        bool suc=false;
                        std::string old_context = manip_model->get_current_context();
                        config_t config;
                        do{
                            config.zero();
                            config.set_position(bounds[0]->uniform_random_bounds(),bounds[1]->uniform_random_bounds(),bounds[2]->uniform_random_bounds());
                            quaty.set_from_euler(0,euler_bounds[1]->uniform_random_bounds(),0);
                            quatx.set_from_euler(euler_bounds[0]->uniform_random_bounds(),0,0);
                            quatz.set_from_euler(0,0,euler_bounds[2]->uniform_random_bounds());
                            quat = quatx*quatz*quaty;
                            config.set_orientation(quat);
                            manip_model->use_context(manipulation_context);
                            manip_model->convert_spaces(manip_model->get_state_space(),manipulation_state,space,point);
                            suc = manip_model->IK(manipulation_state,manipulation_state,config);
                            tries++;
                        }while(!suc && tries <20);

                        if(!suc)
                        {
                            PRX_PRINT("Uniform Sample instead of Bounded...",PRX_TEXT_RED);
                            space->uniform_sample(point);
                        }
                        else
                        {
                            manip_model->convert_spaces(space,point,manip_model->get_state_space(),manipulation_state);
                        }
                        manip_model->use_context(old_context);
                    }
                    else
                    {
                        PRX_PRINT("Uniform Sample instead of Bounded...",PRX_TEXT_RED);
                        space->uniform_sample(point);
                    }
                }
                else//rejection_sampling
                {
                    std::string old_context = manip_model->get_current_context();
                    config_t config;
                    if(uniform_random()<1)//Bias the sampling towards the bounded area
                    {
                        // TAG: DEBUG: TODO: This code is specific for generating a roadmap for the old unigripper model
                        // bool valid_orientation = false;
                        // vector_t up( 0, 0, 1.0 );
                        
                        double x,y,z;
                        do{
                            // PRX_PRINT("Trying Rejection Sampling...",PRX_TEXT_RED);
                            space->uniform_sample(point);
                            state_t* arm_start = manip_model->get_state_space()->alloc_point();
                            manip_model->get_state_space()->copy_from_point(point);
                            manip_model->use_context(manipulation_context);
                            manip_model->convert_spaces(manip_model->get_state_space(),manipulation_state,space,point);
                            manip_model->FK(config);
                            config.get_position(x,y,z);
                            // PRX_PRINT("Config = "<<config.print(),PRX_TEXT_GREEN);
                            // valid_orientation = (config.get_orientation().qv_rotation( up )[0] >= 0);
                        // }while( !bounds[0]->is_valid(x) || !bounds[1]->is_valid(y) || !bounds[2]->is_valid(z) || !valid_orientation );
                        }while( !bounds[0]->is_valid(x) || !bounds[1]->is_valid(y) || !bounds[2]->is_valid(z) );
                    }
                    else
                    {
                        // PRX_PRINT("Uniform Sample instead of Bounded...",PRX_TEXT_RED);
                        space->uniform_sample(point);
                    }
                    manip_model->use_context(old_context);
                }

            }

            void ee_bound_sampler_t::sample_near(const space_t* space, space_point_t* near_point, std::vector<bounds_t*>& in_bounds, space_point_t* point)
            {
                bool suc = false;
                std::string old_context = manip_model->get_current_context();
                config_t config;
                while(!suc){
                    first_sample_tries++;
                    config.zero();
                    config.set_position(initial_sample_bounds[0]->uniform_random_bounds(),initial_sample_bounds[1]->uniform_random_bounds(),initial_sample_bounds[2]->uniform_random_bounds());
                    config.set_orientation(quat);
                    manip_model->use_context(manipulation_context);
                    manip_model->convert_spaces(manip_model->get_state_space(),manipulation_state,space,point);
                    suc = manip_model->IK(manipulation_state,manipulation_state,config);
                    if(first_sample_tries % 20 == 0)
                    {
                        PRX_PRINT("IK FAILED FOR 20 TIMES! Relaxing Bounds...",PRX_TEXT_RED);
                        initial_sample_bounds[0]->set_lower_bound(initial_sample_bounds[0]->get_lower_bound()-SAMPLING_RANGE); 
                        initial_sample_bounds[1]->set_lower_bound(initial_sample_bounds[1]->get_lower_bound()-SAMPLING_RANGE); 
                        initial_sample_bounds[2]->set_lower_bound(initial_sample_bounds[2]->get_lower_bound()-SAMPLING_RANGE); 
                        initial_sample_bounds[0]->set_upper_bound(initial_sample_bounds[0]->get_upper_bound()+SAMPLING_RANGE); 
                        initial_sample_bounds[1]->set_upper_bound(initial_sample_bounds[1]->get_upper_bound()+SAMPLING_RANGE); 
                        initial_sample_bounds[2]->set_upper_bound(initial_sample_bounds[2]->get_upper_bound()+SAMPLING_RANGE); 
                        //ZP TODO Do we need this? We should come up with a correct check. Maybe load this from input?
                        if(initial_sample_bounds[0]->get_lower_bound()<0)
                        {
                            PRX_FATAL_S("Failed to provide IK for the current initial point. Out of Bounds. Please select a different initial point...");
                        }
                    }
                }
                PRX_PRINT("First Sample: "<<config.print(),PRX_TEXT_RED);
                manip_model->convert_spaces(space,point,manip_model->get_state_space(),manipulation_state);
                manip_model->use_context(old_context);
            }


            void ee_bound_sampler_t::link_world_model(plan::world_model_t* wm)
            {
                manip_model = dynamic_cast<manipulation_world_model_t*>(wm);
                std::string old_context = manip_model->get_current_context();
                manip_model->use_context(manipulation_context);
                manipulation_state = manip_model->get_state_space()->alloc_point();
                manip_model->use_context(old_context);
            }
        }
    }
}
