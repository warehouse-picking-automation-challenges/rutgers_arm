/**
 * @file apc_sampler.cpp 
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


#include "planning/samplers/apc_sampler.hpp"
#include "planning/manipulation_world_model.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::packages::apc::apc_sampler_t, prx::plan::sampler_t)

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
            void apc_sampler_t::sample(const space_t* space, space_point_t* point)
            {
                PRX_ASSERT(space->get_space_name()==manip_model->get_state_space()->get_space_name());
                //uniform_sampling_probability determines how much fraction of the samples are random
                if(uniform_random()>uniform_sampling_probability)
                {
                    config_t config;
                    config.set_position(bounds[0]->uniform_random_bounds(),bounds[1]->uniform_random_bounds(),bounds[2]->uniform_random_bounds());
                    config.set_orientation(quat);
                    std::string old_context = manip_model->get_current_context();
                    manip_model->use_context(manipulation_context);
                    manip_model->convert_spaces(manip_model->get_state_space(),manipulation_state,space,point);
                    bool suc = false;
                    if(!use_fk)
                    {
                        //Do IK
                        suc = manip_model->IK(manipulation_state,manipulation_state,config);
                    }
                    else
                    {
                        //Repeatedly do FK till the EE Config falls within the bounds
                        int num_tries = fk_tries;
                        while(--num_tries>=0)
                        {
                            space->uniform_sample(point);
                            space->copy_from_point(point);
                            manip_model->FK(config);
                            double x,y,z;
                            config.get_position(x,y,z);
                            // PRX_PRINT("Sampled "<<config,PRX_TEXT_RED);
                            if(bounds[0]->is_valid(x) && bounds[1]->is_valid(y) && bounds[2]->is_valid(z))
                            {
                                // PRX_PRINT("Sampled "<<config,PRX_TEXT_GREEN);
                                suc = true;
                                manip_model->get_state_space()->copy_to_point(manipulation_state);
                                break;
                            }

                        }
                    }
                    
                    //Either IK did not succeed or repeated FK failed to sample within EE Config bounds
                    if(!suc)
                    {
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
                    //uniform_sampling_probability of the time
                    space->uniform_sample(point);
                }

            }
            
            void apc_sampler_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
            {
                const std::vector<double> min = parameters::get_attribute_as<std::vector<double> >("min", reader, template_reader);
                const std::vector<double> max = parameters::get_attribute_as<std::vector<double> >("max", reader, template_reader);
                uniform_sampling_probability = parameters::get_attribute_as<double >("uniform_sampling_probability", reader, template_reader,0.5);
                use_fk = parameters::get_attribute_as<bool >("use_fk", reader, template_reader,false);
                fk_tries = parameters::get_attribute_as<int >("fk_tries", reader, template_reader,100);
                manipulation_context = parameters::get_attribute_as<std::string>("manipulation_context", reader, template_reader);

                unsigned dimension = min.size();

                for( unsigned i = 0; i < dimension; i++ )
                {
                    bounds.push_back(new bounds_t());
                }
                bounds::set_bounds(bounds, min, max);
                bounds::verify(bounds);


                std::vector<double> orientation = parameters::get_attribute_as<std::vector<double> >("default_orientation", reader, template_reader);
                quat.set(orientation[0],orientation[1],orientation[2],orientation[3]);
                quat.normalize();
            }

            void apc_sampler_t::sample_near(const space_t* space, space_point_t* near_point, std::vector<bounds_t*>& in_bounds, space_point_t* point)
            {
                space->uniform_sample_near(point, near_point, in_bounds);
            }

            bool apc_sampler_t::workspace_near_sample(config_t workspace_point, double neighborhood, const space_t* space, space_point_t* point)
            {
                PRX_ASSERT(space->get_space_name()==manip_model->get_state_space()->get_space_name());
                //uniform_sampling_probability determines how much fraction of the samples are random
                
                config_t config;
                
                std::string old_context = manip_model->get_current_context();
                manip_model->use_context(manipulation_context);
                manip_model->convert_spaces(manip_model->get_state_space(),manipulation_state,space,point);
                bool suc = false;
                double wx, wy, wz;
                workspace_point.get_position(wx,wy,wz);
                //Repeatedly do FK till the EE Config falls within the bounds
                int num_tries = fk_tries*10;
                while(--num_tries>=0)
                {
                    space->uniform_sample(point);
                    space->copy_from_point(point);
                    manip_model->FK(config);
                    double x,y,z;
                    config.get_position(x,y,z);
                    // PRX_PRINT("Sampled "<<config,PRX_TEXT_RED);
                    if( (x-wx)*(x-wx) + (y-wy)*(y-wy) + (z-wz)*(z-wz) < (neighborhood*neighborhood) )
                    {
                        // PRX_PRINT("Sampled "<<config,PRX_TEXT_GREEN);
                        suc = true;
                        manip_model->get_state_space()->copy_to_point(manipulation_state);
                        break;
                    }

                }
                
                
                //Either IK did not succeed or repeated FK failed to sample within EE Config bounds
                
                if(suc)
                {
                    manip_model->convert_spaces(space,point,manip_model->get_state_space(),manipulation_state);
                }
                manip_model->use_context(old_context);

                return suc;
                
            }


            void apc_sampler_t::link_world_model(plan::world_model_t* wm)
            {
                PRX_PRINT("Linking world model to the apc sampler.", PRX_TEXT_GREEN);
                manip_model = dynamic_cast<manipulation_world_model_t*>(wm);
                std::string old_context = manip_model->get_current_context();
                manip_model->use_context(manipulation_context);
                manipulation_state = manip_model->get_state_space()->alloc_point();
                manip_model->use_context(old_context);
            }
        }
    }
}