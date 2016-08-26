/**
 * @file apc_sampler.hpp 
 *  * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_APC_SAMPLER_HPP
#define PRX_APC_SAMPLER_HPP

#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/utilities/math/configurations/bounds.hpp"
#include "prx/utilities/math/configurations/quaternion.hpp"
#include "prx/utilities/math/configurations/config.hpp"

namespace prx 
{    
     namespace packages 
     {
        namespace manipulation
        {
            class manipulation_world_model_t;
        }

        namespace apc
        {
            class apc_sampler_t : public plan::sampler_t
            {
                
                public:
                    
                    apc_sampler_t() : sampler_t() 
                    {
                        manip_model = NULL;
                    }

                    virtual ~apc_sampler_t() {};
                    
                    virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);

                    /**
                     * @copydoc sampler_t::sample(const space_t*, space_point_t*)
                     */
                    virtual void sample(const util::space_t* space,util::space_point_t* point);

                    /**
                     * @copydoc sampler_t::sample_near(const space_t*, space_point_t*, std::vector<bounds_t*>&, space_point_t*)
                     */
                    virtual void sample_near(const util::space_t* space, util::space_point_t* near_point, std::vector<util::bounds_t*>& bounds, util::space_point_t* point);

                    virtual bool workspace_near_sample(util::config_t workspace_point, double neighborhood, const util::space_t* space, util::space_point_t* point);
                    virtual void link_world_model(plan::world_model_t* wm);
                protected:
                    manipulation::manipulation_world_model_t* manip_model;
                    std::vector<util::bounds_t*> bounds;
                    util::quaternion_t quat;
                    std::string manipulation_context;
                    util::space_point_t* manipulation_state;
                    double uniform_sampling_probability;
                    bool use_fk;
                    int fk_tries;

            };
        }

    } 
}

#endif