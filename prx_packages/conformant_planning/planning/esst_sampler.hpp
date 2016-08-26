/**
 * @file esst_sampler.hpp 
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

#ifndef PRX_ESST_SAMPLER_HPP
#define PRX_ESST_SAMPLER_HPP

#include "prx/planning/modules/samplers/sampler.hpp"

namespace prx 
{ 
     namespace packages 
     {
        namespace conformant
        {
            class esst_sampler_t : public plan::sampler_t
            {
                
                public:
                    
                    esst_sampler_t() : sampler_t() {}

                    virtual ~esst_sampler_t() {};
                    
                    /**
                     * @copydoc sampler_t::sample(const space_t*, space_point_t*)
                     */
                    virtual void sample(const util::space_t* space,util::space_point_t* point);

                    /**
                     * @copydoc sampler_t::sample_near(const space_t*, space_point_t*, std::vector<bounds_t*>&, space_point_t*)
                     */
                    virtual void sample_near(const util::space_t* space, util::space_point_t* near_point, std::vector<util::bounds_t*>& bounds, util::space_point_t* point);
            };
        }
    } 
}

#endif