/**
 * @file superball_sampler.hpp 
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

#ifndef PRX_SUPERBALL_SAMPLER_HPP
#define PRX_SUPERBALL_SAMPLER_HPP

#include "prx/planning/modules/samplers/sampler.hpp"

namespace prx 
{ 
     namespace plan 
     {
        /**
         * Performs uniform sampling over a space.
         * @brief <b> Performs uniform sampling over a space.</b>
         * @author Zakary Littlefield
         */
        class superball_sampler_t : public sampler_t
        {
            
            public:
                std::vector<int> indices;
                superball_sampler_t() : sampler_t() 
                {
                    for(int i=0;i<12;i++)
                    {
                        indices.push_back(i);
                    }
                }

                virtual ~superball_sampler_t() {};
                
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

#endif