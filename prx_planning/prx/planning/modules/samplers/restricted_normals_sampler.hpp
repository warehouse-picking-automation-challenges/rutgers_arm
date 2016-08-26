/**
 * @file uniform_sampler.hpp 
 *  * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2016, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_RESTRICTED_NORMALS_SAMPLER_HPP
#define PRX_RESTRICTED_NORMALS_SAMPLER_HPP

#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/utilities/math/configurations/quaternion.hpp"
#include "prx/utilities/math/configurations/vector.hpp"

namespace prx 
{
    namespace plan 
    {
        /**
         * Performs uniform sampling while rejecting samples that violate certain normal constraints.
         * This class assumes that there are euler angles as the last three parts of the state space.
         *
         * @brief <b> Performs rejection sampling on certain normals </b>
         * @author Andrew Kimmel
         */
        class restricted_normals_sampler_t : public sampler_t
        {
            
            public:
                
                restricted_normals_sampler_t() : sampler_t() {}

                virtual ~restricted_normals_sampler_t() {};

                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);
                
                /**
                 * @copydoc sampler_t::sample(const space_t*, space_point_t*)
                 */
                virtual void sample(const util::space_t* space,util::space_point_t* point);

                /**
                 * @copydoc sampler_t::sample_near(const space_t*, space_point_t*, std::vector<bounds_t*>&, space_point_t*)
                 */
                virtual void sample_near(const util::space_t* space, util::space_point_t* near_point, std::vector<util::bounds_t*>& bounds, util::space_point_t* point);
        

            protected:

                util::vector_t default_axis, rotated_axis, converted_euler;

                util::quaternion_t random_quaternion;
        };
    } 
}

#endif