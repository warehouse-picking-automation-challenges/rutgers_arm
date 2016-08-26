/**
 * @file stable_pose_sampler.hpp
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
#pragma once

#ifndef PRX_STABLE_POSE_SAMPLER_HPP
#define PRX_STABLE_POSE_SAMPLER_HPP

#include "prx/planning/modules/samplers/sampler.hpp"
#include "../../../baxter/simulation/plants/manipulator.hpp"
//#include "simulation/plants/movable_body_plant.hpp"

namespace prx
{
    namespace packages
    {
        namespace multi_arm
        {

            using namespace baxter;
            /**
             * Performs sampling for the manipulator over multiple surfaces.
             *
             * @brief <b> Performs sampling for the manipulator over surfaces.</b>
             *
             * @author Andrew Dobson
             */
            class stable_pose_sampler_t : public plan::sampler_t
            {
            public:
                stable_pose_sampler_t();
                ~stable_pose_sampler_t();
                void sample(const util::space_t* space, util::space_point_t* point);
                void sample_near(const util::space_t* space, util::space_point_t* near_point, std::vector<util::bounds_t*>& bounds, util::space_point_t* point);
            };
        }
    }
}

#endif //PRX_STABLE_POSE_SAMPLER_HPP
