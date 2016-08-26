/**
 * @file stable_pose_sampler.cpp
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

#include "planning/modules/stable_pose_sampler.hpp"

#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::multi_arm::stable_pose_sampler_t, prx::plan::sampler_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace multi_arm
        {
            stable_pose_sampler_t::stable_pose_sampler_t()
            {

            }

            stable_pose_sampler_t::~stable_pose_sampler_t()
            {

            }

            void stable_pose_sampler_t::sample(const util::space_t* space, util::space_point_t* point)
            {

            }

            void stable_pose_sampler_t::sample_near(const util::space_t* space, util::space_point_t* near_point, std::vector<util::bounds_t*>& bounds, util::space_point_t* point)
            {

            }

        }
    }
}

