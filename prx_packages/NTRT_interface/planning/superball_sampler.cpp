/**
 * @file superball_sampler.cpp 
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


#include "planning/superball_sampler.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/math/configurations/bounds.hpp"
#include "prx/utilities/definitions/random.hpp"
#include <algorithm>
#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::plan::superball_sampler_t, prx::plan::sampler_t)

namespace prx
{
    using namespace util;
    namespace plan
    {        

        void superball_sampler_t::sample(const space_t* space, space_point_t* point)
        {
            // if(space->get_dimension()==12)
            // {
            //     // int number_of_random = uniform_int_random(1,8);
            //     // for(int i=0;i<200;i++)
            //     // {
            //     //     int i1 = uniform_int_random(0,11);
            //     //     int i2 = uniform_int_random(0,11);
            //     //     std::swap(indices[i1],indices[i2]);
            //     // }
            //     // double val = space->get_bounds()[indices[0]]->uniform_random_bounds();
            //     for(int i=0;i<12;i++)
            //     {
            //         // point->at(indices[i]) = val;//space->get_bounds()[indices[i]]->uniform_random_bounds();
            //         point->at(i) = (uniform_random()<.5?space->get_bounds()[0]->get_upper_bound():space->get_bounds()[0]->get_lower_bound());  //space->get_bounds()[indices[i]]->uniform_random_bounds();
            //     }
            // }
            // else
                space->uniform_sample(point);
        }

        void superball_sampler_t::sample_near(const space_t* space, space_point_t* near_point, std::vector<bounds_t*>& bounds, space_point_t* point)
        {
            space->uniform_sample_near(point, near_point, bounds);
        }

    }
}