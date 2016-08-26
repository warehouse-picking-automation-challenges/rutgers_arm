/**
 * @file rally_car_sampler.cpp 
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


#include "prx/planning/modules/samplers/rally_car_sampler.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/math/configurations/bounds.hpp"
#include "prx/utilities/definitions/random.hpp"

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::plan::rally_car_sampler_t, prx::plan::sampler_t)
        
namespace prx
{
    using namespace util;
    namespace plan
    {
        void rally_car_sampler_t::sample(const space_t* space,space_point_t* point)
        {
            if(space->get_space_name()=="RallyCar")
            {
                space->uniform_sample(point);
                //compute the angle that is created
                double theta = atan2(point->at(1),point->at(0));
                theta+=PRX_PI/2;
                theta = uniform_random(theta-PRX_PI/6.0,theta+PRX_PI/6.0);
                if(theta > 2*PRX_PI)
                    theta -= 2*PRX_PI;
                point->at(2) = theta;
            }
            else
            {
                space->uniform_sample(point);
            }
        }

        void rally_car_sampler_t::sample_near(const space_t* space, space_point_t* near_point, std::vector<bounds_t*>& bounds, space_point_t* point)
        {
            //not used right now.
        //    space->uniform_sample_near(point,near_point,bounds);
        }
    }
}