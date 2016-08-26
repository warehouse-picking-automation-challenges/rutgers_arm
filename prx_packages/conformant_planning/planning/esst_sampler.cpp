/**
 * @file esst_sampler.cpp 
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
#include <algorithm>

#include "planning/esst_sampler.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "utilities/particle_space.hpp"

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::packages::conformant::esst_sampler_t, prx::plan::sampler_t)

namespace prx
{
    using namespace util;
    using namespace plan;

    namespace packages
    {        
        namespace conformant
        {
            void esst_sampler_t::sample(const space_t* space, space_point_t* point)
            {
                const particle_space_t* p_space = dynamic_cast< const particle_space_t*>(space);
                if(p_space!=NULL)
                {
                    const space_t* s_space = p_space->get_point_space();

                    particle_point_t* particle_set = dynamic_cast<particle_point_t*>(point);

                    s_space->uniform_sample(particle_set->links[0]);
                    for(unsigned i=1;i<p_space->get_num_particles();i++)
                    {
                        p_space->copy_particle(i,0,point);
                    }

                }
                else
                {

                    std::vector<bounds_t*> bounds = space->get_bounds();

                    for (int i = 0; i < bounds.size(); ++i)
                    {
                        double offset = .9*(bounds[i]->get_upper_bound()-bounds[i]->get_lower_bound());
                        int val = uniform_int_random(0,2);
                        if(val==0)
                        {
                            point->at(i) =  uniform_random(bounds[i]->get_lower_bound(),bounds[i]->get_upper_bound()-offset);
                        }
                        else if(val==1)
                        {
                            point->at(i) =  uniform_random(bounds[i]->get_lower_bound(),bounds[i]->get_upper_bound());
                        }
                        else
                        {
                            point->at(i) =  uniform_random(bounds[i]->get_lower_bound()+offset,bounds[i]->get_upper_bound());
                        }
                    }

                    space->uniform_sample(point);
                }
            }

            void esst_sampler_t::sample_near(const space_t* space, space_point_t* near_point, std::vector<bounds_t*>& bounds, space_point_t* point)
            {
                sample(space,point);
            }
        }
    }
}