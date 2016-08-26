/**
 * @file temporal_validity_checker.cpp 
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


#include "prx/planning/modules/validity_checkers/temporal_validity_checker.hpp"

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::plan::temporal_validity_checker_t, prx::plan::validity_checker_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace plan
    {

        bool temporal_validity_checker_t::is_valid(const state_t* point)
        {
            return world_model->valid_state(point);
        }

        bool temporal_validity_checker_t::is_valid(const trajectory_t& input)
        {
            bool valid = !input.in_collision();
            if(valid)
            {
                for (trajectory_t::const_iterator iter = input.begin(); iter != input.end(); ++iter)
                {
                    if(valid)
                    {
                        valid = (world_model->get_state_space()->satisfies_bounds(*iter,false));
                    }
                    if(valid)
                    {
                        double x,y,z;
                        x = (*iter)->at(0);
                        y = (*iter)->at(1);
                        z = (*iter)->at(2);
                        double val = sqrt((x-30)*(x-30)+(y-30)*(y-30));
                        if( val < 15 )
                        {
                            valid = false;
                        }
                    }
                }
            }
            return valid;
        }

    }
}