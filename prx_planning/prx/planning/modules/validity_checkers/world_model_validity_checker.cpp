/**
 * @file world_model_validity_checker.cpp
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


#include "prx/planning/modules/validity_checkers/world_model_validity_checker.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::plan::world_model_validity_checker_t, prx::plan::validity_checker_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace plan
    {

        bool world_model_validity_checker_t::is_valid(const state_t* point)
        {
            bool valid = world_model->valid_state(point);      
            // if (!valid)
            // {
            //     sim::collision_list_t* colliding_bodies = world_model->get_colliding_bodies(point);
            //     foreach(collision_pair_t cp, colliding_bodies->get_body_pairs())
            //     {
            //         PRX_PRINT("COLLISION DETECTED BETWEEN : "<<( cp.first )<<"  and "<<( cp.second ), PRX_TEXT_RED);
            //     }  
            // }
            return valid;    
        }

    }
}
