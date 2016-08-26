/**
 * @file agent_data.hpp
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

#ifndef PRX_AGENT_DATA_HPP
#define PRX_AGENT_DATA_HPP

#include "prx/utilities/spaces/space.hpp"

#include "simulation/controllers/behavior_controller.hpp"
#include "simulation/controllers/path_follow_controller.hpp"
#include "simulation/controllers/collision_avoidance_controller.hpp"
#include "simulation/plants/pedestrian.hpp"

#include <vector>

namespace prx
{
    namespace packages
    {
        namespace crowd
        {

            class agent_data_t
            {
              public:
                agent_data_t();
                ~agent_data_t();


                bool is_active();
                
                void set_active( bool flag );

                int get_queue_id();

                behavior_controller_t* behavior_controller;
                path_follow_controller_t* path_controller;
                collision_avoidance_controller_t* VO_controller;
                pedestrian_t* plant;

                bool just_spawned;

                unsigned agent_id;
                int agent_type;

                bool has_luggage;

                /** @brief Either they have luggage or disability */
                bool hindered;
		
        		std::vector<double> get_position();
        		//Sean: need a function to get its current position. points_t get_position();
        		// in queue class, add_agent and pop_first_agent need to move agents. but the function that can move agents does not exsits here.
              protected:

            };
        }
    }
}
#define agent_t agent_data_t
#endif //PRX_AGENT_DATA_HPP

