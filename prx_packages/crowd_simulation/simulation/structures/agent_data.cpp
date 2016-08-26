/**
 * @file agent_data.cpp
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

#include "simulation/structures/agent_data.hpp"

namespace prx
{
    using namespace util;

    namespace packages
    {
        namespace crowd
        {
            agent_data_t::agent_data_t()
            {
                agent_type = 0;
                hindered = false;
                just_spawned = false;
            }

            agent_data_t::~agent_data_t()
            {
            }

            bool agent_data_t::is_active()
            {
                return plant->is_active();
            }

            void agent_data_t::set_active( bool flag )
            {
                behavior_controller->set_active( flag, "" );
                path_controller->set_active( flag, "" );
                VO_controller->set_active( flag, "" );
                plant->set_active( flag, "" );                
            }

            int agent_data_t::get_queue_id() 
            {
                return behavior_controller->get_queue_id();
            }
            
	    	std::vector<double> agent_data_t::get_position()
    		{
    			vector<double> tmp;tmp.resize(3);
    			plant->get_current_position(tmp);
    			return tmp;
    		}
        }
    }
}

