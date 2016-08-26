/**
 * @file node_sensing_info.hpp
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

#ifndef PRX_NODE_SENSING_INFO_HPP
#define	PRX_NODE_SENSING_INFO_HPP

#include "simulation/controllers/VO_structure/neighbor.hpp"
#include "simulation/controllers/path_follow_controller.hpp"

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/config.hpp"

namespace prx
{
    namespace sim
    {
        class plant_t;
    }

    namespace packages
    {
        namespace crowd
        {
            class navigation_graph_sensor_t;

            class node_sensing_info_t : public sim::sensing_info_t
            {
            protected:
                navigation_graph_sensor_t* nav_sensor;

            public:
                node_sensing_info_t() ;
                virtual ~node_sensing_info_t();

                /**
                 * Initializes from the given parameters.
                 *
                 * @brief Initializes from the given parameters.
                 */
                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

                virtual void set_sensors(const std::vector<sim::sensor_t*>& sensors);

                bool updates();

                // The conversion function to turn sensing data into the controller's info
                virtual void update_info();
                // Used to access the info?
                virtual void get_info();

                std::pair< const nav_node_t*, const nav_node_t* > near_nodes;

                util::sys_clock_t stats_clock;

                static double update_time;
                static unsigned update_calls;

                unsigned agent_index;
            };
        }
    }
}

#endif

