/**
 * @file attractor.hpp
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

#ifndef PRX_ATTRACTOR_HPP
#define PRX_ATTRACTOR_HPP

#include "prx/utilities/math/configurations/vector.hpp"
#include "simulation/structures/region.hpp"
#include "simulation/structures/attractor_influence.hpp"

#include <vector>

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
    }

    namespace packages
    {
        namespace crowd
        {
            class nav_node_t;
            class behavior_controller_t;
            
            class attractor_t : public region_t
            {

              public:
                attractor_t( util::space_t* input_space, unsigned in_type, unsigned id );            

                ~attractor_t();

                void init(const util::parameter_reader_t * const reader, double MSEC_2_FRAMES);
                void init_from_file( const YAML::Node& node, double MSEC_2_FRAMES);

                virtual const util::space_point_t* get_point_to_go();

                const nav_node_t* get_open_slot(behavior_controller_t* agent);
                
                const nav_node_t* reserve_slot(behavior_controller_t* agent, std::vector<double>& queue_points, int frames_to_leave, double &orientation);

                void increase_occupancy();

                void decrease_occupancy();

                bool has_available_space();

                // Returns an attractor point
                void get_point_to_go(std::vector<double>& attractor_point);
            };
        }
    }
}

#endif //PRX_RAMP_HPP

