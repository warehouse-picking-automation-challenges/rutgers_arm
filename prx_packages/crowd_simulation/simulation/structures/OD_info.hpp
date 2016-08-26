/**
 * @file OD_info.hpp
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

#ifndef PRX_OD_INFO_HPP
#define PRX_OD_INFO_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/boost/hash.hpp"

#include <vector>
#include <yaml-cpp/yaml.h>

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
            class origin_t;
            class world_structure_t;

            class OD_info_t
            {

              public:

                OD_info_t(origin_t* in_entrance, origin_t* in_exit, unsigned in_agent_type, int in_arrival_frame, int in_duration_frames, int in_departure_frame);

                OD_info_t(world_structure_t* world_structure, unsigned in_agent_type, const util::hash_t<std::string, unsigned>& attractor_index, double MSEC_2_FRAMES, const util::parameter_reader_t * const reader);

                OD_info_t(world_structure_t* world_structure, unsigned in_agent_type, const util::hash_t<std::string, unsigned>& attractor_index, double MSEC_2_FRAMES, const YAML::Node& node);

                OD_info_t(world_structure_t* world_structure, const util::hash_t< std::string, unsigned >& agent_index, const util::hash_t<std::string, unsigned>& attractor_index, double MSEC_2_FRAMES, std::ifstream& fin);

                std::string print();

                origin_t* entrance;
                origin_t* exit_origin;
                unsigned agent_type;
                int arrival_frame;
                int duration_frames;
                int departure_frame;
                bool has_luggage;
                bool has_disability;
                double max_speed;
                double desire_to_go;
                std::vector<double> desires;
            };

            bool operator<(const OD_info_t &od1, const OD_info_t &od2);
        }
    }
}

#endif

