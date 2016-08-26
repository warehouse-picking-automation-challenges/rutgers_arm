/**
 * @file origin.hpp
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

#ifndef PRX_ORIGIN_HPP
#define PRX_ORIGIN_HPP

#include "simulation/structures/region.hpp"

#include "prx/utilities/math/configurations/vector.hpp"

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

            class nav_graph_t;

            class origin_t : public region_t
            {
              public:
                origin_t( util::space_t* input_space, unsigned in_type, unsigned id );
                ~origin_t();

                void init(const util::parameter_reader_t * const reader, double MSEC_2_FRAMES);
                void init_from_file( const YAML::Node& node, double MSEC_2_FRAMES);

                void setup(std::string in_name, unsigned in_type, const std::vector< double >& in_doorways, const std::vector< double >& triangles, const std::vector< unsigned >& input_schedule);
                
                void set_schedule( const std::vector< unsigned >& input_schedule );
                bool ready_to_spawn( double sim_time );      

                const util::space_point_t* get_spawn_point(unsigned frame_id);
                
                const std::vector< util::space_point_t* >& get_next_spawn_points();

                virtual const util::space_point_t* get_point_to_go();
                
              protected:
                unsigned nr_points;
                std::vector< std::pair<unsigned, util::space_point_t*> > spawn_points;
                util::space_point_t* tmp_point;

                //The total number of agents this origin is waiting to spawn
                unsigned agents_to_spawn;
                
                //Buffer for the next spawn rate change
                unsigned next_spawn_rate_change;
                
                //The spawn rate schedule for this origin
                std::vector< std::pair< unsigned, unsigned > > schedule;
                unsigned schedule_index;
                
                //Spawn points for the agents
                std::vector< util::space_point_t* > next_spawns;
                util::space_t* nav_space;
            };
        }
    }
}

#endif //PRX_RAMP_HPP

