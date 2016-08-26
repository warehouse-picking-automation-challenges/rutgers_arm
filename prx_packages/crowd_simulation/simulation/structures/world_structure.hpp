/**
 * @file world_structure.hpp
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

#ifndef PRX_WORLD_STRUCTURE_HPP
#define PRX_WORLD_STRUCTURE_HPP

#include "simulation/structures/ramp.hpp"
#include "simulation/structures/queue.hpp"
#include "simulation/structures/origin.hpp"
#include "simulation/structures/elevator.hpp"
#include "simulation/structures/attractor.hpp"

#include "simulation/controllers/path_follow_controller.hpp"

#include "simulation/graph/nav_node.hpp"

#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/utilities/definitions/defs.hpp"

#include <boost/assign/list_of.hpp>

#include <vector>

namespace prx
{
    namespace util
    {
        class distance_metric_t;
        class parameter_reader_t;
    }

    namespace packages
    {
        namespace crowd
        {

            class ramp_query_node_t : public util::abstract_node_t
            {
            public:
                ramp_query_node_t()
                {
                    ramp = NULL;
                }

                ~ramp_query_node_t()
                {
                }

                ramp_t* ramp;
            };

            class world_structure_t
            {
              public:
                world_structure_t();
                ~world_structure_t();
                void init( const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL );
                void init_regions( const util::parameter_reader_t * reader, const util::hash_t<std::string, unsigned>& origin_index, const util::hash_t<std::string, unsigned>& attractor_index, double MSEC_2_FRAMES);
                void init_regions_from_file( const YAML::Node& node, const util::hash_t<std::string, unsigned>& origin_index, const util::hash_t<std::string, unsigned>& attractor_index, double MSEC_2_FRAMES);

                void frame();

                void add_elevator( double min_x, double max_x, double min_y, double max_y, const std::pair< double, const nav_node_t* >& stop );
                void add_ramp( ramp_t* input_ramp );
                void fix_state( util::space_point_t* agent_state, const path_follow_controller_t* path_cont );

                void add_queue();

                const std::vector< elevator_t* >& get_elevators() const;

                const std::vector< queue_t* >& get_queues() const;
                const std::vector< origin_t* >& get_origins() const;
                const std::vector< attractor_t* >& get_attractors() const;
                const std::vector< region_t* >& get_all_regions();

                origin_t* get_origin(std::string name) const;
                attractor_t* get_attractor(std::string name) const;
                region_t* get_region( std::string name ) const;

                void print();

              protected:
                //Space memory
                util::space_t* navigation_space;
                util::vector_t test_point;
                
                // O/D information structures
                std::vector< origin_t* > origins;
                std::vector< attractor_t* > attractors;

                std::vector< region_t* > regions;
                
                //Elevator things
                std::vector< elevator_t* > elevators;
                
                //Ramp things
                std::vector< ramp_t* > ramps;

                //Queue information
                std::vector< queue_t* > all_queues;

              private:
                double _Nx;
                double _Ny;
                double _Nz;
            };
        }
    }
}

#endif //PRX_WORLD_STRUCTURE_HPP
