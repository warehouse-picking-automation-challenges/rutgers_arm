/**
 * @file region.hpp
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

#ifndef PRX_REGION_HPP
#define PRX_REGION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/definitions/random.hpp"

#include <deque>
#include <vector>
#include <algorithm> 
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

            class departure_t;
            class queue_t;
            class nav_node_t;
            class behavior_controller_t;

            class departure_t
            {
            public:
                double amount;
                unsigned frames; //milliseconds
                departure_t()
                {
                }

                departure_t(unsigned frames, double amount)
                {
                    this->amount = amount;
                    this->frames = frames;
                }
            };

            class region_t
            {

              public:
                region_t( util::space_t* input_space, unsigned in_type, unsigned id );
                virtual ~region_t();

                void init(const util::parameter_reader_t * const reader, double MSEC_2_FRAMES);
                void init_from_file( const YAML::Node& node, double MSEC_2_FRAMES );

                const std::vector< util::space_point_t* >& get_doorways();

                const std::vector< util::space_point_t* >& get_inside_points();

                //This is deprecated for now. 
               // virtual const util::space_point_t* get_point_to_go() = 0;

                virtual bool has_available_space();

                virtual void decrease_occupancy();

                virtual void increase_occupancy();

                //Check if it  is time for departure
                void check_departure(int frame_id);

                std::string name;
                unsigned region_id;
                unsigned type;
                double influence;
                unsigned occupancy;
                unsigned max_capacity;
                //Duration is counted in frames 
                std::pair<int, int> duration_distribution;

                std::vector< util::space_point_t* > points;

                //The navigation nodes for the doorways.
                std::vector< nav_node_t* > nodes;
                std::vector< util::space_point_t* > doorways;

                // Aditya's Code
                // Check if agent is near by the queue
                bool need_to_check_for_queue(util::space_point_t* current_state);

                // Set Queue for the region
                void set_queue(queue_t* queue);

                // Check is queue is set for the region
                bool is_queue_present();

                // Test Method - To be removed
                std::string get_queue_name();

                virtual const nav_node_t* get_open_slot(behavior_controller_t* agent);

                virtual const nav_node_t* reserve_slot(behavior_controller_t* agent, std::vector<double>& queue_points, int frames_to_leave, double &orientation);

                double get_height() const;

                // Returns an attractor point
                void get_point_to_go(std::vector<double>& attractor_point);

                // Returns if this region is evacuation point or not.
                bool is_evacuation_point();

                // End
              protected:
                util::space_t* nav_space;
                queue_t* queue;
                unsigned nr_doorways;
                unsigned nr_inside_points;
                bool evacuation_point;
                unsigned nr_departures;
                std::deque< departure_t > departures;

                virtual void clear();
            };

            bool operator<(const departure_t &departure_1, const departure_t &departure_2); 
            void operator >> (const YAML::Node& node, std::vector<double>& vec);
        }
    }
}

#endif //PRX_RAMP_HPP

