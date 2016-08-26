/**
 * @file attractor.cpp
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

#include "simulation/structures/attractor.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "simulation/structures/queue.hpp"
#include "simulation/controllers/behavior_controller.hpp"


namespace prx
{
    using namespace util;

    namespace packages
    {
        namespace crowd
        {

            attractor_t::attractor_t( space_t* input_space, unsigned in_type, unsigned id) : region_t( input_space, in_type, id )
            {
            }

            attractor_t::~attractor_t()
            {
                
            }

            void attractor_t::init(const util::parameter_reader_t * const reader, double MSEC_2_FRAMES)
            {
                region_t::init(reader,MSEC_2_FRAMES);
                max_capacity = reader->get_attribute_as<double>("Max_Capacity",20);
                std::vector< double > distribution = reader->get_attribute_as< std::vector< double > >("Duration");
                PRX_ASSERT(distribution.size() == 2);
                // duration_distribution = std::make_pair(crowd_global::MSEC_2_FRAMES * distribution[0], crowd_global::MSEC_2_FRAMES * distribution[1]);
                duration_distribution = std::make_pair(MSEC_2_FRAMES * distribution[0], MSEC_2_FRAMES * distribution[1]);                
            }

            void attractor_t::init_from_file( const YAML::Node& node, double MSEC_2_FRAMES)
            {
                region_t::init_from_file(node, MSEC_2_FRAMES);
                max_capacity = node["Max_Capacity"].as<double>();
                duration_distribution = std::make_pair(MSEC_2_FRAMES * node["Duration"][0].as<double>(), MSEC_2_FRAMES * node["Duration"][1].as<double>());                
            }

            const util::space_point_t* attractor_t::get_point_to_go()
            {
                //TODO: return something random inside or from the queue or something from the queue.
                return points[uniform_int_random(0,nr_inside_points)];
            }


            void attractor_t::get_point_to_go(std::vector<double>& attractor_point)
            {
                attractor_point = points[uniform_int_random(0,nr_inside_points)]->memory;
            }

            const nav_node_t* attractor_t::get_open_slot(behavior_controller_t* agent)
            {                
                if(queue!=NULL && ( queue->available_slot_index!=0 || this->max_capacity <= this->occupancy ))
                {
                    return queue->get_open_slot(agent);
                }
                return NULL;
            }

            const nav_node_t* attractor_t::reserve_slot(behavior_controller_t* agent, std::vector<double>& queue_points, int frames_to_leave, double &orientation)
            {
                if(queue!=NULL && this->max_capacity <= this->occupancy)
                {
                    return queue->reserve_slot(agent,queue_points,PRX_INFINITY, orientation);
                }
                return NULL;
            }

            void attractor_t::increase_occupancy()
            {
                ++this->occupancy;
            }

            bool attractor_t::has_available_space()
            {
               return this->max_capacity > this->occupancy;
            }

            void attractor_t::decrease_occupancy()
            {
                
                if(this->occupancy>0)
                {
                    --this->occupancy;
                }
            }
        }
    }
}

