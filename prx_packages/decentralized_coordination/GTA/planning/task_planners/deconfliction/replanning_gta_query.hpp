/**
 * @file replanning_gta_query.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Andrew Kimmel, Andrew Dobson, Athanasios Krontiris, Zakary Littlefield,  Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#pragma once
#ifndef PRX_REPLANNING_GTA_QUERY_HPP
#define	PRX_REPLANNING_GTA_QUERY_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx_simulation/plant_locations_msg.h"

namespace prx
{
    namespace util
    {
        class radial_goal_region_t;
    }
    
    namespace packages
    {
        namespace gta
        {
            class replanning_gta_query_t : public plan::motion_planning_query_t
            {

                public:
                    replanning_gta_query_t();
                    replanning_gta_query_t(util::space_t* state_space, util::space_t* control_space, const std::string& my_plant, int n_limit = PRX_INFINITY);
                    ~replanning_gta_query_t();

                    void print();
                    void process_plant_locations_callback(const prx_simulation::plant_locations_msg& msg);
                    virtual void link_spaces(const util::space_t* state_space, const util::space_t* control_space);
                    
                    // Sensing distance of 0 implies full knowledge
                    void distance_limited_neighborhood(std::vector<sim::state_t*>& new_neighbor_states, double sensing_distance);
                    
                    void set_neighborhood_limit(int limit);

                    sim::state_t* my_true_previous_state;
                    sim::state_t* my_current_state;

                    util::hash_t<std::string, sim::state_t*, util::string_hash> previous_neighbor_states;
                    util::hash_t<std::string, sim::state_t*, util::string_hash> current_neighbor_states;

                    // Neighbors that appear in both previous and current neighbor states
                    std::vector<std::string> valid_neighbors;
                    // Neighbors that appear in the current state (not necessarily valid)
                    std::vector<std::string> all_neighbors;

                private:
                    
                    bool limit_neighborhood(bool simple_prune = false);
                    
                    util::hash_t<std::string, int, util::string_hash> consecutive_appearances;
                    util::hash_t<std::string, bool> reset_neighbors;
                    std::string my_plant;
                    bool once;
                    int time_step;
                    unsigned neighborhood_limit;

            };
        }
    }
}
        
        
#endif