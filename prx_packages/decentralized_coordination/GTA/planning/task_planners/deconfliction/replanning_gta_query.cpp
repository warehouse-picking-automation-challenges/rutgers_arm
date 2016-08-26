/**
 * @file gta_query.cpp
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


#include "planning/task_planners/deconfliction/replanning_gta_query.hpp"
#include "prx/utilities/goals/radial_goal_region.hpp"


#include <boost/range/adaptor/map.hpp>
#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS( prx::packages::gta::replanning_gta_query_t, prx::plan::query_t)
namespace prx
{
    using namespace util;
    using namespace plan;
    
    namespace packages
    {

        namespace gta
        {

            replanning_gta_query_t::replanning_gta_query_t()
            {
            }

            replanning_gta_query_t::replanning_gta_query_t(space_t* state_space, space_t* control_space,const std::string& my_plant, int n_limit)
            {
                this->state_space = state_space;
                this->control_space = control_space;
                start_state = this->state_space->alloc_point();
                this->my_plant = my_plant;
                my_true_previous_state = this->state_space->alloc_point();
                my_current_state = this->state_space->alloc_point();
                once = false;
                neighborhood_limit = n_limit;
            }


            replanning_gta_query_t::~replanning_gta_query_t()
            {
                state_space->free_point(my_true_previous_state);
                state_space->free_point(my_current_state);
            }

            void replanning_gta_query_t::link_spaces(const space_t* state_space, const space_t* control_space)
            {
            //    PRX_WARN_S ("Link da spaces");
                this->state_space = state_space;
                this->control_space = control_space;
                goal->link_space(state_space);
                plan.link_control_space(control_space);
                plan.link_state_space(state_space);
                path.link_space(state_space);

                my_true_previous_state = this->state_space->alloc_point();
            }

            void replanning_gta_query_t::process_plant_locations_callback(const prx_simulation::plant_locations_msg& msg)
            {
                PRX_ASSERT(state_space);
                // We start with no valid neighbors
                valid_neighbors.clear();
                all_neighbors.clear();

                // Set reset flags (to ensure we have 2 occurences in a row
                foreach (std::string reset_path, reset_neighbors | boost::adaptors::map_keys )
                {
                    reset_neighbors[reset_path] = true;
                }

                // Go over the plant paths sent
                for (unsigned i = 0; i < msg.plant_paths.size(); i++)
                {
                    std::string plant_path = msg.plant_paths[i];
                    bool valid = false;
                    if (plant_path != my_plant)
                    {
                        // If this is the first time we've seen it, then allocate memory
                        if (previous_neighbor_states.find(plant_path) == previous_neighbor_states.end())
                        {
                            previous_neighbor_states[plant_path] = state_space->alloc_point();
                        }
                        if (current_neighbor_states.find(plant_path) == current_neighbor_states.end())
                        {
                            current_neighbor_states[plant_path] = state_space->alloc_point();
                        }
                        if (consecutive_appearances.find(plant_path) == consecutive_appearances.end())
                        {
                            consecutive_appearances[plant_path] = 0;
                        }

                        // Increment the number of times we've seen the plant, and make sure it can't be reset
                        consecutive_appearances[plant_path]++;
                        reset_neighbors[plant_path] = false;

                        // If we've seen it more than once, we know it has a valid current state
                        if (consecutive_appearances[plant_path] > 1)
                        {
//                            PRX_DEBUG_COLOR ("Old Neighbor Previous point: " << state_space->print_point(previous_neighbor_states[plant_path]), PRX_TEXT_CYAN);
//                            PRX_DEBUG_COLOR ("Old Neighbor Current point : " << state_space->print_point(current_neighbor_states[plant_path]), PRX_TEXT_CYAN);
//                            PRX_DEBUG_COLOR( "Distance: " << this->state_space->distance(previous_neighbor_states[plant_path],current_neighbor_states[plant_path]), PRX_TEXT_CYAN);
                            state_space->copy_point(previous_neighbor_states[plant_path], current_neighbor_states[plant_path]);
                            if (consecutive_appearances[plant_path] > 2)
                                valid = true;

                        }


                        // Set my current neighbor state from ground truth
                        state_space->set_from_vector(msg.plant_states[i].elements,current_neighbor_states[plant_path]);
                        PRX_DEBUG_COLOR ("NEW Neighbor Current point " << state_space->print_point(current_neighbor_states[plant_path]), PRX_TEXT_RED);
                        // If it's valid, add it to the list
                        if (valid)
                        {
                            valid_neighbors.push_back(plant_path);
                        }
                        all_neighbors.push_back(plant_path);
                    }
                    else
                    {
                        if (once)
                        {
                            state_space->copy_point(my_true_previous_state, my_current_state);
                            state_space->set_from_vector(msg.plant_states[i].elements, my_current_state);
                            PRX_DEBUG_COLOR ("Set my previous state to: " << state_space->print_point(my_true_previous_state), PRX_TEXT_MAGENTA);
                            PRX_DEBUG_COLOR ("Set my current state to: " << state_space->print_point(my_current_state), PRX_TEXT_MAGENTA);
                            PRX_DEBUG_COLOR ("Distance: " << this->state_space->distance(my_true_previous_state,my_current_state), PRX_TEXT_MAGENTA);
                        }
                        else
                        {
                            state_space->set_from_vector(msg.plant_states[i].elements, my_current_state);
                            state_space->copy_point(my_true_previous_state, my_current_state);
                            once = true;
                        }
                    }
                }
                foreach (std::string reset_path, reset_neighbors | boost::adaptors::map_keys )
                {
                    if (reset_neighbors[reset_path])
                    {
                        consecutive_appearances[reset_path] = 0;
                    }
                }
                
                if (all_neighbors.size() > neighborhood_limit)
                {
                    limit_neighborhood();
                }

            }

            void replanning_gta_query_t::set_neighborhood_limit(int limit)
            {
                neighborhood_limit = limit;
            }
            
            void replanning_gta_query_t::print()
            {

            }
            
            void replanning_gta_query_t::distance_limited_neighborhood(std::vector<sim::state_t*>& new_neighbor_states, double sensing_distance)
            {
                if (sensing_distance > 0)
                {
                    for (unsigned i = 0; i < valid_neighbors.size(); i++)
                    {
                        sim::state_t* check_state = current_neighbor_states[valid_neighbors[i]];
                        double dist = this->state_space->distance(this->my_current_state, check_state);
                        if (dist <= sensing_distance)
                        {
                            PRX_PRINT ("Added neighbor: " << state_space->print_point(check_state), PRX_TEXT_CYAN);
                            new_neighbor_states.push_back(check_state);
                        }
                    }
                }
                else
                {
                    for (unsigned i = 0; i < valid_neighbors.size(); i++)
                    {
                        sim::state_t* check_state = current_neighbor_states[valid_neighbors[i]];
                        new_neighbor_states.push_back(check_state);
                    }
                }
            }
            
            bool replanning_gta_query_t::limit_neighborhood(bool simple_prune)
            {
                /** Prunes the neighborhood: the criterion for this can be chosen */
                
                if (simple_prune)
                {
                    /** Simple prune */
                    while (valid_neighbors.size() > neighborhood_limit)
                    {
                        valid_neighbors.pop_back();
                    }
                }
                else
                {
                    /** Distance based prune */
                    std::vector<std::string> limited_neighborhood;
                    
                    // TODO
                }
                
                return true;
                
            }
        }
    }
}
