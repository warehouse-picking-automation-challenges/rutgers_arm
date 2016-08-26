/**
 * @file behavior_controller.cpp
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

#include "simulation/controllers/behavior_controller.hpp"
#include "simulation/controllers/path_follow_controller.hpp"
#include "simulation/structures/origin.hpp"
#include "simulation/structures/OD_info.hpp"
#include "simulation/structures/queue.hpp"

#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/utilities/heuristic_search/astar_search.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/range/adaptor/map.hpp>


PLUGINLIB_EXPORT_CLASS(prx::packages::crowd::behavior_controller_t, prx::sim::system_t);

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {
        namespace crowd
        {
            double behavior_controller_t::cc_time = 0;
            double behavior_controller_t::tot_time = 0;
            unsigned behavior_controller_t::cc_calls = 0;
            
            behavior_controller_t::behavior_controller_t() : simple_controller_t()
            {
                is_leaving = false;
                is_evacuating = false;
                duration = 0;
                goal_origin = NULL;
                curr_region = NULL;
                try_attractors = 0;
                spare_time_thres = 1000;
                frames_to_leave = 0;
                queue_points.resize(3);
                current_position.resize(3);
                attractor_point.resize(3);
                
                queue = NULL;
                index_in_queue = -1;
                waits_in_queue = false;
                checked_queue = false;
                time_for_departure = false;
                queue_orientation = 0;
                lookup_x = 0;
                lookup_y = 0;
            }

            behavior_controller_t::~behavior_controller_t()
            {
                output_control_space->free_point(goal_point);
                queue_points.clear();
            }

            void behavior_controller_t::init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader)
            {
                //First, initialize everybody below
                simple_controller_t::init( reader, template_reader );

                //Then, underneath us should be a path follower, so it should be able to expose an appropriate space
                system_ptr_t child_system = subsystems.begin()->second;
                follower = dynamic_cast< path_follow_controller_t* >( child_system.get() );

                PRX_ASSERT( follower != NULL );

                goal_point = output_control_space->alloc_point();
                

                std::vector< double > goal_vec(3);
                if( parameters::has_attribute("goal", reader, template_reader) )
                {
                    goal_vec = parameters::get_attribute_as< std::vector<double> >("goal", reader, template_reader);
                }
                output_control_space->copy_vector_to_point(goal_vec, goal_point);

                gamma = parameters::get_attribute_as<double>("gamma", reader, template_reader, 0.95);
                try_attractors = parameters::get_attribute_as<double>("max_duration", reader, template_reader, 15);
            }

            void behavior_controller_t::setup(const OD_info_t& info, double MSEC_2_FRAMES)
            {
                desires = info.desires;
                this->desire_to_go = info.desire_to_go;
                this->avg_speed = info.max_speed/2;
                frames_to_leave = info.duration_frames;
                goal_origin = info.exit_origin;
                curr_region = NULL;
                is_leaving = false;
                duration = 0;
                msec2frames = MSEC_2_FRAMES;
                hindered = info.has_disability || info.has_luggage;
                index_in_queue = -1;
                active = true;

                queue = NULL;
                index_in_queue = -1;
                waits_in_queue = false;
            }

            void behavior_controller_t::propagate(const double simulation_step)
            {
                if( active )
                {
                    // frames_to_leave -= simulation_step;
                    // if(duration > 0)
                    //     duration = PRX_MAXIMUM(0, duration - simulation_step);

                    // output_control_space->copy_from_point( intermediate_target_point );

                    subsystems.begin()->second->propagate(simulation_step);
                }
            }
            
            void behavior_controller_t::compute_control()
            {
                if( active )
                {
                    stats_clock.reset();
                    
                    --frames_to_leave;

                    // if(follower->is_the_agent_lost())
                    // {

                    //     if(is_leaving)
                    //     {
                    //         PRX_PRINT(follower->get_id() << ")  Is lost and he is leaving? " << is_leaving, PRX_TEXT_RED);
                    //         follower->go_to_region(goal_origin, hindered);
                    //     }
                    //     else
                    //     {
                    //         PRX_PRINT(follower->get_id() << ")  Is lost and has to get new target: ", PRX_TEXT_BLUE);
                    //         follower->force_new_target();
                    //     }
                    // }
                    
                    if(!is_leaving)
                    {
                        if( follower->need_new_target() )
                        {
                            if(duration <= 0)
                            {
                                if(curr_region != NULL)
                                {
                                    //When an agent is leaving from the region.
                                    curr_region->decrease_occupancy();
                                    //PRX_PRINT(follower->get_id() << " decrease_occupancy  Curr occupancy:"<<curr_region->name, PRX_TEXT_CYAN);
                                    //PRX_PRINT(follower->get_id() << " decrease_occupancy for: "<<curr_region->name,  PRX_TEXT_CYAN);
                                }

                                const nav_node_t* curr_node = follower->get_nearest_node(); 
                                PRX_ASSERT(curr_node != NULL);
                                
                                const hash_t< region_t*, double > & region_dists = ( hindered ? curr_node->hindered_region_distances : curr_node->region_distances );
                                //dist = region_dists[goal_origin];
                                double best_value = goal_origin->influence * leaving_influence(region_dists[goal_origin]);
                                // PRX_DEBUG_COLOR("The exit will be " << goal_origin->name << "  dist:" << region_dists[goal_origin] << "   best_value:" << best_value, PRX_TEXT_MAGENTA);
                                is_leaving = true;
                                duration = 0;

                                curr_region = get_next_region_to_go(curr_node, best_value);
                                //In case that the influence of the goal_origin is higher the function will return NULL. 
                                if(curr_region == NULL)
                                    curr_region = goal_origin;
                                    
                                //We can use the occupancy of the origins as counter of how many people left from this exit. 
                                //curr_region->occupancy++;

                                follower->go_to_region( curr_region, hindered );
                                checked_queue = false;
                                // if(curr_region->is_queue_present())
                                // {
                                //     PRX_PRINT(follower->get_id() <<" GOING INTO QUEUE: "<<curr_region->get_queue_name(),PRX_TEXT_CYAN);
                                // }
                            }
                            else
                            {
                                // follower->reset_lost_counter();
                                // PRX_PRINT(follower->get_id() << ")  duration inside the attractor: " << duration, PRX_TEXT_LIGHTGRAY);
                                --duration;
                            }
                        }
                    }

                    if(curr_region != NULL && !is_evacuating)
                    {                      
                        if(!checked_queue && curr_region->is_queue_present() && follower->need_to_check_for_queue() )
                        {
                            if(follower->go_to_destination(curr_region->get_open_slot(this)))
                            {
                                follower->set_agent_mode( PRX_AGENT_MODE_GOING_TOWARDS_QUEUE );
                                //PRX_PRINT(" Calling OPEN agent:"<<follower->get_id() << " for region:"<<curr_region->name<<" with mode: " << follower->get_agent_mode() <<" frames_to_leave :"<<frames_to_leave, PRX_TEXT_LIGHTGRAY);
                            }
                            else
                            {
                                checked_queue = true;
                                curr_region->increase_occupancy();
                                if(curr_region->is_queue_present())
                                {
                                    //PRX_PRINT("CHEKD FOR QUEUE:"<<follower->get_id() << " increased occupancy "<<curr_region->name<<" Curr occupancy:"<<curr_region->occupancy, PRX_TEXT_BLUE);
                                }                                
                            }
                        }

                        //its if again because it could be that the agent does not have to travel to the open slot. 
                        //The agent can be already on this triangle and just need to reserve the point. 
                        if(!checked_queue && follower->need_to_reserve_queue_point())
                        {
                            //PRX_PRINT(" RESERVING SLOTS "<<follower->get_id() << " for region:"<<curr_region->name<<" with mode: " << follower->get_agent_mode() <<" frames_to_leave :"<<frames_to_leave, PRX_TEXT_CYAN);
                            if(follower->go_to_destination(curr_region->reserve_slot(this,queue_points,frames_to_leave, queue_orientation)))
                            {
                                follower->set_agent_mode( PRX_AGENT_MODE_GOING_TOWARDS_RESERVED_SLOT );
                                follower->set_queue_orientation(queue_orientation);
                            }
                        }

                        if(!checked_queue && follower->need_to_go_to_queue())
                        {
                            follower->go_to_point(queue_points);
                            follower->set_agent_mode( PRX_AGENT_MODE_IN_QUEUE );
                            PRX_PRINT("Agent:"<<follower->get_id() << " IN THE QUEUE for region:"<<curr_region->name<<" with  current occupancy:"<<curr_region->occupancy<<" queue_points:"<<queue_points[0]<<","<<queue_points[1]<<","<<queue_points[2]<<" frames_to_leave:"<<frames_to_leave, PRX_TEXT_CYAN); 
                        }
                    }
                    

                    if(waits_in_queue)
                    {
                        follower->get_current_position(current_position);                        
                        if(queue->is_first_agent(current_position, this))
                        {
                            if( (is_leaving && frames_to_leave <= 0) || curr_region->has_available_space() || time_for_departure )
                            {
                                //The pop_first_agent will release the agent from the queue. 
                                //It will call the function leave_queue() that is implemented here.
                                PRX_PRINT(follower->get_id()<<" is trying to ENTER region:"<<curr_region->name<<" with current occupancy:"<<curr_region->occupancy, PRX_TEXT_GREEN);                                
                                queue->pop_first_agent();
                                curr_region->increase_occupancy();
                                checked_queue = true;
                            }
                            // else if(curr_region->has_available_space())
                            // {
                            //     //The pop_first_agent will release the agent from the queue. 
                            //     //It will call the function leave_queue() that is implemented here.
                            //     PRX_PRINT(follower->get_id()<<" is trying to ENTER attractor:"<<curr_region->name<<" with current occupancy:"<<curr_region->occupancy, PRX_TEXT_GREEN);                                
                            //     queue->pop_first_agent();
                            //     curr_region->increase_occupancy();
                            //     checked_queue = true;
                            // }
                            // else if (time_for_departure)
                            // {

                            // }
                        }
                    }
                   // output_control_space->copy_from_point( target_point );                    
                    
                    cc_time += stats_clock.measure();
                    ++cc_calls;

                    subsystems.begin()->second->compute_control();
                    
                    tot_time += stats_clock.measure();                    
                }
            }

            region_t* behavior_controller_t::get_next_region_to_go(const nav_node_t* curr_node, double best_value)
            {
                double val = 0;
                double dist = 0;
                int tmp_duration;
                //Now, get the proper information
                const std::vector< attractor_info_t* > & attractor_dists = ( hindered ? curr_node->hindered_attractor_distances : curr_node->attractor_distances );                
                int attr_nr = attractor_dists.size()-1;
                int attr_index = 0;
                region_t* new_region = NULL;

                if(attr_nr > 0)
                {
                    for(int i = 0; i<try_attractors; ++i)
                    {
                        // PRX_PRINT("attr_nr: " << attr_nr, PRX_TEXT_LIGHTGRAY);
                        do
                        {
                            attr_index = uniform_int_random(0,attr_nr);
                            tmp_attractor = attractor_dists[attr_index]->attractor;
                        }while(curr_region != NULL && tmp_attractor->region_id == curr_region->region_id);
                        
                        dist = attractor_dists[attr_index]->dist;
                        // tmp_attractor->nodes[attractor_dists[attr_index]->doorway_index]->origin_distances[goal_origin]
                        double tmp_min = tmp_attractor->duration_distribution.first;
                        double tmp_max = tmp_attractor->duration_distribution.second;
                        // PRX_PRINT("[" << tmp_attractor->name << "]: {" << tmp_min << " , " << tmp_max << "}", PRX_TEXT_LIGHTGRAY);
                        tmp_duration = uniform_int_random(tmp_min, tmp_max);
                        if( hindered )
                        {
                            val = tmp_attractor->influence * attractor_influence(tmp_attractor, dist, tmp_attractor->nodes[attractor_dists[attr_index]->doorway_index]->hindered_region_distances[goal_origin], tmp_duration );
                        }
                        else
                        {
                            val = tmp_attractor->influence * attractor_influence(tmp_attractor, dist, tmp_attractor->nodes[attractor_dists[attr_index]->doorway_index]->region_distances[goal_origin], tmp_duration );
                        }

                        if(val > best_value)
                        {
                            best_value = val;
                            new_region = tmp_attractor;
                            duration = tmp_duration;
                            is_leaving = false;
                        }
                    }
                }

                return new_region;
            }


            void behavior_controller_t::link_search_primitives( undirected_graph_t* input_graph, astar_search_t* input_astar, distance_metric_t* input_metric )
            {
                // PRX_DEBUG_COLOR("link search primitives to behavior controller!!", PRX_TEXT_CYAN);
                nav_graph = input_graph;
                graph_search = input_astar;
                metric = input_metric;
                follower->link_search_primitives(input_graph, input_astar, input_metric);
            }

            bool behavior_controller_t::is_agent_leaving() const
            {
                // is_leaving = is_leaving || (frames_to_leave <= 0);
                return is_leaving; 
            }

            int behavior_controller_t::get_frames_to_leave() const
            {
                return frames_to_leave;
            }

            unsigned behavior_controller_t::get_id()
            {
                return follower->get_id();
            }

            void behavior_controller_t::set_goal( const util::space_point_t* new_goal )
            {
                
                output_control_space->copy_from_point( new_goal );
            }

            void behavior_controller_t::set_time_for_departure()
            {
                time_for_departure = true;
            }

            void behavior_controller_t::set_goal( const std::vector<double>& new_goal )
            {
                follower->go_to_point( new_goal );
            }

            void behavior_controller_t::in_queue(queue_t* queue_arg, int queue_position )
            {
                queue = queue_arg;
                index_in_queue = queue_position;
                waits_in_queue = true;
            }

            void behavior_controller_t::move_in_queue(int queue_position, const std::vector<double>& new_goal)
            {
                index_in_queue = queue_position;
                if(follower->is_agent_in_queue())
                {
                    follower->go_to_point(new_goal);
                }
                else
                {
                    queue_points[0] = new_goal[0];
                    queue_points[1] = new_goal[1];
                    queue_points[2] = new_goal[2];
                }
            }

            void behavior_controller_t::leave_queue()
            {
                queue = NULL;
                index_in_queue = -1;
                waits_in_queue = false;
                follower->leave_queue();
            }

            int behavior_controller_t::get_queue_id()
            {
                return queue != NULL? queue->queue_id:-1;
            }

            double behavior_controller_t::get_queue_orientation()
            {
                return queue_orientation;
            }

            double behavior_controller_t::get_lookup_x()
            {
                return lookup_x;
            }

            double behavior_controller_t::get_lookup_y()
            {
                return lookup_y;
            }

            void behavior_controller_t::set_region_lookup()
            {
                if(curr_region!=NULL)
                {
                    curr_region->get_point_to_go(attractor_point);
                    lookup_x = attractor_point[0];
                    lookup_y = attractor_point[1];
                }
            }

            void behavior_controller_t::set_lookup(double lookup_x, double lookup_y)
            {
                this->lookup_x = lookup_x;
                this->lookup_y = lookup_y;
            }

            void behavior_controller_t::evacuate(const std::vector<region_t*>& evacuation_points)
            {
                const nav_node_t* curr_node = follower->get_nearest_node();                 
                const hash_t< region_t*, double > & region_dists = ( hindered ? curr_node->hindered_region_distances : curr_node->region_distances );
                double best_dist = PRX_INFINITY;
                curr_region = NULL;
                // PRX_PRINT(" Agent " << follower->get_id() << " EVACUATE", PRX_TEXT_BROWN);
                // foreach(region_t* r, region_dists | boost::adaptors::map_keys)
                // {
                //     PRX_PRINT("region: " << r->name << ":  " << region_dists[r] << " meters    pointer:" << r , PRX_TEXT_BLUE);
                // }
                foreach(region_t* region, evacuation_points)
                {

                    if(region_dists[region] < best_dist)
                    {
                        curr_region = region;
                        best_dist = region_dists[region];
                    }
                }
                // PRX_PRINT("----------------------------------------------", PRX_TEXT_MAGENTA)
                PRX_ASSERT(curr_region != NULL);
                is_leaving = true;
                waits_in_queue = false;
                is_evacuating = true;
                follower->go_to_region( curr_region, hindered );
                leave_queue();
            }

            double behavior_controller_t::leaving_influence(double dist)
            {
                double spare_time = frames_to_leave - 1000*(dist/avg_speed)*msec2frames;
                // PRX_DEBUG_COLOR("spare time: " << spare_time << "     frames_to_leave: " << frames_to_leave << "    frames need:" << (1000*(dist/avg_speed)*msec2frames), PRX_TEXT_BLUE);
                // PRX_DEBUG_COLOR("dist: " << dist << "  avg_speed:" << avg_speed << "  desire: " << desire_to_go, PRX_TEXT_BLUE);
                if(spare_time <= 0)
                    return PRX_INFINITY;
                // PRX_DEBUG_COLOR("desire to go : " << desire_to_go << "   frames_to_leave:" << frames_to_leave << "   time_need:" << (dist/avg_speed) << "   ==  t:"<< t << "  dist:" << dist, PRX_TEXT_MAGENTA);
                // PRX_DEBUG_COLOR("gamma^dist " << (pow(gamma,dist)) << "   gamma^t " << (pow(gamma,t)), PRX_TEXT_MAGENTA);
                return desire_to_go * spare_time_thres/spare_time;
            }

            double behavior_controller_t::attractor_influence(region_t* region, double dist, double origin_dist, double duration)
            {
                double spare_time = frames_to_leave - ( 1000*msec2frames * ((origin_dist + dist)/avg_speed) + duration);
                // PRX_DEBUG_COLOR("spare time: " << spare_time << "     frames_to_leave: " << frames_to_leave << "    frames needs for attractor:" << ( 1000*msec2frames * ((dist)/avg_speed) + duration) << "    frames needs:" << ( 1000*msec2frames * ((origin_dist + dist)/avg_speed) + duration) << "   cap:" << region->capacity << "/" << region->max_capacity, PRX_TEXT_LIGHTGRAY);
                if(spare_time <= 0)
                    return -PRX_INFINITY;
                // PRX_DEBUG_COLOR("origin_dist:" << origin_dist << "   distance:" << dist << "   duration:" << duration, PRX_TEXT_BLUE);
                // PRX_DEBUG_COLOR("desire : " << disire << "   frames_to_leave:" << frames_to_leave << "   time_need:" << ( (origin_dist + dist)/avg_speed + duration) << "   ==  t:"<< t << "  dist:" << (origin_dist + dist), PRX_TEXT_BLUE);
                // PRX_DEBUG_COLOR("gamma^dist " << (pow(gamma,dist)) << "   gamma^t " << (pow(gamma,t)) <<  "  res: " << ( pow(gamma,dist) + pow(gamma,t)), PRX_TEXT_BLUE);
                return desires[region->type] * ((-spare_time_thres / (spare_time + spare_time_thres))+1) * capacity_influence(region);
            }

            double behavior_controller_t::capacity_influence(region_t* region)
            {
                if(region->occupancy <= region->max_capacity)
                    return (1 - (0.8/region->max_capacity)*region->occupancy );
                return 0.2/(region->occupancy - region->max_capacity + 1);
            }

        }
    }
}