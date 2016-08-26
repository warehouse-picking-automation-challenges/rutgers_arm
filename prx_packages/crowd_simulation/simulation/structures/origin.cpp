/**
 * @file origin.cpp
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

#include "simulation/structures/origin.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "simulation/structures/queue.hpp"

namespace prx
{
    using namespace util;

    namespace packages
    {
        namespace crowd
        {

            origin_t::origin_t( space_t* input_space, unsigned in_type, unsigned id ) : region_t( input_space, in_type, id)
            {
                nav_space = input_space;
                agents_to_spawn = 0;
                next_spawn_rate_change = 0;
                schedule_index = 0;
                schedule.push_back( std::pair< unsigned, unsigned >(0, 0) );
                tmp_point = nav_space->alloc_point();
            }

            origin_t::~origin_t()
            {
                nav_space->free_point(tmp_point);
            }

            void origin_t::init(const util::parameter_reader_t * const reader, double MSEC_2_FRAMES)
            {
                region_t::init(reader, MSEC_2_FRAMES);
                // std::vector<unsigned> input_schedule = reader->get_attribute_as< std::vector<unsigned> >("Arrivals");
                // PRX_ASSERT( input_schedule.size() %2 == 0 );
                // schedule.clear();
                // for( unsigned i=0; i<input_schedule.size(); i+=2 )
                // {
                //     schedule.push_back( std::pair< unsigned, unsigned >( input_schedule[i], input_schedule[i+1] ) );
                // }
                // if( schedule.size() > 1 )
                // {
                //     next_spawn_rate_change = schedule[1].first;
                // }

                nr_points = 0;
                foreach(space_point_t* point, points)
                {
                    spawn_points.push_back(std::make_pair(0,point));
                    ++nr_points;
                }
                //In order to have the correct size of points not the number of the points. 
                nr_points -= 2;

            }

            void origin_t::init_from_file( const YAML::Node& node, double MSEC_2_FRAMES)
            {
                region_t::init_from_file(node, MSEC_2_FRAMES);
                // std::vector<unsigned> input_schedule = reader->get_attribute_as< std::vector<unsigned> >("Arrivals");
                // PRX_ASSERT( input_schedule.size() %2 == 0 );
                // schedule.clear();
                // for( unsigned i=0; i<input_schedule.size(); i+=2 )
                // {
                //     schedule.push_back( std::pair< unsigned, unsigned >( input_schedule[i], input_schedule[i+1] ) );
                // }
                // if( schedule.size() > 1 )
                // {
                //     next_spawn_rate_change = schedule[1].first;
                // }

                nr_points = 0;
                foreach(space_point_t* point, points)
                {
                    spawn_points.push_back(std::make_pair(0,point));
                    ++nr_points;
                }
                //In order to have the correct size of points not the number of the points. 
                nr_points -= 2;
            }
            
            const util::space_point_t* origin_t::get_spawn_point(unsigned frame_id)
            {
                //We don't have to use nr_point-1 because we have already redused one point during the initialization. 
                int index = uniform_int_random(0,nr_points);
                // return spawn_points[index].second;
                if(spawn_points[index].first != frame_id)
                {
                    spawn_points[index].first = frame_id;
                    return spawn_points[index].second;
                }

                for(unsigned i = 0; i < nr_points; ++i)
                {
                    if(spawn_points[i].first != frame_id)
                    {
                        spawn_points[i].first = frame_id;
                        return spawn_points[i].second;
                    }
                }
                return spawn_points[index].second;
                // nav_space->copy_point(tmp_point, spawn_points[index].second);
                // tmp_point->memory[0] += uniform_random(0.0,5.0);
                // tmp_point->memory[1] += uniform_random(0.0,5.0);
                // return tmp_point;
                // return NULL;
            }

            bool origin_t::ready_to_spawn( double sim_time )
            {
                //First, check if the spawn rate is changing
                if( sim_time >= next_spawn_rate_change )
                {
                    //If there are still more entries in the schedule
                    if( schedule_index < schedule.size()-1 )
                    {
                        //Update the spawn rate
                        ++schedule_index;
                        //Then, make sure we know when the next change is
                        next_spawn_rate_change = schedule[ schedule_index+1 ].first;
                    }
                    //Otherwise, we have reached the end of the schedule
                    else
                    {
                        //So we will never update the spawn rate again
                        next_spawn_rate_change = PRX_INFINITY;
                    }
                }
                
                //Then, if it is a minute interval
                double seconds = std::floor(sim_time + PRX_ZERO_CHECK);
                if( (sim_time - seconds) < PRX_ZERO_CHECK && ( ((unsigned)(seconds))%60 == 0) )
                {
                    //Add some agents to spawn
                    agents_to_spawn += schedule[schedule_index].second;
                }
                
                return agents_to_spawn > 0;
            }
            
            const std::vector< space_point_t* >& origin_t::get_next_spawn_points( )
            {
                next_spawns.clear();
                unsigned n = PRX_MINIMUM(agents_to_spawn, points.size());
                for( unsigned i=0; i<n; ++i )
                {
                    --agents_to_spawn;
                    next_spawns.push_back( points[i] );
                }
                return next_spawns;
            }

            const util::space_point_t* origin_t::get_point_to_go()
            {
                //TODO: go to a random point inside the origin.
                return points[uniform_int_random(0,nr_inside_points)];//doorways[uniform_int_random(0,nr_doorways)];
            }

        }
    }
}

