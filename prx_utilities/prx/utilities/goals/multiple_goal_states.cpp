/**
 * @file goal.cpp
 *  * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/utilities/goals/multiple_goal_states.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::util::multiple_goal_states_t, prx::util::goal_t)

namespace prx
{
    namespace util
    {

        multiple_goal_states_t::multiple_goal_states_t() 
        {             
            counter = 0;
            initial_buffer_size = 0;
        }

        multiple_goal_states_t::multiple_goal_states_t(const space_t* inspace, distance_metric_t* inmetric, int buffer_size)
        {
            counter = 0;
            initial_buffer_size = buffer_size;
            goal_t::link_metric(inmetric);
            link_space(inspace);
        }

        multiple_goal_states_t::~multiple_goal_states_t() 
        {
            //abstact goal_t will delete all the points from the goal_points.
        }

        void multiple_goal_states_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            goal_t::init(reader, template_reader);

            if( parameters::has_attribute("goal_states", reader, template_reader) )
            {

                foreach(const parameter_reader_t* reader, parameters::get_list("goal_states", reader, template_reader))
                {
                    states_vec.push_back(reader->get_attribute_as< std::vector<double> >("state"));
                }
            }
            else
            {
                PRX_WARN_S("No goal states specified in input for the multiple_goal_states goal.");
            }

            initial_buffer_size = parameters::get_attribute_as<int>("initial_buffer_size", reader, template_reader, 100);
        }

        void multiple_goal_states_t::link_space(const space_t* inspace)
        {
            if(space != NULL)
            {
                for( int i=0; i<counter; i++ )
                    space->free_point(goal_points[i]);
                goal_points.clear();
            }

            goal_t::link_space(inspace);
            counter = 0;

            for(int i=0; i<initial_buffer_size; ++i)
                goal_points.push_back(space->alloc_point());
            
            if( states_vec.size() != 0 )
            {
                for( unsigned i = 0; i < states_vec.size(); ++i )
                {
                    space->set_from_vector(states_vec[i], goal_points[i]);
                    counter++;
                    if( counter == goal_points.size() )
                        increase_buffer();
                }
            }
        }

        const std::vector<space_point_t*>& multiple_goal_states_t::get_goal_points(unsigned &size)
        {
            size = counter;
            return goal_points;
        }

        void multiple_goal_states_t::clear()
        {
            counter = 0;
        }

        unsigned multiple_goal_states_t::size() const
        {
            return counter;
        }

        bool multiple_goal_states_t::satisfied(const space_point_t* state)
        {

            for( int i=0; i<counter; i++ )
            {
                if( distance_metric->distance_function(goal_points[i], state) <= PRX_ZERO_CHECK )
                    return true;
            }
            return false;
        }

        bool multiple_goal_states_t::satisfied(const space_point_t* state, double& distance)
        {
            for( int i=0; i<counter; i++ )
            {
                distance = distance_metric->distance_function(goal_points[i], state);
                if( distance <= PRX_ZERO_CHECK )
                    return true;
            }
            return false;
        }

        void multiple_goal_states_t::append_goal_state(const space_point_t* goal_state)
        {
            space->copy_point( goal_points[counter], goal_state);
            counter++;
            if( counter == goal_points.size() )
                increase_buffer();
        }

        void multiple_goal_states_t::append_multiple_goal_states(const std::vector<space_point_t*>& goals, int size)
        {
            if(size == -1)
                size = goals.size();

            for(int i=0; i<size; ++i)
            {
                space->copy_point( goal_points[counter], goals[i]);
                counter++;
                if( counter == goal_points.size() )
                    increase_buffer();
            }
        }

        void multiple_goal_states_t::append_goal_state_from_vector(const std::vector<double>& g_vec)
        {
            space->set_from_vector( g_vec, goal_points[counter]);
            counter++;
            if( counter == goal_points.size() )
                increase_buffer();
        }

        void multiple_goal_states_t::append_goal_states_from_vectors(const std::vector< std::vector<double> >& g_vecs, int size)
        {
            if(size == -1)
                size = g_vecs.size();

            for(int i=0; i<size; ++i)
            {
                space->set_from_vector( g_vecs[i], goal_points[counter]);
                counter++;
                if( counter == goal_points.size() )
                    increase_buffer();
            }
        }
        
        void multiple_goal_states_t::copy_goal_state(const space_point_t* goal_state)
        {
            clear();
            append_goal_state(goal_state);
        }

        void multiple_goal_states_t::copy_multiple_goal_states(const std::vector<space_point_t*>& goals, int size)
        {
            clear();
            append_multiple_goal_states(goals, size);
        }

        void multiple_goal_states_t::set_goal_state_from_vector(const std::vector<double>& g_vec)
        {
            clear();
            append_goal_state_from_vector(g_vec);
        }

        void multiple_goal_states_t::set_goal_states_from_vectors(const std::vector< std::vector<double> >& g_vecs, int size)
        {
            clear();
            append_goal_states_from_vectors(g_vecs,size);
        }

        void multiple_goal_states_t::increase_buffer()
        {
            int size = goal_points.size();
            for( int i=0; i<size; i++ )
                goal_points.push_back( space->alloc_point() );
        }
    }
}
