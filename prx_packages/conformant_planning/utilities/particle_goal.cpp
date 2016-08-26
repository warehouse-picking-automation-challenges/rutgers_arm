/**
 * @file particle_goal.cpp
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

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "utilities/particle_goal.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::packages::conformant::particle_goal_t, prx::util::goal_t)

namespace prx
{
    using namespace util;
    namespace packages
    {        
        namespace conformant
        {
            particle_goal_t::particle_goal_t() { }

            particle_goal_t::~particle_goal_t() { }

            void particle_goal_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                PRX_DEBUG_COLOR("Inside particle goal region init!", PRX_TEXT_BLUE);
                goal_t::init(reader, template_reader);

                if( parameters::has_attribute("goal_state", reader, template_reader) )
                {
                    PRX_DEBUG_S("Reading in goal state");
                    state_vec = parameters::get_attribute_as< std::vector<double> >("goal_state", reader, template_reader);
                    for( unsigned i = 0; i < state_vec.size(); i++ )
                    {
                        PRX_DEBUG_COLOR("Goal: " << state_vec[i], PRX_TEXT_RED);
                    }
                }
                else
                {
                    PRX_WARN_S("Missing goal_state attribute in input files.");
                }
                if( parameters::has_attribute("radius", reader, template_reader) )
                {
                    PRX_DEBUG_S("Reading in goal radius");
                    radius = parameters::get_attribute_as< double >("radius", reader, template_reader);
                    PRX_DEBUG_COLOR("Radius is: " << radius, PRX_TEXT_CYAN);
                }
                else
                {
                    PRX_FATAL_S("Missing radius attribute in input files for the particle_goal_t.");
                }
                if( parameters::has_attribute("accepted_threshold", reader, template_reader) )
                {
                    PRX_DEBUG_S("Reading in accepted_threshold");
                    accepted_threshold = parameters::get_attribute_as< double >("accepted_threshold", reader, template_reader);
                    PRX_DEBUG_COLOR("accepted_threshold is: " << accepted_threshold, PRX_TEXT_CYAN);
                }
                else
                {
                    PRX_FATAL_S("Missing accepted_threshold in input files for the particle_goal_t.");
                }
            }

            void particle_goal_t::link_space(const space_t* inspace)
            {
                PRX_DEBUG_COLOR("Radial goal region link space",PRX_TEXT_BLUE);
                space = inspace;
                distance_metric->link_space(inspace);

                point = space->alloc_point();
                test_point = space->alloc_point();
                space->set_from_vector(state_vec, point);
                goal_points.push_back(point);
            }

            bool particle_goal_t::satisfied(const space_point_t* state)
            {
                double distance;
                return satisfied(state,distance);
            }

            bool particle_goal_t::satisfied(const space_point_t* state, double& distance)
            {
                int i=0;
                int accepted=0;
                for(i=0;i<particle_space->get_num_particles();i++)
                {
                    particle_space->copy_to_particle(test_point,i,state);
                    accepted += (distance_metric->distance_function(test_point, point) <= radius);
                }
                // PRX_INFO_S(particle_space->print_point(state)<<" "<<accepted<<" "<<i);
                distance = (accepted*1.0/i);
                return distance>accepted_threshold;
            }
            double particle_goal_t::get_radius() const
            {
                return radius;
            }

            const std::vector<double>& particle_goal_t::get_goal_vec() const
            {
                return state_vec;
            }

            void particle_goal_t::set_goal(distance_metric_t* d_metric, const std::vector<double>& s_vec, double r)
            {
                PRX_FATAL_S("PLEASE DONT DO THIS TO THE PARTICLE GOAL. MAY BE UNDEFINED BEHAVIOR");
                // distance_metric = d_metric;
                // state_vec = s_vec;
                // radius = r;
            }
        }
    }
}
