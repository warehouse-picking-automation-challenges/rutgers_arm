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


#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/goals/goal.hpp"

namespace prx
{
    namespace util
    {

        pluginlib::ClassLoader<goal_t> goal_t::loader("prx_utilities", "prx::util::goal_t");

        goal_t::goal_t() 
        {
            space = NULL;
            distance_metric = NULL;
            metric_from_input = false;
        }

        goal_t::~goal_t()
        {
            distance_metric->clear();
            if(metric_from_input)
                delete distance_metric;
            for(int i=0; i<goal_points.size(); i++)
                space->free_point(goal_points[i]);
            goal_points.clear();
        }

        void goal_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            if( parameters::has_attribute("distance_metric", reader, template_reader) )
            {
                distance_metric = parameters::initialize_from_loader<distance_metric_t > ("prx_utilities", reader, "distance_metric", template_reader, "distance_metric");
                metric_from_input = true;
                // PRX_INFO_S("Distance metric type: " << parameters::get_attribute_as<std::string > ("distance_metric/type", reader, template_reader));
                // std::string type = parameters::get_attribute_as<std::string > ("distance_metric/type",reader,template_reader);
                // distance_metric = distance_metric_t::get_loader().createUnmanagedInstance("prx_utilities/" + type);
                // if(template_reader==NULL && reader->has_attribute("distance_metric"))
                //     distance_metric->init(reader->get_child("distance_metric").get());
                // else if(reader->has_attribute("distance_metric"))
                //     distance_metric->init(reader->get_child("distance_metric").get(),template_reader->get_child("distance_metric").get());            
                // else
                //     distance_metric->init(template_reader->get_child("distance_metric").get());
            }
            else
            {
                PRX_FATAL_S("Missing distance_metric attribute in input files.");
            }
        }

        void goal_t::link_space(const space_t* inspace)
        {
            if(space != NULL && inspace->get_space_name() != space->get_space_name())
                PRX_FATAL_S("You cannot assign different space in a goal_t!");
            space = inspace;
            distance_metric->link_space(inspace);
        }

        void goal_t::link_metric(distance_metric_t* inmetric)
        {
            if(distance_metric == NULL)
                distance_metric = inmetric;
            else
                PRX_FATAL_S("Can't load new metric on an existing goal. Create a new goal.");
        }

        void goal_t::copy_goal_state(const space_point_t* goal_state)
        {
            PRX_ASSERT(space != NULL);
            PRX_ASSERT(goal_points.size() > 0);
            space->copy_point(goal_points[0], goal_state);
        }

        void goal_t::set_goal_state_from_vector(const std::vector<double>& g_vec)
        {
            PRX_ASSERT(space != NULL);
            PRX_ASSERT(goal_points.size() > 0);
            space->set_from_vector(g_vec,goal_points[0]);
        }

        const std::vector<space_point_t*>& goal_t::get_goal_points(unsigned &size)
        {
            size = 1;
            return goal_points;
        }

        const space_point_t* goal_t::get_first_goal_point() const
        {
            PRX_ASSERT(goal_points.size() > 0);
            return goal_points.front();
        }

        unsigned goal_t::size() const
        {
            return 1;
        }

        pluginlib::ClassLoader<goal_t>& goal_t::get_loader()
        {
            return loader;
        }

    }
}
