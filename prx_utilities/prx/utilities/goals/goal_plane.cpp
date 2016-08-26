/**
 * @file goal_plane.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2016, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/goals/goal_plane.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::util::goal_plane_t, prx::util::goal_t)

namespace prx
{
    namespace util
    {

        goal_plane_t::goal_plane_t() { }

        goal_plane_t::goal_plane_t(const space_t* inspace, distance_metric_t* inmetric, const std::vector<double>& input_plane_point, const std::vector<double>& input_plane_normal)
        {
            goal_t::link_metric(inmetric);
            link_space(inspace);

            PRX_ASSERT(input_plane_point.size() == 3);
            PRX_ASSERT(input_plane_normal.size() == 3);

            plane_point = input_plane_point;
            plane_normal = input_plane_normal;
        }

        goal_plane_t::goal_plane_t(const space_t* inspace, distance_metric_t* inmetric, const vector_t& input_plane_point, const vector_t& input_plane_normal)
        {
            goal_t::link_metric(inmetric);
            link_space(inspace);

            PRX_ASSERT(input_plane_point.get_dim() == 3);
            PRX_ASSERT(input_plane_normal.get_dim() == 3);

            plane_point = input_plane_point;
            plane_normal = input_plane_normal;
        }

        goal_plane_t::~goal_plane_t()
        {
            //abstract goal_t will free the goal_state point.
        }

        void goal_plane_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            goal_t::init(reader, template_reader);

            if (parameters::has_attribute("plane_point", reader, template_reader))
            {
                plane_point = parameters::get_attribute_as< vector_t >("plane_point", reader, template_reader);
                PRX_ASSERT(plane_point.get_dim() == 3);
            }

            if (parameters::has_attribute("plane_normal", reader, template_reader))
            {
                plane_normal = parameters::get_attribute_as< vector_t >("plane_normal", reader, template_reader);
                PRX_ASSERT(plane_normal.get_dim() == 3);
            }

        }

        void goal_plane_t::link_space(const space_t* inspace)
        {
            if(space != NULL)
            {
                space->free_point(goal_points[0]);
                goal_points.clear();
            }
            goal_t::link_space(inspace);
            goal_points.push_back(space->alloc_point());
        }       

        bool goal_plane_t::satisfied(const space_point_t* state)
        {
            PRX_ASSERT(plane_point.get_dim() == 3);
            PRX_ASSERT(plane_normal.get_dim() == 3);

            // Assumes that the first 3 points in the state belong to X,Y,Z
            vector_t query_point(state->at(0),state->at(1),state->at(2));

            // Vector to state
            vector_t query_to_plane_point = query_point - plane_point;

            // Get dot product to plane normal
            double result = query_to_plane_point.dot_product(plane_normal);

            return (result > 0);
        }

        bool goal_plane_t::satisfied(const space_point_t* state, double& distance)
        {
            // Assumes that the first 3 points in the state belong to X,Y,Z
            vector_t query_point(state->at(0),state->at(1),state->at(2));

            // Vector to state
            vector_t query_to_plane_point = query_point - plane_point;

            // Get dot product to plane normal
            double result = query_to_plane_point.dot_product(plane_normal);

            // Compute distance to plane
            distance = (result/plane_normal.norm());

            return result;
        }
    }
}
