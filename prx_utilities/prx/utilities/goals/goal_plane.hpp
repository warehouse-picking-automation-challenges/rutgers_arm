/**
 * @file goal_state.hpp 
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

#ifndef PRX_GOAL_STATE_HPP
#define PRX_GOAL_STATE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/goals/goal.hpp"

namespace prx
{
    namespace util
    {

        class parameter_reader_t;

        /**
         * A concrete implementation of a goal. 
         * Goal is satisfied only when the point is beyond (in the direction of the normal) the specified plane
         * @brief  <b> Goal that reasons about a plane .</b>
         * @author Andrew Kimmel
         */
        class goal_plane_t : public goal_t
        {

          public:            
            goal_plane_t();
            goal_plane_t(const space_t* inspace, distance_metric_t* inmetric, const vector_t& input_plane_point, const vector_t& input_plane_normal);
            goal_plane_t(const space_t* inspace, distance_metric_t* inmetric, const std::vector<double>& input_plane_point, const std::vector<double>& input_plane_normal);

            virtual ~goal_plane_t();

            /** @copydoc goal_t::init(const parameter_reader_t*, const parameter_reader_t*) */
            virtual void init(const parameter_reader_t* reader, const parameter_reader_t* template_reader);

            /** @copydoc goal_t::link_space( const space_t* ) */
            virtual void link_space(const space_t* inspace);
            
            /** @copydoc goal_t::satisfied(const space_point_t* ) */
            virtual bool satisfied(const space_point_t* state);

            /** @copydoc goal_t::satisfied(const space_point_t* , double& ) */
            virtual bool satisfied(const space_point_t* state, double& distance);

          protected:
            /**
             * @brief The space point representing a point on desired plane
             */
            vector_t plane_point;
            /**
             * @brief The space point representing the normal of the plane
             */
            vector_t plane_normal;
        };

    }
}

#endif  
