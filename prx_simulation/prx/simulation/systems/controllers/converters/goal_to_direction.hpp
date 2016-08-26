/**
 * @file goal_to_direction.hpp
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

#ifndef PRX_GOAL_TO_DIRECTION_HPP
#define	PRX_GOAL_TO_DIRECTION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/controllers/controller.hpp"

namespace prx
{
    namespace sim
    {

        /**
         * @brief <b> Things </b>
         * 
         * @author Andrew Kimmel
         */
        class goal_to_direction_t : public controller_t
        {

          public:
            goal_to_direction_t();
            virtual ~goal_to_direction_t();
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);
            virtual void compute_control();
            virtual void verify() const;

          protected:
              
            double _Cx, _Cy;
            double max_vel;
            
            control_t* converted_control;
              
            /** @brief */
            static const unsigned int X;
            /** @brief */
            static const unsigned int Y;
            
            /** @brief */
            static const unsigned int V;
            /** @brief */
            static const unsigned int THETA;

        };

    }
}

#endif
