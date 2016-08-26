/**
 * @file motion_query.hpp
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

#ifndef PRX_MOTION_QUERY_HPP
#define	PRX_MOTION_QUERY_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/plan.hpp"
#include "prx/simulation/trajectory.hpp"
#include "prx/planning/queries/query.hpp"

namespace prx
{

    namespace plan
    {

        /**
         * @anchor motion_query_t
         *
         * This class represents a single motion as a result from a task planner.
         *
         * @brief <b> General query for planners that generate a motion. </b>
         *
         * @author Zakary Littlefield
         */
        class motion_query_t : public query_t
        {
          public:
            
            motion_query_t();
            virtual ~motion_query_t();

            /**
             * @brief Clear the plan and trajectory of this query.
             *
             * Clears the plan and the trajectory of the planning query in order to 
             * reuse the planning query with the same start and stopping criterion. 
             */
            virtual void clear();

            /**
             * @brief Links the state and control spaces that the planning query is working over.
             *
             * @param state_space The state space.
             * @param control_space The control space.
             */
            virtual void link_spaces(const util::space_t* state_space, const util::space_t* control_space);
            
            /**
             * @brief Returns the final state in the trajectory for the solution plan.
             *
             * @return The final point in the trajectory.  If there is no trajectory, returns NULL.
             */
            virtual util::space_point_t* get_solution_final_state();

            /** @brief Computed plan which answers this query. */
            sim::plan_t plan;
            /** @brief Computed trajectory which answers this query. */
            sim::trajectory_t path;
            /** @brief State space over which this query is operating. */
            const util::space_t* state_space;
            /** @brief Control space over which this query is operating. */
            const util::space_t* control_space;
        };
    }
}

#endif

