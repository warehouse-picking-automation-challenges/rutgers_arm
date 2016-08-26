/**
 * @file goal.hpp
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
#pragma once

#ifndef PRX_HALF_SPACE_GOAL_HPP
#define	PRX_HALF_SPACE_GOAL_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "planning/manipulation_world_model.hpp" 
#include "prx/utilities/goals/goal.hpp"

namespace prx
{
    namespace util
    {

        class parameter_reader_t;

    }
    
    namespace packages
    {
        namespace manipulation
        {
            /** * 
             * Half space goal
             * @brief <b> Hold all the information for half space goal. </b>
             * 
             * @author Rahul Shome
             */
            class half_space_goal_t : public util::goal_t
            {

              public:
                half_space_goal_t();
                virtual ~half_space_goal_t();


                /**
                 * Initialize the goal from the given parameters.
                 * @brief Links a manipulation world model to the goal to get end effector positions. The plane and hs side denotes the equation of the plane which is being checked for.
                 * 
                 * @param in_model The manipulator_world_model planning is using right now. 
                 * @param in_plane The vector of plane parameters, [a,b,c,d] in equation ax + by + cz
                 * @param in_hs_side The boolean determines which side of the half space we check for. If true, ax + by + cz >= 0, else <=0
                 */
                virtual void setup(prx::plan::world_model_t* in_model, std::vector<double> in_plane, bool in_hs_side);

                /** @copydoc goal_t::link_space( const space_t* ) */
                virtual void link_space(const util::space_t* inspace);

                /**
                 * Checks if the goal is satisfied. 
                 * @brief Checks if the goal is satisfied. 
                 * 
                 * @param state The state to check that might satisfy the goal.
                 * @return Whether the goal is satified or not.
                 */
                virtual bool satisfied(const util::space_point_t* state);

                /**
                 * Checks if the goal is satisfied. 
                 * @brief Checks if the goal is satisfied. 
                 * 
                 * @param state The state to check that might satisfy the goal.
                 * @param distance The distance between the goal criterion and the state that we 
                 *        passed as argument.
                 * @return Whether the goal is satified or not.
                 */
                virtual bool satisfied(const util::space_point_t* state, double& distance);

                virtual util::space_point_t* get_last_satisfied_point();

                const std::vector<util::space_point_t*>& get_goal_points(unsigned &size);

                void copy_goal_state(const util::space_point_t* goal_state);

                virtual unsigned size() const;

              protected:
                prx::packages::manipulation::manipulation_world_model_t* model;
                std::vector<double> plane;
                bool hs_side;
                /**
                 * @brief The space point for temporary storage.
                 */
                util::space_point_t* temp_point;

                util::space_point_t* last_satisfied_point;

                
            };

        }

    }
}

#endif	
