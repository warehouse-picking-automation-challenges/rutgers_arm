/**
 * @file goal.hpp
 *  * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2016, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_AGGREGATE_GOAL_HPP
#define PRX_AGGREGATE_GOAL_HPP


#include "prx/utilities/goals/goal.hpp"

#include <pluginlib/class_loader.h>

namespace prx
{
    namespace util
    {

        class parameter_reader_t;

        /** * 
         * Hold all the information for any abstract version of goal. Concrete implementations of the 
         * class goal deal with different versions of goal. 
         * @brief <b> Hold all the information for any abstract version of goal. </b>
         * 
         * @author Andrew Kimmel, Rahul Shome
         */
        class aggregate_goal_t : public goal_t
        {

          public:
            aggregate_goal_t();
            virtual ~aggregate_goal_t();

            /** 
             * Initialize the goal from the given parameters.
             * @brief Initialize the goal from the given parameters.
             *
             * @param reader A \ref parameter_reader_t with a dictionary of parameters for this goal.
             * @param template_reader A \ref parameter_reader_t with a dictionary of parameters for the template of this goal.
             */
            virtual void init(const parameter_reader_t* reader, const parameter_reader_t* template_reader = NULL);
            

            /**
             * Initialize the goal from the given parameters.
             * @brief Links a space to the goal so the internal distance_metric will have a linked space.
             * 
             * @param inspace The space that the goal is working on.
             */
            virtual void link_space(const space_t* inspace);

            /**
             * Link in a new metric for the goal to use.
             * @brief Link a distance metric.
             * 
             * @param inmetric The metric object to link to this goal.
             */
            virtual void link_metric(distance_metric_t* inmetric);

            /**
             * Checks if the goal is satisfied. 
             * @brief Checks if the goal is satisfied. 
             * 
             * @param state The state to check that might satisfy the goal.
             * @return Whether the goal is satified or not.
             */
            virtual bool satisfied(const space_point_t* state);

            /**
             * Checks if the goal is satisfied. 
             * @brief Checks if the goal is satisfied. 
             * 
             * @param state The state to check that might satisfy the goal.
             * @param distance The distance between the goal criterion and the state that we 
             *        passed as argument.
             * @return Whether the goal is satified or not.
             */
            virtual bool satisfied(const space_point_t* state, double& distance);

            /**
             * Copies the new goal state to this aggregate_goal_t object.
             * 
             * @brief Copies a new goal state. 
             * 
             * @param goal_state The new goal state that will be copied in the aggregate_goal_t object.
             */
            virtual void copy_goal_state(const space_point_t* goal_state);

            /**
             * Copies the new goal state to this aggregate_goal_t object.
             * 
             * @brief Copies a new goal state. 
             * 
             * @param goal_state The new goal state that will be copied in the aggregate_goal_t object.
             */
            virtual void set_goal_state_from_vector(const std::vector<double>& g_vec);

            /**
             * Returns a vector of goal points that will satisfy the goal.
             * @brief Returns a vector of goal points that will satisfy the goal.
             * 
             * @return A vector with goal points. 
             */
            virtual const std::vector<space_point_t*>& get_goal_points(unsigned &size);

            /**
             * Returns the first goal point in the goals list. 
             * 
             * @brief Returns the first goal point in the goals list. 
             *
             * @return The first point in the goals list.
             */
            virtual const space_point_t* get_first_goal_point() const;

            /**
             * Returns the size of the points. Most of the time is 1 except multiple_goal_states.
             * By default will return 1. 
             * 
             * @brief Returns the size of the points. Most of the time is 1 except multiple_goal_states.
             *
             * @return The size of the points. Most of the time is 1 except multiple_goal_states.
             */
            virtual unsigned size() const;


            virtual void add_goal(goal_t* new_goal);
            virtual void clear_goals();

            /**
             * Get the pluginlib loader for this base class.
             * @brief Get the pluginlib loader for this base class.
             * @return The pluginlib loader.
             */
            static pluginlib::ClassLoader<aggregate_goal_t>& get_loader();

          protected:

            /** Store the goals */
            std::vector<goal_t*> goals;

            bool satisfy_all_goals;

        };

    }
}

#endif  
