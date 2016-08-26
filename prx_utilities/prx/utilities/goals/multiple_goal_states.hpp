/**
 * @file multiple_goal_states.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_MULTIPLE_GOAL_STATES_HPP
#define	PRX_MULTIPLE_GOAL_STATES_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/goals/goal.hpp"

namespace prx
{
    namespace util
    {

        class parameter_reader_t;

        /** * 
         * A concrete implementation of a goal_T. Multiple state goals that we want to reach.
         * @brief  <b> Multiple state goals.</b>
         * 
         * @author Athanasios Krontiris
         */
        class multiple_goal_states_t : public goal_t
        {

          public:
            multiple_goal_states_t();

            multiple_goal_states_t(const space_t* inspace, distance_metric_t* inmetric, int buffer_size = 100);

            virtual ~multiple_goal_states_t();

            /** @copydoc goal_t::init(const parameter_reader_t*, const parameter_reader_t*) */
            virtual void init(const parameter_reader_t* reader, const parameter_reader_t* template_reader = NULL);

            /** @copydoc goal_t::link_space( const space_t* ) */
            virtual void link_space(const space_t* inspace);

            /** @copydoc goal_t::get_goal_points(int *) */
            virtual const std::vector<space_point_t*>& get_goal_points(unsigned &size);

            /**
             * Clears the goal points.
             * @brief Clears the goal points.
             */
            virtual void clear();

            /** @copydoc goal_t::size() */
            virtual unsigned size() const;

            /** @copydoc goal_t::satisfied(const space_point_t* ) */
            virtual bool satisfied(const space_point_t* state);

            /** @copydoc goal_t::satisfied(const space_point_t* , double& ) */
            virtual bool satisfied(const space_point_t* state, double& distance);
            
            /**
             * Appends a new goal state to this goal_t object.
             * 
             * @brief Appends a new goal state. 
             * 
             * @param goal_state The new goal state that will be appended. 
             */
            virtual void append_goal_state(const space_point_t* goal_state);
            
            /**
             * Appends many goal states. If the size is -1 then the entire goals vector will be added.
             * Otherwise, will add only the specified amount of points. 
             * 
             * @brief Appends many goal states.
             * 
             * @param goals The new goals that will be appended. 
             */
            virtual void append_multiple_goal_states(const std::vector<space_point_t*>& goals, int size = -1);

            /**
             * Appends a new goal state to this goal_t object.
             * 
             * @brief Appends a new goal state. 
             * 
             * @param goal_state The new goal state that will be appended. 
             */
            virtual void append_goal_state_from_vector(const std::vector<double>& g_vec);

            /**
             * Appends many goal states. If the size is -1 then the entire goals vector will be added. 
             * Otherwise, will add only the specified amount of points. 
             * 
             * @brief Appends many goal states.
             * 
             * @param goals The new goals that will be appended. 
             */
            virtual void append_goal_states_from_vectors(const std::vector< std::vector<double> >& g_vecs, int size = -1);
            
            /**
             * Clears all the previous goal points and copies this goal state.
             * 
             * @brief Clears all the previous goal points and copies this goal state.
             * 
             * @param goal_state The new goal state that will be copied in the vector.
             * 
             */
            virtual void copy_goal_state(const space_point_t* goal_state);
            
            /**
             * Clears all the previous goal points and copies the goal states from the goals vector to the local vector.
             * If the size is -1 then the entire goals vector will be added.
             * Otherwise, will add only the specified amount of points. 
             * 
             * $brief Clears all the previous goal points and copies the goal states from the goals vector to the local vector.
             * 
             * @param goals The new goal points that will be copied in the vector
             */
            virtual void copy_multiple_goal_states(const std::vector<space_point_t*>& goals, int size = -1);

            /**
             * Clears all the previous goal points and copies this goal state.
             * 
             * @brief Clears all the previous goal points and copies this goal state.
             * 
             * @param goal_state The new goal state that will be copied in the vector.
             * 
             */
            virtual void set_goal_state_from_vector(const std::vector<double>& g_vec);

            /**
             * Clears all the previous goal points and copies the goal states from the goals vector to the local vector.
             * If the size is -1 then the entire goals vector will be added.
             * Otherwise, will add only the specified amount of points. 
             * 
             * $brief Clears all the previous goal points and copies the goal states from the goals vector to the local vector.
             * 
             * @param goals The new goal points that will be copied in the vector
             */
            virtual void set_goal_states_from_vectors(const std::vector< std::vector<double> >& g_vecs, int size = -1);

            /**
             * Get the pluginlib loader for this base class.
             * @brief Get the pluginlib loader for this base class.
             * @return The pluginlib loader.
             */
            static pluginlib::ClassLoader<multiple_goal_states_t>& get_loader();

            virtual int get_number_of_points() 
            {
                return counter;
            }

          protected:

            void increase_buffer();

            /**
             * @brief Temporary storage for the goal states from the input.
             */
            std::vector<std::vector<double> > states_vec;

            int initial_buffer_size;
            int counter;

        };

    }
}

#endif	
