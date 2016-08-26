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
#include "prx/utilities/goals/aggregate_goal.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::util::aggregate_goal_t, prx::util::goal_t)

namespace prx
{
    namespace util
    {
        aggregate_goal_t::aggregate_goal_t() 
        {
            satisfy_all_goals = false;
        }

        aggregate_goal_t::~aggregate_goal_t()
        {
            foreach(goal_t* inner_goal, goals)
            {
                delete inner_goal;
            }
        }

        void aggregate_goal_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            //goal_t::init(reader, template_reader);
            satisfy_all_goals = parameters::get_attribute_as<bool>("satisfy_all_goals", reader, template_reader, false);
        }

        void aggregate_goal_t::link_space(const space_t* inspace)
        {
            PRX_WARN_S ("Link space does nothing in aggregate goal!");
        }

        void aggregate_goal_t::link_metric(distance_metric_t* inmetric)
        {
            PRX_WARN_S ("link_metric does nothing in aggregate goal!");
        }

        void aggregate_goal_t::copy_goal_state(const space_point_t* goal_state)
        {
            PRX_WARN_S ("copy_goal_state does nothing in aggregate goal!");
        }

        void aggregate_goal_t::set_goal_state_from_vector(const std::vector<double>& g_vec)
        {
            PRX_WARN_S ("set_goal_state_from_vector does nothing in aggregate goal!");
        }

        bool aggregate_goal_t::satisfied(const space_point_t* state)
        {
            bool goals_satisfied = goals[0]->satisfied(state);
            for(unsigned i = 1; i < goals.size() && !goals_satisfied; ++i)
            {
                bool check = goals[i]->satisfied(state);

                if (satisfy_all_goals)
                {
                    goals_satisfied = (check && goals_satisfied);
                }
                else
                {
                    goals_satisfied = check;
                }
            }
            if (goals_satisfied)
            {
                PRX_PRINT ("Aggregate satisfied check", PRX_TEXT_BROWN);

            }
            return goals_satisfied;

        }

        bool aggregate_goal_t::satisfied(const space_point_t* state, double& distance)
        {
            distance = 0.0;

            return satisfied(state);
        }

        const std::vector<space_point_t*>& aggregate_goal_t::get_goal_points(unsigned &size)
        {
            goal_points.clear();
            unsigned inner_size;
            std::vector<space_point_t*> inner_goal_points;
            foreach (goal_t* inner_goal, goals)
            {
                inner_goal_points = inner_goal->get_goal_points(inner_size);

                for(unsigned i = 0; i < inner_size; ++i)
                {
                    goal_points.push_back(inner_goal_points[i]);
                }
            }
            size = goal_points.size();
            return goal_points;
        }

        void aggregate_goal_t::add_goal(goal_t* new_goal)
        {
            goals.push_back(new_goal);
        }

        void aggregate_goal_t::clear_goals()
        {
            goals.clear();
        }

        // TODO: Undefined behavior for goals that do not reason about points
        const space_point_t* aggregate_goal_t::get_first_goal_point() const
        {
            PRX_ASSERT(goal_points.size() > 0);
            return goals[0]->get_first_goal_point();
        }

        unsigned aggregate_goal_t::size() const
        {

            unsigned size = 0;
            foreach (goal_t* inner_goal, goals)
            {
                size+=inner_goal->size();
            }
            return size;
        }

    }
}
