/**
 * @file distance_from_goal.cpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/simulation/cost_functions/distance_from_goal.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::sim::distance_from_goal_t, prx::sim::cost_function_t)

namespace prx
{
	using namespace util;
    namespace sim
    {       
        distance_from_goal_t::distance_from_goal_t()
        {
            goal = NULL;
        }
        distance_from_goal_t::~distance_from_goal_t()
        {
        }

        double distance_from_goal_t::state_cost(const space_point_t* s)
        {
            return 1;
        }

        double distance_from_goal_t::trajectory_cost(const trajectory_t& t)
        {
            //[0,0,0,0,6,0]
            double total=0;
            if(goal!=NULL)
            {
                foreach(const state_t* s, t)
                {
                    total+=dist(s,goal);
                }
                return total;
            }
            else
            {
                goal = t.state_space->alloc_point();
                goal->at(0) = 0;
                goal->at(1) = 0;
                goal->at(2) = 0;
                goal->at(3) = 0;
                goal->at(4) = 6;
                goal->at(5) = 0;
                return trajectory_cost(t);
            }
        }

        double distance_from_goal_t::true_cost(const trajectory_t& t, const plan_t& p)
        {
            //[0,0,0,0,6,0]
            double total=0;
            if(goal!=NULL)
            {
                foreach(const state_t* s, t)
                {
                    total+=dist(s,goal);
                }
                foreach(plan_step_t step, p)
                {
                    unsigned index = (step.duration/simulation::simulation_step+ .1);
                    for(unsigned i=0;i<index;i++)
                    {
                        total+=step.control->at(0)*step.control->at(0)*.00044;
                    }
                }
                return total;
            }
            else
            {
                goal = t.state_space->alloc_point();
                goal->at(0) = 0;
                goal->at(1) = 0;
                goal->at(2) = 0;
                goal->at(3) = 0;
                goal->at(4) = 6;
                goal->at(5) = 0;
                return true_cost(t,p);
            }
        }

        double distance_from_goal_t::heuristic_cost(const util::space_point_t* s,const util::space_point_t* t)
        {
            return 500000000*pow(dist(s,t),2);
        }


    }
}
