/**
 * @file distance_from_goal.hpp 
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
#pragma once

#ifndef PRX_DISTANCE_GOAL_HPP
#define	PRX_DISTANCE_GOAL_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/system.hpp"
#include "prx/simulation/cost_functions/default_uniform.hpp"

namespace prx 
{ 
	namespace sim 
	{
		/**
		 *  
		 */
		class distance_from_goal_t : public default_uniform_t
		{
		public:
		    distance_from_goal_t();
		    ~distance_from_goal_t();
		    
            virtual double state_cost(const util::space_point_t* s);

            virtual double trajectory_cost(const trajectory_t& t);
            
            virtual double heuristic_cost(const util::space_point_t* s,const util::space_point_t* t);

            virtual double true_cost(const trajectory_t& t, const plan_t& p);

            state_t* goal; 
		    
		};
	} 
}

#endif 