/**
 * @file baxter_end_effector_cost.hpp 
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

#ifndef PRX_BAXTER_END_EFFECTOR_COST_HPP
#define	PRX_BAXTER_END_EFFECTOR_COST_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/cost_functions/default_uniform.hpp"

#ifdef RBDL_FOUND
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#endif

namespace prx 
{ 
	namespace sim 
	{
		/**
		 *  
		 */
		class baxter_end_effector_cost_t : public default_uniform_t
		{
		public:
		    baxter_end_effector_cost_t();
		    ~baxter_end_effector_cost_t();
		    
            virtual double state_cost(const util::space_point_t* s);

            virtual double trajectory_cost(const trajectory_t& t);
            
            virtual double heuristic_cost(const util::space_point_t* s,const util::space_point_t* t);
		protected:
			unsigned id;
			bool update;
#ifdef RBDL_FOUND
            mutable RigidBodyDynamics::Model model;
            mutable RigidBodyDynamics::Math::VectorNd Q;
            RigidBodyDynamics::Math::VectorNd QDot;
            RigidBodyDynamics::Math::VectorNd QDDot;
#endif
		};
	} 
}

#endif 