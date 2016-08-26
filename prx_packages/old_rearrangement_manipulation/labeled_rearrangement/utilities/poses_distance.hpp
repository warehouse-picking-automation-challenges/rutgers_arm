/**
 * @file poses_distance.hpp 
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

#ifndef PRX_POSES_DISTANCE_HPP
#define	PRX_POSES_DISTANCE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/distance_functions/distance_function.hpp"

namespace prx 
{ 
    namespace packages
    {
    	namespace distance_functions
    	{
			/**
			 * A class that computes the poses distance between two points.
			 * @brief <b> A class that computes the poses distance between two points. </b>
			 * @author TBD
			 */
			class poses_distance_t : public util::distance_function_t
			{
			    public:
			        poses_distance_t(){}
			        ~poses_distance_t(){}
			        
			        virtual double distance(const util::space_point_t* s1, const util::space_point_t* s2);
			        
			};
		}
	} 
}

#endif 