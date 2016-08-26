/**
 * @file particle_distance.hpp 
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

#ifndef PRX_PARTICLE_DISTANCE_HPP
#define	PRX_PARTICLE_DISTANCE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/distance_functions/distance_function.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

namespace prx 
{ 
    namespace packages
    {
    	namespace conformant
    	{
			class particle_distance_t : public util::distance_function_t
			{
			    public:
			        particle_distance_t()
			        {
			        	state_size = 0;
			        	number_of_states = 0;
			        	used.clear();
			        }
			        ~particle_distance_t(){}
			        virtual void link_space(const util::space_t* space);
			        virtual double distance(const util::space_point_t* s1, const util::space_point_t* s2);
			        unsigned state_size;
			        unsigned number_of_states;
			        std::vector<bool> used;
			        util::space_t* subspace;
			        std::vector<double*> memory;
			        util::space_point_t* subpoint1;
			        util::space_point_t* subpoint2;
			};
		}
	} 
}

#endif 