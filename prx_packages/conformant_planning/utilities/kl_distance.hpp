/**
 * @file kl_distance.hpp 
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

#ifndef PRX_KL_DISTANCE_HPP
#define	PRX_KL_DISTANCE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/distance_functions/distance_function.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/boost/hash.hpp"

namespace prx 
{ 
    namespace packages
    {
    	namespace conformant
    	{
			class kl_distance_t : public util::distance_function_t
			{
			    public:
			        kl_distance_t()
			        {
			        	state_size = 0;
			        	number_of_states = 0;
			        	index_offsets.clear();
			        	counts.clear();
			        	counts2.clear();
			        }
			        ~kl_distance_t(){}
			        virtual void link_space(const util::space_t* space);
			        virtual double distance(const util::space_point_t* s1, const util::space_point_t* s2);
			        unsigned long state_size;
			        unsigned long number_of_states;
			        util::space_t* subspace;
			        std::vector<double*> memory;
			        util::space_point_t* subpoint1;
			        util::space_point_t* subpoint2;
			        util::hash_t<unsigned long,unsigned long> counts;
			        util::hash_t<unsigned long,unsigned long> counts2;
			        std::set<unsigned long> valid_indices;
			        std::vector<unsigned long> index_offsets;
			        double bin_size;
			    protected:
			    	unsigned long get_index(const double* p, unsigned long index);
			};
		}
	} 
}

#endif 