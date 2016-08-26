/**
 * @file emd_distance.hpp 
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

#ifndef PRX_EMD_DISTANCE_HPP
#define	PRX_EMD_DISTANCE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/distance_functions/distance_function.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "utilities/particle_space.hpp"
#include "emd.h"

namespace prx 
{ 
    namespace packages
    {
    	namespace conformant
    	{
    		class insert
    		{
    		public:
    			insert()
    			{
    				count = 0;
    				real_world = NULL;
    			}
    			unsigned count;
    			double* real_world;
    		};
			class emd_distance_t : public util::distance_function_t
			{
			    public:
			        emd_distance_t()
			        {
			        	state_size = 0;
			        	number_of_states = 0;
			        	total_time = 0;
			        	count_of_calls = 0;
			        }
			        ~emd_distance_t(){}
			        virtual void link_space(const util::space_t* space);
			        virtual double distance(const util::space_point_t* s1, const util::space_point_t* s2);
			        unsigned state_size;
			        unsigned number_of_states;
			        util::hash_t<unsigned long,insert> counts;
			        util::hash_t<unsigned long,insert> counts2;
			        std::vector<unsigned long> index_offsets;
			        std::set<unsigned long> valid_indices;
			        std::set<unsigned long> valid_indices2;
			        double bin_size;
			    protected:
			    	const util::particle_space_t* particle_space;
					util::sys_clock_t clock;
					double total_time;
					int count_of_calls;
		            feature_t* f1;
		            feature_t* f2;
			    	unsigned long get_index(const util::particle_point_t* p, unsigned long index);
			    	void populate_point(const util::particle_point_t* p, unsigned long index, int choice);
			};
		}
	} 
}

#endif 