/**
 * @file hausdorff_distance.hpp 
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

#ifndef PRX_HAUSDORFF_DISTANCE_HPP
#define	PRX_HAUSDORFF_DISTANCE_HPP

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
			class hausdorff_distance_t : public util::distance_function_t
			{
			    public:
			        hausdorff_distance_t()
			        {
			        	state_size = 0;
			        	number_of_states = 0;
			        }
			        ~hausdorff_distance_t(){}
			        virtual void link_space(const util::space_t* space);
			        virtual double distance(const util::space_point_t* s1, const util::space_point_t* s2);
			        unsigned state_size;
			        unsigned number_of_states;
			        util::space_t* subspace;
			        std::vector<double*> memory;
			        util::space_point_t* subpoint1;
			        util::space_point_t* subpoint2;
			        util::hash_t<unsigned long,insert> counts;
			        util::hash_t<unsigned long,insert> counts2;
			        std::vector<unsigned long> index_offsets;
			        std::set<unsigned long> valid_indices;
			        std::set<unsigned long> valid_indices2;
			        double bin_size;
			    protected:
			    	unsigned long get_index(const double* p, unsigned long index);
			    	void populate_point(const double* p, unsigned long index, int choice);
			};
		}
	} 
}

#endif 