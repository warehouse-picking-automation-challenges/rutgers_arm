/**
 * @file coordination_constraints.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_COORDINATION_CONSTRAINTS_HPP
#define	PRX_COORDINATION_CONSTRAINTS_HPP

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"


namespace prx
{
    namespace packages
    {
        namespace coordination_manipulation
        {
            /**
             * PRM star for generating coordination roadmaps for two manipulators
             * 
             * @author Andrew Kimmel
             * 
             */
            typedef std::vector< std::vector<bool> > coord_constraint;
            typedef std::vector< std::vector< coord_constraint > > constraint_matrix;
            typedef util::hash_t< std::string, constraint_matrix> multi_object_constraint;
            
            class coordination_constraints_t
            {
            public:
                coordination_constraints_t();
                virtual ~coordination_constraints_t();

                virtual bool is_valid_state(unsigned right_start_index, unsigned left_start_index, unsigned right_object_index, unsigned left_object_index, std::string right_object, std::string left_object);
                virtual unsigned estimate_conflict_area(unsigned right_object_index, unsigned left_object_index, std::string right_object, std::string left_object);
                virtual coord_constraint* get_constraint(unsigned right_object_index, unsigned left_object_index, std::string right_object, std::string left_object);
                virtual bool deserialize_constraints(const std::vector<std::string>& object_types, const std::vector<int>& num_objects, const std::string& directory, const std::string& experiment);

                virtual void print_constraint(unsigned rob_index, unsigned lob_index, std::string rob_type, std::string lob_type, coord_constraint* constraint_to_print);
                
                virtual unsigned get_max_left_size();
                virtual unsigned get_max_right_size();
            protected:
                  
                virtual bool deserialize_helper(unsigned rob_index, unsigned lob_index, std::string right_object, std::string left_object, const std::string& filename, coord_constraint* constraint_to_fill);
                
                /** Online stuff*/
                
                multi_object_constraint all_constraints;
                util::hash_t<std::string, std::vector< std::vector< unsigned > > > all_constraint_areas;
                util::hash_t<std::string, int> object_limits;
                
                coord_constraint* current_constraint;
                unsigned right_start_index, left_start_index;
                
                unsigned max_right_size, max_left_size;

            private:

            };
        }
    }
}

#endif //PRX_COORDINATION_CONSTRAINTS_HPP