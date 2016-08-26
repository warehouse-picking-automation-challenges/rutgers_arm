/**
 * @file null_constriants.cpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#include "prx/utilities/heuristic_search/null_constraints.hpp"

#include <fstream>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::util::null_constraints_t, prx::util::constraints_t)

namespace prx
{
    namespace util
    {
        null_constraints_t::null_constraints_t()
        {
        }
        
        null_constraints_t::~null_constraints_t()
        {
        }
        
        std::string null_constraints_t::get_type()
        {
            return "null_constraints";
        }
        
        constraints_t& null_constraints_t::operator=( const constraints_t& c )
        {
            return *this;            
        }
        
        bool null_constraints_t::operator==( const constraints_t& c ) const
        {
            return true;
        }
        
        bool null_constraints_t::operator<(const constraints_t& c) const
        {
            return false;
        }
        
        void null_constraints_t::merge(const constraints_t* c)
        {
        }
        
        bool null_constraints_t::intersect(constraints_t* out_constraints, const constraints_t* valid_constraints)
        {
            return false;
        }
        
        bool null_constraints_t::has_intersection(const constraints_t* valid_constraints) const
        {
            return false;
        }
        
        bool null_constraints_t::has_hard_constraints(const constraints_t* valid_constraints) const
        {
            return false;
        }

        void null_constraints_t::add_to_constraint_sets( const constraints_t* c, double d )
        {
            PRX_DEBUG_COLOR("Should not be using NULL constraints when doing constraint search!!", PRX_TEXT_RED);
        }
        
        bool null_constraints_t::exact_constraint_comparison(const constraints_t* new_constraints, double new_distance)
        {
            PRX_FATAL_S("Doing exact constraint comparison on NULL constraints is not possible.  Something is setup incorrectly!");
            return false;
        }
        
    }
}
    
