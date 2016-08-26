/**
 * @file constriants.cpp 
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
#include "prx/utilities/heuristic_search/constraints.hpp"
#include <fstream>

namespace prx
{
    namespace util
    {

        constraints_t::constraints_t(){}

        constraints_t::~constraints_t(){}

        void constraints_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
        }

        std::string constraints_t::get_type()
        {
            return "constraints_t";
        }

        void constraints_t::clear()
        {

        }

        bool constraints_t::exact_constraint_comparison(const constraints_t* new_constraints, double new_distance)
        {
            return true;
        }

        void constraints_t::serialize(std::ofstream& output_stream)
        {
            // PRX_STATUS("Serialization for constraints: " << get_type() << " is not implemented!", PRX_TEXT_BROWN);
        }

        void constraints_t::deserialize(std::ifstream& input_stream)
        {
            // PRX_STATUS("Deserialization for constraints: " << get_type() << " is not implemented!", PRX_TEXT_BROWN);
        }

        std::string constraints_t::print() const
        {
            return "";
        }

        pluginlib::ClassLoader<constraints_t> constraints_t::loader("prx_utilities", "prx::util::constraints_t");

        pluginlib::ClassLoader<constraints_t>& constraints_t::get_loader()
        {
            return loader;
        }
    }
}

