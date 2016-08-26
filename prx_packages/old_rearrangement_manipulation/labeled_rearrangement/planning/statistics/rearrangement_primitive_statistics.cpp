/**
 * @file rearrangement_primitive_statistics.cpp
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

#include "planning/statistics/rearrangement_primitive_statistics.hpp"


namespace prx
{
    namespace packages
    {
        namespace labeled_rearrangement_manipulation
        {

            rearrangement_primitive_statistics_t::rearrangement_primitive_statistics_t()
            {
                clear();
            }

            rearrangement_primitive_statistics_t::~rearrangement_primitive_statistics_t() { }
            
            void rearrangement_primitive_statistics_t::clear()
            {
                statistics_t::clear();
                found_path = false;
                objects_no = -1;
                num_of_moved_objects = 0;
                computation_time = 0;
                path_length = 0;
            }

            std::string rearrangement_primitive_statistics_t::get_statistics() const
            {
                std::stringstream out(std::stringstream::out);
                out << "objects \t found path \t object moved \t Time \t Length";
                out << objects_no << " \t " << found_path << " \t " << num_of_moved_objects << " \t " << computation_time << " \t " << path_length;
                return out.str();
            }
        }
    }
}