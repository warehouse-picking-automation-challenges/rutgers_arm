/**
 * @file irs_statistics.cpp
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

#include "prx/planning/motion_planners/irs/irs_statistics.hpp"

namespace prx 
{ 
    using namespace util;
    namespace plan 
    {
        
        irs_statistics_t::irs_statistics_t() 
        {
            num_vertices = 0;
            num_edges = 0;
            rejected_edges = 0;
            elapsed = 0;
        }
        
        irs_statistics_t::~irs_statistics_t() { }
        
        std::string irs_statistics_t::get_statistics() const
        {
            std::stringstream out(std::stringstream::out);
            
            out << statistics_t::get_statistics() << " , " << elapsed << " , "  << num_vertices << " , " << num_edges << " , " << rejected_edges << " , " << ( (double)num_edges /( (double)( num_edges + rejected_edges ) )) << "\n"; 
            
            return out.str();
        }
        
        bool irs_statistics_t::serialize(std::ofstream& stream) const
        {
            if(stream.is_open())
            {
                stream << get_statistics();
                return true;
            }
            return false;
        }
        
    }
}
