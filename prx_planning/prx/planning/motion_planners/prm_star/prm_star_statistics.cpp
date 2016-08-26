/**
 * @file prm_star_statistics.cpp
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

#include "prx/planning/motion_planners/prm_star/prm_star_statistics.hpp"

namespace prx
{
    using namespace util;
    namespace plan
    {

        prm_star_statistics_t::prm_star_statistics_t()
        {
            num_vertices = 0;
            num_edges = 0;
            num_connected_components = 0;
        }

        prm_star_statistics_t::~prm_star_statistics_t() { }

        std::string prm_star_statistics_t::get_statistics() const
        {
            std::stringstream out(std::stringstream::out);

            out << statistics_t::get_statistics() << " : " << num_vertices << " , " << num_edges << " , " << num_connected_components << " C ";
            if( cc_sizes.size() >= 0 )
            {
                unsigned i = 0;
                for( i = 0; i < cc_sizes.size() - 1; ++i )
                    out << cc_sizes[i] << " , ";
                out << cc_sizes[i];
            }
            return out.str();
        }

        bool prm_star_statistics_t::serialize(std::ofstream& stream) const
        {
            if( stream.is_open() )
            {
                stream << get_statistics();
                stream << "\n";
                return true;
            }
            return false;
        }

    }
}
