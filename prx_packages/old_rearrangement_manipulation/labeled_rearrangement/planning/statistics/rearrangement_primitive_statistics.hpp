/**
 * @file rearrangement_primitive_statistics.hpp
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

#ifndef PRX_REARRANGEMENT_PRIMITIVE_STATISTICS_HPP
#define	PRX_REARRANGEMENT_PRIMITIVE_STATISTICS_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/statistics/statistics.hpp"

#include <iostream>
#include <fstream>
namespace prx
{
    namespace packages
    {
        namespace labeled_rearrangement_manipulation
        {

            /**
             * 
             * 
             * @author Athanasios Krontiris
             */
            class rearrangement_primitive_statistics_t : public util::statistics_t
            {

              public:
                rearrangement_primitive_statistics_t();

                virtual ~rearrangement_primitive_statistics_t();
                
                /** @copydoc statistics_t::clear()*/
                virtual void clear();

                /** @copydoc statistics_t::get_statistics()*/
                virtual std::string get_statistics() const;
                
                int objects_no;
                bool found_path;
                int num_of_moved_objects;
                double computation_time;
                double path_length;
            };

        }
    }
}

#endif
