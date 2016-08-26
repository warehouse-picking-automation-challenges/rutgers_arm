/**
 * @file rearrangement_search_statistics.hpp
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

#ifndef PRX_REARRANGEMENT_SEARCH_STATISTICS_HPP
#define	PRX_REARRANGEMENT_SEARCH_STATISTICS_HPP

#include "planning/statistics/rearrangement_primitive_statistics.hpp"

#include "prx/utilities/definitions/defs.hpp"

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
            class rearrangement_search_statistics_t : public rearrangement_primitive_statistics_t
            {

              public:
                rearrangement_search_statistics_t();

                virtual ~rearrangement_search_statistics_t();

                /** @copydoc statistics_t::get_statistics()*/
                virtual std::string get_statistics() const;
                
                /** @copydoc rearrangement_primitive_statistics_t::print_statistics(bool)*/
                virtual std::string print_statistics(bool print_all = true) const;
                
                virtual void average(bool print_all = true);

                std::vector<const rearrangement_primitive_statistics_t*> sequence_stats;
                const rearrangement_primitive_statistics_t* planner_stats;                
                int num_v, num_e, num_cc;                
                int num_connections_tries;

                int averege_moved_objects;
                double average_node_connection_time;
                double max_connection_time;
                double min_connection_time;

                int num_success;
                double average_node_connection_time_success;
                double max_connection_time_success;
                double min_connection_time_success;

                int num_failures;
                double average_node_connection_time_failed;
                double max_connection_time_failed;
                double min_connection_time_failed;

                double everything_time;
            };
        }
    }
}

#endif
