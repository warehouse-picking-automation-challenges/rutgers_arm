/**
 * @file mcr_test_statistics.hpp
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

#ifndef PRX_MCR_TEST_STATISTICS_HPP
#define	PRX_MCR_TEST_STATISTICS_HPP

#include "planning/statistics/rearrangement_primitive_statistics.hpp"
#include "prx/utilities/definitions/defs.hpp"

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
            class mcr_test_statistics_t : public rearrangement_primitive_statistics_t
            {

              public:

                mcr_test_statistics_t()
                {
                    size = 0;
                    clear();
                }

                virtual ~mcr_test_statistics_t(){ }

                virtual void clear()
                {
                    for( unsigned i = 0; i < size; ++i )
                    {
                        computation_times[i] = 0;
                        path_lengths[i] = 0;
                        path_costs[i] = 0;
                        constraints[i] = 0;
                        lost_paths[i] = 0;
                    }
                }

                virtual void average_results(int num)
                {
                    for( unsigned i = 0; i < size; ++i )
                    {
                        double dev = num - lost_paths[i];
                        computation_times[i] /= dev;
                        path_lengths[i] /= dev;
                        path_costs[i] /= dev;
                        constraints[i] /= dev;
                    }
                }

                virtual void add_stats(std::string name)
                {
                    names.push_back(name);
                    computation_times.push_back(0);
                    path_lengths.push_back(0);
                    path_costs.push_back(0);
                    constraints.push_back(0);
                    lost_paths.push_back(0);
                    size++;
                }

                virtual void add_at(unsigned index, double time, double length, double cost, double constraints_size, int losts)
                {
                    computation_times[index] += time;
                    path_lengths[index] += length;
                    path_costs[index] += cost;
                    constraints[index] += constraints_size;
                    lost_paths[index] += losts;
                }

                /** @copydoc statistics_t::get_statistics()*/
                virtual std::string get_statistics() const
                {
                    std::stringstream out(std::stringstream::out);
                    out << "name \t objects \t lost_paths \t constraints \t time \t cost \t length " << std::endl;
                    for( unsigned i = 0; i < size; ++i )
                        out << names[i] << " \t " << objects_no << " \t " << lost_paths[i]  << " \t " << constraints[i] << " \t " << computation_times[i] << " \t " << path_costs[i] << " \t " << path_lengths[i] << std::endl;
                    return out.str();
                }

                unsigned size;
                std::vector<std::string> names;
                std::vector<double> computation_times;
                std::vector<double> path_lengths;
                std::vector<double> path_costs;
                std::vector<double> constraints;
                std::vector<int> lost_paths;
            };
        }
    }
}

#endif
