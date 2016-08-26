/**
 * @file rearrangement_search_statistics.cpp
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

#include <vector>

#include "planning/statistics/rearrangement_search_statistics.hpp"

namespace prx
{
    namespace packages
    {
        namespace labeled_rearrangement_manipulation
        {

            rearrangement_search_statistics_t::rearrangement_search_statistics_t()
            {
                num_v = 0;
                num_e = 0;
                num_cc = 0;
                num_connections_tries = 0;
                averege_moved_objects = 0;

                average_node_connection_time = 0;
                max_connection_time = 0;
                min_connection_time = PRX_INFINITY;

                num_success = 0;
                average_node_connection_time_success = 0;
                max_connection_time_success = 0;
                min_connection_time_success = PRX_INFINITY;

                num_failures = 0;
                average_node_connection_time_failed = 0;
                max_connection_time_failed = 0;
                min_connection_time_failed = PRX_INFINITY;

                everything_time = 0;
            }

            rearrangement_search_statistics_t::~rearrangement_search_statistics_t() { }

            std::string rearrangement_search_statistics_t::get_statistics() const
            {
                std::stringstream out(std::stringstream::out);

                out << "objects , found path , object moved , Time , Length , Computation Time , Path Planner Time , |V| , |E| , |CC| , Aver Objs , CON tries , CON Succ , Aver T, Min T , Max T, Aver ST , MIN ST , MAX ST , Aver FT , MIN FT , MAX FT" << std::endl;
                out << objects_no << " , " << found_path << " , " << num_of_moved_objects << " , " << everything_time << " , " << path_length << " , " << computation_time << " , " << planner_stats->computation_time << " , ";
                out << num_v << " , " << num_e << " , " << num_cc << " , " << averege_moved_objects << " , " << num_connections_tries << " , ";
                out << average_node_connection_time << " , " << min_connection_time << " , " << max_connection_time << " , " << average_node_connection_time_success << " , " << min_connection_time_success << " , " << max_connection_time_success << " , " << average_node_connection_time_failed << " , " << min_connection_time_failed << " , " << max_connection_time_failed;
                out << std::endl;

                return out.str();
            }

            std::string rearrangement_search_statistics_t::print_statistics(bool print_all) const
            {
                if( print_all )
                    return get_statistics();

                std::stringstream out(std::stringstream::out);
                out << "objects , found path , object moved , Time , Length" << std::endl;
                out << objects_no << " , " << found_path << " , " << num_of_moved_objects << " , " << everything_time << " , " << path_length;
                out << std::endl;
                return out.str();
            }

            void rearrangement_search_statistics_t::average(bool print_all)
            {
                everything_time = computation_time + planner_stats->computation_time;
                num_of_moved_objects = planner_stats->num_of_moved_objects;
                path_length = planner_stats->path_length;

                if( print_all )
                {
                    foreach(const rearrangement_primitive_statistics_t* stat, sequence_stats)
                    {
                        averege_moved_objects += stat->num_of_moved_objects;

                        average_node_connection_time += stat->computation_time;
                        min_connection_time = PRX_MINIMUM(min_connection_time, stat->computation_time);
                        max_connection_time = PRX_MAXIMUM(max_connection_time, stat->computation_time);

                        if( stat->found_path )
                        {
                            num_success++;
                            average_node_connection_time_success += stat->computation_time;
                            min_connection_time_success = PRX_MINIMUM(min_connection_time_success, stat->computation_time);
                            max_connection_time_success = PRX_MAXIMUM(max_connection_time_success, stat->computation_time);
                        }

                        if( !stat->found_path )
                        {
                            num_failures++;
                            average_node_connection_time_failed += stat->computation_time;
                            min_connection_time_failed = PRX_MINIMUM(min_connection_time_failed, stat->computation_time);
                            max_connection_time_failed = PRX_MAXIMUM(max_connection_time_failed, stat->computation_time);
                        }
                    }
                    double num = (double)sequence_stats.size();
                    averege_moved_objects /= num;
                    average_node_connection_time /= num;
                    average_node_connection_time_success /= num_success;
                    average_node_connection_time_failed /= num_failures;
                }
            }
        }
    }
}