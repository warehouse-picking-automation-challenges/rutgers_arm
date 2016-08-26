/**
 * @file rearrangement_search_specificatioN.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors:Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#pragma once

#ifndef PRX_REARRANGEMENT_SEARCH_SPECIFICATION_HPP
#define PRX_REARRANGEMENT_SEARCH_SPECIFICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/problem_specifications/specification.hpp"

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
    }

    namespace packages
    {
        namespace rearrangement_manipulation
        {
            /**
             * @anchor rearrangement_search_specification_t
             *
             * 
             * @Author Athanasios Krontiris
             */
            class rearrangement_search_specification_t : public plan::specification_t
            {

              public:

                rearrangement_search_specification_t();

                virtual ~rearrangement_search_specification_t();

                /**
                 * @brief Initialize the specification from input parameters.
                 *
                 * @param reader The reader for the parameters.
                 * @param template_reader A template reader for reading template parameters 
                 */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);

                /** @brief The safe position where the manipulator will return without colliding with anything */
                std::vector<double> safe_position;
                /** @brief Number of object in the test*/
                unsigned k_objects;
                /** @brief The bias towards the goal poses of the MPG.*/
                int goal_biasing;
                /** @brief The time limit for the application to find a solution.*/
                double time_limit;
                /** @brief How many times we will try to generate a arrangement before we drop it and try a new one*/
                int max_tries;
                /** @brief If we want to gather stats or not */
                bool gather_statistics;
                /** @brief The file where the poses are stored.*/
                std::string poses_file;
                /** @brief The file where the poses with their constraints are stored.*/
                std::string poses_constraints_file;
                /** @brief the file where the informed transit graph is stored.*/
                std::string transit_graph_file;
                /** @brief the file where the informed transfer graph is stored.*/
                std::string transfer_graph_file;
                /** @brief the file where we will store the statistics for the algorithm.*/
                std::string statistics_file;

            };
        }
    }
}

#endif
