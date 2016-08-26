/**
 * @file grasp_data.hpp
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

#ifndef PRX_GRASP_HPP
#define PRX_GRASP_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/config.hpp"

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {

            /**
             * @anchor grasp_t
             *
             * This class represents a grasping entry. 
             * 
             * @brief <b> General grasping data for a single grasp. </b>
             *
             * @author Athanasios Krontiris
             */
            class grasp_t
            {
                public:
                    grasp_t();

                    grasp_t(const util::config_t& conf, double open, double grasp, double score = 0.0, double tie_breaking = 0.0);

                    void clear();

                    grasp_t& operator=(const grasp_t& g);

                    bool operator==(const grasp_t& g);

                    util::config_t relative_config;
                    double release_mode;
                    double grasping_mode;

                    // The score of the grasp relates to how good of a grasp it is
                    double grasp_score;
                    double tie_breaking_score;

                    bool operator < (const grasp_t& rhs) const;
                    bool operator > (const grasp_t& rhs) const;

                    friend std::ostream& operator<<(std::ostream& os, const grasp_t& grasp);
            };

        }
    }
}

#endif

