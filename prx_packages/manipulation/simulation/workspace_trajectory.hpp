/**
 * @file workspace_trajectory.hpp 
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

#ifndef PRACSYS_WORKSPACE_TRAJECTORY_HPP
#define PRACSYS_WORKSPACE_TRAJECTORY_HPP

#include "prx/simulation/trajectory.hpp"
#include "prx/utilities/math/configurations/config.hpp"

#include <deque>

namespace prx 
{ 
    namespace packages
    {
        namespace manipulation
        {
            /**
             * A specialized trajectory used solely for paths through workspace (SE(3)).
             * 
             * @brief <b> Maintains a trajectory in workspace </b>
             * 
             * @author Andrew Dobson
             */
            class workspace_trajectory_t : public sim::trajectory_t
            {
              public:
                workspace_trajectory_t();

                /**
                 * Copy constructor, copies the trajectory from \c t to the current trajectory. 
                 * @param t The source trajectory from where we need to copy the states of the trajectory. 
                 */
                workspace_trajectory_t(const workspace_trajectory_t& t);

                ~workspace_trajectory_t();

                void copy_onto_front( const util::config_t& config );

                void copy_onto_back( const util::config_t& config );
                
                double distance( unsigned index_a, unsigned index_b ) const;

              protected:
                std::vector< double* > space_memory;
            };
        }
    } 
}

#endif
