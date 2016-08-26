/**
 * @file rearrangement_search_query.hpp
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

#ifndef PRX_REARRANGEMENT_SEARCH_QUERY_HPP
#define	PRX_REARRANGEMENT_SEARCH_QUERY_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/state.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
    }

    namespace packages
    {
        namespace labeled_rearrangement_manipulation
        {

            /**             
             *
             * @author Athanasios Krontiris
             */
            class rearrangement_search_query_t : public plan::motion_planning_query_t
            {

              public:

                rearrangement_search_query_t();
                virtual ~rearrangement_search_query_t();

                /** @copydoc motion_planning_query_t::init() */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);

                /** @brief Vector containing all possible initial poses in double form */
                std::vector< std::vector<double> > initial_poses;
                /** @brief Vector containing all possible target poses in double form */
                std::vector< std::vector<double> > target_poses;
                /** @brief Vector containing all extra poses in double form */
                std::vector< std::vector<double> > extra_poses;

            };
        }
    }
}

#endif

