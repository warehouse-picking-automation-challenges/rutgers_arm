/**
 * @file multi_shot_query.hpp
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
#ifndef PRX_MULTI_SHOT_QUERY_HPP
#define PRX_MULTI_SHOT_QUERY_HPP


#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"


namespace prx
{
    namespace plan
    {

        /**
         * @anchor multi_shot_query_t
         *
         * Task planner query which is used by \ref multi_shot_planner_t class.  This
         * query is a tag class which simply inherits from the \ref motion_planning_query_t.
         *
         * @brief <b> Task planner query used for single-shot planning. </b>
         *
         * @author Andrew Kimmel
         */
        class multi_shot_query_t : public motion_query_t
        {
          public:
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);
            
            std::vector< std::vector< double > > goal_vectors;
        };

    }
}


#endif

