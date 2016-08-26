/**
 * @file pebble_test_query.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2012, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Athanasios Krontiris, Andrew Dobson, Andrew Kimmel, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_PEBBLE_TEST_QUERY_HPP
#define	PRX_PEBBLE_TEST_QUERY_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"


namespace prx
{
    namespace util
    {
        class parameter_reader_t;
    }

    namespace packages
    {
        namespace pebble_motion
        {

            /**
             * A brief description of this class (no tag required).
             * 
             * A more detailed description of this class with multiple paragraphs delimited by blank lines. 
             */

            class pebble_test_query_t : public plan::motion_planning_query_t
            {

              public:
                pebble_test_query_t();
                virtual ~pebble_test_query_t();

                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                typedef std::pair<std::string, int> robot_position_t;
                std::vector<robot_position_t> initial_state;
                std::vector<robot_position_t> target_state;


              protected:


            };


        }
    }
}

#endif	// PRX_PEBBLE_TEST_QUERY_HPP
