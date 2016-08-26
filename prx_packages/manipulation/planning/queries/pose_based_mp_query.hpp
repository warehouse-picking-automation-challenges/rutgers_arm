/**
 * @file pose_based_mp_query.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2016, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_POSE_BASED_MOTION_PLANNING_QUERY_HPP
#define PRX_POSE_BASED_MOTION_PLANNING_QUERY_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/modules/heuristic_search/constraints_astar_search.hpp"

#include "utilities/definitions/manip_defs.hpp"
#include "planning/modules/grasp.hpp"

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
    }

    namespace packages
    {
        namespace manipulation
        {

            /**
             * @anchor pose_based_mp_query_t
             *
             * 
             * @brief <b> A motion planning query with a pose as a goal. </b>
             *
             * @author Andrew Dobson
             */
            class pose_based_mp_query_t : public plan::motion_planning_query_t
            {
              public:

                pose_based_mp_query_t();
                ~pose_based_mp_query_t();

                void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                virtual void setup(sim::state_t* input_start_state, plan::search_mode_t input_search_mode, const util::config_t& target_config, std::string target_effector, const util::constraints_t* input_active_constraints = NULL, util::constraints_t* input_path_constraints = NULL, bool should_update_constraints = false);
                
                const util::config_t& get_target_pose();
                const std::string& get_end_effector();
                bool is_pose_query();

                virtual void clear();

                //------------------------//
                //--  Return Variables  --//
                //------------------------//

            protected:
                util::config_t target_pose;
                std::string end_effector;
                
                bool pose_query;
            };
        }
    }
}

#endif
