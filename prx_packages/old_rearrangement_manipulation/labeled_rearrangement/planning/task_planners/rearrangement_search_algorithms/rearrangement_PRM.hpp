/**
 * @file rearrangement_prm.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#pragma once

#ifndef PRX_REARRANGEMENT_PRM_HPP
#define PRX_REARRANGEMENT_PRM_HPP

#include "planning/task_planners/rearrangement_search_algorithm.hpp"

namespace prx
{
    namespace util
    {
        class bounds_t;
        class multiple_goal_states_t;
        class statistics_t;
    }
    
    namespace plan
    {
        class motion_planning_query_t;
    }

    namespace packages
    {
        namespace labeled_rearrangement_manipulation
        {
            using namespace baxter;
            using namespace manipulation;

            class rearrangement_search_query_t;           
            class rearrangement_search_specification_t;
            class rearrangement_primitive_t;
            class rearrangement_primitive_specification_t;
            class rearrangement_query_t;
            class rearrangement_path_planner_t;

            /**
             * 
             * 
             * @autors Athanasios Krontiris
             */
            class rearrangement_prm_t : public rearrangement_search_algorithm_t
            {

              public:
                rearrangement_prm_t();
                virtual ~rearrangement_prm_t();

                /** @copydoc task_planner_t::init(const util::parameter_reader_t*, const util::parameter_reader_t*) */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);
                
                /** @copydoc task_planner_t::setup() */
                virtual void setup();

                /** @copydoc task_planner_t::resolve_query() */
                virtual void resolve_query();

              protected:
                bool try_connect(std::vector<unsigned>& arrangement);

                bool connect_nodes(util::undirected_vertex_index_t v, util::undirected_vertex_index_t u);                
                
                void update_stats(bool found_path);
               
                double primitive_timer;
                util::undirected_vertex_index_t v_start;
                util::undirected_vertex_index_t v_target;
                std::string path_file;
            };
        }

    }

}
#endif	
