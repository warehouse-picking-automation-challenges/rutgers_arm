/**
 * @file ground_truth_query_application.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Andrew Kimmel, Andrew Dobson, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "prx/planning/applications/planning_application.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"

#ifndef PRX_REPLANNING_GTA_APPLICATION_HPP
#define	PRX_REPLANNING_GTA_APPLICATION_HPP

namespace prx
{
    namespace util
    {
        class linear_distance_metric_t;
    }

    namespace plan
    {
        class replanning_application_t : public planning_application_t
        {
        public:
            replanning_application_t();
            virtual ~replanning_application_t();

            virtual void init(const util::parameter_reader_t* reader);

            virtual void execute();

            virtual void process_query_callback(const prx_simulation::query_msg& msg);
            virtual void process_ground_truth_callback(const prx_simulation::state_msg& msg);
            virtual void process_plant_locations_callback(const prx_simulation::plant_locations_msg& msg);

        private:
            util::linear_distance_metric_t* goal_metric;
            int num_queries;

            task_planner_t* task_planner;
            util::space_t* state_space;
            util::space_t* control_space;
            motion_planning_query_t* query;

            bool once;
            std::string my_plant;

            double planning_duration;
        };
    }
}

#endif

