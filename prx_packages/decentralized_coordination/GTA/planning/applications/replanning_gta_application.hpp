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
#include "prx/planning/queries/query.hpp"
#include "planning/task_planners/deconfliction/replanning_gta.hpp"

#ifndef PRX_REPLANNING_GTA_APPLICATION_HPP
#define	PRX_REPLANNING_GTA_APPLICATION_HPP

namespace prx
{
    namespace util
    {
        class linear_distance_metric_t;
    }
    
    namespace packages
    {
        namespace gta
        {
            class replanning_gta_application_t : public plan::planning_application_t
            {
                public:
                    replanning_gta_application_t();
                    virtual ~replanning_gta_application_t();

                    virtual void init(const util::parameter_reader_t* reader);

                    virtual void execute();

                    virtual void process_query_callback(const prx_simulation::query_msg& msg);
                    virtual void process_ground_truth_callback(const prx_simulation::state_msg& msg);
                    virtual void process_plant_locations_callback(const prx_simulation::plant_locations_msg& msg);


                private:


                    util::linear_distance_metric_t* goal_metric;
                    double avg_construction_time, avg_planning_time, avg_resolve_time, avg_visualize_time;
                    double total_construction_time, total_planning_time, total_resolve_time, total_visualize_time;

                    double total_nodes, total_time, total_steps;
                    double avg_number_nodes, avg_time, avg_steps;
                    int num_queries;

                    // GTA stuff
                    int max_neighbors, current_neighbor_size;
                    util::space_t* gta_state_space;
                    util::space_t* gta_control_space;
                    replanning_gta_planner_t* gta_planner;
                    replanning_gta_query_t* gta_query;
                    plan::stopping_criteria_t gta_s_criteria;

                    bool once;
                    bool received_ground_truth;
                    std::string my_plant;

                    // Replanning parameters
                    double planning_duration;
                    
                   


            };
        }
    }
}

#endif

