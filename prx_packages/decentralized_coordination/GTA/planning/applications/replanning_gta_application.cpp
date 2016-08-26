/**
 * @file ground_truth_query_application.cpp 
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

#include "planning/applications/replanning_gta_application.hpp"
#include "prx/planning/communication/planning_comm.hpp"
#include "prx/planning/communication/simulation_comm.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/problem_specifications/specification.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/task_planners/task_planner.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/modules/stopping_criteria/element/goal_criterion.hpp"

#include "prx/utilities/goals/radial_goal_region.hpp"
#include "prx/utilities/distance_metrics/linear_metric/linear_distance_metric.hpp"
#include "prx/utilities/graph/abstract_node.hpp"

#include "prx/utilities/communication/tf_broadcaster.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"

#include "prx_simulation/query_msg.h"
#include "prx_simulation/state_msg.h"
#include "prx_simulation/plant_locations_msg.h"
        
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS( prx::packages::gta::replanning_gta_application_t, prx::plan::planning_application_t)  
    
namespace prx
{
    using namespace util;
    using namespace plan;
    using namespace sim;
    using namespace plan::comm;
    
    namespace packages
    {
        namespace gta
        {

            replanning_gta_application_t::replanning_gta_application_t() 
            {
                goal_metric = new linear_distance_metric_t();
                avg_construction_time = avg_planning_time = avg_resolve_time = avg_visualize_time = 0.0;
                total_construction_time = total_planning_time = total_resolve_time = total_visualize_time = 0.0;
                total_nodes = total_time = total_steps = 0.0;
                avg_number_nodes = avg_time = avg_steps = 0.0;
                num_queries = 0;
                gta_query = NULL;
                received_ground_truth = false;
                once = false;
            }

            replanning_gta_application_t::~replanning_gta_application_t() 
            {
                delete gta_query;
                delete goal_metric;
            }

            void replanning_gta_application_t::init(const parameter_reader_t* reader)
            {

                PRX_DEBUG_S("Before init of planning application");
                planning_application_t::init(reader);
                PRX_DEBUG_S("After init of planning application");
                //planning_application_t::initialize_spaces("ground_truth");

                model->use_context(consumer_to_space_name_mapping[consumer_path]);
                gta_state_space = model->get_state_space();
                gta_control_space = model->get_control_space();

                my_plant = reader->get_attribute("my_plant");
                planning_duration = reader->get_attribute_as<double>("planning_duration");

            //    if (reader->has_attribute("gta_query"))
            //    {
            //        PRX_WARN_S ("we have gta query!");
            //        const parameter_reader_t* template_reader = NULL;  
            //        gta_query = new gta_query_t();
            //        if(reader->has_attribute("template"))
            //        {
            //            std::string template_name = reader->get_attribute("template");
            //            template_reader = new parameter_reader_t(template_name);  
            //        }
            //        gta_query->init_with_space(reader->get_child("gta_query").get(), template_reader, model->get_state_space());
            //        gta_query->state_space = gta_state_space;
            //        gta_query->control_space = gta_control_space;
            ////        gta_query->link_spaces(gta_state_space, gta_control_space);
            //        gta_query->print();
            //    }

                gta_query = new replanning_gta_query_t(gta_state_space, gta_control_space, my_plant);
                //root_specifications[0]->set_stopping_criterion(&gta_s_criteria);

                gta_planner = dynamic_cast<replanning_gta_planner_t*>(this->root_task);
                gta_planner->set_planning_duration(planning_duration);
                if (gta_planner == NULL || gta_query == NULL)
                {
                    PRX_FATAL_S ("Either null gta planner or null gta query");
                }

            }

            void replanning_gta_application_t::execute()
            {
            //    gta_planner->link_query(gta_query);
            //    gta_planner->setup();
            //    gta_planner->execute();
            //    ((planning::communication::planning_comm_t*)planning::communication::plan_comm)->publish_plan(consumer_path, gta_query->plan);
            //    gta_planner->update_visualization();
            //    ((planning::communication::visualization_comm_t*)planning::communication::vis_comm)->send_geometries();

            //    if (!root_queries.empty())
            //    {
            //        this->root_task->link_query(root_queries[0]);    
            //        this->root_task->setup();
            //        this->root_task->execute();
            //
            //        if(visualize)
            //        {
            //            PRX_WARN_S("Updating visualization geoms.");
            //
            //            this->root_task->update_visualization();
            //
            //            ((planning::communication::visualization_comm_t*)planning::communication::vis_comm)->send_geometries();
            //        }
            //
            //        ((planning::communication::planning_comm_t*)planning::communication::plan_comm)->publish_plan(consumer_path,dynamic_cast<motion_planning_query_t*>(root_queries[0])->plan );
            //    }
            //    
            //    
                PRX_DEBUG_S ("Execute complete");



            }

            void replanning_gta_application_t::process_query_callback(const prx_simulation::query_msg& msg)
            {
            //    while (!received_ground_truth)
            //    {
            //        
            //    }
            //    if (received_ground_truth && !once)
                {
                    this->process_plant_locations_callback(msg.plant_locations);
                    received_ground_truth = false;
                    if (once)
                        num_queries++;
                    sys_clock_t timer_thing, total_time_thing;
                    total_time_thing.reset();
                    PRX_INFO_S ("Process query callback!\n Consumer:" << msg.consumer << "\n Goal region radius: " << msg.goal_region_radius);
            //        for(unsigned i = 0; i < msg.goal.size(); i++)
            //        {
            //            PRX_INFO_S("Goal : " << i << " is: " << msg.goal[i]);
            //        }
                    double construction_time = 0, visualize_time = 0;
                    timer_thing.reset();

                    // Construct the query
                    model->use_context(consumer_to_space_name_mapping[consumer_path]);

            //        // Set start state
            //        gta_state_space->set_from_vector(msg.start);
            //
            //        state_t* start_state = gta_state_space->alloc_point();
            //
            //        PRX_ERROR_S ("Start state :" << gta_state_space->print_point(start_state,3));
            //
            //
            //
            //        PRX_DEBUG_S ("Creating motion planning query");
            //        // Create a new motion planning query
            //        gta_query->set_start(start_state);
            //        gta_query->set_start_vec(msg.start);


                //    gta_planner->link_query(gta_query);
                //    gta_planner->setup();
                //    gta_planner->execute();
                    // Query the planner underneath me

                    if (!once)
                    {    
                        root_specifications[0]->link_spaces(gta_state_space, gta_control_space);
                        root_specifications[0]->setup(model);
                        gta_planner->link_specification(root_specifications[0]);
                        gta_state_space->set_from_vector(msg.start);
                        state_t* start_state = gta_state_space->alloc_point();
                        gta_state_space->set_from_vector(msg.start, start_state);
                        gta_query->set_start(start_state);
                        gta_query->set_start_vec(msg.start);
                        // Set the goal and the distance metric
                        radial_goal_region_t* new_goal = new radial_goal_region_t();

                    //    new_metric->link_space(gta_state_space);
                        PRX_DEBUG_S ("Set goal");
                        new_goal->set_goal(goal_metric, msg.goal, msg.goal_region_radius);
                        gta_query->set_goal(new_goal);
                        gta_query->link_spaces(gta_state_space, gta_control_space);
                        PRX_DEBUG_S("Link query");
                        gta_planner->link_query(gta_query);


                    }
                    PRX_DEBUG_S("Setup");
                    gta_planner->setup();
                    construction_time = timer_thing.measure_reset();
                    if (gta_planner->execute())
                    {
//                        PRX_WARN_S ("Attempting to resolve query");
                        PRX_PRINT ("Execute: " << timer_thing.measure(), PRX_TEXT_BROWN);
                        gta_planner->resolve_query();
                    }
                    else
                    {
                        PRX_WARN_S("Something went wrong!");
                        gta_planner->resolve_query();
                    }
                //    try
                //    {
                //        root_task->execute();
                //    }
                //    catch(stopping_criteria_t::stopping_criteria_satisfied e)
                //    {
                //        planning_time =timer_thing.measure_reset();
                //        PRX_ERROR_S("Stopping criteria satisfied " << e.what());
                //        root_task->resolve_query();
                //        resolve_time = timer_thing.measure_reset();
                //    }
                //    catch(stopping_criteria_t::interruption_criteria_satisfied e)
                //    {
                //        planning_time =timer_thing.measure_reset();
                //        PRX_ERROR_S("Interruption criteria!");
                //        root_task->resolve_query();
                //        resolve_time = timer_thing.measure_reset();
                //    }
                    
                    PRX_PRINT ("Execute + resolve query: " << timer_thing.measure(), PRX_TEXT_BROWN);

                    if(visualize)
                    {
                //        PRX_WARN_S("Updating visualization geoms.");

//                        PRX_LOG_ERROR("No visualize!");
                        gta_planner->update_visualization();

                        ((visualization_comm_t*)vis_comm)->send_geometries();
                        visualize_time = timer_thing.measure_reset();
                    }

            //        while (total_time_thing.measure() < planning_duration*duration_scaling)
            //        {
            //            
            //        }
                    received_ground_truth = false;
                    ((planning_comm_t*)plan_comm)->publish_plan(consumer_path,gta_query->plan );
                    if (once)
                    {
                        total_planning_time += total_time_thing.measure();
//                        PRX_INFO_S ("AVG TOTAL TIME: " << total_planning_time/num_queries << " and total time: " << total_time_thing.measure());
                    }
                    once = true;
                //    const rrt_statistics_t* stats = dynamic_cast<const rrt_statistics_t*>(root_task->get_statistics());
                //    total_construction_time += construction_time;
                //    total_planning_time += planning_time;
                //    total_resolve_time += resolve_time;
                //    total_visualize_time += visualize_time;
                //    total_nodes += stats->num_vertices;
                //    total_steps += stats->steps;
                //    total_time += total_time_thing.measure();
                //    
                //    avg_construction_time = total_construction_time / num_queries;
                //    avg_number_nodes = total_nodes / num_queries;
                //    avg_planning_time = total_planning_time / num_queries;
                //    avg_resolve_time = total_resolve_time / num_queries;
                //    avg_visualize_time = total_visualize_time / num_queries; 
                //    avg_steps = total_steps / num_queries;
                //    avg_time = total_time / num_queries;
                //    double total_time = avg_planning_time + avg_resolve_time + avg_visualize_time;
                //    PRX_DEBUG_S ("\n\nSTATS! \nQuery construction time: " << construction_time << "\n Planning time: " << planning_time << "\n Resolve time: " << resolve_time << "\n Visualize time:"
                //             << avg_visualize_time << "\n Number of RRT nodes: " << stats->num_vertices << ", Steps: " << stats->steps << ", Planning Time: " << stats->time << ", Total Time: " << total_time_thing.measure() );
                //    
                //    
                //    PRX_ERROR_S ("\n\nAVERAGED STATS! \nQuery construction time: " << avg_construction_time << "\n Planning time: " << avg_planning_time << "\n Resolve time: " << avg_resolve_time << "\n Visualize time:"
                //             << avg_visualize_time << "\n Number of RRT nodes: " << avg_number_nodes << ", Steps: " << avg_steps << ", Planning Time: " << avg_time << ", Total Time: " << total_time );
                //    
                }
            }

            void replanning_gta_application_t::process_ground_truth_callback(const prx_simulation::state_msg& msg)
            {
            //    PRX_ERROR_S ("Process ground truth callback!");
            //    model->use_space("no_embedding");
            //    state_t* ground_truth = model->get_state_space()->alloc_point();
            //    model->get_state_space()->set_from_vector(sim_to_plan_state(msg.elements, "ground_truth"), ground_truth);
            //    model->push_state(ground_truth);
            //    model->get_state_space()->free_point(ground_truth);

            //    // Set up our neighborhood for GTA
            //    neighbor_query->clear();
            //    // TODO: Make this more general. Right now this assumes intervals of 2 (X,Y) in the state
            //    int counter = 0;
            //    for (unsigned i = 0; i < msg.elements.size(); i+=2)
            //    {
            //        abstract_node_t blah;
            //        blah.point->at(i) = i;
            //        neighbor_nodes[counter]->point->at(i) = msg.elements[i];
            //        neighbor_nodes[counter]->point->at(i+1) = msg.elements[i+1];
            //    }
            }

            void replanning_gta_application_t::process_plant_locations_callback(const prx_simulation::plant_locations_msg& msg)
            {
                PRX_INFO_S ("Process plant locations");
                gta_query->process_plant_locations_callback(msg);
                received_ground_truth = true;
            //    current_neighbor_size = msg.plant_states.size();
            //    if (current_neighbor_size > max_neighbors)
            //    {
            //        PRX_WARN_S ("Current neighbor size exceeds max neighbor size! " << current_neighbor_size << " vs. " << max_neighbors);
            //        max_neighbors = current_neighbor_size + 10;
            //        neighbor_locations.resize(max_neighbors);
            //    }
            //    for(int i = 0; i < current_neighbor_size; i++)
            //    {
            //        neighbor_locations[i].clear();
            //        std::copy(msg.plant_states[i].elements.begin(), msg.plant_states[i].elements.end(), neighbor_locations[i].begin());
            //    }

            //    // TEST! This will make sure things work!
            //    for (int i = 0; i < current_neighbor_size; i++)
            //    {
            //        for (int j = 0; j < msg.plant_states[i].elements.size(); j++)
            //        {
            //            PRX_WARN_S ("I got a: " << msg.plant_states[i].elements[j]);
            //        }
            //    }
            }
        }
    }
}
