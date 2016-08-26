/**
 * @file replanning_waypoints_controller.cpp 
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

#include "simulation/controllers/replanning_waypoints.hpp"
#include "prx/simulation/communication/planning_comm.hpp"
#include "prx/simulation/communication/simulation_comm.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/goals/radial_goal_region.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"

#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>

//DEBUG
#include "prx/utilities/math/2d_geometry/angle.hpp"

#include <iostream>

PLUGINLIB_EXPORT_CLASS( prx::packages::gta::replanning_waypoints_controller_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace sim::comm;
    
    namespace packages
    {
        namespace gta
        {
        
            replanning_waypoints_controller_t::replanning_waypoints_controller_t()
            {
                waypoint_index = 0;
                plan_index = 0;
                got_plan = smooth = sent_query = once = twice =  finished = false;
                goal_radius = PRX_ZERO_CHECK;
                iterations_counter = 0;
                timer_once = false;
                replanning_counter = 0;
                solution_time = 0;
                computation_time = computation_counter = 0.0;
            }

            replanning_waypoints_controller_t::~replanning_waypoints_controller_t()
            {
                foreach (radial_goal_region_t* goal, this->waypoints)
                {
                    delete goal;
                }
                child_state_space->free_point(future_end_state);
                child_state_space->free_point(get_state);
            }

            void replanning_waypoints_controller_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
            {
                PRX_DEBUG_S ("Replanning waypoints controller initialized");
                consumer_controller_t::init(reader, template_reader);
                child_state_space = subsystems.begin()->second.get()->get_state_space();

                smooth = parameters::get_attribute_as<bool>("smooth", reader, template_reader);
                proximity_threshold = parameters::get_attribute_as<double>("proximity_threshold", reader, template_reader, 1.0);
                planning_cycle = parameters::get_attribute_as<double>("planning_cycle", reader, template_reader, 1.0);
                replanning_frames = std::ceil(planning_cycle/sim::simulation::simulation_step);
                filename = parameters::get_attribute_as<std::string>("filename", reader, template_reader, "");
                
                std::vector<const parameter_reader_t*> goal_readers, template_goal_readers;

                if (reader->has_attribute("goals"))
                    goal_readers = reader->get_list("goals");

                std::string template_name;
                parameter_reader_t* child_template_reader = NULL;
                foreach(const parameter_reader_t* r, goal_readers)    
                {
                    PRX_DEBUG_S ("Goal reading!");
                    radial_goal_region_t* goal_region = new radial_goal_region_t();
                    if(r->has_attribute("goal/template"))
                    {
                        template_name = r->get_attribute("goal/template");
                        child_template_reader = new parameter_reader_t(template_name);  
                    }
                    goal_region->init(r->get_child("goal").get(), child_template_reader);
                    goal_region->link_space(child_state_space);
                    waypoints.push_back(goal_region);


                    if (child_template_reader == NULL)
                    {
                        delete child_template_reader;
                        child_template_reader = NULL;
                    }
                }

                if (template_reader)
                {
                    if (template_reader->has_attribute("goals"))
                    {
                        template_goal_readers = template_reader->get_list("goals");

                        foreach(const parameter_reader_t* r, goal_readers)    
                        {
                            radial_goal_region_t* goal_region = new radial_goal_region_t();
                            if(r->has_attribute("goal/template"))
                            {
                                template_name = r->get_attribute("goal/template");
                                child_template_reader = new parameter_reader_t(template_name);  
                            }
                            goal_region->init(r->get_child("goal").get(), child_template_reader);
                            goal_region->link_space(child_state_space);
                            waypoints.push_back(goal_region);
                            if (child_template_reader == NULL)
                            {
                                delete child_template_reader;
                                child_template_reader = NULL;
                            }
                        }

                    }
                }

                goal_radius = waypoints[waypoint_index]->get_radius();
                current_goal_state = waypoints[waypoint_index]->get_goal_points().front();

                future_end_state = child_state_space->alloc_point();
                child_state_space->copy_to_point(future_end_state);

                plan_state = child_state_space->alloc_point();
                current_state = child_state_space->alloc_point();

                get_state = child_state_space->alloc_point();
                
                

            //    sim_comm->create_new_plant_state_publisher(planning_node);

            }

            void replanning_waypoints_controller_t::propagate(const double simulation_step)
            {    
            //    PRX_INFO_S ("Propagate");
            //    if (!got_plan && twice)
            //    {
            //        PRX_LOG_ERROR ("Uh oh spaghetti Os %s", pathname.c_str());
            //    }
                // Solution has timed out
                if (solution_timer.measure() > 800 && !finished)
                {
                    if (!filename.empty())
                    {
                        std::ofstream fout;
                        fout.open(filename.c_str(), std::ios::app);
                        fout << "FAILURE" << std::endl;
                        fout.close();
                    }
                }
                controller_t::propagate(simulation_step);
                if (check_current_state())
                {
            //        PRX_WARN_S ("Increasing plan index from : " << plan_index);
                    plan_index++;
                }

            }

            bool replanning_waypoints_controller_t::check_current_state()
            {
                if (plan.size()!=0 && plan_index < plan.size())
                {
                    for( unsigned i=0; i <child_state_space->get_dimension(); ++i )
                    {
                        plan_state->at(i) = plan[plan_index].control->at(i);
                    }
                    child_state_space->copy_to_point(current_state);
                    double goal_distance = child_state_space->distance(current_state, current_goal_state);
                    if (goal_distance < proximity_threshold)
                    {
                        waypoint_index++;
                        if (waypoint_index < waypoints.size()-1)
                        {
                            goal_radius = waypoints[waypoint_index]->get_radius();
                            current_goal_state = waypoints[waypoint_index]->get_goal_points().front();
                        }
                    }
                    double plan_distance = child_state_space->distance(current_state, plan_state);
                    return (plan_distance< proximity_threshold);
                }
                else
                {
                    return false;
                }
            }

            void replanning_waypoints_controller_t::verify() const
            {
                PRX_ASSERT( controller_state_space->get_dimension() == 1 );

                controller_t::verify();
            }

            void replanning_waypoints_controller_t::copy_plan( const plan_t& inplan )
            {    
            //    PRX_ERROR_S ("Received plan at: " << pathname);
            //    PRX_INFO_S (" Publishing plant states" << planning_node);
                // Publish plant states


                if(!once)
                {

                    sent_query = false;
                    plan = inplan;
                    plan_index = 0;
                    once = true;
                    solution_timer.reset();

                }
                else
                {
                    if (!twice)
                    {
                        computation_timer.reset();
                    }
                    else
                    {
                        computation_counter++;
                        computation_time += computation_timer.measure_reset();
                    }
                    child_state_space->copy_point(future_end_state, inplan.get_end_state());
                    PRX_INFO_S (pathname << ":  end state will be: " << child_state_space->print_point(future_end_state));

                    queued_plans.push_back(inplan);
                    twice = true;
                }


                got_plan = true;
            //    PRX_WARN_S ("Done");

            //    iterations_counter++;

            }

            void replanning_waypoints_controller_t::query_planner()
            {
            //    PRX_DEBUG_S("Query planner");
                // If we haven't reached our final destination
                if (waypoint_index < waypoints.size())
                {
            //        PRX_WARN_S ("ITERATIION COUNTER: " << iterations_counter);
                    iterations_counter++;
            //        PRX_DEBUG_S("We're not done yet");
                    // Get the current state
                    child_state_space->copy_to_point(get_state);
                    PRX_DEBUG_COLOR (pathname << ": current state is: " << child_state_space->print_point(get_state), PRX_TEXT_BLACK);

            //        PRX_INFO_S (" Publishing ground truth " << planning_node);
                    // Publish a ground truth message
            //        sim_comm->publish_ground_truth((*state_memory[0]), planning_node);
            //        sim_comm->publish_plant_states(planning_node);

                    // Make a start state based on our future end state (of the current plan)
                    std::vector<double> start_state;
                    for (unsigned i = 0; i < child_state_space->get_dimension(); i++)
                    {
                        start_state.push_back(future_end_state->at(i));
                    }

                    // Query planning
                    double dist = child_state_space->distance(get_state, current_goal_state);
                    if (dist > goal_radius)
                    {
                        PRX_INFO_S ("Distance: " << dist << " vs. goal radius: " << goal_radius);
                        ((planning_comm_t*)plan_comm)->publish_planning_query(
                            start_state, waypoints[waypoint_index]->get_goal_vec(), waypoints[waypoint_index]->get_radius(), pathname, planning_node, true, smooth, homogeneous_setup,  false);
                    }
                    else if (waypoint_index < waypoints.size()-1)
                    {
            //            PRX_WARN_S ("Waypoint increase!");
            //            double trash;
            //            std::cin >> trash;
                        waypoint_index++;
                        goal_radius = waypoints[waypoint_index]->get_radius();
                        current_goal_state = waypoints[waypoint_index]->get_goal_points().front();
                        ((planning_comm_t*)plan_comm)->publish_planning_query(
                            start_state, waypoints[waypoint_index]->get_goal_vec(), waypoints[waypoint_index]->get_radius(), pathname, planning_node, true, smooth, homogeneous_setup,  false);
                    }
                    else
                    {
                        PRX_INFO_S("Total solution time: " << solution_time);
                        PRX_WARN_S ("No more goals to plan for!");
                    }
                }
                sent_query = true;
            //    planning_timer.reset();
            }

            void replanning_waypoints_controller_t::compute_control()
            {
                replanning_counter++;
                control_t* new_control;
                if (waypoint_index < waypoints.size())
                {
//                    if (planning_timer.measure() >= planning_cycle && twice)
                    if (replanning_counter >= replanning_frames && twice)
                    {
                        PRX_INFO_S (pathname << ":  Getting new plan from queue " );
                //        std::cin >> trash;
                        plan_index = 0;
                        sent_query = false;
//                        planning_timer.reset();
                        replanning_counter = 0;
                        solution_time += planning_cycle;
                        if (!queued_plans.empty())
                        {
            //                PRX_WARN_S ("QUeue not empty yaaay");
                            plan = queued_plans.front();
                            queued_plans.pop_front();
                            got_plan = true;
                        }
                        else
                        {
            //                PRX_LOG_ERROR ("Uh oh spaghetti Os %s", pathname.c_str());
                            got_plan = false;
                        }
                    }
                }
                if (!sent_query)
                    query_planner();
            //    PRX_INFO_S ("Compute control");
            //    if (!got_plan)
            //    {
            //        if (query_timer.measure() > 5)
            //            sent_query = false;
            //        new_control = contingency_plan.get_control_at(0);
            //    }
            //    else
                {
            //        PRX_INFO_S ("Mah planm index: " << plan_index);
                    if (plan_index < plan.size())
                        new_control = plan[plan_index].control;
                    else
                        new_control = NULL;
                    if (waypoint_index < waypoints.size())
                    {
                        if (new_control != NULL)
                        {

                //            PRX_INFO_S("********consumer control : " << output_control_space->print_point(new_control));
                            output_control_space->copy_point(computed_control, new_control);
                            last_control = new_control;
                //            PRX_WARN_S("keep_last_control : " << keep_last_control << "  c size:" << plan.steps.size());
                //            if(keep_last_control && plan.steps.size() == 0)
                //            {            
                //                last_control = new_control;
                //    //            PRX_LOG_INFO("Plan size : %u || Holding control", plan.steps.size() );
                //            }

                            *state_memory[0] = *state_memory[0] + simulation::simulation_step ;
                        }
                        else if (last_control != NULL)
                        {
            //                PRX_WARN_S ("Last control gooo!");
            //                PRX_INFO_S("Total solution time: " << solution_time.measure());
                            new_control = last_control;
                        }
                        else
                        {
            //                PRX_WARN_S ("Contingency plan gooooo");
                            new_control = contingency_plan.get_control_at(0);
                        }
                    }
                    else
                    {
                        if (!timer_once)
                        {
                            timer_once = true;
                            finished = true;
                            if (!filename.empty())
                            {
                                std::ofstream fout;
                                fout.open(filename.c_str(), std::ios::app);
                                fout << solution_time << " " << computation_time/computation_counter << std::endl;
                                fout.close();
                            }
                            PRX_INFO_S("System[" << pathname << "]: Total solution time: " << solution_time);
                        }
                        new_control = last_control;
                    }
                }



                output_control_space->copy_from_point(new_control);
                subsystems.begin()->second->compute_control();

            }
            
            bool replanning_waypoints_controller_t::is_finished()
            {
                return finished;
            }
            
            double replanning_waypoints_controller_t::get_solution_time()
            {
                return solution_time;
            }
        }
    }
}