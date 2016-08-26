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

#include "simulation/controllers/GTA_replanning.hpp"
#include "prx/simulation/communication/planning_comm.hpp"
#include "prx/simulation/communication/simulation_comm.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/goals/radial_goal_region.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/communication/tf_broadcaster.hpp"

#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>

//DEBUG
#include "prx/utilities/math/2d_geometry/angle.hpp"

#include <iostream>

PLUGINLIB_EXPORT_CLASS( prx::packages::gta::GTA_replanning_controller_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace sim::comm;
    
    namespace packages
    {
        namespace gta
        {
        
            GTA_replanning_controller_t::GTA_replanning_controller_t()
            {
                waypoint_index = 0;
                smooth =  once = twice =  finished = false;
                goal_radius = PRX_ZERO_CHECK;
                iterations_counter = 0;
                timer_once = false;
                replanning_counter = 0;
                solution_time = 0;
                computation_time = computation_counter = 0.0;
                path_length = 0.0;
            }

            GTA_replanning_controller_t::~GTA_replanning_controller_t()
            {
                foreach (radial_goal_region_t* goal, this->waypoints)
                {
                    delete goal;
                }
                child_state_space->free_point(future_end_state);
                child_state_space->free_point(get_state);
                child_state_space->free_point(previous_state);
            }

            void GTA_replanning_controller_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
            {
                PRX_DEBUG_COLOR ("GTA Replanning controller initialized", PRX_TEXT_RED);
                consumer_controller_t::init(reader, template_reader);
                child_state_space = subsystems.begin()->second.get()->get_state_space();
                vo_subsystem = dynamic_cast<VO::VO_controller_t*>(subsystems.begin()->second.get());

                smooth = parameters::get_attribute_as<bool>("smooth", reader, template_reader);
                planning_cycle = parameters::get_attribute_as<double>("planning_cycle", reader, template_reader, 1.0);
                replanning_frames = std::ceil(planning_cycle/sim::simulation::simulation_step);
                filename = parameters::get_attribute_as<std::string>("filename", reader, template_reader, "");
                visualizes_text = parameters::get_attribute_as<bool>("visualize_text", reader, template_reader, false);
                
                std::vector<const parameter_reader_t*> goal_readers, template_goal_readers;

                if (reader->has_attribute("goals"))
                    goal_readers = reader->get_list("goals");

                std::string template_name;
                parameter_reader_t* child_template_reader = NULL;
                foreach(const parameter_reader_t* r, goal_readers)    
                {
                    PRX_ERROR_S ("Goal reading!");
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

                get_state = child_state_space->alloc_point();
                previous_state = child_state_space->alloc_point();
                
                contingency_plan.augment_plan(planning_cycle, false);
            //    sim_comm->create_new_plant_state_publisher(planning_node);

            }
            
            void GTA_replanning_controller_t::set_points_back()
            {
                contingency_plan.clear();
                child_state_space->copy_to_point(get_state);
                child_state_space->copy_to_point(previous_state);
                child_state_space->copy_to_point(future_end_state);
                contingency_plan.augment_plan(planning_cycle, false);
            }

            void GTA_replanning_controller_t::propagate(const double simulation_step)
            {    
                replanning_counter++;
                
                if (solution_timer.measure() > 800 && !finished)
                {
                    if (!filename.empty())
                    {
                        finished = true;
                    }
                }
                controller_t::propagate(simulation_step);
                if (replanning_counter >= replanning_frames && twice)
                {
                    replanning_counter = 0;
                    solution_time += planning_cycle;
                    queried_planner = false;
                }

            }

            void GTA_replanning_controller_t::verify() const
            {
                PRX_ASSERT( controller_state_space->get_dimension() == 1 );

                controller_t::verify();
            }

            void GTA_replanning_controller_t::copy_plan( const plan_t& inplan )
            {    
            //    PRX_ERROR_S ("Received plan at: " << pathname);
            //    PRX_INFO_S (" Publishing plant states" << planning_node);
                // Publish plant states


                if(!once)
                {

                    queried_planner = false;
                    plan = contingency_plan;
                    once = true;
                    solution_timer.reset();
                    visualize_goals();

                }
                else
                {
                    if (!twice)
                    {
                        computation_timer.reset();
                        twice = true;
                        if (visualizes_text)
                            visualize_text();
                    }
                    else
                    {
                        computation_counter++;
                        computation_time += computation_timer.measure_reset();
                        child_state_space->copy_point(future_end_state, inplan.get_end_state());
                    }
                    
//                    PRX_INFO_S (pathname << ":  end state will be: " << child_state_space->print_point(future_end_state));
                    plan += inplan;
//                    if (pathname =="consumer05")
//                    {
//                        foreach (plan_step_t step, plan)
//                        {
//                            PRX_DEBUG_COLOR("Step: " << output_control_space->print_point(step.control) << " , dur: " << step.duration, PRX_TEXT_GREEN);
//                        }
//                    }
                    
                }


                got_plan = true;
            //    PRX_WARN_S ("Done");

            //    iterations_counter++;

            }

            void GTA_replanning_controller_t::query_planner()
            {
            //    PRX_DEBUG_S("Query planner");
                // If we haven't reached our final destination
                if (waypoint_index < waypoints.size())
                {
                    got_plan = false;
            //        PRX_WARN_S ("ITERATIION COUNTER: " << iterations_counter);
                    iterations_counter++;
            //        PRX_DEBUG_S("We're not done yet");
                    // Get the current state
                    
                    path_length += child_state_space->distance(previous_state, get_state);
                    child_state_space->copy_point(previous_state, get_state);
                    child_state_space->copy_to_point(get_state);
//                    PRX_WARN_S (pathname << ": current state is: " << child_state_space->print_point(get_state));

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
//                        PRX_INFO_S ("Distance: " << dist << " vs. goal radius: " << goal_radius);
                        ((planning_comm_t*)plan_comm)->publish_planning_query(
                            start_state, waypoints[waypoint_index]->get_goal_vec(), waypoints[waypoint_index]->get_radius(), pathname, planning_node, true, smooth,  false, homogeneous_setup);
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
                            start_state, waypoints[waypoint_index]->get_goal_vec(), waypoints[waypoint_index]->get_radius(), pathname, planning_node, true, smooth, false, homogeneous_setup);
                    }
                    else
                    {
                        got_plan = true;
                        if (!timer_once)
                        {
                            timer_once = true;
                            finished = true;
                            
                            vo_subsystem->fully_reciprocal = true;
                            vo_subsystem->toggle_vanish();
                            std::vector<std::string> names;
                            names.push_back(ros::this_node::getName() + "/" + pathname +"/goal");
                            std::vector<config_t> confs;
                            std::vector<vector_t> colors;
                            vector_t color(4);
                            color[0] = 1.0; color[1] = 0.0; color[2] = 0.4; color[3] = 0.8;
                            colors.push_back(color);

                            ((visualization_comm_t*)vis_comm)->update_info_geoms(names, confs, colors, false);
                            
                            plan.clear();
                            if (!filename.empty())
                            {
                                std::ofstream fout;
                                fout.open(filename.c_str(), std::ios::app);
                                fout << solution_time << " " << computation_time/computation_counter << std::endl;
                                fout.close();
                            }
                            PRX_WARN_S("System[" << pathname << "]: Total solution time: " << solution_time);
                        }
                        PRX_INFO_S("Total solution time: " << solution_time);
                        PRX_WARN_S ("No more goals to plan for!");
                    }
                }
                
                queried_planner = true;
                
            //    planning_timer.reset();
            }

            void GTA_replanning_controller_t::compute_control()
            {
//                PRX_DEBUG_COLOR ("Begin compute control", PRX_TEXT_BLUE);
                control_t* new_control = plan.get_next_control( simulation::simulation_step );
                if (!queried_planner && !finished)
                {
            //        PRX_DEBUG_COLOR("Query planner", PRX_TEXT_MAGENTA);
                    query_planner();
                }

                // Case 1: Our plan has given us a non-NULL control
                if(new_control != NULL)
                {
//                    if (pathname == "simulator/consumer00")
//                    {
//                        PRX_DEBUG_COLOR("New control not NULL", PRX_TEXT_CYAN);
//                    }
                    //        PRX_INFO_S("********consumer control : " << output_control_space->print_point(new_control));
                    output_control_space->copy_point(computed_control, new_control);

            //        PRX_WARN_S("keep_last_control : " << keep_last_control << "  c size:" << plan.steps.size());
                    if(plan.size() == 0)
                    {            
                        last_control = new_control;
            //            PRX_LOG_INFO("Plan size : %u || Holding control", plan.steps.size() );
                    }

                    _time = _time + simulation::simulation_step ;
                }
                // Case 2: Last control has been set to a valid control, and we want to keep our last control
                else if (last_control != NULL && keep_last_control)
                {
//                    if (pathname == "simulator/consumer00")
//                    {
//                        PRX_DEBUG_COLOR("Use last control", PRX_TEXT_LIGHTGRAY);
//                    }
                    new_control = last_control;
                }
                else if (keep_last_state)
                {
                    new_control = get_state;
                }
                // Case 3: If all else fails, use the contingency plan (system specific)
                else
                {
//                    if (pathname == "simulator/consumer00")
//                    {
//                        PRX_DEBUG_COLOR("Contingency plan", PRX_TEXT_BROWN);
//                    }
                    new_control = contingency_plan.get_control_at(0);
                }

            //    if (queried_planner)
            //    {
            //        PRX_DEBUG_COLOR("Queried planner true!", PRX_TEXT_RED);
            //    }
            //    else
            //    {
            //        PRX_DEBUG_COLOR("Queried planner false!", PRX_TEXT_GREEN);
            //    }
                // Check if we still need to query planner


//                if (pathname == "simulator/consumer00")
//                {
//                        PRX_DEBUG_COLOR("New control: " << this->output_control_space->print_point(new_control), PRX_TEXT_MAGENTA);
//                }
                output_control_space->copy_from_point(new_control);
                subsystems.begin()->second->compute_control();

            }
            
            bool GTA_replanning_controller_t::is_finished()
            {
                return finished;
            }
            
            double GTA_replanning_controller_t::get_solution_time()
            {
                return solution_time;
            }
            
            void GTA_replanning_controller_t::visualize_goals()
            {
                std::vector<double> params;

                PRX_DEBUG_COLOR("Next Step : ", PRX_TEXT_BROWN );
                std::string full_name = pathname + "/goal";


                std::string name = ros::this_node::getName() + "/" + pathname;
                params.push_back(10);
//                vector_t colorr(3);
//                colorr.set(0.5,0.2,0.8);

                geometry_info_t geom_info(name, "goal", PRX_SPHERE, params, "blue");

                config_t goal_conf;
                std::vector<double> vec = waypoints[waypoint_index]->get_goal_vec();
                goal_conf.set_position(vec[0], vec[1], 2.0);
                goal_conf.set_orientation(0,0,0,1);


                hash_t<std::string, geometry_info_t> geoms;
                geoms[ geom_info.full_name ] = geom_info;
                ((visualization_comm_t*)vis_comm)->send_extra_geometries(geoms);
                
                std::vector<std::string> names;
                names.push_back(name+"/goal");
                std::vector<config_t> confs;
                confs.push_back(goal_conf);
                std::vector<vector_t> colors;
                
                ((visualization_comm_t*)vis_comm)->update_info_geoms(names, confs, colors, false);
                
            }
            
            void GTA_replanning_controller_t::visualize_text()
            {
                std::string name = ros::this_node::getName() + "/" + pathname;
                
                std::vector<double> relative_position;
                relative_position.push_back(0);
                relative_position.push_back(0);
                relative_position.push_back(10);
                std::vector<double> color;
                color.push_back(0.0);
                color.push_back(0.0);
                color.push_back(0.9);
                color.push_back(1.0);
                ((visualization_comm_t*)vis_comm)->visualize_scene_text(name+"/goal", "", name+"/goal", relative_position, color, 10, "");
            }
            
            std::string GTA_replanning_controller_t::get_planning_node_name()
            {
                return planning_node;
            }
            
            double GTA_replanning_controller_t::get_path_length() const
            {
                return path_length;
            }
        }
    }
}