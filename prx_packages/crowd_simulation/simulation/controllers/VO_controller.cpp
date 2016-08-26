/**
 * @file VO_controller.cpp
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


#include "prx/utilities/definitions/string_manip.hpp"

#include "simulation/controllers/VO_controller.hpp"

#include "prx/simulation/system_graph.hpp"
#include "prx/simulation/systems/plants/plant.hpp"
#include "prx/simulation/communication/visualization_comm.hpp"

#include <math.h>
#include <pluginlib/class_list_macros.h>
#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/range/adaptor/map.hpp> //adaptors
#include <boost/assign/list_of.hpp>

PLUGINLIB_EXPORT_CLASS( prx::packages::crowd::VO_controller_t, prx::sim::system_t)

namespace prx
{
    namespace sim
    {
        using namespace comm;
    }
    
    using namespace util;
    using namespace sim;

    namespace packages
    {
        namespace crowd
        {
            const unsigned int VO_controller_t::X = 0;
            const unsigned int VO_controller_t::Y = 1;

            const unsigned int VO_controller_t::V = 0;
            const unsigned int VO_controller_t::THETA = 1;

            double VO_controller_t::cc_time = 0;
            unsigned VO_controller_t::cc_calls = 0;

            VO_controller_t::VO_controller_t()
            {
                finished = vanished = select_current = added_to_metric = goal_set = false;
                nr_hrvos = 0;
                collisions = 0;

                dist_to_goal = PRX_INFINITY;
                progress_counter = 100;
                resolve_counter = 0;
                point_a.resize(2);
                point_b.resize(2);
                point_c.resize(2);
                point_d.resize(2);
                obst_b.resize(2);
                obst_cent.resize(2);
                computed_vector_control.resize(2);
                desired_control.resize(2);
                curr_vel.resize(2);
                zero_center.resize(2);
                zero_center.zero();
                last_pos.resize(2);

                prev_control_1.resize(2);
                prev_control_2.resize(2);
                prev_control_3.resize(2);

                safety_horizon = PRX_INFINITY;

                if (std::strcmp(ros::this_node::getName().c_str(),"/planning"))
                    planning = false;
                else
                    planning = true;

                visualize_VOs = false;
                computed_vos = false;
                was_vis = false;
                near_goal = false;

                _Cx = _Cy = _Cz = 0;
                control_memory = { &_Cx , &_Cy , &_Cz };
                input_control_space = new space_t( "XYZ", control_memory );

                selected = false;
                fully_reciprocal = false;
                once = false;

                in_backtracking = false;
                b_step_count = 0; b_back_count = 0;

                goal_set = false;
                distance_threshold = -1.0;
                
                debug_colors.push_back("blue");
                debug_colors.push_back("red");
                debug_colors.push_back("yellow");
                debug_colors.push_back("green");
                debug_colors.push_back("teal");
                debug_colors.push_back("purple");
                debug_colors.push_back("grey");
                debug_colors.push_back("pink");
                debug_colors.push_back("dark_green");
                debug_colors.push_back("magenta");
                debug_colors.push_back("brown");
                debug_colors.push_back("indigo");
                debug_colors.push_back("orange");
            }


            VO_controller_t::~VO_controller_t()
            {
            }

            void VO_controller_t::pre_init( void* controller )
            {
                parent_controller = controller;
            }

            void VO_controller_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
            {
                stats_clock.reset();
                
                radius = parameters::get_attribute_as<double>("radius", reader, template_reader);
                    //vo_radius = parameters::get_attribute_as<double>("vo_radius", reader, template_reader);
                inactive = parameters::get_attribute_as<bool>("inactive", reader, template_reader, false);
                minimally_invasive = parameters::get_attribute_as<bool>("minimally_invasive", reader, template_reader, true);

                backtracking = parameters::get_attribute_as<bool>("backtracking", reader, template_reader, false);
                non_vo_avoidance = parameters::get_attribute_as<bool>("non_vo_avoidance", reader, template_reader, false);
                std::string csc_mode_string = parameters::get_attribute_as<std::string>("csc_mode", reader, template_reader, "MIXED_CONTROL");
                if (csc_mode_string == "MIXED_CONTROL")
                    csc_mode = MIXED_CONTROL;
                else if (csc_mode_string == "BLENDED_HISTORY")
                    csc_mode = BLENDED_HISTORY;
                else
                    csc_mode = BASIC;

                step_count_limit = parameters::get_attribute_as<int>("step_count", reader, template_reader, 50);
                back_count_limit = parameters::get_attribute_as<int>("back_count", reader, template_reader, 20);
                num_samples = parameters::get_attribute_as<unsigned>("num_samples", reader, template_reader, 50);

                if (parameters::has_attribute("goal", reader, template_reader))
                {
                    goal_state = parameters::get_attribute_as< vector_t > ("goal", reader, template_reader);
                    PRX_ASSERT( goal_state.get_dim() == 2 );
                    goal_set = true;
                }
                else
                {
                    goal_state.resize(2);
                }

                bool type = parameters::get_attribute_as<bool>("vanish", reader, template_reader);
                if( type == true )
                {
                    finish_type = PRX_VO_FT_VANISH;
                    vanishing = true;
                }
                else
                {
                    finish_type = PRX_VO_FT_STAY;
                    vanishing = false;
                }

                simple_controller_t::init(reader, template_reader);

                _X = X;
                _Y = Y;

                _Cx = goal_state[0];
                _Cy = goal_state[1];

                last_pos = get_center();
                curr_vel.zero();

                max_vel = this->output_control_space->get_bounds().at(V)->get_upper_bound();
                min_vel = this->output_control_space->get_bounds().at(V)->get_lower_bound();

                vo_radius = radius + ( max_vel * sim::simulation::simulation_step ) + 0.01;

                b_last_center = get_center();

                VO_sensing_info_t* test = NULL;
                for(unsigned i = 0; i < sensing_info.size() && test == NULL; i++)
                {
                    test = dynamic_cast<VO_sensing_info_t*>(sensing_info[i]);
                }
                PRX_ASSERT_MSG(test != NULL, "VO Controllers require a VO sensing info declared!");
                vo_sensing_info = test;
                vo_sensing_info->set_follow_controller( parent_controller );

                unsigned max_n = vo_sensing_info->get_max_num_neighbors();
                VOs.resize(max_n);
                max_hrvos = max_n;
                
                cc_time += stats_clock.measure();
                ++cc_calls;
            }


            void VO_controller_t::toggle_vanish()
            {
                if (vanished)
                {
                    set_param(subsystem_names.front(), "z", boost::any(3.0));
                    vanished = false;
                }
                else
                {
                    set_param(subsystem_names.front(), "z", boost::any(-10.0));
                    vanished = true;
                }
            }


            void VO_controller_t::propagate(const double simulation_step)
            {
                subsystems.begin()->second->propagate(simulation_step);

                goal_distance = get_center().distance(goal_state);
                double threshold = (vo_radius - radius);

                if( goal_distance < threshold )
                {
                    finished = true;
                    if( finish_type == PRX_VO_FT_VANISH )
                    {
                        vanished = true;
                    }
                }
                else
                {
                    finished = false;
                }

                //Then, we need to check for slowing down
                // near_goal = (goal_distance < 4.0*threshold) ? true : false;

                if (selected)
                {
                    update_vis_info(true);
                }
            }


            //ALRIGHTY THEN... this should be a lot simpler than it is..
            void VO_controller_t::compute_control()
            {
                stats_clock.reset();

                goal_state[_X] = _Cx;
                goal_state[_Y] = _Cy;

                computed_vos = false;

                //First, let's make sure the system understands its velocity
                update_velocity();

                //Update our knowledge about whether or not we have finished the problem
                finished = get_center().distance( goal_state ) < (vo_radius - radius);
                
                //DEBUG: Try to just stop moving?
                if( finished )
                {
                    computed_control->at(V) = 0;
                    output_control_space->copy_from_point( computed_control );
                    return;
                }

                //Our desired control is simply where we were told to go
                compute_desired_control();

                //Compute the velocity obstacles
                compute_VOs();

                //Then, if our desired control is valid
                if( velocity_is_valid(desired_control) )
                {
                    // PRX_PRINT(pathname << " valid.", PRX_TEXT_GREEN);
                    //Simply follow the desired control.
                    computed_vector_control = desired_control;
                }
                //We can't go the way we want, so we have to try alternatives
                else
                {
                    // PRX_PRINT(pathname << " invalid.", PRX_TEXT_RED);
                    //If we can't continue going the way we were already going
                    // if( !velocity_is_valid( computed_vector_control ) || computed_vector_control.squared_norm() < 0.5 )
                    // {
                        computed_vos = true;
                        //We have to sample some controls to find one that is valid
                        compute_best_sample_control(num_samples);
                    // }
                }

                //Then, update the actual computed control (Why aren't we just doing this directly?)
                double magn = computed_vector_control.norm();
                if( magn > 0.1 )
                {
                    computed_control->at(V) = PRX_MINIMUM( magn, max_vel );
                    computed_control->at(THETA) = atan2(computed_vector_control[Y], computed_vector_control[X]);
                }
                else
                {
                    computed_control->at(V) = 0;
                }
                //So many indirections...
                output_control_space->copy_from_point(computed_control);
                
                cc_time += stats_clock.measure();
                ++cc_calls;
            }


            int VO_controller_t::point_in_vos(vector_t* point, std::vector<bool>& vec, int index, double eps)
            {
                int counter = 0;

                for( size_t i = 0; i < nr_hrvos; ++i )
                {
                    //If the line we are considering is in this VO
                    if( (int)i == index / 6 )
                    {
                        //Report that the point is not in the VO
                        vec[i] = false;
                    }
                    else
                    {
                        if( VOs[i].in_obstacle(*point, eps) )
                        {
                            vec[i] = true;
                            ++counter;
                        }
                        else
                        {
                            vec[i] = false;
                        }
                    }
                }
                for( size_t k = nr_hrvos; k < vec.size(); ++k )
                {
                    vec[k] = false;
                }

                return counter;
            }


            void VO_controller_t::compute_VOs()
            {
                vector_t& other_center = point_a;
                const vector_t& my_center = get_center();

                neighbor_t* other_sys;

                unsigned int neighbor_index;
                neighborhood = vo_sensing_info->get_current_neighbors();

                // PRX_PRINT("Computing VOs for " << neighborhood.size() << " neighbors.", PRX_TEXT_MAGENTA );

                for( neighbor_index = 0; neighbor_index < neighborhood.size(); ++neighbor_index )
                {
                    // PRX_DEBUG_COLOR ("Constructing a VO for: " << neighborhood[neighbor_index]->get_name()<< " of type "<<neighborhood[neighbor_index]->get_geotype(), PRX_TEXT_BLUE);// << " with center: " << other_sys->get_center(), PRX_TEXT_MAGENTA);
                    other_sys = neighborhood[neighbor_index];
                    other_center[X] = other_sys->get_center()[X] - my_center[X];
                    other_center[Y] = other_sys->get_center()[Y] - my_center[Y];

                    double rec = 0.5;
                    if( neighborhood[neighbor_index]->get_reciprocity() )
                    {
                        const vector_t& ovel = other_sys->get_velocity();
                        // PRX_DEBUG_COLOR("RVO Construct :: oc(" << other_center[0] << "," << other_center[1] << ") :: v(" << curr_vel[0] << "," << curr_vel[1] << ") :: ov(" << ovel[0] << "," << ovel[1] << ")", PRX_TEXT_RED );
                        VOs[neighbor_index].rvo_construct(other_center, radius + other_sys->get_radius(), curr_vel, ovel, rec);
                        VOs[neighbor_index].set_source(1);
                    }
                    else
                    {
                        if (minkowski_obstacles->find(neighborhood[neighbor_index]->get_name()) != minkowski_obstacles->end())
                        {
                            switch( neighborhood[neighbor_index]->get_geotype() )
                            {
                                case PRX_BOX:
                                    VOs[neighbor_index].obstacle_construct( &((*(minkowski_obstacles))[neighborhood[neighbor_index]->get_name()]), my_center, safety_horizon, max_vel, minimally_invasive);
                                    VOs[neighbor_index].set_source(0);
                                    VOs[neighbor_index].set_velocity(max_vel);
                                    break;
                                case PRX_CYLINDER:
                                    VOs[neighbor_index].construct(other_center, radius + other_sys->get_radius(), other_sys->get_velocity());
                                    VOs[neighbor_index].set_source(1);
                                    VOs[neighbor_index].set_velocity(max_vel);
                                    break;
                                default:
                                    VOs[neighbor_index].construct(other_center, radius + other_sys->get_radius(), other_sys->get_velocity());
                                    VOs[neighbor_index].set_source(1);
                                    VOs[neighbor_index].set_velocity(max_vel);
                                    break;
                            }
                        }
                        else
                        {
                            PRX_FATAL_S("Minkowski obstacle " << neighborhood[neighbor_index]->get_name() << " not found.");
                        }
                    }
                }

                nr_hrvos = neighborhood.size();

                // for( unsigned i=0; i<nr_hrvos; ++i )
                // {
                //     PRX_PRINT("[" << i << "]: \n" << VOs[i].print(), PRX_TEXT_LIGHTGRAY );
                // }
            }


            void VO_controller_t::compute_desired_control()
            {
                //Obviously, if we are finished, we do not desired to move
                if( finished )
                {
                    desired_control[_X] = 0;
                    desired_control[_Y] = 0;
                    return;
                }

                //Our desired control is simply where we were told to go
                desired_control[_X] = _Cx - state_space->at(_X);
                desired_control[_Y] = _Cy - state_space->at(_Y);
                //Renormalize it
                desired_control.normalize();
                if( near_goal )
                {
                    desired_control *= (max_vel/8.0);
                }
                else
                {
                    desired_control *= max_vel;
                }
            }


            // void VO_controller_t::compute_best_sample_control( unsigned sample_size )
            // {
            //     // Stack variables
            //     bool control_found = false;
                
            //     // =========================================
            //     //   First, Binary search off to the right
            //     // =========================================

            //     // ==================================================
            //     //   Then, try random samples near our last control
            //     // ==================================================
            //     for( unsigned i=0; i<sample_size && !control_found; ++i )
            //     {
            //         //Sample something nearby
            //         output_control_space->uniform_sample_near( sample_point, computed_control, sampling_bounds );
                    
            //         //If the sample is valid, use it.
            //     }

            //     // =======================================
            //     //   If there's still nothing, slow down
            //     // =======================================
            //     computed_control->at(V) *= 0.6;
            // }

            void VO_controller_t::compute_best_sample_control(unsigned sample_size)
            {
                /** Visualization */
                sampled_x.clear();
                sampled_y.clear();
                sampled_validity.clear();

                vector_t current_control(2);
                vector_t best_control(2);

                double temp_x = 0; double temp_y = 0; double temp_vel = 0; double temp_magn = 0;

                double best_dist = PRX_INFINITY;
                double curr_dist = 0;

                double desired_x = desired_control[X];
                double desired_y = desired_control[Y];

                bool found_valid_control = false;

                double prev_control_x = computed_vector_control[X];
                double prev_control_y = computed_vector_control[Y];

                double prev_weight = 1;
                double goal_weight = 1;

                vector_t mixed_desired_control(2);

                if (csc_mode == MIXED_CONTROL)
                {
                    mixed_desired_control[X] = (prev_weight * prev_control_x + goal_weight * desired_x)/(prev_weight + goal_weight);
                    mixed_desired_control[Y] = (prev_weight * prev_control_y + goal_weight * desired_y)/(prev_weight + goal_weight);
                }

                bool initial_run = (computed_vector_control[X] == 0 && computed_vector_control[Y] == 0);

                double curr_control_weight = 2;
                double pc1_weight = 0.5;
                double pc2_weight = 0.4;
                double pc3_weight = 0.3;
                prev_vel = (*output_control_space)[V];

                double delta = 0.9;

                bool use_delta = true;

                //Random sampling
                for (unsigned i=0; i < sample_size; i++)
                {
                    temp_x = rand() % 200 - 100;
                    temp_y = rand() % 200 - 100;

                    temp_magn = sqrt(pow(temp_x, 2.0) + pow(temp_y, 2.0));

                    switch(csc_mode)
                    {
                        case BASIC:
                        temp_vel = max_vel;
                        break;
                        case MIXED_CONTROL:
                        temp_vel = (prev_vel + (rand() % ((int)(max_vel + 0.5) - (int)min_vel) + min_vel) )/2;
                        break;
                        case BLENDED_HISTORY:
                        temp_vel = rand() % ((int)(max_vel + 0.5) - (int)min_vel) + min_vel;
                        break;
                    }

                    temp_x = (temp_x / temp_magn) * temp_vel;
                    temp_y = (temp_y / temp_magn) * temp_vel;

                    current_control[X] = temp_x;
                    current_control[Y] = temp_y;

                    bool valid = velocity_is_valid(current_control);

                    sampled_x.push_back(temp_x);
                    sampled_y.push_back(temp_y);
                    sampled_validity.push_back(valid);

                    if(!valid)
                    {
                        continue;
                    }

                    found_valid_control = true;

                    switch (csc_mode)
                    {
                        case BASIC:
                            curr_dist = sqrt(pow(temp_x - desired_x, 2.0) + pow(temp_y - desired_y, 2.0));
                            break;
                        case MIXED_CONTROL:
                            if (initial_run)
                            {
                                curr_dist = sqrt(pow(temp_x - desired_x, 2.0) + pow(temp_y - desired_y, 2.0));
                            }
                            else
                            {
                                curr_dist = sqrt(pow(temp_x - mixed_desired_control[X], 2.0) + pow(temp_y - mixed_desired_control[Y], 2.0));
                            }
                            break;
                        case BLENDED_HISTORY:
                            curr_dist = sqrt(pow(temp_x - desired_x, 2.0) + pow(temp_y - desired_y, 2.0));
                            break;
                        default:
                            curr_dist = sqrt(pow(temp_x - desired_x, 2.0) + pow(temp_y - desired_y, 2.0));
                            break;
                    }

                    if (curr_dist < best_dist)
                    {
                        best_control = current_control;
                        best_dist = curr_dist;
                    }

                }

                if (!found_valid_control)
                {
                    computed_vector_control[X] = 0;
                    computed_vector_control[Y] = 0;
                }
                else
                {
                     switch (csc_mode)
                     {
                        case BASIC:
                            computed_vector_control = best_control;
                            break;
                        case MIXED_CONTROL:
                            computed_vector_control = best_control;
                            break;
                        case BLENDED_HISTORY:
                            if (initial_run)
                            {
                                computed_vector_control = best_control;
                                prev_control_1 = computed_vector_control;
                                break;
                            }

                            if (use_delta)
                            {
                                computed_vector_control[X] = (delta * best_control[X] + pow(delta, 2.0) * prev_control_1[X] + pow(delta, 3.0) * prev_control_2[X] + pow(delta, 4.0) * prev_control_3[X])/(delta + pow(delta, 2.0) + pow(delta, 3.0) + pow(delta, 4.0));

                                computed_vector_control[Y] = (delta * best_control[Y] + pow(delta, 2.0) * prev_control_1[Y] + pow(delta, 3.0) * prev_control_2[Y] + pow(delta, 4.0) * prev_control_3[Y])/(delta + pow(delta, 2.0) + pow(delta, 3.0) + pow(delta, 4.0));
                            }
                            else
                            {
                                computed_vector_control[X] = (curr_control_weight * best_control[X] + pc1_weight * prev_control_1[X] + pc2_weight * prev_control_2[X] + pc3_weight * prev_control_3[X])/(curr_control_weight + pc1_weight + pc2_weight + pc3_weight);

                                computed_vector_control[Y] = (curr_control_weight * best_control[Y] + pc1_weight * prev_control_1[Y] + pc2_weight * prev_control_2[Y] + pc3_weight * prev_control_3[Y])/(curr_control_weight + pc1_weight + pc2_weight + pc3_weight);
                            }

                            prev_control_1 = computed_vector_control;
                            prev_control_2 = prev_control_1;
                            prev_control_3 = prev_control_2;
                            break;
                        default:
                            computed_vector_control = best_control;
                            break;
                    }
                }
            }


            bool VO_controller_t::velocity_is_valid(const vector_t& vec, double eps)
            {
                //Check if our desired control is outside of the HRVOs
                for( unsigned int i = 0; i < nr_hrvos; ++i )
                {
                    if( VOs[i].in_obstacle(vec, eps) )
                    {
                        return false;
                    }
                }
                return true;
            }


            void VO_controller_t::update_velocity()
            {
                const vector_t& cc = get_center();
                curr_vel = (cc - last_pos) / (sim::simulation::simulation_step);
                last_pos = cc;
            }


            const vector_t& VO_controller_t::get_center() const
            {
                vector_t& ret = point_d;

                ret[X] = (*state_space)[_X];
                ret[Y] = (*state_space)[_Y];

                return ret;
            }


            const vector_t& VO_controller_t::get_velocity() const
            {
                return curr_vel;
            }


            double VO_controller_t::get_radius()
            {
                return vo_radius;
            }


            void VO_controller_t::set_obstacles(hash_t<std::string, minkowski_body_t>& inobst)
            {
                minkowski_obstacles = &inobst;
                                
                // typedef std::pair<double, vector_t> vpair;
                // std::set< vpair > holdset;

                // // Iterate over each obstacle
                // foreach(system_ptr_t sys, inobst | boost::adaptors::map_values)
                // {
                //     geom_map_t obstacles;
                //     config_list_t configs;

                //     unsigned int zero = 0;
                //     // Get each part of the obstacle and it's configuration
                //     sys->update_phys_geoms(obstacles);
                //     sys->update_phys_configs(configs,zero);

                //     // Calculate minkowski sum (approximation)
                //     foreach(std::string name, obstacles | boost::adaptors::map_keys)
                //     {
                //         holdset.clear();

                //         geometry_t body = obstacles[name];
                //         config_t conf;
                //         foreach(config_list_element_t element, configs)
                //         {
                //             if(element.first == name)
                //                 conf = element.second;
                //         }

                //         double angle = conf.get_orientation().get_yaw();
                //         double x_offset = conf.get_position().at(0);
                //         double y_offset = conf.get_position().at(1);

                //         geometry_t* minkowski_sum = body.get_minkowski(false, radius, radius*0.1);

                //         if( minkowski_sum != NULL )
                //         {
                //             minkowski_body_t* new_body = new minkowski_body_t(minkowski_sum, x_offset, y_offset);

                //             foreach(vector_t vec, new_body->body->get_trimesh()->get_vertices())
                //             {
                //                 vector_t v((vec[0] * cos(angle) - vec[1] * sin(angle)) + x_offset, (vec[0] * sin(angle) + vec[1] * cos(angle)) + y_offset);
                //                 holdset.insert( std::pair<double, vector_t>( atan2(v[0]-x_offset,v[1]-y_offset), v ) );
                //             }

                //             //Now put them into the minkowski body
                //             foreach( vpair v , holdset )
                //             {
                //                 new_body->vertices.push_back( v.second );
                //             }
                //             minkowski_obstacles[name] = new_body;
                //             // PRX_DEBUG_COLOR ("Added new minkowski body with name: " << name, PRX_TEXT_RED);
                //         }
                //     }
                // }
            }


            void VO_controller_t::update_vis_info(bool visualize_VO) const
            {
                std::vector<geometry_info_t> geoms;
                std::vector<geometry_info_t> sim_geoms;
                std::vector<config_t> configs;
                std::vector<config_t> sim_configs;
                std::vector<double> params;

                //Populate the system configuration
                config_t sys_config;

                //Visualize Minkowski Body things.
                // unsigned mind = 0;
                // params.push_back( 0.06 );
                // params.push_back( 0.06 );
                // params.push_back( 0.06 );
                // foreach(minkowski_body_t& minky, *(minkowski_obstacles) | boost::adaptors::map_values)
                // {
                //     for( unsigned i=0; i<minky.vertices.size(); ++i )
                //     {
                //         std::string name = ros::this_node::getName() + pathname + "Minkoski_" + int_to_str( mind ) + "_" + int_to_str( i );
                        
                //         const vector_t& vertex = minky.vertices[i];
                        
                //         sys_config.set_position( vertex[0], vertex[1], 1 );
                        
                //         geoms.push_back( geometry_info_t(name, "vo", PRX_BOX, params, "orange") );
                //         configs.push_back( sys_config );
                //     }
                //     ++mind;
                // }

                if(visualize_VO)
                {
                    was_vis = true;
                    double agent_z = (*(subsystems.begin()->second->get_state_space()))[2];

                    std::string full_name = pathname + "--VO";

                    params.clear();
                    sys_config.set_position( computed_control->memory[0], computed_control->memory[1], computed_control->memory[2] );
                    PRX_DEBUG_COLOR("Computed control point: " << computed_control->memory[0] << "," << computed_control->memory[1] << "," << computed_control->memory[2], PRX_TEXT_BLUE);
                    params.push_back(5);
                    geoms.push_back(geometry_info_t("computed_velocity", "vo", PRX_SPHERE, params, "red"));
                    configs.push_back( sys_config );
                    params.clear();

                    //Then, actually show them
                    for( unsigned int i = 0; i < nr_hrvos; ++i )
                    {
                        std::string name = ros::this_node::getName() + pathname + "/VO_" + int_to_str( i + 1 );
                        params.clear();
                        sys_config.set_position( (*state_space)[_X], (*state_space)[_Y], 0.5 );

                        //LEFT, RIGHT, VERTEX
                        VOs[i].get_points( point_a, point_b, point_c );
                        params.push_back(point_a[0]);
                        params.push_back(point_a[1]);
                        params.push_back(agent_z);
                        params.push_back(point_c[0]);
                        params.push_back(point_c[1]);
                        params.push_back(agent_z);
                        params.push_back(point_b[0]);
                        params.push_back(point_b[1]);
                        params.push_back(agent_z);

                        geoms.push_back(geometry_info_t(name, "vo", PRX_LINESTRIP, params, debug_colors[i%debug_colors.size()]));
                        configs.push_back( sys_config );
                        params.clear();
                    }

                    //Put all the others below
                    for( unsigned i=nr_hrvos; i<max_hrvos; ++i )
                    {
                        std::string name = ros::this_node::getName() + pathname + "/VO_" + int_to_str( i + 1 );

                        params.push_back( 0 );
                        params.push_back( 1 );
                        params.push_back( -1 );
                        params.push_back( 0 );
                        params.push_back( 0 );
                        params.push_back( -1 );
                        params.push_back( 1 );
                        params.push_back( 0 );
                        params.push_back( -1 );
                        geoms.push_back(geometry_info_t(name, "vo", PRX_LINESTRIP, params, "white") );
                        configs.push_back( sys_config );
                        params.clear();
                    }

                    /**
                    for( unsigned int i = 0; i < sampled_validity.size(); ++i )
                    {
                        std::string name = ros::this_node::getName() + pathname;
                        params.clear();

                        if (sampled_validity[i])
                        {
                            params.push_back(sampled_x[i]/2);
                            params.push_back(sampled_y[i]/2);
                            params.push_back(1.0);
                            params.push_back(0);
                            params.push_back(0);
                            params.push_back(1.0);
                            geoms.push_back(geometry_info_t(name, "/VO_sample" + int_to_str( i + 1 ), PRX_LINESTRIP, params, "green"));
                        }
                        else
                        {
                            params.push_back(sampled_x[i]/2);
                            params.push_back(sampled_y[i]/2);
                            params.push_back(1.0);
                            params.push_back(0);
                            params.push_back(0);
                            params.push_back(1.0);
                            geoms.push_back(geometry_info_t(name, "/VO_sample" + int_to_str( i + 1 ), PRX_LINESTRIP, params, "red"));
                        }
                        configs.push_back( sys_config );
                        params.clear();
                    }

                    for( unsigned i=sampled_validity.size(); i<num_samples; ++i )
                    {
                        std::string name = ros::this_node::getName() + pathname;
                        params.clear();

                        params.push_back(0);
                        params.push_back(1);
                        params.push_back(-1);
                        params.push_back(0);
                        params.push_back(1);
                        params.push_back(-1);
                        geoms.push_back(geometry_info_t(name, "/VO_sample" + int_to_str( i + 1 ), PRX_LINESTRIP, params, "green"));
                        configs.push_back( sys_config );
                        params.clear();
                    }
                    */

                    std::string name = ros::this_node::getName() + pathname + "/VO_control" ;
                    params.clear();
                    // PRX_WARN_S ("My centerx: " << (*state_space)[_X] << "  y: " <<(*state_space)[_Y]);
                    // PRX_DEBUG_COLOR("Computed vector control: " << computed_vector_control, PRX_TEXT_MAGENTA);

                    params.push_back(computed_vector_control[0]);
                    params.push_back(computed_vector_control[1]);
                    params.push_back(agent_z + 1);
                    params.push_back(0);
                    params.push_back(0);
                    params.push_back(agent_z + 1);

                    geoms.push_back(geometry_info_t(name, "vo", PRX_LINESTRIP, params, "pink"));
                    configs.push_back( sys_config );

                    ((visualization_comm_t*)vis_comm)->visualization_geom_map[full_name] = geoms;

                    ((visualization_comm_t*)vis_comm)->visualization_configs_map[full_name] = configs;
                }
                else if( was_vis )
                {
                    std::string full_name = pathname + "--VO";

                    was_vis = false;

                    params.clear();
                    sys_config.set_position( computed_control->memory[0], computed_control->memory[1], computed_control->memory[2] );
                    PRX_DEBUG_COLOR("Computed control point: " << computed_control->memory[0] << "," << computed_control->memory[1] << "," << computed_control->memory[2], PRX_TEXT_BLUE);
                    params.push_back(5);
                    geoms.push_back(geometry_info_t("computed_velocity", "vo", PRX_SPHERE, params, "red"));
                    configs.push_back( sys_config );
                    params.clear();

                    for( unsigned i=0; i<max_hrvos; ++i )
                    {
                        std::string name = ros::this_node::getName() + pathname + "/VO_" + int_to_str( i + 1 );

                        params.push_back( 0 );
                        params.push_back( 1 );
                        params.push_back( -1 );
                        params.push_back( 0 );
                        params.push_back( 0 );
                        params.push_back( -1 );
                        params.push_back( 1 );
                        params.push_back( 0 );
                        params.push_back( -1 );
                        geoms.push_back(geometry_info_t(name, "vo", PRX_LINESTRIP, params, "teal") );
                        configs.push_back( sys_config );
                        params.clear();
                    }

                    for( unsigned i=0; i<num_samples; ++i )
                    {
                        std::string name = ros::this_node::getName() + pathname;
                        params.clear();

                        params.push_back(0);
                        params.push_back(1);
                        params.push_back(-1);
                        params.push_back(0);
                        params.push_back(1);
                        params.push_back(-1);
                        geoms.push_back(geometry_info_t(name, "/VO_sample" + int_to_str( i + 1 ), PRX_LINESTRIP, params, "green"));
                        configs.push_back( sys_config );
                        params.clear();
                    }
                    std::string name = ros::this_node::getName() + pathname + "/VO_control" ;
                    params.clear();

                    params.push_back(0);
                    params.push_back(0);
                    params.push_back(-10.0);
                    params.push_back(0);
                    params.push_back(0);
                    params.push_back(-10.0);

                    geoms.push_back(geometry_info_t(name, "vo", PRX_LINESTRIP, params, "pink"));
                    configs.push_back( sys_config );

                    ((visualization_comm_t*)vis_comm)->visualization_geom_map[full_name] = geoms;

                    ((visualization_comm_t*)vis_comm)->visualization_configs_map[full_name] = configs;
                }

                ((visualization_comm_t*)vis_comm)->send_geometries();
            }

            void VO_controller_t::update_max_velocity(double velocity)
            {
                max_vel = velocity;
            }
        }
    }
}



