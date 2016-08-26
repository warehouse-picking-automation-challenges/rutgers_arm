/**
 * @file collision_avoidance_controller.cpp
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

#include "simulation/controllers/collision_avoidance_controller.hpp"

#include "prx/simulation/system_graph.hpp"
#include "prx/simulation/systems/plants/plant.hpp"
#include "prx/simulation/communication/visualization_comm.hpp"

#include <math.h>
#include <pluginlib/class_list_macros.h>
#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/range/adaptor/map.hpp> //adaptors
#include <boost/assign/list_of.hpp>

PLUGINLIB_EXPORT_CLASS( prx::packages::crowd::collision_avoidance_controller_t, prx::sim::system_t)

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
            double collision_avoidance_controller_t::cc_time = 0;
            unsigned collision_avoidance_controller_t::cc_calls = 0;

            const unsigned int collision_avoidance_controller_t::X = 0;
            const unsigned int collision_avoidance_controller_t::Y = 1;

            const unsigned int collision_avoidance_controller_t::V = 0;
            const unsigned int collision_avoidance_controller_t::THETA = 1;

            collision_avoidance_controller_t::collision_avoidance_controller_t()
            {
                nr_hrvos = 0;

                temp_vector.resize(2);
                best_vector_control.resize(2);
                desired_control.resize(2);

                _Cx = _Cy = _Cz = 0;
                control_memory = { &_Cx , &_Cy , &_Cz };
                input_control_space = new space_t( "XYZ", control_memory );
                
                safety = 30;
                desired_velocity = 0;
                conservative_turns = true;
                in_queue = false;
                stay_safe = true;
                agent_id = 1e4;
                queue_orientation = 0;
            }


            collision_avoidance_controller_t::~collision_avoidance_controller_t()
            {
                output_control_space->free_point( current_control );
                output_control_space->free_point( best_control );
            }

            void collision_avoidance_controller_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
            {
                minimally_invasive = parameters::get_attribute_as<bool>("minimally_invasive", reader, template_reader, true);

                simple_controller_t::init(reader, template_reader);

                max_vel = this->output_control_space->get_bounds().at(V)->get_upper_bound();
                sq_max_vel = max_vel * max_vel;
                min_vel = this->output_control_space->get_bounds().at(V)->get_lower_bound();

                neighbor_sensing_info_t* test = NULL;
                for(unsigned i = 0; i < sensing_info.size() && test == NULL; i++)
                {
                    test = dynamic_cast<neighbor_sensing_info_t*>(sensing_info[i]);
                }

                PRX_ASSERT_MSG(test != NULL, "VO Controllers require a VO sensing info declared!");
                neighbor_info = test;

                unsigned max_n = parameters::get_attribute_as< unsigned >("max_num_neighbors", reader, template_reader, 12);
                VOs.resize(max_n);
                max_hrvos = max_n;
                
                current_control = output_control_space->alloc_point();
                best_control = output_control_space->alloc_point();
            }


            void collision_avoidance_controller_t::propagate(const double simulation_step)
            {
                subsystems.begin()->second->propagate(simulation_step);
            }

            void collision_avoidance_controller_t::set_id( unsigned input_id )
            {
                neighbor_info->agent_index = input_id;
            }

            void collision_avoidance_controller_t::set_passive( bool input_flag )
            {
                passive = input_flag;
            }

            void collision_avoidance_controller_t::set_conservative_turns(bool flag)
            {
                conservative_turns = flag;   
            }

            void collision_avoidance_controller_t::set_in_queue(bool flag, unsigned agent_id)
            {
                in_queue = flag;
                this->agent_id = agent_id;
            }

            void collision_avoidance_controller_t::compute_control()
            {
                bool desired_safe = true;
                stats_clock.reset();

                //If we are passive, i.e., in an elevator, simply DO NOT MOVE.
                if( passive )
                {
                    computed_control->memory[V] = 0;
                    output_control_space->copy_from_point( computed_control );
                    return;
                }

                //Our desired control is simply where we were told to go
                compute_desired_control();
                
                //Compute the velocity obstacles
                compute_VOs();
                
                //Convert our desired control into what we have computed
                to_polar( computed_control, desired_control );

                //Only if don't have a lot of agents around us that are blocking us we will compute
                //safe velocity. If there are more than two agents around us then we will follow our 
                //desire velocity. 
                if(stay_safe)
                {
                    //Then, if our desired control is invalid
                    if( !velocity_is_valid(desired_control) )
                    {
                        desired_safe = false;
                        //We have to sample some controls to find one that is valid
                        compute_best_control( );
                    }

                    //Now that we think we have a good control, check for safety.
                    //check_control_for_safety( desired_safe );
                }   

                if(computed_control->memory[V] == 0)
                    computed_control->memory[THETA] =  current_control->memory[THETA];
                
                
                //Make sure to inform the control space what control we are using
                output_control_space->copy_from_point( computed_control );

                cc_time += stats_clock.measure();
                ++cc_calls;
                // if(cc_calls%1000 == 0 && agent_id != 1e4)
                // {
                //     PRX_PRINT("agent:"<<agent_id<<" desired_velocity:"<<desired_velocity<<" desired_safe:"<<desired_safe<<" stay_safe:"<<stay_safe<<" safety:"<<safety,PRX_TEXT_LIGHTGRAY);
                //     PRX_PRINT("agent:"<<agent_id<<" conservative_turns:"<<conservative_turns<<" passive:"<<passive<<" in_queue:"<<in_queue,PRX_TEXT_LIGHTGRAY);
                // }
                subsystems.begin()->second->compute_control();
            }


            void collision_avoidance_controller_t::compute_desired_control()
            {
                //Our desired control is simply where we were told to go
                desired_control[0] = _Cx - state_space->at(0);
                desired_control[1] = _Cy - state_space->at(1);
                double dist = desired_control.norm();
                //Renormalize it
                desired_control.normalize();
                desired_velocity = dist / sim::simulation::simulation_step;
                desired_velocity = PRX_MINIMUM( max_vel, desired_velocity );
                // if(dist<0.1)
                // {
                //     PRX_PRINT("velocity:"<<desired_velocity,PRX_TEXT_CYAN);
                // }
                desired_control *= desired_velocity;
            }

            bool collision_avoidance_controller_t::velocity_is_valid( const util::vector_t& vec, double eps, int excluded )
            {
                for( int i = 0; i < nr_hrvos; ++i )
                {
                    if( i != excluded )
                    {
                        if( VOs[i].in_obstacle(vec, eps) )
                        {
                            return false;
                        }
                    }
                }
                return true;                
            }

            bool collision_avoidance_controller_t::velocity_is_valid_obstacle_check( const util::vector_t& vec, double eps, int excluded )
            {
                //Check if our desired control is outside of the HRVOs
                for( int i = 0; i < nr_hrvos; ++i )
                {
                    if( i != excluded )
                    {
                        //We only check the obstacle VOs first here
                        if( VOs[i].get_source_type() == 0 )
                        {
                            if( VOs[i].in_obstacle(vec, eps) )
                            {
                                return false;
                            }
                        }
                    }
                }

                return true;                
            }
            
            void collision_avoidance_controller_t::compute_VOs()
            {
                stay_safe = true;
                int safe_counter = 0;
                vector_t& other_center = temp_vector;
                unsigned neighbor_index;

                neighbor_t* other_sys;

                // PRX_PRINT("Compute VOs", PRX_TEXT_BLUE);
                // PRX_PRINT("neighbor_info: " << neighbor_info, PRX_TEXT_BLUE);
                // // PRX_PRINT("neighborhood: " << neighbor_info->neighborhood, PRX_TEXT_BLUE);
                // PRX_PRINT("neighborhood size: " << neighbor_info->neighborhood.size(), PRX_TEXT_LIGHTGRAY);

                const std::vector< neighbor_t* >& neighborhood = neighbor_info->neighborhood;              
                const vector_t& my_center = neighborhood.back()->get_center();

                // PRX_PRINT("========================================================", PRX_TEXT_GREEN);
                // PRX_PRINT("  Computing VOs for agent: " << neighborhood.back()->get_name(), PRX_TEXT_CYAN);
                // PRX_PRINT("========================================================", PRX_TEXT_GREEN);

                for( neighbor_index = 0; neighbor_index<neighborhood.size() -1; ++neighbor_index )
                {
                    other_sys = neighborhood[neighbor_index];
                    other_center[X] = other_sys->get_center()[X] - my_center[X];
                    other_center[Y] = other_sys->get_center()[Y] - my_center[Y];

                    if( neighborhood[neighbor_index]->get_reciprocity() )
                    {
                        const vector_t& ovel = other_sys->get_velocity();
                        double diff = fabs ( ovel[V] - current_control->memory[V]);
                        double res = (diff * 0.5) /max_vel;
                        if( current_control->memory[V] > ovel[V] )
                            VOs[neighbor_index].rvo_construct(other_center, neighborhood.back()->get_radius() + other_sys->get_radius(), neighborhood.back()->get_velocity(), ovel, 0.5 + res);
                        else
                            VOs[neighbor_index].rvo_construct(other_center, neighborhood.back()->get_radius() + other_sys->get_radius(), neighborhood.back()->get_velocity(), ovel, 0.5 - res);
                        VOs[neighbor_index].set_source(1);

                        if(my_center.distance(other_sys->get_center()) < 1)
                        {
                            ++safe_counter;
                        }
                    }
                    else
                    {
                        if (minkowski_obstacles->find(neighborhood[neighbor_index]->get_name()) != minkowski_obstacles->end())
                        {
                            VOs[neighbor_index].obstacle_construct( &((*(minkowski_obstacles))[neighborhood[neighbor_index]->get_name()]), my_center, PRX_INFINITY, max_vel, minimally_invasive);
                            VOs[neighbor_index].set_source(0);
                        }
                        else
                        {
                            PRX_FATAL_S("Minkowski obstacle " << neighborhood[neighbor_index]->get_name() << " not found.");
                        }
                    }
                }

                if(safe_counter > 2)
                {
                    stay_safe = false;

                    // PRX_PRINT("================================================================",PRX_TEXT_BLUE);
                    // PRX_PRINT("\n",PRX_TEXT_BLUE);  
                    // PRX_PRINT("\n",PRX_TEXT_BLUE);  
                    // PRX_PRINT("Agent " << neighbor_info->agent_index << " Will go anywhere He does not care about anyone!!!!",PRX_TEXT_RED);  
                    // PRX_PRINT("\n",PRX_TEXT_BLUE);  
                    // PRX_PRINT("\n",PRX_TEXT_BLUE);
                    // PRX_PRINT("================================================================",PRX_TEXT_BLUE);
                }
                //  stay_safe = true;

                nr_hrvos = neighborhood.size() -1;
            }

            void collision_avoidance_controller_t::compute_best_control()
            {
                // PRX_PRINT( "=== Computing a Best Control ===", PRX_TEXT_MAGENTA);
                // PRX_PRINT("Max Vel: " << max_vel << "(" << sq_max_vel << ")", PRX_TEXT_GREEN);
                
                //holding our results
                std::vector< std::pair< vector_t, unsigned > > intersection_points;
                std::vector< unsigned > valid_indices;
                
                //Begin by assuming our best control is to stay put
                output_control_space->zero( best_control );
                
                //First, get all the VOs to make the appropriate segments
                for( unsigned i=0; i<nr_hrvos; ++i )
                {
                    VOs[i].construct_segments( max_vel );
                }
                
                //Go over each VO, and get the points from the valid segments
                for( unsigned i=0; i<nr_hrvos; ++i )
                {
                    //For each possible segment
                    for( unsigned k=0; k<2; ++k )
                    {
                        //Get the segment
                        const line_t& seg = ((k == 0) ? VOs[i].get_left_seg() : VOs[i].get_right_seg());
                        //If it is valid
                        if( seg.is_valid() )
                        {
                            //get the point A
                            seg.get_point_a( scratch_vector );
                            //If it is near the radius we expect
                            // PRX_PRINT("[" << i << "] Point (A) found: " << scratch_vector[0] << ", " << scratch_vector[1] << "  : " << scratch_vector.norm() << "(" << scratch_vector.squared_norm() << ")", PRX_TEXT_CYAN + k );
                            if( fabs( scratch_vector.squared_norm() - sq_max_vel ) < 0.01 )
                            {
                                //This is a possible point of interest
                                intersection_points.push_back( std::pair< vector_t, unsigned >( scratch_vector, i ) );
                                // PRX_PRINT("^ is go...", PRX_TEXT_BLUE);
                            }
                            //get the point B
                            seg.get_point_b( scratch_vector );
                            //If it is near the radius we expect
                            // PRX_PRINT("[" << i << "] Point (B) found: " << scratch_vector[0] << ", " << scratch_vector[1] << "  : " << scratch_vector.norm() << "(" << scratch_vector.squared_norm() << ")", PRX_TEXT_CYAN + k );
                            if( fabs( scratch_vector.squared_norm() - sq_max_vel ) < 0.01 )
                            {
                                //This is a possible point of interest
                                intersection_points.push_back( std::pair< vector_t, unsigned >( scratch_vector, i ) );
                                // PRX_PRINT("^ is go...", PRX_TEXT_BLUE);
                            }
                        }
                    }
                }
                // PRX_PRINT( "(" << pathname << ") From " << nr_hrvos << " vos, got " << intersection_points.size() << " points.", PRX_TEXT_BROWN);
                
                //For each intersection point
                for( unsigned i=0; i<intersection_points.size(); ++i )
                {
                    // PRX_PRINT("[" << i << "] : " << intersection_points[i].first[0] << " , " << intersection_points[i].first[1] << " (" << intersection_points[i].second << ")", PRX_TEXT_LIGHTGRAY);
                    //If it is valid
                    if( velocity_is_valid( intersection_points[i].first, -0.00001, intersection_points[i].second ) )
                    {
                        //Mark it as such
                        valid_indices.push_back(i);
                    }
                }
                // PRX_PRINT("Of those, only " << valid_indices.size() << " are valid.", PRX_TEXT_RED);
                // PRX_PRINT("Trying to go in the direction: " << computed_control->memory[THETA], PRX_TEXT_RED);
                
                //For each valid intersection point
                double min_diff = 4;
                int best_index = -1;
                for( unsigned i=0; i<valid_indices.size(); ++i )
                {
                    //Get the control this point yields
                    to_polar( best_control, intersection_points[valid_indices[i]].first );
                    //Get the angular difference
                    double diff = minimum_angle_with_orientation( best_control->memory[THETA], computed_control->memory[THETA] );
                    //TEST: let's try slighly biasing them to one side.
                    if( diff > 0 )
                    {
                        diff *= 0.9;
                    }
                    diff = fabs( diff );
                    
                    // PRX_PRINT(":: [" << valid_indices[i] << "]  THETA: " << best_control->memory[THETA] << "   diff " << diff, PRX_TEXT_LIGHTGRAY );
                    //If the difference to our desired is the smallest we have seen AND it is within acceptable bounds from our current
                    if( diff < min_diff )
                    {
                        min_diff = diff;
                        best_index = i;
                    }
                }
                
                //If we found something
                if( best_index > -1 )
                {
                    //our computed control is that point
                    to_polar( best_control, intersection_points[valid_indices[best_index]].first );
                }
                
                //make sure to copy over our final result
                output_control_space->copy_point( computed_control, best_control );
            }

            void collision_avoidance_controller_t::check_control_for_safety( bool desired_safe )
            {
                //Get our current control
                output_control_space->copy_to_point( current_control );
                
                //If our control is quite different than what we were travelling at previously
                double difference = minimum_angle_with_orientation( computed_control->memory[THETA], current_control->memory[THETA] );

                //Also, if we're wanting to do a 90-degree or so turn, we should probably play it a little safe
                if( fabs( difference ) > 0.10 && in_queue )
                {
                    difference = fabs( difference );
                    // double unsafety = 2 * difference;
                    double unsafety = (difference + 3) * (difference + 3) - 9;
                    safety = PRX_MAXIMUM( safety - unsafety, 0 );
                }
                //Otherwise, we feel more safe
                else
                {
                    safety = PRX_MINIMUM( safety + 0.4, 30 );
                }
                
                //Update our desired velocity based on our percieved safety
                if( desired_safe )
                {
                    desired_velocity = desired_velocity*( ((double)safety)/30.0 );
                }
                else
                {
                    desired_velocity = (max_vel/2.0)*( ((double)safety)/30.0 );

                }
                
                //Update what we think our current control will be given this information
                if(conservative_turns)
                    current_control->memory[THETA] += norm_angle_pi( PRX_MAXIMUM( PRX_MINIMUM( difference, 0.04 ), -0.04 ) );
                else
                {
                    current_control->memory[THETA] += norm_angle_pi(  difference);
                    //PRX_PRINT("difference:" << difference, PRX_TEXT_RED);
                    //desired_velocity = 0.1;
                }

                if(in_queue)
                {
                    current_control->memory[THETA] = queue_orientation;    
                }

                current_control->memory[V] = desired_velocity;
                
                //Then, we need to see if the direction we're going crashes us into a wall
                to_euclidean( best_vector_control, current_control );
                if( !velocity_is_valid_obstacle_check( best_vector_control, -0.00001 ) )
                {
                    //If it does, we need to not move forward
                    desired_velocity = 0;
                }
                                
                //Then, adjust our velocity based on our currently percieved safety.
                computed_control->memory[V] = desired_velocity;
                //computed_control->memory[THETA] = current_control->memory[THETA];
            }

            void collision_avoidance_controller_t::set_obstacles(hash_t<std::string, minkowski_body_t>& inobst)
            {
                minkowski_obstacles = &inobst;
            }

            void collision_avoidance_controller_t::update_max_velocity( double velocity )
            {
                max_vel = velocity;
                sq_max_vel = max_vel * max_vel;
                //TODO: Don't we also need to mess with control bounds?
            }

            double collision_avoidance_controller_t::get_current_velocity() const
            {
                return computed_control->memory[V];
            }

            // double collision_avoidance_controller_t::get_current_orientation() const
            // {
            //     return (fabs(computed_control->memory[THETA]) < 5e-3)? 0:computed_control->memory[THETA];
            // }

            void collision_avoidance_controller_t::to_polar( space_point_t* polar, const vector_t& vector )
            {
                polar->memory[V] = vector.norm();
                polar->memory[THETA] = atan2( vector[Y], vector[X] );
            }
            
            void collision_avoidance_controller_t::to_euclidean( vector_t& vector, space_point_t* polar )
            {
                vector[X] = polar->memory[V] * cos( polar->memory[THETA] );
                vector[Y] = polar->memory[V] * sin( polar->memory[THETA] );
            }

            void collision_avoidance_controller_t::visualize_VOs() const
            {
                std::vector<geometry_info_t> geoms;
                std::vector<config_t> configs;
                std::vector<double> params;

                vector_t point_a(2), point_b(2), point_c(2);

                //Populate the system configuration
                config_t sys_config;

                double agent_z = (*(subsystems.begin()->second->get_state_space()))[2];

                std::string full_name = "velocity_obstacles";

                params.clear();
                sys_config.set_position( (*input_control_space)[0], (*input_control_space)[1], agent_z );
                // PRX_DEBUG_COLOR("Computed control point: " << computed_control->memory[0] << "," << computed_control->memory[1] << "," << computed_control->memory[2], PRX_TEXT_BLUE);
                params.push_back(0.3);
                geoms.push_back(geometry_info_t("computed_velocity", "vo", PRX_SPHERE, params, "dark_grey"));
                configs.push_back( sys_config );
                params.clear();

                std::vector<double> eucl(2);
                eucl[X] = computed_control->memory[V] * cos( computed_control->memory[THETA] );
                eucl[Y] = computed_control->memory[V] * sin( computed_control->memory[THETA] );

                sys_config.set_position( _Cx + eucl[0], _Cy + eucl[1], agent_z);
                params.push_back(0.4);
                geoms.push_back(geometry_info_t("computed_control", "vo", PRX_SPHERE, params, "red"));
                configs.push_back( sys_config );
                params.clear();


                //Then, actually show them
                for( unsigned int i = 0; i < nr_hrvos; ++i )
                {
                    std::string name = "VO_" + int_to_str( i );
                    params.clear();
                    sys_config.set_position( (*state_space)[X], (*state_space)[Y], 0.5 );

                    //LEFT, RIGHT, VERTEX
                    VOs[i].get_points( point_a, point_b, point_c );
                    params.push_back(point_a[0]); params.push_back(point_a[1]); params.push_back(agent_z);
                    params.push_back(point_c[0]); params.push_back(point_c[1]); params.push_back(agent_z);
                    params.push_back(point_b[0]); params.push_back(point_b[1]); params.push_back(agent_z);

                    geoms.push_back(geometry_info_t(name, "vo", PRX_LINESTRIP, params, (VOs[i].get_source_type() == 0 ? "blue" : "orange") ) );
                    configs.push_back( sys_config );
                    params.clear();
                }

                //Put all the others below
                for( unsigned i=nr_hrvos; i<max_hrvos; ++i )
                {
                    std::string name = "VO_" + int_to_str( i );

                    params.push_back( 0 ); params.push_back( 1 ); params.push_back( -100 );
                    params.push_back( 0 ); params.push_back( 0 ); params.push_back( -100 );
                    params.push_back( 1 ); params.push_back( 0 ); params.push_back( -100 );
                    geoms.push_back(geometry_info_t(name, "vo", PRX_LINESTRIP, params, "white") );
                    configs.push_back( sys_config );
                    params.clear();
                }

                ((visualization_comm_t*)vis_comm)->visualization_geom_map[full_name] = geoms;
                ((visualization_comm_t*)vis_comm)->visualization_configs_map[full_name] = configs;

                ((visualization_comm_t*)vis_comm)->send_geometries();
            }

            void collision_avoidance_controller_t::set_queue_orientation(double queue_orientation)
            {
                this->queue_orientation = queue_orientation;
            }

        }
    }
}

