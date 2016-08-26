/**
 * @file path_follow_controller.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/utilities/heuristic_search/astar_search.hpp"

#include "simulation/controllers/path_follow_controller.hpp"
#include "simulation/controllers/collision_avoidance_controller.hpp"

#include "prx/simulation/communication/visualization_comm.hpp"
#include "simulation/sensing/node_sensing_info.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <math.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::crowd::path_follow_controller_t, prx::sim::system_t);

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
            double path_follow_controller_t::cc_time = 0;
            unsigned path_follow_controller_t::cc_calls = 0;
            double path_follow_controller_t::pf_time = 0;
            unsigned path_follow_controller_t::pf_calls = 0;

            path_follow_controller_t::path_follow_controller_t()
            {
                metric = NULL;
                graph_search = NULL;
                path_vertices_to_queue_counter = 0;

                _Cx = _Cy = _Cz = 0;
                control_memory = { &_Cx , &_Cy , &_Cz };
                input_control_space = new space_t( "XYZ", control_memory );
                input_control_space->zero();

                has_target = hindered = false;
                near_goal = false;
                goal_region = NULL;
                
                triangle_node = NULL;
                waypoint_node = NULL;
                next_point = NULL;
                target_point = NULL;
                frame_count = 0;
                father_to_ggfather_dist = 0;
                side_bias = 0.85;
                point_to_go.resize(3);
                agent_mode = PRX_AGENT_MODE_NORMAL;
                distance_to_target = PRX_INFINITY;
                lost_counter = 0;
            }

            path_follow_controller_t::~path_follow_controller_t()
            {
                child_state_space->free_point( current_state );
                child_state_space->free_point( prev_state );
                child_state_space->free_point( next_point );
                child_state_space->free_point( target_point );
                child_state_space->free_point( point_a );
                child_state_space->free_point( point_b );
            }

            void path_follow_controller_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
            {
                controller_t::init(reader, template_reader);

                system_ptr_t child_system = subsystems.begin()->second;
                child_state_space = child_system->get_state_space();
                current_state = child_state_space->alloc_point();
                prev_state = child_state_space->alloc_point();
                vo_controller = dynamic_cast< collision_avoidance_controller_t*>( child_system.get() );

                node_sensing_info_t* test = NULL;
                for(unsigned i = 0; i < sensing_info.size() && test == NULL; i++)
                {
                    test = dynamic_cast<node_sensing_info_t*>(sensing_info[i]);
                }
                PRX_ASSERT_MSG(test != NULL, "Path-Follower requires a node sening info (nav_graph_sensor).");
                node_info = test;
                
                next_point = child_state_space->alloc_point();
                target_point = child_state_space->alloc_point();

                point_a = child_state_space->alloc_point();
                point_b = child_state_space->alloc_point();
                called = false;
            }

            void path_follow_controller_t::propagate(const double simulation_step)
            {
                if( active )
                {
                    controller_t::propagate(simulation_step);

                    const std::pair< const nav_node_t*, const nav_node_t* >& near_nodes = node_info->near_nodes;                    
                    update_node_information( near_nodes.first, near_nodes.second );
                }
            }

            void path_follow_controller_t::clear_queue_data()
            {
                path_vertices_to_queue.resize(0);
                path_vertices_to_queue_counter = 0;
            }

            /// MAKE A generic FUNCTION FOR Astar search with source and destination
            bool path_follow_controller_t::generate_path_towards_queue(const nav_node_t* destination)
            {
                if(graph_search->solve(triangle_node->index,destination->index))
                {
                    graph_search->extract_path( path_vertices_to_queue);
                    path_vertices_to_queue_counter = 1; //Remove the starting node.
                    return true;
                }
                return false;
            }            

            void path_follow_controller_t::compute_control()
            {
                stats_clock.reset();

                double step_distance = child_state_space->distance(prev_state, current_state);
                // if(agent_mode == PRX_AGENT_MODE_NORMAL && step_distance <= MIN_STEP_CHECK)
                //     lost_counter++;
                // else
                //     lost_counter = 0;

                child_state_space->copy_point(prev_state, current_state);
                // PRX_PRINT(node_info->agent_index << ") mode:" << agent_mode << "    lost_counter: " << lost_counter << "/" << MAX_FRAME_COUNTER << "     step_distance:" << step_distance,  PRX_TEXT_CYAN);

                if( waypoint_node != NULL && agent_mode != PRX_AGENT_MODE_IN_QUEUE )
                {
                    
                    //If we are in a moving elevator
                    elevator_t* e = triangle_node->near_elevator;
                    //Make sure we have updated state information.
                    child_state_space->copy_to_point( current_state );
                    if( e != NULL && !e->is_open() && e->in_elevator( current_state->memory[0], current_state->memory[1], current_state->memory[2] ) )
                    {
                        //Then we should actually inform the VO controller to chill.
                        output_control_space->copy_from_point( current_state );
                        //PACIFY the VO controller... make it behave.
                        vo_controller->set_passive( true );
                    }
                    //Otherwise, chug as planned
                    else
                    {
                        //Make sure the vo_controller is not passive
                        vo_controller->set_passive( false );
                        //Compute where to go (INTERPOLATION METHOD)
                        find_next_point();
                        //And report what we compute
                        output_control_space->copy_from_point( computed_control );
                    }
                }
                else if(agent_mode == PRX_AGENT_MODE_IN_QUEUE)
                {
                    // lost_counter = 0;
                    vo_controller->set_passive( false );
                    find_next_point();
                    output_control_space->copy_from_point( computed_control );
                    // child_state_space->copy_to_point( current_state );
                    // output_control_space->copy_from_point( computed_control );
                }

                cc_time += stats_clock.measure();
                ++cc_calls;

                subsystems.begin()->second->compute_control();
            }

            const nav_node_t* path_follow_controller_t::get_nearest_node() const
            {
                return triangle_node;
            }

            void path_follow_controller_t::link_metric( distance_metric_t* input_metric )
            {
                metric = input_metric;
            }

            void path_follow_controller_t::link_search_primitives(util::undirected_graph_t* input_graph, util::astar_search_t* input_astar, util::distance_metric_t* input_metric )
            {
                nav_graph = input_graph;
                graph_search = input_astar;
                metric = input_metric;
            }


            bool path_follow_controller_t::need_new_target()
            {

                if( agent_mode == PRX_AGENT_MODE_NORMAL && (!has_target || waypoint_node == triangle_node ) )
                {
                    has_target = false;
                    return true;
                }
                return false;
            }

            void path_follow_controller_t::force_new_target()
            {
                agent_mode = PRX_AGENT_MODE_NORMAL;
                has_target = false;
            }

            bool path_follow_controller_t::is_the_agent_lost()
            {
                return lost_counter >= MAX_FRAME_COUNTER;
            }

            void path_follow_controller_t::reset_lost_counter()
            {
                lost_counter = 0;
            }

            bool path_follow_controller_t::is_near_by()
            {
                distance_to_target < QUEUE_EPSILON;
            }

            bool path_follow_controller_t::need_to_check_for_queue()
            {
                bool check_for_queue = false;
                if(agent_mode == PRX_AGENT_MODE_NORMAL && fabs(triangle_node->get_height() - goal_region->get_height())< Z_DISTANCE_CHECK)
                {
                    if(distance_to_target < RE_COMPUTE_DISTANCE)
                    {
                        check_for_queue = goal_region->need_to_check_for_queue(current_state);
                    }
                }
               return check_for_queue;
            }

            bool path_follow_controller_t::need_to_reserve_queue_point()
            {
                return ((agent_mode == PRX_AGENT_MODE_GOING_TOWARDS_QUEUE) && (path_vertices_to_queue_counter >=  path_vertices_to_queue.size()) );
            }

            bool path_follow_controller_t::need_to_go_to_queue()
            {
                return ((agent_mode == PRX_AGENT_MODE_GOING_TOWARDS_RESERVED_SLOT) && (path_vertices_to_queue_counter >=  path_vertices_to_queue.size()) );
            }

            bool path_follow_controller_t::is_agent_in_queue()
            {
                return agent_mode == PRX_AGENT_MODE_IN_QUEUE;
            }

            bool path_follow_controller_t::at_goal()
            {
                if( near_goal && waypoint_node == triangle_node )
                {
                    has_target = false;
                    return true;
                }
                return false;
            }

            void path_follow_controller_t::set_id( unsigned input_id )
            {
                node_info->agent_index = input_id;
            }

            unsigned path_follow_controller_t::get_id()
            {
                return node_info->agent_index;
            }

            void path_follow_controller_t::set_bias( double input_bias )
            {
                side_bias = input_bias;
            }

            void path_follow_controller_t::update_node_information( const nav_node_t* triangle, const nav_node_t* point )
            {
                if( triangle == NULL || point != NULL )
                {
                    // PRX_PRINT(node_info->agent_index << ") triangle is : " << triangle << "  point is: " << point, PRX_TEXT_RED);
                    triangle_node = triangle;
                    waypoint_node = point;
                    child_state_space->copy_point( next_point, point->point );
                    output_control_space->copy_point( computed_control, triangle_node->point );
                    father_to_ggfather_dist = PRX_ZERO_CHECK;
                    max_ratio = 1;
                    return;
                }
                //If we in the same triangle as before, we don't need to update anything
                if( triangle == triangle_node )
                {
                    return;
                }
                

                //Now, if the triangle is associated with our goal region, we are basically done
                if( goal_region != NULL && triangle->corresponding_region == goal_region && !near_goal)
                {
                    //We must be close to the goal
                    near_goal = true;
                }

                if( goal_region != NULL )
                {
                    // Check for agent - aim
                    PRX_ASSERT(triangle != NULL);
                    distance_to_target = triangle->region_distances[goal_region];
                    
                    // Check for agent_mode
                    triangle_node = triangle;
                    if(agent_mode == PRX_AGENT_MODE_NORMAL)
                    {
                        setup_waypoints( goal_region );

                    }
                    else if(agent_mode == PRX_AGENT_MODE_GOING_TOWARDS_QUEUE || agent_mode == PRX_AGENT_MODE_GOING_TOWARDS_RESERVED_SLOT)
                    {
                        setup_waypoints_towards_queue(goal_region);
                    }

                    //Then, do tracing from the triangle to find the grandparent
                    father_to_ggfather_dist = child_state_space->distance( target_point, next_point );
                    max_ratio = (child_state_space->distance( current_state, next_point ) / father_to_ggfather_dist) -1;
                }

                triangle_node = triangle;
                // PRX_PRINT(node_info->agent_index << ") update_node_information for agent : " << node_info->agent_index, PRX_TEXT_BROWN);
            }

            void path_follow_controller_t::go_to_region( region_t* new_region, bool set_hindered )
            {
                stats_clock.reset();
                
                //When we are given a goal region, we now report that we have a target
                has_target = true;
                
                //If we are directed to go to the same region, then we don't actually need to compute anything
                if( goal_region == new_region )
                {
                    return;
                }

                goal_region = new_region;
                hindered = set_hindered;

                near_goal = false;
                if( triangle_node->corresponding_region == goal_region )
                {
                    near_goal = true;
                }

                setup_waypoints( goal_region );               

                //Compute the father-to-great grandfather distance
                father_to_ggfather_dist = child_state_space->distance( target_point, next_point );
                max_ratio = (child_state_space->distance( current_state, next_point ) / father_to_ggfather_dist) -1;
            }

            void path_follow_controller_t::find_next_point()
            {
                //From our current state
                child_state_space->copy_to_point( current_state );
                
                //A pointer to the next position we want to get to.
                output_control_space->copy_point( computed_control, target_point );
                //If we are not at our destination triangle
                if( waypoint_node != triangle_node )
                {
                    //Interpolation Method
                    double t = 1;
                    if( father_to_ggfather_dist > 0.00001 )
                    {
                        double ratio = (child_state_space->distance( next_point, current_state ) / father_to_ggfather_dist) -1;
                        t = PRX_MAXIMUM( PRX_MINIMUM( ratio/max_ratio, 1), 0);
                    }
                    output_control_space->interpolate( next_point, target_point, t, computed_control );
                }
            }

            void path_follow_controller_t::setup_waypoints( region_t* goal_region )
            {
                if( hindered )
                {
                    waypoint_node = triangle_node->hindered_back_pointer[ goal_region ];
                    compute_segment_target( waypoint_node, current_state, target_point );
                    //If this waypoint has a grandfather override, listen to him
                    if( waypoint_node->hindered_grandfather_override.find( goal_region ) != waypoint_node->hindered_grandfather_override.end() )
                    {
                        child_state_space->copy_point( next_point, waypoint_node->hindered_grandfather_override[ goal_region ] );
                        child_state_space->copy_point( target_point, next_point );
                    }
                    //Otherwise, search up the next guy as you would normally
                    else
                    {
                        compute_segment_target( waypoint_node->hindered_back_pointer[ goal_region ], target_point, next_point );
                        // child_state_space->copy_point( next_point, (waypoint_node->hindered_back_pointer[ goal_region ])->point );
                    }
                }
                else
                {
                    waypoint_node = triangle_node->back_pointer[ goal_region ];
                    compute_segment_target( waypoint_node, current_state, target_point );
                    //If this waypoint has a grandfather override, listen to him
                    if( waypoint_node->grandfather_override.find( goal_region ) != waypoint_node->grandfather_override.end() )
                    {
                        child_state_space->copy_point( next_point, waypoint_node->grandfather_override[ goal_region ] );
                        child_state_space->copy_point( target_point, next_point );
                    }
                    //Otherwise, search up the next guy as you would normally
                    else
                    {
                        compute_segment_target( waypoint_node->back_pointer[ goal_region ], target_point, next_point );
                        // child_state_space->copy_point( next_point, (waypoint_node->back_pointer[ goal_region ])->point );
                    }
                }
            }

            void path_follow_controller_t::set_agent_mode_to_normal()
            {
                agent_mode = PRX_AGENT_MODE_NORMAL;
                vo_controller->set_conservative_turns(true);
            }

            void path_follow_controller_t::set_agent_mode(agent_mode_t mode)
            {
                agent_mode = mode;
                if(mode == PRX_AGENT_MODE_IN_QUEUE)
                {
                    vo_controller->set_conservative_turns(false);
                    vo_controller->set_in_queue(true,get_id());
                }
                else
                {
                    vo_controller->set_conservative_turns(true);
                    vo_controller->set_in_queue(false,get_id());
                }
            }

            agent_mode_t path_follow_controller_t::get_agent_mode()
            {
                return agent_mode;
            }

            bool path_follow_controller_t::is_agent_mode_normal()
            {
                return agent_mode == PRX_AGENT_MODE_NORMAL;
            }

            const nav_node_t* path_follow_controller_t::get_current_triangle_node()
            {
                return triangle_node;
            }

            void path_follow_controller_t::go_to_point( const vector<double>& point)
            {
                child_state_space->set_from_vector(point, target_point);
                child_state_space->set_from_vector(point, next_point);
            }

            void path_follow_controller_t::set_queue_orientation(double queue_orientation)
            {
                vo_controller->set_queue_orientation(queue_orientation);
            }

            bool path_follow_controller_t::go_to_destination( const nav_node_t* new_destination )
            {
                if(new_destination!=NULL)
                {
                    clear_queue_data();
                    if(new_destination == triangle_node || generate_path_towards_queue(new_destination))
                    {
                        destination = new_destination;
                        child_state_space->copy_point(target_point, destination->point);
                        child_state_space->copy_point(next_point, destination->point);
                        setup_waypoints_towards_queue(goal_region);
                        return true;
                    }
                }

                return false;
            }

            void path_follow_controller_t::leave_queue()
            {
                agent_mode = PRX_AGENT_MODE_NORMAL;
                // goal_region->get_point_to_go(point_to_go);
                // go_to_point(point_to_go);
                setup_waypoints(goal_region);
            }

            void path_follow_controller_t::setup_waypoints_towards_queue(region_t* goal_region)
            {
                int size = path_vertices_to_queue.size();
                bool first_line = true;
                bool get_next_point = false;
                if(path_vertices_to_queue_counter < size)
                {
                    do
                    {
                        get_next_point = false;
                        waypoint_node = nav_graph->get_vertex_as<nav_node_t>(path_vertices_to_queue[path_vertices_to_queue_counter]);
                        ++path_vertices_to_queue_counter;
                        if(triangle_node == waypoint_node && path_vertices_to_queue_counter < size)
                        {
                            get_next_point = true;
                        }
                        else if(waypoint_node->triangle_id == -1 && first_line)
                        {
                            first_line = false;
                            get_next_point = true;
                        }

                    }while(get_next_point);
                    child_state_space->copy_point(target_point, waypoint_node->point);
                    child_state_space->copy_point(next_point, waypoint_node->point);                    
                }
            }
            
            void path_follow_controller_t::compute_segment_target( const nav_node_t* segment_node, space_point_t* reference_state, space_point_t* segment_point )
            {
                //If the "segment" node ended up being a triangle (the end of the line)
                if( !segment_node->has_segment() )
                {
                    //Simply use the node's point
                    child_state_space->copy_point( segment_point, segment_node->point );
                    return;
                }
                
                //Let's get some shorthand to help out here
                const std::vector< double >& a = segment_node->segment[0];
                const std::vector< double >& b = segment_node->segment[1];
                std::vector< double >& c = reference_state->memory;
                
                //Check the normal
                double norm_z = ((b[0]-a[0])*(c[1]-a[1]))-((b[1]-a[1])*(c[0]-a[0]));

                const std::vector< double >& left = ( norm_z > 0 ) ? b : a;
                const std::vector< double >& right = ( norm_z > 0 ) ? a : b;
                
                //set left and right up
                child_state_space->copy_vector_to_point( left, point_a );
                child_state_space->copy_vector_to_point( right, point_b );
                
                //Then, simply set the segment point to be our preferred interpolation
                child_state_space->interpolate( point_a, point_b, side_bias, segment_point );
            }

            void path_follow_controller_t::go_to_closest_triangle()
            {
                if(triangle_node->triangle_id != -1 && triangle_node->neighboring_triangles.size() > 0)
                {
                    child_state_space->copy_point_to_vector(triangle_node->neighboring_triangles[0]->point, point_to_go);
                    go_to_point(point_to_go);
                }
            }

            system_ptr_t path_follow_controller_t::create_subsystem(const std::string& path, const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                system_ptr_t subsystem;

                subsystem.reset(parameters::create_from_loader<system_t > ("prx_simulation", reader, "", template_reader, ""));
                subsystem->set_pathname(path);

                subsystem->init(reader, template_reader);

                return subsystem;
            }
            
            void path_follow_controller_t::visualize_marker() const
            {
                std::vector<geometry_info_t> geoms;
                std::vector<config_t> configs;
                std::vector<double> params;

                config_t marker_config;
                config_t next_config;

                //Make sure we have a name for this
                std::string full_name = pathname + "/marker";

                params.push_back( 0.1 );
                params.push_back( 0.1 );
                params.push_back( 0.1 );
                
                //Also see what the low-level state we're steering to is?
                if( triangle_node != NULL )
                {
                    space_point_t* pt = triangle_node->point;
                    next_config.set_position( pt->memory[0], pt->memory[1], pt->memory[2] );
                }
                else
                {
                    next_config.set_position( 0, 0, -1 );
                }
                
                geoms.push_back( geometry_info_t( full_name, "next", PRX_BOX, params, "red" ) );
                configs.push_back( next_config );
                
                ((visualization_comm_t*)vis_comm)->visualization_geom_map[full_name] = geoms;
                ((visualization_comm_t*)vis_comm)->visualization_configs_map[full_name] = configs;                
                ((visualization_comm_t*)vis_comm)->send_geometries();
            }

            void path_follow_controller_t::get_current_position(std::vector<double>& current_position)
            {
                child_state_space->copy_point_to_vector(current_state,current_position);
            }
        }
    }
}
