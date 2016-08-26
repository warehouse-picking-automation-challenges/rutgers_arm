/**
 * @file elevator.hpp
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

#include "simulation/structures/elevator.hpp"
#include "prx/simulation/systems/system.hpp"

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/definitions/random.hpp"

namespace prx
{
    using namespace util;

    namespace packages
    {
        namespace crowd
        {

            elevator_t::elevator_t()
            {
                shaft_geometry = NULL;
                bounds.resize( 3 );
                
                moving = false;
                going_up = true;

                max_transition_time = 400;
                frames_in_transition = uniform_int_random( 100, max_transition_time );
                velocity = 0;
                height = 0;
                at_stop = 0;
            }

            elevator_t::~elevator_t()
            {
                if( shaft_geometry != NULL )
                {
                    delete shaft_geometry;
                }
            }

            void elevator_t::frame()
            {
                //Move the elevator according to its velocity
                height += velocity;
                //Update the frame counter
                --frames_in_transition;
                
                //If we have reached the end of this transition
                if( frames_in_transition == 0 )
                {
                    //We always will restart our frame counter
                    frames_in_transition = max_transition_time;
                    //We always transition between moving and not moving
                    moving = !moving;
                    //If we have started moving
                    if( moving )
                    {
                        //See if we are at the top or bottom
                        if( at_stop == 0 )
                        {
                            //We're at the bottom so make sure to go up
                            going_up = true;
                        }
                        else if( at_stop == stops.size()-1 )
                        {
                            //We're at the top, so make sure to go down
                            going_up = false;
                        }
                        
                        //Compute the velocity we have to follow
                        if( going_up )
                        {
                            velocity = ( stops[at_stop+1].first - stops[at_stop].first)/((double)max_transition_time );
                            ++at_stop;
                        }
                        else
                        {
                            velocity = ( stops[at_stop-1].first - stops[at_stop].first)/((double)max_transition_time );
                            --at_stop;
                        }
                    }
                    //Otherwise, if we have stopped
                    else
                    {
                        //Simply stop the elevator
                        velocity = 0;
                    }
                }
            }

            bool elevator_t::in_elevator( double in_x, double in_y, double in_z )
            {
                return  bounds[0].is_valid( in_x ) &&
                        bounds[1].is_valid( in_y ) &&
                        bounds[2].is_valid( in_z );
            }

            bool elevator_t::approximately_in_elevator( double in_x, double in_y, double in_z )
            {
                return  approximate_bounds[0].is_valid( in_x ) &&
                        approximate_bounds[1].is_valid( in_y ) &&
                        approximate_bounds[2].is_valid( in_z );
            }

            bool elevator_t::is_open()
            {
                return velocity == 0;
            }
            
            double elevator_t::current_height()
            {
                return height + 1;
            }
            
            const nav_node_t* elevator_t::current_node()
            {
                return stops[at_stop].second;
            }

            bool elevator_t::matches_bounds( double min_x, double max_x, double min_y, double max_y )
            {
                if( stops.size() > 0 )
                {
                    return  min_x == bounds[0].get_lower_bound() &&
                            max_x == bounds[0].get_upper_bound() &&
                            min_y == bounds[1].get_lower_bound() &&
                            max_y == bounds[1].get_upper_bound();
                }
                return false;
            }

            void elevator_t::set_bounds( double min_x, double max_x, double min_y, double max_y )
            {
                bounds[0].set_bounds( min_x, max_x );
                bounds[1].set_bounds( min_y, max_y );
            }
            
            void elevator_t::add_stop( const std::pair< double, const nav_node_t* >& stop  )
            {
                //If we already have this height
                if( std::find( stops.begin(), stops.end(), stop ) != stops.end() )
                {
                    //Something went wrong
                    PRX_WARN_S("Elevator shaft already has a stop at " << stop.first);
                    return;
                }
                
                stops.push_back( stop );
                std::sort( stops.begin(), stops.end() );
                
                bounds[2].set_bounds( stops[0].first, stops[ stops.size()-1 ].first+3 );
            }

            void elevator_t::compute_shaft_geometry()
            {
                if( shaft_geometry != NULL )
                {
                    delete shaft_geometry;
                }
                shaft_geometry = new geometry_t();
                
                shaft_geometry->set_box(bounds[0].get_upper_bound() - bounds[0].get_lower_bound() + 0.25,
                                        bounds[1].get_upper_bound() - bounds[1].get_lower_bound() + 0.25,
                                        bounds[2].get_upper_bound() - bounds[2].get_lower_bound() + 0.25);
                
                configuration.set_position( (bounds[0].get_upper_bound() + bounds[0].get_lower_bound())/2.0,
                                            (bounds[1].get_upper_bound() + bounds[1].get_lower_bound())/2.0,
                                            (bounds[2].get_upper_bound() + bounds[2].get_lower_bound())/2.0 );
                
                //We also know where we should be starting then
                height = bounds[2].get_lower_bound();
                
                //Make the approximate bounds
                approximate_bounds.resize( 3 );
                approximate_bounds[0] = bounds[0];
                approximate_bounds[1] = bounds[1];
                approximate_bounds[2] = bounds[2];
                approximate_bounds[0].expand( bounds[0].get_upper_bound() + 0.001 );
                approximate_bounds[0].expand( bounds[0].get_lower_bound() - 0.001 );
                approximate_bounds[1].expand( bounds[1].get_upper_bound() + 0.001 );
                approximate_bounds[1].expand( bounds[1].get_lower_bound() - 0.001 );
                approximate_bounds[2].expand( bounds[2].get_upper_bound() + 0.001 );
                approximate_bounds[2].expand( bounds[2].get_lower_bound() - 0.001 );
                
                //Once we've computed this geometry, we can set up the neighbor information...?
                neighbor.set_name( name );
                neighbor.set_reciprocity( false );
                neighbor.set_obstacle_marker( true );
                neighbor.set_neighbor_radius( shaft_geometry->get_bounding_radius() );
                neighbor.set_neighbor_geotype( PRX_BOX );
                neighbor.set_neighbor_center( configuration.get_position()[0], configuration.get_position()[1] );
                neighbor.set_neighbor_velocity( 0, 0 );
                neighbor.set_valid( true );
            }

            bool elevator_t::has_stop_at_height( double test_height )
            {
                for( unsigned i=0; i<stops.size(); ++i )
                {
                    //If it's close enough
                    if( fabs( test_height - 1 - stops[i].first ) < 0.2  )
                    {
                        //We stop here
                        return true;
                    }
                }
                //Nothing worked out, so we must not stop here
                return false;
            }
            
            double elevator_t::distance( double in_x, double in_y )
            {
                double difx = configuration.get_position()[0] - in_x;
                double dify = configuration.get_position()[1] - in_y;
                
                return difx*difx + dify*dify;
            }
            
            void elevator_t::print() const
            {
                PRX_PRINT("Elevator:" << bounds[0] << " :" << bounds[1] << " :" << bounds[2], PRX_TEXT_GREEN);
                PRX_PRINT("Aprx. Bnds: :" << approximate_bounds[0] << " :" << approximate_bounds[1] << " :" << approximate_bounds[2], PRX_TEXT_BROWN);
                PRX_PRINT("Neighbor name: " << neighbor.get_name(), PRX_TEXT_CYAN);
                for( unsigned i=0; i<stops.size(); ++i )
                {
                    PRX_PRINT("Stop [" << i << "]: " << stops[i].first, PRX_TEXT_LIGHTGRAY);
                }
            }
            
        }
    }
}


