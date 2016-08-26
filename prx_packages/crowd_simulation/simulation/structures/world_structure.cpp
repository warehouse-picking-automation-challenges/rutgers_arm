/**
 * @file world_structure.cpp
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

#include "simulation/structures/world_structure.hpp"
 
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"

namespace prx
{
    using namespace util;

    namespace packages
    {
        namespace crowd
        {
            world_structure_t::world_structure_t()
            {
                navigation_space = new space_t( "XYZ", {&_Nx,&_Ny,&_Nz} );
                PRX_PRINT("Allocated the navigation space: " << navigation_space << " : " << navigation_space->get_dimension(), PRX_TEXT_BROWN);
                
                origins.resize(0);
                attractors.resize(0);
                regions.resize(0);
            }

            world_structure_t::~world_structure_t()
            {
                for( unsigned i=0; i<attractors.size(); ++i )
                {
                    delete attractors[i];
                }
                for( unsigned i=0; i<origins.size(); ++i )
                {
                    delete origins[i];
                }

                delete navigation_space;
            }

            void world_structure_t::init( const parameter_reader_t * reader, const parameter_reader_t* template_reader )
            {
            }

            void world_structure_t::add_elevator( double min_x, double max_x, double min_y, double max_y, const std::pair< double, const nav_node_t* >& stop )
            {
                // PRX_PRINT("Elevator shaft at: (" << min_x << ", " << min_y << ") - (" << max_x << ", " << max_y << ") -- " << height, PRX_TEXT_RED);
                
                //Search through all of the elevators we have
                for( unsigned i=0; i<elevators.size(); ++i )
                {
                    if( elevators[i]->matches_bounds( min_x, max_x, min_y, max_y ) )
                    {
                        elevators[i]->add_stop( stop );
                        return;
                    }
                }
                //We didn't find this elevator shaft, so it must be new
                elevators.push_back( new elevator_t() );
                elevators.back()->set_bounds( min_x, max_x, min_y, max_y );
                elevators.back()->add_stop( stop );
            }

            void world_structure_t::add_ramp( ramp_t* input_ramp )
            {
                ramps.push_back( input_ramp );
            }

            void world_structure_t::fix_state( space_point_t* agent_state, const path_follow_controller_t* path_cont )
            {
                //Alright, figure out what's the closest ramp
                double& x = agent_state->memory[0];
                double& y = agent_state->memory[1];
                
                ramp_t* closest_ramp = NULL;
                elevator_t* closest_elevator = NULL;
                
                const nav_node_t* node = path_cont->get_nearest_node();
                
                if( node != NULL )
                {
                    closest_ramp = node->near_ramp;
                    closest_elevator = node->near_elevator;

                    if( closest_ramp != NULL )
                    {
                        if( closest_ramp->on_ramp( x, y ) )
                        {
                            if( closest_ramp->is_escalator )
                            {
                                x += closest_ramp->flow[0];
                                y += closest_ramp->flow[1];
                            }
                            agent_state->memory[2] = closest_ramp->height_on_ramp( x, y );
                        }
                    }
                    else if( closest_elevator != NULL )
                    {
                        if( closest_elevator->in_elevator( x, y, agent_state->memory[2] ) )
                        {
                            agent_state->memory[2] = closest_elevator->current_height();
                        }
                    }
                    
                }
            }

            void world_structure_t::init_regions( const util::parameter_reader_t * reader, const util::hash_t<std::string, unsigned>& origin_index, const util::hash_t<std::string, unsigned>& attractor_index, double MSEC_2_FRAMES)
            {
                unsigned id = 0; 
                foreach( const parameter_reader_t* r, reader->get_list("Local") )
                {
                    //This is an origin
                    if( r->has_attribute("OriginsType") )
                    {
                        origins.push_back( new origin_t( navigation_space, origin_index[r->get_attribute("OriginsType")], id ) );
                        ++id;
                        origins.back()->init(r, MSEC_2_FRAMES);
                        PRX_DEBUG_COLOR(" :: Origin: " << origins.back()->name<< "    type: " << origins.back()->type, PRX_TEXT_CYAN );
                    }
                    //This is an attraction
                    if( r->has_attribute("AttractionType") )
                    {
                        std::string type = r->get_attribute("AttractionType");
                        attractors.push_back( new attractor_t( navigation_space, attractor_index[type], id ) );
                        ++id;
                        attractors.back()->init(r,MSEC_2_FRAMES);
                        PRX_DEBUG_COLOR(" :: Attraction: " << attractors.back()->name << "    type: " << attractors.back()->type, PRX_TEXT_LIGHTGRAY );
                    }
                }
            }

            void world_structure_t::init_regions_from_file( const YAML::Node& node, const util::hash_t<std::string, unsigned>& origin_index, const util::hash_t<std::string, unsigned>& attractor_index, double MSEC_2_FRAMES)
            {
                unsigned id = 0; 
                for(int i = 0; i < node.size(); ++i)
                {
                    if(node[i]["OriginsType"])
                    {
                        origins.push_back( new origin_t( navigation_space, origin_index[ node[i]["OriginsType"].as<std::string>() ], id ) );
                        ++id;
                        origins.back()->init_from_file(node[i], MSEC_2_FRAMES);
                        PRX_DEBUG_COLOR(" :: Origin: " << origins.back()->name<< "    type: " << origins.back()->type, PRX_TEXT_CYAN );
                    }
                    else if(node[i]["AttractionType"])
                    {
                        attractors.push_back( new attractor_t( navigation_space, attractor_index[ node[i]["AttractionType"].as<std::string>() ], id ) );
                        ++id;
                        attractors.back()->init_from_file(node[i],MSEC_2_FRAMES);
                        PRX_DEBUG_COLOR(" :: Attraction: " << attractors.back()->name << "    type: " << attractors.back()->type, PRX_TEXT_LIGHTGRAY );
                    }
                }
            }

            void world_structure_t::frame()
            {
                for( unsigned i=0; i<elevators.size(); ++i )
                {
                    elevators[i]->frame();
                }
            }

            const std::vector< elevator_t* >& world_structure_t::get_elevators() const
            {
                return elevators;
            }

            const std::vector< origin_t* >& world_structure_t::get_origins() const
            {
                return origins;
            }

            const std::vector< attractor_t* >& world_structure_t::get_attractors() const
            {
                return attractors;
            }
            
            const std::vector< region_t* >& world_structure_t::get_all_regions()
            {                
                if(regions.empty())
                {
                   regions.assign( attractors.begin(), attractors.end() );
                   regions.insert( regions.end(), origins.begin(), origins.end() );
                }
                return regions;
            }

            origin_t* world_structure_t::get_origin(std::string name) const
            {
                foreach(origin_t* origin, origins)
                {
                    if(origin->name == name)
                    {
                        return origin;
                    }
                }
                return NULL;
            }

            attractor_t* world_structure_t::get_attractor(std::string name) const
            {
                foreach(attractor_t* attraction, attractors)
                {
                    if(attraction->name == name)
                        return attraction;
                }
                return NULL;
            }
            
            region_t* world_structure_t::get_region( std::string name ) const
            {
                region_t* region = get_attractor( name );
                if( region != NULL )
                    return region;
                region = get_origin( name );
                return region;
            }

            void world_structure_t::print()
            {
                PRX_PRINT("Has " << ramps.size() << " ramps.", PRX_TEXT_GREEN );
                for( unsigned i=0; i<ramps.size(); ++i )
                {
                    ramps[i]->print_points();
                }
                
                PRX_PRINT("Has " << elevators.size() << " elevators.", PRX_TEXT_RED);
                for( unsigned i=0; i<elevators.size(); ++i )
                {
                    elevators[i]->print();
                }
            }

        }
    }
}

