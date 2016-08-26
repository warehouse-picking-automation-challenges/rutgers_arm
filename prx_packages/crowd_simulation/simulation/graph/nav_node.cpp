/**
 * @file nav_node.cpp
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

#include "simulation/graph/nav_node.hpp"
#include <boost/range/adaptor/map.hpp> 

namespace prx
{
    namespace packages
    {
        namespace crowd
        {
            using namespace util;
            using namespace sim;            

            nav_node_t::nav_node_t() 
            {
                obstacle_info = NULL;
                is_region = false;
                corresponding_region = NULL;
                near_elevator = NULL;
                near_ramp = NULL;

                search_id = -1;
                neighbor_search_id = -1;
                added_in_the_list = -1;
                triangle_id = -1;
                triangle.clear();
                neighboring_triangles.clear();
                occupying_agents.clear();
                wall_segment_structures.clear();
            }

            nav_node_t::~nav_node_t() 
            {
                triangle.clear();
                neighboring_triangles.clear();
                occupying_agents.clear();
                neighboring_triangles.clear();

                for(unsigned i = 0; i < wall_segment_structures.size(); ++i)
                {
                    delete(wall_segment_structures[i]);
                    // delete(wall_buffer_lines[i]);
                }            
                //wall_segments.clear();
                wall_segment_structures.clear();
                // wall_buffer_lines.clear();
            }

            void nav_node_t::set_region(region_t* region)
            {
                is_region = true;
                corresponding_region = region;
            }
            
            void nav_node_t::set_triangle(int id, std::vector< double >::iterator start )
            {
                triangle_id = id;
                //Set up the triangle information
                triangle.resize( 3 );
                triangle[0].assign( start, start+3 );
                triangle[1].assign( start+3, start+6 );
                triangle[2].assign( start+6, start+9 );

                //Let's get some shorthand to help out here
                std::vector< double >& a = triangle[0];
                std::vector< double >& b = triangle[1];
                std::vector< double >& c = triangle[2];
                
                //Check the normal
                double norm_z = ((b[0]-a[0])*(c[1]-a[1]))-((b[1]-a[1])*(c[0]-a[0]));
                
                //If the normal is pointing down, we need to adjust the vertex order
                if( norm_z < 0 )
                {
                    //Swap points b and c
                    std::vector< double > swap = c;
                    c = b;
                    b = swap;
                }
                
                // PRX_PRINT( "Embiggening Triangle: ", PRX_TEXT_CYAN );
                
                //Need to slightly embiggen these things?
                for( unsigned i=0; i<triangle.size(); ++i )
                {
                    std::vector< double >& v = triangle[i];
                    // PRX_PRINT("Before (" << v[0] << ", " << v[1] << ", " << v[2] << ")", PRX_TEXT_LIGHTGRAY);
                    double difx = v[0] - point->memory[0];
                    double dify = v[1] - point->memory[1];
                    v[0] += difx * 0.0001;
                    v[1] += dify * 0.0001;                        
                    // PRX_PRINT("After  (" << v[0] << ", " << v[1] << ", " << v[2] << ")", PRX_TEXT_LIGHTGRAY);
                }
                
                //Also need to set the area constant
                area_const = 1.0/( -b[1]*c[0] + a[1]*( c[0]-b[0] ) +a[0]*( b[1]-c[1] ) +b[0]*c[1] );
            }
            
            bool nav_node_t::point_in_triangle( const std::vector< double >& p ) const
            {
                //Let's get some shorthand to help out here
                const std::vector< double >& a = triangle[0];
                const std::vector< double >& b = triangle[1];
                const std::vector< double >& c = triangle[2];
                
                double s = area_const*( a[1]*c[0] - a[0]*c[1] + p[0]*( c[1]-a[1] ) + p[1]*( a[0]-c[0] ) );
                if( s < 0 )
                {
                    return false;
                }
                double t = area_const*( a[0]*b[1] - a[1]*b[0] + p[0]*( a[1]-b[1] ) + p[1]*( b[0]-a[0] ) );
                if( t < 0 || (1-s-t) < 0 )
                {
                    return false;
                }
                
                return true;
            }
            
            bool nav_node_t::has_triangle() const
            {
                return triangle.size() > 0;
            }

            void nav_node_t::set_segment( std::vector< double >::iterator start )
            {   
                //Assign the segment information                 
                segment.resize( 2 );
                segment[0].assign( start, start+3 );
                segment[1].assign( start+3, start+6 );
            }
            
            bool nav_node_t::has_segment() const
            {
                return segment.size() > 0;
            }
            
            void nav_node_t::copy_navigation_info( nav_node_t* other )
            {
                attractor_distances = other->attractor_distances;
                hindered_attractor_distances = other->hindered_attractor_distances;
                /* This is Andrew's Code - Now we are using region instead of origin*/
                /*origin_distances = other->origin_distances;
                hindered_origin_distances = other->hindered_origin_distances;
                */
                region_distances = other->region_distances;
                hindered_region_distances = other->hindered_region_distances;
                
                back_pointer = other->back_pointer;
                hindered_back_pointer = other->hindered_back_pointer;
            }
            
            void nav_node_t::add_agent( neighbor_t* agent )
            {
                occupying_agents.push_back( agent );
            }
            
            void nav_node_t::remove_agent( neighbor_t* agent )
            {
                std::vector< neighbor_t* >::iterator loc = std::find( occupying_agents.begin(), occupying_agents.end(), agent );
                if( loc != occupying_agents.end() )
                {
                    occupying_agents.erase( loc );
                }
            }

            void nav_node_t::remove_agent( int agent_id)
            {
                std::vector< neighbor_t* >::iterator loc;
                for(loc=occupying_agents.begin();loc!=occupying_agents.end();++loc)
                {
                    if((*loc)->get_agent_index() == agent_id)
                    {
                        break;
                    }
                }
                if( loc != occupying_agents.end() )
                {
                    occupying_agents.erase( loc );
                }

            }

            void nav_node_t::insert_queue_segment_structure(segment_struct_t* queue_segment_structure)
            {
                if(queue_segment_structures.find(queue_segment_structure->object_id) != queue_segment_structures.end())
                {
                    queue_segment_structures[queue_segment_structure->object_id].push_back(queue_segment_structure);    
                }
                else
                {
                    queue_segment_structure_list.clear();
                    queue_segment_structure_list.push_back(queue_segment_structure);
                    queue_segment_structures[queue_segment_structure->object_id] = queue_segment_structure_list;
                }

            }

            void nav_node_t::insert_wall_segment_structure(segment_struct_t* wall_segment_structure)
            {
                wall_segment_structures.push_back(wall_segment_structure);
            }

            void nav_node_t::fill_queue_info(std::deque<segment_struct_t*>& arg_wall_segment_structures, util::hash_t<int, std::vector<segment_struct_t*> >& arg_queue_segment_structures, int segment_structure_search_id)
            {
                foreach(segment_struct_t* segment_structure, wall_segment_structures)
                {
                    if(segment_structure->search_id != segment_structure_search_id)
                    {
                        segment_structure->search_id = segment_structure_search_id;
                        arg_wall_segment_structures.push_back(segment_structure);
                    }
                }

                foreach(std::vector<segment_struct_t*> segment_structures, queue_segment_structures | boost::adaptors::map_values)
                {
                    foreach(segment_struct_t* segment_structure, segment_structures)
                    {
                        if(segment_structure->search_id != segment_structure_search_id)
                        {
                            segment_structure->search_id = segment_structure_search_id;
                            if(arg_queue_segment_structures.find(segment_structure->object_id) != arg_queue_segment_structures.end())
                            {
                                arg_queue_segment_structures[segment_structure->object_id].push_back(segment_structure);
                            }
                            else
                            {
                                queue_segment_structure_list.clear();
                                queue_segment_structure_list.push_back(segment_structure);
                                arg_queue_segment_structures[segment_structure->object_id] = queue_segment_structure_list;
                            }
                        }
                    }            
                }
            }

            double nav_node_t::get_height() const
            {
                return point->at(2);
            }
        }
    }
}

