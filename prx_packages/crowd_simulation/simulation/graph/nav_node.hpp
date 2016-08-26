/**
 * @file nav_node.hpp
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

#pragma once
#ifndef PRX_NAV_NODE_HPP
#define PRX_NAV_NODE_HPP

#include "simulation/structures/ramp.hpp"
#include "simulation/structures/region.hpp"
#include "simulation/structures/attractor.hpp"
#include "simulation/structures/origin.hpp"
#include "simulation/structures/elevator.hpp"
#include "simulation/structures/queue_struct.hpp"

#include "prx/utilities/graph/undirected_node.hpp"
#include "prx/utilities/boost/hash.hpp"

#include "prx/simulation/sensing/infos/2d_proximity_sensing_info.hpp"


namespace prx
{
    namespace packages
    {
        namespace crowd
        {
            class path_follow_controller_t;
            class agent_data_t;
            
            //Isn't this specified in the semantics file...?
            //enum influence_t {PRX_RESTROOM, PRX_VENDOR, PRX_FOOD, PRX_TICKETING, PRX_ATM, PRX_BENCH, PRX_PATROL_POINT, PRX_ORIGIN, PRX_FRIEND, PRX_NONE};
            struct attractor_info_t
            {
                attractor_info_t(attractor_t* attr, double distance, unsigned index )
                {
                    attractor = attr;
                    dist = distance;
                    doorway_index = index;
                }

                std::string print()
                {
                    std::stringstream output(std::stringstream::out);
                    output << attractor->name << "  distance:" << dist << "  doorway:" << doorway_index;
                    return output.str();
                }

                attractor_t* attractor;
                double dist;
                unsigned doorway_index;
            };

            class nav_node_t : public util::undirected_node_t 
            {
              public:
                nav_node_t();

                ~nav_node_t();

                void set_region(region_t* region);
                
                void set_triangle(int id, std::vector< double >::iterator start );
                
                bool point_in_triangle( const std::vector< double >& p ) const;
                
                bool has_triangle() const;

                void set_segment( std::vector< double >::iterator start );
                
                bool has_segment() const;

                void copy_navigation_info( nav_node_t* other );
                
                void add_agent( neighbor_t* agent );

                // Remove neighbor
                void remove_agent( neighbor_t* agent );

                // Remove neighbor using agent_id
                void remove_agent( int agent_id);

                // Insert queue segment and buffers
                void insert_queue_segment_structure(segment_struct_t* queue_segment_structure);

                void insert_wall_segment_structure(segment_struct_t* wall_segment_structure);

                void fill_queue_info(std::deque<segment_struct_t*>& arg_wall_segment_structures, util::hash_t<int, std::vector<segment_struct_t*> >& arg_queue_segment_structures, int segment_structure_search_id);

                double get_height() const;

                int triangle_id;
                
                sim::prox_elem_t* obstacle_info;
                std::vector< neighbor_t* > obstacle_neighbors;
                
                std::vector< attractor_info_t* > attractor_distances;
                //util::hash_t< origin_t*, double > origin_distances;
                util::hash_t< region_t*, double > region_distances;

                std::vector< attractor_info_t* > hindered_attractor_distances;
                //util::hash_t< origin_t*, double > hindered_origin_distances;
                util::hash_t< region_t*, double > hindered_region_distances;
                
                util::hash_t< region_t*, nav_node_t* > back_pointer;
                util::hash_t< region_t*, nav_node_t* > hindered_back_pointer;

                util::hash_t< region_t*, util::space_point_t* > grandfather_override;
                util::hash_t< region_t*, util::space_point_t* > hindered_grandfather_override;

                //Ramps stuffs
                unsigned ramp_index;
                ramp_t* near_ramp;

                //Elevator stuffs
                unsigned elevator_index;
                elevator_t* near_elevator;

                //Checks if this node is a region.
                bool is_region;
                region_t* corresponding_region;

                int search_id;
                int added_in_the_list;
                double stored_dist;

                std::vector< std::vector< double > > triangle;
                std::vector< std::vector< double > > segment;
    
                //Sensing helpers
                std::vector< nav_node_t* > neighboring_triangles;
                std::vector< neighbor_t* > occupying_agents;

                /////////////
                //  Queues //
                /////////////
                
                int neighbor_search_id;
                //This is the information for the queue manager for the near by walls and queue segments. 
                std::vector< std::pair< std::vector< double > , std::vector< double > > > walls;
                
                std::vector<segment_struct_t*>  queue_segment_structure_list;
                util::hash_t<int, std::vector<segment_struct_t*> > queue_segment_structures;
                std::deque< segment_struct_t* > wall_segment_structures;

                //Sean's code above

              protected:
                double area_const;
            };
        }
    }
}

#endif



