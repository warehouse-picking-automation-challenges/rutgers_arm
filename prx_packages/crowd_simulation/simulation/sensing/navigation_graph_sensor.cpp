/**
 * @file navigation_graph_sensor.cpp 
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

#include "prx/utilities/parameters/parameter_reader.hpp"

#include "simulation/graph/nav_node.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/simulation/simulators/simulator.hpp"
#include "prx/simulation/system_graph.hpp"
#include "prx/simulation/systems/plants/plant.hpp"

#include "simulation/controllers/VO_structure/neighbor.hpp"
#include "simulation/sensing/navigation_graph_sensor.hpp"
#include "simulation/structures/agent_data.hpp"

#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::crowd::navigation_graph_sensor_t, prx::sim::sensor_t);

namespace prx
{
    using namespace util;
    using namespace sim;
    
    namespace packages
    {
        namespace crowd
        {
            double navigation_graph_sensor_t::update_time = 0;
            unsigned navigation_graph_sensor_t::update_calls = 0;
        
            navigation_graph_sensor_t::navigation_graph_sensor_t()
            {
                point_vector_memory.resize( 3 );
                query_point = neighbor_point = NULL;
                active_extra_neighbors = 0;
                extra_neighbors_per_triangle.clear();
            }

            navigation_graph_sensor_t::~navigation_graph_sensor_t()
            {
                if( query_point != NULL )
                {
                    all_plants[0]->get_state_space()->free_point( query_point );
                    all_plants[0]->get_state_space()->free_point( neighbor_point );                
                }

                foreach(neighbor_t* neigh, extra_neighbors)
                {
                    delete neigh;
                }
                extra_neighbors.clear();

                for(unsigned i = 0; i < extra_neighbors_per_triangle.size(); ++i)
                    extra_neighbors_per_triangle[i].clear();
                extra_neighbors_per_triangle.clear();
            }
            
            void navigation_graph_sensor_t::init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader)
            {
                sensor_t::init(reader, template_reader);

                agent_radius = parameters::get_attribute_as< double >("agent_radius", reader, template_reader);
                sensing_radius = parameters::get_attribute_as< double >("sensing_radius", reader, template_reader, 4);
                max_neighbors = parameters::get_attribute_as< unsigned >("maximum_neighbors", reader, template_reader);
            }

            void navigation_graph_sensor_t::initialize_sensor(simulator_t* sim)
            {
            }

            void navigation_graph_sensor_t::link_navigation_primitives( util::undirected_graph_t* input_graph, distance_metric_t* input_metric, std::vector< agent_data_t >& input_agents )
            {
                graph_pointer = input_graph;
                metric = input_metric;
                
                for( unsigned i=0; i<input_agents.size(); ++i )
                {
                    all_agents.push_back( &( input_agents[i] ) );
                    all_plants.push_back( input_agents[i].plant );
                }
                
                //An X and Y coordinate for each of these plants
                prior_position.resize( 2*all_plants.size() );

                //Initialize node pointers to NULL
                agent_nodes.resize( all_plants.size() );
                waypoint_nodes.resize( all_plants.size() );
                for( unsigned i=0; i<agent_nodes.size(); ++i )
                {
                    agent_nodes[i] = NULL;
                    waypoint_nodes[i] = NULL;
                }
                //Because we're doing this just for the PABT, we can assume all the plants are the same
                query_point = all_plants[0]->get_state_space()->alloc_point();
                neighbor_point = all_plants[0]->get_state_space()->alloc_point();
                
                //We also need to try to initialize some neighbor information
                all_neighbors.resize( input_agents.size() );
                all_neighborhoods.resize( input_agents.size() );
                for( unsigned i=0; i<all_neighbors.size(); ++i )
                {
                    //A little bit of a hack, but this is replacing basically the same assumption we had elsewhere in the code
                    all_neighbors[i].set_agent_index(i);
                    all_neighbors[i].set_name( all_plants[i]->get_pathname() + "/body" );
                    all_neighbors[i].set_neighbor_geotype( PRX_CYLINDER );
                    all_neighbors[i].set_neighbor_radius( agent_radius );
                    all_neighbors[i].set_reciprocity( true );
                    all_neighbors[i].set_obstacle_marker( false );
                }
            }
            
            void navigation_graph_sensor_t::update_data()
            {
                // PRX_PRINT("UPDATE DATA navigation_graph_sensor", PRX_TEXT_BROWN);
                const nav_node_t* new_triangle = NULL;
                const nav_node_t* waypoint = NULL;
                
                //Go over every plant
                for( unsigned i=0; i<all_plants.size(); ++i )
                {

                    plant_t* system = all_plants[i];
                    //Reset the waypoint pointer
                    waypoint = NULL;
                    
                    //If it is active
                    if( system->is_active() )
                    {
                        stats_clock.reset();

                        //If it has node information
                        if( agent_nodes[i] != NULL && !(all_agents[i]->just_spawned) )
                        {
                            //Get the agent's state
                            system->get_state_space()->copy_to_vector( point_vector_memory );

                            //If we are in a moving elevator
                            elevator_t* e = agent_nodes[i]->near_elevator;
                            if( e != NULL && !e->is_open() && e->in_elevator( point_vector_memory[0], point_vector_memory[1], point_vector_memory[2] ) )
                            {
                                //Our triangle point shall be dictated by the elevator
                                new_triangle = e->current_node();
                            }
                            //Otherwise, we proceed as normal
                            else
                            {
                                //If it is no longer in the triangle we believe it to be
                                if( !agent_nodes[i]->point_in_triangle( point_vector_memory ) )
                                {
                                    //Search for the appropriate triangle for this agent
                                    const nav_node_t* result = find_triangle( agent_nodes[i], point_vector_memory );

                                    //If we couldn't find anything
                                    if( result == NULL )
                                    {
                                        //PRX_PRINT("Agent who lost his triangle  index "<<i<<" id "<<all_agents[i]->agent_id<<" type "<<all_agents[i]->agent_type,PRX_TEXT_BROWN);
                                        //Enforce that we use whatever our previous triangle information was
                                        new_triangle = agent_nodes[i];
                                        //AND, tell it specifically to move to that point
                                        waypoint = agent_nodes[i];
                                    }
                                    else
                                    {
                                        new_triangle = result;
                                    }
                                }
                                else
                                {
                                    new_triangle = agent_nodes[i];
                                }
                            }
                        }
                        //Otherwise, it has no info, so it must have just spawned
                        else
                        {
                            //Perform a NN query to find the correct triangle to use 
                            system->get_state_space()->copy_to_point( query_point );
                            new_triangle = dynamic_cast< const nav_node_t*>( metric->single_query( query_point ) );
                            //This agent must have just spawned, so make sure to set its prior state information
                            prior_position[ 2*i ] = query_point->memory[0];
                            prior_position[ 2*i +1 ] = query_point->memory[1];
                            //Then, report that this agent spawn has been detected
                            all_agents[i]->just_spawned = false;
                        }
                        //Then, we need to update the information on the nav graph
                        update_node_information( i, new_triangle, waypoint );

                        //We also need to update the neighbor_t information for this agent
                        update_neighbor_information( i );

                        update_time += stats_clock.measure();
                    }
                    //Otherwise, if it is inactive, clear out its information
                    else
                    {
                        //Ensure that it has no node data
                        agent_nodes[i] = NULL;
                        waypoint_nodes[i] = NULL;
                    }
                }
                
                // PRX_PRINT("==========================================", PRX_TEXT_MAGENTA);
                // PRX_PRINT("    Computing neighborhood Information    ", PRX_TEXT_BLUE);
                // PRX_PRINT("==========================================", PRX_TEXT_MAGENTA);
                //A second run to update actual neighborhood information
                for( unsigned i=0; i<all_plants.size(); ++i )
                {
                    plant_t* system = all_plants[i];
                    //If it is active

                    if( system->is_active() )
                    {
                        stats_clock.reset();
                        update_neighborhood( i );
                        update_time += stats_clock.measure();
                        ++update_calls;
                    }                  
                }
            }

            bool navigation_graph_sensor_t::fires()
            {
                if(!periodic_sensing)
                {
                    return false;
                }
                //Okay, so just need the logic here which appropriately handles things?
                ros::Duration duration_since_update = simulation::simulation_time - last_sense_time;

                if( sensor_delay <= duration_since_update.toSec() || duration_since_update.toSec() == 0)
                {
                    last_sense_time = simulation::simulation_time;
                    return true;
                }
                return false;
            }

            void navigation_graph_sensor_t::get_near_nodes( unsigned index, std::pair< const nav_node_t*, const nav_node_t* >& node_data )
            {
                node_data.first = agent_nodes[index];
                node_data.second = waypoint_nodes[index];
            }

            const nav_node_t* navigation_graph_sensor_t::find_triangle( const nav_node_t* start, const std::vector< double >& current_state )
            {
                //Keep a list of triangles we have tested (closed list)
                std::vector< const nav_node_t* > checked_triangles;

                // Check if the current_state is in the same nav node - ADITYA
                if( start->point_in_triangle( current_state ) )
                {
                    return start;
                }

                checked_triangles.push_back( start );
                
                //Also need a list that is a frontier of nodes (open list)
                std::deque< std::pair< const nav_node_t*, unsigned > > frontier;
                //Initialize the frontier to be start's neighborhood
                foreach( nav_node_t* v, start->neighboring_triangles )
                {
                    frontier.push_back( std::pair< const nav_node_t*, unsigned >( v, 1 ) );
                }
                
                //Now, we search
                while( !frontier.empty() )
                {
                    //Get the next frontier node to search
                    std::pair< const nav_node_t*, unsigned >& front = frontier.front();
                    const nav_node_t* neighbor = front.first;
                    unsigned depth = front.second;
                    frontier.pop_front();
                    checked_triangles.push_back( neighbor );
                    
                    //If we are in this triangle, we are done immediately
                    if( neighbor->point_in_triangle( current_state ) )
                    {
                        return neighbor;
                    }
                    
                    if( depth < 5 )
                    {
                        foreach( nav_node_t* candidate, neighbor->neighboring_triangles )
                        {
                            //If this candidate is still on our floor
                            if( fabs( candidate->point->memory[2] - current_state[2] ) < 2.0 )
                            {
                                // If we have not checked this triangle yet
                                if( std::find( checked_triangles.begin(), checked_triangles.end(), candidate ) == checked_triangles.end() )
                                {
                                    // Add it to the frontier
                                    frontier.push_back( std::pair< const nav_node_t*, unsigned >( candidate, depth +1 ) );
                                }
                            }
                        }
                    }
                }
                
                // PRX_PRINT("Agent got completely lost, could not find its triangle!",PRX_TEXT_BROWN);
                //PRX_PRINT("Agent at ::  " << current_state[0] << ", " << current_state[1] << ", " << current_state[2], PRX_TEXT_LIGHTGRAY );
                //PRX_PRINT("Checked Triangles: (" << checked_triangles.size() << ")", PRX_TEXT_CYAN );
                // for( unsigned i=0; i<checked_triangles.size(); ++i )
                // {
                //     PRX_PRINT("[" << i << "]:  (" << checked_triangles[i]->triangle[0][0] << ", " << checked_triangles[i]->triangle[0][1] << ", " << checked_triangles[i]->triangle[0][2] << ") ,  (" << checked_triangles[i]->triangle[1][0] << ", " << checked_triangles[i]->triangle[1][1] << ", " << checked_triangles[i]->triangle[1][2] << ") ,  (" << checked_triangles[i]->triangle[2][0] << ", " << checked_triangles[i]->triangle[2][1] << ", " << checked_triangles[i]->triangle[2][2] << ")", PRX_TEXT_LIGHTGRAY);
                // }
                return NULL;
            }

            void navigation_graph_sensor_t::update_node_information( unsigned index, const nav_node_t* triangle, const nav_node_t* point )
            {
                //Let's start by updating the near-node information
                waypoint_nodes[index] = point;

                //If we are going to the same triangle as before, we don't need to update anything
                if( triangle == agent_nodes[index] )
                {
                    return;
                }
                //We have changed triangle, so we need to update the nav-graph information
                if( agent_nodes[index] != NULL )
                {
                    //Remove it from this thing's list
                    (const_cast< nav_node_t* >(agent_nodes[index]))->remove_agent( &all_neighbors[index] );
                }
                agent_nodes[index] = triangle;
                all_neighbors[index].set_triangle_id(triangle->triangle_id);
                //Add ourselves to the new triangle
                (const_cast< nav_node_t* >(agent_nodes[index]))->add_agent( &all_neighbors[index] );
            }

            void navigation_graph_sensor_t::get_neighbors( unsigned index, std::vector< neighbor_t* >& neighborhood )
            {
                neighborhood = all_neighborhoods[index];
            }
            
            void navigation_graph_sensor_t::update_neighbor_information( unsigned index )
            {
                //Get the state information for this agent
                all_plants[index]->get_state_space()->copy_to_vector( point_vector_memory );
                
                //First compute the velocity based on the difference in position
                double vel_x = (point_vector_memory[0] - prior_position[ 2*index ])/sensor_delay;
                double vel_y = (point_vector_memory[1] - prior_position[ 2*index +1 ])/sensor_delay;
                all_neighbors[index].set_neighbor_velocity( vel_x, vel_y );

                //Overwrite the new state
                all_neighbors[index].set_neighbor_center( point_vector_memory[0], point_vector_memory[1] );
                
                //Then, update the "prior state"
                prior_position[ 2*index ] = point_vector_memory[0];
                prior_position[ 2*index +1 ] = point_vector_memory[1];
                
                //Finally, let's ensure this neighbor is considered to be valid
                all_neighbors[index].set_valid( true );
            }

            void navigation_graph_sensor_t::update_neighborhood( unsigned index )
            {
                //Stack variables
                neighbor_t* elevator_neighbor = NULL;
                std::vector< std::pair< double, neighbor_t* > > agent_neighbors;
                
                //Also make sure we have the agent's state
                // all_plants[index]->get_state_space()->copy_to_point( query_point );
                const util::vector_t query_pos = all_neighbors[index].get_center();

                // PRX_PRINT("Neighborhood computation for agent: " << index << "  :: " << all_plants[index]->get_state_space()->print_point( query_point, 3 ), PRX_TEXT_MAGENTA);
                
                // = ====================================================
                //  1.) First, we need to get the information from the elevator
                // = ====================================================
                elevator_t* e = agent_nodes[index]->near_elevator;
                if( e != NULL )
                {
                    //If the elevator is not open at this height
                    if( !e->is_open() || fabs( e->current_height() - agent_nodes[index]->point->memory[2] ) > 0.4 )
                    {
                        //Then we add it as a neighbor
                        elevator_neighbor = &(e->neighbor);
                        // PRX_PRINT("Elevator: " << elevator_neighbor->get_name(), PRX_TEXT_CYAN);
                    }
                }

                // = ====================================================
                //  2.) Get the information for the static obstacles
                // = ====================================================
                const std::vector< neighbor_t* >& obstacle_neighbors = agent_nodes[index]->obstacle_neighbors;
                // PRX_PRINT("Static obstacle neighbors: " << obstacle_neighbors.size(), PRX_TEXT_BLUE);

                // = ====================================================
                //  3.) Then, get the information for the other agents
                // = ====================================================
                std::vector< const nav_node_t* > triangles;
                triangles.push_back( agent_nodes[index] );
                for( unsigned i=0; i<agent_nodes[index]->neighboring_triangles.size(); ++i )
                {
                    triangles.push_back( agent_nodes[index]->neighboring_triangles[i] );
                }
                
                // PRX_PRINT("Searching through " << triangles.size() << " triangles.", PRX_TEXT_GREEN);
                unsigned limit = max_neighbors - obstacle_neighbors.size() - ( elevator_neighbor == NULL ? 0 : 1 ) - 1;

                if( limit < 1 )
                {
                    PRX_FATAL_S("Neighborhood has become saturated, cannot account for nearby agents!");
                }

                neighbor_t* agent_neigh;
                //With triangles in tow, we can start putting them in the list
                for( unsigned i=0; i<triangles.size(); ++i )
                {
                    // PRX_PRINT("Triangle " << i << " has " << triangles[i]->occupying_agents.size() << " agents:", PRX_TEXT_BROWN);
                    for( unsigned j=0; j<triangles[i]->occupying_agents.size(); ++j )
                    {
                        agent_neigh = triangles[i]->occupying_agents[j];
                        // PRX_PRINT("Neighbor ID: " << agent_neigh->get_agent_index(), PRX_TEXT_LIGHTGRAY);
                        //We will add the information about the current agent_neigh at the end of the list.
                        if( agent_neigh->get_agent_index() != index )
                        {
                            double distance = agent_neigh->distance(query_pos) - 2*agent_radius;
                            
                            if( distance < sensing_radius )
                            {
                                std::pair< double, neighbor_t* > temp( distance, agent_neigh );
                                if( agent_neighbors.size() < limit )
                                {
                                    agent_neighbors.push_back( temp );
                                }
                                else
                                {
                                    std::vector< std::pair< double, neighbor_t* > >::iterator it = std::max_element(agent_neighbors.begin(), agent_neighbors.end());
                                    if (temp < *it)
                                    {
                                        *it = temp;
                                    }
                                }
                            }
                        }
                    }
                    
                    //TODO: Add the neighbors from the other nodes
                    PRX_ASSERT(triangles[i]->triangle_id != -1);
                    int triangle_id = triangles[i]->triangle_id;
                    
                    if(extra_neighbors_per_triangle.size() > 0)
                    {
                        // PRX_PRINT("Done with this node and we might want to add from the other nodes for triangle (" << triangle_id << ") : " << extra_neighbors_per_triangle[triangle_id].size() << "     size of the vector: " << extra_neighbors_per_triangle.size(), PRX_TEXT_GREEN);
                        for(unsigned j=0; j < extra_neighbors_per_triangle[triangle_id].size(); ++j)
                        {
                            agent_neigh = extra_neighbors_per_triangle[triangle_id][j];
                            double distance = agent_neigh->distance(query_pos) - 2*agent_radius;
                                
                            if( distance < sensing_radius )
                            {
                                std::pair< double, neighbor_t* > temp( distance, agent_neigh );
                                if( agent_neighbors.size() < limit )
                                {
                                    agent_neighbors.push_back( temp );
                                }
                                else
                                {
                                    std::vector< std::pair< double, neighbor_t* > >::iterator it = std::max_element(agent_neighbors.begin(), agent_neighbors.end());
                                    if (temp < *it)
                                    {
                                        *it = temp;
                                    }
                                }
                            }
                        }
                    }
                    // PRX_PRINT("------------------------  DONE WITH AGENT " << index, PRX_TEXT_LIGHTGRAY);
                }
                
                // = ====================================================
                //  4.) Save the finalized list
                // = ====================================================
                all_neighborhoods[index].clear();
                
                if( elevator_neighbor != NULL )
                {
                    all_neighborhoods[index].push_back( elevator_neighbor );
                }
                for( unsigned i=0; i<obstacle_neighbors.size(); ++i )
                {
                    all_neighborhoods[index].push_back( obstacle_neighbors[i] );
                }
                for( unsigned i=0; i<agent_neighbors.size(); ++i )
                {
                    all_neighborhoods[index].push_back( agent_neighbors[i].second );
                }
                //We will always put out own information at the end.
                all_neighborhoods[index].push_back( &(all_neighbors[ index ]) );
                
                // PRX_PRINT("Finalized List: ", PRX_TEXT_RED);
                // for( unsigned i=0; i<all_neighborhoods[index].size(); ++i )
                // {
                //     PRX_PRINT("[" << i << "]: " << all_neighborhoods[index][i]->get_name() << "  radius: " << all_neighborhoods[index][i]->get_radius(), PRX_TEXT_LIGHTGRAY);
                // }
                // PRX_PRINT("", PRX_TEXT_RED);
            }

            unsigned navigation_graph_sensor_t::get_max_num_neighbors()
            {
                return max_neighbors;
            }

            void navigation_graph_sensor_t::set_num_of_triangles(int num_of_triangles)
            {
                extra_neighbors_per_triangle.resize(num_of_triangles);
            }            

            void navigation_graph_sensor_t::generate_node_msg(int node_id, prx_simulation::pabt_node_msg& node_msg)
            {
                node_msg.node_id = node_id;
                for(unsigned i = 0; i < all_neighbors.size(); ++i)
                {
                    // PRX_PRINT("Node " << node_id << ") agent " << i << " is active: " << (all_plants[i]->is_active()?"true":"false"), PRX_TEXT_MAGENTA);
                    if(all_plants[i]->is_active())
                    {
                        prx_simulation::pabt_agent_msg msg; 
                        msg.node_id = node_id;
                        msg.agent_id = all_neighbors[i].get_agent_index();
                        msg.triangle_id = all_neighbors[i].get_triangle_id();
                        all_neighbors[i].get_vo_info(msg.x, msg.y, msg.v, msg.w);
                        msg.queue_id = all_agents[i]->get_queue_id();
                        msg.has_luggage = all_agents[i]->has_luggage;
                        node_msg.agents.push_back(msg);
                        // PRX_PRINT("node id: " << msg.node_id << "/" <<  node_id << "  agent_id:" << msg.agent_id << "/" <<  all_neighbors[i].get_agent_index() << "  triangle_id:" << msg.triangle_id << "/" << all_neighbors[i].get_triangle_id() << " (" << msg.x << "," << msg.y << "," << msg.v << "," << msg.w <<")", PRX_TEXT_BLUE);
                        // PRX_PRINT("(x,y,v,w):(" << msg.x << "," << msg.y << "," << msg.v << "," << msg.w <<")", PRX_TEXT_BLUE); 
                    }
                }

                for(unsigned i = 0; i<extra_neighbors_per_triangle.size(); ++i)
                    extra_neighbors_per_triangle[i].clear();
                active_extra_neighbors = 0;
            }

            void navigation_graph_sensor_t::update_node_info(int node_id, const prx_simulation::pabt_node_msg& node_msg)
            {
                // PRX_PRINT("Got updates from node: " << node_msg.node_id, PRX_TEXT_GREEN); 
                foreach(prx_simulation::pabt_agent_msg msg, node_msg.agents)
                {
                    // PRX_PRINT("extra_neighbors size:" << extra_neighbors.size() << "   active:" << active_extra_neighbors, PRX_TEXT_CYAN);
                    if(active_extra_neighbors == extra_neighbors.size())
                    {
                        //Generating 10 extra slots so to avoid pushing back every time.
                        for(unsigned i = 0; i < 10; ++i)
                        {
                            // PRX_PRINT("new neighbor " << i << "/" << extra_neighbors.size(), PRX_TEXT_GREEN);
                            extra_neighbors.push_back(new neighbor_t());
                            extra_neighbors.back()->setup_neighbor(msg.agent_id, int_to_str(msg.node_id)+"_"+int_to_str(msg.agent_id), PRX_CYLINDER, agent_radius, true, false);
                        }
                    }

                    // PRX_PRINT("Node " << node_id << ") Gets new agent from node: " << msg.node_id << "  agent: " << msg.agent_id << "    at triangle: " << msg.triangle_id << " (" << msg.x << "," << msg.y << ")",PRX_TEXT_LIGHTGRAY);
                    extra_neighbors[active_extra_neighbors]->update_data(msg.agent_id, msg.triangle_id, msg.x, msg.y, msg.v, msg.w);
                    extra_neighbors_per_triangle[msg.triangle_id].push_back(extra_neighbors[active_extra_neighbors]);
                    active_extra_neighbors++;
                    
                }
            }
        }
    }
}
