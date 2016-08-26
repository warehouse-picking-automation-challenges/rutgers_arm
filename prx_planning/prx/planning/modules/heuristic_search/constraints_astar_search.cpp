/**
 * @file astar_search.cpp 
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

#include "prx/planning/modules/heuristic_search/constraints_astar_search.hpp"
#include "prx/planning/motion_planners/motion_planner_edge.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"

//DEBUG
#include "prx/utilities/heuristic_search/object_collision_constraints.hpp"

#include <algorithm>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_iterator.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::plan::constraints_astar_search_t, prx::plan::constraints_astar_search_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace plan
    {

        constraints_astar_search_t::constraints_astar_search_t()
        {
            _search_mode = STANDARD_SEARCH;
            new_constraints = NULL;
            valid_constraints = NULL;
            saved_valid_constraints = NULL;
            empty_constraints = NULL;
            goal_node = NULL;
            length_multiplier = 1.3;
            update_constraints = false;
            lazy_iterations = PRX_INFINITY;
            total_solve_time = 0;
        }

        constraints_astar_search_t::~constraints_astar_search_t()
        {
            clear_memory();
        }

        void constraints_astar_search_t::clear_memory()
        {
            if( new_constraints != NULL )
            {
                delete new_constraints;
            }
            
            if( empty_constraints != NULL )
            {
                delete empty_constraints;
            }
        }

        void constraints_astar_search_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
        {
            PRX_DEBUG_COLOR("Initialize Constraints A* search...", PRX_TEXT_GREEN);
            astar_search_t::init(reader,template_reader);
            
            //Default search mode is standard search
            std::string mode_str = parameters::get_attribute("search_mode", reader, template_reader, "standard_search");

            // Set the search mode based on input
            if(mode_str == "standard_search")
                _search_mode = STANDARD_SEARCH;
            else if(mode_str == "lazy_search")
                _search_mode = LAZY_SEARCH;
            else if(mode_str == "trusted_mcr")
                _search_mode = TRUSTED_MCR;
            else if(mode_str == "untrusted_mcr")
                _search_mode = UNTRUSTED_MCR;
            
            PRX_DEBUG_COLOR("Set the Constraint Astar Search mode: " << _search_mode, PRX_TEXT_LIGHTGRAY);
            
            // Used in Bounded Exact MCR cases 
            length_multiplier = parameters::get_attribute_as< double >("length_multiplier", reader, template_reader, 1.3);
        }

        void constraints_astar_search_t::restart()
        {   
            astar_search_t::restart();
            edge_path.clear();
            edge_plan.clear();
        }


        void constraints_astar_search_t::setup_modules(undirected_graph_t *g, const space_t* state_sp, const space_t* control_sp, validity_checker_t* checker, local_planner_t* loc_planner)
        {
            astar_search_t::link_graph(g);
            restart();
            state_space = state_sp;
            control_space = control_sp;
            edge_path.link_space(state_sp);
            edge_plan.link_control_space(control_sp);
            validity_checker = checker;
            local_planner = loc_planner;

            //When we get the validity checker, allocate some constraint classes we will be using.
            clear_memory();
            new_constraints = validity_checker->alloc_constraint();
            empty_constraints = validity_checker->alloc_constraint();
        }

        void constraints_astar_search_t::setup_astar_search(search_mode_t heuristic_search_mode, const constraints_t* input_valid_constraints, bool restart_astar, int lazy_iters)
        {
            if (restart_astar)
                restart();
            _search_mode = heuristic_search_mode;
            valid_constraints = input_valid_constraints;
            lazy_iterations = lazy_iters;
            PRX_DEBUG_COLOR("Setting the new search mode: " << _search_mode, PRX_TEXT_CYAN);
        }

        
        //TODO: This should be using cost functions....??
        //TODO: ??????????????????????????????????????
        double constraints_astar_search_t::get_path_cost() const
        {
            return edge_path.size();
        }

        bool constraints_astar_search_t::recompute_and_validate_edge_info(undirected_vertex_index_t vertex, undirected_vertex_index_t parent, undirected_edge_index_t e)
        {
            // PRX_DEBUG_COLOR("Inside recompute_and_validate_edge_info", PRX_TEXT_BLUE);
            if( local_planner != NULL )
            {
                undirected_edge_t* edge = graph->get_edge_as<undirected_edge_t>(e);
                /** If the information currently on the edge belongs to a different search... **/
                if(edge->search_id != search_id)
                {
                    // PRX_DEBUG_COLOR("recompute_and_validate_edge_info search_id != search_id", PRX_TEXT_CYAN);
                    
                    /** Clear out memory in container variables **/
                    edge_plan.clear();
                    edge_path.clear();

                    /** Compute the trajectory/plan by steering from source_vertex to end_vertex **/
                    local_planner->steer(graph->at(parent)->point, graph->at(vertex)->point, edge_plan, edge_path);

                    /** Update graph edge to be the current search **/
                    graph->get_edge_as<undirected_edge_t>(e)->search_id = search_id;

                    if( validity_checker != NULL )
                    {
                        //PRX_PRINT("Pre-VC constraints: " << graph->get_edge_as<undirected_edge_t>(e)->constraints->print(), PRX_TEXT_LIGHTGRAY);
                        allocate_transient_constraint( edge );
                        if( validity_checker->validate_and_generate_constraints(transient_constraints[edge], edge_path) )
                        {
                           //PRX_PRINT("Post-VC constraints: " << graph->get_edge_as<undirected_edge_t>(e)->constraints->print(), PRX_TEXT_LIGHTGRAY);
                            // PRX_DEBUG_COLOR("Validity checker says good!  Reusing edge info.", PRX_TEXT_GREEN);
                            return reuse_edge_info(e, true);
                        }
                        else
                        {
                            PRX_DEBUG_COLOR("Validity checker says BAD!!!", PRX_TEXT_RED);
                        }
                    } 
                    else
                    {
                        PRX_WARN_S("Validity checker not specified in module_astar.");
                    }
                }
                else
                {
                    return reuse_edge_info(e, true);
                } 
            }
            else
            {
                PRX_WARN_S("Local Planner not specified in module_astar.");
            }
            
            return false;
        }

        bool constraints_astar_search_t::reuse_edge_info(undirected_edge_index_t e, bool transient)
        {
            undirected_edge_t* edge = graph->get_edge_as< undirected_edge_t >(e);
            constraints_t* edge_constraints = (transient ? transient_constraints[edge] :  edge->constraints );

            if( _search_mode == STANDARD_SEARCH || _search_mode == LAZY_SEARCH )
            {
                return !edge_constraints->has_intersection( valid_constraints );
            }
            else
            {
                constrained_astar_node_t* a_node = top_node->as<constrained_astar_node_t>();
                edge_constraints->intersect(new_constraints, valid_constraints);
                new_constraints->merge( a_node->constraints );
                // PRX_DEBUG_COLOR("Final edge info: " << new_constraints->print(), PRX_TEXT_LIGHTGRAY);
                return true;
            }
        }

        bool constraints_astar_search_t::solve(const std::vector<undirected_vertex_index_t>& starts, const std::vector<undirected_vertex_index_t>& goals)
        {
            _solve_time.reset();
            /** If standard, we do not need the bound of the path
                so just call the astar_search_t::solve */
            if(_search_mode == STANDARD_SEARCH )
            {
                return astar_search_t::solve( starts, goals );
            }
            /** IF in LAZY_SEARCH, we must validate the trajectory after each solve **/
            else if (_search_mode == LAZY_SEARCH)
            {
                PRX_DEBUG_COLOR("Finding path for LAZY colllisions till "<<lazy_iterations<<" iterations.", PRX_TEXT_GREEN);
                bool found_path = false;
                //While we still don't have a solution and A* is still giving us new paths to test
                int lazy_iters = lazy_iterations;
                while( !found_path && astar_search_t::solve( starts, goals ) && --lazy_iters>=0 )
                {
                    bool blocked_node = false;
                    //Extract the path
                    std::deque< undirected_vertex_index_t > path_vertices;
                    astar_search_t::extract_path( path_vertices );

                    PRX_DEBUG_COLOR("Extracted a path of size: " << path_vertices.size(), PRX_TEXT_MAGENTA);

                    PRX_DEBUG_COLOR("Checking all the nodes in the path for collision...",PRX_TEXT_CYAN);
                    for( size_t i = 0; i < path_vertices.size(); ++i )
                    {
                        state_t* node_state = graph->at(path_vertices[i])->point;
                        PRX_DEBUG_COLOR("[" << i << "]: " << state_space->print_point(node_state, 5), PRX_TEXT_LIGHTGRAY );
                        if(validity_checker->is_valid(node_state))
                        {
                            // PRX_PRINT("State is valid",PRX_TEXT_GREEN);
                        }
                        else
                        {
                            typedef boost::graph_traits < undirected_graph_type >::adjacency_iterator adjacency_iterator;
                            std::pair<adjacency_iterator, adjacency_iterator> neighbors = boost::adjacent_vertices(path_vertices[i], graph->graph);
                            for(; neighbors.first != neighbors.second; ++neighbors.first)
                            {
                                undirected_edge_index_t e = boost::edge(path_vertices[i], *neighbors.first, graph->graph).first;
                                astar_search_t::block_edge(e);
                            }
                            blocked_node = true;
                            PRX_DEBUG_COLOR("Found non valid node. Blocking outgoing edges...",PRX_TEXT_RED);
                        }
                    }
                    if(blocked_node)
                    {
                        continue;
                    }

                    //Begin by assuming the path is good
                    found_path = true;
                    double total_length = 0.0;
                    // Then, for each pair of vertices in the solution path
                    // Reaching here means that no nodes were blocked
                    for( size_t i = 0; i < path_vertices.size() - 1 ; ++i )
                    {
                        // PRX_PRINT("Checking Path...",PRX_TEXT_CYAN);
                        //Get the edge adjoining them
                        undirected_edge_index_t e = boost::edge(path_vertices[i], path_vertices[i + 1], graph->graph).first;
                        //Extract the plan and path for this edge
                        edge_path.clear();
                        edge_plan.clear();
                        local_planner->steer(graph->at(path_vertices[i])->point, graph->at(path_vertices[i + 1])->point, edge_plan, edge_path);
                        total_length += edge_path.length();
                        //Remember if this edge is trusted
                        bool edge_is_trusted = astar_search_t::is_edge_trusted(e);
                        bool edge_is_blocked = astar_search_t::is_edge_blocked(e);

                        //PRX_DEBUG_COLOR("Trusted edge? " << (edge_is_trusted ? "YES":"NO"), PRX_TEXT_CYAN);
                        // If our edge is blocked, then we have no path
                        if (edge_is_blocked)
                        {
                            // PRX_DEBUG_COLOR("BLOCKED edge for path vertices:  " << path_vertices[i] << " - " << path_vertices[i+1], PRX_TEXT_MAGENTA);
                            found_path = false;
                            break;
                        }
                        // If our edge is not trusted..
                        else if( !edge_is_trusted )
                        {
                            undirected_edge_t* edge = graph->get_edge_as<undirected_edge_t>(e);
                            allocate_transient_constraint( edge );
                            bool edge_is_valid = validity_checker->validate_and_generate_constraints(transient_constraints[edge], edge_path);
                            //PRX_DEBUG_COLOR("Valid edge? " << (edge_is_valid ? "YES":"NO"), PRX_TEXT_CYAN);
                            bool edge_is_constrained = !reuse_edge_info(e, true);
                            //PRX_DEBUG_COLOR("Constrained edge? " << (edge_is_constrained ? "YES":"NO"), PRX_TEXT_CYAN);
                            // If invalid edge or constraints on edge
                            if (!edge_is_valid || edge_is_constrained)
                            {
                                // PRX_PRINT ("Found bad edge for path vertex: " << path_vertices[i] << " - " << path_vertices[i+1], PRX_TEXT_RED);
                                //We're going to block the use of this edge for future searches
                                astar_search_t::block_edge(e);
                                //And now, we still haven't found our path
                                found_path = false;
                                break;
                            }
                            //Otherwise, it if the edge is trusted or the validity checking just checked out
                            else
                            {
                                // PRX_DEBUG_COLOR("Found good edge for path vertices: "  << path_vertices[i] << " - " << path_vertices[i+1], PRX_TEXT_GREEN );
                                //We're going to mark this edge as having been validated so future searches don't have to worry
                                if( !edge_is_trusted )
                                {
                                    astar_search_t::trust_edge(e);
                                }
                            }
                        }
                        else
                        {
                             //PRX_DEBUG_COLOR("TRUSTED edge for path vertices: "  << path_vertices[i] << " - " << path_vertices[i+1], PRX_TEXT_BROWN );
                        }
                    }

                    PRX_DEBUG_COLOR ("+_+_+_+_+ TOTAL PATH LENGTH: " << total_length, PRX_TEXT_RED);
                }
                
                total_solve_time+=_solve_time.measure();
                PRX_DEBUG_COLOR("Total Time Taken by ASTAR : "<<total_solve_time,PRX_TEXT_BLUE);
                free_transient_constraints();
                return found_path;
            }
            else if (_search_mode == TRUSTED_MCR || _search_mode == UNTRUSTED_MCR )
            {
                // If we are in MCR mode, we must compute bound first
                // PRX_PRINT("============================", PRX_TEXT_BLUE);
                PRX_PRINT(" Minimum Conflict A* solve.", PRX_TEXT_CYAN);
                // PRX_PRINT("============================", PRX_TEXT_BLUE);
                
                // Save the previous search mode
                search_mode_t prior_mode = _search_mode;
                //We have to first do a search while ignoring obstacles
                _search_mode = (_search_mode == TRUSTED_MCR ? STANDARD_SEARCH : LAZY_SEARCH );
                
                // Disable constraints first to get the "shortest path"
                disable_valid_constraints();
                
                // Find path length by recursive call to solve, this time with search_mode set to LAZY_SEARCH
                PRX_PRINT(" - Finding shortest path: ", PRX_TEXT_CYAN);
                // PRX_PRINT("===========================", PRX_TEXT_BLUE);
                bool have_shortest_path = solve( starts, goals );

                // PRX_PRINT("Did we get a shortest path: " << (have_shortest_path ? "YEP" : "nope..."), PRX_TEXT_BROWN );

                //We must revert the mode
                _search_mode = prior_mode;
                
                // Enable the constraints
                reenable_valid_constraints();
                
                restart();
                
                //If we do indeed have a shortest path
                if( have_shortest_path )
                {
                    //store whatever our old length limiter was
                    double old_length_limit = bounded_length;
                    
                    //Setup the new bounded length
                    bounded_length = length_multiplier * graph->distances[found_goal];
                    // PRX_PRINT("Graph distances at found goal: " << graph->distances[found_goal], PRX_TEXT_CYAN); 
                    // PRX_PRINT("Length multiplier: " << length_multiplier, PRX_TEXT_CYAN);

                    // PRX_PRINT("Old length limit: " << old_length_limit << ", Bounded Length Limit: " << bounded_length, PRX_TEXT_LIGHTGRAY);
                    
                    PRX_PRINT(" - Finding MCR path with bound: " << bounded_length, PRX_TEXT_CYAN);
                    // PRX_PRINT("=======================================", PRX_TEXT_BLUE);
                    
                    //Then, do the EXACT mode solve
                    restart();
                    bool exact_solution_found = astar_search_t::solve( starts, goals );

                    // PRX_PRINT("Then, it tries to find an exact solution.", PRX_TEXT_CYAN);
                    // PRX_PRINT("Found? " << (exact_solution_found ? "YEP" : "nope..."), PRX_TEXT_LIGHTGRAY);
                    
                    //Restore the old length limit
                    bounded_length = old_length_limit;
                    
                    free_transient_constraints();
                    return exact_solution_found;
                }
                //If we couldn't even get a shortest path
                else
                {
                    //We must report failure
                    free_transient_constraints();
                    return false;
                }
            }
            else
            {
                PRX_FATAL_S ("Invalid search mode");
            }
        }

        bool constraints_astar_search_t::solve(const util::undirected_vertex_index_t& start, const std::vector<util::undirected_vertex_index_t>& goals )
        {
            return astar_search_t::solve( start, goals );
        }
        bool constraints_astar_search_t::solve(const util::undirected_vertex_index_t& start, util::undirected_vertex_index_t& goal )
        {
            return astar_search_t::solve( start, goal );
        }

        void constraints_astar_search_t::set_length_multiplier( double new_multiplier )
        {
            length_multiplier = new_multiplier;
        }

        void constraints_astar_search_t::set_constraints_after_search( bool flag )
        {
            update_constraints = flag;
        }

        
        void constraints_astar_search_t::disable_valid_constraints()
        {
            //PRX_PRINT("DISABLE VCON: Valid con is: " << valid_constraints->print(), PRX_TEXT_LIGHTGRAY);
            saved_valid_constraints = valid_constraints;
            //PRX_PRINT("DISABLE VCON: Saved valid con is: " << saved_valid_constraints->print(), PRX_TEXT_LIGHTGRAY);
            valid_constraints = empty_constraints;
            //PRX_PRINT("DISABLE VCON: Valid EMPTY con is: " << valid_constraints->print(), PRX_TEXT_LIGHTGRAY);
        }
        
        void constraints_astar_search_t::reenable_valid_constraints()
        {
            if (saved_valid_constraints != NULL)
            {
                valid_constraints = saved_valid_constraints;
                // PRX_DEBUG_COLOR("REENABLE VCON: Valid con is: " << valid_constraints->print(), PRX_TEXT_BROWN);
            }
            else
            {
                PRX_WARN_S ("Reenable valid constraints has been called with NULL in saved_valid_constraints.  You should call disable_valid_constraints first");
            }
        }

        undirected_vertex_index_t constraints_astar_search_t::extract_path(std::deque<undirected_vertex_index_t>& vertices)
        {
            if( _search_mode == STANDARD_SEARCH || _search_mode == LAZY_SEARCH )
            {
                return astar_search_t::extract_path( vertices );
            }
            else
            {
                vertices = goal_node->path;
                return found_start;
            }
        }

        void constraints_astar_search_t::extract_path_constraints(constraints_t* constraints)
        {
            if( constraints != NULL )
            {
                *constraints = *(goal_node->constraints);
            }
        }

        astar_node_t* constraints_astar_search_t::generate_new_node(undirected_vertex_index_t vertex, undirected_vertex_index_t parent, double g, double h)
        {
            constrained_astar_node_t* node = new constrained_astar_node_t(vertex, g, h);
            node->constraints = validity_checker->alloc_constraint();

            if( top_node != NULL )
            {
                node->merge( dynamic_cast< constrained_astar_node_t* >(top_node), g, h );
            }
            
            node->path.push_back(vertex);
            node->add_constraints( new_constraints );
            //PRX_PRINT("Generating new node, merging in " << new_constraints->print(), PRX_TEXT_LIGHTGRAY);
            return node;
        }

        void constraints_astar_search_t::allocate_transient_constraint( undirected_edge_t* edge )
        {
            //if we don't already have this transient constraint allocated
            if( transient_constraints.find( edge ) == transient_constraints.end() )
            {
                //Allocate it
                transient_constraints[edge] = validity_checker->alloc_constraint();
                // PRX_DEBUG_COLOR("Allocating a transient constraint!", PRX_TEXT_GREEN);
            }
        }
        
        void constraints_astar_search_t::free_transient_constraints()
        {
            foreach( abstract_edge_t* edge, transient_constraints | boost::adaptors::map_keys )
            {
                if( update_constraints )
                {
                    *edge->constraints = *transient_constraints[edge];
                }
                validity_checker->free_constraint( transient_constraints[edge] );
            }
            transient_constraints.clear();
        }

        void constraints_astar_search_t::initialize_vertex(undirected_vertex_index_t vertex, undirected_vertex_index_t parent, double g, double h)
        {   
            astar_search_t::initialize_vertex(vertex, parent, g, h);
            undirected_node_t* node = graph->get_vertex_as<undirected_node_t>(vertex);
            node->constraints->clear();
            //Special case handling here: start node
            if( vertex == parent )
            {
                //We actually need to figure out what the starting constraints are
                validity_checker->validate_and_generate_constraints( node->constraints, node->point );
                if( _search_mode == UNTRUSTED_MCR || _search_mode == TRUSTED_MCR )
                {
                    //And add that to the constraint sets
                    node->constraints->add_to_constraint_sets( node->constraints, 0 );
                }
            }
        }


        bool constraints_astar_search_t::examine_vertex(undirected_vertex_index_t vertex)
        {
            constrained_astar_node_t* heap_node = top_node->as<constrained_astar_node_t>();
            
            bool should_expand = mcr_node_addition_check( vertex, heap_node->constraints, heap_node->g );
            // PRX_DEBUG_COLOR("======================================================================================", PRX_TEXT_BLUE);
            // PRX_DEBUG_COLOR("  Examining node [" << heap_node << "]  v[" << state_space->print_point( graph->get_vertex_as< undirected_node_t >( heap_node->vertex )->point, 1 ) << "]  g[" << heap_node->g << "]  : expanding? " << ( should_expand ? "YES" : "NO" ) << "  " << heap_node->constraints->print(), PRX_TEXT_GREEN);
            // PRX_DEBUG_COLOR("======================================================================================", PRX_TEXT_BLUE);
            
            return should_expand;
        }


        bool constraints_astar_search_t::discover_vertex(undirected_vertex_index_t vertex, undirected_vertex_index_t parent, double new_distance, double h) 
        {
            if( _search_mode == STANDARD_SEARCH || _search_mode == LAZY_SEARCH )
            {
                return astar_search_t::discover_vertex(vertex,parent,new_distance,h);
            }
                        
            //If we have never visited this vertex on this search
            undirected_node_t* node = graph->get_vertex_as<undirected_node_t>(vertex);
            if( node->search_id != search_id )
            {
                //Initialize it, add it to the open set, and return true.
                node->search_id = search_id;
                initialize_vertex(vertex, parent, new_distance, h);
                // PRX_DEBUG_COLOR("Adding Heap node (new search id) v[" << state_space->print_point( node->point, 1 ) << "] p[" << parent << "] d[" << new_distance << "] h[" << h << "]", PRX_TEXT_MAGENTA);
                astar_node_t* heap_node = openset_insert_node(vertex, parent, new_distance, h);

                //First time we see this graph vertex, so it needs its constraint sets to get the heap_node's constraints/distance
                node->constraints->add_to_constraint_sets( dynamic_cast< constrained_astar_node_t* >( heap_node )->constraints, new_distance );
                
                return true;
            }
            
            if(mcr_node_addition_check( vertex, new_constraints, new_distance ))
            {
                // PRX_DEBUG_COLOR("Adding Heap node (MCR addition) v[" << state_space->print_point( node->point, 1 ) << "] p[" << parent << "] d[" << new_distance << "] h[" << h << "]", PRX_TEXT_CYAN);
                openset_insert_node(vertex, parent, new_distance, h);
                return true;
            }           
            else
            {
                // PRX_DEBUG_COLOR("Not adding heap node! v[" << state_space->print_point( node->point, 1 ) << "] p[" << parent << "] d[" << new_distance << "] h[" << h << "]", PRX_TEXT_RED);
                return false;
            }
        }
        

        bool constraints_astar_search_t::mcr_node_addition_check( undirected_vertex_index_t graph_vertex, constraints_t* heap_constraints, double heap_distance )
        {
            //PRX_PRINT ("MCR NODE ADDITION CHECK...", PRX_TEXT_LIGHTGRAY);
            if( _search_mode == TRUSTED_MCR || _search_mode == UNTRUSTED_MCR )
            {
                constraints_t* graph_constraints = graph->get_vertex_as<undirected_node_t>(graph_vertex)->constraints;

                // bool heap_node_has_constraints = heap_constraints->has_intersection(valid_constraints);
                
                // //This needs to be reasoning over the constraint_sets ?????
                // bool graph_has_constraints = graph_constraints->has_sets_intersection(valid_constraints);

                // PRX_PRINT("In MCR mode, have valid constraints: " << valid_constraints->print(), PRX_TEXT_CYAN);
                // if(!graph_has_constraints)
                // {
                //     PRX_PRINT ("Graph has no constraints! " << graph_constraints->print(), PRX_TEXT_RED);
                //     if(heap_node_has_constraints)
                //     {
                //         PRX_PRINT ("Heap has constraints. " << heap_constraints->print(), PRX_TEXT_BLUE);
                //         return false;
                //     }
                //     PRX_PRINT ("Comparing graph dist: " << graph->distances[graph_vertex] << " vs heap : " << heap_distance, PRX_TEXT_CYAN);
                //     //Here both new_constraints and the node's constraints are 0
                //     if( fabs(graph->distances[graph_vertex] - heap_distance) > PRX_ZERO_CHECK && graph->distances[graph_vertex] < heap_distance )
                //         return false;

                //     PRX_PRINT ("VALID MCR NODE ADDITION CHECK!", PRX_TEXT_BROWN);
                // }
                // else if(!heap_node_has_constraints)
                // {
                //     PRX_PRINT ("Heap has no constraints!", PRX_TEXT_GREEN);
                //     //In this case the node on the graph has constraints and the new node does not have.
                //     graph_constraints->clear(); //????????????????????????????????????????????
                //     new_constraints->clear();
                // }
                // else
                // {
                //     PRX_PRINT ("Both graph and Heap node have constraints!", PRX_TEXT_BROWN);
                //     PRX_ASSERT(heap_node_has_constraints && graph_has_constraints);
                return graph_constraints->exact_constraint_comparison(heap_constraints, heap_distance);
                // }
            }
            return true;            
        }

        bool constraints_astar_search_t::examine_edge(undirected_vertex_index_t vertex, undirected_vertex_index_t parent, undirected_edge_index_t e, double new_distance)
        {
            // PRX_DEBUG_COLOR(" - Examining edge for neighbor: " << state_space->print_point( graph->get_vertex_as< undirected_node_t >( vertex )->point , 1 ) 
            //     << "   current info: " << graph->get_edge_as< undirected_edge_t >(e)->constraints->print() , PRX_TEXT_BLUE );
            new_constraints->clear();
            
            //TODO: We should probably have a setup where we can consider blocked edges for MCR search modes
            bool good_edge = true;
            
            //If we are in a non-MCR mode, use the base-class checks for examine edge
            if( _search_mode == STANDARD_SEARCH || _search_mode == LAZY_SEARCH )
            {
                good_edge = astar_search_t::examine_edge(vertex,parent,e,new_distance);
            }
            
            //If we are in Lazy mode, we do not want to reuse info, so simply return the base class examine
            if( _search_mode == LAZY_SEARCH )
            {
                return good_edge;
            }
            
            if( good_edge )
            {
                constrained_astar_node_t* a_node = top_node->as<constrained_astar_node_t>();
                PRX_ASSERT(a_node->vertex == parent);
                // PRX_DEBUG_COLOR(v, PRX_TEXT_BROWN);
                
                if( _search_mode == UNTRUSTED_MCR || _search_mode == TRUSTED_MCR )
                {
                    if (!a_node->path_has_vertex(vertex))
                    {
                        //PRX_DEBUG_COLOR(vertex, PRX_TEXT_GREEN);
                        if( _search_mode == UNTRUSTED_MCR )
                        {
                            return recompute_and_validate_edge_info(vertex,parent,e);
                        }
                        else
                        {
                            return reuse_edge_info(e);
                        }
                    }
                    // else
                    // {
                    //     PRX_DEBUG_COLOR("Path ALREADY HAS the vertex!", PRX_TEXT_RED);
                    // }
                }
                else
                {
                    return reuse_edge_info(e);
                }
            }
            return false;
        }

        //TODO: Why would we need a separate behavior (goal_node pointer?);
        void constraints_astar_search_t::finalize_search(undirected_vertex_index_t vertex)
        {
            if( _search_mode == TRUSTED_MCR || _search_mode == UNTRUSTED_MCR )
            {
                found_goal = vertex;
                goal_node = (constrained_astar_node_t*)top_node;
                found_start = goal_node->path[0];
            }
            else
            {
                astar_search_t::finalize_search( vertex );
                goal_node = (constrained_astar_node_t*)top_node;
            }
        }
            
        astar_node_t* constraints_astar_search_t::openset_insert_node(undirected_vertex_index_t vertex, undirected_vertex_index_t parent, double g, double h)
        {
            if( _search_mode == TRUSTED_MCR || _search_mode == UNTRUSTED_MCR )
            {
                astar_node_t* node = generate_new_node(vertex, parent, g, h);
                open_set.insert(node);
                return node;
            }
            else
            {
                return astar_search_t::openset_insert_node( vertex, parent, g, h );
            }
        }

        //TODO: ????????????????????????
        double constraints_astar_search_t::get_node_distance(astar_node_t* node)
        {
            return node->as<constrained_astar_node_t>()->g;
        }

        pluginlib::ClassLoader<constraints_astar_search_t> constraints_astar_search_t::loader("prx_planning", "prx::plan::constraints_astar_search_t");

        pluginlib::ClassLoader<constraints_astar_search_t>& constraints_astar_search_t::get_loader()
        {
            return loader;
        }

    }
}
