/**
 * @file astar_test_tp.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */


#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/heuristic_search/object_collision_constraints.hpp"

#include "prx/planning/modules/local_planners/bvp_local_planner.hpp"

#include "planning/modules/object_constraints_checker.hpp"
#include "planning/task_planners/astar_test_tp.hpp"

// #include "planning/specifications/astar_test_specification.hpp"

#include <boost/range/adaptor/map.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::astar_test_tp_t ,prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace manipulation
        {

            astar_test_tp_t::astar_test_tp_t()
            {

            }

            astar_test_tp_t::~astar_test_tp_t()
            {

            }

            void astar_test_tp_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                PRX_DEBUG_COLOR("==========================================", PRX_TEXT_RED);
                PRX_DEBUG_COLOR(" Initializing astar_test task planner ...", PRX_TEXT_BROWN);
                PRX_DEBUG_COLOR("==========================================", PRX_TEXT_RED);
                task_planner_t::init(reader,template_reader);
            }

            void astar_test_tp_t::link_world_model(world_model_t * const model)
            {
                task_planner_t::link_world_model(model);
            }

            void astar_test_tp_t::link_query(query_t* new_query)
            {
                task_planner_t::link_query(new_query);
                in_query = static_cast<motion_planning_query_t*>(new_query);
            }

            void astar_test_tp_t::setup()
            {
            }

            //Let's just do everything in the execute()
            bool astar_test_tp_t::execute()
            {
                //Some local variables
                std::vector< std::string > objects;
                
                //Setup the world model to use the appropriate context?
                model->use_context("rigidbody_pc");
                
                //Set up the spaces we are working in
                for( unsigned i=0; i<3; ++i )
                {
                    state_memory.push_back( new double );
                    control_memory.push_back( new double );
                }
                state_space = new space_t("SE2", state_memory);
                control_space = new space_t("SE2", control_memory);
                
                //Need an Object constraints checker
                OCC = new object_constraints_checker_t();
                OCC->link_model( model );
                //Need a local planner
                local_planner_t* local_planner = new bvp_local_planner_t();
                local_planner->link_model( model );
                local_planner->link_state_space( state_space );
                local_planner->link_control_space( control_space );

                //Generate constraint names and figure out how many there are
                OCC->generate_constraint_names();
                OCC->get_constraint_names( objects );
                
                //Need to load up a graph
                load_graph();
                
                //Create an A* module
                constraints_astar_search_t astar;
                double length_multiplier = 1.3;
                astar.set_length_multiplier(length_multiplier);
                //Link the graph and VC to that module
                astar.setup_modules( &graph, state_space, control_space, OCC, local_planner);                
                //Set the A* module to the mode we want
                object_collision_constraints_t* valid_constraints = dynamic_cast< object_collision_constraints_t* >( OCC->alloc_constraint() );

                //Need to push in an index for each of the movable bodies in the world?
                for( unsigned i=0; i<objects.size(); ++i )
                {
                    valid_constraints->constraints.insert( i );
                }
                
                //Should be able to make it store through function call
                astar.set_constraints_after_search( true );
                
                //SEARCH_MODE set up here
                astar.setup_astar_search( UNTRUSTED_MCR, valid_constraints);
                
                //Ask for a solution
                // for( unsigned i=0; i<10; ++i )
                {
                    PRX_PRINT("=====================", PRX_TEXT_BLUE);
                    PRX_PRINT(" Beginning A* search ", PRX_TEXT_CYAN);
                    PRX_PRINT("=====================", PRX_TEXT_BLUE);
                    if( astar.solve( start_vertex, goal_vertex ) )
                    {
                        PRX_PRINT("=====================", PRX_TEXT_GREEN);
                        PRX_PRINT(" A* found a solution ", PRX_TEXT_LIGHTGRAY);
                        PRX_PRINT("=====================", PRX_TEXT_GREEN);

                        //Extract that solution                    
                        std::deque< undirected_vertex_index_t > path_vertices;
                        astar.extract_path( path_vertices );

                        for( unsigned i=0; i<path_vertices.size(); ++i )
                        {
                            PRX_PRINT("[" << i << "] : " << state_space->print_point( graph.get_vertex_as<undirected_node_t>( path_vertices[i] )->point ,3), PRX_TEXT_LIGHTGRAY );
                        }
                        
                        //Print out constraints
                        constraints_t* path_constraints = OCC->alloc_constraint();
                        astar.extract_path_constraints( path_constraints );
                        PRX_PRINT( path_constraints->print(), PRX_TEXT_CYAN );
                    }
                    else
                    {
                        PRX_PRINT("===========================", PRX_TEXT_RED);
                        PRX_PRINT(" A* didn't find a solution ", PRX_TEXT_BROWN);
                        PRX_PRINT("===========================", PRX_TEXT_RED);
                    }
                }
                
                // Should clean up memory
                
                PRX_PRINT("Constraints on the edges after search: ", PRX_TEXT_BROWN);
                for( unsigned i=0; i<eis.size(); ++i )
                {
                    PRX_PRINT("[" << i << "]: " << graph.get_edge_as< undirected_edge_t >( eis[i] )->constraints->print(), PRX_TEXT_LIGHTGRAY );
                }
                
                return true;
            }

            bool astar_test_tp_t::succeeded() const
            {
                return true;
            }

            const util::statistics_t* astar_test_tp_t::get_statistics()
            {
                return NULL;
            }

            void astar_test_tp_t::resolve_query()
            {
            }
            
            void astar_test_tp_t::load_graph()
            {
                //Let's keep around a vector of vertex indices to make edges with
                std::vector< undirected_vertex_index_t > vis;
                
                //Add the nodes
                for( unsigned i=0; i<9; ++i )
                {
                    vis.push_back( graph.add_vertex< undirected_node_t >() );
                    undirected_node_t* new_node = graph.get_vertex_as< undirected_node_t >( vis.back() );
                    new_node->point = state_space->alloc_point();
                    new_node->constraints = OCC->alloc_constraint();
                    //Node needs appropriate location information
                    new_node->point->memory[0] = i%3;
                    new_node->point->memory[1] = i/3;
                }
                
                //Set the start and goal vertices
                start_vertex = vis[0];
                goal_vertex = vis[8];
                
                //Add all the friggin' edges
                eis.push_back( graph.add_edge< undirected_edge_t >( vis[0], vis[1], 1.0 ) );
                eis.push_back( graph.add_edge< undirected_edge_t >( vis[1], vis[2], 1.2 ) );
                eis.push_back( graph.add_edge< undirected_edge_t >( vis[0], vis[3], 1.0 ) );
                eis.push_back( graph.add_edge< undirected_edge_t >( vis[1], vis[4], 1.0 ) );
                eis.push_back( graph.add_edge< undirected_edge_t >( vis[2], vis[5], 1.0 ) );
                eis.push_back( graph.add_edge< undirected_edge_t >( vis[3], vis[4], 1.0 ) );
                eis.push_back( graph.add_edge< undirected_edge_t >( vis[4], vis[5], 1.0 ) );
                eis.push_back( graph.add_edge< undirected_edge_t >( vis[3], vis[6], 1.0 ) );
                eis.push_back( graph.add_edge< undirected_edge_t >( vis[4], vis[7], 1.0 ) );
                eis.push_back( graph.add_edge< undirected_edge_t >( vis[5], vis[8], 1.0 ) );
                eis.push_back( graph.add_edge< undirected_edge_t >( vis[6], vis[7], 1.0 ) );
                eis.push_back( graph.add_edge< undirected_edge_t >( vis[7], vis[8], 1.0 ) );
                
                //Add the constraints to the edges
                for( unsigned i=0; i<eis.size(); ++i )
                {
                    undirected_edge_t* edge = graph.get_edge_as< undirected_edge_t >( eis[i] );
                    edge->constraints = OCC->alloc_constraint();
                }
                
                //If we are pre-loading up some constraints (trusted mode tests)
                if( true )
                {
                    std::vector< unsigned > c;
                    
                    //First, edges only constrained by 1
                    c.push_back( 1 );
                    set_edge_constraints( eis[4], c );
                    set_edge_constraints( eis[9], c );
                    
                    //Edges constrained by both 0 and 1
                    c.push_back( 0 );
                    set_edge_constraints( eis[3], c );
                    set_edge_constraints( eis[5], c );
                    set_edge_constraints( eis[6], c );
                    
                    //Edge constrained by all 0, 1, & 2
                    c.push_back( 2 );
                    set_edge_constraints( eis[8], c );
                    
                    //Edge constrained by only 2
                    c.clear();
                    c.push_back( 2 );
                    set_edge_constraints( eis[7], c );
                    
                    //Edges constrained by both 0 and 2
                    c.push_back( 0 );
                    set_edge_constraints( eis[10], c );
                    set_edge_constraints( eis[11], c );
                    
                    //Edges constrained only by 3
                    c.clear();
                    c.push_back( 3 );
                    set_edge_constraints( eis[0], c );
                    set_edge_constraints( eis[2], c );
                }
                
                PRX_PRINT("Constraints on the edges BEFORE search: ", PRX_TEXT_GREEN);
                for( unsigned i=0; i<eis.size(); ++i )
                {
                    PRX_PRINT("[" << i << "]: " << graph.get_edge_as< undirected_edge_t >( eis[i] )->constraints->print(), PRX_TEXT_LIGHTGRAY );
                }

            }
            
            void astar_test_tp_t::set_edge_constraints( undirected_edge_index_t e, std::vector< unsigned >& input_constraints )
            {
                //Get the actual edge
                undirected_edge_t* edge = graph.get_edge_as< undirected_edge_t >( e );
                
                //Get the cast constraints from the edge
                object_collision_constraints_t* obj_constraints = dynamic_cast< object_collision_constraints_t* >( edge->constraints );
                
                //Then, assign into those constraints our input constraints
                obj_constraints->constraints.insert( input_constraints.begin(), input_constraints.end() );
            }
            
            
        }
    }
}

