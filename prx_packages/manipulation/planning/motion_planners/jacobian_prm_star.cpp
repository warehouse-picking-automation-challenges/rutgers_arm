/**
 * @file jacobian_prm_star.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2016, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Athanasios Krontiris, Andrew Dobson, Andrew Kimmel, Zakary Littlefield, James Marble, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "planning/motion_planners/jacobian_prm_star.hpp"

#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"

#include "planning/queries/pose_based_mp_query.hpp"
#include "planning/specifications/pose_based_mp_specification.hpp"
#include "planning/manipulation_world_model.hpp"

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::jacobian_prm_star_t, prx::plan::planner_t)

namespace prx
{

    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {

        namespace manipulation
        {

            jacobian_prm_star_t::jacobian_prm_star_t()
            {
                _se3_mem.resize( 7, new double() );
                se3 = new space_t("SE3", _se3_mem);
                resolving_query = false;
                prm_star_query = false;
            }
            
            jacobian_prm_star_t::~jacobian_prm_star_t()
            {
                foreach( double* ds, _se3_mem )
                {
                    delete ds;
                }
                delete se3;
            }
            
            void jacobian_prm_star_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                end_effector_names = parameters::get_attribute_as< std::vector< std::string > >( "end_effector_names", reader, template_reader );
                prm_star_t::init( reader, template_reader );
                
                //For each of these end-effectors, we should build a distance metric
                foreach( std::string ee_name, end_effector_names )
                {
                    end_effector_metrics[ee_name] = parameters::initialize_from_loader< distance_metric_t >( "prx_utilities", reader, "map_metric", template_reader, "map_metric" );
                    end_effector_metrics[ee_name]->link_space( se3 );
            //     astar = parameters::initialize_from_loader< constraints_astar_search_t > ("prx_planning", reader, "heuristic_search", template_reader, "heuristic_search");
                }
            }

            void jacobian_prm_star_t::link_specification(specification_t* new_spec)
            {
                prm_star_t::link_specification( new_spec );
                manip_checker = dynamic_cast< manipulation_validity_checker_t* >( validity_checker );
            }

            std::pair<bool, undirected_vertex_index_t> jacobian_prm_star_t::add_node(space_point_t* n_state)
            {
                if( metric->get_nr_points() > 0 )
                {
                    const prm_star_node_t* node = metric->single_query(n_state)->as<prm_star_node_t > ();
                    //TODO: removed near query type
                    if( node != NULL && ( metric->distance_function(n_state, node->point) <= similarity_threshold) )
                    {
                        return std::make_pair(false, node->index);
                    }
                }

                v_new = graph.add_vertex<prm_star_node_t > ();
                num_vertices++;
                graph.get_vertex_as< prm_star_node_t > (v_new)->init_node(state_space, n_state, validity_checker->alloc_constraint());
                
                if( !resolving_query && !prm_star_query )
                {
                    manipulation_world_model_t* manip_model = dynamic_cast< pose_based_mp_specification_t* >( input_specification )->get_manipulation_model();
                    add_links_to_metrics( graph[v_new], manip_model );                
                }

                if( delta_prm )
                    connect_node(v_new, r_n);
                else
                    connect_node(v_new);

                return std::make_pair(true, v_new);
            }

            void jacobian_prm_star_t::resolve_query()
            {
                sys_clock_t timer;
                bool valid_path;

                // PRX_PRINT("Jacobian PRM* Resolve Query: " << path << " resolving with mode: " << input_query->search_mode, PRX_TEXT_RED);
                resolving_query = true;
                prm_star_query = false;
                //We're going to need the manipulation world model (from specification?)
                manipulation_world_model_t* manip_model = dynamic_cast< pose_based_mp_specification_t* >( input_specification )->get_manipulation_model();
                PRX_PRINT("manipWorldModel Context:"<<manip_model->get_current_context(),PRX_TEXT_MAGENTA);
                //Also cast the query so as to be able to access the target poses
                pose_based_mp_query_t* pose_query = dynamic_cast< pose_based_mp_query_t* >( input_query );
                config_t target_config = pose_query->get_target_pose();
                
                timer.reset();
                //THIS CHECK MAGICALLY DISAPPEARED!
                if( !pose_query->is_pose_query() )
                {
                    resolving_query = false;
                    prm_star_query = true;
                    prm_star_t::resolve_query();
                    prm_star_query = false;
                    return;
                }
                append_to_stat_file("\n[][][][][]JKPRM* CHECK 1: " + boost::lexical_cast<std::string>(timer.measure_reset()));
                
                //Let's try to add the start.  Begin by assuming it is fine to add it.
                //If we are trusting the graph
                if( input_query->search_mode == STANDARD_SEARCH || input_query->search_mode == TRUSTED_MCR )
                {
                    //Have to validity check the start node
                    if( !validity_checker->is_valid( input_query->get_start_state() ) )
                    {
                        PRX_WARN_S("In Jacobian PRM* resolve query: start state is not valid [" << input_query->get_start_state()->memory << "]");
						append_to_stat_file("\n-- -- --FAILED MOVE TO CONFIG: START STATE NOT VALID");
                        input_query->found_solution = false;
                        resolving_query = false;
                        return;
                    }
                }


                append_to_stat_file("\n[][][][][]JKPRM* CHECK 2: " + boost::lexical_cast<std::string>(timer.measure_reset()));
                
                //Very first thing to try: Jacobian steer from start to goal
                workspace_trajectory_t ws_trajectory;
                space_point_t* result_state = state_space->alloc_point();
                new_plan.clear();
                new_path.clear();
                if( manip_model->jac_steering( new_plan, ws_trajectory, result_state, input_query->get_start_state(), target_config ) )
                {
                    //If we have a plan, propagate it,
                    manip_model->propagate_plan( input_query->get_start_state(), new_plan, new_path );
                    //And if it is valid
                    if( manip_checker != NULL )
                    {
                        valid_path = manip_checker->is_valid( new_path, ws_trajectory );
                    }
                    else
                    {
                        valid_path = validity_checker->is_valid( new_path );
                    }

                    if( valid_path )
                    {
                        //This is a success, so report such
                        input_query->plan = new_plan;
                        input_query->path = new_path;
                        input_query->found_solution = true;
                        resolving_query = false;
                        return;                        
                    }
                }
                new_plan.clear();
                new_path.clear();


                append_to_stat_file("\n[][][][][]JKPRM* CHECK 3: " + boost::lexical_cast<std::string>(timer.measure_reset()));
                
                //Alright, now for the target pose, we need a set of candidate neighbors, & corresponding configurations
                space_point_t* pose_point = se3->alloc_point();
                conf_to_point( pose_point, target_config );
                
                distance_metric_t* q_metric = end_effector_metrics[ pose_query->get_end_effector() ];
                const std::vector< const abstract_node_t* > neighbors = q_metric->multi_query( pose_point, k );
                
                std::vector< undirected_vertex_index_t > v_goals;

                if( do_connection_test )
                {
                    adjacent_components.clear();
                }

                //Now that we know the start and goals are valid, we can add them to the graph
                undirected_vertex_index_t v_start;
                boost::tie(remove_start, v_start) = add_node(input_query->get_start_state());
                
                if( do_connection_test )
                {
                    add_adjacent_components( v_start );
                }
                
                unsigned total_failures = 0;
                unsigned bad_final_states = 0;
                unsigned jacobian_failures = 0;
                
                //Now, try to steer from each of these points
                append_to_stat_file("Neighbors size: " + boost::lexical_cast<std::string>(neighbors.size()));
                for( unsigned i=0; i<neighbors.size(); ++i )
                {
                    const abstract_node_t* n = neighbors[i];

                    new_plan.clear();
                    new_path.clear();
                    ws_trajectory.clear();

                    const mapping_node_t* casted_node = dynamic_cast< const mapping_node_t* >( n );
                    PRX_ASSERT(casted_node != NULL);
                    undirected_node_t* graph_node = dynamic_cast< undirected_node_t* >( casted_node->get_linked_node() );
                    PRX_ASSERT(graph_node != NULL);
                    space_point_t* robot_state = graph_node->point;

                    //If we're doing connection tests
                    if( do_connection_test )
                    {
                        //If it would not be one of the start components
                        if( adjacent_components[ v_start ].count( graph.components[graph_node->index] ) == 0 )
                        {
                            continue;
                        }
                    }

                    
                    //If we are able to do the steering
                    if( manip_model->jac_steering( new_plan, ws_trajectory, result_state, robot_state, target_config ) )
                    {
                        // PRX_PRINT ("JACOBIAN STEERED GOAL STATE: " << state_space->print_point(result_state,5), PRX_TEXT_GREEN);
                        //Propagate the plan we get
                        manip_model->propagate_plan( robot_state, new_plan, new_path );
                        //And if it is valid
                        if( manip_checker != NULL )
                        {
                            valid_path = manip_checker->is_valid( new_path, ws_trajectory );
                        }
                        else
                        {
                            valid_path = validity_checker->is_valid( new_path );
                        }

                        if( valid_path )
                        {
                            // PRX_PRINT("Good Jacobian path.", PRX_TEXT_BLUE);
                            //Then this is a potential goal
                            v_goals.push_back( graph.add_vertex< prm_star_node_t >() );
                            num_vertices++;
                            graph.get_vertex_as< prm_star_node_t >( v_goals.back() )->init_node( state_space, robot_state, validity_checker->alloc_constraint() );
                            //We also need the edge with the specific plan/trajectory
                            double dist = metric->distance_function( robot_state, result_state );
                            undirected_edge_index_t e = graph.add_edge< prm_star_edge_t > ( graph_node->index, v_goals.back(), dist);
                            graph.get_edge_as<prm_star_edge_t >(e)->id = num_edges;
                            graph.get_edge_as<prm_star_edge_t >(e)->constraints = validity_checker->alloc_constraint();
                            graph.get_edge_as< prm_star_edge_t > (e)->path = new_path;
                            graph.get_edge_as< prm_star_edge_t > (e)->plan = new_plan;
                            remove_goals.push_back( true );
                            add_adjacent_components( v_goals.back() );
                            num_edges++;
                            
                            if( do_connection_test )
                            {
                                break;
                            }
                        }
                        else
                        {
                            // PRX_PRINT("Jacobian path is INVALID", PRX_TEXT_RED);
                            ++total_failures;
                            if( !validity_checker->is_valid( new_path.back() ) )
                            {
                                ++bad_final_states;
                            }
                        }
                    }
                    else
                    {
                        ++jacobian_failures;
                    }
                }
                PRX_PRINT("Bad states: " << bad_final_states << " / " << total_failures, PRX_TEXT_RED);
                PRX_PRINT(" -> Jacobian failures: " << jacobian_failures, PRX_TEXT_LIGHTGRAY);
                
                append_to_stat_file("\n[][][][][]JKPRM* CHECK 4: " + boost::lexical_cast<std::string>(timer.measure_reset()));

                //Now, check if the start and goals are in the same connected component (otherwise assume they are connected)
                bool connected_goal = true;
                
                //Test connectivity
                if( do_connection_test )
                {
                    // connected_goal = test_adjacent_connections( v_start );
                    connected_goal = v_goals.size() > 0;
                }

                append_to_stat_file("\n[][][][][]JKPRM* CHECK 5: "+ boost::lexical_cast<std::string>(timer.measure_reset()));

                //If nothing ended up being connected
                if( !connected_goal )
                {
                    //Then we need to let the caller know that we have no plan.
                    PRX_WARN_S("In Jacobian PRM* resolve query: none of the goal states are connected to the start!");
                    append_to_stat_file("\n-- -- FAILED MOVE TO CONFIG: GOAL STATE NOT CONNECTED");
                    append_to_stat_file("\n-- -- -- FINAL STATES / OVERALL STATES IN COLLISION: " + boost::lexical_cast<std::string>(bad_final_states) + " / " + boost::lexical_cast<std::string>(total_failures));
                    append_to_stat_file("\n-- -- -- JACOBIAN FAILURES: " + boost::lexical_cast<std::string>(jacobian_failures));

                    input_query->found_solution = false;
                    remove_start_and_goals( v_start, v_goals );
                    //Also need to do this specifically for JPRM*
                    metric->remove_point(graph.get_vertex_as<undirected_node_t > (v_start));
                    resolving_query = false;
                    return;                
                }
                // PRX_PRINT("Trying with lazy iterations:: "<<input_query->lazy_astar_iterations, PRX_TEXT_CYAN);
                //Let's get things set up for the A* search to begin
                astar->link_graph(&graph);
                //Set the A* mode to what the input query says to do.
                astar->setup_astar_search( input_query->search_mode, input_query->active_constraints, input_query->restart_astar_search, input_query->lazy_astar_iterations );
                //Then, pass on the memo to the A* on whether he should be updating any constratins he's computing
                astar->set_constraints_after_search( input_query->update_constraints );


                //Search.  Then, if the search finds a path through the graph
                if( astar->solve(v_start, v_goals) )
                {
                    //We need to extract the path vertices
                    std::deque< undirected_vertex_index_t > path_vertices;
                    astar->extract_path( path_vertices );
                    //Let the caller know that we have found a solution
                    input_query->found_solution = true;
                    //Then, trace through the path vertices to construct the actual path/plan
                    trace_vertices( path_vertices );
                    // We must fill the path constraints of the query
                    astar->extract_path_constraints(input_query->path_constraints);
                    //Let's also remember the actual state the planning ends up in
                    pose_query->satisfied_goal_state = state_space->clone_point( graph[ path_vertices.back() ]->point );
                }


                append_to_stat_file("\n[][][][][]JKPRM* CHECK 6: " + boost::lexical_cast<std::string>(timer.measure_reset()));

                //Now, just a little bit of cleanup before we return out.
                remove_start_and_goals( v_start, v_goals );
                //Also need to do this specifically for JPRM*
                metric->remove_point(graph.get_vertex_as<undirected_node_t > (v_start));
                
                v_goals.clear(); //TODO: WHY IS THING BROKEN HERE?
                remove_goals.clear();

                //Reset some of our flags for these checks
                no_collision_query_type = false;
                resolving_query = false;

                append_to_stat_file("\n[][][][][]JKPRM* CHECK 7: "+ boost::lexical_cast<std::string>(timer.measure_reset()));

            }

            bool jacobian_prm_star_t::deserialize()
            {
                PRX_INFO_S(" Inside Jacobian PRM* deserialization now, opening file: " << deserialization_file);
                char* w = std::getenv("PRACSYS_PATH");
                std::string dir(w);
                dir += ("/prx_roadmaps/");
                std::string file = dir + deserialization_file;
                PRX_INFO_S("File directory is: " << file);
                std::ifstream fin;
                if( !graph.deserialize<prm_star_node_t, prm_star_edge_t > (file, fin, state_space) )
                {
                    PRX_FATAL_S("File could not deserialize!");
                    return false;
                }
                int counter = 0;

                manipulation_world_model_t* manip_model = dynamic_cast< pose_based_mp_specification_t* >( input_specification )->get_manipulation_model();

                hash_t<unsigned, undirected_vertex_index_t> node_map;
                hash_t<unsigned, undirected_edge_index_t> edge_map;

                foreach(undirected_edge_index_t e, boost::edges(graph.graph))
                {
                    if( update_weights_on_deserialize )
                    {
                        double dist = metric->distance_function(graph[boost::source(e, graph.graph)]->point, graph[boost::target(e, graph.graph)]->point);
                        graph.set_weight(e, dist);
                    }
                    graph.get_edge_as< prm_star_edge_t >(e)->id = counter;
                    graph.get_edge_as< prm_star_edge_t >(e)->plan.link_control_space(this->control_space);
                    graph.get_edge_as< prm_star_edge_t >(e)->plan.link_state_space(this->state_space);
                    counter++;
                    //added to make sure that constraint classes are allocated upon deserialize: ZL 2/17
                    graph[e]->constraints = validity_checker->alloc_constraint();
                    edge_map[graph[e]->edge_id] = e;
                }


                foreach(undirected_vertex_index_t nd, boost::vertices(graph.graph))
                {
                    metric->add_point(graph[nd]);
                    //added to make sure that constraint classes are allocated upon deserialize: ZL 2/17
                    graph[nd]->constraints = validity_checker->alloc_constraint();
                    node_map[graph[nd]->node_id] = nd;
                    
                    //Make sure the world is in the correct state
                    manip_model->push_state( graph[nd]->point );
                    add_links_to_metrics( graph[nd], manip_model );
                }
                num_vertices = boost::num_vertices(graph.graph);
                num_edges = boost::num_edges(graph.graph);

                //Make sure we have computed at least the basic connected components information
                boost::connected_components(graph.graph, graph.components);

                update_k(num_vertices);

                for (int i = 0; i < num_vertices; ++i)
                {
                    unsigned id;
                    fin>>id;
                    graph[node_map[id]]->constraints->deserialize(fin);
                }

                for (int i = 0; i < num_edges; ++i)
                {
                    unsigned id;
                    fin>>id;
                    graph[edge_map[id]]->constraints->deserialize(fin);
                }
                fin.close();

                return true;
            }


            void jacobian_prm_star_t::add_links_to_metrics( abstract_node_t* node, manipulation_world_model_t* manip_model )
            {
                //To get the config information, we will need to perform FK (should just update the phys configs, since we might have more than one)
                config_list_t config_list;
                unsigned nr_configs = 0;
                config_t ee_config;

                //Create a point for us to use for the distance metric
                space_point_t* pose_point = se3->alloc_point();

                manipulator_t* manipulator = manip_model->get_current_manipulation_info()->manipulator;
                
                manipulator->update_phys_configs( config_list, nr_configs );
                //We need to fill in the distance metrics with things
                foreach( std::string& ee_name, end_effector_names )
                {
                    std::string full_name = manipulator->get_pathname() + "/" + ee_name;
                    bool done = false;
                    //Find the config for this end-effector
                    for( unsigned i=0; i<config_list.size() && !done; ++i )
                    {
                        auto pair = config_list[i];
                        if( pair.first == full_name )
                        {
                            ee_config = pair.second;
                            done = true;
                        }
                    }
                    
                    PRX_ASSERT( done );
                    
                    //Need to convert this configuration into the se3 pose point.
                    conf_to_point( pose_point, ee_config );
                    
                    //Now, this configuration needs to be inserted as a point in the corresponding metric.
                    mapping_nodes.push_back( new mapping_node_t( se3, pose_point, node ) );
                    end_effector_metrics[ ee_name ]->add_point( mapping_nodes.back() );
                }
                
                se3->free_point( pose_point );
            }
            
            void jacobian_prm_star_t::conf_to_point( space_point_t* point, const config_t& conf )
            {
                std::vector< double >& m = point->memory;
                conf.get_position().get( m[0], m[1], m[2] );
                conf.get_orientation().get( m[3], m[4], m[5], m[6] );                
            }

            void jacobian_prm_star_t::remove_vertex( undirected_vertex_index_t v )
            {
                foreach(undirected_vertex_index_t u, boost::adjacent_vertices(v, graph.graph))
                {
                    undirected_edge_index_t e = boost::edge(v, u, graph.graph).first;
                    prm_star_edge_t* edge = graph.get_edge_as< prm_star_edge_t >(e);
                    edge->clear(control_space);
                }

                if( !resolving_query )
                {
                    metric->remove_point(graph.get_vertex_as<undirected_node_t > (v));
                }
                graph.clear_and_remove_vertex(v);
                num_vertices--;
            }

        }
    }
}
