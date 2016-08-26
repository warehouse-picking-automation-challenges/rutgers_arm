/**
 * @file prm.cpp
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

#include "prx/planning/motion_planners/prm_star/prm_star.hpp"
#include "prx/planning/motion_planners/prm_star/prm_star_statistics.hpp"

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"

#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/communication/visualization_comm.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/compressed_sparse_row_graph.hpp>


PLUGINLIB_EXPORT_CLASS(prx::plan::prm_star_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace plan
    {

        prm_star_t::prm_star_t()
        {
            random_point = NULL;
            update_k(0);
            statistics = new prm_star_statistics_t();
            num_edges = 0;
            num_vertices = 0;
            num_generated = 0;
            pno_mode = false;
            no_collision_query_type = false;
            astar = NULL;
            update_weights_on_deserialize = false;
            streaming_adjacency_test = false;
        }

        prm_star_t::~prm_star_t()
        {
            state_space->free_point(random_point);
            new_path.clear();
            new_plan.clear();
            if( astar != NULL )
            {
                delete astar;
                astar = NULL;
            }
        }

        void prm_star_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            PRX_DEBUG_COLOR("Initializing PRM motion planner... ", PRX_TEXT_CYAN);
            motion_planner_t::init(reader, template_reader);

            //REMOVED DUE TO ASTAR BEING IN SPECIFICATION
            // if( parameters::has_attribute("heuristic_search", reader, template_reader) )
            // {
            //     astar = parameters::initialize_from_loader< constraints_astar_search_t > ("prx_planning", reader, "heuristic_search", template_reader, "heuristic_search");
            // }
            // else
            // {
            //     PRX_WARN_S("Missing A*, heuristic search algorithm, during initialization of PRM!");
            // }

            visualize_graph = parameters::get_attribute_as<bool>("visualize_graph", reader, template_reader, true);
            visualize_solutions = parameters::get_attribute_as<bool>("visualize_solutions", reader, template_reader, true);
            visualization_graph_name = parameters::get_attribute_as<std::string > ("visualization_graph_name", reader, template_reader, "prm/graph");
            visualization_solutions_name = parameters::get_attribute_as<std::string > ("visualization_solutions_name", reader, template_reader, "prm/solutions");
            graph_color = parameters::get_attribute_as<std::string > ("graph_color", reader, template_reader, "black");
            delta_prm = parameters::get_attribute_as<bool>("delta_prm", reader, template_reader, false);
            pno_mode = parameters::get_attribute_as<bool>("pno_mode", reader, template_reader, false);
            serialize_plan = parameters::get_attribute_as<bool>("serialize_plan", reader, template_reader, false);
            update_weights_on_deserialize = parameters::get_attribute_as<bool>("update_weights_on_deserialize", reader, template_reader, false);
            do_connection_test = parameters::get_attribute_as<bool>("do_connection_test", reader, template_reader, true);

            if( parameters::has_attribute("solutions_colors", reader, template_reader) )
                solutions_colors = parameters::get_attribute_as<std::vector<std::string> >("solutions_colors", reader, template_reader);
            else
                solutions_colors.push_back("white");

            if( parameters::has_attribute("visualization_bodies", reader, template_reader) )
                visualization_bodies = parameters::get_attribute_as<std::vector<std::string> >("visualization_bodies", reader, template_reader);
            else if( parameters::has_attribute("visualization_body", reader, template_reader) )
                visualization_bodies = parameters::get_attribute_as<std::vector<std::string> >("visualization_body", reader, template_reader);
            else
                PRX_WARN_S("Visualization_systems have not been specified for PRM motion planner!");

            similarity_threshold = parameters::get_attribute_as<double>("similarity_threshold", reader, template_reader, PRX_DISTANCE_CHECK);
            execute_num = 0;

        }

        void prm_star_t::reset()
        {
            // PRX_DEBUG_COLOR("PRM* is getting reset, dummy!", PRX_TEXT_RED);
            std::vector<undirected_vertex_index_t> to_delete;

            foreach(undirected_vertex_index_t v, boost::vertices(graph.graph))
            {
                to_delete.push_back(v);
            }

            foreach(undirected_vertex_index_t v, to_delete)
            {

                foreach(undirected_vertex_index_t u, boost::adjacent_vertices(v, graph.graph))
                {
                    undirected_edge_index_t e = boost::edge(v, u, graph.graph).first;
                    graph.get_edge_as<prm_star_edge_t > (e)->clear(control_space);
                }
                graph.clear_and_remove_vertex(v);
            }

            metric->clear();
            new_path.clear();
            update_k(0);
        }

        void prm_star_t::link_specification(specification_t* new_spec)
        {
            motion_planner_t::link_specification(new_spec);

            if( input_specification->astar != NULL )
            {
                if( astar != NULL )
                {
                    PRX_WARN_S("--------------------------------------------------------------------");
                    PRX_WARN_S("PRM's A* module will replaced with the A* module from specification!");
                    PRX_WARN_S("--------------------------------------------------------------------");
                }
                astar = input_specification->astar;
            }
            if( astar == NULL )
                PRX_FATAL_S("No A* module has been specified in motion planner " << path);

        }

        void prm_star_t::setup()
        {
            //DEBUG: for sanity's sake, make sure k is updated before we add any seeds, in the case that there are things already in the graph
            update_k(num_vertices);
            // PRX_DEBUG_COLOR("\n\nPRM* SETUP!\n", PRX_TEXT_RED);
            // PRX_DEBUG_COLOR("Setting up PRM* : " << num_vertices << " (" << k << ")", PRX_TEXT_MAGENTA);

            random_point = state_space->alloc_point();
            new_path.link_space(state_space);
            new_plan.link_control_space(control_space);
            new_plan.link_state_space(state_space);

            std::vector<state_t*>& seeds = input_specification->get_seeds();
            std::vector<bool>& valid_seeds = input_specification->get_validity();
            for( size_t i = 0; i < seeds.size(); ++i )
            {
                if( validity_checker->is_valid(seeds[i]) )
                {
                    PRX_PRINT("GOOD [" << i << "] seed: " << state_space->print_point( seeds[i], 3 ), PRX_TEXT_GREEN);
                    valid_seeds[i] = true;
                    add_node(seeds[i]);
                    update_k(num_vertices);
                }
                else
                {
                    PRX_PRINT("BAD  [" << i << "] seed: " << state_space->print_point( seeds[i], 3 ), PRX_TEXT_RED);
                    valid_seeds[i] = false;
                }
            }

            astar->setup_modules( &graph, state_space, control_space, validity_checker, local_planner );
        }

        bool prm_star_t::execute()
        {
            PRX_ASSERT(input_specification != NULL);

            // This is something that people didn't want to have it in the motion planner 
            // so we had to overwrite the execute function for the roadmap motion planners.
            if(deserialize_flag)
            {
                deserialize();
                throw stopping_criteria_t::stopping_criteria_satisfied("Deserialized the roadmap");
            }
            do
            {
                PRX_STATUS("PRM STAR STEP ["<< execute_num << "]...", PRX_TEXT_BROWN);
                step();
            }
            while( !input_specification->get_stopping_criterion()->satisfied() );
            
            return succeeded();
        }

        void prm_star_t::step()
        {
            valid_random_sample();
            add_node(random_point);
            update_k(num_vertices);
            execute_num++;
        }

        bool prm_star_t::succeeded() const
        {
            PRX_INFO_S("prm_star_t");
            if( input_specification->get_stopping_criterion()->satisfied() )
                return true;
            return false;
        }


        void prm_star_t::link_query(motion_planning_query_t* new_query)
        {
            motion_planner_t::link_query(new_query);
            astar->setup_astar_search(new_query->search_mode, new_query->active_constraints, new_query->restart_astar_search, new_query->lazy_astar_iterations);
        }

        //TODO This function should do what it is supposed to depending upon the modes, possibly?
        void prm_star_t::remove_start_and_goals( undirected_vertex_index_t v_start, std::vector<undirected_vertex_index_t>& v_goals )
        {
            if( remove_start )
            {
                remove_vertex( v_start );
            }

            for( size_t i = 0; i < v_goals.size(); ++i )
            {
                if( remove_goals[i] )
                {
                    remove_vertex( v_goals[i] );
                }
            }
            remove_goals.clear();
        }
        
        void prm_star_t::remove_vertex( undirected_vertex_index_t v )
        {
            foreach(undirected_vertex_index_t u, boost::adjacent_vertices(v, graph.graph))
            {
                undirected_edge_index_t e = boost::edge(v, u, graph.graph).first;
                prm_star_edge_t* edge = graph.get_edge_as< prm_star_edge_t >(e);
                edge->clear(control_space);
            }

            metric->remove_point(graph.get_vertex_as<undirected_node_t > (v));
            graph.clear_and_remove_vertex(v);
            num_vertices--;
        }

        void prm_star_t::trace_vertices( const std::deque< undirected_vertex_index_t >& path_vertices )
        {
            //First, clear out the input query path and plan
            input_query->path.clear();
            input_query->plan.clear();
            //For each pair of vertices in the solution path
            for( size_t i = 0; i < path_vertices.size() - 1; ++i )
            {
                //Get the edge connecting those vertices
                undirected_edge_index_t e = boost::edge(path_vertices[i], path_vertices[i + 1], graph.graph).first;
                //Determine what parts of the information is missing on the edges
                bool path_exists = graph.get_edge_as<prm_star_edge_t > (e)->path.size() > 0;
                bool plan_exists = graph.get_edge_as<prm_star_edge_t > (e)->plan.size() > 0;

                //On each step (except the first one), we remove the last state in the path to prevent duplicate states
                if( i != 0 )
                {
                    input_query->path.resize(input_query->path.size() - 1);
                }
                //If both the plan and the path exist, simply append
                if( path_exists && plan_exists )
                {
                    input_query->path += graph.get_edge_as<prm_star_edge_t > (e)->path;
                    input_query->plan += graph.get_edge_as<prm_star_edge_t > (e)->plan;
                }
                //Otherwise, if we have the plan but not the path, repropagate
                else if( plan_exists && !path_exists )
                {
                    new_path.clear();
                    local_planner->propagate(graph[path_vertices[i]]->point, graph.get_edge_as<prm_star_edge_t > (e)->plan, new_path);
                    input_query->path += new_path;
                    input_query->plan += graph.get_edge_as<prm_star_edge_t > (e)->plan;
                }
                //If we have the trajectory, but not the plan, we need to re-steer to regenerate the plan that creates this trajectory
                else if( path_exists && !plan_exists )
                {
                    new_plan.clear();
                    local_planner->steer(graph[path_vertices[i]]->point, graph[path_vertices[i + 1]]->point, new_plan, random_point);
                    input_query->plan += new_plan;
                    input_query->path += graph.get_edge_as<prm_star_edge_t > (e)->path;
                }
                //Otherwise, we have nothing and have to do it all from scratch
                else
                {
                    new_path.clear();
                    new_plan.clear();
                    local_planner->steer(graph[path_vertices[i]]->point, graph[path_vertices[i + 1]]->point, new_plan, new_path);

                    input_query->plan += new_plan;
                    input_query->path += new_path;
                }
            }
        }

        void prm_star_t::resolve_query()
        {
            sys_clock_t prm_timer;
            prm_timer.reset();
            // PRX_PRINT("PRM* Resolve Query: " << path << " resolving with mode: " << input_query->search_mode, PRX_TEXT_RED);

            //DEBUGGING: Go to random places
            // unsigned goals_size = 1;
            // std::vector< space_point_t* > goals;
            
            // bool goal_found = false;
            // while( !goal_found )
            // {
            //     unsigned goi = uniform_int_random(0, num_vertices-1);
            //     foreach( undirected_vertex_index_t v, boost::vertices(graph.graph) )
            //     {
            //         if( goi == 0 && validity_checker->is_valid( graph[v]->point ) )
            //         {
            //             goals.push_back( graph[v]->point );
            //             goal_found = true;
            //         }
            //         --goi;
            //     }
            // }

            //ORIGINAL CODE, BRING THIS BACK IN
            //First, get all the goals
            unsigned goals_size;
            std::vector< space_point_t* > goals = input_query->get_goal()->get_goal_points( goals_size );
            goals.resize(goals_size);
            std::vector<undirected_vertex_index_t> v_goals;
            
            no_collision_query_type = false;

            PRX_PRINT ("Start state: " << state_space->print_point(input_query->get_start_state(), 3), PRX_TEXT_CYAN);
            PRX_PRINT ("Goal state: " << state_space->print_point(goals[0], 3), PRX_TEXT_CYAN);
            
            //Let's try to add the start.  Begin by assuming it is fine to add it.
            //If we are trusting the graph
            if( input_query->search_mode == STANDARD_SEARCH || input_query->search_mode == TRUSTED_MCR )
            {
                //Have to validity check the start node
                if( !validity_checker->is_valid( input_query->get_start_state() ) )
                {
                    PRX_WARN_S("In PRM* resolve query: start state is not valid!");
                    append_to_stat_file("\n-- -- --FAILED PRM* MOVE: START STATE NOT VALID");
                    input_query->found_solution = false;
                    return;
                }
            }
            
            //Now, let's try adding the goals
            std::vector< unsigned > invalid_goals;
            //For each of the goal points
            for( unsigned i=0; i<goals.size(); ++i )
            {
                //If we are trusting the graph
                if( input_query->search_mode == STANDARD_SEARCH || input_query->search_mode == TRUSTED_MCR )
                {
                    //If the goal is not valid
                    if( !validity_checker->is_valid( goals[i]) )
                    {
                        PRX_WARN_S("In PRM* resolve query: goal state is not valid!");
                        append_to_stat_file("\n-- -- --FAILED PRM* MOVE: GOAL STATE NOT VALID");
                        //Mark it as invalid
                        invalid_goals.push_back( i );
                    }
                }
            }
            
            //Remove the invalid goals from our "goals" vector
            if( invalid_goals.size() > 0 )
            {
                for( int i=invalid_goals.size()-1; i >= 0; --i )
                {
                    goals.erase( goals.begin() + invalid_goals[i] );
                }
            }
            //Then, see if we ended up with any goals
            if( goals.size() == 0 )
            {
                PRX_WARN_S("In PRM* resolve query: none of the goal states are valid!");
                append_to_stat_file("\n-- -- --FAILED PRM* MOVE: GOAL STATES NOT VALID");
                input_query->found_solution = false;
                return;
            }
            prm_timer.reset();
            // //Before we add anything to the graph, we should just try to steer to the goals
            for( unsigned i=0; i<goals.size(); ++i )
            {
                //Try steering between the points
                new_path.clear();
                new_plan.clear();
                local_planner->steer(input_query->get_start_state(), goals[i], new_plan, new_path);
                if( validity_checker->is_valid( new_path ) )
                {
                    //Horay, we did it!
                    input_query->plan = new_plan;
                    input_query->path = new_path;
                    input_query->found_solution = true;
                    return;
                }
            }
            append_to_stat_file("\n=========PRM* CHECK 1: " + boost::lexical_cast<std::string>(prm_timer.measure_reset()));

            
            

            //Adding the goal            
            if( do_connection_test )
            {
                adjacent_components.clear();
            }
            append_to_stat_file("\n=========PRM* CHECK 2: " + boost::lexical_cast<std::string>(prm_timer.measure_reset()));
            undirected_vertex_index_t v_g;
            bool remove_goal;
            
            for( unsigned i=0; i<goals.size(); ++i )
            {
                boost::tie(remove_goal, v_g) = add_node( goals[i] );
                PRX_PRINT("Adding goal:    [" << state_space->print_point( goals[i], 4 ) << "]  remove flag: " << (remove_goal ? "true" : "false"), PRX_TEXT_LIGHTGRAY );
                if( do_connection_test )
                {
                    add_adjacent_components( v_g );
                }
                v_goals.push_back(v_g);
                remove_goals.push_back( remove_goal );
            }
            
            //Now that we know the start and goals are valid, we can add them to the graph
            streaming_adjacency_test = do_connection_test;
            undirected_vertex_index_t v_start;
            boost::tie(remove_start, v_start) = add_node( input_query->get_start_state() );
            PRX_PRINT("Adding start:    [" << state_space->print_point( input_query->get_start_state(), 4 ) << "]  remove flag: " << (remove_start ? "true" : "false") << "  components size: " << adjacent_components[v_start].size(), PRX_TEXT_LIGHTGRAY );
            streaming_adjacency_test = false;
            append_to_stat_file("\n=========PRM* CHECK 3: " + boost::lexical_cast<std::string>(prm_timer.measure_reset()));

            bool connected_goal = true;
            if( do_connection_test )
            {
                if( !remove_start )
                {
                    adjacent_components[v_start].insert( graph.components[v_start] );
                    
                    connected_goal = test_adjacent_connections( v_start );
                }
                else
                {
                    connected_goal = (adjacent_components[ v_start ].size() > 0);
                }
            }

            //OLD TESTS
            // boost::tie(remove_start, v_start) = add_node(input_query->get_start_state());
            
            // if( do_connection_test )
            // {
            //     adjacent_components.clear();
            //     add_adjacent_components( v_start );
            // }

            // //Now, check if the start and goals are in the same connected component (otherwise assume they are connected)
            // bool connected_goal = true;
            
            // //NEW TEST
            // if( do_connection_test )
            // {
            //     connected_goal = test_adjacent_connections( v_start );
            // }
            

            //If nothing ended up being connected
            if( !connected_goal )
            {
                //Then we need to let the caller know that we have no plan.
                PRX_WARN_S("In PRM* resolve query: none of the goal states are connected to the start!");
                append_to_stat_file("\n-- -- --FAILED PRM* MOVE: GOAL STATES ARE NOT CONNECTED");
                input_query->found_solution = false;
                remove_start_and_goals( v_start, v_goals );
                return;                
            }
            append_to_stat_file("\n=========PRM* CHECK 4: " + boost::lexical_cast<std::string>(prm_timer.measure_reset()));

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
                //DEBUG: Print the points of the path
                PRX_DEBUG_COLOR("PATH VERTICES!!!", PRX_TEXT_GREEN);
                for (unsigned i = 0; i < path_vertices.size(); ++i)
                {
                    PRX_DEBUG_COLOR(state_space->print_point(graph[path_vertices[i]]->point, 7), PRX_TEXT_RED);
                }
            }
            else
            {
                PRX_PRINT ("A* failed", PRX_TEXT_RED);
                append_to_stat_file("\n--------A* Failed To Produce a Solution");
            }

            append_to_stat_file("\n=========PRM* CHECK 5: " + boost::lexical_cast<std::string>(prm_timer.measure_reset()));


            //Now, just a little bit of cleanup before we return out.
            remove_start_and_goals( v_start, v_goals );
            v_goals.clear();
            remove_goals.clear();

            //Reset some of our flags for these checks
            no_collision_query_type = false;

            append_to_stat_file("\n=========PRM* CHECK 6: " + boost::lexical_cast<std::string>(prm_timer.measure_reset()));

        }

        void prm_star_t::add_adjacent_components( undirected_vertex_index_t vertex )
        {
            foreach( undirected_vertex_index_t vp, boost::adjacent_vertices(vertex, graph.graph) )
            {
                adjacent_components[ vertex ].insert( graph.components[vp] );
            }
            // PRX_PRINT("Added adjacent components for [" << vertex << "]", PRX_TEXT_LIGHTGRAY);
        }
        
        bool prm_star_t::test_adjacent_connections( undirected_vertex_index_t vertex )
        {
            //Get the start state's connections
            std::set< int >& start_set = adjacent_components[ vertex ];
            
            //First of all, if the start set is empty, then we can't do anything, because the start is not connected to anything
            if( start_set.empty() )
            {
                PRX_PRINT("START is disconnected", PRX_TEXT_RED);
                return false;
            }
            if( adjacent_components.size() < 2 )
            {
                PRX_PRINT("GOALS are all completely disconnected!" << adjacent_components.size(), PRX_TEXT_RED);
                return false;
            }

            //Now, we have to check the sets of all the goals
            foreach( undirected_vertex_index_t goal_v, adjacent_components | boost::adaptors::map_keys )
            {                
                //Ensure we look at the things which aren't the start
                if( vertex != goal_v )
                {
                    //Then, go over each thing in the set
                    foreach( int c, adjacent_components[goal_v] )
                    {
                        //If ever the start set connects to this component
                        if( start_set.count( c ) > 0 )
                        {
                            //We can do it
                            return true;
                        }
                    }
                }
            }
            
            //Nothing worked, report failure
            return false;
        }

        void prm_star_t::update_vis_info() const
        {
            if( visualization_bodies.size() <= 0 || input_query == NULL)
                return;

            std::vector<geometry_info_t> geoms;
            std::vector<config_t> configs;
            hash_t<std::string, std::vector<double> > map_params;
            std::vector<double> params;

            int count;

            if( visualize_graph )
            {

                count = 0;
                std::vector<std::string> system_names;
                system_names.push_back(visualization_bodies[0]);
                

                foreach(undirected_edge_index_t e, boost::edges(graph.graph))
                {
                    std::string name = ros::this_node::getName() + visualization_graph_name + "/edge_" + int_to_str(count);
                    params.clear();

                    //PRX_WARN_S("edge size: " << graph.get_edge_as<prm_star_edge_t > (e)->path.size());

                    map_params.clear();
                    ((comm::visualization_comm_t*)comm::vis_comm)->compute_configs(graph[boost::source(e, graph.graph)]->point, system_names, map_params);
                    params.insert(params.end(), map_params[system_names[0]].begin(), map_params[system_names[0]].end());
                    //                    PRX_INFO_S("source( " << state_space->print_point(graph[boost::source(e, graph.graph)]->point,2) << ")  params size: " << params.size());


                    map_params.clear();
                    ((comm::visualization_comm_t*)comm::vis_comm)->compute_configs(graph[boost::target(e, graph.graph)]->point, system_names, map_params);
                    params.insert(params.end(), map_params[system_names[0]].begin(), map_params[system_names[0]].end());
                    //                    PRX_INFO_S("target( " << state_space->print_point(graph[boost::target(e, graph.graph)]->point,2) << ")  params size: " << params.size());

                    geoms.push_back(geometry_info_t(visualization_bodies[0], name, PRX_LINESTRIP, params, graph_color));
                    configs.push_back(config_t());
                    count++;
                }

                ((comm::visualization_comm_t*)comm::vis_comm)->visualization_geom_map[visualization_graph_name] = geoms;
                ((comm::visualization_comm_t*)comm::vis_comm)->visualization_configs_map[visualization_graph_name] = configs;
                geoms.clear();
                configs.clear();
            }


            if( visualize_solutions && input_query->found_solution && input_query->path.size() > 0 )
            {
                hash_t<std::string, std::vector<double> > to_map_params;
                count = 0;
                for( size_t i = 0; i < input_query->path.size() - 1; i++ )
                {
                    map_params.clear();
                    to_map_params.clear();

                    ((comm::visualization_comm_t*)comm::vis_comm)->compute_configs(input_query->path[i], visualization_bodies, map_params);
                    ((comm::visualization_comm_t*)comm::vis_comm)->compute_configs(input_query->path[i + 1], visualization_bodies, to_map_params);

                    for( size_t s = 0; s < visualization_bodies.size(); s++ )
                    {
                        params.clear();
                        params.insert(params.end(), map_params[visualization_bodies[s]].begin(), map_params[visualization_bodies[s]].end());
                        params.insert(params.end(), to_map_params[visualization_bodies[s]].begin(), to_map_params[visualization_bodies[s]].end());

                        std::string name = ros::this_node::getName() + visualization_solutions_name + "/" + visualization_bodies[s] + "/path_" + int_to_str(count);

                        geoms.push_back(geometry_info_t(visualization_bodies[s], name, PRX_LINESTRIP, params, solutions_colors[s % solutions_colors.size()]));
                        configs.push_back(config_t());
                        count++;
                    }
                }

                ((comm::visualization_comm_t*)comm::vis_comm)->visualization_geom_map[visualization_solutions_name] = geoms;
                ((comm::visualization_comm_t*)comm::vis_comm)->visualization_configs_map[visualization_solutions_name] = configs;
                geoms.clear();
                configs.clear();
            }
        }

        void prm_star_t::valid_random_sample()
        {
            do
            {
                sampler->sample(state_space, random_point);
                ++num_generated;
            }
            while( !(validity_checker->is_valid(random_point)) );
        }

        std::pair<bool, util::undirected_vertex_index_t> prm_star_t::add_node(space_point_t* n_state)
        {
            if( metric->get_nr_points() > 0 )
            {
                const prm_star_node_t* node = metric->single_query(n_state)->as<prm_star_node_t > ();
                //TODO: removed near query type
                if( node != NULL && ( metric->distance_function(n_state, node->point) <= similarity_threshold) )
                {
                    PRX_PRINT ("IT IS ALREADY THERE. KOSTAS", PRX_TEXT_MAGENTA);
                    append_to_stat_file ("<><><><><><><><>IT IS ALREADY THERE. KOSTAS");
                    return std::make_pair(false, node->index);
                }
            }

            v_new = graph.add_vertex<prm_star_node_t > ();
            num_vertices++;
            graph.get_vertex_as< prm_star_node_t > (v_new)->init_node(state_space, n_state, validity_checker->alloc_constraint());

            if( delta_prm )
                connect_node(v_new, r_n);
            else
                connect_node(v_new);

            return std::make_pair(true, v_new);
        }

        void prm_star_t::connect_node(undirected_vertex_index_t v)
        {
            std::vector<const abstract_node_t*> neighbors;
            neighbors = metric->multi_query(graph[v], k);

            link_node_to_neighbors(v, neighbors);

            //add the generated node to the metric module. Add node is not doing it
            //so as the nearest neighbor will not return itself.
            if( metric->has_point(graph[v]) )
            {
                PRX_WARN_S("Metric already has the point! Skipped add");
            }
            else
                metric->add_point(graph[v]);
        }

        void prm_star_t::connect_node(undirected_vertex_index_t v, double rad)
        {
            std::vector< const abstract_node_t* > neighbors;
            neighbors = metric->radius_query(graph[v], rad);

            link_node_to_neighbors(v, neighbors);

            //add the generated node to the metric module. Add node is not doing it
            //so as the nearest neighbor will not return itself.
            if( metric->has_point(graph[v]) )
            {
                PRX_WARN_S("Metric already has the point! Skipped add");
            }
            else
                metric->add_point(graph[v]);
        }

        void prm_star_t::link_node_to_neighbors(undirected_vertex_index_t v, const std::vector< const abstract_node_t* >& neighbors)
        {
            const undirected_node_t* node;

            // PRX_DEBUG_COLOR("Linking node: " << v, PRX_TEXT_GREEN);
            // PRX_DEBUG_COLOR("Linking to neighbors: " << neighbors.size(), PRX_TEXT_RED);

            new_path.clear();
            for( size_t i = 0; i < neighbors.size(); i++ )
            {
                node = neighbors[i]->as< undirected_node_t > ();
                //Get the component
                unsigned component;

                new_plan.clear();

                if( streaming_adjacency_test )
                {
                    //Get the actual component
                    component = graph.components[ node->index ];
                    //Check if it is any of the goal's components
                    bool connected = false;
                    //Now, we have to check the sets of all the goals
                    foreach( undirected_vertex_index_t goal_v, adjacent_components | boost::adaptors::map_keys )
                    {                
                        //Then, go over each thing in the set
                        foreach( int c, adjacent_components[goal_v] )
                        {
                            //If ever the start set connects to this component
                            if( c == component )
                            {
                                //We can do it
                                connected = true;
                            }
                        }
                    }
                    //If it's not connected, move to the next neighbor
                    if( !connected )
                    {
                        continue;
                    }
                }

                local_planner->steer(graph[v]->point, node->point, new_plan, new_path);

                //If the path is valid
                if( new_plan.size() != 0 && is_valid_trajectory(new_path) )
                {
                    
                    //Add the edge to the graph
                    double dist = metric->distance_function(graph[v]->point, node->point);
                    undirected_edge_index_t e = graph.add_edge< prm_star_edge_t > (v, node->index, dist);
                    graph.get_edge_as<prm_star_edge_t >(e)->id = num_edges;
                    graph.get_edge_as<prm_star_edge_t >(e)->constraints = validity_checker->alloc_constraint();
                    num_edges++;
                    if( visualize_graph )
                        graph.get_edge_as< prm_star_edge_t > (e)->path = new_path;
                    //If we're doing the streaming test
                    if( streaming_adjacency_test )
                    {
                        //Need to return
                        adjacent_components[ v ].insert( component );
                        return;
                    }
                }
                

                new_path.clear();
            }
        }

        void prm_star_t::update_k(unsigned nr_nodes)
        {
            if( nr_nodes == 0 )
            {
                k = 0;
                r_n = PRX_ZERO_CHECK;
            }
            else
            {
                double d = state_space->get_dimension();
                double val = (1.0 / d);
                k = PRX_MAXIMUM(std::ceil((log(nr_nodes)*2.7182818284 * (1 + val))), 1);
                if( pno_mode )
                    k *= pow(2.0, d);
                if( delta_prm )
                {
                    r_n = 2 * pow((log(nr_nodes) / (double)nr_nodes)*(1 + val)*(state_space->space_size() / state_space->n_ball_measure(1.0)), val);
                    if( pno_mode )
                    {
                        r_n *= 2;
                        //PRX_DEBUG_COLOR("Radius: " << r_n, PRX_TEXT_GREEN);
                    }
                }
            }
        }

        bool prm_star_t::is_valid_trajectory(const sim::trajectory_t& path)
        {
            if( no_collision_query_type )
                return true;
            return validity_checker->is_valid(path);
        }

        bool prm_star_t::serialize()
        {
            PRX_INFO_S(" Inside PRM serialization now, saving to file: " << serialization_file);
            char* w = std::getenv("PRACSYS_PATH");
            std::string dir(w);
            dir += ("/prx_roadmaps/");
            std::string file = dir + serialization_file;
            PRX_DEBUG_COLOR("PRM* serializing to file: " << file, PRX_TEXT_LIGHTGRAY);
            std::ofstream fout;
            fout.open(file.c_str());
            PRX_ASSERT(fout.is_open());
            graph.serialize(fout, state_space);

            if( serialize_plan )
            {

                foreach(undirected_edge_index_t e, boost::edges(graph.graph))
                {
                    if( graph.get_edge_as<prm_star_edge_t > (e)->plan.size() > 0 )
                    {
                        graph.get_edge_as<prm_star_edge_t > (e)->plan.save_to_stream(fout);
                    }
                    else
                    {
                        local_planner->steer(graph[boost::source(e, graph.graph)]->point, graph[boost::target(e, graph.graph)]->point, new_plan, new_path);
                        new_plan.save_to_stream(fout);
                    }
                }
            }
            fout.close();
            return true;

        }

        bool prm_star_t::deserialize()
        {
            PRX_INFO_S(" Inside PRM deserialization now, opening file: " << deserialization_file);
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

            hash_t<unsigned, undirected_vertex_index_t> node_map;
            hash_t<unsigned, undirected_edge_index_t> edge_map;

            foreach(undirected_edge_index_t e, boost::edges(graph.graph))
            {
                if( update_weights_on_deserialize )
                {
                    double dist = metric->distance_function(graph[boost::source(e, graph.graph)]->point, graph[boost::target(e, graph.graph)]->point);
                    graph.set_weight(e, dist);
                }
                graph.get_edge_as<prm_star_edge_t > (e)->id = counter;
                graph.get_edge_as<prm_star_edge_t > (e)->plan.link_control_space(this->control_space);
                graph.get_edge_as<prm_star_edge_t > (e)->plan.link_state_space(this->state_space);
                counter++;
                //added to make sure that constraint classes are allocated upon deserialize: ZL 2/17
                graph[e]->constraints = validity_checker->alloc_constraint();
                edge_map[graph[e]->edge_id] = e;
            }

            double val_mu = (double)boost::num_edges(graph.graph) / (double)boost::num_vertices(graph.graph);
            double diff;
            double val_dev = 0.0;

            foreach(undirected_vertex_index_t nd, boost::vertices(graph.graph))
            {
                PRX_DEBUG_S("Added to metric: " << state_space->print_point(graph.graph[nd].node->point));
                metric->add_point(graph[nd]);
                PRX_DEBUG_S("Metric now has: " << metric->get_nr_points() << " points");
                diff = (double)boost::out_degree(nd, graph.graph) - val_mu;
                val_dev += diff * diff;
                //added to make sure that constraint classes are allocated upon deserialize: ZL 2/17
                graph[nd]->constraints = validity_checker->alloc_constraint();
                node_map[graph[nd]->node_id] = nd;
            }
            val_dev = sqrt(val_dev);
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

            // PRX_PRINT("Deserialized roadmap with " << boost::num_vertices(graph.graph) << " vertices ... " << boost::num_edges(graph.graph) << " edges.", PRX_TEXT_MAGENTA);
            // PRX_PRINT("Average Valence: " << val_mu << "   Valence Deviation: " << val_dev, PRX_TEXT_GREEN);

            return true;
        }

        const statistics_t* prm_star_t::get_statistics()
        {
            prm_star_statistics_t* stats = new prm_star_statistics_t(); // statistics->as<prm_star_statistics_t > ();
            int num = boost::connected_components(graph.graph, graph.components);
            stats->cc_sizes.resize(num);

            foreach(util::undirected_vertex_index_t v, boost::vertices(graph.graph))
            {
                stats->cc_sizes[graph.components[v]]++;
            }
            stats->num_vertices = boost::num_vertices(graph.graph);
            stats->num_edges = boost::num_edges(graph.graph);
            stats->num_connected_components = num;

            return stats;
        }

    }
}
