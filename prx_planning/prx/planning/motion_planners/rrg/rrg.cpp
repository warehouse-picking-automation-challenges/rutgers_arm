
#include "prx/planning/motion_planners/rrg/rrg.hpp"
#include "prx/planning/motion_planners/rrg/rrg_statistics.hpp"

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
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
#include <boost/graph/graph_traits.hpp>
#include <boost/config.hpp>

PLUGINLIB_EXPORT_CLASS(prx::plan::rrg_t, prx::plan::planner_t)
namespace fs = boost::filesystem;
namespace prx
{
    using namespace util;
    using namespace sim;

    namespace plan
    {

        rrg_t::rrg_t()
        {
            PRX_DEBUG_COLOR("RRG: Constructing Object", PRX_TEXT_BLUE);
            random_point = NULL;
            update_k(0);
            statistics = new rrg_statistics_t();
            num_edges = 0;
            num_vertices = 0;
            num_generated = 0;
            pno_mode = false;
            no_collision_query_type = false;
            astar = NULL;
            miniumum_connections = 0;
        }

        rrg_t::~rrg_t()
        {
            PRX_DEBUG_COLOR("RRG: ~Object", PRX_TEXT_BLUE);
            state_space->free_point(random_point);
            new_path.clear();
            new_plan.clear();
            if( astar != NULL )
            {
                delete astar;
                astar = NULL;
            }
        }

        void rrg_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            PRX_DEBUG_COLOR("RRG: Initializing Motion Planner... ", PRX_TEXT_BLUE);
            motion_planner_t::init(reader, template_reader);

            if( parameters::has_attribute("heuristic_search", reader, template_reader) )
            {
                astar = parameters::initialize_from_loader< constraints_astar_search_t > ("prx_planning", reader, "heuristic_search", template_reader, "heuristic_search");
            }
            else
            {
                PRX_WARN_S("Missing A*, heuristic search algorithm, during initialization of RRG!");
            }
            using_rrg_sampler = parameters::get_attribute_as<int>("using_rrg_sampler", reader, template_reader, false);
            number_of_nodes = parameters::get_attribute_as<int>("number_of_nodes", reader, template_reader, 100);
            visualize_graph = parameters::get_attribute_as<bool>("visualize_graph", reader, template_reader, true);
            visualize_solutions = parameters::get_attribute_as<bool>("visualize_solutions", reader, template_reader, true);
            visualization_graph_name = parameters::get_attribute_as<std::string > ("visualization_graph_name", reader, template_reader, "rrg/graph");
            visualization_solutions_name = parameters::get_attribute_as<std::string > ("visualization_solutions_name", reader, template_reader, "rrg/solutions");
            graph_color = parameters::get_attribute_as<std::string > ("graph_color", reader, template_reader, "black");
            pno_mode = parameters::get_attribute_as<bool>("pno_mode", reader, template_reader, false);
            serialize_plan = parameters::get_attribute_as<bool>("serialize_plan", reader, template_reader, false);

            if( parameters::has_attribute("solutions_colors", reader, template_reader) )
                solutions_colors = parameters::get_attribute_as<std::vector<std::string> >("solutions_colors", reader, template_reader);
            else
                solutions_colors.push_back("white");

            if( parameters::has_attribute("visualization_bodies", reader, template_reader) )
                visualization_bodies = parameters::get_attribute_as<std::vector<std::string> >("visualization_bodies", reader, template_reader);
            else if( parameters::has_attribute("visualization_body", reader, template_reader) )
                visualization_bodies = parameters::get_attribute_as<std::vector<std::string> >("visualization_body", reader, template_reader);
            else
                PRX_WARN_S("Visualization_systems have not been specified for RRG motion planner!");

            similarity_threshold = parameters::get_attribute_as<double>("similarity_threshold", reader, template_reader, PRX_DISTANCE_CHECK);
            execute_num = 0;
        }

        void rrg_t::reset()
        {
            PRX_DEBUG_COLOR("RRG: Reset", PRX_TEXT_BLUE);
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
                    graph.get_edge_as<rrg_edge_t > (e)->clear(control_space);
                }
                graph.clear_and_remove_vertex(v);
            }

            metric->clear();
            new_path.clear();
            update_k(0);
        }

        void rrg_t::link_specification(specification_t* new_spec)
        {
            PRX_DEBUG_COLOR("RRG: Linking Specification...", PRX_TEXT_BLUE);
            motion_planner_t::link_specification(new_spec);

            if( input_specification->astar != NULL )
            {
                if( astar != NULL )
                {
                    PRX_WARN_S("RRG's A* module will be replaced with the A* module from specification!");
                    delete astar;
                }
                astar = input_specification->astar;
            }
            if( astar == NULL )
                PRX_FATAL_S("No A* module has been specified in motion planner " << path);

        }

        void rrg_t::setup()
        {
            PRX_DEBUG_COLOR("RRG: Setting Up...", PRX_TEXT_BLUE);
            //DEBUG: for sanity's sake, make sure k is updated before we add any seeds, in the case that there are things already in the graph
            update_k(num_vertices);
            // PRX_DEBUG_COLOR("Setting up RRG : " << num_vertices << " (" << k << ")", PRX_TEXT_BLUE);

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
                    PRX_DEBUG_COLOR("GOOD [" << i << "] seed", PRX_TEXT_GREEN);
                    valid_seeds[i] = true;
                    add_node(seeds[i], false);
                    update_k(num_vertices);
                }
                else
                {
                    PRX_DEBUG_COLOR("BAD  [" << i << "] seed", PRX_TEXT_RED);
                    valid_seeds[i] = false;
                }
            }

            astar->setup_modules( &graph, state_space, control_space, validity_checker, local_planner );
        }

        bool rrg_t::execute()
        {
            // PRX_DEBUG_COLOR("RRG: Executing...", PRX_TEXT_BLUE);
            PRX_ASSERT(input_specification != NULL);

            // This is something that people didn't want to have it in the motion planner 
            // so we had to overwrite the execute function for the roadmap motion planners.
            if(deserialize_folder_flag)
            {
                PRX_PRINT("RRG: Deserializing Folder!\n", PRX_TEXT_BLUE);
                // PRX_INFO_S(" Inside RRG deserialization now, opening file: " << deserialization_file_1);
                char* w = std::getenv("PRACSYS_PATH");
                std::string dir(w);
                dir += ("/prx_roadmaps/");
                dir += deserialization_folder;
                fs::path p(dir);
                fs::directory_iterator end_iter;
                roadmaps_bounds.push_back(0);
                for (fs::directory_iterator dir_itr(p); dir_itr != end_iter; ++dir_itr)
                {
                    if (fs::is_regular_file(dir_itr->status()))
                    {
                        PRX_PRINT( "Deserializing File: "<< dir_itr->path().string(), PRX_TEXT_LIGHTGRAY);
                        
                        //Actually perform deserialization
                        deserialization_files.push_back(dir_itr->path().string());
                        deserialize(dir_itr->path().string());
                        
                        roadmaps_bounds.push_back(num_vertices);
                    }
                }
                component_connections.resize( roadmaps_bounds.size()-1 );
                
                
                //DEBUG: Let's try to gather some states
                // foreach( undirected_vertex_index_t v, boost::vertices(graph.graph) )
                // {
                //     if( validity_checker->is_valid( graph[v]->point ) )
                //     {
                //         PRX_PRINT("[" << state_space->print_point( graph[v]->point, 6 ) << "]", PRX_TEXT_LIGHTGRAY);
                //     }
                // }
                // PRX_FATAL_S("NO U");
                

                //Fill in the distance metric
                metric->clear();
                foreach(undirected_vertex_index_t nd, boost::vertices(graph.graph))
                {
                    // PRX_PRINT("Adding to metric: [" << metric->get_nr_points() << "]  " << state_space->print_point(graph.graph[nd].node->point, 3), PRX_TEXT_BROWN);
                    metric->add_point(graph[nd]);
                    // PRX_PRINT("Metric now has: " << metric->get_nr_points() << " points", PRX_TEXT_GREEN);
                    graph[nd]->constraints = validity_checker->alloc_constraint();
                }

                //Finished deserializing all the files inside the folder

                // NEW (naive) Method assuming nodes are concurrent
                if( component_connections.size() > 1 )
                {
                    //A list of all the pairs of nodes which are the same
                    std::vector< std::pair< undirected_vertex_index_t, undirected_vertex_index_t > > concurrent_nodes;
                    std::vector< const abstract_node_t* > neighbors;
                    bool is_first = true;                    

                    //Go over every vertex in the graph
                    foreach( undirected_vertex_index_t v, boost::vertices( graph.graph ) )
                    {
                        //Do a 2-neighbor query: one of those should be itself
                        neighbors = metric->multi_query( graph[v]->point, 2 );
                        undirected_vertex_index_t vp = neighbors[0]->as< undirected_node_t >()->index;
                        is_first = ( v == vp );
                        if( is_first )
                        {
                            vp = neighbors[1]->as< undirected_node_t >()->index;
                        }
                        bool consider_pair = v < vp;
                        
                        if( consider_pair )
                        {
                            //If the other one is zero-distance
                            if( metric->distance_function( graph[v]->point, graph[vp]->point ) < PRX_DISTANCE_CHECK )
                            {
                                //Create the pair 
                                concurrent_nodes.push_back( std::make_pair( v, vp ) );
                            }
                        }
                    }
                    
                    PRX_PRINT("Deserialized graphs have [" << concurrent_nodes.size() << "] overlapping nodes.  Attempting stitching...", PRX_TEXT_RED);
                    
                    //Go over each vertex pair which have the same state
                    foreach( auto p, concurrent_nodes )
                    {
                        //For each edge adjacent to the second
                        foreach( undirected_vertex_index_t vn, boost::adjacent_vertices( p.second, graph.graph ) )
                        {
                            //Add a corresponding edge to the first
                            double dist = metric->distance_function( graph[p.first]->point, graph[vn]->point );
                            undirected_edge_index_t e = graph.add_edge< rrg_edge_t >( p.first, vn, dist );
                            graph.get_edge_as< rrg_edge_t >(e)->constraints = validity_checker->alloc_constraint();
                            if( visualize_graph )
                            {
                                graph.get_edge_as< rrg_edge_t >(e)->path = dynamic_cast< rrg_edge_t* >( graph[ boost::edge( p.second, vn, graph.graph ).first ] )->path;
                            }
                        }
                        //Then, clear and remove second
                        graph.clear_and_remove_vertex( p.second );
                    }
                    
                    unsigned counter = 0;
                    // //DEBUG: what does the graph look like now?
                    // PRX_PRINT("Before fixing", PRX_TEXT_LIGHTGRAY);
                    // foreach( undirected_vertex_index_t v, boost::vertices(graph.graph) )
                    // {
                    //     PRX_PRINT("n[" << counter++ << "] (" << v << ") <" << graph[v]->node_id << ">: " << state_space->print_point( graph[v]->point, 3 ), PRX_TEXT_GREEN );
                    // }
                    // counter = 0;
                    // foreach( undirected_edge_index_t e, boost::edges(graph.graph) )
                    // {
                    //     rrg_edge_t* edge = dynamic_cast< rrg_edge_t* >(graph[e]);
                    //     PRX_PRINT("e[" << counter++ << "] (" << e << ") <" << boost::source(e, graph.graph) << " -> " << boost::target(e, graph.graph) << "> {" << edge->source_vertex << " -> " << edge->target_vertex << "}", PRX_TEXT_BLUE );
                    // }
                    
                    if( concurrent_nodes.size() > 0 )
                    {
                        //Fix up the metric...
                        metric->clear();
                        foreach(undirected_vertex_index_t nd, boost::vertices(graph.graph))
                        {
                            // PRX_PRINT("Added to metric: " << state_space->print_point(graph.graph[nd].node->point, 3), PRX_TEXT_BROWN);
                            metric->add_point(graph[nd]);
                            // PRX_PRINT("Metric now has: " << metric->get_nr_points() << " points", PRX_TEXT_BROWN);
                            graph[nd]->constraints = validity_checker->alloc_constraint();
                        }

                        //Fix up the indices and stuff
                        counter = 0;
                        //Now, our indices are completely screwed, so go over every vertex
                        foreach( undirected_vertex_index_t v, boost::vertices( graph.graph ) )
                        {
                            //Assign it a new id
                            dynamic_cast< rrg_node_t* >( graph[v] )->node_id = counter++;
                        }
                        
                        counter = 0;
                        //Then, go over every edge
                        foreach( undirected_edge_index_t e, boost::edges( graph.graph ) )
                        {
                            //Get the edge
                            rrg_edge_t* edge = dynamic_cast< rrg_edge_t* >( graph[e] );
                            //Get the source and target vertcies
                            undirected_vertex_index_t s = boost::source( e, graph.graph );
                            undirected_vertex_index_t t = boost::target( e, graph.graph );
                            //Re-assign the edge source/target ids based on the updated ids
                            edge->source_vertex = graph[s]->node_id;
                            edge->target_vertex = graph[t]->node_id;
                        }
                    }

                    counter = 0;
                    // //DEBUG: what does the graph look like now?
                    // PRX_PRINT("After fixing", PRX_TEXT_LIGHTGRAY);
                    // foreach( undirected_vertex_index_t v, boost::vertices(graph.graph) )
                    // {
                    //     PRX_PRINT("n[" << counter++ << "] (" << v << ") <" << graph[v]->node_id << ">: " << state_space->print_point( graph[v]->point, 3 ), PRX_TEXT_GREEN );
                    // }
                    // counter = 0;
                    // foreach( undirected_edge_index_t e, boost::edges(graph.graph) )
                    // {
                    //     rrg_edge_t* edge = dynamic_cast< rrg_edge_t* >(graph[e]);
                    //     PRX_PRINT("e[" << counter++ << "] (" << e << ") <" << boost::source(e, graph.graph) << " -> " << boost::target(e, graph.graph) << "> {" << edge->source_vertex << " -> " << edge->target_vertex << "}", PRX_TEXT_BLUE );
                    // }

                }

                //Weird Chuples stitching method: we should only bother if there is more than one connected component anyway...
                // if( component_connections.size() > 1 )
                // {
                //     //For some specified number of attempts (probably should use the stopping criterion)
                //     do
                //     {
                //         //Generate a valid random sample
                //         valid_random_sample();
                //         //Find all of its candidate neighbors, and see if they are in different components
                //         std::vector< const abstract_node_t* > candidate_neighbors;
                //         if( discover_candidate_neighbors( random_point, candidate_neighbors ) )
                //         {
                //             //Assuming they do, then validity check the edges
                //             std::vector< bool > valid_edges( candidate_neighbors.size(), false );
                //             //If edges survived which connect to multiple components
                //             if( validity_check_candidate_edges( random_point, candidate_neighbors, valid_edges ) )
                //             {
                //                 //Actually add the node
                //                 v_new = graph.add_vertex< rrg_node_t > ();
                //                 num_vertices++;
                //                 graph.get_vertex_as< rrg_node_t > (v_new)->init_node( state_space, random_point, validity_checker->alloc_constraint());

                //                 //Add the collision-free edges
                //                 for( unsigned i=0; i<candidate_neighbors.size(); ++i )
                //                 {
                //                     if( valid_edges[i] )
                //                     {
                //                         undirected_edge_index_t e = graph.add_edge< rrg_edge_t >( v_new, dynamic_cast< const rrg_node_t* >( candidate_neighbors[i] )->index, metric->distance_function( random_point, candidate_neighbors[i]->point ) );
                //                         graph.get_edge_as<rrg_edge_t >(e)->id = num_edges;
                //                         graph.get_edge_as<rrg_edge_t >(e)->constraints = validity_checker->alloc_constraint();
                //                         num_edges++;
                //                     }
                //                 }

                //                 PRX_STATUS("RRG stitching [" << execute_num << ":" << num_generated << "]...", PRX_TEXT_BROWN);
                //                 ++execute_num;
                //             }
                //         }
                //     }
                //     while( !input_specification->get_stopping_criterion()->satisfied() );
                // }
                
                //OLD CODE

                // int comp_num = print_components();
                // component_connections.assign(comp_num+1,0);
                // if(comp_num>1 && connect_flag)
                // {
                //     for(int i=0; i<roadmaps_bounds.size(); i++)
                //     {
                //         PRX_PRINT("Bound ["<<i<<"] = "<<roadmaps_bounds.at(i), PRX_TEXT_CYAN);
                //     }
                //     miniumum_connections = 20;
                //     PRX_PRINT("Trying to Connect all components... ", PRX_TEXT_RED);
                //     int tries = 0;
                //     int bridge_points=0;
                //     while(!minimum_connections_satisfied() & tries < 2000)
                //     {                    
                //         tries++;
                //         valid_random_sample();
                //         bool node_added = false;
                //         undirected_vertex_index_t added_vertex;
                //         boost::tie(node_added, added_vertex) = add_node(random_point, false);
                //         update_k(num_vertices);
                //         if(node_added && connects_components(added_vertex))
                //         {
                //             bridge_points++;
                //         }
                //         // PRX_STATUS("Trying Node["<<tries<<"] -> Bridge Nodes Found["<<bridge_points<<"]",PRX_TEXT_LIGHTGRAY);
                //     }
                //     // PRX_PRINT("",PRX_TEXT_CYAN);
                // }
                
                PRX_PRINT("Finished trying to connect things together.", PRX_TEXT_RED);
                // print_components();
                
                if(!expand_flag)
                {
                    throw stopping_criteria_t::stopping_criteria_satisfied("Finished deserializing all the roadmaps.");
                    return true;
                }
                else
                {
                    update_k(number_of_nodes);
                    number_of_nodes += num_vertices;
                    throw stopping_criteria_t::stopping_criteria_satisfied("Finished deserializing all the roadmaps.");
                }
                
            }
            if(deserialize_flag)
            {
                PRX_DEBUG_COLOR("RRG: Deserializing...", PRX_TEXT_BLUE);
                PRX_INFO_S(" Inside RRG deserialization now, opening file: " << deserialization_file);
                char* w = std::getenv("PRACSYS_PATH");
                std::string dir(w);
                dir += ("/prx_roadmaps/");
                std::string file = dir + deserialization_file;
                PRX_INFO_S("File directory is: " << file);
                bool suc = deserialize(file);
                
                //Fill in the distance metric
                foreach(undirected_vertex_index_t nd, boost::vertices(graph.graph))
                {
                    PRX_PRINT("Added to metric: " << state_space->print_point(graph.graph[nd].node->point, 3), PRX_TEXT_BROWN);
                    metric->add_point(graph[nd]);
                    PRX_PRINT("Metric now has: " << metric->get_nr_points() << " points", PRX_TEXT_BROWN);
                    graph[nd]->constraints = validity_checker->alloc_constraint();
                }

                print_components();
                if(!expand_flag)
                {
                    return suc; 
                }
                else
                {
                    update_k(number_of_nodes);
                    number_of_nodes += num_vertices;
                }
                throw stopping_criteria_t::stopping_criteria_satisfied("Finished deserializing the roadmap.");
            }
            PRX_PRINT("num_vertices = " << num_vertices << "   number_of_nodes = " << number_of_nodes, PRX_TEXT_CYAN);
            do
            {
                PRX_STATUS("RRG EXECUTE ["<< num_vertices << " : " << num_generated << "]...", PRX_TEXT_BLUE);
                step();
                if(succeeded()){print_components();}
            }
            while( !input_specification->get_stopping_criterion()->satisfied() );
            return succeeded();
        }

        void rrg_t::step()
        {
            // PRX_DEBUG_COLOR("RRG: Step", PRX_TEXT_BLUE);
            bool success = false;
            undirected_vertex_index_t added_vertex;

            do
            {
                valid_random_sample();
                boost::tie(success, added_vertex) = add_node(random_point, false);
                if(success)
                {
                    update_k(num_vertices);
                    execute_num++;
                }
            }
            while(!success);
        }

        bool rrg_t::succeeded() const
        {
            // PRX_DEBUG_COLOR("RRG: Succeeded", PRX_TEXT_BLUE);
            return (number_of_nodes == num_vertices);
            // if( input_specification->get_stopping_criterion()->satisfied() )
            //     return true;
            // return false;
        }


        void rrg_t::link_query(query_t* new_query)
        {
            PRX_PRINT("RRG: Linking Query...", PRX_TEXT_BLUE);
            motion_planning_query_t* cast_query = dynamic_cast< motion_planning_query_t* >( new_query );
            motion_planner_t::link_query(cast_query);
            astar->setup_astar_search(cast_query->search_mode, cast_query->active_constraints, cast_query->restart_astar_search, cast_query->lazy_astar_iterations);
            
            //Have to add the start state as the seed
            add_node( cast_query->get_start_state(), false );
            PRX_PRINT("Adding the state: " << state_space->print_point(cast_query->get_start_state(), 4) << " graph has: " << boost::num_vertices( graph.graph ), PRX_TEXT_LIGHTGRAY );
        }

        //TODO This function should do what it is supposed to depending upon the modes, possibly?
        void rrg_t::remove_start_and_goals( undirected_vertex_index_t v_start, std::vector<undirected_vertex_index_t>& v_goals )
        {
            PRX_DEBUG_COLOR("RRG: Removing start and goal nodes...", PRX_TEXT_BLUE);
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
        
        void rrg_t::remove_vertex( undirected_vertex_index_t v )
        {
            PRX_DEBUG_COLOR("RRG: Removing Vertex...", PRX_TEXT_BLUE);
            foreach(undirected_vertex_index_t u, boost::adjacent_vertices(v, graph.graph))
            {
                undirected_edge_index_t e = boost::edge(v, u, graph.graph).first;
                graph.get_edge_as< rrg_edge_t > (e)->clear(control_space);
                e = boost::edge(u, v, graph.graph).first;
                graph.get_edge_as< rrg_edge_t > (e)->clear(control_space);
            }

            metric->remove_point(graph.get_vertex_as<undirected_node_t > (v));
            graph.clear_and_remove_vertex(v);
            num_vertices--;
        }

        void rrg_t::trace_vertices( const std::deque< undirected_vertex_index_t >& path_vertices )
        {
            PRX_DEBUG_COLOR("RRG: Tracing vertices...", PRX_TEXT_BLUE);
            //First, clear out the input query path and plan
            input_query->path.clear();
            input_query->plan.clear();
            //For each pair of vertices in the solution path
            for( size_t i = 0; i < path_vertices.size() - 1; ++i )
            {
                //Get the edge connecting those vertices
                undirected_edge_index_t e = boost::edge(path_vertices[i], path_vertices[i + 1], graph.graph).first;
                //Determine what parts of the information is missing on the edges
                bool path_exists = graph.get_edge_as<rrg_edge_t > (e)->path.size() > 0;
                bool plan_exists = graph.get_edge_as<rrg_edge_t > (e)->plan.size() > 0;

                //On each step (except the first one), we remove the last state in the path to prevent duplicate states
                if( i != 0 )
                {
                    input_query->path.resize(input_query->path.size() - 1);
                }
                //If both the plan and the path exist, simply append
                if( path_exists && plan_exists )
                {
                    input_query->path += graph.get_edge_as<rrg_edge_t > (e)->path;
                    input_query->plan += graph.get_edge_as<rrg_edge_t > (e)->plan;
                }
                //Otherwise, if we have the plan but not the path, repropagate
                else if( plan_exists && !path_exists )
                {
                    new_path.clear();
                    local_planner->propagate(graph[path_vertices[i]]->point, graph.get_edge_as<rrg_edge_t > (e)->plan, new_path);
                    input_query->path += new_path;
                    input_query->plan += graph.get_edge_as<rrg_edge_t > (e)->plan;
                }
                //If we have the trajectory, but not the plan, we need to re-steer to regenerate the plan that creates this trajectory
                else if( path_exists && !plan_exists )
                {
                    new_plan.clear();
                    local_planner->steer(graph[path_vertices[i]]->point, graph[path_vertices[i + 1]]->point, new_plan, random_point);
                    input_query->plan += new_plan;
                    input_query->path += graph.get_edge_as<rrg_edge_t > (e)->path;
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

        void rrg_t::resolve_query()
        {
            PRX_DEBUG_COLOR("RRG: Resolving query...", PRX_TEXT_BLUE);
            PRX_DEBUG_COLOR("RRG Resolve Query: " << path << " resolving with mode: " << input_query->search_mode, PRX_TEXT_BLUE);

            //First, get all the goals
            unsigned goals_size;
            std::vector< space_point_t* > goals = input_query->get_goal()->get_goal_points( goals_size );
            goals.resize(goals_size);
            std::vector<undirected_vertex_index_t> v_goals;

            PRX_WARN_S ("1");
            
            no_collision_query_type = false;
            
            //Let's try to add the start.  Begin by assuming it is fine to add it.
            //If we are trusting the graph
            if( input_query->search_mode == STANDARD_SEARCH || input_query->search_mode == TRUSTED_MCR )
            {
                //Have to validity check the start node
                if( !validity_checker->is_valid( input_query->get_start_state() ) )
                {
                    PRX_WARN_S("In RRG resolve query: start state is not valid!");
                    input_query->found_solution = false;
                    return;
                }
            }
            PRX_WARN_S ("2");
                        
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
                        PRX_WARN_S("In RRG resolve query: goal state is not valid!");
                        //Mark it as invalid
                        invalid_goals.push_back( i );
                    }
                }
            }
            PRX_WARN_S ("3");
            //Remove the invalid goals from our "goals" vector
            if( invalid_goals.size() > 0 )
            {
                for( int i=invalid_goals.size()-1; i >= 0; --i )
                {
                    goals.erase( goals.begin() + invalid_goals[i] );
                }
            }
            PRX_WARN_S ("4");
            //Then, see if we ended up with any goals
            if( goals.size() == 0 )
            {
                PRX_WARN_S("In RRG resolve query: none of the goal states are valid!");
                input_query->found_solution = false;
                return;
            }
            PRX_WARN_S ("5");
            
            //Now that we know the start and goals are valid, we can add them to the graph
            undirected_vertex_index_t v_start;
            boost::tie(remove_start, v_start) = add_node(input_query->get_start_state(), true);
            if(remove_start == true) 
            {
                PRX_PRINT("Succesfully connected start!",PRX_TEXT_GREEN);
            }
            else
            {
                PRX_PRINT("Could not connect start!",PRX_TEXT_RED);
            }
            undirected_vertex_index_t v_g;
            bool remove_goal;
            
            for( unsigned i=0; i<goals.size(); ++i )
            {
                boost::tie(remove_goal, v_g) = add_node( goals[i] , true);
                if(remove_start == true) 
                {
                    PRX_PRINT("Succesfully connected goal!",PRX_TEXT_GREEN);
                }
                else
                {
                    PRX_PRINT("Could not connect goal!",PRX_TEXT_RED);
                }
                v_goals.push_back(v_g);
                remove_goals.push_back( remove_goal );
            }

            //Now, check if the start and goals are in the same connected component
            bool connected_goal = false;
            print_components();

            for( unsigned i = 0; i < v_goals.size(); ++i )
            {
                //As long as one of the goals is in the same component
                if( graph.components[v_start] == graph.components[v_goals[i]] )
                {
                    //We will do the search
                    connected_goal = true;
                    break;
                }
            }
            //If nothing ended up being connected
            if( !connected_goal )
            {
                //Then we need to let the caller know that we have no plan.
                PRX_WARN_S("In RRG resolve query: none of the goal states are connected to the start!");
                input_query->found_solution = false;
                remove_start_and_goals( v_start, v_goals );
                return;                
            }

            //Let's get things set up for the A* search to begin
            astar->link_graph(&graph);
            //Set the A* mode to what the input query says to do.
            astar->setup_astar_search( input_query->search_mode, input_query->active_constraints, input_query->restart_astar_search, input_query->lazy_astar_iterations  );
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
            }

            //Now, just a little bit of cleanup before we return out.
            remove_start_and_goals( v_start, v_goals );
            v_goals.clear();
            remove_goals.clear();

            //Reset some of our flags for these checks
            no_collision_query_type = false;
        }


        void rrg_t::update_vis_info() const
        {
            PRX_DEBUG_COLOR("RRG: Updating Visualization Info...", PRX_TEXT_BLUE);
            if( visualization_bodies.size() <= 0 )
                return;

            std::vector<geometry_info_t> geoms;
            std::vector<config_t> configs;
            hash_t<std::string, std::vector<double> > map_params;
            std::vector<double> params;

            int count;

            std::vector< std::string > colors;
            colors.push_back( "red" );
            colors.push_back( "green" );
            colors.push_back( "white" );
            colors.push_back( "pink" );
            colors.push_back( "indigo" );
            colors.push_back( "dark_red" );
            colors.push_back( "viridian" );
            colors.push_back( "orange" );

            if( visualize_graph )
            {

                count = 0;
                std::vector<std::string> system_names;
                system_names.push_back(visualization_bodies[0]);

                std::vector< double > dims;
                dims.push_back(0.002); dims.push_back(0.002); dims.push_back(0.002);
                //Visualize the nodes separately (note: since this is RRG, we shouldn't see anything floating)
                foreach( undirected_vertex_index_t v, boost::vertices( graph.graph ) )
                {
                    std::string name = ros::this_node::getName() + visualization_graph_name + "/node_" + int_to_str(count);
                    params.clear();
                    map_params.clear();
                    
                    ((comm::visualization_comm_t*)comm::vis_comm)->compute_configs(graph[v]->point, system_names, map_params);
                    params.insert(params.end(), map_params[system_names[0]].begin(), map_params[system_names[0]].end());                    
                    
                    geoms.push_back( geometry_info_t( visualization_bodies[0], name, PRX_BOX, dims, (state_space->equal_points( graph[v]->point, input_query->get_start_state(), 0.001 ) ? "blue" : colors[graph.components[v] % colors.size() ]) ) );
                    
                    config_t conf;
                    conf.set_position( params[0], params[1], params[2] );
                    configs.push_back(conf);
                    
                    ++count;
                }
                
                count = 0;
                //Visualize the edges
                foreach(undirected_edge_index_t e, boost::edges(graph.graph))
                {
                    std::string name = ros::this_node::getName() + visualization_graph_name + "/edge_" + int_to_str(count);
                    params.clear();

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


            if( visualize_solutions && input_query->path.size() > 0 )
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

        void rrg_t::valid_random_sample()
        {
            // PRX_DEBUG_COLOR("RRG: Generating a random sample...", PRX_TEXT_BLUE);
            std::vector<prx::util::bounds_t*> dummy_v;
            prx::util::space_point_t* dummy_sp;
            do
            {
                num_vertices == 0 && using_rrg_sampler ? sampler->sample_near(state_space,dummy_sp, dummy_v, random_point) : sampler->sample(state_space, random_point);
                ++num_generated;
            }
            while( !(validity_checker->is_valid(random_point)) );
        }

        std::pair<bool, util::undirected_vertex_index_t> rrg_t::add_node(space_point_t* n_state, bool is_resolving)
        {
            // PRX_DEBUG_COLOR("Adding a Node...", PRX_TEXT_CYAN);
            new_path.clear();
            new_plan.clear();
            if( metric->get_nr_points() > 0 )
            {
                //Find Nearest node
                const rrg_node_t* node = metric->single_query(n_state)->as<rrg_node_t > ();
                // PRX_DEBUG_COLOR("Found Nearest Node...", PRX_TEXT_CYAN);
                //steer to that direction
                if(!is_resolving)
                {
                    local_planner->steer(node->point, n_state, new_plan, new_path, false);
                    n_state = new_path.back();
                }
                if( node != NULL && ( metric->distance_function(n_state, node->point) <= similarity_threshold) )
                {
                    return std::make_pair(false, node->index);
                }
                else
                {
                    if(!is_resolving)
                    {
                        if( new_plan.size() != 0 && is_valid_trajectory(new_path) )
                        {
                            //Add the node AND the edge to the graph
                            v_new = graph.add_vertex<rrg_node_t > ();
                            num_vertices++;
                            graph.get_vertex_as< rrg_node_t > (v_new)->init_node(state_space, n_state, validity_checker->alloc_constraint());
                            double dist = metric->distance_function(n_state, node->point);
                            undirected_edge_index_t e = graph.add_edge< rrg_edge_t > (v_new, node->index, dist);
                            graph.get_edge_as<rrg_edge_t >(e)->id = num_edges;
                            graph.get_edge_as<rrg_edge_t >(e)->constraints = validity_checker->alloc_constraint();
                            num_edges++;
                            if( visualize_graph )
                            {
                                graph.get_edge_as< rrg_edge_t > (e)->path = new_path;
                            }
                            connect_node(v_new, is_resolving);
                            return std::make_pair(true, v_new);
                        }
                        else
                        {
                            //SKIP THIS NODE
                            execute_num--;
                            // PRX_DEBUG_COLOR("Skipping this node...",PRX_TEXT_RED);
                            return std::make_pair(false, node->index);
                        }
                    }
                    else
                    {
                        local_planner->steer(node->point, n_state, new_plan, new_path);
                        v_new = graph.add_vertex<rrg_node_t > ();
                        num_vertices++;
                        graph.get_vertex_as< rrg_node_t > (v_new)->init_node(state_space, n_state, validity_checker->alloc_constraint());
                        connect_node(v_new, is_resolving);
                        return std::make_pair(true, v_new);
                    }
                }
            }
            else
            {
                v_new = graph.add_vertex<rrg_node_t > ();
                num_vertices++;
                graph.get_vertex_as< rrg_node_t > (v_new)->init_node(state_space, n_state, validity_checker->alloc_constraint());
                connect_node(v_new, is_resolving);
                return std::make_pair(true, v_new);
            }
        }

        void rrg_t::connect_node(undirected_vertex_index_t v, bool is_resolving)
        {
            // PRX_DEBUG_COLOR("RRG: Connecting Node...", PRX_TEXT_BLUE);
            std::vector<const abstract_node_t*> neighbors;
            neighbors = metric->multi_query(graph[v], k);
            if(!is_resolving && neighbors.size()>0)
            {
                neighbors.erase(neighbors.begin());
            }
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

        void rrg_t::link_node_to_neighbors(undirected_vertex_index_t v, const std::vector< const abstract_node_t* >& neighbors)
        {
            // PRX_DEBUG_COLOR("RRG: Linking to Neighbours...", PRX_TEXT_BLUE);
            const undirected_node_t* node;

            // PRX_DEBUG_COLOR("Linking node: " << v, PRX_TEXT_GREEN);
            // PRX_PRINT("Linking to neighbors: " << neighbors.size(), PRX_TEXT_BLUE);
            
            for( size_t i = 0; i < neighbors.size(); i++ )
            {
                node = neighbors[i]->as< undirected_node_t > ();
                new_plan.clear();
                new_path.clear();
                local_planner->steer(graph[v]->point, node->point, new_plan, new_path);

                //If the path is valid
                if( new_plan.size() != 0 && is_valid_trajectory(new_path) )
                {
                    //Add the edge to the graph
                    double dist = metric->distance_function(graph[v]->point, node->point);
                    if(!boost::edge(v,node->index,graph.graph).second){
                        // PRX_PRINT("Linking to neighbor [" << i<<"]", PRX_TEXT_GREEN);
                        undirected_edge_index_t e = graph.add_edge< rrg_edge_t > (v, node->index, dist);
                        graph.get_edge_as<rrg_edge_t >(e)->id = num_edges;
                        graph.get_edge_as<rrg_edge_t >(e)->constraints = validity_checker->alloc_constraint();
                        num_edges++;
                        if( visualize_graph )
                            graph.get_edge_as< rrg_edge_t > (e)->path = new_path;
                    }
                }

                new_path.clear();
            }
        }

        void rrg_t::update_k(unsigned nr_nodes)
        {
            // PRX_DEBUG_COLOR("RRG: Updating k...", PRX_TEXT_BLUE);
            if( nr_nodes == 0 )
            {
                k = 0;
            }
            else
            {
                double d = state_space->get_dimension();
                double val = (1.0 / d);
                k = PRX_MAXIMUM(std::ceil((log(nr_nodes)*2.7182818284 * (1 + val))), 1);
            }
        }

        bool rrg_t::is_valid_trajectory(const sim::trajectory_t& path)
        {
            // PRX_DEBUG_COLOR("RRG: Is Valid Trajectory...", PRX_TEXT_BLUE);
            if( no_collision_query_type )
                return true;
            return validity_checker->is_valid(path);
        }

        bool rrg_t::serialize()
        {
            PRX_DEBUG_COLOR("RRG: Serializing...", PRX_TEXT_BLUE);
            PRX_INFO_S(" Inside RRG serialization now, saving to file: " << serialization_file);
            char* w = std::getenv("PRACSYS_PATH");
            std::string dir(w);
            dir += ("/prx_roadmaps/");
            std::string file = dir + serialization_file;
            PRX_DEBUG_COLOR("RRG serializing to file: " << file, PRX_TEXT_LIGHTGRAY);
            std::ofstream fout;
            fout.open(file.c_str());
            PRX_ASSERT(fout.is_open());
            graph.serialize(fout, state_space);

            if( serialize_plan )
            {

                foreach(undirected_edge_index_t e, boost::edges(graph.graph))
                {
                    if( graph.get_edge_as<rrg_edge_t > (e)->plan.size() > 0 )
                    {
                        graph.get_edge_as<rrg_edge_t > (e)->plan.save_to_stream(fout);
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

        bool rrg_t::deserialize(std::string file)
        {
            int v_offset=0;
            int e_offset=0;
            if(boost::num_vertices(graph.graph)>0){v_offset=boost::num_vertices(graph.graph)-1;}
            if(boost::num_edges(graph.graph)>0){e_offset=boost::num_edges(graph.graph)-1;}
            std::ifstream fin;
            
            if( !graph.deserialize<rrg_node_t, rrg_edge_t > (file, fin, state_space) )
            {
                PRX_FATAL_S("File could not deserialize!");
                return false;
            }
            int counter = 0;
            //    int blah;
            
            //Let's also clean up the components a bit
            clean_components( PRX_MINIMUM( boost::num_vertices(graph.graph), 1000 ) );

            foreach(undirected_edge_index_t e, boost::edges(graph.graph))
            {
                double dist = metric->distance_function(graph[boost::source(e, graph.graph)]->point, graph[boost::target(e, graph.graph)]->point);
                graph.set_weight(e, dist);
                graph.get_edge_as< rrg_edge_t >(e)->id = counter;
                graph.get_edge_as< rrg_edge_t >(e)->plan.link_control_space(this->control_space);
                graph.get_edge_as< rrg_edge_t >(e)->plan.link_state_space(this->state_space);
                counter++;
                //added to make sure that constraint classes are allocated upon deserialize: ZL 2/17
                graph[e]->constraints = validity_checker->alloc_constraint();
                edge_map[graph[e]->edge_id] = e;
            }

            double val_mu = (double)boost::num_edges(graph.graph) / (double)boost::num_vertices(graph.graph);
            double diff;

            foreach(undirected_vertex_index_t nd, boost::vertices(graph.graph))
            {
                node_map[graph[nd]->node_id+v_offset] = nd;
            }
            num_vertices = boost::num_vertices(graph.graph);
            num_edges = boost::num_edges(graph.graph);

            update_k(num_vertices);

            //TODO: deserializing the constraints is defunct, and going to be more trouble than it is worth
            // for (int i = v_offset+1; i < num_vertices; ++i)
            // {
            //     unsigned id;
            //     fin>>id;
            //     graph[node_map[id+v_offset]]->constraints->deserialize(fin);
            // }

            fin.close();

            // PRX_PRINT("Deserialized roadmap with " << boost::num_vertices(graph.graph) << " vertices ... " << boost::num_edges(graph.graph) << " edges.", PRX_TEXT_MAGENTA);
            // PRX_PRINT("Average Valence: " << val_mu << "   Valence Deviation: " << val_dev, PRX_TEXT_GREEN);
            return true;
        }

        const statistics_t* rrg_t::get_statistics()
        {
            PRX_DEBUG_COLOR("RRG: Getting Statistics...", PRX_TEXT_BLUE);
            rrg_statistics_t* stats = statistics->as<rrg_statistics_t > ();
            int num = boost::connected_components(graph.graph, graph.components);
            stats->cc_sizes.clear();
            stats->cc_sizes.resize(num);

            foreach(util::undirected_vertex_index_t v, boost::vertices(graph.graph))
            {
                stats->cc_sizes[graph.components[v]]++;
            }
            stats->num_vertices = boost::num_vertices(graph.graph);
            stats->num_edges = boost::num_edges(graph.graph);
            stats->num_connected_components = num;

            return statistics;
        }

        int rrg_t::print_components()
        {
            unsigned num_component = boost::connected_components(graph.graph, graph.components);
            PRX_PRINT("Graph has: " << num_component << " components", PRX_TEXT_RED);
            std::vector<int> comp_size(num_component,0);
            foreach(undirected_vertex_index_t vind, boost::vertices(graph.graph)){
                comp_size[graph.components[vind]]++;
            }
            for(int i=0; i<num_component; i++){
                PRX_PRINT("Component ["<<i<<"] = "<<comp_size[i], PRX_TEXT_LIGHTGRAY);
                comp_size[i]=0;
            }
            for(int i=0; i<component_connections.size(); i++)
            {
                PRX_PRINT("Connections to Component["<<i<<"] = "<<component_connections.at(i),PRX_TEXT_BLUE);
            }
            return num_component;
        }
        
        //TODO: This whole process seems... inefficient.  Gotta figure out a better way.
        void rrg_t::clean_components( unsigned graph_size )
        {
            PRX_PRINT("Cleaning components!", PRX_TEXT_BROWN);
            
            //Make sure we have up-to-date connected components information
            unsigned num_component = boost::connected_components(graph.graph, graph.components);
            
            //Then, we're going to count up how large each connected component is
            std::vector<int> comp_size(num_component,0);
            foreach(undirected_vertex_index_t vind, boost::vertices(graph.graph))
            {
                comp_size[graph.components[vind]]++;
            }
            
            //We will have a threshold for how large a component has to be in order to retain it.
            unsigned threshold = graph_size * 0.1;
            
            //Then, we will find all vertices which belong to small components
            std::vector< undirected_vertex_index_t > nodes_to_remove;
            foreach(undirected_vertex_index_t vind, boost::vertices(graph.graph))
            {
                if( comp_size[graph.components[vind]] < threshold )
                {
                    nodes_to_remove.push_back( vind );
                }
            }
            
            //Now that we know everything that has to get removed, do so
            foreach( undirected_vertex_index_t vind, nodes_to_remove )
            {
                // metric->remove_point( graph[vind] );
                graph.clear_and_remove_vertex( vind );
            }
            
            num_vertices -= nodes_to_remove.size();
            
            //Alright, now, we also need to fix the edge information, because we've removed some random vertices.
            foreach( undirected_edge_index_t e, boost::edges(graph.graph) )
            {
                //Get the edge
                abstract_edge_t* edge = graph[e];
                
                //And reset its source and target
                edge->source_vertex = dynamic_cast< rrg_node_t* >( graph[ boost::source( e, graph.graph ) ] )->node_id;
                edge->target_vertex = dynamic_cast< rrg_node_t* >( graph[ boost::target( e, graph.graph ) ] )->node_id;
            }
        }

        bool rrg_t::connects_components(undirected_vertex_index_t added_vertex)
        {
            bool connects =false;
            std::vector<int> target_nodes;
            boost::graph_traits<undirected_graph_type>::out_edge_iterator ei, ei_end;
            for (boost::tie(ei, ei_end) = boost::out_edges(added_vertex, graph.graph); ei != ei_end; ++ei) 
            {
                // undirected_vertex_index_t source = boost::source ( *ei, graph.graph );
                undirected_vertex_index_t target = boost::target ( *ei, graph.graph );
                target_nodes.push_back(graph[target]->node_id);
                // std::cout << "There is an edge from " << graph[source]->node_id <<  " to " << graph[target]->node_id << std::endl;
                for(int i=0; i<component_connections.size(); i++)
                {
                    for(int j=i+1; j<component_connections.size() && j<=i+1; j++)
                    {
                        // PRX_PRINT("Now Checking if: "<<graph[target]->node_id<<" is between ["<<roadmaps_bounds.at(i)<<","<<roadmaps_bounds.at(j)<<"]",PRX_TEXT_BLUE);
                        if(graph[target]->node_id >= roadmaps_bounds.at(i) && graph[target]->node_id < roadmaps_bounds.at(j))
                        {
                            // PRX_PRINT("TRUE",PRX_TEXT_GREEN);
                            component_connections.at(i)++;
                            connects=true;
                        }      
                    }
                }
            }
            return connects;
        }

        bool rrg_t::minimum_connections_satisfied()
        {
            // print_components();
            for(int i=0; i<component_connections.size()-1; i++)
            {
                if (component_connections.at(i)<miniumum_connections)
                {
                    return false;
                }
            }
            return true;
        }
        
        bool rrg_t::discover_candidate_neighbors( space_point_t* point, std::vector< const abstract_node_t* >& neighbor_set )
        {
            //Let's reset the component connections information
            for( unsigned i=0; i<component_connections.size(); ++i )
            {
                component_connections[i] = 0;
            }
            
            //Then, let's do the nearest neighbor query
            neighbor_set = metric->multi_query(point, k);
            
            //Next, let's figure out where every point is in terms of the connected components
            for( unsigned i=0; i<neighbor_set.size(); ++i )
            {
                unsigned comp = component( neighbor_set[i] );
                //If it is a valid connection
                if( comp < component_connections.size() )
                {
                    //Increment the corresponding component connection
                    ++component_connections[comp];
                }
            }
            
            //Then, let's start counting how many components we are adjacent to
            unsigned count = 0;
            for( unsigned i=0; i<component_connections.size() && count < 2; ++i )
            {
                if( component_connections[i] > 0 )
                {
                    ++count;
                }
            }
            
            return count > 1;
        }

        bool rrg_t::validity_check_candidate_edges( space_point_t* point, std::vector< const abstract_node_t*>& neighbor_set, std::vector< bool >& valid_edges )
        {
            //Let's reset the component connections information
            for( unsigned i=0; i<component_connections.size(); ++i )
            {
                component_connections[i] = 0;
            }            
            
            //For each candidate neighbor
            for( unsigned i=0; i<neighbor_set.size(); ++i )
            {
                //Local plan to that neighbor
                new_plan.clear();
                new_path.clear();
                local_planner->steer(point, neighbor_set[i]->point, new_plan, new_path);

                //If the path is valid
                if( new_plan.size() != 0 && is_valid_trajectory(new_path) )
                {
                    //Mark it as such
                    valid_edges[i] = true;

                    //Then, make sure we are counting
                    unsigned comp = component( neighbor_set[i] );
                    if( comp < component_connections.size() )
                    {
                        //Increment the corresponding component connection
                        ++component_connections[comp];
                    }

                }
            }
            
            //Now that we've validity checked, let's count again to see if we are still adjacent to different components
            unsigned count = 0;
            for( unsigned i=0; i<valid_edges.size() && count < 2; ++i )
            {
                if( component_connections[i] > 0 && valid_edges[i] )
                {
                    ++count;
                }
            }
            
            return count > 1;
        }

        unsigned rrg_t::component( const abstract_node_t* node )
        {
            unsigned id = dynamic_cast< const rrg_node_t* >( node )->node_id;
            
            for( unsigned i=1; i<roadmaps_bounds.size(); ++i )
            {
                if( id < roadmaps_bounds[i] )
                {
                    return i-1;
                }
            }
            return PRX_INFINITY;
        }

    }
}
