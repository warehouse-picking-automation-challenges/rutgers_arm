/**
 * @file h_graph_planner.cpp
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

#include "planning/motion_planners/subhrajit/h_graph_planner.hpp"

#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/utilities/distance_metrics/graph_metric/graph_metric.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/communication/visualization_comm.hpp"


#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/range/adaptor/map.hpp>

PLUGINLIB_EXPORT_CLASS( prx::packages::homotopies::h_graph_planner_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace plan;
    using namespace plan::comm;
    using namespace sim;
    
    
    namespace packages
    {
        namespace homotopies
        {

            std::vector<double> custom_graph;
            int custom_counter;

            h_graph_planner_t::h_graph_planner_t() 
            {
                random_point = NULL;
                update_k( 0 );
                statistics = new prm_star_statistics_t();
                num_edges = 0;
                num_vertices = 0;
                solutions_colors = {"white","red","orange","yellow","green","blue","black"};
                KL_vals = {-2,-1,0,1,2};
                custom_counter = 0;
                BL_set = TR_set = false;

            }

            h_graph_planner_t::~h_graph_planner_t() 
            {
                state_space->free_point(random_point);
                state_space->free_point(temp_state);
                new_plan.clear();
                new_plan2.clear();

            }

            void h_graph_planner_t::init(const parameter_reader_t* reader,const parameter_reader_t* template_reader)
            {
                PRX_DEBUG_S("Initializing H-graph motion planner... ");

                /** Initialize the PRM as normal */
                PRX_DEBUG_S("prm init with reader: " << reader->trace());
                motion_planner_t::init( reader, template_reader);

                visualize_graph = parameters::get_attribute_as<bool>("visualize_graph",reader,template_reader,true);
                visualize_solutions = parameters::get_attribute_as<bool>("visualize_solutions",reader,template_reader,true);
                visualize_homotopies = parameters::get_attribute_as<bool>("visualize_homotopies",reader,template_reader,true);

                visualization_graph_name = parameters::get_attribute_as<std::string>("visualization_graph_name",reader,template_reader,"prm/graph");
                visualization_solutions_name = parameters::get_attribute_as<std::string>("visualization_solutions_name",reader,template_reader,"prm/solutions");
                graph_color = parameters::get_attribute_as<std::string>("graph_color",reader,template_reader,"black");
                homotopy_threshold = parameters::get_attribute_as<double>("homotopy_threshold",reader,template_reader, 1.5);
                number_explored_homotopies = parameters::get_attribute_as<double>("number_explored_homotopies",reader,template_reader, 3);
                custom_counter = parameters::get_attribute_as<int>("custom_graph",reader,template_reader, -1);
                if (custom_counter != -1)
                    custom_counter = 0;
                computed_homotopies.resize(number_explored_homotopies);
                computed_plans.resize(number_explored_homotopies);

                weight_factor = parameters::get_attribute_as<double>("weight_factor", reader, template_reader, 1000);
                split_edge_length = parameters::get_attribute_as<double>("split_edge_length", reader, template_reader);
                max_step_length = parameters::get_attribute_as<double>("max_step_length", reader, template_reader);
                
                // IF bottom left exists, read it
                if(parameters::has_attribute("BL",reader,template_reader)) 
                {
                    std::vector<double> BL_point = parameters::get_attribute_as< std::vector<double> >("BL",reader,template_reader);
                    BL.real() = BL_point[0];
                    BL.imag() = BL_point[1];
                    BL_set = true;
                }
                if(parameters::has_attribute("TR",reader,template_reader)) 
                {
                    std::vector<double> TR_point = parameters::get_attribute_as< std::vector<double> >("TR",reader,template_reader);
                    TR.real() = TR_point[0];
                    TR.imag() = TR_point[1];
                    TR_set = true;
                }

                if(parameters::has_attribute("solutions_colors",reader,template_reader))    
                    solutions_colors = parameters::get_attribute_as<std::vector<std::string> >("solutions_colors",reader,template_reader);   
                else
                    solutions_colors.push_back("white");

                if(parameters::has_attribute("visualization_bodies",reader,template_reader))    
                    visualization_bodies = parameters::get_attribute_as<std::vector<std::string> >("visualization_bodies",reader,template_reader);   
                else if(parameters::has_attribute("visualization_body",reader,template_reader))
                    visualization_bodies = parameters::get_attribute_as<std::vector<std::string> >("visualization_body",reader,template_reader);
                else
                    PRX_WARN_S("Visualization_systems have not been specified for PRM motion planner!");

                PRX_DEBUG_COLOR ("INITIAL GRAPH CONDITION: " << boost::num_vertices(graph.graph) << " and edges: " << boost::num_edges(graph.graph), PRX_TEXT_RED);

                /** Read in the repsentative points from input */
                std::vector<const parameter_reader_t*> representative_points_readers;
                if (reader->has_attribute("representative_points"))
                    representative_points_readers = reader->get_list("representative_points");
                else
                {
                    PRX_FATAL_S ("No representative points to read!");
                }

                foreach(const parameter_reader_t* r, representative_points_readers)    
                {
                    PRX_DEBUG_COLOR ("Representative Point reading!", PRX_TEXT_GREEN);
                    std::vector<double> new_representative_point;
                    new_representative_point = r->get_attribute_as<std::vector <double> >("point");        
                    representative_points.push_back(complex_t(new_representative_point[0],new_representative_point[1]));
                    PRX_DEBUG_COLOR("Read in complex number: " << representative_points.back(), PRX_TEXT_CYAN);
                }


            }

            void h_graph_planner_t::reset()
            {
            //    if (boost::num_vertices(graph.graph) != 0)
                {
                    std::vector<directed_vertex_index_t> to_delete;
                    foreach(directed_vertex_index_t v, boost::vertices(graph.graph))
                    {
                        to_delete.push_back(v);
                    }
                    foreach(directed_vertex_index_t v, to_delete)
                    {
                        foreach(directed_vertex_index_t u, boost::adjacent_vertices(v,graph.graph))
                        {
                            directed_edge_index_t e = boost::edge(v,u,graph.graph).first;
                            graph.get_edge_as<h_edge_t>(e)->clear(control_space);
                        }
                        graph.clear_and_remove_vertex(v);
                    }
                }

                metric->clear();
                path1.clear();
                path2.clear();
                update_k( 0 );
            }

            void h_graph_planner_t::setup()
            {
                random_point = state_space->alloc_point();
                path1.link_space(state_space);
                path2.link_space(state_space);
                new_plan.link_control_space(control_space);
                new_plan2.link_control_space(control_space);
                new_plan.link_state_space(state_space);
                new_plan2.link_state_space(state_space);
                start_plan.link_control_space(control_space); start_connection.link_space(state_space);
                end_plan.link_control_space(control_space); end_connection.link_space(state_space);
                temp_state = state_space->alloc_point();
                if (!BL_set)
                {
                    // Set bottom left point of environment
                    BL.real() = state_space->get_bounds()[0]->get_lower_bound();
                    BL.imag() = state_space->get_bounds()[1]->get_lower_bound();
                }

                if (!TR_set)
                {
                    // Set top right point of environment
                    TR.real() = state_space->get_bounds()[0]->get_upper_bound();
                    TR.imag() = state_space->get_bounds()[1]->get_upper_bound();
                }

                PRX_DEBUG_COLOR("Set BL: " << BL <<", and TR: " << TR, PRX_TEXT_CYAN);

            //    scale_representative_points();
            //    BL.real() = 0.0; BL.imag() = 0.0;
            //    TR.real() = 1.0; TR.imag() = 1.0;
                calculate_a_b();
                calculate_Al();
            }

            void h_graph_planner_t::step()
            {    
                valid_random_sample();    
                add_node( random_point );       
                update_k( num_vertices);   
            }

            bool h_graph_planner_t::succeeded() const
            {
                if(input_specification->get_stopping_criterion()->satisfied())
                    return true;
                return false;
            }

            void h_graph_planner_t::resolve_query()
            {
                PRX_DEBUG_COLOR ("Inside resolve query", PRX_TEXT_BROWN);
                sys_clock_t resolve_timer;
                resolve_timer.reset();
                std::vector<space_point_t*> goals = input_query->get_goal()->get_goal_points();
                std::vector<directed_vertex_index_t> v_goals;

                input_query->plan.clear();

            //    PRX_DEBUG_S ("Num vertices: " << boost::num_vertices(graph.graph));
            //    PRX_DEBUG_S ("Num edges: " << boost::num_edges(graph.graph));

                directed_vertex_index_t v_start = add_node(input_query->get_start_state());
            //    PRX_DEBUG_S ("Num vertices #2: " << boost::num_vertices(graph.graph));
            //    PRX_DEBUG_S ("Num edges #2: " << boost::num_edges(graph.graph));

                foreach(space_point_t* g, goals)
                {
                    v_goals.push_back(add_node(g));
                }

                default_heuristic = new h_graph_distance_heuristic(metric);
                default_visitor = new h_graph_astar_visitor();

                default_heuristic->set(&graph,v_goals);
                default_visitor->set_new_goals(&v_goals);
                try
                {        
                    astar_search<
                            directed_graph_type,
                            directed_graph_t,
                            directed_vertex_index_t,
                            h_graph_distance_heuristic,
                            h_graph_astar_visitor
                    >( graph, v_start, default_heuristic , default_visitor);
                }
                catch( h_graph_found_goal fg )
                { 
                    PRX_DEBUG_COLOR ("Found goal", PRX_TEXT_MAGENTA);
                    //Smoothing functionality that has to be moved to a different prm. 
            //        if (smooth)
            //        {
            //            // construct the path
            //            std::deque<directed_vertex_index_t> path;
            //            for( directed_vertex_index_t v = v_goal; v != graph.predecessors[v]; v = graph.predecessors[v] )
            //            {
            //                path.push_front(v);
            //                PRX_ERROR_S ("Pushed onto path: " << state_space->print_point(graph[v]->point));
            //            }
            //            
            //            double smooth_duration;
            //            directed_vertex_index_t smooth_goal;
            //            PRX_ERROR_S ("Smoothing time!!!");
            //            for (double half_index = path.size() - 1; half_index > 1.0; half_index /= 2)
            //            {
            //                PRX_ERROR_S ("Half index: " << half_index);
            //                int index = (int) half_index;
            //                smooth_goal = path[index];
            //                PRX_ERROR_S ("Going from: " << state_space->print_point(graph[v_start]->point) << " to: " << state_space->print_point(graph[smooth_goal]->point));
            //                trajectory_t candidate_trajectory = local_planner->propagate(graph[v_start]->point, graph[smooth_goal]->point, sim_step, new_control);
            //                if (time_limited)
            //                {
            //                    if (time.measure() > time_limit)
            //                    {
            ////                        smooth_goal = v_goal;
            //                        break;
            //                    }
            //                }
            //                if (input_specification->validity_checker->is_valid(candidate_trajectory))
            //                {
            //                    PRX_ERROR_S ("We are smooth");
            //                    smooth_duration = candidate_trajectory.states.size()*sim_step;
            //                    break;
            //                }
            //            }
            ////            for( directed_vertex_index_t start = v_start; smooth_goal != v_start; smooth_goal = graph.predecessors[smooth_goal] )
            ////            {
            ////                PRX_ERROR_S ("Going from: " << state_space->print_point(graph[start]->point) << " to: " << state_space->print_point(graph[smooth_goal]->point));
            ////                trajectory_t blah = local_planner->propagate(graph[start]->point, graph[smooth_goal]->point, sim_step, new_control);
            ////                if (input_specification->validity_checker->is_valid(blah))
            ////                {
            ////                    PRX_ERROR_S ("We are smooth");
            ////                    smooth_duration = blah.states.size()*sim_step;
            ////                    break;
            ////                }
            ////            }
            //            for( directed_vertex_index_t v = v_goal; v != smooth_goal; v = graph.predecessors[v] )
            //            {
            //                control_t* ctrl = control_space->alloc_point();
            //                element_iterator_t s = state_space->get_element_iterator(graph[v]->point);
            //                element_iterator_t c = control_space->get_element_iterator(ctrl);
            //
            //                for( unsigned int i = 0; i < control_space->get_dimension(); ++i )
            //                    c[i] = s[i];
            //
            //                directed_edge_index_t e = boost::edge(v,graph.predecessors[v],graph.graph).first;
            //                PRX_WARN_S("plan steps:" << graph.get_edge_as<h_edge_t>(e)->sampling_steps << "   sim time:" << sim_step);
            //                PRX_WARN_S("ctrl : " << control_space->print_point(ctrl));
            //                plan.steps.push_front(plan_step_t(ctrl, graph.get_edge_as<h_edge_t>(e)->sampling_steps*sim_step));
            //
            //    //            for( boost::tie(out_iter, out_end) = boost::out_edges(graph.predecessors[v], graph.graph); out_iter != out_end; ++out_iter )
            //    //            {
            //    //                if( boost::target(*out_iter, graph.graph) == v )
            //    //                {
            //    //                    PRX_WARN_S("plan steps (out_iter):" << graph.get_edge_as<h_edge_t>(*out_iter)->sampling_steps << "   sim time:" << sim_step);
            //    //                    plan.steps.push_front(plan_step_t(ctrl, graph.get_edge_as<h_edge_t>(*out_iter)->sampling_steps*sim_step));
            //    //                }
            //    //            }
            //                solution.states.push_back(graph[v]->point);
            //            }
            //
            //            control_t* ctrl = control_space->alloc_point();
            //            element_iterator_t s = state_space->get_element_iterator(graph[smooth_goal]->point);
            //            element_iterator_t c = control_space->get_element_iterator(ctrl);
            //
            //            for( unsigned int i = 0; i < control_space->get_dimension(); ++i )
            //                c[i] = s[i];
            //
            ////            directed_edge_index_t e = boost::edge(v_start,graph.predecessors[v_start],graph.graph).first;
            ////            PRX_WARN_S("plan steps:" << graph.get_edge_as<h_edge_t>(e)->sampling_steps << "   sim time:" << sim_step);
            //            PRX_WARN_S("ctrl : " << control_space->print_point(ctrl));
            //            plan.steps.push_front(plan_step_t(ctrl, smooth_duration));
            //            solution.states.push_back(graph[smooth_goal]->point); 
            //            solution.states.push_back(graph[v_start]->point); 
            //        }
            //        PRX_ERROR_S ("Resolving took: " << resolve_timer.measure_reset());
            //        PRX_INFO_S ("Constructing plan");
                    std::deque<directed_vertex_index_t> path_vertices;
                    for( directed_vertex_index_t v = fg.v_goal ; v != v_start; v = graph.predecessors[v] )
                    {
                        path_vertices.push_front(v);
            //            PRX_DEBUG_COLOR("v : " << state_space->print_point(graph[v]->point),PRX_TEXT_MAGENTA);
                    }
                    path_vertices.push_front(v_start);

                    for(size_t i = 0; i<path_vertices.size()-1; ++i )
                    {
                        directed_edge_index_t e = boost::edge(path_vertices[i],path_vertices[i+1],graph.graph).first;
                        plan_t* ctrls = &graph.get_edge_as<h_edge_t>(e)->plan;
                        foreach(plan_step_t step, *ctrls)
                        {                
                            input_query->plan.copy_onto_back(step.control,step.duration);                
                        }
                        input_query->path += graph.get_edge_as<h_edge_t>(e)->path;  

                    }
                }
                foreach(directed_vertex_index_t u, boost::adjacent_vertices(v_start,graph.graph))
                {
                    directed_edge_index_t e = boost::edge(v_start,u,graph.graph).first;
                    graph.get_edge_as<h_edge_t>(e)->clear(control_space);
                    e = boost::edge(u,v_start, graph.graph).first;
                    graph.get_edge_as<h_edge_t>(e)->clear(control_space);
                }
                metric->remove_point(graph.get_vertex_as<directed_node_t>(v_start));
                graph.clear_and_remove_vertex(v_start);
                foreach(directed_vertex_index_t v, v_goals)
                {
                    foreach(directed_vertex_index_t u, boost::adjacent_vertices(v,graph.graph))
                    {
                        directed_edge_index_t e = boost::edge(v,u,graph.graph).first;
                        graph.get_edge_as<h_edge_t>(e)->clear(control_space);
                        e = boost::edge(u,v,graph.graph).first;
                        graph.get_edge_as<h_edge_t>(e)->clear(control_space);
                    }
                    metric->remove_point(graph.get_vertex_as<directed_node_t>(v));
                    graph.clear_and_remove_vertex(v);
                } 

                v_goals.clear();    

                delete default_heuristic;
                delete default_visitor;
            }

            bool h_graph_planner_t::resolve_homotopies(int num_homotopies)
            {
                if (num_homotopies == 0)
                {
                    num_homotopies = number_explored_homotopies;
                }
                current_number_homotopies = num_homotopies;
                computed_homotopies_list.clear();
            //    computed_homotopies.clear();
            //    computed_plans.clear();

            //    PRX_WARN_S ("Inside resolve homotopies");
                sys_clock_t resolve_timer;
                resolve_timer.reset();
                complex_t H_value;

                input_query->plan.clear();
                PRX_DEBUG_COLOR("Start state: " << input_query->get_start_state()->at(0) <<" , " << input_query->get_start_state()->at(1), PRX_TEXT_CYAN);
                directed_vertex_index_t v_start = add_node(input_query->get_start_state());
                directed_vertex_index_t v_goal = add_node(input_query->get_goal()->get_goal_points().front());
            //    PRX_WARN_S ("Before homotopy resolver");
                H_augmented_graph_t homotopy_resolver;
                homotopy_resolver.link_space(state_space);
                homotopy_resolver.setup(&graph, v_start, v_goal, num_homotopies, homotopy_threshold);
                bool found_all_homotopies = homotopy_resolver.compute_homotopies(computed_homotopies_list);
                if(found_all_homotopies)
                {
                    PRX_DEBUG_COLOR("Found all homotopies!", PRX_TEXT_LIGHTGRAY);
                    create_trajectories_and_plans();
                }

                // Removing start and goal vertices, and edges that connect them to the graph
            //    PRX_DEBUG_COLOR("Removing start vertex from graph", PRX_TEXT_GREEN);
                foreach(directed_vertex_index_t u, boost::adjacent_vertices(v_start,graph.graph))
                {
            //        PRX_WARN_S ("Removing vertex: " << u);
                    directed_edge_index_t e = boost::edge(v_start,u,graph.graph).first;
                    graph.get_edge_as<h_edge_t>(e)->clear(control_space);
                    e = boost::edge(u,v_start,graph.graph).first;
                    graph.get_edge_as<h_edge_t>(e)->clear(control_space);
                }
                metric->remove_point(graph.get_vertex_as<directed_node_t>(v_start));
                graph.clear_and_remove_vertex(v_start);
            //    PRX_DEBUG_COLOR("Removing goal vertex from graph", PRX_TEXT_BLUE);
                foreach(directed_vertex_index_t u, boost::adjacent_vertices(v_goal,graph.graph))
                {
            //        PRX_WARN_S ("Removing vertex: " << u);
                    directed_edge_index_t e = boost::edge(v_goal,u,graph.graph).first;
                    graph.get_edge_as<h_edge_t>(e)->clear(control_space);
                    e = boost::edge(u,v_goal,graph.graph).first;
                    graph.get_edge_as<h_edge_t>(e)->clear(control_space);
                }
                metric->remove_point(graph.get_vertex_as<directed_node_t>(v_goal));
                graph.clear_and_remove_vertex(v_goal);

                return found_all_homotopies;

            //    PRX_DEBUG_COLOR("Finished!", PRX_TEXT_RED);
            }
            
            bool h_graph_planner_t::resolve_homotopies(int num_homotopies, std::vector<sim::plan_t>& resolved_plans, std::vector<sim::trajectory_t>& resolved_trajectories)
            {
                current_number_homotopies = num_homotopies;
                computed_homotopies_list.clear();
                input_query->plan.clear();
                
                PRX_DEBUG_COLOR("Start state: " << input_query->get_start_state()->at(0) <<" , " << input_query->get_start_state()->at(1), PRX_TEXT_CYAN);
                directed_vertex_index_t v_start = add_node(input_query->get_start_state());
                directed_vertex_index_t v_goal = add_node(input_query->get_goal()->get_goal_points().front());
            //    PRX_WARN_S ("Before homotopy resolver");
                H_augmented_graph_t homotopy_resolver;
                homotopy_resolver.link_space(state_space);
                homotopy_resolver.setup(&graph, v_start, v_goal, current_number_homotopies, homotopy_threshold);
                bool found_all_homotopies = homotopy_resolver.compute_homotopies(computed_homotopies_list);
                if(found_all_homotopies)
                {
                    PRX_DEBUG_COLOR("Found all homotopies!", PRX_TEXT_LIGHTGRAY);
                    create_trajectories_and_plans(resolved_plans, resolved_trajectories);
                }

                // Removing start and goal vertices, and edges that connect them to the graph
            //    PRX_DEBUG_COLOR("Removing start vertex from graph", PRX_TEXT_GREEN);
                foreach(directed_vertex_index_t u, boost::adjacent_vertices(v_start,graph.graph))
                {
            //        PRX_WARN_S ("Removing vertex: " << u);
                    directed_edge_index_t e = boost::edge(v_start,u,graph.graph).first;
                    graph.get_edge_as<h_edge_t>(e)->clear(control_space);
                    e = boost::edge(u,v_start,graph.graph).first;
                    graph.get_edge_as<h_edge_t>(e)->clear(control_space);
                }
                metric->remove_point(graph.get_vertex_as<directed_node_t>(v_start));
                graph.clear_and_remove_vertex(v_start);
            //    PRX_DEBUG_COLOR("Removing goal vertex from graph", PRX_TEXT_BLUE);
                foreach(directed_vertex_index_t u, boost::adjacent_vertices(v_goal,graph.graph))
                {
            //        PRX_WARN_S ("Removing vertex: " << u);
                    directed_edge_index_t e = boost::edge(v_goal,u,graph.graph).first;
                    graph.get_edge_as<h_edge_t>(e)->clear(control_space);
                    e = boost::edge(u,v_goal,graph.graph).first;
                    graph.get_edge_as<h_edge_t>(e)->clear(control_space);
                }
                metric->remove_point(graph.get_vertex_as<directed_node_t>(v_goal));
                graph.clear_and_remove_vertex(v_goal);

                return found_all_homotopies;

            //    PRX_DEBUG_COLOR("Finished!", PRX_TEXT_RED);
            }

            void h_graph_planner_t::update_vis_info() const
            {
                if(visualization_bodies.size() <= 0)    
                {
                    PRX_FATAL_S("Empty visualization bodies");
                    return;
                }
                PRX_DEBUG_COLOR("PRM update vis info!", PRX_TEXT_RED);
                std::vector<geometry_info_t> geoms;
                std::vector<config_t> configs;    
                hash_t<std::string, std::vector<double> > map_params;
                std::vector<double> params;

                int count;

                if(visualize_graph)
                {

                    count = 0;        
                    std::vector<std::string> system_names;
                    system_names.push_back(visualization_bodies[0]);


                    foreach(directed_edge_index_t e, boost::edges(graph.graph))
                    {
                        std::string name = ros::this_node::getName() + visualization_graph_name + "/edge_" + int_to_str(count);
                        params.clear();  

                        map_params.clear();
                        ((visualization_comm_t*)vis_comm)->compute_configs(graph[boost::source( e, graph.graph)]->point,system_names,map_params);
                        params.insert(params.end(),map_params[system_names[0]].begin(),map_params[system_names[0]].end());

                        map_params.clear();
                        ((visualization_comm_t*)vis_comm)->compute_configs(graph[boost::target( e, graph.graph)]->point,system_names,map_params);
                        params.insert(params.end(),map_params[system_names[0]].begin(),map_params[system_names[0]].end());

                        geoms.push_back(geometry_info_t(visualization_bodies[0], name, PRX_LINESTRIP, params, graph_color));
                        configs.push_back(config_t());                   
                        count++;
                    }

                    ((visualization_comm_t*)vis_comm)->visualization_geom_map[visualization_graph_name] = geoms;
                    ((visualization_comm_t*)vis_comm)->visualization_configs_map[visualization_graph_name] = configs;   
                    geoms.clear();
                    configs.clear();
                }


                if(visualize_solutions && input_query->path.size() > 0)
                {        

                    hash_t<std::string, std::vector<double> > to_map_params;
                    count = 0;
                    for(size_t i=0; i<input_query->path.size()-1; i++)
                    {
                        map_params.clear();
                        to_map_params.clear();

                        ((visualization_comm_t*)vis_comm)->compute_configs(input_query->path[i],visualization_bodies,map_params);
                        ((visualization_comm_t*)vis_comm)->compute_configs(input_query->path[i+1],visualization_bodies,to_map_params);            

                        for(size_t s = 0; s < visualization_bodies.size(); s++)           
                        {         
                            params.clear();
                            params.insert(params.end(),map_params[visualization_bodies[s]].begin(),map_params[visualization_bodies[s]].end());
                            params.insert(params.end(),to_map_params[visualization_bodies[s]].begin(),to_map_params[visualization_bodies[s]].end());

                            std::string name = ros::this_node::getName() + visualization_solutions_name + "/" + visualization_bodies[s] + "/path_" + int_to_str(count);

                            geoms.push_back(geometry_info_t(visualization_bodies[s], name, PRX_LINESTRIP, params, solutions_colors[s%solutions_colors.size()]));
                            configs.push_back(config_t());                
                            count++;
                        }            
                    }   

                    ((visualization_comm_t*)vis_comm)->visualization_geom_map[visualization_solutions_name] = geoms;
                    ((visualization_comm_t*)vis_comm)->visualization_configs_map[visualization_solutions_name] = configs;   
                    geoms.clear();
                    configs.clear();
                }

                if (visualize_homotopies)
                {
                    PRX_DEBUG_COLOR("Homotopies!" , PRX_TEXT_CYAN);
                    hash_t<std::string, std::vector<double> > to_map_params;
                    count = 0;
                    int random_height = 3;
                    for (int j = 0; j < current_number_homotopies; j++)
                    {
            //            PRX_WARN_S ("Trajectory: " << j);
                        random_height += 3;
                        for(size_t i=0; i<computed_homotopies[j].size()-1; i++)
                        {
            //                PRX_ERROR_S ("State : " << computed_homotopies[j][i]->at(0) << ", " << computed_homotopies[j][i]->at(1));
                            map_params.clear();
                            to_map_params.clear();

                            ((visualization_comm_t*)vis_comm)->compute_configs(computed_homotopies[j][i],visualization_bodies,map_params);
                            ((visualization_comm_t*)vis_comm)->compute_configs(computed_homotopies[j][i+1],visualization_bodies,to_map_params);            

                            for(size_t s = 0; s < visualization_bodies.size(); s++)           
                            {         
                                params.clear();
                                params.insert(params.end(),map_params[visualization_bodies[s]].begin(),map_params[visualization_bodies[s]].end());
                                params.insert(params.end(),to_map_params[visualization_bodies[s]].begin(),to_map_params[visualization_bodies[s]].end());
                                params.back() += random_height;

                                std::string name = ros::this_node::getName() + visualization_solutions_name + "/" + visualization_bodies[s] + "/homotopy_" + int_to_str(count);

                                geoms.push_back(geometry_info_t(visualization_bodies[s], name, PRX_LINESTRIP, params, solutions_colors[j%solutions_colors.size()]));
                                configs.push_back(config_t());                
                                count++;
                            }            
                        }  
                    }

                    ((visualization_comm_t*)vis_comm)->visualization_geom_map[visualization_solutions_name] = geoms;
                    ((visualization_comm_t*)vis_comm)->visualization_configs_map[visualization_solutions_name] = configs;   
                    geoms.clear();
                    configs.clear();
                }
            }
            
            void h_graph_planner_t::approximate_resolve_query(int num_neighbors)
            {
                /** Get the set of nodes near the start state */
                std::vector<const abstract_node_t*> start_state_neighbors;
                start_state_neighbors = metric->multi_query( input_query->get_start_state(), num_neighbors );

                /** Get the set of nodes near the goal state */
                std::vector<const abstract_node_t*> goal_state_neighbors;
                goal_state_neighbors = metric->multi_query( input_query->get_goal()->get_goal_points().front(), num_neighbors );

                // TODO: Reason over more than just one neighbor
                // For now, reasons over 1 neighbor


                directed_vertex_index_t v_start = start_state_neighbors.front()->as<directed_node_t>()->index;
                directed_vertex_index_t v_goal = goal_state_neighbors.front()->as<directed_node_t>()->index;
                std::vector<directed_vertex_index_t> v_goals; v_goals.push_back(v_goal);
                //PRX_DEBUG_COLOR("V_start: " << v_start << ", V_goal: " << v_goal, PRX_TEXT_CYAN);

                input_query->plan.clear();
                input_query->path.clear();
                start_plan.clear(); end_plan.clear();
                start_connection.clear(); end_connection.clear();
            //    PRX_DEBUG_S ("Vgoals size: " << v_goals.size());
                h_graph_distance_heuristic* heuristic = new h_graph_distance_heuristic(&graph,v_goals,metric);
                h_graph_astar_visitor* visitor = new h_graph_astar_visitor();
                visitor->set_new_goals(&v_goals);
                try
                {        
                    astar_search<
                            directed_graph_type,
                            directed_graph_t,
                            directed_vertex_index_t,
                            h_graph_distance_heuristic,
                            h_graph_astar_visitor
                    >( graph, v_start, heuristic , visitor);
                }
                catch( h_graph_found_goal fg )
                { 
//                    PRX_ERROR_S("Found goal!");
                    for( directed_vertex_index_t v = fg.v_goal ; v != v_start; v = graph.predecessors[v] )
                    {
                        //copy the point to the control.
                        directed_edge_index_t e = boost::edge(graph.predecessors[v],v,graph.graph).first;
                        plan_t* ctrl = &graph.get_edge_as<h_edge_t>(e)->plan;
                        foreach(plan_step_t step, *ctrl)
                        {
                            input_query->plan.copy_onto_front(step.control,step.duration);
            //                input_query->plan.steps.push_front(plan_step_t(control_space->clone_point(step.control),step.duration ));
            //                PRX_WARN_S ("Pushed onto plan : " << control_space->print_point(step.control,3) << " of duration: " << step.duration);
            //                PRX_DEBUG_S("what is there: "<<input_query->plan[0].duration);
                        }
                    }
                    input_specification->local_planner->propagate(input_query->get_start_state(), input_query->plan, input_query->path);

            //        PRX_WARN_S("path END : " << input_query->path.size());
                }      

                if (input_query->path.size() > 0)
                {
                    
//                    PRX_DEBUG_COLOR ("Standard connection", PRX_TEXT_MAGENTA);
                    // Connect the start state 
                    input_specification->local_planner->steer(input_query->get_start_state(), input_query->path.at(0),start_plan, start_connection);
                    for (size_t index = 0; index < start_connection.size(); index++)
                    {
                        input_query->path.copy_onto_front(start_connection[index]);
                    }
                    start_plan += input_query->plan;
//                    start_connection += input_query->path;

                    // Connect the end state 

//                    input_specification->local_planner->steer(input_query->path.back(), input_query->path.at(0),start_plan, start_connection);
                    input_specification->local_planner->steer(input_query->path.back(), input_query->get_goal()->get_goal_points().front(),end_plan, end_connection);
                    input_query->path += end_connection;
                    start_plan += end_plan;
                    input_query->plan = start_plan;
                }
                else
                {
//                    PRX_DEBUG_COLOR ("Triangular connection", PRX_TEXT_BLUE);
                    input_specification->local_planner->steer(input_query->get_start_state(), start_state_neighbors.front()->as<directed_node_t>()->point,start_plan, start_connection);

                    // Connect the end state 
                    input_specification->local_planner->steer(start_connection.back(), goal_state_neighbors.front()->as<directed_node_t>()->point,end_plan, end_connection);
                    start_connection += end_connection;
                    input_query->path = start_connection;
                    input_query->plan = start_plan;
                    input_query->plan += end_plan;
                    
                    
                }
//                PRX_DEBUG_COLOR("PRE SMOOTH Path reconnection: ", PRX_TEXT_RED);
//                for (int i = 0; i < input_query->path.size(); i ++)
//                {
//                    PRX_DEBUG_COLOR("State: " << input_query->state_space->print_point(input_query->path[i]), PRX_TEXT_GREEN);
//                }
                // Attempt to smooth path
                if (input_query->path.size() > 2)
                {
                    bool smooth = false;
                    int counter = 25;
                    while (!smooth && counter >= 1)
                    {
                        start_connection.clear();
                        end_connection.clear();
                        start_plan.clear();
                        int num = (counter/25) * (input_query->path.size() - 1);
                        input_specification->local_planner->steer(input_query->get_start_state(), input_query->path[num], start_plan, start_connection );
                        if(input_specification->validity_checker->is_valid(start_connection))
                        {
                            if (counter == 25)
                            {
                                input_query->path = start_connection;
                            }
                            else
                            {
                                input_query->path.copy_segment(num, input_query->path.size()-1, &end_connection);
                                start_connection += end_connection;
                                input_query->path =  start_connection;
                                
                            }
                            
//                            PRX_DEBUG_COLOR("Path reconnection at counter: " << counter, PRX_TEXT_LIGHTGRAY);
//                            for (int i = 0; i < input_query->path.size(); i ++)
//                            {
//                                PRX_DEBUG_COLOR("State: " << input_query->state_space->print_point(input_query->path[i]), PRX_TEXT_GREEN);
//                            }
                            smooth = true;
                        }
                        counter--;
                    }
                    
                    if (smooth)
                    {
//                        PRX_DEBUG_COLOR("Smoothed!", PRX_TEXT_CYAN);
                        input_query->plan.clear();
                        for (int i = 0; i < input_query->path.size(); i ++)
                        {
                            input_query->plan.copy_onto_back(input_query->path[i], simulation::simulation_step);
                        }
                    }
                }

                v_goals.clear();
                delete heuristic;
                delete visitor;
            //    PRX_ERROR_S ("Deleting took: " << resolve_timer.measure_reset());

            }
            
            bool h_graph_planner_t::magical_resolve_query(int num_homotopies,const std::vector<state_t*>& invalid_states, double collision_radius,  
                    std::vector<plan_t>& resolved_plans, std::vector<trajectory_t>& resolved_trajectories)
            {
                altered_edges.clear();
                original_costs_for_edges.clear();
                state_t* query_start_state = input_query->get_start_state();
                state_t* query_goal_state = input_query->get_goal()->get_goal_points().front();
                double query_distance = state_space->distance(query_start_state, query_goal_state);
                std::vector<const abstract_node_t*> invalid_nodes;
                PRX_DEBUG_COLOR ("Start state:" << state_space->print_point(query_start_state) << " and goal state: " << state_space->print_point(query_goal_state), PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR ("Collision radius: " << collision_radius, PRX_TEXT_GREEN);
                /** Iterate over invalid states, and invalidate nodes */
                foreach(state_t* invst, invalid_states)
                {
                    // If the neighbor is sitting on our goal state, and we are far away, ignore this invalid state
//                    PRX_DEBUG_COLOR ("Neighbor-to-goal: " << state_space->distance(invst, query_goal_state),PRX_TEXT_GREEN);
//                    PRX_DEBUG_COLOR ("Me-to-goal: " << query_distance, PRX_TEXT_BLUE);
                    if(state_space->distance(invst, query_goal_state) >= (2*collision_radius))// || query_distance <= (4*collision_radius))
                    {
                        /** Find nearest neighbor states */
                        PRX_DEBUG_COLOR("Checking neighbor state: " << state_space->print_point(invst), PRX_TEXT_MAGENTA);
                        // TODO: Check if I can do this directly through the metric
                        std::vector<const abstract_node_t*> potential_invalid_nodes;
                        potential_invalid_nodes =  metric->radius_query(invst, 2*collision_radius);

                        PRX_DEBUG_COLOR ("Number of potential invalid nodes: " << potential_invalid_nodes.size(), PRX_TEXT_BLUE);
                        /** Iterate over potential invalid nodes */
                        foreach(const abstract_node_t* node, potential_invalid_nodes)
                        {
                            /** Collision Check */
                            if(state_space->distance(node->point, invst) <= (2*collision_radius))
                            {

                                PRX_DEBUG_COLOR ("This   point is invalid: " << state_space->print_point(node->point), PRX_TEXT_RED);
                                /** Collision check has passed, mark node for invalidation*/
                                invalid_nodes.push_back(node);

                            }
                            else
                            {
                                PRX_DEBUG_COLOR ("This   point is valid: " << state_space->print_point(node->point), PRX_TEXT_RED);
                            }

                        }
                    }
                    else
                    {
                        PRX_DEBUG_COLOR("Ignoring neighbor state: " << state_space->print_point(invst), PRX_TEXT_CYAN);
                    }
                }
                
                                
                /** Iterate over each invalid node, and invalidate their edges from the graph*/
                foreach(const abstract_node_t* node, invalid_nodes)
                {
                    directed_vertex_index_t v = node->as<directed_node_t>()->index;
                    foreach(directed_vertex_index_t u, boost::adjacent_vertices(v,graph.graph))
                    {
                        directed_edge_index_t adjacent_edge1 = boost::edge(v,u,graph.graph).first;
                        directed_edge_index_t adjacent_edge2 = boost::edge(u,v,graph.graph).first;
//                        graph.get_edge_as<prm_edge_t>(adjecent_edge1)->id
                        original_costs_for_edges.push_back(graph.weights[adjacent_edge1]);
                        altered_edges.push_back(adjacent_edge1);
                        graph.weights[adjacent_edge1] +=  weight_factor;
//                        graph.weights[adjacent_edge1] =  PRX_INFINITY;
                        original_costs_for_edges.push_back(graph.weights[adjacent_edge2]);
                        altered_edges.push_back(adjacent_edge2);
                        graph.weights[adjacent_edge2] += weight_factor;
//                        graph.weights[adjacent_edge2] = PRX_INFINITY;
                    }
                    
                }

                current_number_homotopies = num_homotopies;
                computed_homotopies_list.clear();

                input_query->plan.clear();
                PRX_DEBUG_COLOR("Start state: " << input_query->get_start_state()->at(0) <<" , " << input_query->get_start_state()->at(1), PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("Goal state: " << input_query->get_goal()->get_goal_points().front()->at(0) <<" , " << input_query->get_goal()->get_goal_points().front()->at(1), PRX_TEXT_CYAN);
                directed_vertex_index_t v_start = add_node(query_start_state);
                directed_vertex_index_t v_goal = add_node(query_goal_state);
            //    PRX_WARN_S ("Before homotopy resolver");
                PRX_DEBUG_COLOR ("SUBphase 2", PRX_TEXT_GREEN);
                
                H_augmented_graph_t homotopy_resolver;
                homotopy_resolver.link_space(state_space);
                homotopy_resolver.setup(&graph, v_start, v_goal, num_homotopies, homotopy_threshold);
                
                PRX_DEBUG_COLOR ("Checkpoint 2", PRX_TEXT_LIGHTGRAY);
//                bool found_all_homotopies = homotopy_resolver.selective_compute_homotopies(computed_homotopies_list, desired_homotopies, index_list);
                bool found_all_homotopies = homotopy_resolver.compute_homotopies(computed_homotopies_list);
                if(found_all_homotopies)
                {
                    PRX_DEBUG_COLOR("Found all homotopies!", PRX_TEXT_LIGHTGRAY);
                    create_trajectories_and_plans(resolved_plans, resolved_trajectories);
                }
                PRX_DEBUG_S ("Resetting edge weights");
                // Reset the edge weights back to what it was
                for(int i = altered_edges.size()-1; i >= 0; i--)
                {
                   graph.weights[altered_edges[i]] = original_costs_for_edges[i];
                }
                PRX_DEBUG_COLOR ("SUBphase 3", PRX_TEXT_GREEN);

                // Removing start and goal vertices, and edges that connect them to the graph
                metric->remove_point(graph.get_vertex_as<directed_node_t>(v_start));
                graph.clear_and_remove_vertex(v_start);
                metric->remove_point(graph.get_vertex_as<directed_node_t>(v_goal));
                graph.clear_and_remove_vertex(v_goal);

                return found_all_homotopies;
            }

 
            void h_graph_planner_t::invalidate_and_resolve_query_mk2(std::vector<state_t*> invalid_states, double collision_radius, double horizon_factor )
            {
                std::vector<space_point_t*> goals = input_query->get_goal()->get_goal_points();
                std::vector<directed_vertex_index_t> v_goals;


            //    PRX_DEBUG_S ("Num vertices: " << boost::num_vertices(graph.graph));
            //    PRX_DEBUG_S ("Num edges: " << boost::num_edges(graph.graph));

                directed_vertex_index_t v_start = add_node(input_query->get_start_state());
                state_t* current_state = input_query->get_start_state();
                PRX_DEBUG_COLOR ("Set start state: " << state_space->print_point(input_query->get_start_state()), PRX_TEXT_CYAN);
                
                foreach(space_point_t* g, goals)
                {
                    v_goals.push_back(add_node(g, false));
                    PRX_DEBUG_COLOR ("Set goal state: " << state_space->print_point(g), PRX_TEXT_MAGENTA);
                }
                input_query->plan.clear();
                input_query->path.clear();
                start_plan.clear(); end_plan.clear();
                start_connection.clear(); end_connection.clear();
            //    PRX_DEBUG_S ("Vgoals size: " << v_goals.size());
                                
                state_t* query_start_state = input_query->get_start_state();
                state_t* query_goal_state = input_query->get_goal()->get_goal_points().front();
                double query_distance = state_space->distance(query_start_state, query_goal_state);
                std::vector<const abstract_node_t*> invalid_nodes;
                PRX_DEBUG_COLOR ("Start state:" << state_space->print_point(query_start_state) << " and goal state: " << state_space->print_point(query_goal_state), PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR ("Collision radius: " << collision_radius, PRX_TEXT_GREEN);
                sys_clock_t test_clock;
                test_clock.reset();
                /** Iterate over invalid states, and invalidate nodes */
                foreach(state_t* invst, invalid_states)
                {
                    // If the neighbor is sitting on our goal state, and we are far away, ignore this invalid state
//                    PRX_DEBUG_COLOR ("Neighbor-to-goal: " << state_space->distance(invst, query_goal_state),PRX_TEXT_GREEN);
//                    PRX_DEBUG_COLOR ("Me-to-goal: " << query_distance, PRX_TEXT_BLUE);
                    if(state_space->distance(invst, query_goal_state) >= (2*collision_radius))// || query_distance <= (4*collision_radius))
                    {
                        /** Find nearest neighbor states */
//                        PRX_DEBUG_COLOR("Checking neighbor state: " << state_space->print_point(invst), PRX_TEXT_MAGENTA);
                        // TODO: Check if I can do this directly through the metric
                        std::vector<const abstract_node_t*> potential_invalid_nodes;
                        potential_invalid_nodes =  metric->radius_query(invst, 2*collision_radius);

//                        PRX_DEBUG_COLOR ("Number of potential invalid nodes: " << potential_invalid_nodes.size(), PRX_TEXT_BLUE);
//                        /** Iterate over potential invalid nodes */
                        foreach(const abstract_node_t* node, potential_invalid_nodes)
                        {
                            /** Collision Check */
                            if(state_space->distance(node->point, invst) <= (2*collision_radius))
                            {

//                                PRX_DEBUG_COLOR ("This   point is invalid: " << state_space->print_point(node->point), PRX_TEXT_RED);
                                /** Collision check has passed, mark node for invalidation*/
                                invalid_nodes.push_back(node);

                            }
//                            else
//                            {
//                                PRX_DEBUG_COLOR ("This   point is valid: " << state_space->print_point(node->point), PRX_TEXT_RED);
//                            }

                        }
                    }
                    else
                    {
                        PRX_DEBUG_COLOR("Ignoring neighbor state: " << state_space->print_point(invst), PRX_TEXT_CYAN);
                    }
                }
                
//                PRX_PRINT ("Invalidation took: " << test_clock.measure_reset(), PRX_TEXT_CYAN);
//                PRX_DEBUG_COLOR ("Adding weight factor: " << weight_factor, PRX_TEXT_CYAN);
//                PRX_DEBUG_COLOR ("Horizon factor: " << horizon_factor, PRX_TEXT_GREEN);
                
                /** Iterate over each invalid node, and invalidate their edges from the graph*/
                foreach(const abstract_node_t* node, invalid_nodes)
                {
                    directed_vertex_index_t v = node->as<directed_node_t>()->index;
                    foreach(directed_vertex_index_t u, boost::adjacent_vertices(v,graph.graph))
                    {
                        double dist = state_space->distance(current_state, graph.get_vertex_as<directed_node_t>(u)->point);
//                        PRX_DEBUG_COLOR ("Current state: " << state_space->print_point(current_state), PRX_TEXT_GREEN);
//                        PRX_DEBUG_COLOR ("Neighbor check: " << state_space->print_point(graph.get_vertex_as<directed_node_t>(u)->point), PRX_TEXT_BLUE);
//                        PRX_WARN_S ("Dist: " << dist);
                        if (dist < horizon_factor)
                        {
//                            PRX_DEBUG_COLOR ("Dist: " << dist << " has factor: " <<  ((horizon_factor-dist)/horizon_factor), PRX_TEXT_MAGENTA);
                            directed_edge_index_t adjacent_edge1 = boost::edge(v,u,graph.graph).first;
                            directed_edge_index_t adjacent_edge2 = boost::edge(u,v,graph.graph).first;
    //                        graph.get_edge_as<prm_edge_t>(adjecent_edge1)->id
                            original_costs_for_edges.push_back(graph.weights[adjacent_edge1]);
                            altered_edges.push_back(adjacent_edge1);
                            graph.weights[adjacent_edge1] +=  ((horizon_factor-dist)/horizon_factor)*weight_factor;
    //                        graph.weights[adjacent_edge1] =  PRX_INFINITY;
                            original_costs_for_edges.push_back(graph.weights[adjacent_edge2]);
                            altered_edges.push_back(adjacent_edge2);
                            graph.weights[adjacent_edge2] += ((horizon_factor-dist)/horizon_factor)*weight_factor;
    //                        graph.weights[adjacent_edge2] = PRX_INFINITY;
                        }
                    }
                    
                }
                
//                PRX_PRINT ("Weight changing took: " << test_clock.measure_reset(), PRX_TEXT_CYAN);
                h_graph_distance_heuristic* heuristic = new h_graph_distance_heuristic(&graph,v_goals,metric);
                h_graph_astar_visitor* visitor = new h_graph_astar_visitor();
                visitor->set_new_goals(&v_goals);
                try
                {        
                    astar_search<
                            directed_graph_type,
                            directed_graph_t,
                            directed_vertex_index_t,
                            h_graph_distance_heuristic,
                            h_graph_astar_visitor
                    >( graph, v_start, heuristic , visitor);
                }
                catch( h_graph_found_goal fg )
                { 
//                    PRX_DEBUG_COLOR ("Found goal", PRX_TEXT_CYAN);
//                    PRX_PRINT ("ASTAR took: " << test_clock.measure_reset(), PRX_TEXT_CYAN);
                    for( directed_vertex_index_t v = fg.v_goal ; v != v_start; v = graph.predecessors[v] )
                    {
                        //copy the point to the control.
                        directed_edge_index_t e = boost::edge(graph.predecessors[v],v,graph.graph).first;
//                        PRX_DEBUG_COLOR ("WEIGHT OF EDGE: " << graph.weights[e], PRX_TEXT_RED);
                        
                        foreach(const abstract_node_t* node, invalid_nodes)
                        {
                            directed_vertex_index_t v_test = node->as<directed_node_t>()->index;
                            if (v_test == v)
                            {
                                PRX_DEBUG_COLOR ("--------------WARNING---------------- --------------WARNING -------------- WARNING", PRX_TEXT_MAGENTA);
                            }
                        }
                        
                        plan_t* ctrl = &graph.get_edge_as<h_edge_t>(e)->plan;

                        foreach(plan_step_t step, *ctrl)
                        {
                            input_query->plan.copy_onto_front(step.control,step.duration);
            //                returned_plan.steps.push_front(plan_step_t(control_space->clone_point(step.control),step.duration ));
                            PRX_DEBUG_COLOR ("Pushed onto plan: " << control_space->print_point(step.control,3) << " of duration: " << step.duration, PRX_TEXT_BROWN);
                        }
                        input_query->plan.copy_end_state(graph[v]->point);
                    }
                    input_specification->local_planner->propagate(graph[v_start]->point, input_query->plan, input_query->path);
            //        PRX_WARN_S("path END : " << input_query->path.size());

                }
                PRX_DEBUG_S ("Resetting edge weights");
                for(int i = altered_edges.size()-1; i >= 0; i--)
                {
                   graph.weights[altered_edges[i]] = original_costs_for_edges[i];
                }
 
                metric->remove_point(graph.get_vertex_as<directed_node_t>(v_start));
                graph.clear_and_remove_vertex(v_start);
                foreach(directed_vertex_index_t v, v_goals)
                {
                    metric->remove_point(graph.get_vertex_as<directed_node_t>(v));
                    graph.clear_and_remove_vertex(v);
                } 

                original_costs_for_edges.clear();
                altered_edges.clear();
                
                delete heuristic;
                delete visitor;
                
            }

            // PRM specifics

            void h_graph_planner_t::valid_random_sample()
            {
            //    PRX_WARN_S ("K: " << k);
                do
                {
                    //TODO : we should remove that code someday :)
                    //Keep that code here for the embedded link point. 
                    //You need to allocate a new point every time so as the
                    //collision checker will know the new point. 
            //        if( random_point != NULL )
            //        {
            //            state_space->free_point( random_point );
            //        }
            //        //Up to here to remove :) 
            //        random_point = state_space->alloc_point();
                    sampler->sample( state_space, random_point );        
                } 
                while( !(input_specification->validity_checker->is_valid( random_point )) );
            }

            directed_vertex_index_t h_graph_planner_t::add_node(const space_point_t* n_state, bool collision_check)
            {
                v_new = graph.add_vertex<h_node_t>();
                num_vertices++;
                graph.get_vertex_as<h_node_t>(v_new)->init_node(state_space, n_state);
                PRX_DEBUG_COLOR ("New node: " << state_space->print_point(graph[v_new]->point), PRX_TEXT_BROWN);

                connect_node(v_new, collision_check);

                return v_new;
            }

            void h_graph_planner_t::connect_node( directed_vertex_index_t v, bool collision_check)
            {
                std::vector<const abstract_node_t*> neighbors;
                neighbors = metric->multi_query( graph[v], k );

                link_node_to_neighbors( v, neighbors, collision_check );
            }

            void h_graph_planner_t::connect_node( directed_vertex_index_t v, double rad , bool collision_check)
            {
                std::vector< const abstract_node_t* > neighbors;
                neighbors = metric->radius_query( graph[v], rad );

                link_node_to_neighbors( v, neighbors, collision_check );
            }


            void h_graph_planner_t::link_node_to_neighbors( directed_vertex_index_t v, const std::vector< const abstract_node_t* >& neighbors, bool collision_check)
            {
                const directed_node_t* node;

//                PRX_DEBUG_COLOR ("Number of neighbors : " << neighbors.size(), PRX_TEXT_BROWN);
                path1.clear( );
                path2.clear( );
                for (size_t i = 0; i < neighbors.size(); i++)
                {
                    node = neighbors[i]->as<directed_node_t>();
                    //First propagate to that point
            //        PRX_DEBUG_S ("Propagating the local planner... BEFORE");
//                    PRX_DEBUG_COLOR ("Going from " << state_space->print_point(graph[v]->point) << " to : " << state_space->print_point(node->point), PRX_TEXT_BROWN);
                    new_plan.clear();
                    new_plan2.clear();

                    input_specification->local_planner->steer( graph[v]->point, node->point, new_plan, path1 ); 
                    input_specification->local_planner->steer( node->point, graph[v]->point, new_plan2, path2 ); 
            //        PRX_WARN_S ("Printing path 1: ");
            //        foreach( state_t* stat, path1)
            //        {
            //            PRX_ERROR_S ("State: " << state_space->print_point(stat));
            //        }
            //        PRX_WARN_S ("Printing path 2: ");
            //        foreach( state_t* stat, path2)
            //        {
            //            PRX_ERROR_S ("State: " << state_space->print_point(stat));
            //        }     
                    //If the path is valid
                    if( !collision_check || (input_specification->validity_checker->is_valid(path1) && input_specification->validity_checker->is_valid(path2)) )
                    {
                        
                        //Add the edge to the graph
                        double dist = metric->distance_function(graph[v]->point, node->point);     
                        
                        directed_edge_index_t e = graph.add_edge<h_edge_t>( v, node->index, dist);
//                        PRX_DEBUG_COLOR("Valid with weight: " << graph.weights[e], PRX_TEXT_CYAN);
                        graph.get_edge_as<h_edge_t>(e)->id = num_edges;
                        num_edges++;
                        graph.get_edge_as<h_edge_t>(e)->init(control_space,new_plan,path1);

                        graph.get_edge_as<h_edge_t>(e)->h_value =  calculate_H_signature(&path1);

                        directed_edge_index_t e2 = graph.add_edge<h_edge_t>( node->index, v, dist); 
                        graph.get_edge_as<h_edge_t>(e2)->id = num_edges;
                        num_edges++;
                        graph.get_edge_as<h_edge_t>(e2)->init(control_space,new_plan2,path2);

                        // potentially calculate reverse edge h-value
                        graph.get_edge_as<h_edge_t>(e2)->h_value = calculate_H_signature(&path2);

                    }

                    path1.clear( );
                    path2.clear( );

                }

                //add the generated node to the metric module. Add node is not doing it
                //so as the nearest neighbor will not return itself.
                if (metric->has_point(graph[v]))
                {
                    PRX_WARN_S ("Metric already has the point! Skipped add");
                }
                else
                    metric->add_point( graph[v] );
            }
            void h_graph_planner_t::update_k( unsigned nr_nodes )
            {
                if( nr_nodes == 0 )
                    k = 0;
                else
                    k = (log( nr_nodes )*2.7182818284*(1 + (1.0/(double)control_space->get_dimension())))+0.5000000;
            }

            const std::vector<trajectory_t>& h_graph_planner_t::get_homotopic_trajectories()
            {
            //    PRX_DEBUG_COLOR("Retrieving homotopic trajectories", PRX_TEXT_LIGHTGRAY);
                return computed_homotopies;
            }


            const std::vector<plan_t>& h_graph_planner_t::get_homotopic_plans()
            {
            //    PRX_DEBUG_COLOR("Retrieving homotopic plans", PRX_TEXT_LIGHTGRAY);

                //
                return computed_plans;
            }



            complex_t h_graph_planner_t::polynomial_analytic_function(complex_t z)
            {
                return pow((z-BL),a)*pow((z-TR), b);
            }

            void h_graph_planner_t::calculate_a_b()
            {
                PRX_DEBUG_COLOR("Rep points size: " << representative_points.size(), PRX_TEXT_BLUE);
            //    int N = std::ceil(representative_points.size() / 10);
                int N = representative_points.size();
//                if (N == 1)
//                {
//                    a = 0;
//                    b = 0;
//                }
//                else
                {
                    b = N/2;
                    PRX_DEBUG_COLOR ("b: " << b << " vs: N: " << N, PRX_TEXT_BLUE);
                    float temp = (float(N))/2;
                    a = std::ceil(temp) - 1;
                    PRX_DEBUG_COLOR ("a: " << a << " vs: N: " << N, PRX_TEXT_BLUE);
             
                }
                if ( std::abs(a-b) > 1 || (a+b) != N - 1 )
                {
                    PRX_FATAL_S("Incorrect a: " << a <<" vs b: " << b);
                }

            //    PRX_DEBUG_COLOR("Chose a:" << a << " and b: " << b, PRX_TEXT_CYAN);
            }

            void h_graph_planner_t::calculate_Al()
            {
            //    complex_t sum(0,0);
                for (int l = 0; l < representative_points.size(); l++)
                {
                    // Calculates the top half of Al
                    complex_t top_half (polynomial_analytic_function(representative_points[l]));
            //        PRX_WARN_S ("Top half: " << top_half);
                    complex_t bottom_half;
                    bool first_multiplication = true;

                    // Calculates the bottom half of Al
                    for (int j = 0; j < representative_points.size(); j++)
                    {
                        if ( j != l )
                        {
                            if (first_multiplication)
                            {
                                bottom_half = (representative_points[l] - representative_points[j]);
                                first_multiplication = false;
                            }
                            else
                                bottom_half *= (representative_points[l] - representative_points[j]);
                        }
                    }
            //        PRX_WARN_S ("Bottom half: " << bottom_half);
            //        PRX_ERROR_S ("Top/bottom: " << std::norm(bottom_half));
                    Al.push_back(top_half/bottom_half);
                    PRX_DEBUG_COLOR("Calculated Al(" << l << ") :" << Al.back(), PRX_TEXT_CYAN);
            //        sum += Al.back();
                }
            //    sum /= representative_points.size();
            //    PRX_DEBUG_COLOR("Averaged Al: " << sum, PRX_TEXT_RED);
            }

            /** Obstacle marker functions */

            complex_t h_graph_planner_t::calculate_edge_signature(complex_t z1, complex_t z2)
            {
                complex_t sum(0,0);
                for (int l = 0; l < representative_points.size(); l++)
                {
                    complex_t diff2 = z2 - representative_points[l];
                    complex_t diff1 = z1 - representative_points[l];
                    complex_t left_inner = std::log(std::abs(diff2));
                    left_inner -= std::log(std::abs(diff1));

                    // Calculate right imaginary sum
                    double right_inner_temp = std::arg(diff2) - std::arg(diff1);
                    double lowest_product = PRX_INFINITY;
                    // Find the smallest sum
                    int smallest_index;
                    for (unsigned i = 0; i < KL_vals.size(); i++)
                    {
                        double test_product = std::fabs(right_inner_temp + 2*KL_vals[i]*PRX_PI);
                        if (test_product < lowest_product)
                        {
                            lowest_product = test_product;
                            smallest_index = i;

                        }
                    }
            //        PRX_DEBUG_COLOR("Lowest product of: " << lowest_product, PRX_TEXT_BROWN);
                    lowest_product = (right_inner_temp + 2*KL_vals[smallest_index]*PRX_PI);
                    complex_t right_inner(0, lowest_product);
                    sum += (Al[l]*(left_inner + right_inner));

                }
                return sum;
            }

            // ---- H-Graph Specifics

            complex_t h_graph_planner_t::calculate_H_signature(trajectory_t* path)
            {
                complex_t edge_signature(0,0);
                if (path->size() > 1)
                {
                    complex_t point1, point2;
                    point1 = complex_t((*path)[0]->at(0), (*path)[0]->at(1));
                    point2 = complex_t((*path)[1]->at(0), (*path)[1]->at(1));

                    edge_signature = calculate_edge_signature(point1, point2);
                    for (int i = 1; i < path->size() - 1;  i++)
                    {
                        point1.real() = (*path)[i]->at(0); point1.imag() = (*path)[i]->at(1);
                        point2.real() = (*path)[i+1]->at(0); point2.imag() = (*path)[i+1]->at(1);
                        edge_signature += calculate_edge_signature(point1, point2);
                    }

//                    PRX_DEBUG_COLOR("Calculated edge signature!", PRX_TEXT_MAGENTA);
//                    PRX_DEBUG_COLOR("H-signature: " << edge_signature, PRX_TEXT_CYAN);
                }
                else
                {
            //        PRX_WARN_S ("Only one point in trajectory. No signature computed");
                }
                return edge_signature;
            }

            void h_graph_planner_t::create_trajectories_and_plans()
            {
                int counter = 0;
                foreach(edge_index_list_t edge_list, computed_homotopies_list)
                {
                    computed_homotopies[counter] = graph.get_edge_as<h_edge_t>(edge_list[0])->path;
            //        complex_t test_val2 = graph.get_edge_as<h_edge_t>(edge_list[0])->h_value;
            //        complex_t test_val3 = calculate_H_signature(&graph.get_edge_as<h_edge_t>(edge_list[0])->path);
            //        PRX_DEBUG_COLOR("Test val2: " << test_val2 << "vs. test val3: " << test_val3, PRX_TEXT_BLUE);
                    for (size_t i=1; i<edge_list.size(); i++)
                    {
                        computed_homotopies[counter] +=graph.get_edge_as<h_edge_t>(edge_list[i])->path;
            //            PRX_DEBUG_COLOR("Test 2 what are you?!: " << graph.get_edge_as<h_edge_t>(edge_list[i])->h_value, PRX_TEXT_CYAN);
            //            test_val2 += graph.get_edge_as<h_edge_t>(edge_list[i])->h_value;
            //            
            //            test_val3 += calculate_H_signature(&graph.get_edge_as<h_edge_t>(edge_list[i])->path);

            //            PRX_DEBUG_COLOR("Test val2: " << test_val2 << "vs. test val3: " << test_val3, PRX_TEXT_RED);
                    }
            //        complex_t test_val = calculate_H_signature(&computed_homotopies[counter]);
            //        PRX_DEBUG_COLOR("Comparison solution homotopy: " << test_val << " vs. original: " << test_val2 << " vs. alternate: " << test_val3, PRX_TEXT_GREEN);
                    counter++;
//                    foreach(state_t* st, computed_homotopies[counter-1])
//                    {
//                        PRX_DEBUG_COLOR("Homotopy traj: " << state_space->print_point(st), PRX_TEXT_GREEN);
//                    }
                }

                counter = 0;
                foreach(edge_index_list_t edge_list, computed_homotopies_list)
                {
                    computed_plans[counter] = graph.get_edge_as<h_edge_t>(edge_list[0])->plan;

                    for (size_t i=1; i<edge_list.size(); i++)
                    {
                        computed_plans[counter] +=graph.get_edge_as<h_edge_t>(edge_list[i])->plan;

                    }
                    counter++;
                }

            }
            
            void h_graph_planner_t::create_trajectories_and_plans(std::vector<sim::plan_t>& resolved_plans, std::vector<sim::trajectory_t>& resolved_trajectories)
            {
                int counter = 0;
//                foreach(edge_index_list_t edge_list, computed_homotopies_list)
                for(unsigned X = 0; X < computed_homotopies_list.size(); X++)
                {
                    edge_index_list_t edge_list = computed_homotopies_list[X];
                    resolved_trajectories[counter] = graph.get_edge_as<h_edge_t>(edge_list[0])->path;
                    
                    for (size_t i=1; i<edge_list.size(); i++)
                    {
                        resolved_trajectories[counter] +=graph.get_edge_as<h_edge_t>(edge_list[i])->path;
 
                    }
                    counter++;
                }

                counter = 0;
//                foreach(edge_index_list_t edge_list, computed_homotopies_list)
                for(unsigned X = 0; X < computed_homotopies_list.size(); X++)
                {
                    edge_index_list_t edge_list = computed_homotopies_list[X];
                    resolved_plans[counter] = graph.get_edge_as<h_edge_t>(edge_list[0])->plan;

                    for (size_t i=1; i<edge_list.size(); i++)
                    {
                        resolved_plans[counter] +=graph.get_edge_as<h_edge_t>(edge_list[i])->plan;

                    }
                    counter++;
                }

            }

            void h_graph_planner_t::scale_representative_points()
            {
                double scale_max = 1, scale_min = 0;

                double x_min = BL.real();
                double x_max = TR.real();

                double y_min = BL.imag();
                double y_max = TR.imag();

                for (size_t i = 0; i < representative_points.size(); i++)
                {
                    complex_t scaled_point;
                    double x = representative_points[i].real();
                    double y = representative_points[i].imag();
                    scaled_point.real() = (scale_max-scale_min)*(x-x_min)/(x_max-x_min) + scale_min;
                    scaled_point.imag() = (scale_max-scale_min)*(y-y_min)/(y_max-y_min) + scale_min;

                    representative_points[i] = scaled_point;
                }

            }
            
            bool h_graph_planner_t::serialize()
            {
                PRX_DEBUG_COLOR(" Inside H-graph serialization now, saving to file: " << serialization_file, PRX_TEXT_RED);
                char* w = std::getenv("PRACSYS_PATH");
                std::string dir(w);
                dir += ("/prx_output/");
                std::string file = dir + serialization_file;
                PRX_DEBUG_S("File directory is: " << file);
                std::ofstream fout;
                fout.open(file.c_str());
                PRX_ASSERT(fout.is_open());
                graph.serialize(fout, state_space);

                foreach(directed_edge_index_t e, boost::edges(graph.graph))
                {
                    graph.get_edge_as<h_edge_t > (e)->plan.save_to_stream(fout);
                }
                fout.close();
                return true;

            }

            bool h_graph_planner_t::deserialize()
            {
                PRX_DEBUG_COLOR(" Inside H-graph deserialization now, opening file: " << deserialization_file, PRX_TEXT_RED);
                char* w = std::getenv("PRACSYS_PATH");
                std::string dir(w);
                dir += ("/prx_input/");
                std::string file = dir + deserialization_file;
                PRX_DEBUG_S("File directory is: " << file);
    //            fin.open(file.c_str());
    //            PRX_ASSERT(fin.is_open());
                std::ifstream fin;
                
                
//                if (!skip_sleep)
//                {
                    double sleep_time = uniform_random(0.0,10.0);
                    sleep(sleep_time);
//                }
                    
                if (!graph.deserialize<h_node_t, h_edge_t > (file, fin, state_space))
                {
                    PRX_FATAL_S ("File could not deserialize!");
                    return false;
                }
                int counter = 0;
                //    int blah;

                foreach(directed_edge_index_t e, boost::edges(graph.graph))
                {

                    graph.get_edge_as<h_edge_t > (e)->id = counter;
                    graph.get_edge_as<h_edge_t > (e)->plan.link_control_space(this->control_space);
                    graph.get_edge_as<h_edge_t > (e)->plan.link_state_space(this->state_space);
                    graph.get_edge_as<h_edge_t > (e)->plan.read_from_stream(fin);
                    graph.get_edge_as<h_edge_t >(e)->path.link_space(this->state_space);
                    counter++;
                }

                foreach(directed_vertex_index_t nd, boost::vertices(graph.graph))
                {
                    PRX_DEBUG_S("Added to metric: " << state_space->print_point(graph.graph[nd].node->point));
                    metric->add_point(graph[nd]);
                    PRX_DEBUG_S("Metric now has: " << metric->get_nr_points() << " points");
                }
                update_k(boost::num_vertices(graph.graph));
                split_edges(split_edge_length);
                
                int edge_counter = 0;
                foreach(directed_vertex_index_t nd, boost::vertices(graph.graph))
                {
                    foreach(directed_vertex_index_t u, boost::adjacent_vertices(nd,graph.graph))
                    {
                        directed_edge_index_t e = boost::edge(nd,u,graph.graph).first;
                        path1.clear( );
    //                    directed_vertex_index_t target = graph.id_to_node_map[graph.get_edge_as<h_edge_t >(e)->target_vertex];
    //                    input_specification->local_planner->steer( graph[source]->point, graph[target]->point, graph.get_edge_as<h_edge_t >(e)->plan, path1 );
                        input_specification->local_planner->propagate(graph[nd]->point, graph.get_edge_as<h_edge_t >(e)->plan, graph.get_edge_as<h_edge_t >(e)->path );
//                        PRX_DEBUG_COLOR("Steered from: " << state_space->print_point(graph[nd]->point) << " to: " << state_space->print_point(graph[u]->point) << " obtain: ", PRX_TEXT_BROWN);
    //                    PRX_DEBUG_COLOR(" Plan: " << graph.get_edge_as<h_edge_t >(e)->plan.print(), PRX_TEXT_BLUE);
//                        foreach(state_t* st, path1)
//                        {
//                            PRX_DEBUG_COLOR("St: " << state_space->print_point(st), PRX_TEXT_GREEN);
//                        }
                        graph.get_edge_as<h_edge_t>(e)->h_value = calculate_H_signature(&graph.get_edge_as<h_edge_t >(e)->path);
                        edge_counter++;
                    }
                }
                
                PRX_DEBUG_COLOR ("EDGE COUNT: " << edge_counter, PRX_TEXT_MAGENTA);
                
                fin.close();
                PRX_DEBUG_S("Finished deserialization!");
                return true;
            }
            void h_graph_planner_t::split_edges(double min_length)
            {
                
                std::vector<directed_vertex_index_t> verts;
                std::vector< std::vector<directed_vertex_index_t> > adj_verts;
                foreach(directed_vertex_index_t v, boost::vertices(graph.graph))
                {
                    verts.push_back(v);
                    std::vector<directed_vertex_index_t> temp_verts;
                    foreach(directed_vertex_index_t u, boost::adjacent_vertices(v,graph.graph))
                    {
                        bool inserted = false;
                        for (unsigned test = 0; test < verts.size() && !inserted; test++)
                        {
                            if (verts[test] == u)
                                inserted = true;
                        }
                        if (!inserted)
                            temp_verts.push_back(u);
                    }
                    adj_verts.push_back(temp_verts);
                }
                int interval = std::floor(min_length/max_step_length);
                PRX_DEBUG_COLOR ("Splitting edges with min length: " << min_length << " and max step : " << max_step_length, PRX_TEXT_CYAN);
            
                for (unsigned ind = 0; ind < verts.size(); ind++)
                {
                    
                    directed_vertex_index_t v = verts[ind];
                    foreach(directed_vertex_index_t u, adj_verts[ind])
                    {
            
                        directed_edge_index_t e = boost::edge(v,u,graph.graph).first;
                        directed_edge_index_t e2 = boost::edge(u,v,graph.graph).first;
                        path1.clear(); path2.clear();
                        new_plan.clear(); new_plan2.clear();
//                        PRX_ERROR_S ("Left point: " << state_space->print_point(graph[v]->point));
//                        PRX_ERROR_S ("Right point: " << state_space->print_point(graph[u]->point));
//                        PRX_DEBUG_COLOR ("Plan: " << graph.get_edge_as<motion_planner_edge_t>(e)->plan.print(), PRX_TEXT_LIGHTGRAY );
                        input_specification->local_planner->propagate(graph.get_vertex_as<directed_node_t>(v)->point,graph.get_edge_as<h_edge_t>(e)->plan, path1);
                        double path_length = path1.length();
//                        PRX_DEBUG_COLOR ("Trajectory: " << path1.print(), PRX_TEXT_MAGENTA);
//                        sleep(10);
                        if (path_length > min_length)
                        {
                            
//                            PRX_DEBUG_COLOR ("Path 1 has length: " << path_length, PRX_TEXT_MAGENTA);
                            // Build vector of new split vertices and add points to metric
                            std::vector<directed_vertex_index_t> new_vertices;
                            new_vertices.push_back(v);
                            unsigned i;
//                            PRX_DEBUG_COLOR ("checkpoint 1" ,PRX_TEXT_GREEN);
                            for (i = interval; i < path1.size(); i+= interval)
                            {
//                                PRX_DEBUG_COLOR ("i: " << i << " vs: path size: " << path1.size(), PRX_TEXT_GREEN);
                                new_vertices.push_back(graph.add_vertex<h_node_t > ());
                                graph.get_vertex_as<h_node_t > (new_vertices.back())->init_node(state_space, path1[i]);
                                if( metric->has_point(graph[new_vertices.back()]) )
                                {
                                    PRX_WARN_S("Metric already has the point! Skipped add");
                                }
                                else
                                    metric->add_point(graph[new_vertices.back()]);
                            }
                            new_vertices.push_back(u);
//                            PRX_DEBUG_COLOR ("checkpoint 2" ,PRX_TEXT_BLUE);
                            // Build edges
                            for (unsigned j=0; j < new_vertices.size() - 1; j++)
                            {
//                                PRX_DEBUG_COLOR ("Current: " << j << ", TOTAL: " << new_vertices.size(), PRX_TEXT_BROWN);
                                directed_vertex_index_t left = new_vertices[j];
                                directed_vertex_index_t right = new_vertices[j+1];
//                                PRX_DEBUG_S ("Left point: " << state_space->print_point(graph[left]->point));
//                                PRX_DEBUG_S ("Right point: " << state_space->print_point(graph[right]->point));
                                double dist = metric->distance_function(graph[left]->point, graph[right]->point);
                                
                                directed_edge_index_t new_e = graph.add_edge<h_edge_t >(left,right,dist);
                                path2.clear(); new_plan2.clear();
                                input_specification->local_planner->steer(graph[left]->point,graph[right]->point,new_plan2, path2);
                                graph.get_edge_as<h_edge_t > (new_e)->id = num_edges;
                                num_edges++;
                                graph.get_edge_as<h_edge_t > (new_e)->init(control_space, new_plan2, path2);
                                
                                directed_edge_index_t new_e2 = graph.add_edge<h_edge_t >(right,left,dist);
                                path2.clear(); new_plan2.clear();
                                input_specification->local_planner->steer(graph[right]->point,graph[left]->point,new_plan2, path2);
                                graph.get_edge_as<h_edge_t > (new_e2)->id = num_edges;
                                num_edges++;
                                graph.get_edge_as<h_edge_t > (new_e2)->init(control_space, new_plan2, path2);
                            }
                            

                            // Remove old edge from the graph
                            graph.remove_edge(e);
                            graph.remove_edge(e2);
                        }
                    }
                }
                PRX_DEBUG_COLOR ("Done splitting edges", PRX_TEXT_BLUE);
            }
            
            void h_graph_planner_t::set_connection_properties(int max_n, bool collision_check)
            {
                max_neighbor_links = max_n;
                collision_check_links = collision_check;
            }
            
            bool h_graph_planner_t::is_same_homotopy(complex_t h1, complex_t h2)
            {
                double normed = std::norm(h1 - h2) ;
//                double h_thresh = std::sqrt(homotopy_threshold);
//                double normed = std::abs(h1.real()-h2.real()) + std::abs(h1.imag() - h2.imag());
                PRX_DEBUG_COLOR ("H1.real: " << h1.real() << " h2.real: " << h2.real() << " diff: " << h1.real() - h2.real(), PRX_TEXT_BROWN);
                PRX_DEBUG_COLOR ("H2.imag: " << h1.imag() << " h2.imag: " << h2.imag() << " diff: " << h1.imag() - h2.imag(), PRX_TEXT_BROWN);
                PRX_DEBUG_COLOR("Normed: " << normed << " vs: thresh: " << homotopy_threshold, PRX_TEXT_CYAN);
                if (normed < homotopy_threshold)
                    return true;
                return false;

            }
        }
    }
}
