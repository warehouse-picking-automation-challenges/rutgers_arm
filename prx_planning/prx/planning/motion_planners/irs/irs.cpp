/**
 * @file irs.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2011, Board of Regents, NSHE, obo UNR 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Athanasios Krontiris, Andrew Dobson, Andrew Kimmel, Zacharias Psarakis, Zakary Littlefield, James Marble, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/planning/motion_planners/irs/irs_statistics.hpp"
#include "prx/planning/motion_planners/irs/irs.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"

#include <boost/property_map/vector_property_map.hpp>
#include <boost/graph/connected_components.hpp>
#include "prx/utilities/heuristic_search/null_constraints.hpp"
#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS(prx::plan::irs_t, prx::plan::planner_t)


namespace prx
{
    using namespace util;
    using namespace plan;

    namespace plan
    {

        irs_t::irs_t() 
        {
            rejected_edges = 0;
            statistics = new irs_statistics_t();
            sparsify = false;
        }

        irs_t::~irs_t() { }

        void irs_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            PRX_INFO_S("Initializing IRS motion planner ...");
            prm_star_t::init(reader, template_reader);
            stretch_factor = parameters::get_attribute_as<double>("stretch_factor", reader, template_reader);
            sparsify = parameters::get_attribute_as<bool>("sparsify", reader, template_reader);
            clock.reset();
        }

        bool irs_t::deserialize(){
            prm_star_t::deserialize();
            PRX_PRINT("Succesfull Deserialized Roadmap...",PRX_TEXT_MAGENTA);
            if(sparsify)
            {
                int edge_counter=0;
                astar->link_graph(&graph);
                null_constraints_t* nc = new null_constraints_t();
                foreach(undirected_vertex_index_t vi, boost::vertices(graph.graph))
                {
                    boost::graph_traits<undirected_graph_type>::out_edge_iterator ei, ei_end;
                    std::vector<undirected_edge_index_t> edge_vector;
                    for (boost::tie(ei, ei_end) = boost::out_edges(vi, graph.graph); ei != ei_end; ++ei) 
                    {
                        undirected_vertex_index_t source = boost::source ( *ei, graph.graph );
                        undirected_vertex_index_t target = boost::target ( *ei, graph.graph );
                        undirected_edge_index_t e;
                        bool valid;
                        boost::tie(e, valid) = boost::edge(source, target, graph.graph);
                        edge_vector.push_back(e);
                    }

                    foreach(undirected_edge_index_t eind, edge_vector)
                    {
                        undirected_vertex_index_t source=boost::source(eind,graph.graph);
                        undirected_vertex_index_t target=boost::target(eind,graph.graph);
                        astar->setup_astar_search( STANDARD_SEARCH, nc);
                        astar->astar_search_t::block_edge(eind);
                        double dist = get_path_distance(source, target);
                        double new_dist = metric->distance_function(graph[source]->point, graph[target]->point);
                        if(  dist > stretch_factor * new_dist ) //!same_connected_component_in_graph(graph, source_sg, target_sg) ||
                        {
                            PRX_STATUS("Adding Edge "<<"["<<edge_counter<<"]"<<"dist: "<<dist<<" new_dist: "<<new_dist<<"Edge count: "<<boost::num_edges(graph.graph),PRX_TEXT_GREEN);
                            edge_counter++;
                            if( visualize_graph )
                                graph.get_edge_as< prm_star_edge_t >(eind)->path = new_path;
                        }
                        else
                        {
                            graph.graph.remove_edge(eind);
                            ++rejected_edges;
                            PRX_STATUS("Rejecting Edge "<<stretch_factor<<"["<<rejected_edges<<"]"<<"dist: "<<dist<<" new_dist: "<<new_dist<<"Edge count: "<<boost::num_edges(graph.graph),PRX_TEXT_RED);
                        }  
                    }
                }
                PRX_PRINT("Sparse Roadmap Complete... Rejected: "<<rejected_edges<<", "<<" Accepted: "<<boost::num_edges(graph.graph),PRX_TEXT_GREEN);
            }
            return true;
        }

        bool irs_t::same_connected_component_in_graph(const undirected_graph_t& g, const undirected_vertex_index_t v1, const undirected_vertex_index_t v2)
        {
            int num = boost::connected_components(g.graph, g.components);

            if( num == 1 )
                return true;
            if( g.components[v1] == g.components[v2] )
                return true;

            return false;

        }

        bool irs_t::same_connected_component(const undirected_vertex_index_t v1, const undirected_vertex_index_t v2)
        {
            return same_connected_component_in_graph(graph, v1, v2);
        }

        double irs_t::get_path_distance( undirected_vertex_index_t start,  undirected_vertex_index_t goal)
        {
            astar->link_graph(&graph);
            double distance = PRX_INFINITY;
            if( astar->solve(start, goal) )
            {
                //We need to extract the path vertices
                std::deque< undirected_vertex_index_t > path_vertices;
                astar->extract_path( path_vertices );
                distance = 0;
                for(int i=0; i<path_vertices.size()-1; i++)
                {
                    undirected_edge_index_t e = boost::edge(path_vertices[i], path_vertices[i+1], graph.graph).first;
                    distance += graph.get_weight(e);
                }
            }
            return distance;
        }





        void irs_t::link_node_to_neighbors(undirected_vertex_index_t v, const std::vector< const abstract_node_t* >& neighbors)
        {
            //Attempt to connect to the node's nearest neighbors
            const undirected_node_t* node;
            null_constraints_t* nc = new null_constraints_t();
            astar->setup_astar_search( STANDARD_SEARCH, nc);
            for( size_t i = 0; i < neighbors.size(); i++ )
            {
                node = neighbors[i]->as<undirected_node_t > ();

                new_path.clear();
                new_plan.clear();

                local_planner->steer(graph[v]->point, node->point, new_plan, new_path);
                
                //If the path is valid
                if( validity_checker->is_valid(new_path) )
                {
                    double dist = get_path_distance(v, node->index);
                    double new_dist = metric->distance_function(graph[v]->point, node->point);

                    if( !same_connected_component(v, node->index) || dist > stretch_factor * new_dist )
                    {
                        //Add the edge to the graph
                        double dist = metric->distance_function(graph[v]->point, node->point);
                        undirected_edge_index_t e = graph.add_edge< prm_star_edge_t > (v, node->index, new_dist);
                        graph.get_edge_as<prm_star_edge_t > (e)->id = num_edges;
                        graph.get_edge_as<prm_star_edge_t >(e)->constraints = validity_checker->alloc_constraint();
                        num_edges++;
                        if( visualize_graph )
                            graph.get_edge_as< prm_star_edge_t >(e)->path = new_path;
                    }
                    else
                    {
                        ++rejected_edges;
                    }
                }

                new_path.clear();
            }
        }
        
        const statistics_t* irs_t::get_statistics()
        {
            statistics_t* returned_stats = new irs_statistics_t();
            
            returned_stats->as< irs_statistics_t >()->num_vertices = boost::num_vertices(graph.graph);
            returned_stats->as< irs_statistics_t >()->num_edges = boost::num_edges(graph.graph);
            returned_stats->as< irs_statistics_t >()->rejected_edges = rejected_edges;
            returned_stats->as< irs_statistics_t >()->elapsed = clock.measure();
            
            return returned_stats;
        }

    }
}



