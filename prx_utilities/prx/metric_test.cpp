/**
 * @file param_dump.cpp
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

#include "prx/utilities/distance_metrics/linear_metric/linear_distance_metric.hpp"
#include "prx/utilities/distance_metrics/graph_metric/graph_metric.hpp"
#include "prx/utilities/distance_metrics/ann_metric/ann_distance_metric.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <boost/assign/list_of.hpp>

using namespace prx::util;
using namespace std;

//////////////////////////////////////////////////////
// POINT ADDITIONs
//////////////////////////////////////////////////////
void point_addition( space_t* state_space, linear_distance_metric_t* linear, std::vector<const abstract_node_t*> nodes_linear, graph_distance_metric_t* graph, std::vector<const abstract_node_t*> nodes_graphs, graph_distance_metric_t* graph_2, std::vector<const abstract_node_t*> nodes_graphs_2 )
{
    sys_clock_t clock;
    clock.reset();

    cout << "=====" <<endl;
    for(int i=0; i<10000; i++ )
        linear->add_point(nodes_linear[i]);
    cout << "Cost of adding points in linear data structure " << clock.measure_reset() << endl;

    for(int i=0; i<10000; i++ )
        graph->add_point(nodes_graphs[i]);
    cout << "Cost of adding points in graph data structure 1 " << clock.measure_reset()  << endl;

    graph_2->add_points(nodes_graphs_2);
    cout << "Cost of adding points in graph data structure 2 " << clock.measure_reset()  << endl;

    cout << "=====" <<endl;
}

//////////////////////////////////////////////////////
// POINT REMOVAL
//////////////////////////////////////////////////////
void point_removal( linear_distance_metric_t* linear, std::vector<const abstract_node_t*> nodes_linear, graph_distance_metric_t* graph, std::vector<const abstract_node_t*> nodes_graphs, graph_distance_metric_t* graph_2, std::vector<const abstract_node_t*> nodes_graphs_2 )
{
    std::cout<<"Removing points..."<<endl;
    for( int i=0; i<10000; i+=100)
    {
        linear->remove_point(nodes_linear[i]);
        graph->remove_point(nodes_graphs[i]);
        graph_2->remove_point(nodes_graphs_2[i]);
    }
    std::cout<<"Points removed..."<<endl;
    cout << "=====" <<endl;
}

//////////////////////////////////////////////////////
// SINGLE QUERY
//////////////////////////////////////////////////////
void test_single_query( space_t* state_space, linear_distance_metric_t* linear, graph_distance_metric_t* graph, graph_distance_metric_t* graph_2 )
{
    cout<<"Testing single query..."<<endl;

    std::vector<space_point_t*> query_points;
    query_points.resize(5000);
    for(int i=0; i<5000; i++ )
    {
	query_points[i] = state_space->alloc_point();
	state_space->uniform_sample(query_points[i]);
    }

    std::vector<const abstract_node_t*> linear_results;
    linear_results.resize(5000);
    unsigned linear_single_count = 5000;

    std::vector<const abstract_node_t*> graph_results;
    graph_results.resize(5000);
    unsigned graph_single_count = 0;

    std::vector<const abstract_node_t*> graph_2_results;
    graph_2_results.resize(5000);
    unsigned graph_2_single_count = 0;

    sys_clock_t  clock;
    clock.reset();
    for( int i=0; i<5000; i++ )
	linear_results[i] = linear->single_query( query_points[i] );
    cout << "Cost of 5000 single queries in linear data structure " << clock.measure_reset() << endl;

    clock.reset();
    for( int i=0; i<5000; i++ )
	graph_results[i] = graph->single_query( query_points[i] );
    cout << "Cost of 5000 single queries in graph data structure " << clock.measure_reset() << endl;

    clock.reset();
    for( int i=0; i<5000; i++ )
	graph_2_results[i] = graph_2->single_query( query_points[i] );
    cout << "Cost of 5000 single queries in graph 2 data structure " << clock.measure_reset() << endl;


    for( int i=0; i<5000; i++ )
    {
	if(linear_results[i]->point == graph_results[i]->point )
	    graph_single_count++;
	else
	{
/*
	    cout<<"Graph failed for linear result: "  <<  state_space->print_point( linear_results[i]->point );
	    cout << endl;
	    cout<<"For query point: "<< state_space->print_point( query_points[i] );
	    cout << endl;
	    cout<<"Graph result: " << state_space->print_point( graph_results[i]->point );
	    cout << endl;
	    cout<<"Graph 2 result: " <<  state_space->print_point( graph_2_results[i]->point );
	    cout << endl;
*/
	}

	if(linear_results[i]->point == graph_2_results[i]->point )
	    graph_2_single_count++;
	else
	{
/*
	    cout<<"Graph 2 failed for linear result: " <<  state_space->print_point( linear_results[i]->point ) <<endl;
	    cout << endl;
	    cout<<"For query point: "<<  state_space->print_point( query_points[i] ) << endl;
	    cout << endl;
	    cout<<"Graph result: " << state_space->print_point( graph_results[i]->point )  << endl;
	    cout << endl;
	    cout<<"Graph 2 result: " <<  state_space->print_point( graph_2_results[i]->point ) << endl;
	    cout << endl;
*/
	}
    }

    cout<<"Single Queries Success Statistics:"<<endl;
    cout<<"Attempts: "<<linear_single_count<<"\tGraph Successes: "<<graph_single_count<<"\tGraph 2 Successes: "<<graph_2_single_count<<endl;
    cout << "=====" <<endl;
}

//////////////////////////////////////////////////////
// RADIUS QUERY
//////////////////////////////////////////////////////
void test_range_query( space_t* state_space, linear_distance_metric_t* linear, graph_distance_metric_t* graph, graph_distance_metric_t* graph_2 )
{
    cout<<"Testing range query..."<<endl;

    std::vector< space_point_t* > query_points;
    query_points.resize( 5000 );
    for(int i=0; i<5000; i++ )
    {
	query_points[i] = state_space->alloc_point();
	state_space->uniform_sample(query_points[i]);
    }

    std::vector<const abstract_node_t*> linear_set[5000];
    unsigned linear_radius_count = 0;

    std::vector<const abstract_node_t*> graph_set[5000];
    unsigned graph_radius_count = 0;

    std::vector<const abstract_node_t*> graph_2_set[5000];
    unsigned graph_2_radius_count = 0;

    for( int i=0; i<5000; i++ )
    {
	linear_set[i].resize(10);
	graph_set[i].resize(10);
	graph_set[i].resize(10);
    }

    sys_clock_t clock;
    clock.reset();

    for(int i=0;i<5000;i++)
	linear_set[i] = linear->radius_query(query_points[i],10);
    cout << "Cost of 5000 range queries in linear data structure " << clock.measure_reset() << endl;

    for(int i=0;i<5000;i++)
	graph_set[i] = graph->radius_query(query_points[i],10);
    cout << "Cost of 5000 range queries in graph data structure " << clock.measure_reset() << endl;

    for(int i=0;i<5000;i++)
	graph_2_set[i] = graph_2->radius_query(query_points[i],10);
    cout << "Cost of 5000 range queries in graph 2 data structure " << clock.measure_reset() << endl;

    for( int i=0; i<5000; i++ )
    {
	linear_radius_count += linear_set[i].size();

	int graph_size = PRX_MINIMUM(linear_set[i].size(),graph_set[i].size());
	for(unsigned j=0;j<graph_size;j++)
	{
	    if(linear_set[i][j]->point == graph_set[i][j]->point)
		graph_radius_count++;
	}
	
	int graph_2_size = PRX_MINIMUM(linear_set[i].size(),graph_2_set[i].size());
	for(unsigned j=0;j<graph_2_size;j++)
	{
	    if(linear_set[i][j]->point==graph_2_set[i][j]->point)
		graph_2_radius_count++;
	}
    }

    cout<<"Range Queries Success Statistics:"<<endl;
    cout<<"Attempts: "<<linear_radius_count<<"\tGraph Successes: "<<graph_radius_count<<"\tGraph 2 Successes: "<<graph_2_radius_count<<endl;
    cout << "=====" <<endl;
}   

//////////////////////////////////////////////////////
// K QUERY
//////////////////////////////////////////////////////
void test_k_query( space_t* state_space, linear_distance_metric_t* linear, graph_distance_metric_t* graph, graph_distance_metric_t* graph_2 )
{
    cout<<"Testing k query..."<<endl;

    std::vector< space_point_t* > query_points;
    query_points.resize( 5000 );
    for(int i=0; i<5000; i++ )
    {
	query_points[i] = state_space->alloc_point();
	state_space->uniform_sample(query_points[i]);
    }

    std::vector<const abstract_node_t*> linear_set[5000];
    unsigned linear_k_count = 0;

    std::vector<const abstract_node_t*> graph_set[5000];
    unsigned graph_k_count = 0;

    std::vector<const abstract_node_t*> graph_2_set[5000];
    unsigned graph_2_k_count = 0;

    for( int i=0; i<5000; i++ )
    {
	linear_set[i].resize(35);
	graph_set[i].resize(35);
	graph_set[i].resize(35);
    }

    sys_clock_t clock;
    clock.reset();

    for(int i=0;i<5000;i++)
	linear_set[i] = linear->multi_query(query_points[i],35);
    cout << "Cost of 5000 k queries in linear data structure " << clock.measure_reset() << endl;

    for(int i=0;i<5000;i++)
	graph_set[i] = graph->multi_query(query_points[i],35);
    cout << "Cost of 5000 k in graph data structure " << clock.measure_reset() << endl;

    for(int i=0;i<5000;i++)
	graph_2_set[i] = graph_2->multi_query(query_points[i],35);
    cout << "Cost of 5000 k queries in graph 2 data structure " << clock.measure_reset() << endl;

    for( int i=0; i<5000; i++ )
    {
	linear_k_count += linear_set[i].size();

	int graph_size = PRX_MINIMUM(linear_set[i].size(),graph_set[i].size());
	for(unsigned j=0;j<graph_size;j++)
	{
	    if(linear_set[i][j]->point == graph_set[i][j]->point)
		graph_k_count++;
	}
	
	int graph_2_size = PRX_MINIMUM(linear_set[i].size(),graph_2_set[i].size());
	for(unsigned j=0;j<graph_2_size;j++)
	{
	    if(linear_set[i][j]->point==graph_2_set[i][j]->point)
		graph_2_k_count++;
	}
    }

    cout<<"K Queries Success Statistics:"<<endl;
    cout<<"Attempts: "<<linear_k_count<<"\tGraph Successes: "<<graph_k_count<<"\tGraph 2 Successes: "<<graph_2_k_count<<endl;
    cout << "=====" <<endl;
}   

//////////////////////////////////////////////////////
// MAIN
//////////////////////////////////////////////////////
int main(int ac, char* av[])
{
    ros::init(ac, av, "metric_test");
    ros::NodeHandle main_node_handle;
    global_storage = new parameter_storage_t( "" );
    parameter_reader_t reader("metric_test", global_storage);

    double _x,_y,_z;
    std::vector<double*> state_memory = {&_x,&_y,&_z};
    space_t* state_space = new space_t("XYZ", state_memory);
    state_space->get_bounds()[0]->set_bounds(-100, 100);
    state_space->get_bounds()[1]->set_bounds(-100, 100);
    state_space->get_bounds()[2]->set_bounds(-100, 100);

    linear_distance_metric_t* linear = new linear_distance_metric_t();
    graph_distance_metric_t* graph = new graph_distance_metric_t();
    graph_distance_metric_t* graph_2 = new graph_distance_metric_t();

    linear->link_space(state_space);
    graph->link_space(state_space);
    graph_2->link_space(state_space);

    std::vector<const abstract_node_t*> nodes_linear;
    std::vector<const abstract_node_t*> nodes_graphs;
    std::vector<const abstract_node_t*> nodes_graphs_2;
    nodes_linear.resize(10000);
    nodes_graphs.resize(10000);
    nodes_graphs_2.resize(10000);
    cout<<"Populating metrics..."<<endl;
    for( int i=0; i<10000; i++)
    {
	abstract_node_t* the_node_linear =  new abstract_node_t();
	the_node_linear->point =  state_space->alloc_point();
        state_space->uniform_sample(the_node_linear->point);

	abstract_node_t* the_node_graphs =  new abstract_node_t();
	the_node_graphs->point =  state_space->alloc_point();
	the_node_graphs->point = the_node_linear->point;

	abstract_node_t* the_node_graphs_2 =  new abstract_node_t();
	the_node_graphs_2->point =  state_space->alloc_point();
	the_node_graphs_2->point = the_node_linear->point;

        nodes_linear[i] = the_node_linear;
        nodes_graphs[i] = the_node_graphs;
        nodes_graphs_2[i] = the_node_graphs_2;
    }

    // POINT ADDITION
    point_addition( state_space, linear, nodes_linear, graph, nodes_graphs, graph_2, nodes_graphs_2 );

    // TESTING QUERIES
    test_single_query( state_space, linear, graph, graph_2 );
    
    test_range_query( state_space, linear, graph, graph_2 );
    
    test_k_query( state_space, linear, graph, graph_2 );

    // POINT REMOVAL
    point_removal( linear, nodes_linear, graph, nodes_graphs, graph_2, nodes_graphs_2 );

    // TESTING QUERIES
    test_single_query( state_space, linear, graph, graph_2 );
    
    test_range_query( state_space, linear, graph, graph_2 );
    
    test_k_query( state_space, linear, graph, graph_2 );

    exit;
}
