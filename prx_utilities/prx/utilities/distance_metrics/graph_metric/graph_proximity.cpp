/**
 * @file graph_proximity.cpp 
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

#include "prx/utilities/distance_metrics/graph_metric/graph_proximity.hpp"
#include <cmath>

using namespace std;

namespace prx 
{ 
 namespace util 
 {

graph_proximity_t::graph_proximity_t( abstract_node_t* state )
{
    added_nodes.clear();

    nodes = (proximity_node_t**)malloc(INIT_NODE_SIZE*sizeof(proximity_node_t*));
    nr_nodes = 0;
    cap_nodes = INIT_NODE_SIZE;

    second_nodes = (proximity_node_t**)malloc( MAX_KK *sizeof(proximity_node_t*));
    second_distances = (double*)malloc( MAX_KK *sizeof(double));
}

graph_proximity_t::~graph_proximity_t()
{
    free( nodes );

    free( second_nodes );
    free( second_distances);
}

void graph_proximity_t::average_valence()
{
    int nr_neigh;
    unsigned int* neighs;
    double all_neighs = 0;
    for( int i=0; i<nr_nodes; i++ )
    {
	neighs = nodes[i]->get_neighbors( &nr_neigh );
	all_neighs += nr_neigh;
    }
    all_neighs /= (double)nr_nodes;
}

void graph_proximity_t::add_node( proximity_node_t* graph_node )
{    
    int k = percolation_threshold();

    int new_k = find_k_close( (abstract_node_t*)(graph_node->get_state()), second_nodes, second_distances, k );

    if( nr_nodes >= cap_nodes-1 )
    {
	cap_nodes = 2 * cap_nodes;
	nodes = (proximity_node_t**)realloc(nodes, cap_nodes*sizeof(proximity_node_t*));
    }
    nodes[nr_nodes] = graph_node;
    
    graph_node->set_index(nr_nodes);
    nr_nodes++;

    for( int i=0; i<new_k; i++ )
    {
	graph_node->add_neighbor( second_nodes[i]->get_index() );
	second_nodes[i]->add_neighbor( graph_node->get_index() );
    }
}

void graph_proximity_t::add_nodes( proximity_node_t** graph_nodes, int nr_new_nodes)
{
    if( nr_nodes + nr_new_nodes >= cap_nodes - 1 )
    {
	cap_nodes = nr_nodes + nr_new_nodes + 10; 
	nodes = (proximity_node_t**)realloc(nodes, cap_nodes*sizeof(proximity_node_t*));
    }

    for( int i=0; i<nr_new_nodes; i++ )
    {
	int k = percolation_threshold();

	int new_k = find_k_close( (abstract_node_t*)(graph_nodes[i]->get_state()), second_nodes, second_distances, k );

	nodes[nr_nodes] = graph_nodes[i]; 
	graph_nodes[i]->set_index(nr_nodes);
	nr_nodes++;

	for( int j=0; j<new_k; j++ )
	{
	    graph_nodes[i]->add_neighbor( second_nodes[j]->get_index() );
	    second_nodes[j]->add_neighbor( graph_nodes[i]->get_index() );
	}
    }
}
 
void graph_proximity_t::remove_node( proximity_node_t* graph_node )
{
    int nr_neighbors;
    unsigned int* neighbors = graph_node->get_neighbors( &nr_neighbors );
    for( int i=0; i<nr_neighbors; i++ )
	nodes[ neighbors[i] ]->delete_neighbor( graph_node->get_index() );

    int index = graph_node->get_index();
    if( index < nr_nodes-1 )
    {
	nodes[index] = nodes[nr_nodes-1];
	nodes[index]->set_index( index );
	
	neighbors = nodes[index]->get_neighbors( &nr_neighbors );
	for( int i=0; i<nr_neighbors; i++ )
	    nodes[ neighbors[i] ]->replace_neighbor( nr_nodes-1, index ); 
    }
    nr_nodes--;

    if( nr_nodes < (cap_nodes-1)/2 )
    {
	cap_nodes *= 0.5;
	nodes = (proximity_node_t**)realloc(nodes,cap_nodes*sizeof(proximity_node_t*));
    }
}

proximity_node_t* graph_proximity_t::find_closest( abstract_node_t* state, double* the_distance )
{
    int min_index = -1;
    return basic_closest_search( state, the_distance, &min_index );
}     
     
int graph_proximity_t::find_k_close( abstract_node_t* state, proximity_node_t** close_nodes, double* distances, int k )
{
    if( nr_nodes == 0 )
        return 0;
    
    if(k > MAX_KK)
    {
	PRX_WARN_S("Trying to return " << k << " points when the max is "<<MAX_KK);
	k = MAX_KK;
    }
    else if( k >= nr_nodes )
    {
	for( int i=0; i<nr_nodes; i++ )
	{
	    close_nodes[i] = nodes[i];
	    distances[i] = nodes[i]->distance( state );
	}        
	sort_proximity_nodes( close_nodes, distances, 0, nr_nodes-1 );
	return nr_nodes;
    }

    added_nodes.clear();
   
    int min_index = -1;
    close_nodes[0] = basic_closest_search( state, &(distances[0]), &min_index );
    added_nodes[nodes[min_index]] = true;

    min_index = 0;
    int nr_elements = 1;
    double max_distance = distances[0];

    /* Find the neighbors of the closest node if they are not already in the set of k-closest nodes.
       If the distance to any of the neighbors is less than the distance to the k-th closest element,
       then replace the last element with the neighbor and resort the list. In order to decide the next
       node to pivot about, it is either the next node on the list of k-closest
     */
    do
    {
	int nr_neighbors;
	unsigned int* neighbors = nodes[ close_nodes[min_index]->get_index() ]->get_neighbors( &nr_neighbors );
	int lowest_replacement = nr_elements;
 
	for( int j=0; j<nr_neighbors; j++ )
	{
	    proximity_node_t* the_neighbor = nodes[neighbors[j]];
	    if( does_node_exist( the_neighbor ) == false )
	    {
		added_nodes[ the_neighbor ] = true;

		double distance = the_neighbor->distance( state );
		bool to_resort = false;
		if( nr_elements < k )
		{
		    close_nodes[nr_elements] = the_neighbor;
		    distances[nr_elements] = distance;
		    nr_elements++;
		    to_resort = true;
		}
		else if( distance < distances[k-1] )
		{
		    close_nodes[k-1] = the_neighbor;
		    distances[k-1] = distance;
		    to_resort = true;
		}

		if( to_resort )
		{
		    int test = resort_proximity_nodes( close_nodes, distances, nr_elements-1 );
		    lowest_replacement = (test<lowest_replacement?test:lowest_replacement);
		}
	    }
	}

	/* In order to decide the next node to pivot about, 
	   it is either the next node on the list of k-closest (min_index)
	   or one of the new neighbors in the case that it is closer than nodes already checked.
	*/
	if(min_index < lowest_replacement)
	    min_index++;
	else
	    min_index = lowest_replacement;
    }
    while( min_index < nr_elements );

    return nr_elements;
}

int graph_proximity_t::find_delta_close_and_closest( abstract_node_t* state, proximity_node_t** close_nodes, double* distances, double delta )
{
    if( nr_nodes == 0 )
        return 0;
    
    added_nodes.clear();
   
    int min_index = -1;
    close_nodes[0] = basic_closest_search( state, &(distances[0]), &min_index );

    if( distances[0] > delta )
	return 1;

    added_nodes[nodes[min_index]] = true;
    
    int nr_points = 1;
    for( int counter = 0; counter<nr_points; counter++ )
    {
	int nr_neighbors;
	unsigned int* neighbors = close_nodes[counter]->get_neighbors( &nr_neighbors );	
	for( int j=0; j<nr_neighbors; j++ )
	{
	    proximity_node_t* the_neighbor = nodes[neighbors[j]];
	    if( does_node_exist( the_neighbor ) == false )
	    {
		added_nodes[the_neighbor] = true;
		double distance = the_neighbor->distance( state );
		if( distance < delta && nr_points < MAX_KK)
		{
		    close_nodes[ nr_points ] = the_neighbor;
		    distances[ nr_points ] = distance;
		    nr_points++;
		}
	    }
	}
    }

    if( nr_points > 0 )
	sort_proximity_nodes( close_nodes, distances, 0, nr_points - 1 );

    return nr_points;
}
     
int graph_proximity_t::find_delta_close( abstract_node_t* state, proximity_node_t** close_nodes, double* distances, double delta )
{
    if( nr_nodes == 0 )
        return 0;
    
    added_nodes.clear();
   
    int min_index = -1;
    close_nodes[0] = basic_closest_search( state, &(distances[0]), &min_index );

    if( distances[0] > delta )
	return 0;

    added_nodes[nodes[min_index]] = true;
    
    int nr_points = 1;	
    for( int counter = 0; counter<nr_points; counter++ )
    {
	int nr_neighbors;
	unsigned int* neighbors = close_nodes[counter]->get_neighbors( &nr_neighbors );	
	for( int j=0; j<nr_neighbors; j++ )
	{
	    proximity_node_t* the_neighbor = nodes[neighbors[j]];
	    if( does_node_exist( the_neighbor ) == false )
	    {
		added_nodes[ the_neighbor ] = true;
		double distance = the_neighbor->distance( state );
		if( distance < delta && nr_points < MAX_KK)
		{
		    close_nodes[ nr_points ] = the_neighbor;
		    distances[ nr_points ] = distance;
		    nr_points++;
		}
	    }
	}
    }

    if( nr_points > 0 )
	sort_proximity_nodes( close_nodes, distances, 0, nr_points - 1 );

    return nr_points;
}

////////////////////////////////////////
////// SORTING
///////////////////////////////////////

void graph_proximity_t::sort_proximity_nodes( proximity_node_t** close_nodes, double* distances, int low, int high )
{ 
    if( low < high )
    {
	int left, right, pivot;
	double pivot_distance = distances[low];
	proximity_node_t* pivot_node = close_nodes[low];

	double temp;
	proximity_node_t* temp_node;

	pivot = left = low;
	right = high;
	while( left < right )
	{
	   while( left <= high && distances[left] <= pivot_distance )
		left++;
	   while( distances[right] > pivot_distance )
		right--;
	   if( left < right )
	   {
		temp = distances[left];
		distances[left] = distances[right];
		distances[right] = temp;

		temp_node = close_nodes[left];
		close_nodes[left] = close_nodes[right];
		close_nodes[right] = temp_node;
	   }
	}
	distances[low] = distances[right];
	distances[right] = pivot_distance;

	close_nodes[low] = close_nodes[right];
	close_nodes[right] = pivot_node;

	sort_proximity_nodes( close_nodes, distances, low, right-1 );
	sort_proximity_nodes( close_nodes, distances, right+1, high );
    }
}

int graph_proximity_t::resort_proximity_nodes( proximity_node_t** close_nodes, double* distances, int index )
{
    double temp;
    proximity_node_t* temp_node;

    while( index > 0 && distances[ index ] < distances[ index-1 ] )
    {
	temp = distances[index];
	distances[index] = distances[index-1];
	distances[index-1] = temp;
	
	temp_node = close_nodes[index];
	close_nodes[index] = close_nodes[index-1];
	close_nodes[index-1] = temp_node;

	index--;
    }
    return index;
}
          
     proximity_node_t* graph_proximity_t::basic_closest_search( abstract_node_t* state, double* the_distance, int* the_index )
{
    if( nr_nodes == 0 )
        return NULL;
    
    int nr_samples = sampling_function();
    double min_distance = std::numeric_limits<double>::max();
    int min_index = -1;
    for( int i=0; i<nr_samples; i++ )
    {
	int index = rand() % nr_nodes;
	double distance = nodes[index]->distance( state );
	if( distance < min_distance )
	{
	    min_distance = distance;
	    min_index = index;
	}
    }

    int old_min_index = min_index;
    do
    {
	old_min_index = min_index;
	int nr_neighbors;
	unsigned int* neighbors = nodes[min_index]->get_neighbors( &nr_neighbors );
	for( int j=0; j<nr_neighbors; j++ )
	{
	    double distance = nodes[ neighbors[j] ]->distance( state );
	    if( distance < min_distance )
	    {
		min_distance = distance;
		min_index = neighbors[j];			
	    }
	}
    }
    while( old_min_index != min_index );

    *the_distance = min_distance;
    *the_index = min_index;
    return nodes[min_index];
}     


 } 
}
