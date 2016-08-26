/**
 * @file proximity_node.cpp 
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

#include "prx/utilities/distance_metrics/graph_metric/proximity_node.hpp"
#include <cmath>

namespace prx 
{ 
 namespace util 
 {

proximity_node_t::proximity_node_t( const abstract_node_t* st )
{
    d = NULL;
    state = st;

    neighbors = (unsigned int*)malloc(INIT_CAP_NEIGHBORS*sizeof(unsigned int));
    nr_neighbors = 0;
    cap_neighbors = INIT_CAP_NEIGHBORS;
}

proximity_node_t::~proximity_node_t()
{
    free( neighbors );
}

double proximity_node_t::distance ( const abstract_node_t* st )
{
    return d(state->point,st->point);
}

double proximity_node_t::distance ( const proximity_node_t* other )
{
    return d(state->point,other->state->point);
}

const abstract_node_t* proximity_node_t::get_state( )
{
    return state;
}

int proximity_node_t::get_index()
{
    return index;
}

void proximity_node_t::set_index( int indx )
{
    index = indx;
}

unsigned int* proximity_node_t::get_neighbors( int* nr_neigh )
{
    *nr_neigh = nr_neighbors;
    return neighbors;
}

void proximity_node_t::add_neighbor( unsigned int nd )
{
    if( nr_neighbors >= cap_neighbors-1 )
    {
	cap_neighbors = 2*cap_neighbors;
	neighbors = (unsigned int*)realloc( neighbors, cap_neighbors*sizeof(unsigned int));
    }
    neighbors[nr_neighbors] = nd;
    nr_neighbors++;
}

void proximity_node_t::delete_neighbor( unsigned int nd )
{
    int index;
    for( index=0; index<nr_neighbors; index++ )
    {
	if( neighbors[index] == nd )
	    break;
    }
    PRX_ASSERT( index < nr_neighbors );

    for( int i=index; i<nr_neighbors-1; i++ )
	neighbors[i] = neighbors[i+1];
    nr_neighbors--;
}

void proximity_node_t::replace_neighbor( unsigned prev, int new_index )
{
    int index;
    for( index=0; index<nr_neighbors; index++ )
    {
	if( neighbors[index] == prev )
	    break;
    }
    PRX_ASSERT( index < nr_neighbors );

    neighbors[index] = new_index;
}

 }
}
