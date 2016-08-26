/**
 * @file graph_proximity.hpp 
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

#ifndef KEB_GRAPH_PROXIMITY_HPP
#define KEB_GRAPH_PROXIMITY_HPP

#include "prx/utilities/distance_metrics/graph_metric/proximity_node.hpp"

#include "prx/utilities/boost/hash.hpp"

#include "prx/utilities/definitions/sys_clock.hpp"

namespace prx 
{ 
 namespace util 
 {

#define INIT_NODE_SIZE    1000
#define MAX_KK            2000

     class abstract_node_t;	
     class proximity_node_t;

/**
 * A proximity structure based on graph literature. Each node maintains a list of neighbors.
 * When performing queries, the graph is traversed to determine other locally close nodes.
 * @brief <b> A proximity structure based on graph literature. </b>
 * @author Kostas Bekris
 */
    class graph_proximity_t
    {

        public:
        /**
	 * @brief Constructor
	 * @param state The first node to add to the structure.
	 */
	graph_proximity_t( abstract_node_t* state );
	~graph_proximity_t();
	 
        /**
	 * Adds a node to the proximity structure
	 * @brief Adds a node to the proximity structure
	 * @param node The node to insert.
	 */
	void add_node( proximity_node_t* node );
	
        /**
	 * Adds a node to the proximity structure
	 * @brief Adds a node to the proximity structure
	 * @param node The node to insert.
	 */
	void add_nodes( proximity_node_t** nodes, int nr_nodes );
	    
	/**
	 * @brief Removes a node from the structure.
         * @param node
         */
	void remove_node( proximity_node_t* node );
	    
	/**
	 * Prints the average degree of all vertices in the data structure.
         * @brief Prints the average degree of all vertices in the data structure.
         */
        void average_valence();

        /**
         * Returns the closest node in the data structure.
         * @brief Returns the closest node in the data structure.
         * @param state The query point.
         * @param distance The resulting distance between the closest point and the query point.
         * @return The closest point.
         */
        proximity_node_t* find_closest( abstract_node_t* state, double* distance );          
        
        /**
         * Find the k closest nodes to the query point. 
         * @brief Find the k closest nodes to the query point.
         * @param state The query state.
         * @param close_nodes The returned close nodes.
         * @param distances The corresponding distances to the query point.
         * @param k The number to return.
         * @return The number of nodes actually returned.
         */
        int find_k_close( abstract_node_t* state, proximity_node_t** close_nodes, double* distances, int k );
        
        /**
         * Find all nodes within a radius and the closest node. 
         * @brief Find all nodes within a radius and the closest node.
         * @param state The query state.
         * @param close_nodes The returned close nodes.
         * @param distances The corresponding distances to the query point.
         * @param delta The radius to search within.
         * @return The number of nodes returned.
         */
        int find_delta_close_and_closest( abstract_node_t* state, proximity_node_t** close_nodes, double* distances, double delta );
        
        /**
         * Find all nodes within a radius. 
         * @brief Find all nodes within a radius.
         * @param state The query state.
         * @param close_nodes The returned close nodes.
         * @param distances The corresponding distances to the query point.
         * @param delta The radius to search within.
         * @return The number of nodes returned.
         */
        int find_delta_close( abstract_node_t* state, proximity_node_t** close_nodes, double* distances, double delta );

        protected:
        /**
	 * Sorts a list of proximity_node_t's. Performed using a quick sort operation. 
	 * @param close_nodes The list to sort.
	 * @param distances The distances that determine the ordering.
	 * @param low The lower index.
	 * @param high The upper index.
	 */
	void sort_proximity_nodes( proximity_node_t** close_nodes, double* distances, int low, int high );
	
        /**
	 * Performs sorting over a list of nodes. Assumes all nodes before index are sorted.
	 * @param close_nodes The list to sort.
	 * @param distances The distances that determine the ordering.
	 * @param index The index to start from.
	 */
	int resort_proximity_nodes( proximity_node_t** close_nodes, double* distances, int index );

        /**
         * Helper function for determining existance in a list.
         * @brief Helper function for determining existance in a list.
         * @param query_node The node to search for.
         * @param node_list The list to search.
         * @param list_size The size of the list.
         * @return If query_node exists in node_list.
         */
        inline bool does_node_exist( proximity_node_t* query_node )
	{
	    return (added_nodes.find(query_node) != added_nodes.end() );
	}

        /**
         * Determine the number of nodes to sample for initial populations in queries.
         * @brief Determine the number of nodes to sample for initial populations in queries.
         * @return The number of random nodes to initially select.
         */
	inline int sampling_function()
	{
	    if( nr_nodes < 500 )
		return nr_nodes/5 + 1;
	    else
		return 100 + nr_nodes/500;
	}
        
        /**
         * Given the number of nodes, get the number of neighbors required for connectivity (in the limit).
         * @brief Given the number of nodes, get the number of neighbors required for connectivity (in the limit).
         * @return 
         */
	inline int percolation_threshold()
	{
	    if( nr_nodes > 12)
		return( 3.5 * log( nr_nodes ));
	    else 
		return nr_nodes;
	}


        /**
         * The basic search process for finding the closest node to the query state.
         * @brief Find the closest node to the query state.
         * @param state The query state.
         * @param distance The corresponding distance to the query point.
         * @param node_index The index of the returned node.
         * @return The closest node.
         */
        proximity_node_t* basic_closest_search( abstract_node_t* state, double* distances, int* node_index );
        
        /**
         * @brief The nodes being stored.
         */
        proximity_node_t** nodes;
        
        /**
         * @brief The current number of nodes being stored.
         */
        int nr_nodes;
        
        /**
         * @brief The maximum number of nodes that can be stored. 
         */
        int cap_nodes;

        /**
         * @brief Temporary storage for query functions.
         */
        proximity_node_t** second_nodes;
        
        /**
         * @brief Temporary storage for query functions.
         */
        double* second_distances;

        hash_t<proximity_node_t*,int> added_nodes;
    };

 } 
}



#endif
