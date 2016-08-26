/**
 * @file proximity_node.hpp 
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

#ifndef KEB_NODE_PROXIMITY_HPP
#define KEB_NODE_PROXIMITY_HPP

#include "prx/utilities/graph/abstract_node.hpp"
#include "prx/utilities/distance_functions/distance_function.hpp"

#include "prx/utilities/boost/hash.hpp"

namespace prx 
{ 
    namespace util 
    {	

#define INIT_CAP_NEIGHBORS 200

	class abstract_node_t;

        /**
	 * @brief <b> Proximity node for the graph-based distance metric.</b>
	 * 
	 * Proximity node for the graph-based distance metric. 
	 * It wraps around the PRACSYS node, considering it as only storage.
	 * 
	 * @author Kostas Bekris
	 */
	class proximity_node_t
	{
	    public:
	    /**
	     * @brief Constructor
	     * @param st The node to store.
	     */
	    proximity_node_t( const abstract_node_t* st );
	    virtual ~proximity_node_t();

	    /**
	     * Determines distance with another node.
	     * @brief Determines distance with another node.
	     * @param st The node to determine distance with.
	     * @return The distance value.
	     */
	    double distance ( const abstract_node_t* st ); 
        
	    /**
	     * Determines distance with another node.
	     * @brief Determines distance with another node.
	     * @param other The node to determine distance with.
	     * @return The distance value.
	     */
	    double distance ( const proximity_node_t* other );

	    /**
	     * Gets the internal node that is represented.
	     * @brief Gets the internal node that is represented.
	     * @return The internal node.
	     */
	    const abstract_node_t* get_state( );
	    
	    /**
	     * Gets the position of the node in the data structure. Used for fast deletion.
	     * @brief Gets the position of the node in the data structure.
	     * @return The index value.
	     */
	    int get_index();
        
	    /**
	     * Sets the position of the node in the data structure. Used for fast deletion.
	     * @brief Sets the position of the node in the data structure.
	     * @param indx The index value.
	     */
	    void set_index( int indx );
	    
	    /**
	     * Returns the stored neighbors.
	     * @brief Returns the stored neighbors.
	     * @param nr_neigh Storage for the number of neighbors returned.
	     * @return The neighbor indices.
	     */
	    unsigned int* get_neighbors( int* nr_neigh );
	    
	    /**
	     * Adds a node index into this node's neighbor list.
	     * @brief Adds a node index into this node's neighbor list.
	     * @param node The index to add.
	     */
	    void add_neighbor( unsigned int node );
	    
	    /**
	     * Deletes a node index from this node's neighbor list.
	     * @brief Deletes a node index from this node's neighbor list.
	     * @param node The index to delete.
	     */
	    void delete_neighbor( unsigned int node );
	    
	    /**
	     * Replaces a node index from this node's neighbor list.
	     * @brief Replaces a node index from this node's neighbor list.
	     * @param prev The index to look for.
	     * @param new_index The index to replace with.
	     */
	    void replace_neighbor( unsigned prev, int new_index );	    
        
	    /**
	     * @brief The distance function used for this node.
	     */
	    distance_t d;

	    protected:
	    /**
	     * @brief The node represented.
	     */
	    const abstract_node_t* state; 

	    /**
	     * @brief Index in the data structure. Serves as an identifier to other nodes.
	     */
	    int index;

	    /**
	     * @brief The max number of neighbors.
	     */
	    int cap_neighbors;
	    
	    /**
	     * @brief The current number of neighbors.
	     */
	    int nr_neighbors;
	    
	    /**
	     * @brief The neighbor list for this node.
	     */
	    unsigned int* neighbors;
	};

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


    } 
 }

#endif
