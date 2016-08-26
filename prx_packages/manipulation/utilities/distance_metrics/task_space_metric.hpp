/**
 * @file distance_metric.hpp 
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
#pragma once
#include "prx/utilities/distance_metrics/distance_metric.hpp"

#ifndef PRX_TASK_SPACE_METRIC_HPP
#define PRX_TASK_SPACE_METRIC_HPP

namespace prx 
{ 
    namespace util 
    {
        class parameter_reader_t;

        /**
         * A hybrid metric which reasons about configuration and work spaces
         * @brief <b> An encapsulation of a distance function and a nearest neighbor query structure. </b>
         * @authors Andrew Kimmel, Rahul Shome
         */
        class task_space_metric_t : public distance_metric_t
        {
            
          public:
            task_space_metric_t( );
            virtual ~task_space_metric_t();
            /**
             * Initializes parameters of the distance metric.
             * @brief Initializes parameters of the distance metric.
             * @param reader The priority reader for reading parameters. If a parameters exists here, it will be used.
             * @param template_reader The backup reader. Any necessary parameters not in reader will be found here.
             */
            virtual void init(const parameter_reader_t * reader, const parameter_reader_t* template_reader = NULL);

            
            /**
             * Informs the metric about the type of points it is using.
             * @brief Informs the metric about the type of points it is using.
             * @param inspace The space to perform queries in.
             */
            virtual void link_space( const space_t* inspace );
            
            /**
             * Adds a node to the nearest neighbor structure.
             * @brief Adds a node to the nearest neighbor structure.
             * @param embed The node to add.
             * @return The number of points in total.
             */
            virtual unsigned add_point( const abstract_node_t* embed );
            
            /**
             * Adds multiple nodes to the nearest neighbor structure.
             * @brief Adds multiple nodes to the nearest neighbor structure.
             * @param embeds The nodes to add.
             * @return The number of points in total.
             */
            virtual unsigned add_points( const std::vector< const abstract_node_t* >& embeds );
            
            /**
             * Removes a node for the nearest neighbor structure.
             * @brief Removes a node for the nearest neighbor structure.
             * @param node The node that should be removed.
             */
            virtual void remove_point( const abstract_node_t* node );
            
            /**
             * Performs a query for multiple nodes.
             * @brief Performs a query for multiple nodes.
             * @param query_point The point to query around.
             * @param ink The number of closest points to find.
             * @return A vector containing the nodes that are closest to the query point.
             */
            virtual const std::vector< const abstract_node_t* > multi_query( const space_point_t* query_point, unsigned ink ) const;
            
            /**
             * Performs a query for multiple nodes within a radius.
             * @brief Performs a query for multiple nodes within a radius.
             * @param query_point The point to query around.
             * @param rad The radius around the query point to search.
             * @return A vector containing all nodes within the radius around the query point.
             */
            virtual const std::vector< const abstract_node_t* > radius_query( const space_point_t* query_point, double rad )const;
            
            /**
             * Query for the single closest point.
             * @brief Query for the single closest point.
             * @param query_point The point to query around.
             * @return The closest node to the query point.
             */
            virtual const abstract_node_t* single_query( const space_point_t* query_point, double* dist = NULL ) const;
            
            /**
             * Clears the data structure of all data.
             * @brief Clears the data structure of all data.
             */
            virtual void clear( );
            
            /**
             * Restructures internal data representations. Necessary for certain internal data representations, but not all.
             * @brief Restructures internal data representations.
             */
            virtual void rebuild_data_structure( );
            

            /**
             * Performs a query for multiple nodes within a radius and the closest node. This version requires an allocated vector.
             * @brief Performs a query for multiple nodes within a radius and the closest node.
             * @param query_point The node to query around.
             * @param rad The radius around the query point to search.
             * @param closest A vector that will be populated with all nodes within the radius around the query point.
             * @return The number of points in the vector that are valid.
             */
            virtual unsigned radius_and_closest_query( const space_point_t* query_point, double rad, std::vector<const abstract_node_t*>& closest ) const ;

            /**
             * Performs a query for multiple nodes within a radius. This version requires an allocated vector.
             * @brief Performs a query for multiple nodes within a radius.
             * @param query_point The node to query around.
             * @param rad The radius around the query point to search.
             * @param A vector that will be populated with all nodes within the radius around the query point.
             * @return The number of points in the vector that are valid.
             */
            virtual unsigned radius_query( const space_point_t* query_point, double rad, std::vector<const abstract_node_t*>& closest ) const ;
            

            virtual const std::vector< const abstract_node_t* > multi_query( const abstract_node_t* query_point, unsigned ink ) const;

            virtual const std::vector< const abstract_node_t* > radius_query( const abstract_node_t* query_point, const double rad ) const;

            virtual const abstract_node_t* single_query( const abstract_node_t* query_point, double* dist = NULL ) const;


            /**
             * Prints important information about the distance metric.
             * @brief Prints important information about the distance metric.
             */
            virtual void print();
            
            /**
             * Determines if the metric contains a node that is equivalent to the argument.
             * @brief Determines if the metric contains a node that is equivalent to the argument.
             */
            virtual bool has_point( const abstract_node_t* embed );

            /**
             * Get the current size of the nearest neighbor structure.
             * @brief Get the current size of the nearest neighbor structure.
             * @return The number of points stored.
             */
            unsigned get_nr_points( );

            bool use_workspace_metric;

          protected:

            space_t* end_effector_space;
            distance_metric_t* workspace_metric, *cspace_metric;
            // Used during serialization and deserialization 
            double _x, _y, _z, _qx, _qy, _qz, _qw;

        };

    } 
}

#endif  

