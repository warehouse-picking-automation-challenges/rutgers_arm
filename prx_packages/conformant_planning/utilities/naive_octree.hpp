/**
 * @file naive_octree.hpp 
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
#ifndef PRX_NAIVE_OCTREE_HPP
#define PRX_NAIVE_OCTREE_HPP

#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "utilities/particle_space.hpp"

namespace prx 
{ 
    namespace packages 
    {
        namespace conformant
        {
            /**
             * NAIVE OCTREE
             * 
             * @brief <b>  NAIVE OCTREE </b>
             * @author Zakary Littlefield
             */
            class naive_octree_t : public util::distance_metric_t
            {
                public:
                    naive_octree_t( );
                    ~naive_octree_t();

    				virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

                    virtual void link_space( const util::space_t* inspace );
                    /**
                     * @copydoc distance_metric_t::add_point( const abstract_node_t* )
                     */
                    unsigned add_point( const util::abstract_node_t* embed );
                    
                    /**
                     * @copydoc distance_metric_t::add_points( const std::vector< const abstract_node_t* >& )
                     */
                    unsigned add_points( const std::vector< const util::abstract_node_t* >& embeds );
                    
                    /**
                     * @copydoc distance_metric_t::remove_point( const abstract_node_t* )
                     */
                    void remove_point( const util::abstract_node_t* embed );
                    
                    /**
                     * @copydoc distance_metric_t::multi_query( const space_point_t*, unsigned ) const
                     */
                    const std::vector< const util::abstract_node_t* > multi_query( const util::space_point_t* query_point, unsigned ink ) const;
                    
                    /**
                     * @copydoc distance_metric_t::radius_query( const space_point_t*, double ) const
                     */
                    const std::vector< const util::abstract_node_t* > radius_query( const util::space_point_t* query_point, double rad ) const;
                    
                     
                    const std::vector< const util::abstract_node_t* > radius_and_closest_query( const util::space_point_t* query_point, double rad, const util::abstract_node_t*& closest )const;
                    
                    /**
                     * @copydoc distance_metric_t::single_query( const space_point_t*,double* ) const
                     */        
                    const util::abstract_node_t* single_query( const util::space_point_t* query_point, double* dist = NULL ) const;
                    
                    
                    /**
                     * @copydoc distance_metric_t::radius_and_closest_query( const space_point_t*, double, std::vector<const abstract_node_t*>& ) const 
                     */  
                    unsigned radius_and_closest_query( const util::space_point_t* query_point, double rad, std::vector<const util::abstract_node_t*>& closest ) const ;
                    
                    /**
                     * @copydoc distance_metric_t::radius_query( const space_point_t*, double, std::vector<const abstract_node_t*>& ) const 
                     */          
                    unsigned radius_query( const util::space_point_t* query_point, double rad, std::vector<const util::abstract_node_t*>& closest ) const ;

                    /**
                     * @copydoc distance_metric_t::clear()
                     */    
                    void clear( );
                    
                    /**
                     * Not necessary for linear_distance_metric_t. Empty implementation.
                     * @brief Not necessary for linear_distance_metric_t. Empty implementation.
                     */
                    void rebuild_data_structure( );

                protected:

                    const util::particle_space_t* particle_space;

                	double kostas_distance(const util::space_point_t* p1, const util::space_point_t* p2) const;

                    std::vector<const util::abstract_node_t*> points;

                    double threshold;
            };
        }
    } 
}

#endif 