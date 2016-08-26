/**
 * @file 2d_proximity_sensing_info.hpp
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

#ifndef PRX_TWOD_PROX_SENSING_INFO_HPP
#define	PRX_TWOD_PROX_SENSING_INFO_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/sensing/sensing_info.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/math/3d_geometry/geometry.hpp"

#include "prx/utilities/distance_metrics/distance_metric.hpp"

namespace prx
{
    namespace util
    {
        class geometry_t;
        class space_t;
        class space_point_t;
        class distance_metric_t;
        class ramp_t; 

        class proximity_query_node_t : public abstract_node_t
        {
          public:
            proximity_query_node_t()
            {

            }

            ~proximity_query_node_t()
            {

            }

            bool operator<( const proximity_query_node_t& other ) const
            {
                return sort_value < other.sort_value;
            }

            unsigned obstacle_index;
            double sort_value;
        };
    }

    namespace sim
    {
        class config_sensor_t;
        class geometry_sensor_t;

        struct prox_elem_t
        {
            public:
                prox_elem_t() {}

                prox_elem_t(const std::string& name, const util::config_t& conf, double dist, util::geometry_t* geo)
                {
                    config_names.push_back( name );
                    configs.push_back( conf );
                    distances.push_back( dist );
                    geometries.push_back( geo );
                }

                prox_elem_t(const std::vector< std::string >& names, const std::vector< util::config_t >& confs, const std::vector< double >& dists, const std::vector< util::geometry_t* >& geos)
                {
                    config_names = names;
                    configs = confs;
                    distances = dists;
                    geometries = geos;
                }

                std::vector< std::string > config_names;
                std::vector< util::config_t > configs;
                std::vector< double > distances;
                std::vector< util::geometry_t* > geometries;
        };

        inline bool operator< (const prox_elem_t& lhs, const prox_elem_t& rhs)
        {
            return lhs.distances[0] < rhs.distances[0];
        }

        typedef std::vector<prox_elem_t> proximity_list_t;

        /**
         * This class generates two-dimensional proximity information for a specific
         * system_path. The proximity query can be bounded by bother maximum number
         * of nearest neighbors and a 2D sensing radius. The following is assumed:
         *
         * 1. The geometry of your system_path is nicely bounded by a circle.
         * 2. Operates over uniformly scaled Euclidean distances.
         *
         * The output of this sensing info gives a list of pairs of sensed paths
         * and their corresponding distances.
         *
         * @brief Generates two dimensional proximity information
         *
         *
         * @author Andrew Kimmel
         */
        class twoD_proximity_sensing_info_t : public sim::sensing_info_t
        {
        protected:
            /** @brief Determines the perspective of the sensing info*/
            std::string system_path;
            util::config_t system_config;
            util::geometry_t* system_geometry;
            /** @brief Determines the maximum size of proximity list*/
            unsigned max_num_neighbors;
            /** @brief Determines the sensing radius (max allowed distance) */
            double sensing_radius;
            double obstacle_sensing_radius;
            double closest_plant_distance;
            double closest_obstacle_distance;
            util::ramp_t* closest_ramp;

            /** Sensor Pointers */
            sim::config_sensor_t* conf_sensor;
            sim::geometry_sensor_t* geom_sensor;

            /** Sensing Info's Info */
            proximity_list_t prox_list;
            util::config_list_t obstacle_configs;

            std::vector< util::geometry_t > plant_geoms;
            std::vector< util::geometry_t > obstacle_geoms;

            util::space_t* navigation_space;
            util::space_point_t* agent_state;

            unsigned system_index;

            double _Nx;
            double _Ny;
            double _Nz;

            bool static_configs_retrieved;

            util::distance_metric_t* metric;

            std::vector< util::proximity_query_node_t > pq_nodes;

            std::vector<unsigned> sensed_obstacle_indices;
            std::vector<double> sensed_obstacle_distances;

            util::config_t transform_config;

            void compute_static_obstacle_vertices();

            bool find_system_configuration_index( const util::config_list_t& plant_configs );

        public:

            twoD_proximity_sensing_info_t() ;
            virtual ~twoD_proximity_sensing_info_t();

            /**
             * Initializes from the given parameters.
             *
             * @brief Initializes from the given parameters.
             *
             * @param reader A \ref util::parameter_reader_t with a dictionary of parameters for this system.
             * @param template_reader A template \ref util::parameter_reader_t with a dictionary of parameters.
             * Default value for the template reader is NULL. The priority goes to primary reader \c reader, if something
             * is not specified in the \c reader then it will be read from the \c template_reader
             */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            virtual void set_sensors(const std::vector<sim::sensor_t*>& sensors);

            // The conversion function to turn sensing data into the controller's info
            virtual void update_info();
            // Used to access the info?
            virtual void get_info();
            virtual double get_closest_plant_distance();
            virtual double get_closest_obstacle_distance();

            virtual proximity_list_t get_proximity_list();
            virtual unsigned get_max_num_neighbors();
            virtual double get_sensing_radius();
            virtual double get_obstacle_sensing_radius();
            virtual void set_system_path(const std::string& path);

        };

    }
}

#endif

