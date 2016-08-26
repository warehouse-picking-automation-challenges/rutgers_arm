/**
 * @file distance_metric.cpp
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

#include "prx/utilities/distance_metrics/distance_metric.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"

namespace prx
{
    namespace util
    {

        pluginlib::ClassLoader<distance_metric_t> distance_metric_t::loader("prx_utilities", "prx::util::distance_metric_t");

        distance_metric_t::distance_metric_t( )
        {
            nr_points = 0;
            function_name = "prx_utilities/default_euclidean";
            space = NULL;
            distance_function = NULL;
            function = NULL;
        }

        void distance_metric_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            function_name = "prx_utilities/";
            function_name += parameters::get_attribute_as<std::string>("distance_function",reader,template_reader,"default_euclidean");
            PRX_DEBUG_COLOR("Distance_function: " << function_name, PRX_TEXT_GREEN);
        }

        unsigned distance_metric_t::get_nr_points( )
        {
            return nr_points;
        }

        void distance_metric_t::link_space( const space_t* inspace )
        {
            // PRX_PRINT("=========================================================", PRX_TEXT_CYAN);
            // PRX_PRINT(" Linking space to distance metric [" << this << "]", PRX_TEXT_GREEN);
            // PRX_PRINT("=========================================================", PRX_TEXT_CYAN);
            
            // PRX_PRINT("SPACE [" << space << "]  FUNCTION [" << function << "]", PRX_TEXT_LIGHTGRAY);
            
            //If the space is NULL, but we have been given a distance function
            bool overwrite = true;
            if( space == NULL && function != NULL )
            {
                //Then we need to not overwrite things
                overwrite = false;
            }
            // PRX_PRINT("Overwriting? :" << (overwrite ? "YES":"NO" ), PRX_TEXT_RED + (overwrite ? 0 : 1 ) );
            
            if (space != inspace)
            {
                this->clear();
                space = inspace;
                unsigned dim = space->get_dimension();
                gamma_val = exp(1)*(1+1.0/dim)+.001;
                space_measure = 1;
                const std::vector<bounds_t*>& sp_bounds = space->get_bounds();
                for( unsigned i=0; i<space->get_dimension(); ++i )
                    space_measure *= sp_bounds[i]->get_upper_bound() - sp_bounds[i]->get_lower_bound();

                if(function_name=="prx_utilities/default")
                {
                    function_name = "prx_utilities/default_euclidean";
                }
                if( overwrite )
                {
                    function = distance_function_t::get_loader().createUnmanagedInstance(function_name);
                }
                function->link_space(inspace);
                distance_function = function->dist;
            }

            // PRX_PRINT("SPACE [" << space << "]  FUNCTION [" << function << "]", PRX_TEXT_LIGHTGRAY);

        }
        
        void distance_metric_t::link_distance_function( distance_function_t* input_function )
        {
            // PRX_PRINT("Linking in new distance function [" << input_function << "] overwriting old function [" << function << "]", PRX_TEXT_CYAN);
            function = input_function;
        }

        distance_function_t* distance_metric_t::get_distance_function()
        {
            return function;
        }

        pluginlib::ClassLoader<distance_metric_t>& distance_metric_t::get_loader()
        {
            return loader;
        }

    }
}
