/**
 * @file homing_application.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_HOMING_APPLICATION_HPP
#define	PRX_HOMING_APPLICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/applications/application.hpp"

namespace prx
{
    namespace sim
    {
        
    }
    
    namespace packages
    {
        namespace two_dim_problems
        {
            class homing_controller_t;

            /**
             * An application used in homing experiments
             * 
             * @brief <b> An application used in homing experiments </b>
             * 
             * @authors Zakary Littlefield
             */
            class homing_application_t : public sim::application_t
            {

              public:
                homing_application_t();
                virtual ~homing_application_t();

                /**
                 * Initializes from the given parameters.
                 * 
                 * @brief Initializes from the given parameters.
                 * 
                 * @param reader A \ref util::parameter_reader_t with a dictionary of parameters for this system.
                 */
                virtual void init(const util::parameter_reader_t * const reader);

                /** @copydoc application_t::set_selected_path(const std::string&)*/
                virtual void set_selected_path(const std::string& path);
                virtual void set_selected_point(double x, double y, double z);
                
            protected:
                homing_controller_t* homing_controller;
                

            };
        }

    }
}

#endif

