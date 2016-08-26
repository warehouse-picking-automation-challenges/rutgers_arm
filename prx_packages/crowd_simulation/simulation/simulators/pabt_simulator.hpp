/**
 * @file pabt_simulator.hpp
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

#ifndef PRX_PABT_SIMULATOR_HPP
#define	PRX_PABT_SIMULATOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/simulation/simulators/default_simulator.hpp"

namespace prx
{
    namespace packages
    {
        namespace crowd
        {
            /**
             * Basic parallel simulator for the PABT project.
             * 
             * @brief <b>Basic parallel simulator for the PABT project.. </b>
             * 
             * @author Athanasios Krontiris
             */
            class pabt_simulator_t : public sim::default_simulator_t
            {

              public:
                pabt_simulator_t();
                virtual ~pabt_simulator_t();

                virtual void propagate_and_respond();

                virtual void push_control(const sim::control_t * const control);

            };
        }

    }
}

#endif
