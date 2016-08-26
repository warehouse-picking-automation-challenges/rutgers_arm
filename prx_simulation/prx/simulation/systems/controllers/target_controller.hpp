/**
 * @file target_controller.hpp 
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#pragma once

#ifndef PRX_TARGET_CONTROLLER_HPP
#define PRX_TARGET_CONTROLLER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/system.hpp"
#include "prx/simulation/systems/controllers/simple_controller.hpp"
#include "prx/simulation/control.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/spaces/space.hpp"

namespace prx
{
    namespace sim
    {

        class target_controller_t : public simple_controller_t
        {

          public:
            target_controller_t();

            /** @copydoc system_t::~system_t() */
            virtual ~target_controller_t();

            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            virtual void propagate(const double simulation_step = 0);

            virtual void compute_control();
            
          protected:
            //Child system information.
            system_ptr_t child_system;
            const util::space_t *child_state_space;
        };
    }
}


#endif