/**
 * @file pendulum_task_mapping.hpp 
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

#ifndef PRX_PENDULUM_TASK_MAPPING_HPP
#define	PRX_PENDULUM_TASK_MAPPING_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/mapping_functions/mapping_function.hpp"

namespace prx
{
    namespace packages
    {
        namespace mapping_functions
        {
            /**
             * @brief Converts the pendulum state space into a single value.
             * @author Zakary Littlefield
             */
            class pendulum_task_mapping_t : public util::mapping_function_t
            {
            public:
                pendulum_task_mapping_t() 
                {
                    domain = 2;
                    range = 1;
                    image_space = NULL;
                    subspace = NULL;
                    preimage_space = NULL;
                    image_interval = std::make_pair<unsigned,unsigned>(0,0);
                    preimage_interval = std::make_pair<unsigned,unsigned>(0,0);
                    output_space_name = "R";
                    mapping_name = "pendulum_task_mapping";
                }
                virtual ~pendulum_task_mapping_t() {}    

                /**
                 * @copydoc mapping_function_t::init_spaces()
                 */
                void init_spaces();
                /**
                 * @copydoc mapping_function_t::embed() const
                 */
                virtual void embed() const;
                /**
                 * @copydoc mapping_function_t::invert() const
                 */
                virtual void invert() const;

            };
    }
    }
}


#endif