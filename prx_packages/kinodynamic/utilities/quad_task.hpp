/**
 * @file quad_task.hpp 
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

#ifndef PRX_QUAD_TASK_HPP
#define	PRX_QUAD_TASK_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/mapping_functions/mapping_function.hpp"
#include "prx/utilities/math/configurations/quaternion.hpp"

namespace prx
{
    namespace packages
    {
        namespace mapping_functions
        {
            /**
             * @brief Provides an embedding from the 12D quadrotor space to a SE3
             * @author Zakary Littlefield
             */
            class quad_task_t : public util::mapping_function_t
            {
            public:
                quad_task_t() 
                {
                    domain = 12;
                    range = 7;
                    image_space = NULL;
                    subspace = NULL;
                    preimage_space = NULL;
                    image_interval = std::make_pair<unsigned,unsigned>(0,0);
                    preimage_interval = std::make_pair<unsigned,unsigned>(0,0);
                    output_space_name = "SE3";
                    mapping_name = "quad_task";
                }
                virtual ~quad_task_t() {}    

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
            protected:
                mutable util::quaternion_t quat;
                mutable util::vector_t v;

            };
        }
    }
}


#endif