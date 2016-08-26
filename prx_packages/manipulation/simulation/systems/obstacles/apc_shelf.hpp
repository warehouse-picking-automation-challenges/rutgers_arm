/**
 * @file apc_shelf.hpp 
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

#ifndef PRX_APC_SHELF_HPP
#define PRX_APC_SHELF_HPP

#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/simulation/systems/obstacle.hpp"

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
            class apc_shelf_t : public sim::obstacle_t
            {

              public:
                apc_shelf_t();

                virtual ~apc_shelf_t();

                /** @copydoc system_t::init() */
                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

              protected:

                double FOURTH_SHELF_HEIGHT ;
                double THIRD_SHELF_HEIGHT;
                double SECOND_SHELF_HEIGHT;
                double FIRST_SHELF_HEIGHT ;
                double LEFT_SHELF_WIDTH ;
                double MIDDLE_SHELF_WIDTH;
                double RIGHT_SHELF_WIDTH ;
                double SHELF_DEPTH;
                double LEG_HEIGHT;
                double TOP_SHELF_OFFSET;
                double HOR_SHELF_OFFSET;

                std::vector<double> shelf_dims;

                void compute_shelf_geoms();
                void compute_lips();
                void compute_dividers();
                void compute_large_regions();

                void add_geom(std::string geom_name, util::config_t& rel_config);

            };
        }
    }
}


#endif