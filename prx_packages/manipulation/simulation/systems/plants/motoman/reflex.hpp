/**
 * @file reflex.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2016, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_REFLEX_HPP
#define	PRX_REFLEX_HPP

#include "prx/utilities/definitions/defs.hpp"

#include "simulation/systems/plants/ff_end_effector.hpp"

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
            /**
             * 
             */
            class reflex_t : public ff_end_effector_t
            {

              public:
                reflex_t();

                virtual ~reflex_t(){ }

                /** @copydoc plant_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*) */
                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);
                
                virtual void update_phys_configs(util::config_list_t& configs, unsigned& index) const;

                virtual void update_collision_info();
                
                virtual void FK_solver( util::config_t& link_config, std::string link_name);

              protected:

                void update_hand_geometries() const;

                std::vector<double*> reflex_gripper_angles;

            };
        }

    }
}

#endif
