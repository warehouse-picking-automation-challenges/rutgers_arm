/**
 * @file particle_controller.hpp 
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

#ifndef PRX_PARTICLE_CONTROLLER_HPP
#define PRX_PARTICLE_CONTROLLER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/system.hpp"
#include "prx/simulation/systems/controllers/simple_controller.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/simulation/control.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <boost/any.hpp>
#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>
#include <vector>

namespace prx
{
    namespace packages
    {
        namespace conformant
        {
            class particle_controller_t : public sim::simple_controller_t
            {
                public:
                    particle_controller_t();

                    /** @copydoc system_t::~system_t() */
                    virtual ~particle_controller_t();

                    /**
                    * @brief Initializes the particle_controller_t 
                    * @param reader Used to initialize the controller
                    * @param template_reader Used to initialize the controller from template files
                    */
                    virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

                    virtual void propagate(const double simulation_step = 0);

                    virtual void compute_control();

                    /** 
                    * @copydoc simple_controller_t::verify()
                    */
                    virtual void verify() const;
                protected:
                    unsigned state_size;
                    unsigned number_of_states;
                    double failure_rate;
                    std::vector<double> control_variance;

                    sim::system_ptr_t child_system;
                    const util::space_t *child_state_space;
                    sim::control_t* sample_control;
                    sim::control_t* stored_control;
                    sim::state_t* stored_state;
                    std::vector<double> temp_state;
            };
        }
    }
}

#endif