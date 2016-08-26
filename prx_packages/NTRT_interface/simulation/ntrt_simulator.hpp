/**
 * @file bullet_simulator.hpp 
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#ifdef BULLET_FOUND

#ifndef BT_USE_DOUBLE_PRECISION
#define BT_USE_DOUBLE_PRECISION
#endif

#pragma once

#ifndef PRX_NTRT_SIMULATOR_HPP
#define PRX_NTRT_SIMULATOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/simulation/simulators/simulator.hpp"
#include "simulation/ntrt_plant.hpp"


class tgGround;
class tgWorld;
namespace prx
{

    namespace sim
    {
        class ntrt_simulator_t : public simulator_t
        {

          public:
            ntrt_simulator_t();
            ~ntrt_simulator_t();
            void add_system(const std::string& path, system_ptr_t system);
            void init(const util::parameter_reader_t * const reader, const util::parameter_reader_t* template_reader = NULL);
            void propagate(const double simulation_step = 0);
            virtual void propagate_and_respond();
            void remove_system(const std::string& path);
            void replace_system(const std::string& path, system_ptr_t system);

            virtual void push_state(const state_t * const source);
            virtual state_t* pull_state();
            
            virtual bool in_collision();
            virtual bool internal_state_push()
            {
                  return true;
            }

          private:
            tgGround* ground ;
            tgWorld* world;
            std::vector< ntrt_plant_t* > plants;

            state_t* prev_state;
            control_t* prev_control;
            state_t* curr_state;
            control_t* curr_control;
            const util::space_t* plant_control_space;
            const util::space_t* plant_state_space;


            std::vector<state_t*> stored_states;
            plan_t stored_plan;
            std::vector<double> control_variance;
            double current_time;
            bool do_simulated_particles;


            void add_bodies_from_plants();
        };
    }
}
#endif
#endif

