/**
 * @file GTA_application.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Kimmel, Andrew Dobson, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_GTA_SIM_APPLICATION_HPP
#define PRX_GTA_SIM_APPLICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "../../../VO/simulation/applications/VO_application.hpp"
#include "simulation/controllers/GTA_replanning.hpp"
#include "simulation/controllers/replanning_waypoints.hpp"

namespace prx
{
    namespace util
    {
        class sys_clock_t;
    }
    namespace packages
    {
        namespace gta 
        {
            /**
             * @author Andrew Kimmel
             */
            class GTA_application_t : public VO::VO_application_t
            {
            public:
                GTA_application_t();
                virtual ~GTA_application_t();
                
            protected:
                /**
                 *
                 */
                void init(const util::parameter_reader_t * const reader);
                /**
                 *
                 */
                void frame(const ros::TimerEvent& event);
                
                virtual void shutdown();
                
                void find_random_starts();
                
                virtual void initialize_sensing();
                
            private:
                std::vector<GTA_replanning_controller_t*> consumer_controllers;
                
                // Checks if the controllers have finished planning: automatically exits then
                bool check_controllers_finished();
                
                
                // TODO: Eventually remove once and twice from GTA, and start
                // with an initial plan of 0.
                // Once checks if initial plan has been sent
                bool check_controllers_once();
                // Twice checks if neighborhood has been generated
                bool check_controllers_twice();
                
                bool check_controllers_synchronization();
                
                double timeout, wait_time;
                int timeout_steps;
                util::sys_clock_t timeout_timer, wait_timer;
                
                std::vector<double> sol_times, path_lengths;
                std::string experiment_filename;
                
                bool automatic, synchronized, random_start, non_vo_avoidance;
                
                sim::state_t* sampled_state, *sub_sampled_state, *zero_state;

            };
        }
    }
}

#endif

