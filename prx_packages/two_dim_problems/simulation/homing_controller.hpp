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

#ifndef PRX_HOMING_CONTROLLER_HPP
#define	PRX_HOMING_CONTROLLER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/controllers/simple_controller.hpp"
#include "prx/simulation/systems/obstacle.hpp"

namespace prx
{
    namespace sim
    {
    }
    
    namespace packages
    {
        namespace two_dim_problems
        {

            class homing_controller_t : public sim::simple_controller_t
            {

              public:
                /**
                 * Initializes internal variables and sets up the controller's state space with the time parameter.
                 * 
                 * @brief Initializes internal variables and sets up the controller's state space with the time parameter.
                 */
                homing_controller_t();

                /** @copydoc system_t::~system_t() */
                virtual ~homing_controller_t();

                /**
                 * 
                 */
                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

                /**
                 * Calls \c compute_control to its single subsystem.
                 * 
                 * @brief \c Calls compute_control to its single subsystem.
                 */
                virtual void compute_control();
                
                virtual void set_landmarks(const util::config_list_t& new_landmarks);
              protected:
                  
                  void update_current_and_goal();
                  void get_direction_to_landmark(int landmark_index, util::vector_t& current_direction, util::vector_t& goal_direction);
                  
                  void voting_method();
                  void kostas_method();
                  
                  util::config_list_t landmarks;
                  bool use_all_landmarks, use_voting_method, conservative;
                  std::vector<unsigned> direction_votes;
                  util::vector_t up_vector;
                  
                  util::config_t current_config, goal_config;
                  const util::space_t* child_state_space;
                  const sim::state_t *current_goal_state;
                  sim::state_t* get_state;
                  
                  double _x,_y,_z;
                  util::vector_t current_angle;
                  double error_threshold, vote_threshold;
                  double homing_direction;
            };
        }
    }
}


#endif

