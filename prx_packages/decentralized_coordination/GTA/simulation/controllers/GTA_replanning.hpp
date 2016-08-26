#pragma once
/**
 * @file replanning_controller.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Andrew Kimmel, Andrew Dobson, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#ifndef PRX_GTA_REPLANNING_CONTROLLER
#define PRX_GTA_REPLANNING_CONTROLLER

#include "prx/simulation/systems/controllers/consumer/consumer_controller.hpp"
#include "../../../VO/simulation/controllers/VO_controller.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include <list>

namespace prx
{
    namespace util
    {
        class radial_goal_region_t;
    }
    namespace packages
    {
        namespace VO
        {
            class VO_controller_t;
        }
        namespace gta
        {
            
            class GTA_replanning_controller_t : public sim::consumer_controller_t
            {

              public:
                /**
                 * Initializes the state space of the controller with the parameter time.
                 */
                GTA_replanning_controller_t();

                /** @copydoc system_t::~system_t() */
                virtual ~GTA_replanning_controller_t();

                /** @copydoc system_t::init(const parameter_reader_t * const) */
                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

                /** 
                 * @copydoc stateful_controller_t::propagate()
                 *
                 * This function will increment internal state value of time based on
                 * the set simulation step for the consumer controller.
                 * 
                 */
                virtual void propagate(const double simulation_step = 0);

                /** 
                 * @copydoc stateful_controller_t::verify()
                 *
                 * Checks to make sure the simulation step is set to something valid
                 *
                 */
                virtual void verify() const;

                virtual void compute_control();

                /**
                 * Copies the given plan for consumption
                 */
                virtual void copy_plan(const sim::plan_t& inplan);
                
                virtual void visualize_goals();
                
                virtual void visualize_text();
                
                bool is_finished();
                double get_solution_time();
                
                std::string get_planning_node_name();
                double get_path_length() const;

                
                void set_points_back();
                bool once, twice, got_plan;
                sim::state_t* get_state;
                
              protected:

                  virtual void query_planner();
                  
//                  virtual void adapt_plan(sim::plan_t& inplan);

                  std::vector<util::radial_goal_region_t*> waypoints;

                  // This queue stores the plans computed from all future queries
                  sim::plan_t most_recent_plan;
                  double planning_cycle;
                  int replanning_frames;
                  int replanning_counter;

                  // Counters to determine which waypoint we are on
                  size_t waypoint_index;
                  sim::state_t *future_end_state, *current_goal_state, *previous_state;
                  const util::space_t * child_state_space;
                  double goal_radius;

                  // Various flags
                  bool sent_query, smooth,  finished;
                  bool adapt_plans;

                  double proximity_threshold;
                  double solution_time;
                  double path_length;
                  util::sys_clock_t solution_timer;
                  util::sys_clock_t computation_timer;
                  double computation_time, computation_counter;
                  
                  int iterations_counter;
                  bool reset_timer;
                  bool timer_once;
                  
                  // Visualization flags
                  bool visualizes_text;
                  
                  std::string filename;
                  VO::VO_controller_t* vo_subsystem;

            };
        }
    }
}

#endif