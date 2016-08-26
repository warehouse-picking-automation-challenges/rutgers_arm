#pragma once
/**
 * @file lqr_task_planner.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2011, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Zakary Littlefield, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, James Marble, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#ifndef PRX_LQR_TASK_PLANNER_HPP
#define	PRX_LQR_TASK_PLANNER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/planning/task_planner.hpp"
#include "prx/utilities/data_structures/hash.hpp"
#include "prx/utilities/math/configurations/vector.hpp"

#ifdef OCTAVE_FOUND
#include "prx/planning/motion_planners/sampling_planners/LQR_tree/LQR_tree_planner.hpp"
#endif

PRX_START

/**
 * A brief description of this class (no tag required).
 * 
 * A more detailed description of this class with multiple paragraphs delimited by blank lines. 
 */
class lqr_task_planner_t : public task_planner_t
{

  public:
    lqr_task_planner_t();
    virtual ~lqr_task_planner_t();
    void init(const parameter_reader_t* reader);
    
    void run();
    void frame(const ros::TimerEvent& event) {}
  protected:
#ifdef OCTAVE_FOUND
    LQR_tree_planner_t* planner;
#endif
    double cycle_time;
    unsigned planning_cycles;
    std::string plant_path;
    std::string controller_path;
    hash_t<std::string, vector_t> goal_states;
    hash_t<std::string, vector_t> start_states;

  private:

};

PRX_FINISH

#endif	// PRX_LQR_TASK_PLANNER_HPP

