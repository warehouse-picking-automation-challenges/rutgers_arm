/**
 * @file grasp_data.hpp
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

#ifndef PRX_GRASP_DATA_HPP
#define PRX_GRASP_DATA_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/heuristic_search/constraints.hpp"

#include "prx/simulation/state.hpp"
#include "prx/simulation/plan.hpp"
#include "prx/simulation/trajectory.hpp"

#include "planning/modules/grasp.hpp"

namespace prx
{
    namespace plan
    {
        class validity_checker_t;
    }

    namespace packages
    {
        namespace manipulation
        {

            /**
             * @anchor grasp_data_t
             *
             * This class is the data that the grasping planner will return with the grasping query to the manipulation task planner.
             * This data are per state of the object that we are trying to grasp.
             * 
             * @brief <b> General grasping data returned from the grasping planner. </b>
             *
             * @author Andrew Dobson, Andrew Kimmel, Rahul Shome
             */
            class grasp_data_t
            {

              public:                
                grasp_data_t();

                grasp_data_t(const util::space_t* state_space, const util::space_t* control_space, const plan::validity_checker_t* new_checker);
                
                virtual ~grasp_data_t();

                /**
                 * @brief Setup a new grasping data struct. 
                 * 
                 * @details Setup a new grasping data struct. 
                 * 
                 * @param state_space The state space for the trajectory.
                 * @param control_space The control space for the plan.
                 * @param new_checker The validity checker used to allocate/deallocate constraints
                 */
                virtual void setup(const util::space_t* state_space, const util::space_t* control_space, const plan::validity_checker_t* new_checker);

                /**
                 * @brief It clears all the data.
                 * @details It clears all the data.
                 */
                virtual void clear();

                /**
                * @brief Deep copy the input grasp data into the current grasp data
                * @details Copy grasp data
                * @param input_grasp_data The data whose values will be copied into the current grasp data
                **/
                virtual void copy_from_data(grasp_data_t* input_grasp_data);

                std::string print(unsigned prec);                

                /** Contains the information for the specific  grasp that is used in order to build this grasp_data. This memory belongs to the grasp_data.*/
                grasp_t* relative_grasp;

                sim::state_t* releasing_state;
                sim::state_t* grasping_state;
                sim::state_t* retracted_open_state;
                sim::state_t* retracted_closed_state;
                
                /** @brief The plan for reaching the object.*/
                sim::plan_t reaching_plan;
                sim::trajectory_t reaching_path;
                util::constraints_t* open_reaching_constraints;
                util::constraints_t* closed_reaching_constraints;
                
                /** @brief The plan for retracting from the object.*/
                sim::plan_t retracting_plan;
                sim::trajectory_t retracting_path;
                util::constraints_t* open_retracting_constraints;
                util::constraints_t* closed_retracting_constraints;

                /**  */
                sim::plan_t connection_plan;
                util::constraints_t* connection_constraints;

            protected:
                const util::space_t* state_space;
                const util::space_t* control_space;
                const plan::validity_checker_t* validity_checker;
                
            };
        }
    }
}

#endif

