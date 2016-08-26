/**
 * @file manipulation_validity_checker.hpp 
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

#ifndef PRX_MANIPULATION_VALIDITY_CHECKER_HPP
#define PRX_MANIPULATION_VALIDITY_CHECKER_HPP

#include "prx/planning/modules/validity_checkers/ignore_list_validity_checker.hpp"

#include "simulation/workspace_trajectory.hpp"

#include "prx/utilities/heuristic_search/constraints.hpp"

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
            class manipulation_world_model_t;

            /**
             * @anchor manipulation_validity_checker_t
             *
             * Validity checker which uses the world model to retrieve the names
             * of the systems which cause collisions.
             *
             * @brief <b> Validity checker which retrieves system names and returns the constraints in a form of ids. </b>
             *
             * @author Andrew Dobson
             */
            class manipulation_validity_checker_t : public plan::ignore_list_validity_checker_t
            {
              public:
                manipulation_validity_checker_t() : ignore_list_validity_checker_t(){}
                virtual ~manipulation_validity_checker_t(){ }

                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);
                
                virtual bool is_valid(const sim::state_t* point);
                virtual bool is_valid(const sim::trajectory_t& path);
                virtual bool is_valid(const sim::trajectory_t& path, const workspace_trajectory_t& ee_trajectory );
                virtual bool validate_and_generate_constraints( util::constraints_t* constraints, const sim::trajectory_t& path );
                virtual bool validate_and_generate_constraints( util::constraints_t* constraints, const sim::trajectory_t& path, const workspace_trajectory_t& ee_trajectory );

                virtual void set_ee_resolution( double input_ee_resolution );

                virtual void link_model(plan::world_model_t* model);

              protected:
                void generate_checked_indices( const workspace_trajectory_t& ee_trajectory );

                double ee_resolution;
                bool use_bounds;
                manipulation::manipulation_world_model_t* manip_model;
                std::vector<util::bounds_t*> bounds;

            };
        }
    }
}


#endif
