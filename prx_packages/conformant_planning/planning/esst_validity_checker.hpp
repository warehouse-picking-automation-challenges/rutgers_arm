/**
 * @file esst_validity_checker.hpp 
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

#ifndef PRX_ESST_VALIDITY_CHECKER_HPP
#define PRX_ESST_VALIDITY_CHECKER_HPP

#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "utilities/particle_space.hpp"
#include "simulation/particle_trajectory.hpp"

namespace prx
{
    namespace packages
    {
        namespace conformant
        {


            class esst_validity_checker_t : public plan::validity_checker_t
            {
              public:

                esst_validity_checker_t(){}

                virtual ~esst_validity_checker_t(){ }

                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);
                
                /**
                 * @copydoc validity_checker_t::is_valid()
                 */
                virtual bool is_valid(const sim::state_t* point);

                /**
                 * @brief Validity check for an entire trajectory.
                 *
                 * @param input The input trajectory to be checked for validity.
                 */
                virtual bool is_valid(const sim::trajectory_t& input);

                double probability_of_validity(const sim::trajectory_t& input);

                void set_previous_collisions(const std::vector<bool>& collisions)
                {
                  collided = collisions;
                }
                std::vector<bool>& get_new_collisions()
                {
                  return collided;
                }
                /**
                 * @brief Link a world model for this validity checker to use.
                 *
                 * @param model The world model to link to this validity checker.
                 */
                virtual void link_model(plan::world_model_t* model);


                void link_particle_space(util::particle_space_t* p_space)
                {
                    particle_space = p_space;
                    state_size = particle_space->get_point_space()->get_dimension();
                    num_states = particle_space->get_num_particles();
                }
              
              private:

                util::particle_space_t* particle_space;
                unsigned state_size;
                unsigned num_states;
                double collision_threshold;
                bool boundary_check;
                std::vector<bool> collided;

                
            };
        }

    }
}


#endif