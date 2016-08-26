/**
 * @file object_constraints_checker.hpp 
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

#ifndef PRX_OBJECT_CONSTRAINTS_CHECKER_HPP
#define PRX_OBJECT_CONSTRAINTS_CHECKER_HPP

#include "prx/utilities/heuristic_search/constraints.hpp"
#include "planning/modules/manipulation_validity_checker.hpp"

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
            /**
             * @anchor object_constraints_checker_t
             *
             * Validity checker which uses the world model to retrieve the names
             * of the systems which cause collisions.
             *
             * @brief <b> Validity checker which retrieves system names and returns the constraints in a form of ids. </b>
             *
             * @author Athanasios Krontiris
             */
            class object_constraints_checker_t : public manipulation_validity_checker_t
            {
              public:

                object_constraints_checker_t() : manipulation_validity_checker_t(){ }

                virtual ~object_constraints_checker_t(){ }

                void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                virtual void setup_checker( const std::string& object_name );

                /** @copydoc constraints_validity_checker_t::is_valid() */
                virtual bool is_valid(const sim::state_t* point);
                            
                virtual bool validate_and_generate_constraints(util::constraints_t* constraints, const sim::state_t* state);
                
                virtual void collides_with(std::vector<std::string>& bodies);

                virtual util::constraints_t* alloc_constraint() const;

                virtual void get_constraint_names( std::vector< std::string >& names );

                virtual void generate_constraint_names();

                virtual void clear_constraint_names();

              protected:                
                std::string object_name;
                std::string end_effector_name;
                std::vector< std::string > object_names;

                sim::collision_list_t* colliding_bodies;
                
                bool map_generated;
            };
        }
    }
}


#endif
