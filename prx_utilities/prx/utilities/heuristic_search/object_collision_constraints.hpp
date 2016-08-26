/**
 * @file object_collision_constraints.hpp 
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
#ifndef PRX_OBJECT_COLLISION_CONSTRAINTS_HPP
#define	PRX_OBJECT_COLLISION_CONSTRAINTS_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/heuristic_search/constraints.hpp"

namespace prx
{
    namespace util
    {
        /**         
         * An abstract class to keep the information for the soft constraints.
         * 
         * @brief <b> An abstract class to keep the information for the soft constraints.
         * 
         * @Author: Athanasios Krontiris
         */
        class object_collision_constraints_t : public util::constraints_t
        {

          public:

            object_collision_constraints_t();

            virtual ~object_collision_constraints_t();

            /**
             * @brief Initializes the class.
             * @details Initializes the class. By default will clean the constraint sets that correspond 
             * to the set of the constraints and the shortest distance that a set appeared. 
             * 
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

            /** @copydoc constraints::get_type()*/
            std::string get_type();

            virtual void clear();            

            virtual constraints_t& operator=(const constraints_t& c);

            virtual bool operator == ( const constraints_t& c) const;

            virtual bool operator<(const constraints_t& c) const;

            virtual void merge(const constraints_t* c);


            virtual bool intersect(constraints_t* out_constraints, const constraints_t* valid_constraints);

            /**
             * @copydoc constraints::has_hard_constraints(const std::vector<std::set<unsigned > >&);
             * 
             * For this problem we know that the hard constraints will be in the level 0. Thats why will be
             * enough to check all the other levels except the level 0 for valid constraints. 
             */
            virtual bool has_intersection(const constraints_t* valid_constraints) const;
    
            /**
             * @copydoc constraints::has_hard_constraints(const std::vector<std::set<unsigned > >&);
             * 
             * For this problem we know that the hard constraints will be in the level 0. Thats why will be
             * enough to check only the \c constraints[0]
             */
            virtual bool has_hard_constraints(const constraints_t* valid_constraints) const;

            virtual void add_to_constraint_sets( const constraints_t* c, double d );


            /**
             * @copydoc constraints::exact_constraint_comparison(std::vector<std::set<unsigned > >&, double);
             * 
             * For this specific implementation we are interested in the level 1 constraints. Which are the most 
             * important soft constraints. These constraints represent the object that we can move.
             */
            virtual bool exact_constraint_comparison(const constraints_t* new_constraints, double new_distance);

            virtual void serialize(std::ofstream& output_stream);

            virtual void deserialize(std::ifstream& input_stream);

            virtual std::string print() const;   

            /** @brief Constraints that the current element has */
            std::set< unsigned > constraints;
            
            std::vector< std::pair< std::set< unsigned > , double > > constraint_sets;

            /** @brief Distance used for exact constraint comparison. */
            double distance;

          protected:
            bool dominates(const constraints_t* constraints, double dist);
        };
    }
    
}
#endif	

