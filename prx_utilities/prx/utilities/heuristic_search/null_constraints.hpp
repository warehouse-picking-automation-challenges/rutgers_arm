/**
 * @file null_constraints.hpp 
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
#ifndef PRX_NULL_CONSTRAINTS_HPP
#define PRX_NULL_CONSTRAINTS_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/heuristic_search/constraints.hpp"
 
#include <pluginlib/class_loader.h>

namespace prx
{
    namespace util
    {
        /**         
         * An implementation of the constraints class which essentially means there are no
         * constraints.
         * 
         * @brief <b> A non-constraints constraints class.
         * 
         * @Author: Andrew Dobson
         */
        class null_constraints_t : public constraints_t
        {

          public:

            null_constraints_t();
            virtual ~null_constraints_t();

            /**
             * @brief Returns the type of the soft constraint so the abstract node can generate the soft constraint while serialize/deserialize
             * @details Returns the type of the soft constraint so the abstract node can generate the soft constraint while serialize/deserialize
             * The type will be used inside the null_constraints_t::loader in order to instantiate a class of this type.
             * 
             * @return Returns the type of this class.
             */
            virtual std::string get_type();

            virtual constraints_t& operator=(const constraints_t& c);
            virtual bool operator == ( const constraints_t& c) const;
            virtual bool operator<(const constraints_t& c) const;

            /**
             * @brief Merges the current null_constraints with the given one.
             * @details Merges the current null_constraints with the given one.
             * 
             * @param c The null_constraints that we want to merge into the current ones.
             */
            virtual void merge(const constraints_t* c);

            /**
             * @brief Returns the valid null_constraints on this soft null_constraints struct. 
             * @details Returns the valid null_constraints on this soft null_constraints struct. Will use the list of the valid_null_constraints
             * in order to return the null_constraints that are actually valid.
             * 
             * @param null_constraints The returned set of valid null_constraints, based on the list of the valid null_constraints. 
             * @param valid_null_constraints The valid set of null_constraints.
             *
             */
            virtual bool intersect(constraints_t* out_constraints, const constraints_t* valid_constraints);

            /**
             * @brief Returns true if there are any kind valid soft null_constraints.
             * @details Returns true if there are any kind valid soft constraints. This function will check the list of the full
             * null_constraints that this node has and will decide if this element has soft null_constraints. Its up to the user to 
             * select and detect the soft constraints for its problem. 
             * 
             * @param valid_null_constraints The valid null_constraints for this run. 
             * @return Returns true if there are any kind of valid valid_null_constraints, otherwise false.
             */
            virtual bool has_intersection(const constraints_t* valid_constraints) const;

            /**
             * @brief Checks if the node or edges has hard constraints. 
             * @details Checks if the node or edges has hard constraints. Based on the specification of the user
             * this function checks if the node/edge has constraints that block the expansion. 
             * 
             * @param valid_null_constraints The valid null_constraints for this run. The function will check against the valid null_constraints
             * in order to find hard null_constraints.
             * @return True if there are hard null_constraints, otherwise false.
             */
            virtual bool has_hard_constraints(const constraints_t* valid_constraints) const;

            virtual void add_to_constraint_sets( const constraints_t* c, double d );

             /**
              * @brief Checks if the new null_constraints will be added in the current soft constraint of the node/edge. 
              * 
              * @details Checks if the new null_constraints will be added in the current soft constraint of the node/edge. 
              * If the new null_constraints will not be eliminated from other null_constraints it means that a new set of null_constraints
              * has been discovered and the vertex will be added again in the heap.
              * 
              * @param new_null_constraints The new set of null_constraints
              * @param new_distance The distance from the start that we detected this set of null_constraints. 
              * 
              * @return If new_null_constraints is desirable to keep around, returns true.
              */
            virtual bool exact_constraint_comparison(const constraints_t* new_constraints, double new_distance);

        };
    }
}
#endif  

