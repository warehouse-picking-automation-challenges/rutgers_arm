/**
 * @file constraints.hpp 
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
#ifndef PRX_CONSTRAINTS_HPP
#define	PRX_CONSTRAINTS_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include <pluginlib/class_loader.h>

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
        class constraints_t
        {

          public:

            constraints_t();

            virtual ~constraints_t();

            /**
             * @brief Initializes the class.
             * @details Initializes the class. By default will clean the constraint sets that correspond 
             * to the set of the constraints and the shortest distance that a set appeared. 
             * 
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);

            /**
             * @brief Returns the type of the soft constraint so the abstract node can generate the soft constraint while serialize/deserialize
             * @details Returns the type of the soft constraint so the abstract node can generate the soft constraint while serialize/deserialize
             * The type will be used inside the constraints_t::loader in order to instantiate a class of this type.
             * 
             * @return Returns the type of this class.
             */
            virtual std::string get_type();

            /**
             * @brief Clears the soft constraints.
             * @details Clears the soft constraints.
             */
            virtual void clear();    

            virtual constraints_t& operator=(const constraints_t& c) = 0;

            virtual bool operator == ( const constraints_t& c) const = 0;

            virtual bool operator<(const constraints_t& c) const = 0;

            /**
             * @brief Merges the current constraints with the given one.
             * @details Merges the current constraints with the given one.
             * 
             * @param c The constraints that we want to merge into the current ones.
             */
            virtual void merge(const constraints_t* c) = 0;

            /**
             * @brief Returns the valid constraints on this soft constraints struct. 
             * @details Returns the valid constraints on this soft constraints struct. Will use the list of the valid_constraints
             * in order to return the constraints that are actually valid.
             * 
             * @param constraints The returned set of valid constraints, based on the list of the valid constraints. 
             * @param valid_constraints The valid set of constraints.
             *
             */
            virtual bool intersect(constraints_t* out_constraints, const constraints_t* valid_constraints) = 0;

            /**
             * @brief Returns true if there are any kind valid soft constraints.
             * @details Returns true if there are any kind valid soft constraints. This function will check the list of the full
             * constraints that this node has and will decide if this element has soft constraints. Its up to the user to 
             * select and detect the soft constraints for its problem. 
             * 
             * @param valid_constraints The valid constraints for this run. 
             * @return Returns true if there are any kind of valid valid_constraints, otherwise false.
             */
            virtual bool has_intersection(const constraints_t* valid_constraints) const = 0;

            /**
             * @brief Checks if the node or edges has hard constraints. 
             * @details Checks if the node or edges has hard constraints. Based on the specification of the user
             * this function checks if the node/edge has constraints that block the expansion. 
             * 
             * @param valid_constraints The valid constraints for this run. The function will check against the valid constraints
             * in order to find hard constraints.
             * @return True if there are hard constraints, otherwise false.
             */
            virtual bool has_hard_constraints(const constraints_t* valid_constraints) const = 0;

            /**
             * @brief Enforce the addition of the given constraints to the set of all path constraints (for MCR search).
             */
            virtual void add_to_constraint_sets( const constraints_t* c, double d ) = 0;
            
             /**
              * @brief Checks if the new constraints will be added in the current soft constraint of the node/edge. 
              * 
              * @details Checks if the new constraints will be added in the current soft constraint of the node/edge. 
              * If the new constraints will not be eliminated from other constraints it means that a new set of constraints
              * has been discovered and the vertex will be added again in the heap.
              * 
              * @param new_constraints The new set of constraints
              * @param new_distance The distance from the start that we detected this set of constraints. 
              * 
              * @return If new_constraints is desirable to keep around, returns true.
              */
            virtual bool exact_constraint_comparison(const constraints_t* new_constraints, double new_distance);

            virtual void serialize(std::ofstream& output_stream);

            virtual void deserialize(std::ifstream& input_stream);

            virtual std::string print() const;        

            /**
             * @brief Retrieve the loader for this class instance.
             *
             * @return The pluginlib class loader for creating instances of planners.
             */
            static pluginlib::ClassLoader<constraints_t>& get_loader();

          private:
            /** @brief The pluginlib loader which is returned by the get_loader() class. */
            static pluginlib::ClassLoader<constraints_t> loader;
        };
    }
}
#endif	

