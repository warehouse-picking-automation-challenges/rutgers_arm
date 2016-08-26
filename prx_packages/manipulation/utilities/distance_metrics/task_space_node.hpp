/**
 * @file task_space_node.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2016, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#pragma once
#ifndef PRX_TASK_SPACE_NODE_HPP
#define PRX_TASK_SPACE_NODE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/graph/abstract_node.hpp"

namespace prx
{
    namespace util
    {

        /**
         * @brief <b> . </b>
         */
        class task_space_node_t : public abstract_node_t
        {

          public:

            task_space_node_t();
            task_space_node_t(const task_space_node_t* in_node);
            task_space_node_t(const space_t* arm_space, space_point_t* input_arm_point, space_point_t* input_end_effector_point, const config_t& input_end_effector_config );

            virtual ~task_space_node_t();

            /**
             * Serializes the node information to an output stream.
             * @brief Serializes the node information to an output stream.
             * @param output_stream The stream to output information to.
             * @param point_space The space that \c point belongs to.
             */
            virtual void serialize(std::ofstream& output_stream, const space_t* point_space);

            /**
             * Deserializes the node information from an input stream.
             * @brief Deserializes the node information from an input stream.
             * @param input_stream The stream to output information to.
             * @param point_space The space that \c point belongs to.
             */
            virtual void deserialize(std::ifstream& input_stream, const space_t* point_space, std::string soft_constraint_type = "");

            space_point_t* arm_point;
            space_point_t* end_effector_point;
            config_t end_effector_config;
            task_space_node_t* other_metric_node;
            abstract_node_t* motion_planning_node;


          protected:

            const space_t* arm_space, *end_effector_space;

            bool cloned_point;

            // Used during serialization and deserialization 
            double _x, _y, _z, _qx, _qy, _qz, _qw;

        };

    }
}

#endif


