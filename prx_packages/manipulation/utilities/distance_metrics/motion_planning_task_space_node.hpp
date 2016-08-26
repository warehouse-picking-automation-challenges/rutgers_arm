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
#ifndef PRX_MOTION_PLANNING_TASK_SPACE_NODE_HPP
#define PRX_MOTION_PLANNING_TASK_SPACE_NODE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/graph/abstract_node.hpp"
#include "utilities/distance_metrics/task_space_node.hpp"

namespace prx
{
    namespace util
    {

        /**
         * @authors Rahul Shome, Andrew Kimmel
         * @brief <b> Wrapper class which allows a motion planning node (i.e. rrt_node, prm_star_node) to gain access to the task space node interface. </b>
         */
        template<typename node_type>
        class motion_planning_task_space_node_t : public node_type
        {

          public:

            motion_planning_task_space_node_t()
            {
                task_space_node = NULL;
            }

            ~motion_planning_task_space_node_t()
            {
                if (task_space_node != NULL)
                    delete task_space_node;
            }

            /**
             * Serializes the node information to an output stream.
             * @brief Serializes the node information to an output stream.
             * @param output_stream The stream to output information to.
             * @param point_space The space that \c point belongs to.
             */
            virtual void serialize(std::ofstream& output_stream, const space_t* point_space)
            {
                node_type t;
                node_type::serialize(output_stream, point_space);
                PRX_ASSERT(task_space_node != NULL);
                task_space_node->serialize(output_stream, point_space);
            }

            /**
             * Deserializes the node information from an input stream.
             * @brief Deserializes the node information from an input stream.
             * @param input_stream The stream to output information to.
             * @param point_space The space that \c point belongs to.
             */
            virtual void deserialize(std::ifstream& input_stream, const space_t* point_space, std::string soft_constraint_type = "")
            {
                node_type t;
                node_type::deserialize(input_stream, point_space, soft_constraint_type);

                task_space_node = new task_space_node_t();
                task_space_node->deserialize(input_stream, point_space, soft_constraint_type);
                task_space_node->motion_planning_node = this;
            }

            task_space_node_t* task_space_node;

        };

    }
}

#endif


