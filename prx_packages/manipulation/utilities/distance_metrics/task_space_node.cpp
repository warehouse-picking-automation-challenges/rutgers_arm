/**
 * @file task_space_node.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */


#include "utilities/distance_metrics/task_space_node.hpp"
#include "prx/utilities/spaces/embedded_space.hpp"
#include <fstream>

#include <boost/assign/list_of.hpp>
#include <boost/graph/adjacency_list.hpp>

namespace prx
{
    namespace util
    {

        task_space_node_t::task_space_node_t()
        {
            cloned_point = false;

            //PRX_WARN_S ("Default constructor: " << this);
            std::vector<double*> state_dim = boost::assign::list_of(&_x)(&_y)(&_z)(&_qx)(&_qy)(&_qz)(&_qw);
            end_effector_space = new space_t("SE3", state_dim);

            arm_point = NULL;
            end_effector_point = NULL;
            other_metric_node = NULL;
            motion_planning_node = NULL;
        }

        task_space_node_t::task_space_node_t(const task_space_node_t* in_node)
        {
            cloned_point = true;
            std::vector<double*> state_dim = boost::assign::list_of(&_x)(&_y)(&_z)(&_qx)(&_qy)(&_qz)(&_qw);
            end_effector_space = new space_t("SE3", state_dim);
            //PRX_WARN_S ("Non cloned constructor: " << this);
            arm_point = in_node->arm_point;
            end_effector_point = in_node->end_effector_point;
            end_effector_config = in_node->end_effector_config;
            other_metric_node = in_node->other_metric_node;
            motion_planning_node = in_node->motion_planning_node;
            arm_space = in_node->arm_space;
        }

        task_space_node_t::task_space_node_t(const space_t* input_arm_space, space_point_t* input_arm_point, space_point_t* input_end_effector_point, const config_t& input_end_effector_config )
        {
            cloned_point = false;
            //PRX_WARN_S ("Cloned constructor: " << this);
            std::vector<double*> state_dim = boost::assign::list_of(&_x)(&_y)(&_z)(&_qx)(&_qy)(&_qz)(&_qw);
            end_effector_space = new space_t("SE3", state_dim);

            arm_space = input_arm_space;
            arm_point = arm_space->clone_point(input_arm_point);
            point = arm_point;

            end_effector_point = end_effector_space->clone_point(input_end_effector_point);
            end_effector_config = input_end_effector_config;
            motion_planning_node = NULL;
        }

        task_space_node_t::~task_space_node_t()
        {
            //PRX_WARN_S ("Task space node destructor!: " << this);
            if (!cloned_point)
            {
                if (end_effector_point != NULL)
                {
                    //PRX_WARN_S ("End effector point delete!");
                    end_effector_space->free_point(end_effector_point);
                    end_effector_point = NULL;
                }

                if (arm_point != NULL)
                {
                    //PRX_WARN_S ("Arm point delete!");
                    arm_space->free_point(arm_point);
                    arm_point = NULL;
                }   
            }

            delete end_effector_space;
        }

        void task_space_node_t::serialize(std::ofstream& output_stream, const space_t* point_space)
        {
            //    PRX_ERROR_S ("Node serialization");
            output_stream << node_id << " " << point_space->print_point(arm_point);

            output_stream << std::endl << end_effector_space->print_point(end_effector_point);

        }

        // Accepts an ifstream and deserializes a node to output_node

        void task_space_node_t::deserialize(std::ifstream& input_stream, const space_t* point_space, std::string soft_constraint_type)
        {
            //    PRX_DEBUG_S ("Node deserialization");

            double value;
            char trash;
            input_stream >> node_id;
            std::vector<double> vals;
            std::vector<double> ee_vals;


            arm_point = point_space->alloc_point();

            arm_space = point_space;

            for( unsigned int i = 0; i < point_space->get_dimension(); i++ )
            {
                input_stream >> value;
                PRX_DEBUG_S("Value " << value);
                vals.push_back(value);
                if( i < point_space->get_dimension() - 1 )
                    input_stream >> trash;
                PRX_DEBUG_S("Trash : " << trash);
            }
            point_space->set_from_vector(vals, arm_point);
            point = arm_point;

            // Read in newline character?
            input_stream >> trash;
            PRX_DEBUG_S("Trash : " << trash);

            end_effector_point = end_effector_space->alloc_point();

            for( unsigned int i = 0; i < end_effector_space->get_dimension(); i++ )
            {
                input_stream >> value;
                PRX_DEBUG_S("Value " << value);
                ee_vals.push_back(value);
                if( i < end_effector_space->get_dimension() - 1 )
                    input_stream >> trash;
                PRX_DEBUG_S("Trash : " << trash);
            }

            end_effector_space->set_from_vector(ee_vals, end_effector_point);
            end_effector_config = config_t(ee_vals);


        }

    }
}
