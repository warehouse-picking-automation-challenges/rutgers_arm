/**
 * @file grasp_generator.hpp
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

#pragma once

#ifndef PRX_UNIGRIPPER_GRASP_GENERATOR_HPP
#define PRX_UNIGRIPPER_GRASP_GENERATOR_HPP

#include "planning/modules/grasp_generator.hpp"

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
            /**
             * Grasp Class for UniGripper grasp generation
             *
             * @authors Andrew Kimmel
             */
            class unigripper_grasp_generator_t : public grasp_generator_t
            {

              public:

                unigripper_grasp_generator_t();
                virtual ~unigripper_grasp_generator_t();

                /**
                 * @copydoc planner_t::init()
                 */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                const std::vector<grasp_t>& compute_grasps(const util::geometry_t* ee_geom, const util::config_t& ee_local_config, movable_body_plant_t* object);


                /**
                 * @brief This function calls IK for the specific manipulation context.
                 * 
                 * @details This function calls IK for the specific manipulation context.
                 * 
                 * @param result_state The resulting state of the IK solution. 
                 * @param start_state The initial state of the manipulator. From this state we can retrieve the starting configuration of the link 
                 * that we are interesting in solving IK for and the mode of the manipulator if it is grasping or not.  
                 * @param goal_config The goal configuration that we need to achieve.
                 * @param validate Only return successful IK if the state is collision-free. 
                 * 
                 * @return True if the IK_solver found a solution, otherwise false.
                 */
                //virtual bool IK( sim::state_t* result_state, const util::space_point_t* start_state, const util::config_t& goal_config, bool validate = false);

                 
                
              protected:

                const std::vector<grasp_t>& compute_grasps_on_entire_mesh(const util::geometry_t* ee_geom, movable_body_plant_t* object);
                const std::vector<grasp_t>& compute_grasps_on_descriptors(const util::geometry_t* ee_geom, movable_body_plant_t* object);

                double evaluate_overlapping_area(const util::geometry_t* ee_geom, const util::config_t& ee_config, const grasp_surface_t& grasp_surface, const util::config_t& object_config);
                

                //Local frame of the end effector that points into the surface
                util::vector_t default_ee_normal;
                //Vector of restricted normals
                std::vector<util::vector_t> restricted_normals;
                //Vector of preferred normals. These will be sorted in the sequence of input
                std::vector<util::vector_t> preferred_normals;
                //Modes to populate in the grasp
                unsigned manual_grasp_mode, manual_release_mode;
                //For manual sampling mode
                double manual_angle_increment;
                //Flag to toggle usage of the grasp descriptors
                bool grasp_entire_mesh;
                //Normal in a 45 degree cone around which the arm should be constrained. This needs a hand_long_axis and a limiting_axis(+X)
                bool limiting_normal;
                //Number of grasps to sample on a surface
                int nr_grasps_to_sample;
                //Total number of grasps to return
                unsigned total_nr_grasps;
                //Axis of the arm in the local coordinate frame of the end effector
                util::vector_t hand_long_axis;
                //Axis(in the global frame) around which the arm is restricted
                util::vector_t limiting_axis;
                //Specific check for grippy or old unigripper with an extra dof
                bool ee_dof_check;
                //Local EE config
                util::config_t local_ee_config;
                //Minimum overlap area
                double minimum_overlap_area;
                // Determines if the final set of grasps is randomized
                bool randomize_final_grasps;

                


            };
        }
    }
}


#endif
