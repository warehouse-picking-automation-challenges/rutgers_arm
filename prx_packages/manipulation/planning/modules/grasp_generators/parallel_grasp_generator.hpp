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

#ifndef PRX_PARALLEL_GRASP_GENERATOR_HPP
#define PRX_PARALLEL_GRASP_GENERATOR_HPP

#include "planning/modules/grasp_generator.hpp"

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
            /**
             * Grasp Class for parallel grasp generation
             *
             * @authors Andrew Kimmel
             */
            class parallel_grasp_generator_t : public grasp_generator_t
            {

              public:

                parallel_grasp_generator_t();
                virtual ~parallel_grasp_generator_t();

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

                util::vector_t default_ee_normal;
                std::vector<util::vector_t> restricted_normals;
                std::vector<util::vector_t> preferred_normals;
                unsigned manual_grasp_mode, manual_release_mode;
                double manual_angle_increment;
                bool grasp_entire_mesh;
                bool save_grasps_to_file;
                // unsigned nr_grasps_to_sample;
                bool limiting_normal;
                util::vector_t limiting_axis;
                util::vector_t hand_long_axis;
                unsigned total_nr_grasps;
                util::config_t local_ee_config;


            };
        }
    }
}


#endif
