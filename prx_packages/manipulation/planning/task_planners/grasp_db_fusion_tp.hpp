/**
 * @file grasp_db_fusion_tp.hpp
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

#ifndef PRX_GRASP_DB_FUSION_TP_HPP
#define PRX_GRASP_DB_FUSION_TP_HPP

 #include "planning/task_planners/apc_grasping_planner.hpp"

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
            class movable_body_plant_t;

            /**
             * Manipulation task planner. Computes the path for moving an object from an
             * initial to a target position.
             *
             * @authors Andrew Kimmel, Zakary Littlefield
             */
             
            class grasp_db_fusion_tp_t : public apc_grasping_planner_t
            {

              public:

                grasp_db_fusion_tp_t();
                virtual ~grasp_db_fusion_tp_t();

                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                virtual void setup();

                /**
                 * @copydoc planner_t::resolve_query()
                 */
                virtual void resolve_query();

              protected:

                virtual void create_quaternion_rotations( std::vector< util::quaternion_t >& output_quats );

                virtual bool serialize_all_grasps(const std::vector<grasp_t>& grasp_database, const std::string& top_front_face, const std::string& object_type, const std::string& output_folder);

                virtual bool serialize_non_duplicate_grasps(const std::vector<grasp_t>& grasp_database, const std::string& top_front_face, const std::string& object_type, const std::string& output_folder, std::vector<bool>& non_duplicate_grasp_markers);

                /** @brief The movable body plants in the world model. In this task planner, only a single object is allowed */
                std::vector<movable_body_plant_t* > objects;

                std::vector<double> bounding_box;
                double back_plane, bottom_plane;
                double back_plane_offset;
                double bottom_plane_offset;
                std::string output_data_folder;

              };

        }
    }
}

#endif
