/**
 * @file grasp_visualizer.hpp 
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

#ifndef PRX_GRASP_VISUALIZER
#define PRX_GRASP_VISUALIZER

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/plan.hpp"
#include "prx/simulation/trajectory.hpp"
#include "prx/simulation/systems/controllers/controller.hpp"
#include "prx/simulation/state.hpp"
#include "prx/simulation/control.hpp"
#include "simulation/systems/plants/manipulator.hpp"
#include "simulation/systems/plants/movable_body_plant.hpp"



namespace prx
{
    namespace packages
    {
        namespace manipulation
        {

            class grasp_visualizer_t : public sim::controller_t
            {

              public:

                grasp_visualizer_t();

                virtual ~grasp_visualizer_t();

                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

                virtual void compute_control();

                virtual void read_database_file(std::string grasp_filename);

              protected:

                void determine_top_and_front_faces(movable_body_plant_t* input_object, std::string& top_front_face, std::vector<std::string>& alternative_faces);
                void determine_face(const util::vector_t& axis, const util::quaternion_t& rotation, std::string& face);

                sim::trajectory_t grasping_states;

                int current_grasp_state, current_grasp_pose;

                manipulator_t* manipulator;

                std::vector<std::vector<double> > object_poses;
                std::string grasp_path, grasp_file, start_link, end_link;
                movable_body_plant_t* object;
            };
        }
    }
}

#endif