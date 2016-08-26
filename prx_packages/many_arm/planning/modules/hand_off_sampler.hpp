/**
 * @file hand_off_sampler.hpp
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

#ifndef PRX_HANDOFF_SAMPLER_HPP
#define PRX_HANDOFF_SAMPLER_HPP

#include "prx/planning/modules/samplers/sampler.hpp"
#include "../../../baxter/simulation/plants/manipulator.hpp"
#include "prx/planning/world_model.hpp"

namespace prx
{
    namespace plan
    {
        class local_planner_t;
        class validity_checker_t;
        class motion_planning_specification_t;
    }

    namespace packages
    {
        namespace manipulation
        {
            class manip_sampler_t;
            // class movable_body_plant_t;
            // class manipulation_specification_t;
        }

        namespace multi_arm
        {
            using namespace baxter;
            using namespace manipulation;

            /**
             * A structure for returning all of this very complex grasp information
             */
            struct grasp_data_t
            {
                util::space_point_t* grasped;
                util::space_point_t* released;
                util::space_point_t* retracted;
                sim::plan_t* retract_plan;
                sim::plan_t* approach_plan;
                sim::trajectory_t* path;
                unsigned grasp_id;
            };

            /**
             * Performs sampling for the manipulator over multiple surfaces.
             *
             * @brief <b> Performs sampling for the manipulator over surfaces.</b>
             *
             * @author Andrew Dobson
             */
            class hand_off_sampler_t : public plan::sampler_t
            {
              public:
                hand_off_sampler_t();
                ~hand_off_sampler_t();
                void sample(const util::space_t* space, util::space_point_t* point);
                void sample_near(const util::space_t* space, util::space_point_t* near_point, std::vector<util::bounds_t*>& bounds, util::space_point_t* point);

                const grasp_data_t* get_first_grasp();
                const grasp_data_t* get_second_grasp();

                //The actual sampling of a single grasp
                bool sample_grasp( unsigned manip_index, const util::space_point_t* object_point, double retraction_distance );
                bool sample_handoff( unsigned first_manip, unsigned second_manip, unsigned grasp_index, const util::space_point_t* object_point, double grasp_retraction_distance );

                bool pose_satisfies_safety( const util::space_point_t* object_pose );
                bool valid_retraction(int manip_index, sim::plan_t* plan, sim::trajectory_t* path, const util::space_point_t* start, util::config_t & goal_config);

                void swap_grasp_data();

                //Linking in all the friggin' things it will need
                void link_manipulators( const std::vector< manipulator_plant_t* > inmanips );
                void link_composite_spaces( const std::vector< util::space_t* > inspaces );
                void link_object_space( util::space_t* inspace );
                void link_move_names( const std::vector< std::string >& innames );
                void link_transfer_names( const std::vector< std::string >& innames );
                void link_grasp_zs( const std::vector< double >& inzs );
                void link_start_state( util::space_point_t* start );
                void link_manipulation_sampler( manip_sampler_t* insampler );
                void link_validity_checker( plan::validity_checker_t* inchecker );
                void link_local_planner( plan::local_planner_t* inlp );
                void link_specifications( const std::vector< plan::motion_planning_specification_t* > inspecs );
                void link_world_model( plan::world_model_t* inmodel );

                bool verify();

              protected:
                //Some utility functions for dealing with global controls and such
                void go_to_start();
                void set_zero_control();

                bool check_for_links();


                std::vector< manipulator_plant_t* > manipulators; //All of the Manipulators
                std::vector< util::space_t* > move_spaces; //State spaces of the Manipulators
                std::vector< util::space_t* > transfer_spaces; //State spaces in the joint Manipulator-Object space
                util::space_t* full_space; //The pointer to the full state space
                util::space_t* object_space; //State space for the object in question.
                std::vector< std::string > move_context_names; //Context names for moving the arms without objects
                std::vector< std::string > transfer_context_names; //Context names for each arm manipulating the object
                std::vector< double > grasp_zs; //The grasp_z values for all of the manipulators (oh lord how will this work in the general case?)
                util::space_point_t* collision_check_state; //A state to check against collisions in which the sampling may be interested in
                util::space_point_t* global_start_state; //The global start state that is read in from input
                util::space_point_t* saved_state; //A global temporary state for saving/restoring the simulation state
                std::vector< util::space_point_t* > states_to_check; //A vector of states against which we will collision check

                manip_sampler_t* manipulation_sampler; //A pointer to a manipulation sampler to generate manipulation states
                plan::validity_checker_t* validity_checker; //A validity checker to determine state validity
                plan::local_planner_t* local_planner; //A local planner to come up with the entire paths which we have to test.
                std::vector< plan::motion_planning_specification_t* > move_specifications; //Specifications we will pass to the motion planner (move)
                plan::world_model_t* model; //The world model which actually holds on to all the magic.

                unsigned max_tries;

                //Global boolean which states things are ready
                bool ready;

                grasp_data_t first_grasp_data;
                grasp_data_t second_grasp_data;

                //Probably want to keep local memory for the points
                util::space_point_t* first_grasped_state;
                util::space_point_t* second_grasped_state;
                util::space_point_t* first_released_state;
                util::space_point_t* second_released_state;
                util::space_point_t* first_retracted_state;
                util::space_point_t* second_retracted_state;
                sim::plan_t* first_plan;
                sim::plan_t* first_approach_plan;
                sim::trajectory_t* first_path;
                sim::plan_t* second_plan;
                sim::plan_t* second_approach_plan;
                sim::trajectory_t* second_path;
                unsigned first_grasp_id;
                unsigned second_grasp_id;
            };
        }
    }
}

#endif //PRX_HANDOFF_SAMPLER_HPP
