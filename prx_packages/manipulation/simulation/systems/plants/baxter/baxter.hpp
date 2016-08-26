// /**
//  * @file urdf_plant.hpp
//  *
//  * @copyright Software License Agreement (BSD License)
//  * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
//  * All Rights Reserved.
//  * For a full description see the file named LICENSE.
//  *
//  * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Rahul Shome, Zakary Littlefield, Kostas Bekris
//  *
//  * Email: pracsys@googlegroups.com
//  */
// #pragma once

// #ifndef PRX_BAXTER_HPP
// #define PRX_BAXTER_HPP

// #include "prx/utilities/definitions/defs.hpp"
// #include "prx/utilities/boost/hash.hpp"
// #include "prx/simulation/systems/plants/integration_plant.hpp"
 
// #include "simulation/systems/plants/manipulator.hpp"
// #include "simulation/systems/plants/baxter/ikfast/ikfastdemo.hpp"

// namespace prx
// {
//     namespace util
//     {
//         class parameter_reader_t;
//     }

//     namespace packages
//     {
//         namespace manipulation
//         {
//             /**
//              * A general plant for URDF systems loaded from a file.
//              *
//              * @brief <b> A general plant for URDF systems.  </b>
//              *
//              * @author Andrew Dobson
//              */
//             class baxter_t : public manipulator_t
//             {

//               public:
//                 baxter_t();
//                 virtual ~baxter_t();

//                 /**
//                  * @copydoc plant_t::init()
//                  *
//                  * Reads in all of the information needed to specify the physical
//                  * plant in terms that ODE understands: rigid bodies, joints, and
//                  * controls.
//                  */
//                 void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

//                 *
//                  * @copydoc plant_t::update_phys_configs()
//                  *
//                  * Performs the update according to the actual internal state of ODE.
                 
//                 void update_phys_configs(util::config_list_t& configs, unsigned& index) const;
                
//                 virtual void link_collision_info(sim::collision_checker_t* collision_checker);
                
//                 virtual void update_collision_info();
                
//                 /**
//                  *
//                  */
//                 virtual bool is_grasping() const;

//                 virtual void get_end_effector_configuration(util::config_t& effector_config);

//                 virtual bool IK_solver( const util::config_t& effector_config, util::space_point_t* computed_state, bool set_grasping, const util::space_point_t* seed_state = NULL, bool do_min = false );

//                 virtual bool IK_steering( const util::config_t& start_config, const util::config_t& goal_config, sim::plan_t& result_plan, bool set_grasping = false );

//                 void append_contingency(sim::plan_t& result_plan, double duration);

//                 bool is_left_handed();

//               protected:

//                 double* joints_;
//                 double gripper_;
//                 double* controls_;
//                 double gripper_control;

//                 double wrist_to_end_effector_distance;

//                 std::vector<std::string> colliding_bodies;
//                 std::vector<std::string> full_colliding_bodies;

//                 sim::state_t* prev_st;
//                 sim::state_t* inter_st;

//                 double MAX_IK_STEP;

//               private:
//                 kinematic_model_t kinematic_model;

//                 bool _is_left_handed;
//             };
//         }
//     }
// }

// #endif

#pragma once

#ifndef PRX_BAXTER_HPP
#define	PRX_BAXTER_HPP

#include "prx/utilities/definitions/defs.hpp"

#include "simulation/systems/plants/manipulator.hpp"

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {

            /**
             * 
             */
            class baxter_t : public manipulator_t
            {

              public:
                baxter_t();

                virtual ~baxter_t(){ }

                /** @copydoc plant_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*) */
                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

              protected:

                virtual void create_spaces();
            };
        }

    }
}

#endif