/**
 * @file bullet_plant.hpp 
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#ifdef BULLET_FOUND

#ifndef BT_USE_DOUBLE_PRECISION
#define BT_USE_DOUBLE_PRECISION
#endif

#pragma once

#ifndef PRX_BULLET_PLANT
#define PRX_BULLET_PLANT

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/plants/plant.hpp"
#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"


namespace prx
{
    namespace util
    {
        class parameter_reader_t;
    }
    
    namespace sim
    {
        class bullet_control_t;
        class bullet_constraint_t;
        /**
         * A general plant for Bullet systems. 
         * 
         * @brief <b> A general plant for Bullet systems.  </b>
         * 
         * @author Zakary Littlefield
         */
        class bullet_plant_t : public plant_t
        {

          public:
            bullet_plant_t();
            virtual ~bullet_plant_t();

            /**
             * @copydoc plant_t::init()
             *
             * Reads in all of the information needed to specify the physical
             * plant in terms that Bullet understands: rigid bodies, joints, and
             * controls.
             */
            void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /**
             * @copydoc plant_t::update_phys_configs()
             *
             * Performs the update according to the actual internal state of Bullet.
             */
            void update_phys_configs(util::config_list_t& configs,unsigned& index) const;
            /**
             * @copydoc plant_t::verify()
             *
             * Makes an additional check to see if any rigid bodies have actually
             * been specified for Bullet to simulate.
             */
            void verify() const;

            /**
             * @copydoc plant_t::propagate()
             * 
             * This function actually only provides the force accumulators for
             * the rigid bodies which comprise it.  The actual world propagation
             * happens at a more global scope with Bullet stepping functions.
             */
            void propagate(const double simulation_step = 0);

            /**
             * Activate/inactivate the system. Set the \c in_active parameter to the 
             * \c active variable.
             * 
             * @brief Activate/inactivate the system.
             * 
             * @param in_active The value to activate the system. True to activate
             * the system or False to deactivate it. 
             * @path path Is the path that we want to activate/deactivate in the simulation.
             */
            virtual void set_active(bool in_active, const std::string& path = "");

            void add_bodies(std::vector<std::pair<btCollisionShape*,btRigidBody*> >& global_list);

            void add_constraints(btDynamicsWorld* world);

            void update_to_bullet();

            void update_from_bullet();

            void recreate_bodies();

          protected:
            virtual void update_vis_info() const;

            std::vector< const util::parameter_reader_t* > joint_readers;
            /** @brief A list of all controls which directly apply forces/torques to rigid bodies. */
            std::vector< bullet_control_t* > body_controls;

            std::vector< bullet_constraint_t* > body_constraints;

            std::vector<std::pair<btCollisionShape*,btRigidBody*> > rigid_bodies;


            /** @brief Storage for each rigid body space, used to create the plant's state space. */
            std::vector< const util::space_t* > body_spaces;

            /** @brief Storage for each control, used to create the plant's control space. */
            std::vector< const util::space_t* > body_control_spaces;

            /** @brief Storage for the sub-body state. */
            std::vector< std::vector<double*> > body_state_memory;

            /** @brief Storage for the sub-body state points. */
            std::vector< std::vector<double*> > body_control_memory;

            unsigned joint_state_count;

        };

    }
}

#endif
#endif