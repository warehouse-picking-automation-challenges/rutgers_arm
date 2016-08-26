/**
 * @file motoman_sampler.hpp
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

#ifndef PRX_MOTOMAN_SAMPLER_HPP
#define PRX_MOTOMAN_SAMPLER_HPP

#include "../../../manipulation/planning/modules/samplers/manip_sampler.hpp"

namespace prx
{
    namespace packages
    {
        namespace labeled_rearrangement_manipulation
        {
            using namespace baxter;
            /**
             * Performs sampling for the manipulator over a plane. It will randomly compute
             * x,y and a theta, but it will return the state space point for the manipulator.
             *
             * @brief <b> Performs sampling for the manipulator over a plane.</b>
             *
             * @author Athanasios Krontiris
             */
            class motoman_sampler_t : public manipulation::manip_sampler_t
            {

              public:

                motoman_sampler_t();

                virtual ~motoman_sampler_t();

                /**
                 * @copydoc manip_sampler_t::init(const parameter_reader_t* , const parameter_reader_t*)
                 */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);

                virtual void link_info(manipulator_plant_t* manip, const util::space_t* manipulator_state_space, const util::space_t* object_state_space, const util::space_t* combined_space);

                /**
                 * @copydoc manip_sampler_t::sample(const space_t*, space_point_t*)
                 */
                virtual void sample(const util::space_t* space, util::space_point_t* point);

                /**
                 * @copydoc manip_sampler_t::sample_near(const space_t*, space_point_t*, std::vector<bounds_t*>&, space_point_t*)
                 */
                virtual bool sample_near_object(sim::state_t* result_point, const sim::state_t* target_point);

                /**
                 * @copydoc manip_sampler_t::sample_near(const space_t*, space_point_t*, std::vector<bounds_t*>&, space_point_t*)
                 */
                virtual bool sample_near_object_with_theta(sim::state_t* result_point, const sim::state_t* target_point, double theta);

              protected:

                void impose_hand_state( util::space_point_t* target_state);

                util::quaternion_t negative_orientation;
                bool impose_hand;
                bool is_left_arm;
                std::vector<double> safe_state;
                sim::state_t* manip_seed_state;

                util::space_t* poses_space;
                sim::state_t* tmp_pose_state;
                std::vector<double*> poses_memory;
            };
        }
    }
}

#endif
