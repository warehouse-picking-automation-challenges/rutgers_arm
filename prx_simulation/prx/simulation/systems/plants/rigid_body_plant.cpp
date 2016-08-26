/**
 * @file rigid_body_plant.cpp
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

#include "prx/simulation/systems/plants/rigid_body_plant.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/simulation/plan.hpp"

#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::rigid_body_plant_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    namespace sim
    {
        rigid_body_plant_t::rigid_body_plant_t() : kinematic_plant_t()
        {
            state_memory = {&_x,&_y,&_theta};
            control_memory = {&_Cx,&_Cy,&_Ctheta};

            state_space = new space_t("SE2", state_memory);
            input_control_space = new space_t("SE2", control_memory);

            _z = 0;
        }

        void rigid_body_plant_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            kinematic_plant_t::init(reader, template_reader);

            _z = parameters::get_attribute_as<double>("z", reader, template_reader, 1.5);
        }

        void rigid_body_plant_t::update_phys_configs(config_list_t& configs, unsigned& index) const
        {
            root_config.set_position(_x, _y, _z);
            root_config.set_xyzw_orientation(0.0, 0.0, sin(_theta / 2.0), cos(_theta / 2.0));
            plant_t::update_phys_configs(configs, index);
        }

        void rigid_body_plant_t::update_collision_info()
        {
            root_config.set_position(_x, _y, _z);
            root_config.set_xyzw_orientation(0.0, 0.0, sin(_theta / 2.0), cos(_theta / 2.0));
            plant_t::update_collision_info();
        }


        void rigid_body_plant_t::append_contingency(plan_t& result_plan, double duration)
        {
            double difference = duration - result_plan.length();
            //    PRX_DEBUG_S("Difference in append: "<<difference);
            PRX_ASSERT(difference >= 0);
            if( result_plan.size() == 0 )
            {
                state_t* state = state_space->alloc_point();
                result_plan.copy_onto_back(state, 0.0);
                state_space->free_point(state);
            }
            result_plan.back().duration += difference;
        }


    }
}

