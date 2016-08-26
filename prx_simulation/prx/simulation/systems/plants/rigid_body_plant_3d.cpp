/**
 * @file rigid_body_plant_3d.cpp
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

#include "prx/simulation/systems/plants/rigid_body_plant_3d.hpp"
#include "prx/simulation/plan.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::rigid_body_plant_3d_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    namespace sim
    {
        rigid_body_plant_3d_t::rigid_body_plant_3d_t() : kinematic_plant_t()
        {
            state_memory = {&_x,&_y,&_z,&_qx,&_qy,&_qz,&_qw};
            control_memory = {&_Cx,&_Cy,&_Cz,&_Cqx,&_Cqy,&_Cqz,&_Cqw};

            state_space = new space_t("SE3", state_memory);
            input_control_space = new space_t("SE3", control_memory);
        }

        void rigid_body_plant_3d_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            kinematic_plant_t::init(reader, template_reader);
        }

        void rigid_body_plant_3d_t::update_phys_configs(config_list_t& configs, unsigned& index) const
        {
            root_config.set_position(_x, _y, _z);
            root_config.set_xyzw_orientation(_qx, _qy, _qz, _qw);
            plant_t::update_phys_configs(configs, index);
        }

        void rigid_body_plant_3d_t::append_contingency(plan_t& result_plan, double duration)
        {
            result_plan.link_control_space(input_control_space);
            result_plan.copy_onto_back(state, duration-result_plan.length());
        }

    }
}

