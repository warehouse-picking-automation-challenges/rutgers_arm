/**
 * @file target_controller.cpp 
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "prx/simulation/systems/controllers/target_controller.hpp"

#include <boost/any.hpp>
#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>
#include <vector>


PLUGINLIB_EXPORT_CLASS(prx::sim::target_controller_t, prx::sim::system_t);


namespace prx
{
    using namespace util;

    namespace sim
    {

        target_controller_t::target_controller_t()
        {
        }

        target_controller_t::~target_controller_t() { }

        void target_controller_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            PRX_DEBUG_COLOR("Starting target_controller_t::init()...", PRX_TEXT_GREEN);

            controller_t::init(reader, template_reader);
            child_system = subsystems.begin()->second;
            child_state_space = child_system->get_state_space();

            for (int i = 0; i < child_state_space->get_dimension(); ++i)
            {
                control_memory.push_back(new double);
            }

            input_control_space = new space_t(child_state_space->get_space_name(),control_memory);

        }

        void target_controller_t::propagate(const double simulation_step)
        {
            controller_t::propagate(simulation_step);

            PRX_STATUS_S("S0["<<std::setw(7)<<std::setprecision(3)<<child_state_space->at(0)<<"]    S1["<<std::setw(7)<<std::setprecision(3)<<child_state_space->at(1)<<"]    E0["<<std::setw(7)<<std::setprecision(3)<<child_state_space->at(2)<<"]    E1["<<std::setw(7)<<std::setprecision(3)<<child_state_space->at(3)<<"]    W0["<<std::setw(7)<<std::setprecision(3)<<child_state_space->at(4)<<"]    W1["<<std::setw(7)<<std::setprecision(3)<<child_state_space->at(5)<<"]    W2["<<std::setw(7)<<std::setprecision(3)<<child_state_space->at(6)<<"]    Gripper["<<std::setw(7)<<(child_state_space->at(14)>1?"closed":"open")<<"]   ");

            // PRX_STATUS(child_state_space->print_memory(3));
        }

        void target_controller_t::compute_control()
        {
            control_t* output_control = output_control_space->alloc_point();

            for (int i = 0; i < child_state_space->get_dimension()-2; ++i)
            {
                double diff = input_control_space->at(i) - child_state_space->at(i);
                diff/=simulation::simulation_step;
                output_control->at(i) = diff;
            }
            unsigned index = child_state_space->get_dimension()-2;
            output_control->at(index) = input_control_space->at(index);
            index++;
            output_control->at(index) = input_control_space->at(index);

            output_control_space->copy_from_point(output_control);
            output_control_space->free_point(output_control);
            subsystems.begin()->second->compute_control();
        }

    }
}
