/**
 * @file LQR_controller.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2012, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Zakary Littlefield, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, James Marble, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/simulation/dynamics/systems/controllers/LQR_controller/LQR_controller.hpp"
#include "prx/simulation/dynamics/systems/plant.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include <boost/range/adaptor/map.hpp> //adaptors


#include <boost/tuple/tuple.hpp> // boost::tie
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::LQR_controller_t, prx::system_t)

PRX_START

LQR_controller_t::LQR_controller_t() { }

LQR_controller_t::~LQR_controller_t() { }

void LQR_controller_t::init(const parameter_reader_t * reader)
{
#ifdef OCTAVE_FOUND
    simple_controller_t::init(reader);

    std::string plant_path = reader->get_attribute_as<std::string>("plant_path");
    
    octave_caller_t test;
    octave_value_list args;

    int state_size = state_space->get_dimension();
    int control_size = input_control_space->get_dimension();

    Matrix A(state_size, state_size);
    Matrix B(state_size, control_size);
    Matrix Q(state_size, state_size);
    Matrix R(control_size, control_size);
    
    goal_state = reader->get_attribute_as<vector_t > ("goal_state");
    
    state_t* point = this->get_system(plant_path).get()->pull_state_space()->alloc_point();
    this->get_system(plant_path).get()->pull_state_space()->set_from_vec(point,goal_state);
    control_t* control = this->get_system(plant_path).get()->pull_control_space()->alloc_point();
    this->get_system(plant_path).get()->pull_control_space()->zero(control);
    
    
    vector_t read_vector;
    read_vector = dynamic_cast<plant_t*>(this->get_system(plant_path).get())->get_diff_wrt_state(point,control);
    for( int i = 0; i < state_size; i++ )
    {
        for( int j = 0; j < state_size; j++ )
        {
            A(i, j) = read_vector[i * state_size + j];
        }
    }
    read_vector = dynamic_cast<plant_t*>(this->get_system(plant_path).get())->get_diff_wrt_control(point,control);
    for( int i = 0; i < state_size; i++ )
    {
        for( int j = 0; j < control_size; j++ )
        {
            B(i, j) = read_vector[i * control_size + j];
        }
    }
    read_vector = reader->get_attribute_as<vector_t > ("Q");
    for( int i = 0; i < state_size; i++ )
    {
        for( int j = 0; j < state_size; j++ )
        {
            Q(i, j) = read_vector[i * state_size + j];
        }
    }
    read_vector = reader->get_attribute_as<vector_t > ("R");
    for( int i = 0; i < control_size; i++ )
    {
        for( int j = 0; j < control_size; j++ )
        {
            R(i, j) = read_vector[i * control_size + j];
        }
    }

    args(0) = A;
    args(1) = B;
    args(2) = Q;
    args(3) = R;
    
    PRX_INFO_S("\n"<<A<<B<<Q<<R);

    return_val = test.call_function("LQR_Octave", args);
    PRX_DEBUG_S("\n"<<return_val(0).matrix_value()<<" \n\n"<<return_val(1).matrix_value());
#endif
}

void LQR_controller_t::push_control(const control_t * const source)
{
#ifdef OCTAVE_FOUND
    const space_t* subspace = subsystem->pull_state_space();
    state_t* substate = subsystem->pull_state();
    unsigned int state_size = state_space->get_dimension();
    unsigned int control_size = input_control_space->get_dimension();

    Matrix inner_state(state_size, 1);
    Matrix inner_control(control_size, 1);

    for( unsigned int i = 0; i < state_size; i++ )
    {
        inner_state(i, 0) = subspace->get_element(substate, i) - goal_state[i];
    }

    inner_control = -1 * return_val(0).matrix_value()*(inner_state);

    for( unsigned int i = 0; i < control_size; i++ )
        input_control_space->set_element(subsystem_control, i, inner_control(i, 0));

    subsystem->push_control(subsystem_control);
#endif
}

void LQR_controller_t::set_param(const std::string& parameter_name, const boost::any& value) {
 }

PRX_FINISH

