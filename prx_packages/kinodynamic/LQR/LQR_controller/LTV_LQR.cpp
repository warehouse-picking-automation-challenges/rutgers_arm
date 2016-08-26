/**
 * @file LTV_LQR.cpp
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

#include "prx/simulation/dynamics/systems/controllers/LQR_controller/LTV_LQR.hpp"
#include "prx/utilities/spaces/composite_space.hpp"

#include "prx/simulation/dynamics/systems/plant.hpp"
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::LTV_LQR_t, prx::system_t)

PRX_START

LTV_LQR_t::LTV_LQR_t()
{
    std::vector< std::string > names = {"time"};
    controller_state_space = new simple_space_t < 1 > (names);

    controller_state = controller_state_space->alloc_point();

    controller_state_space->zero(controller_state);
}

LTV_LQR_t::~LTV_LQR_t() { }

void LTV_LQR_t::init(const parameter_reader_t* reader)
{
#ifdef OCTAVE_FOUND
    stateful_controller_t::init(reader);

    int state_size = subsystem->pull_state_space()->get_dimension();
    int control_size = subsystem->pull_control_space()->get_dimension();

    std::string plant_path = reader->get_attribute_as<std::string > ("plant_path");

    //read a trajectory from file
    //read a plan from file
    Matrix Ai(state_size, state_size);
    Matrix Bi(state_size, control_size);

    std::string traj_file = reader->get_attribute_as<std::string > ("trajectory_file");
    std::string plan_file = reader->get_attribute_as<std::string > ("plan_file");
    double sim_step = 0.017;

    trajectory.read_from_file(this->get_system(plant_path).get()->pull_state_space(), traj_file);
    trajectory.states.push_back(this->get_system(plant_path).get()->pull_state_space()->clone_point(trajectory.states.back()));
    plan.read_from_file(this->get_system(plant_path).get()->pull_control_space(), plan_file);
    plan.steps.push_back(plan_step_t(this->get_system(plant_path).get()->pull_control_space()->alloc_point(), 3.0));
    this->get_system(plant_path).get()->pull_control_space()->zero(plan.steps.back().control);
    vector_t read_vector;

    for( unsigned num_states = 0; num_states < trajectory.states.size(); num_states++ )
    {

        read_vector = dynamic_cast<plant_t*>(this->get_system(plant_path).get())->get_diff_wrt_state(trajectory.states[num_states], plan.steps[plan.get_index_at(sim_step * num_states)].control);

        for( int i = 0; i < state_size; i++ )
        {
            for( int j = 0; j < state_size; j++ )
            {
                Ai(i, j) = read_vector[i * state_size + j];
            }
        }
        A.push_back(Ai);
        read_vector = dynamic_cast<plant_t*>(this->get_system(plant_path).get())->get_diff_wrt_state(trajectory.states[num_states], plan.steps[plan.get_index_at(sim_step * num_states)].control);
        for( int i = 0; i < state_size; i++ )
        {
            for( int j = 0; j < control_size; j++ )
            {
                Bi(i, j) = read_vector[i * control_size + j];
            }
        }
        B.push_back(Bi);

    }
    Matrix Qi(state_size, state_size);
    Matrix Ri(control_size, control_size);
    read_vector = reader->get_attribute_as<vector_t > ("Q");
    for( int i = 0; i < state_size; i++ )
    {
        for( int j = 0; j < state_size; j++ )
        {
            Qi(i, j) = read_vector[i * state_size + j];
        }
    }
    read_vector = reader->get_attribute_as<vector_t > ("R");
    for( int i = 0; i < control_size; i++ )
    {
        for( int j = 0; j < control_size; j++ )
        {
            Ri(i, j) = read_vector[i * control_size + j];
        }
    }
    Q = Qi;
    R = Ri;

    octave_caller_t caller;
    //if they exist, perform the LQR step for each state and control
    //  store the resulting matrices in the K and S vectors
    for( unsigned i = 0; i < A.size(); i++ )
    {
        octave_value_list args;
        octave_value_list ret;
        args(0) = A[i];
        args(1) = B[i];
        args(2) = Q;
        args(3) = R;
        //PRX_DEBUG_S(args(0).matrix_value() << "\n" << args(1).matrix_value() << "\n" << args(2).matrix_value() << "\n" << args(3).matrix_value());
        ret = caller.call_function("LQR_Octave", args);
        Matrix temp(ret(0).matrix_value());
        Matrix temp2(ret(1).matrix_value());

        //PRX_DEBUG_S(temp << "\n" << temp2);

        K.push_back(Matrix(temp));
        S.push_back(Matrix(temp2));
    }
#endif
}

void LTV_LQR_t::propagate(const double simulation_step)
{

    double time = state_space->get_element(state, "time");
    state_space->set_element(state, "time", time + simulation_step);
    subsystem->propagate(simulation_step);
    return;
}

void LTV_LQR_t::set_param(const std::string& parameter_name, const boost::any& value)
{
    return;
}

void LTV_LQR_t::push_control(const control_t * const source)
{
    //This copy control doesn't care about the control being passed in.
#ifdef OCTAVE_FOUND
    const space_t* subspace = subsystem->pull_state_space();
    state_t* substate = subsystem->pull_state();
    unsigned int state_size = subspace->get_dimension();
    unsigned int control_size = subsystem->pull_control_space()->get_dimension();


    Matrix inner_state(state_size, 1);
    Matrix inner_control(control_size, 1);


    double time = state_space->get_element(state, "time");
    double sim_step = 0.017;

    unsigned trajectory_index = (unsigned)std::floor(time / sim_step + .5);

    if( trajectory_index >= trajectory.states.size() )
        trajectory_index = trajectory.states.size() - 1;



    unsigned index = plan.get_index_at(time);

    for( unsigned int i = 0; i < state_size; i++ )
    {
        inner_state(i, 0) = subspace->get_element(substate, i) - subspace->get_element(trajectory.states[trajectory_index], i);
    }

    inner_control = -1 * K[index]*(inner_state);

    for( unsigned int i = 0; i < control_size; i++ )
        subsystem->pull_control_space()->set_element(subsystem_control, i, inner_control(i, 0) + subsystem->pull_control_space()->get_element(plan.steps[index].control, i));

    subsystem->push_control(subsystem_control);
#endif
    return;
}

PRX_FINISH

