/**
 * @file LQR_tree.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2011, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Zakary Littlefield, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, James Marble, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/simulation/dynamics/systems/controllers/LQR_controller/LQR_tree.hpp"
#include "prx/utilities/spaces/composite_space.hpp"

#include <pluginlib/class_list_macros.h>
#include <fstream>

PLUGINLIB_EXPORT_CLASS( prx::LQR_tree_t, prx::system_t)

PRX_START

LQR_tree_t::LQR_tree_t() { }

LQR_tree_t::~LQR_tree_t() { }

void LQR_tree_t::init(const parameter_reader_t* reader)
{
    simple_controller_t::init(reader);

    std::string plant_path = reader->get_attribute_as<std::string > ("plant_path");
    goal_state = reader->get_attribute_as<vector_t > ("goal_state");
    planning = true;
}

void LQR_tree_t::set_internal_gains(std::vector<double>& in_radii, std::vector<vector_t>& in_centers, std::vector<vector_t>& in_controls, std::vector<vector_t>& in_gains, std::vector<vector_t>& in_costs)
{

    const space_t* subspace = subsystem->pull_state_space();
    const space_t* subcontrolspace = subsystem->pull_control_space();
    unsigned int state_size = state_space->get_dimension();
    unsigned int control_size = input_control_space->get_dimension();
#ifdef OCTAVE_FOUND
    for( unsigned i = 0; i < in_radii.size(); i++ )
    {
        radii.push_back(in_radii[i]);
        state_t* state = subspace->alloc_point();
        subspace->set_from_vec(state, in_centers[i]);
        centers.push_back(state);
        
        control_t* inner_control = subcontrolspace->alloc_point();
        
        subcontrolspace->set_from_vec(inner_control,in_controls[i]);
        controls.push_back(inner_control);        
        
        Matrix gain_matrix(control_size, state_size);
        Matrix cost_matrix(state_size,state_size);

        for( unsigned row = 0; row < control_size; row++ )
        {
            for( unsigned col = 0; col < state_size; col++ )
            {
                gain_matrix(row,col) = in_gains[i][row*state_size+col];
            }
        }        
        gains.push_back(gain_matrix);
        
        for( unsigned row = 0; row < state_size; row++ )
        {
            for( unsigned col = 0; col < state_size; col++ )
            {
                cost_matrix(row,col) = in_costs[i][row*state_size+col];
            }
        }   
        costs.push_back(cost_matrix);
        
    }
    planning = false;
    
    
#endif
    
    std::ofstream fout;
    fout.open("lqr_tree.txt");
    for(unsigned i=0;i<radii.size();i++)
    {
        fout<<radii[i]<<" "<<subspace->print_point(centers[i])<<std::endl;
    }
    fout.close();
}

void LQR_tree_t::push_control(const control_t * const source)
{
    if(!planning)
    {
#ifdef OCTAVE_FOUND
        const space_t* subspace = subsystem->pull_state_space();
        state_t* substate = subsystem->pull_state();
        unsigned int state_size = state_space->get_dimension();
        unsigned int control_size = input_control_space->get_dimension();

        Matrix inner_state(state_size, 1);
        Matrix inner_control(control_size, 1);
        Matrix cost_value(1,1);

        Matrix gain_matrix(control_size, state_size);
        state_t* center_state;
        control_t* stable_control;

        bool found=false;
        //find the matrix to use
        
        double max_val = PRX_INFINITY;

        for( unsigned i = 0; i < centers.size(); i++ )
        {
            center_state = centers[i];
            double radius = radii[i];
            stable_control = controls[i];
            

            for( unsigned int i = 0; i < state_size; i++ )
            {
                inner_state(i, 0) = subspace->get_element(substate, i) - subspace->get_element(center_state,i);
            }
            
            //if( subspace->distance(substate, center_state) < radius )
            if((radius - ((inner_state.transpose())*costs[i]*inner_state)(0,0)  ) > 0 )
            {
                if(((inner_state.transpose())*costs[i]*inner_state)(0,0) < max_val  )
                {
                    max_val = ((inner_state.transpose())*costs[i]*inner_state)(0,0);
                    gain_matrix = gains[i];
                    found=true;
                }
            }
        }  

        for( unsigned int i = 0; i < state_size; i++ )
        {
            inner_state(i, 0) = subspace->get_element(substate, i) - subspace->get_element(center_state,i);
        }

        inner_control = -1 * gain_matrix * (inner_state);

        for( unsigned int i = 0; i < control_size; i++ )
            input_control_space->set_element(subsystem_control, i, inner_control(i, 0) + input_control_space->get_element(stable_control,i));

        subsystem->push_control(subsystem_control);
#endif
    }
    else
    {
        //const composite_space_point_t* ctrl;
        //ctrl = source->as<composite_space_point_t > ();    
        subsystem->push_control(source);
    }
}

/** @copydoc system_t::set_param(const std::string&, const boost::any&) */
void LQR_tree_t::set_param(const std::string& parameter_name, const boost::any& value) { }

PRX_FINISH

