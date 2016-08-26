/**
 * @file particle_controller.cpp 
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

#include "simulation/particle_controller.hpp"
#include "prx/utilities/definitions/random.hpp"
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(prx::packages::conformant::particle_controller_t, prx::sim::system_t);


namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {
        namespace conformant
        {
            particle_controller_t::particle_controller_t()
            {
                state_size = 0;
                number_of_states = 0;
            }

            particle_controller_t::~particle_controller_t() 
            {
                if(state_memory.size()>0)
                {
                    foreach(double* s, state_memory)
                    {
                        delete s;
                    }
                }
            }

            void particle_controller_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
            {
                controller_t::init(reader, template_reader);
                number_of_states = parameters::get_attribute_as<unsigned> ("number_of_states", reader, template_reader, 10);
                control_variance = parameters::get_attribute_as<std::vector<double> > ("control_variance", reader, template_reader);
                failure_rate = parameters::get_attribute_as<double> ("failure_rate", reader, template_reader,0);
                PRX_ASSERT(number_of_states>1);
                child_system = subsystems.begin()->second;
                child_state_space = child_system->get_state_space();
                stored_state = child_state_space->alloc_point();
                std::string old_name = child_state_space->get_space_name();
                state_size = child_state_space->get_dimension();
                temp_state.resize(state_size);
                std::string controller_space_name = "";
                unsigned new_size = 0;
                for(unsigned i=1;i<number_of_states;++i)
                {
                    if(i!=1)
                        controller_space_name += "|";
                    controller_space_name += old_name;
                    new_size+=state_size;
                }
                state_memory.resize(new_size);
                for(unsigned i=0;i<new_size;++i)
                {
                    state_memory[i] = new double;
                }
                controller_state_space = new space_t(controller_space_name, state_memory);
                construct_spaces();
                input_control_space = output_control_space;
                sample_control = input_control_space->alloc_point();
                stored_control = input_control_space->alloc_point();

                //set the bounds
                const std::vector<bounds_t*> plant_bounds = child_state_space->get_bounds();
                const std::vector<bounds_t*> controller_bounds = controller_state_space->get_bounds();
                for(unsigned i=1;i<number_of_states;i++)
                {
                    double low,high;
                    for(unsigned j=0;j<state_size;j++)
                    {
                        plant_bounds[j]->get_bounds(low,high);
                        controller_bounds[(i-1)*state_size+j]->set_bounds(low,high);
                    }
                }
            }

            void particle_controller_t::propagate(const double simulation_step)
            {
                if( active )
                {
                    input_control_space->copy_to_point(stored_control);
                    child_state_space->copy_to_point(stored_state);
                    //take original state sequences and have the plant propagate them.
                    for(unsigned i=1;i<number_of_states;i++)
                    {
                        //set the plant state for this particle
                        for(unsigned j=0;j<state_size;j++)
                        {
                            temp_state[j] = *state_memory[(i-1)*state_size+j];
                        }
                        child_state_space->set_from_vector(temp_state);
                        //generate a noisy control
                            
                        for(unsigned j=0;j<input_control_space->get_dimension();j++)
                        {
                            sample_control->at(j) = stored_control->at(j) + uniform_random(-.5,.5)*control_variance[j];
                        }
                        if(uniform_random()<failure_rate)
                        {
                            sample_control->at(uniform_int_random(0,input_control_space->get_dimension()-1)) = 0;
                        }
                        input_control_space->copy_from_point(sample_control);
                        //propagate forward for this particle
                        child_system->propagate(simulation_step);
                        //store the result
                        for(unsigned j=0;j<state_size;j++)
                        {
                            *state_memory[(i-1)*state_size+j] = (*child_state_space)[j];
                        }
                    }
                    //propagate the original point
                    child_state_space->copy_from_point(stored_state);
                    for(unsigned j=0;j<input_control_space->get_dimension();j++)
                    {
                        sample_control->at(j) = stored_control->at(j) + uniform_random(-.5,.5)*control_variance[j];
                    }
                    if(uniform_random()<failure_rate)
                    {
                        sample_control->at(uniform_int_random(0,input_control_space->get_dimension()-1)) = 0;
                    }
                    input_control_space->copy_from_point(sample_control);
                    child_system->propagate(simulation_step);
                }   
            }

            void particle_controller_t::compute_control()
            {
                if( active )
                {
                    child_system->compute_control();
                }
            }

            void particle_controller_t::verify() const
            {
                controller_t::verify();
            }
        }
    }
}
