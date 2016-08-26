/**
 * @file grasp_data.cpp
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

#include "planning/modules/grasp_data.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"

#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {
        namespace manipulation
        {

            grasp_data_t::grasp_data_t()
            {
                state_space = NULL;
                control_space = NULL;
                releasing_state = NULL;
                grasping_state = NULL;
                retracted_open_state = NULL;
                retracted_closed_state = NULL;
                validity_checker = NULL;
                relative_grasp = new grasp_t();
            	clear();
            }

            grasp_data_t::grasp_data_t(const space_t* state_space, const space_t* control_space, const plan::validity_checker_t* new_checker)
            {
            	setup(state_space, control_space, new_checker);

            }

            grasp_data_t::~grasp_data_t()
            {
                clear();
                delete relative_grasp;                
            }

            void grasp_data_t::setup(const space_t* in_state_space, const space_t* in_control_space, const plan::validity_checker_t* new_checker)
            {
                if(state_space != in_state_space)
                {
                    if(state_space != NULL)
                    {
                        state_space->free_point(releasing_state);
                        state_space->free_point(grasping_state);
                        state_space->free_point(retracted_open_state);
                        state_space->free_point(retracted_closed_state);
                        releasing_state = NULL;
                        grasping_state = NULL;
                        retracted_open_state = NULL;
                        retracted_closed_state = NULL;
                    }
                    state_space = in_state_space;

                    releasing_state = state_space->alloc_point();
                    grasping_state = state_space->alloc_point();
                    retracted_open_state = state_space->alloc_point();
                    retracted_closed_state = state_space->alloc_point();
                }

                if(control_space != in_control_space)
                {
                    reaching_plan.link_control_space(in_control_space);
                    retracting_plan.link_control_space(in_control_space);
                    connection_plan.link_control_space( in_control_space );
                    control_space = in_control_space;
                }

                if(validity_checker != new_checker)
                {
                    validity_checker = new_checker;
                    if (validity_checker != NULL)
                    {
                        open_reaching_constraints = validity_checker->alloc_constraint();
                        closed_reaching_constraints = validity_checker->alloc_constraint();
                        open_retracting_constraints = validity_checker->alloc_constraint();
                        closed_retracting_constraints = validity_checker->alloc_constraint();

                        connection_constraints = validity_checker->alloc_constraint();
                    }
                    else
                    {
                        PRX_DEBUG_COLOR("Validity checker is NULL", PRX_TEXT_MAGENTA);
                    }
                }
                
            }

            void grasp_data_t::copy_from_data(grasp_data_t* input_grasp_data)
            {
                //If the spaces are not identical it is not safe to copy data.
                if(state_space != NULL && state_space != input_grasp_data->state_space)
                {
                    PRX_FATAL_S("Trying to copy data generated in "<<input_grasp_data->state_space->get_space_name()<<" to a data in "<<state_space->get_space_name());
                }

                //Setup the current data again
                setup(input_grasp_data->state_space, input_grasp_data->control_space, input_grasp_data->validity_checker);
                state_space->copy_point(releasing_state, input_grasp_data->releasing_state);
                state_space->copy_point(grasping_state, input_grasp_data->grasping_state);
                state_space->copy_point(retracted_open_state, input_grasp_data->retracted_open_state);
                state_space->copy_point(retracted_closed_state, input_grasp_data->retracted_closed_state);

                //Copy over the plans
                reaching_plan = input_grasp_data->reaching_plan;
                retracting_plan = input_grasp_data->retracting_plan;
                connection_plan = input_grasp_data->connection_plan;

                //Clear the Constraints
                open_reaching_constraints->clear();
                closed_reaching_constraints->clear();
                open_retracting_constraints->clear();
                closed_retracting_constraints->clear();

                //Merge in the new constraints
                open_reaching_constraints->merge(input_grasp_data->open_reaching_constraints);
                closed_reaching_constraints->merge(input_grasp_data->closed_reaching_constraints);
                open_retracting_constraints->merge(input_grasp_data->open_retracting_constraints);
                closed_retracting_constraints->merge(input_grasp_data->closed_retracting_constraints);




            }

            void grasp_data_t::clear()
            {
                if(state_space != NULL)
                {
                    state_space->free_point(releasing_state);
                    state_space->free_point(grasping_state);
                    state_space->free_point(retracted_open_state);
                    state_space->free_point(retracted_closed_state);
                    releasing_state = NULL;
                    grasping_state = NULL;
                    retracted_open_state = NULL;
                    retracted_closed_state = NULL;
                }
                state_space = NULL;
                control_space = NULL;

                //We don't have to delete the points because both plans and trajectories have the state space 
                //and the control space stored internally. When you link a new state or control space they both
                //going to delete the points and initialize new ones.                
                reaching_plan.clear();
                retracting_plan.clear();
                connection_plan.clear();
                relative_grasp->clear();
                
                /** Delete the constraints **/
                if (validity_checker != NULL)
                {
                    validity_checker->free_constraint(open_reaching_constraints);
                    validity_checker->free_constraint(closed_reaching_constraints);
                    validity_checker->free_constraint(open_retracting_constraints);
                    validity_checker->free_constraint(closed_retracting_constraints);
                    validity_checker->free_constraint(connection_constraints);
                }

                validity_checker =  NULL;
            }

            std::string grasp_data_t::print(unsigned prec)
            {
                std::stringstream out(std::stringstream::out);
                out << "Releasing state       : " << state_space->print_point(releasing_state, prec) << std::endl;
                out << "Grasping state        : " << state_space->print_point(grasping_state, prec) << std::endl;
                out << "Retracted open state  : " << state_space->print_point(retracted_open_state, prec) << std::endl;
                out << "Retracted close state : " << state_space->print_point(retracted_closed_state, prec) << std::endl;
                out << "Reaching plan: " << std::endl;
                out << reaching_plan.print(prec);
                out << "Retracting plan: " << std::endl;
                out << retracting_plan.print(prec);
                
                return out.str();
            }
        }
    }
}
