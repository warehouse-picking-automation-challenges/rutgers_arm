/**
 * @file hand_off_sampler.cpp
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

#include "planning/modules/hand_off_sampler.hpp"
#include "../../../manipulation/planning/modules/samplers/manip_sampler.hpp"

#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"

#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>

 PLUGINLIB_EXPORT_CLASS(prx::packages::multi_arm::hand_off_sampler_t, prx::plan::sampler_t)

 namespace prx
 {
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace multi_arm
        {
            hand_off_sampler_t::hand_off_sampler_t()
            {
                ready = false;

                max_tries = 4;

                //And we should be able to show that things are indeed not here
                first_grasped_state = NULL;
                second_grasped_state = NULL;
                first_released_state = NULL;
                second_released_state = NULL;
                first_retracted_state = NULL;
                second_retracted_state = NULL;
                first_plan = NULL;
                first_approach_plan = NULL;
                first_path = NULL;
                second_plan = NULL;
                second_approach_plan = NULL;
                second_path = NULL;
            }

            hand_off_sampler_t::~hand_off_sampler_t()
            {
                //If we have allocated the one thing, everything else must be as well
                if( first_grasped_state != NULL )
                {
                    transfer_spaces[0]->free_point( first_grasped_state );
                    transfer_spaces[0]->free_point( second_grasped_state );

                    move_spaces[0]->free_point( first_released_state );
                    move_spaces[0]->free_point( second_released_state );
                    move_spaces[0]->free_point( first_retracted_state );
                    move_spaces[0]->free_point( second_retracted_state );

                    full_space->free_point( collision_check_state );
                    full_space->free_point( saved_state );

                    delete first_plan;
                    delete first_approach_plan;
                    delete first_path;
                    delete second_plan;
                    delete second_approach_plan;
                    delete second_path;
                }
            }

            void hand_off_sampler_t::sample(const util::space_t* space, util::space_point_t* point)
            {
                //Should I just be sampling arm configs here?
            }

            void hand_off_sampler_t::sample_near(const util::space_t* space, util::space_point_t* near_point, std::vector<util::bounds_t*>& bounds, util::space_point_t* point)
            {
                //Uhhhh no?
            }

            const grasp_data_t* hand_off_sampler_t::get_first_grasp()
            {
                //Just in case things are swapped around due to grasping stuff not finishing, relink
                first_grasp_data.grasped = first_grasped_state;
                first_grasp_data.released = first_released_state;
                first_grasp_data.retracted = first_retracted_state;
                first_grasp_data.retract_plan = first_plan;
                first_grasp_data.approach_plan = first_approach_plan;
                first_grasp_data.path = first_path;
                first_grasp_data.grasp_id = first_grasp_id;

                return &first_grasp_data;
            }

            const grasp_data_t* hand_off_sampler_t::get_second_grasp()
            {
                //Just in case things are swapped around due to grasping stuff not finishing, relink
                second_grasp_data.grasped = second_grasped_state;
                second_grasp_data.released = second_released_state;
                second_grasp_data.retracted = second_retracted_state;
                second_grasp_data.retract_plan = second_plan;
                second_grasp_data.approach_plan = second_approach_plan;
                second_grasp_data.path = second_path;
                second_grasp_data.grasp_id = second_grasp_id;

                return &second_grasp_data;
            }

            bool hand_off_sampler_t::sample_grasp( unsigned manip_index, const space_point_t* object_point, double retraction_distance )
            {
                bool grasp_found = false;
                unsigned tries = 0;
                config_t tmp_config;
                config_t retraction_config;

                if( !verify() )
                {
                    PRX_WARN_S("Hand-off sampler does not verify!  Have not linked in the correct information.");
                    return false;
                }
                //Make sure we use appropriate contexts
                std::string old_context = model->get_current_context();

                //Make sure the manipulation sampler is linked to the correct manipulator
                manipulation_sampler->link_info( manipulators[manip_index], move_spaces[manip_index], object_space, transfer_spaces[manip_index] );

                //Copy the state data to the state to check
                full_space->copy_to_point( collision_check_state );
                // PRX_DEBUG_COLOR("Looking for grasp for state: " << full_space->print_memory(3), PRX_TEXT_LIGHTGRAY);

                while( !grasp_found && tries < max_tries )
                {
                    ++tries;
                    retraction_config.set_position( 0, 0, -retraction_distance );
                    retraction_config.set_xyzw_orientation( 0, 0, 0, 1 );
                    //Need to switch to the appropriate context every time we try
                    model->use_context( transfer_context_names[manip_index] );
                    //Sample a grasp using the manip sampler
                    if( manipulation_sampler->sample_near_object( first_grasped_state, object_point ) )
                    {
                        //Grasp Generated, need to generate all of the relevant points
                        transfer_spaces[manip_index]->copy_from_point( first_grasped_state );
                        //And generate the released point
                        model->use_context(move_context_names[manip_index]);
                        move_spaces[manip_index]->copy_to_point(first_released_state);
                        //A bit hacky here, but ensure the state is non-grasping
                        first_released_state->memory.back() = 0;
                        move_spaces[manip_index]->copy_from_point( first_released_state );
                        //Compute an end-effector offset configuration which is retracted away from the object
                        manipulators[manip_index]->get_end_effector_configuration( tmp_config );
                        retraction_config.relative_to_global( tmp_config );

                        // PRX_DEBUG_COLOR("End Effector: " << tmp_config.print(), PRX_TEXT_RED);
                        // PRX_DEBUG_COLOR("Retracted Pt: " << retraction_config.print(), PRX_TEXT_GREEN);

                        grasp_found = true;
                        //Now, for each of the states we need to check against
                        for( unsigned i=0; i<states_to_check.size() && grasp_found; ++i )
                        {
                            //Load in the state we want to check
                            full_space->copy_from_point(states_to_check[i]);
                            //Overwrite the object state to be this specific grasp
                            object_space->copy_from_point( object_point );
                            //Then go back to the move context
                            model->use_context( move_context_names[manip_index] );

                            //Clear out the first_path and first_plan
                            first_path->clear();
                            first_plan->clear();

                            //Link up the spaces.  Why was this not breaking before?
                            first_plan->link_control_space( model->get_control_space() );
                            first_plan->link_state_space( move_spaces[manip_index] );
                            first_path->link_space( move_spaces[manip_index] );

                            //First check we have to do is in the transfer space
                            model->use_context( transfer_context_names[manip_index] );
                            //Check that the grasped state itself is valid
                            if( model->valid_state( first_grasped_state ) )
                            {
                                //And if that is the case, we need to check the retraction in the move space
                                model->use_context( move_context_names[manip_index] );

                                //If there is no valid retraction, our grasp has failed...
                                if( !valid_retraction(manip_index, first_plan, first_path, first_released_state, retraction_config) )
                                {
                                    grasp_found = false;
                                }
                            }
                            //Grasped state itself isn't even valid: failure
                            else
                            {
                                grasp_found = false;
                            }
                        }
                        //Then, if there was indeed a grasp found
                        if( grasp_found )
                        {
                            //Save the retracted point
                            model->use_context( move_context_names[manip_index] );
                            move_spaces[manip_index]->copy_point( first_retracted_state, first_path->back() );

                            //Have to fill the approach path with the reversed retraction plan
                            first_approach_plan->reverse_plan( *first_plan );
                        }
                    }
                }

                //Now... restore the state
                full_space->copy_from_point( collision_check_state );

                //Restore the context
                model->use_context(old_context);

                //And report if we win
                return grasp_found;
            }

            bool hand_off_sampler_t::sample_handoff( unsigned first_manip, unsigned second_manip, unsigned grasp_index, const space_point_t* object_point, double grasp_retraction_distance )
            {
                //Check that the sampler has been setup properly
                if( !verify() )
                {
                    PRX_WARN_S("Hand-off sampler does not verify!  Have not linked in the correct information.");
                    return false;
                }

                //Also, check to see if we even got a valid point
                if( !pose_satisfies_safety( object_point ) )
                {
                    // PRX_DEBUG_COLOR("Pose does not satisfy safety...", PRX_TEXT_RED);
                    return false;
                }

                //Flag indicating success
                bool found = false;

                //Store context
                std::string old_context = model->get_current_context();

                //Make sure to copy in the point location, and keep everyone still
                object_space->copy_from_point( object_point );
                set_zero_control();
                //Set a different grasp z for each sample
                first_grasp_id = grasp_index%3;
                manipulation_sampler->set_grasp_z( grasp_zs[first_grasp_id] );

                //Sanity check: make sure the model is in the right context?
                model->use_context( move_context_names[first_manip] );

                //Then, we need to sample a grasp
                if( sample_grasp( first_manip, object_point, grasp_retraction_distance ) )
                {
                    //First thing's first, let's ensure the other manipulator is in its grasp position (released state for max safety)
                    move_spaces[first_manip]->copy_from_point( first_released_state );
                    model->use_context( move_context_names[second_manip] );

                    //things are going well, swap role of primary and secondary grasp stuff
                    swap_grasp_data();

                    //Set a different grasp z for each sample
                    first_grasp_id = (grasp_index+1+(grasp_index%2))%3;
                    manipulation_sampler->set_grasp_z( grasp_zs[first_grasp_id] );

                    //Furthermore, ensure nobody is moving!
                    set_zero_control();

                    if( sample_grasp( second_manip, object_point, grasp_retraction_distance ) )
                    {
                        //Let's swap the data again so that the first data corresponds to the first manipulator we used
                        swap_grasp_data();

                        //Ensure we put the second manip at the grasp location && the object at its place
                        move_spaces[second_manip]->copy_from_point( second_released_state );
                        object_space->copy_from_point( object_point );
                        //Then, ensure nobody is moving!
                        set_zero_control();

                        //Have to be in the context of the first manip
                        model->use_context( move_context_names[first_manip] );

                        //Then, if the first path is still valid
                        if( validity_checker->is_valid( *first_path ) )
                        {
                            //Report success
                            found = true;
                        }
                    }
                }

                //Restore the old context
                model->use_context( old_context );

                return found;
            }

            bool hand_off_sampler_t::pose_satisfies_safety( const space_point_t* object_pose )
            {
                //Remember what context we were in
                std::string old_context = model->get_current_context();

                //Store the state
                full_space->copy_to_point( saved_state );

                //Move all the arms to safe
                go_to_start();
                //Then, place the object at the specified pose
                object_space->copy_from_point( object_pose );
                //And copy this state to a temporary storage
                full_space->copy_to_point( collision_check_state );
                //Get the result of validity checking, but make sure we are in the right context
                model->use_context("full_space");
                bool valid = model->valid_state( collision_check_state );

                //Restore the global state
                full_space->copy_from_point( saved_state );

                //Restore the context
                model->use_context( old_context );

                return valid;
            }

            bool hand_off_sampler_t::valid_retraction(int manip_index, plan_t* plan, trajectory_t* path, const space_point_t* start, config_t & goal_config)
            {
                //Some local variables
                bool retractable = false;
                config_t start_config;

                //Store the current context
                std::string old_context = model->get_current_context();

                //Ensure we get the config at the start state
                move_spaces[manip_index]->copy_from_point( start );
                manipulators[manip_index]->get_end_effector_configuration( start_config );

                //Make sure we are in the movement context while doing IK steering
                model->use_context( move_context_names[manip_index] );
                if( manipulators[manip_index]->IK_steering( start_config, goal_config, *plan ) )
                {
                    //Here, we need to add to the retraction plan a bit
                    plan->copy_onto_front( start, sim::simulation::simulation_step );
                    move_specifications[manip_index]->local_planner->propagate(start, *plan, *path);
                    if( path->size() != 0 && validity_checker->is_valid(*path) )
                    {
                        retractable = true;
                    }
                }
                //Restore the context
                model->use_context( old_context );

                return retractable;
            }


            void hand_off_sampler_t::go_to_start()
            {
                //Okay fine, things
                std::string old_context = model->get_current_context();
                std::vector< double > v;

                //Go to the start AND make sure we don't move
                full_space->copy_from_point( global_start_state );
                for( unsigned i=0; i<manipulators.size(); ++i )
                {
                    model->use_context( move_context_names[i] );
                    move_spaces[i]->copy_to_vector( v );
                    model->get_control_space()->set_from_vector( v );
                }

                //Restorate
                model->use_context(old_context);
            }

            void hand_off_sampler_t::set_zero_control()
            {
                std::vector<double> v;
                //For each manipulator, set its control to its state
                for( unsigned i=0; i < manipulators.size(); ++i )
                {
                    model->use_context( move_context_names[i] );
                    move_spaces[i]->copy_to_vector( v );
                    model->get_control_space()->set_from_vector( v );
                }
            }

            void hand_off_sampler_t::swap_grasp_data()
            {
                //Locals
                space_point_t* hold_point;
                plan_t* hold_plan;
                trajectory_t* hold_path;

                //grasp states
                hold_point = first_grasped_state;
                first_grasped_state = second_grasped_state;
                second_grasped_state = hold_point;
                //released states
                hold_point = first_released_state;
                first_released_state = second_released_state;
                second_released_state = hold_point;
                //retracted states
                hold_point = first_retracted_state;
                first_retracted_state = second_retracted_state;
                second_retracted_state = hold_point;
                //paths
                hold_path = first_path;
                first_path = second_path;
                second_path = hold_path;
                //plans
                hold_plan = first_plan;
                first_plan = second_plan;
                second_plan = hold_plan;
                //other plans
                hold_plan = first_approach_plan;
                first_approach_plan = second_approach_plan;
                second_approach_plan = hold_plan;

                //Grasp ids
                unsigned i = first_grasp_id;
                first_grasp_id = second_grasp_id;
                second_grasp_id = i;
            }

            void hand_off_sampler_t::link_manipulators( const std::vector< manipulator_plant_t* > inmanips )
            {
                manipulators = inmanips;
            }

            void hand_off_sampler_t::link_object_space( space_t* inspace )
            {
                object_space = inspace;
            }

            void hand_off_sampler_t::link_move_names( const std::vector< std::string >& innames )
            {
                move_context_names = innames;
            }

            void hand_off_sampler_t::link_transfer_names( const std::vector< std::string >& innames )
            {
                transfer_context_names = innames;
            }

            void hand_off_sampler_t::link_grasp_zs( const std::vector< double >& inzs )
            {
                grasp_zs = inzs;
            }

            void hand_off_sampler_t::link_start_state( util::space_point_t* start )
            {
                global_start_state = start;
            }

            void hand_off_sampler_t::link_manipulation_sampler( manip_sampler_t* insampler )
            {
                manipulation_sampler = insampler;
            }

            void hand_off_sampler_t::link_validity_checker( validity_checker_t* invc )
            {
                validity_checker = invc;
            }

            void hand_off_sampler_t::link_local_planner( local_planner_t* inlp )
            {
                local_planner = inlp;
            }

            void hand_off_sampler_t::link_specifications( const std::vector< plan::motion_planning_specification_t* > inspecs )
            {
                move_specifications = inspecs;
            }

            void hand_off_sampler_t::link_world_model( plan::world_model_t* inmodel )
            {
                model = inmodel;
            }

            bool hand_off_sampler_t::verify()
            {
                // PRX_DEBUG_COLOR("Verifying hand-off sampler...", PRX_TEXT_LIGHTGRAY);
                if(!ready)
                {
                    ready = check_for_links();
                    //Now, if we have just verified, we need to allocate some things
                    if( ready )
                    {
                        //First thing's first, let's remember whatever context things were in
                        std::string old_context = model->get_current_context();
                        // PRX_DEBUG_COLOR("\n\nHand-off Sampler Verifies!\n", PRX_TEXT_GREEN);

                        //Do some extra asserts to make sure our user isn't a dummy
                        PRX_ASSERT( manipulators.size() != 0 );
                        PRX_ASSERT( move_context_names.size() != 0 );
                        PRX_ASSERT( transfer_context_names.size() != 0 );

                        //Retrieve all of the spaces
                        for( unsigned i=0; i<move_context_names.size(); ++i )
                        {
                            model->use_context( move_context_names[i] );
                            move_spaces.push_back( model->get_state_space() );
                        }
                        for( unsigned i=0; i<transfer_context_names.size(); ++i )
                        {
                            model->use_context( transfer_context_names[i] );
                            transfer_spaces.push_back( model->get_state_space() );
                        }
                        model->use_context("full_space");
                        full_space = model->get_state_space();
                        collision_check_state = full_space->alloc_point();
                        saved_state = full_space->alloc_point();

                        //Let's make sure we save a vector with these points for easy and fast iteration
                        states_to_check.push_back( collision_check_state );
                        states_to_check.push_back( global_start_state );

                        first_grasped_state = transfer_spaces[0]->alloc_point();
                        second_grasped_state = transfer_spaces[0]->alloc_point();

                        first_released_state = move_spaces[0]->alloc_point();
                        second_released_state = move_spaces[0]->alloc_point();
                        first_retracted_state = move_spaces[0]->alloc_point();
                        second_retracted_state = move_spaces[0]->alloc_point();

                        //Make sure we're in the right context for the control spaces
                        model->use_context( move_context_names[0] );
                        //Plans and paths are also in the move context
                        first_plan = new plan_t( model->get_control_space() );
                        first_approach_plan = new plan_t( model->get_control_space() );
                        first_path = new trajectory_t( move_spaces[0] );
                        second_plan = new plan_t( model->get_control_space() );
                        second_approach_plan = new plan_t( model->get_control_space() );
                        second_path = new trajectory_t( move_spaces[0] );

                        //Then, restore context
                        model->use_context( old_context );
                    }
                    else
                    {
                        PRX_DEBUG_COLOR("manips : " << (manipulators.size() != 0 ? "true" : "false"), PRX_TEXT_LIGHTGRAY );
                        PRX_DEBUG_COLOR("objsp  : " << (object_space != NULL ? "true" : "false"), PRX_TEXT_LIGHTGRAY );
                        PRX_DEBUG_COLOR("mov_sp : " << (move_context_names.size() != 0 ? "true" : "false"), PRX_TEXT_LIGHTGRAY );
                        PRX_DEBUG_COLOR("tran_sp: " << (transfer_context_names.size() != 0 ? "true" : "false"), PRX_TEXT_LIGHTGRAY );
                        PRX_DEBUG_COLOR("graspzs: " << (grasp_zs.size() != 0 ? "true" : "false"), PRX_TEXT_LIGHTGRAY );
                        PRX_DEBUG_COLOR("gl_star: " << (global_start_state != NULL ? "true" : "false"), PRX_TEXT_LIGHTGRAY );
                        PRX_DEBUG_COLOR("man_sam: " << (manipulation_sampler != NULL ? "true" : "false"), PRX_TEXT_LIGHTGRAY );
                        PRX_DEBUG_COLOR("valid_c: " << (validity_checker != NULL ? "true" : "false"), PRX_TEXT_LIGHTGRAY );
                        PRX_DEBUG_COLOR("local_p: " << (local_planner != NULL ? "true" : "false"), PRX_TEXT_LIGHTGRAY );
                        PRX_DEBUG_COLOR("specs  : " << (move_specifications.size() != 0 ? "true" : "false"), PRX_TEXT_LIGHTGRAY );
                        PRX_DEBUG_COLOR("model  : " << (model != NULL ? "true" : "false"), PRX_TEXT_LIGHTGRAY );
                    }
                }
                return ready;
            }

            bool hand_off_sampler_t::check_for_links()
            {
                return manipulators.size() != 0 && object_space != NULL
                && move_context_names.size() != 0 && transfer_context_names.size() != 0 && grasp_zs.size() != 0
                && global_start_state != NULL && manipulation_sampler != NULL && local_planner != NULL
                && move_specifications.size() != 0 && model != NULL && validity_checker != NULL;
            }
        }
    }
}

