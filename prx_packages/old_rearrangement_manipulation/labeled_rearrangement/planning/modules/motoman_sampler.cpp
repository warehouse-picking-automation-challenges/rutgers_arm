/**
 * @file manipulation_sampler.cpp
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

#include "planning/modules/motoman_sampler.hpp"

#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::labeled_rearrangement_manipulation::motoman_sampler_t, prx::plan::sampler_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace labeled_rearrangement_manipulation
        {

            motoman_sampler_t::motoman_sampler_t()
            {
                impose_hand = false;
                is_left_arm = true;

                poses_memory.push_back(new double());
                poses_memory.push_back(new double());
                poses_memory.push_back(new double());
                poses_memory.push_back(new double());
                poses_memory.push_back(new double());
                poses_memory.push_back(new double());
                poses_memory.push_back(new double());
                poses_space = new space_t("SE3", poses_memory);
            }

            motoman_sampler_t::~motoman_sampler_t() 
            {
                poses_space->free_point(tmp_pose_state); 
            }

            void motoman_sampler_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                manip_sampler_t::init(reader,template_reader);

                if(parameters::has_attribute("relative_configuration",reader,template_reader))
                {
                     std::vector<double> config_vec = parameters::get_attribute_as<std::vector<double> >("relative_configuration",reader,template_reader);
                     relative_configuration.copy_from_vector(config_vec);
                     negative_orientation = relative_configuration.get_orientation();
                     negative_orientation = -negative_orientation;
                }  
                
                if(parameters::has_attribute("impose_hand",reader,template_reader))
                {
                    impose_hand = parameters::get_attribute_as<bool>("impose_hand", reader, template_reader);
                    is_left_arm = parameters::get_attribute_as<bool>("is_left_arm", reader, template_reader);
                    safe_state = parameters::get_attribute_as<std::vector<double> >("safe_state", reader,template_reader);
                }

                if( reader->has_attribute("poses_space") )
                    poses_space->init(reader->get_child("poses_space").get());
                else if( template_reader != NULL )
                    poses_space->init(template_reader->get_child("poses_space").get());
                else
                    PRX_FATAL_S("Missing poses space for rearrangement rearrangement task planner!");

                tmp_pose_state = poses_space->alloc_point();
            }   

            void motoman_sampler_t::link_info(manipulator_plant_t* manip, const space_t* manipulator_state_space, const space_t* object_state_space, const space_t* combined_state_space)
            {
                manip_sampler_t::link_info(manip,manipulator_state_space,object_state_space,combined_state_space);
                manip_seed_state = manip_space->alloc_point();
            }     

            void motoman_sampler_t::sample(const space_t* space, space_point_t* point)
            {
                PRX_DEBUG_COLOR("Sample",PRX_TEXT_CYAN);

                // //Local configurations
                // config_t tmp_config;
                // config_t effector_config;
                // std::vector< double > config_vector(8);

                //Sample a point
                manip_space->uniform_sample(manip_point);
                //Set the correct gripper mode
                manip_point->memory.back() = gripper_mode;
                manip_space->copy_from_point(manip_point);

                //If we are transferring, we also need to set the state of the object for this sample
                if( gripper_mode == GRIPPER_CLOSED )
                {
                    do{
                        poses_space->uniform_sample(tmp_pose_state);
                        PRX_DEBUG_COLOR("new pose: " << poses_space->print_point(tmp_pose_state,4),PRX_TEXT_LIGHTGRAY);
                    }while(!sample_near_object_with_theta(point,tmp_pose_state,0));

                    if(impose_hand)
                    {
                        impose_hand_state(manip_point);
                        manip_space->copy_from_point(manip_point);
                        space->copy_to_point(point);
                    }

                    // _manipulator->get_end_effector_configuration(effector_config);

                    // tmp_config = relative_configuration;
                    // // tmp_config.set_orientation(0,0,0,1);
                    // tmp_config.relative_to_global(effector_config);
                    // tmp_config.normalize_orientation();

                    // tmp_config.copy_to_vector(config_vector);
                    // object_space->set_from_vector(config_vector);
                }
                else
                {
                    //Sample a point
                    manip_space->uniform_sample(manip_point);
                    //Set the correct gripper mode
                    manip_point->memory.back() = gripper_mode;
                    if(impose_hand)
                        impose_hand_state(manip_point);

                    manip_space->copy_from_point(manip_point);
                    space->copy_to_point(point);
                }
                
                PRX_DEBUG_COLOR("End of Sample : " << space->print_point(point, 5),PRX_TEXT_CYAN);
            
                // manip_sampler_t::sample(space,point);
                // PRX_DEBUG_COLOR("Now impose hand: " << impose_hand << " point: " << space->print_point(point,4), PRX_TEXT_GREEN);
                // if(impose_hand)
                // {
                //     impose_hand_state(manip_point);
                //     manip_space->copy_from_point(manip_point);
                //     space->copy_to_point(point);
                // }
                // PRX_DEBUG_COLOR("The new point: " << space->print_point(point,4), PRX_TEXT_BLUE );
            }

            bool motoman_sampler_t::sample_near_object(state_t* result_point, const state_t* target_point)
            {
                PRX_DEBUG_COLOR("Sample NEAR : " << object_space->print_point(target_point,4) ,PRX_TEXT_CYAN);
                config_t relative_grasp;
                config_t object_config;
                //Set the object config to the state data
                object_config.set_position(target_point->memory[0], target_point->memory[1], target_point->memory[2]);
                object_config.set_orientation(target_point->memory[3], target_point->memory[4], target_point->memory[5], target_point->memory[6]);

                bool good_grasp = false;
                for( unsigned i = 0; i < max_tries && !good_grasp; ++i )
                {
                    //Sample a random angle to grasp the cylinder from
                    double theta = uniform_random(min_theta, max_theta) / 2;
                    PRX_DEBUG_COLOR("Try: [" << i << "]   at Angle: " << theta * 2 << "  with grasp_z: " << grasp_z, PRX_TEXT_CYAN);

                    //Set the relative grasp to be a rotation about z, as this is just a cylinder grasper
                    quaternion_t relative_orient(0, 0, sin(theta/2), cos(theta/2));
                    config_t relative_grasp = relative_configuration;
                    //Going from the object's orientation to the end-effector's orientation
                    relative_grasp.set_orientation(relative_orient * negative_orientation); 

                    //Compute the global end-effector position with the relative angle
                    relative_grasp.relative_to_global(object_config);

                    manip_space->uniform_sample(manip_seed_state);
                    good_grasp = _manipulator->IK_solver(relative_grasp, manip_point, true, manip_seed_state);
                    PRX_DEBUG_COLOR("good_grasp:" << good_grasp, PRX_TEXT_GREEN);
                    PRX_DEBUG_COLOR("relative_grasp:" << relative_grasp.print(), PRX_TEXT_GREEN);
                    PRX_DEBUG_COLOR("manip_point:  " << manip_space->print_point(manip_point, 5), PRX_TEXT_GREEN);
                    PRX_DEBUG_COLOR("target_point:  " << object_space->print_point(target_point, 5), PRX_TEXT_GREEN);
                }

                //If we ended with a successful manipulator configuration
                if( good_grasp )
                {
                    //Copy the data from the object and manipulator point
                    object_space->copy_from_point(target_point);
                    manip_space->copy_from_point(manip_point);
                    if(impose_hand)
                        impose_hand_state(manip_point);
                    //Need to set the manipulator state
                    combined_space->copy_to_point(result_point);
#ifndef NDEBUG
                    //DEBUG
                    //DEBUG: get the end-effector configuration
                    //DEBUG
                    config_t ee_config;
                    _manipulator->get_end_effector_configuration(ee_config);

                    //And setup the object config
                    config_t obj_config;
                    obj_config.set_position(target_point->memory[0], target_point->memory[1], target_point->memory[2]);

                    //Then, assert that the difference in pose isn't hueg
                    double rel_dist = obj_config.get_position().distance(relative_grasp.get_position());
                    double true_dist = ee_config.get_position().distance(relative_grasp.get_position());

                    if( true_dist > PRX_DISTANCE_CHECK || rel_dist > fabs(grasp_z)  + fabs(end_effector_distance) + PRX_DISTANCE_CHECK )
                    {
                        //So... some bad things happened here.. let's dig
                        PRX_DEBUG_COLOR("Manipulator at: " << manip_space->print_point(manip_point, 3), PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("Grasp Z : " << grasp_z, PRX_TEXT_LIGHTGRAY);
                        PRX_DEBUG_COLOR("Gives ee_config: " << ee_config.print(), PRX_TEXT_GREEN);
                        PRX_DEBUG_COLOR("For grasping at: " << obj_config.print(), PRX_TEXT_LIGHTGRAY);
                        PRX_DEBUG_COLOR("Relative grasp: " << relative_grasp.print(), PRX_TEXT_MAGENTA);
                        PRX_DEBUG_COLOR("True Dist: " << true_dist << "   Relative Dist: " << rel_dist, PRX_TEXT_LIGHTGRAY);
                        // PRX_FATAL_S("DEBUG this stuff.");
                    }
                    // << DEBUG
#endif
                    return true;
                }
                return false;
            }

            bool motoman_sampler_t::sample_near_object_with_theta(sim::state_t* result_point, const sim::state_t* target_point, double theta)
            {
                PRX_DEBUG_COLOR("Sample NEAR with theta: " << theta,PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("Object: " << object_space->print_point(target_point,5),PRX_TEXT_CYAN);
                
                config_t object_config;
                //Set the object config to the state data
                object_config.set_position(target_point->memory[0], target_point->memory[1], target_point->memory[2]);
                object_config.set_orientation(target_point->memory[3], target_point->memory[4], target_point->memory[5], target_point->memory[6]);

                quaternion_t relative_orient(0, 0, sin(theta/2), cos(theta/2));
                config_t relative_grasp = relative_configuration;
                //Going from the object's orientation to the end-effector's orientation
                relative_grasp.set_orientation(relative_orient * negative_orientation); 

                //Compute the global end-effector position with the relative angle
                relative_grasp.relative_to_global(object_config);

                bool good_grasp = _manipulator->IK_solver(relative_grasp, manip_point, true);
                PRX_DEBUG_COLOR("good grasp ? : " << good_grasp  << "      max_tries: " << max_tries ,  PRX_TEXT_BLUE);
                for( unsigned i = 0; i < max_tries && !good_grasp; ++i )
                {
                    manip_space->uniform_sample(manip_seed_state);
                    // PRX_DEBUG_COLOR(i << ") state :" << manip_space->print_point(manip_seed_state,5),PRX_TEXT_GREEN);
                    good_grasp = _manipulator->IK_solver(relative_grasp, manip_point, true, manip_seed_state);
                }

                if(good_grasp)
                {
                    //Copy the data from the object and manipulator point
                    object_space->copy_from_point(target_point);
                    manip_space->copy_from_point(manip_point);
                    if(impose_hand)
                        impose_hand_state(manip_point);
                    //Need to set the manipulator state
                    combined_space->copy_to_point(result_point);
#ifndef NDEBUG
                    config_t ee_config;
                    _manipulator->get_end_effector_configuration(ee_config);

                    //And setup the object config
                    config_t obj_config;
                    obj_config.set_position(target_point->memory[0], target_point->memory[1], target_point->memory[2]);

                    //Then, assert that the difference in pose isn't hueg
                    double rel_dist = obj_config.get_position().distance(relative_grasp.get_position());
                    double true_dist = ee_config.get_position().distance(relative_grasp.get_position());

                    if( true_dist > PRX_DISTANCE_CHECK || rel_dist > fabs(grasp_z) + fabs(end_effector_distance) + PRX_DISTANCE_CHECK )
                    {
                        //So... some bad things happened here.. let's dig
                        PRX_DEBUG_COLOR("Manipulator at: " << manip_space->print_point(manip_point, 3), PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("Grasp Z : " << grasp_z, PRX_TEXT_LIGHTGRAY);
                        PRX_DEBUG_COLOR("Gives ee_config: " << ee_config.print(), PRX_TEXT_GREEN);
                        PRX_DEBUG_COLOR("For grasping at: " << obj_config.print(), PRX_TEXT_LIGHTGRAY);
                        PRX_DEBUG_COLOR("Relative grasp: " << relative_grasp.print(), PRX_TEXT_MAGENTA);
                        PRX_DEBUG_COLOR("True Dist: " << true_dist << " > " << PRX_DISTANCE_CHECK, PRX_TEXT_LIGHTGRAY);
                        PRX_DEBUG_COLOR("Relative Dist: " << rel_dist << " > " << (fabs(grasp_z) + fabs(end_effector_distance) + PRX_DISTANCE_CHECK), PRX_TEXT_LIGHTGRAY);
                        //PRX_FATAL_S("DEBUG this stuff.");
                    }
                    PRX_DEBUG_COLOR("Got the theta that you asked: " << theta << "   for point: " << object_space->print_point(target_point,5), PRX_TEXT_MAGENTA);
                    PRX_DEBUG_COLOR("Grasped with: " << combined_space->print_point(result_point,5), PRX_TEXT_MAGENTA);
                    //PRX_ASSERT(false);
#endif                                        
                    return true;
                }

                PRX_DEBUG_COLOR("previous relative_configuration failed: " << relative_configuration,PRX_TEXT_LIGHTGRAY);
                quaternion_t q = relative_configuration.get_orientation();

                relative_grasp = relative_configuration;
                q.set(-q.get_x(), q.get_y(), -q.get_z(), q.get_w());
                relative_grasp.set_orientation(q);
                PRX_DEBUG_COLOR("we will try with different relative_configuration: " << q,PRX_TEXT_LIGHTGRAY);
                q = -q;

                //Going from the object's orientation to the end-effector's orientation
                relative_grasp.set_orientation(relative_orient * q); 

                //Compute the global end-effector position with the relative angle
                relative_grasp.relative_to_global(object_config);
                
                good_grasp = _manipulator->IK_solver(relative_grasp, manip_point, true);
                PRX_DEBUG_COLOR("good grasp ? : " << good_grasp  << "      max_tries: " << max_tries ,  PRX_TEXT_BLUE);
                for( unsigned i = 0; i < max_tries && !good_grasp; ++i )
                {
                    manip_space->uniform_sample(manip_seed_state);
                    // PRX_DEBUG_COLOR(i << ") state :" << manip_space->print_point(manip_seed_state,5),PRX_TEXT_GREEN);
                    good_grasp = _manipulator->IK_solver(relative_grasp, manip_point, true, manip_seed_state);
                }

                if(good_grasp)
                {
                    //Copy the data from the object and manipulator point
                    object_space->copy_from_point(target_point);
                    manip_space->copy_from_point(manip_point);
                    if(impose_hand)
                        impose_hand_state(manip_point);
                    //Need to set the manipulator state
                    combined_space->copy_to_point(result_point);                                      
                    return true;
                }

                return false;
                // bool flag_sample = sample_near_object(result_point,target_point);
                // if(flag_sample)
                // {
                //     PRX_DEBUG_COLOR("Got a grasp that is not the theta " << theta << " you asked... for point: " << object_space->print_point(target_point,5), PRX_TEXT_CYAN);
                //     PRX_DEBUG_COLOR("Grasped with: " << combined_space->print_point(result_point,5), PRX_TEXT_CYAN);
                // }
                // else
                // {
                //     PRX_DEBUG_COLOR(" DID NOT GET a grasp while tried the theta " << theta << " you asked... for point: " << object_space->print_point(target_point,5), PRX_TEXT_RED);
                // }
                
                // return flag_sample;
            }
        
            void motoman_sampler_t::impose_hand_state( space_point_t* target_state)
            {
                if( is_left_arm )
                {
                    for( int i=8; i<15; i++ )
                        target_state->at(i) = safe_state[i];
                }
                else
                {
                    for( int i=1; i<8; i++ )
                        target_state->at(i) = safe_state[i];
                }
           }
        }
    }
}
