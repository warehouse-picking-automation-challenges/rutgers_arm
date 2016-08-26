/**
 * @file minimal_avoidance_controller.cpp 
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "simulation/homing_controller.hpp"
#include "prx/utilities/math/2d_geometry/angle.hpp"
#include "homing_controller.hpp"
#include <pluginlib/class_list_macros.h>

#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>
PLUGINLIB_EXPORT_CLASS(prx::packages::two_dim_problems::homing_controller_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    
    namespace packages
    {
        namespace two_dim_problems
        {
           homing_controller_t::homing_controller_t()
            {
                control_memory = {&_x,&_y};
                input_control_space = new space_t("XY", control_memory);
                use_all_landmarks = true;
                // Set up direction votes to be 360 degrees, initialized to zero
                direction_votes.resize(360,0);
                up_vector.resize(3);
                up_vector.set(0.0,0.0,1.0);
                homing_direction = 0.0;
                current_angle.resize(3);
            }

            homing_controller_t::~homing_controller_t() 
            { 
            }

            void homing_controller_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
            {
                PRX_DEBUG_COLOR("Homing controller init", PRX_TEXT_GREEN);
                simple_controller_t::init(reader,template_reader);
                _z = parameters::get_attribute_as<double>("z", reader, template_reader, 1.5);
                error_threshold = parameters::get_attribute_as<double>("error_threshold", reader, template_reader, 0.1);
                vote_threshold = parameters::get_attribute_as<double>("vote_threshold", reader, template_reader, 3);
                conservative = parameters::get_attribute_as<bool>("conservative", reader, template_reader, false);
                use_all_landmarks = parameters::get_attribute_as<bool>("use_all_landmarks", reader, template_reader, true);
                use_voting_method = parameters::get_attribute_as<bool>("use_voting_method", reader, template_reader, false);
                child_state_space = subsystems.begin()->second.get()->get_state_space();
                get_state = child_state_space->alloc_point();

            }

            void homing_controller_t::compute_control()
            {
                if (use_voting_method)
                {
                    voting_method();
                }
                else
                {
                    kostas_method();
                }
                subsystems.begin()->second->compute_control();
            }
            
            // TODO: Use sensing to retrieve landmarks
            void homing_controller_t::set_landmarks(const util::config_list_t& new_landmarks)
            {
                landmarks = new_landmarks;
            }
           
            // TODO: Introduce false-positive matches
            void homing_controller_t::get_direction_to_landmark(int landmark_index, vector_t& current_direction, vector_t& goal_direction)
            {
                vector_t temp1(3);
                
                // Compute Current Direction to Landmark
                vector_t temp_current_direction = (landmarks[landmark_index].second - current_config).get_position();
                temp_current_direction.normalize();
                // Project direction
                temp1.cross_product(temp_current_direction, up_vector);
                current_direction.cross_product(up_vector, temp1);
                current_direction+=current_angle;
                current_direction.normalize();
                // Compute Goal Direction to Landmark
                temp_current_direction = (landmarks[landmark_index].second - goal_config).get_position();
                temp_current_direction.normalize();
                // Project direction
                temp1.cross_product(temp_current_direction, up_vector);
                goal_direction.cross_product(up_vector, temp1);
                goal_direction+=current_angle;
                goal_direction.normalize();
            }
            
            void homing_controller_t::update_current_and_goal()
            {
                // Update current state
                child_state_space->copy_to_point(get_state);
                current_config.set_position(get_state->at(0),get_state->at(1), _z);
                current_angle.set_at(0, cos(get_state->at(2)));
                current_angle.set_at(1, sin(get_state->at(2)));
                current_angle.set_at(2,0);
                // Update current goal
                goal_config.set_position(input_control_space->at(0), input_control_space->at(1), _z);
                PRX_DEBUG_COLOR("CURRENT THETA: " << get_state->at(2), PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("Set current config to: " << current_config, PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("Set goal config to: " << goal_config, PRX_TEXT_CYAN);
            }
            
            void homing_controller_t::kostas_method()
            {
                
                control_t* output_control = output_control_space->alloc_point();
                // Update current and goal state
                update_current_and_goal();
                // Compute new homing direction
                vector_t motion_vector(2);
                double angular_error = 0;
                unsigned count = 0;
                // Go over all landmarks
                if (use_all_landmarks)
                {
                    //PRX_DEBUG_COLOR("Will use all landmarks", PRX_TEXT_GREEN);
                    for (unsigned i = 0; i < landmarks.size(); ++i)
                    {
                        //PRX_DEBUG_COLOR("--Landmark i: " << i << ", " << landmarks[i].first, PRX_TEXT_RED);
                        for (unsigned j = 0 ; j < landmarks.size(); ++j)
                        {
                            if (j > i)
                            {
                                //PRX_DEBUG_COLOR("----Landmark j: " << j << ", " << landmarks[j].first, PRX_TEXT_BLUE);
                                // Get Direction to Landmark 1
                                vector_t current_direction1(3), goal_direction1(3);
                                get_direction_to_landmark(i, current_direction1, goal_direction1);
                                //PRX_DEBUG_COLOR("Current direction to i: " << current_direction1 << ", goal : " << goal_direction1, PRX_TEXT_CYAN);
                                
                                // Get Direction to Landmark 2
                                vector_t current_direction2(3), goal_direction2(3);
                                get_direction_to_landmark(j, current_direction2, goal_direction2);
                                //PRX_DEBUG_COLOR("Current direction to j: " << current_direction2 << ", goal : " << goal_direction2, PRX_TEXT_BROWN);
                                
                                // Compute current angle between Landmark Pair
                                double current_angle = angle_between_points(current_direction1.dot_product(current_direction2), current_direction1.norm(), current_direction2.norm());
                                // Compute goal angle between Landmark Pair
                                double goal_angle = angle_between_points(goal_direction1.dot_product(goal_direction2), goal_direction1.norm(), goal_direction2.norm());
                                // Compute angular difference
                                double delta_angle = goal_angle - current_angle;
                                //PRX_DEBUG_COLOR("CURRENT angle: " << current_angle, PRX_TEXT_LIGHTGRAY);
                                //PRX_DEBUG_COLOR("GOAL angle: " << goal_angle, PRX_TEXT_LIGHTGRAY);
                                //PRX_DEBUG_COLOR("Delta angle: " << delta_angle, PRX_TEXT_LIGHTGRAY);
                                angular_error += fabs ( current_angle - goal_angle);
                                
                                // Compute bisector
                                vector_t bisector(2);
                                bisector.set_at(0, cos(delta_angle/2));
                                bisector.set_at(1, sin(delta_angle/2));
                                //PRX_DEBUG_COLOR("Bisector: " << bisector, PRX_TEXT_CYAN);
                                
                                // Compute motion vector
                                if (delta_angle >= -1*PRX_PI && delta_angle <= PRX_PI)
                                {
                                    PRX_DEBUG_COLOR("PRE Motion Vector Case 1 : " << motion_vector, PRX_TEXT_CYAN);
                                    motion_vector += (bisector*delta_angle);
                                    PRX_DEBUG_COLOR("PRE Motion Vector Case 1 : " << motion_vector, PRX_TEXT_CYAN);
                                }
                                else if (delta_angle > PRX_PI)
                                {
                                    PRX_DEBUG_COLOR("PRE Motion Vector Case 2 : " << motion_vector, PRX_TEXT_CYAN);
                                    motion_vector += (bisector*(2*PRX_PI - delta_angle));
                                    PRX_DEBUG_COLOR("PRE Motion Vector Case 2 : " << motion_vector, PRX_TEXT_CYAN);
                                }
                                else
                                {
                                    PRX_DEBUG_COLOR("PRE Motion Vector Case 3 : " << motion_vector, PRX_TEXT_CYAN);
                                    motion_vector += (bisector*(-2*PRX_PI - delta_angle));
                                    PRX_DEBUG_COLOR("PRE Motion Vector Case 3 : " << motion_vector, PRX_TEXT_CYAN);
                                }
                                count++;
                            }
                        }
                        
                    }
                }
                
                angular_error/=count;
                
                if (angular_error > error_threshold)
                {
                    // Compute angle to steer towards
                    homing_direction = atan2(motion_vector.at(1), motion_vector.at(0));
                    PRX_DEBUG_COLOR ("Compuuted Homing direction: " << homing_direction, PRX_TEXT_MAGENTA);
                    output_control->at(0) = output_control_space->get_bounds().front()->get_upper_bound();
                    //output_control->at(1) = norm_angle_pi(homing_direction-get_state->at(2));
                    output_control->at(1) = norm_angle_pi(homing_direction);
                    //output_control->at(1) = (homing_direction-get_state->at(2));
                    PRX_DEBUG_COLOR ("HOMING GOOOOOOOOOOOOOOOOOOOO: " << output_control->at(1), PRX_TEXT_MAGENTA);
                    output_control_space->copy_from_point(output_control);
                }
                else
                {
                    PRX_DEBUG_COLOR ("HOMING OUTTTTTTTTTTTTTT", PRX_TEXT_RED);
                    output_control->at(0) = 0.0;
                    output_control->at(1) = 0.0;
                    output_control_space->copy_from_point(output_control);
                }
            }
            
            void homing_controller_t::voting_method()
            {
                
                control_t* output_control = output_control_space->alloc_point();
                
                // Set up direction votes to be 360 degrees, initialized to zero
                direction_votes.resize(360,0);
                
                // Update current and goal state
                update_current_and_goal();
                
                // Compute new homing direction
                double angular_error = 0;
                unsigned count = 0;
                // Go over all landmarks
                if (use_all_landmarks)
                {
                    PRX_DEBUG_COLOR("Will use all landmarks", PRX_TEXT_GREEN);
                    for (unsigned i = 0; i < landmarks.size(); ++i)
                    {
                        PRX_DEBUG_COLOR("--Landmark i: " << i << ", " << landmarks[i].first, PRX_TEXT_RED);
                        for (unsigned j = 0 ; j < landmarks.size(); ++j)
                        {
                            if (j > i)
                            {
                                PRX_DEBUG_COLOR("----Landmark j: " << j << ", " << landmarks[j].first, PRX_TEXT_BLUE);
                                // Get Direction to Landmark 1
                                vector_t current_direction1(3), goal_direction1(3);
                                get_direction_to_landmark(i, current_direction1, goal_direction1);
                                PRX_DEBUG_COLOR("Current direction to i: " << current_direction1 << ", goal : " << goal_direction1, PRX_TEXT_CYAN);
                                
                                // Get Direction to Landmark 2
                                vector_t current_direction2(3), goal_direction2(3);
                                get_direction_to_landmark(j, current_direction2, goal_direction2);
                                PRX_DEBUG_COLOR("Current direction to j: " << current_direction2 << ", goal : " << goal_direction2, PRX_TEXT_BROWN);
                                
                                // TODO: Implement conservative approach
                                if (conservative)
                                {
                                    if (false)
                                    {
                                        continue;
                                    }
                                }
                                //PRX_WARN_S ("Cdotprod: " << current_direction1.dot_product(current_direction2));
                                //PRX_WARN_S ("Cnorm1: " << current_direction1.norm() << ", Cnorm2: " << current_direction2.norm());
                                double current_angle = angle_between_points(current_direction1.dot_product(current_direction2), current_direction1.norm(), current_direction2.norm());
                                double goal_angle = angle_between_points(goal_direction1.dot_product(goal_direction2), goal_direction1.norm(), goal_direction2.norm());
                                
                                PRX_DEBUG_COLOR("Current angle : " << current_angle << " vs. goal angle:  " << goal_angle, PRX_TEXT_GREEN);
                                if (current_angle > (PRX_PI/2))
                                {
                                    current_angle = acos(current_direction2.dot_product(current_direction1));
                                    goal_angle = acos(goal_direction2.dot_product(goal_direction1));
                                    PRX_DEBUG_COLOR("NEWWWW  Current angle : " << current_angle << " vs. goal angle:  " << goal_angle, PRX_TEXT_GREEN);
                                }
                                PRX_ASSERT(current_angle <= PRX_PI);
                                angular_error += fabs ( current_angle - goal_angle);
                                count++;
                                unsigned floored_L2 = std::floor(current_angle*(180/PRX_PI));
                                if (current_angle > goal_angle)
                                {
                                    // Vote for Region Rb1
                                    unsigned floored_L2_begin = std::floor(floored_L2/2);
                                    for (unsigned i = floored_L2_begin; i < (360-floored_L2_begin); ++i )
                                    {
                                        ++direction_votes[i];
                                    }
                                }
                                else
                                {
                                    // Vote for Region Rb2
                                    double x = (360 - (2*floored_L2))/2;
                                    unsigned start_region = floored_L2 + x;
                                    unsigned end_region = floored_L2 + 2*x;
                                    PRX_DEBUG_COLOR("Defined region: [" << start_region << "," << end_region << "]", PRX_TEXT_MAGENTA);
                                    for (unsigned i = 0; i < 360; ++i )
                                    {
                                        if ( i < start_region || i > end_region)
                                                ++direction_votes[i];
                                    }
                                }
                                
                            }
                        }
                    }
                    
                    // Go over bins
                    double total_direction = 0.0;
                    unsigned total = 0;
                    unsigned max_votes = 0;
                    for (unsigned i = 0; i < 360; ++i)
                    {
                        if (direction_votes[i] > max_votes)
                        {
                            max_votes = direction_votes[i];
                        }
                    }
                    for (unsigned i = 0; i < 360; ++i)
                    {
                        if (direction_votes[i] == max_votes)
                        {
                            total_direction+=i;
                            total++;
                        }
                    }
                    
                    // Compute the average homing direction converted to radians
                    homing_direction = (total_direction/total)*(PRX_PI/180);
                    
                    PRX_DEBUG_COLOR("Computed homing direction: " << homing_direction, PRX_TEXT_MAGENTA);
                }
                
                angular_error /= count;
                
                // If the average angular error exceeds threshold, apply homing control
                if (angular_error > error_threshold)
                {
                    output_control->at(0) = output_control_space->get_bounds().front()->get_upper_bound();
                    output_control->at(1) = norm_angle_pi(homing_direction- get_state->at(2));
//                    output_control->at(1) = norm_angle_pi(homing_direction- get_state->at(2));
                    PRX_DEBUG_COLOR ("HOMING GOOOOOOOOOOOOOOOOOOOO: " << output_control->at(1), PRX_TEXT_MAGENTA);
                    
                }
                else
                {
                    PRX_DEBUG_COLOR ("HOMING OUTTTTTTTTTTTTTT", PRX_TEXT_RED);
                    output_control->at(0) = 0.0;
                    output_control->at(1) = 0.0;
                }
                
                output_control_space->copy_from_point(output_control);
            }
        }
    }
}
