/**
 * @file pose_based_mp_query.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2016, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "planning/queries/pose_based_mp_query.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::pose_based_mp_query_t, prx::plan::query_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;
    
    namespace packages
    {
        namespace manipulation
        {
            pose_based_mp_query_t::pose_based_mp_query_t()
            {
                pose_query = false;
            }
            
            pose_based_mp_query_t::~pose_based_mp_query_t()
            {
                
            }
            
            void pose_based_mp_query_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                motion_planning_query_t::init( reader, template_reader );
                if( parameters::has_attribute("target_position", reader, template_reader) )
                {
                    target_pose.set_position( parameters::get_attribute_as< vector_t >("target_position", reader, template_reader) );
                    target_pose.set_orientation( parameters::get_attribute_as< quaternion_t >("target_orientation", reader, template_reader) );
                }
                
                if( parameters::has_attribute("end_effector", reader, template_reader) )
                {
                    end_effector = parameters::get_attribute("end_effector", reader, template_reader);
                }
            }

            void pose_based_mp_query_t::setup( state_t* input_start_state, search_mode_t input_search_mode, const config_t& target_config, std::string target_effector, const constraints_t* input_active_constraints, constraints_t* input_path_constraints, bool should_update_constraints )
            {
                target_pose = target_config;
                end_effector = target_effector;
                motion_planning_query_t::setup( input_start_state, input_search_mode, input_active_constraints, input_path_constraints, should_update_constraints );
                pose_query = true;
            }

            const config_t& pose_based_mp_query_t::get_target_pose()
            {
                return target_pose;
            }
            
            const std::string& pose_based_mp_query_t::get_end_effector()
            {
                return end_effector;
            }
            
            bool pose_based_mp_query_t::is_pose_query()
            {
                return pose_query;
            }

            void pose_based_mp_query_t::clear()
            {
                motion_planning_query_t::clear();
                pose_query = false;
            }
            
        }
    }
}


