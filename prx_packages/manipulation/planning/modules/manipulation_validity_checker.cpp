/**
 * @file manipulation_validity_checker.cpp 
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


#include "planning/modules/manipulation_validity_checker.hpp"
#include "prx/utilities/heuristic_search/object_collision_constraints.hpp"

#include "planning/manipulation_world_model.hpp"

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::packages::manipulation::manipulation_validity_checker_t, prx::plan::validity_checker_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace manipulation
        {

            void manipulation_validity_checker_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                ignore_list_validity_checker_t::init(reader,template_reader);
                ee_resolution = parameters::get_attribute_as<double>("ee_resolution", reader, template_reader, 0.015);
                PRX_DEBUG_COLOR ("EE resolution set to: " << ee_resolution, PRX_TEXT_BROWN);

                use_bounds = parameters::get_attribute_as< bool >("use_bounds", reader, template_reader, false);
                if( use_bounds )
                {
                    const std::vector<double> min = parameters::get_attribute_as<std::vector<double> >("min", reader, template_reader);
                    const std::vector<double> max = parameters::get_attribute_as<std::vector<double> >("max", reader, template_reader);

                    for( unsigned i = 0; i<3; i++ )
                    {
                        bounds.push_back(new bounds_t());
                    }
                    
                    PRX_ASSERT( min.size() == 3 && min.size() == max.size() );
                    bounds::set_bounds(bounds, min, max);                    
                }

            }


            void manipulation_validity_checker_t::link_model(world_model_t* model)
            {
                world_model = model;
                manip_model = NULL;
                if( use_bounds )
                {
                    manip_model = dynamic_cast< manipulation_world_model_t* >(model);
                }
            }

            bool manipulation_validity_checker_t::is_valid(const state_t* point)
            {
                if( use_bounds )
                {
                    config_t config;
                    
                    manip_model->get_state_space()->copy_from_point(point);
                    manip_model->FK(config);

                    const vector_t& pos = config.get_position();
                    // PRX_PRINT("Testing position: " << pos[0] << " " << pos[1] << " " << pos[2], PRX_TEXT_LIGHTGRAY );

                    if( !bounds[0]->is_valid( pos[0] ) || !bounds[1]->is_valid( pos[1] ) || !bounds[2]->is_valid( pos[2] ) )
                    {
                        // PRX_PRINT("Out of bounds!", PRX_TEXT_RED);
                        return false;
                    }
                    // PRX_PRINT("In bounds", PRX_TEXT_GREEN);
                }
                return ignore_list_validity_checker_t::is_valid( point );
            }

            bool manipulation_validity_checker_t::is_valid(const sim::trajectory_t& path )
            {
                return validity_checker_t::is_valid(path);
            }
            
            bool manipulation_validity_checker_t::is_valid(const sim::trajectory_t& path, const workspace_trajectory_t& ee_trajectory )
            {
                generate_checked_indices( ee_trajectory );
                
                return validate_and_generate_constraints( NULL, path );
            }

            bool manipulation_validity_checker_t::validate_and_generate_constraints( util::constraints_t* constraints, const sim::trajectory_t& path )
            {
                return validity_checker_t::validate_and_generate_constraints( constraints, path );
            }

            bool manipulation_validity_checker_t::validate_and_generate_constraints( util::constraints_t* constraints, const sim::trajectory_t& path, const workspace_trajectory_t& ee_trajectory )
            {
                PRX_DEBUG_COLOR("Manip validity checker", PRX_TEXT_CYAN);
                generate_checked_indices( ee_trajectory );

                return validate_and_generate_constraints( constraints, path );
            }

            void manipulation_validity_checker_t::generate_checked_indices( const workspace_trajectory_t& ee_trajectory )
            {
                PRX_DEBUG_COLOR("Manip validity checker", PRX_TEXT_CYAN);
                //Clear out any previous indices we may have had
                state_check_indices.clear();
                
                //First, let's assume we are always checking the first state.
                state_check_indices.push_back(0);
                
                //We'll need to keep track of the last index we checked
                unsigned last_checked = 0;
                
                //Then, for every ee position in the trajectory
                for( unsigned i=1; i<ee_trajectory.size(); ++i )
                {
                    //Check if we have moved sufficiently far from our last checked position
                    //TODO: Parameterize this somehow?
                    if( ee_trajectory.distance( last_checked, i ) > ee_resolution )
                    {
                        state_check_indices.push_back(i);
                        last_checked = i;
                    }
                }
                
                PRX_DEBUG_COLOR("EE TRAJ SIZE: " << ee_trajectory.size() << " vs: " << state_check_indices.size(), PRX_TEXT_GREEN);

                indices_set = true;
            }

            void manipulation_validity_checker_t::set_ee_resolution( double input_ee_resolution )
            {
                ee_resolution = input_ee_resolution;
            }

        }
    }
}
