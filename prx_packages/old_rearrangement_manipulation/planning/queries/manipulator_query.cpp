/**
 * @file manipulation_query.cpp
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

#include "planning/queries/manipulator_query.hpp"
#include "prx/utilities/goals/goal.hpp"

#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>
#include <set>

PLUGINLIB_EXPORT_CLASS(prx::packages::rearrangement_manipulation::manipulator_query_t, prx::plan::query_t)

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {
        using namespace manipulation;
        namespace rearrangement_manipulation
        {

            manipulator_query_t::manipulator_query_t()
            {
                state_space = NULL;
                control_space = NULL;
                mode = PRX_FULL_PATH;
                path_quality = PRX_FIRST_PATH;    
                found_path = false;
            }

            manipulator_query_t::~manipulator_query_t()
            {                
                clear();
            }

            void manipulator_query_t::clear()
            {
                restart();
                goal_states.clear();
            }

            void manipulator_query_t::restart()
            {
                manipulation_query_t::clear();
                
                PRX_ASSERT(constraints.size() == full_constraints.size());
                for( unsigned i = 0; i < constraints.size(); ++i )
                {
                    constraints[i].clear();                                       
                }
                for( unsigned i = 0; i < constraints_in_order.size(); ++i )
                {
                    constraints_in_order[i].clear();
                }
                for( unsigned i = 0; i < full_constraints.size(); ++i )
                {
                     full_constraints[i].clear();
                }
                constraints.clear();
                constraints_in_order.clear();
                full_constraints.clear();
                solutions_costs.clear();
                reaching_points.clear();
                retracting_points.clear();
                found_path = false;
            }
            
            void manipulator_query_t::add_constraint(unsigned constraint, unsigned index)
            {
                constraints[index].insert(constraint);
            }

            void manipulator_query_t::add_constraints(std::set<unsigned> new_constraints, unsigned index)
            {
                constraints[index].insert(new_constraints.begin(),new_constraints.end());
            }
        }
    }
}
