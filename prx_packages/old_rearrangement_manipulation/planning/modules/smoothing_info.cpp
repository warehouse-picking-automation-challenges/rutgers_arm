/**
 * @file smoothing_info.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "planning/modules/smoothing_info.hpp"
#include "prx/simulation/trajectory.hpp"
#include "prx/simulation/plan.hpp"
#include <fstream>
#include <sstream>
#include <set>

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {
        namespace rearrangement_manipulation
        {

            smoothing_info_t::smoothing_info_t() { }

            smoothing_info_t::smoothing_info_t(const util::space_t* control_space)
            {
                plan.link_control_space(control_space);
            }

            smoothing_info_t::~smoothing_info_t() { }

            bool smoothing_info_t::is_constrained_by(unsigned pose)
            {
                return from_pose == pose || to_pose == pose || constraints.count(pose) == 1;
            }

            control_t* smoothing_info_t::get_reaching_point()
            {
                return plan.at(reaching_point).control;
            }

            control_t* smoothing_info_t::get_retracting_point()
            {
                return plan.at(retracting_point).control;
            }

            void smoothing_info_t::trim_plan_after_retracting_point()
            {
                int index = plan.size() - 1;
                while( index > retracting_point )
                {
                    plan.pop_back();
                    index--;
                }

            }

            void smoothing_info_t::trim_plan_before_reaching_point(const space_t* control_space)
            {
                PRX_ASSERT(control_space->equal_points(plan.at(reaching_point).control,plan.at(reaching_point-1).control));                
                PRX_ASSERT(control_space->equal_points(plan.at(retracting_point).control,plan.at(retracting_point+1).control));
                plan_t new_plan;
                new_plan.link_control_space(control_space);
                for( unsigned i = reaching_point; i < plan.size(); ++i )
                {
                    new_plan.copy_onto_back(plan.at(i).control, plan.at(i).duration);
                }
                plan = new_plan;
                retracting_point -= reaching_point;
                reaching_point = 0;
                new_plan.clear();
                PRX_ASSERT(control_space->equal_points(plan.at(retracting_point).control,plan.at(retracting_point+1).control));                
            }

            std::string smoothing_info_t::print() const
            {
                std::stringstream output(std::stringstream::out);

                output << object_id << ") " << from_pose << " -> " << to_pose << " |P|:" << plan.size() << "  reaching:" << reaching_point << "  retracted:" << retracting_point << "   ";
                output << "  c:";

                foreach(unsigned i, constraints)
                {
                    output << i << " , ";
                }
                return output.str();
            }
        }
    }
}
