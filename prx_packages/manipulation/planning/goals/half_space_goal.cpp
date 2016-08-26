/**
 * @file goal.cpp
 *  * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "planning/goals/half_space_goal.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/distance_metrics/ann_metric/ann_distance_metric.hpp"

namespace prx
{
    using namespace util;
    using namespace plan;
    namespace packages
    {
        namespace manipulation
        {

            half_space_goal_t::half_space_goal_t() 
            {
                space = NULL;
            }

            half_space_goal_t::~half_space_goal_t()
            {
                space->free_point(temp_point);
            }

            void half_space_goal_t::setup(world_model_t* in_model, std::vector<double> in_plane, bool in_hs_side)
            {
                model = dynamic_cast< manipulation_world_model_t* > (in_model);

                if(model == NULL)
                {
                    PRX_FATAL_S("Half Space Goal must be linked with a manipulation world model.");
                }
                
                if(in_plane.size() == 4)
                {
                    plane = in_plane;
                }
                else
                {
                    PRX_ERROR_S("Incorrect Plane Specified");
                    plane.resize(4);
                    plane[0] = 100;
                    plane[1] = 0;
                    plane[2] = 0;
                    plane[3] = 0;
                }

                hs_side = in_hs_side;
            }

            void half_space_goal_t::link_space(const space_t* inspace)
            {
                if(inspace != NULL)
                {
                    if(space != NULL && inspace->get_space_name() != space->get_space_name())
                    {
                        space->free_point(temp_point);
                        space->free_point(last_satisfied_point);
                        delete distance_metric;
                    }

                    if(space == NULL || inspace->get_space_name() != space->get_space_name())
                    {
                        temp_point = inspace->alloc_point();
                        last_satisfied_point = inspace->alloc_point();
                        distance_metric = (distance_metric_t*) (new ann_distance_metric_t());
                        distance_metric->link_space(inspace);
                    }

                    space = inspace;
                }
                else
                {
                    PRX_FATAL_S("Space not linked correctly in half space goal.");
                }

                goal_points.clear();

            }

            util::space_point_t* half_space_goal_t::get_last_satisfied_point()
            {
                return last_satisfied_point;
            }


            bool half_space_goal_t::satisfied(const space_point_t* state)
            {
                // PRX_DEBUG_POINT("Call to half space satisfied.");
                util::config_t ee_config;
                space->copy_to_point(temp_point);
                space->copy_from_point(state);
                model->FK(ee_config);
                space->copy_from_point(temp_point);
                double x,y,z;
                ee_config.get_position(x,y,z);

                double point_plane_equation = x*plane[0] + y*plane[1] + z*plane[2] + plane[3];

                // PRX_PRINT("half space:: "<<x<<", "<<y<<", "<<z<<" -> "<<point_plane_equation, PRX_TEXT_LIGHTGRAY);
                bool is_satisifed = false;
                if(hs_side)
                {
                    is_satisifed = point_plane_equation >= 0;
                }
                else
                {
                    is_satisifed = point_plane_equation <= 0;
                }

                if(is_satisifed)
                {
                    PRX_DEBUG_COLOR("SATISFIED half space goal:: "<<x<<", "<<y<<", "<<z<<" at "<<space->print_point(state,3), PRX_TEXT_GREEN);
                    space->copy_point(last_satisfied_point, state);
                    // goal_points.clear();
                    // goal_points.push_back(last_satisfied_point);
                }
                return is_satisifed;

            }

            bool half_space_goal_t::satisfied(const space_point_t* state, double& distance)
            {
                //TODO: Maybe compute distance from goal at this point? Not sure if that is relevant for half space checks
                bool ret = this->satisfied(state);
                if (ret)
                    distance = 0.0;
                return ret;
            }

            const std::vector<space_point_t*>& half_space_goal_t::get_goal_points(unsigned &size)
            {
                goal_points.clear();
                size = 0;//goal_points.size();
                return goal_points;
            }

            void half_space_goal_t::copy_goal_state(const space_point_t* goal_state)
            {
                //Do not need a goal state
                PRX_WARN_S ("Copy goal state does nothing in half space goal");
            }

            unsigned half_space_goal_t::size() const
            {
                return 0;
            }
        }

    }
}
