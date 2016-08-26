/**
 * @file mcr.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "planning/task_planners/rearrangement_search_algorithms/mcr.hpp"
#include "planning/task_planners/rearrangement_primitive.hpp"
#include "planning/problem_specifications/rearrangement_search_specification.hpp"
#include "planning/problem_specifications/rearrangement_primitive_specification.hpp"
#include "planning/queries/rearrangement_search_query.hpp"
#include "planning/queries/rearrangement_query.hpp"
#include "planning/modules/path_part.hpp"


#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/simulation/plan.hpp"
#include "prx/planning/modules/heuristic_search/astar_module.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"

#include <vector>
#include <pluginlib/class_list_macros.h>
#include <set>

PLUGINLIB_EXPORT_CLASS(prx::packages::labeled_rearrangement_manipulation::mcr_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        using namespace manipulation;
        using namespace rearrangement_manipulation;

        namespace labeled_rearrangement_manipulation
        {

            mcr_t::mcr_t() { }

            mcr_t::~mcr_t() { }

            void mcr_t::setup()
            {
                rearrangement_search_algorithm_t::setup();
                statistics_file = prx_output_dir + "Statistics/MCR_" + in_specs->statistics_file;
            }

            void mcr_t::resolve_query()
            {
                PRX_DEBUG_COLOR("Resolve_query for mcr_t", PRX_TEXT_CYAN);
                int pose_index;
                //============================//
                //         Initial Node       //
                //============================//
                for( unsigned i = 0; i < in_specs->k_objects; ++i )
                {
                    object_state_space->copy_vector_to_point(in_query->initial_poses[i], object_state);
                    if( (pose_index = detect_pose(object_state)) != -1 )
                    {
                        initial_arrangement[i] = pose_index;
                    }
                    else
                    {
                        PRX_FATAL_S("Initial pose is not included in the informed poses! " << object_state_space->print_point(object_state, 8));
                    }
                }

                //============================//
                //         Target Node        //
                //============================//
                for( unsigned i = 0; i < in_specs->k_objects; ++i )
                {
                    object_state_space->copy_vector_to_point(in_query->target_poses[i], object_state);
                    if( (pose_index = detect_pose(object_state)) != -1 )
                    {
                        target_arrangement[i] = pose_index;
                    }
                    else
                    {
                        PRX_FATAL_S("Initial pose is not included in the informed poses! " << object_state_space->print_point(object_state, 8));
                    }
                }

                //================================//
                // Try to connect Start and Goal  //
                //================================//
                primitive_query->link_spaces(manip_state_space, manip_control_space);
                primitive_query->start_state = safe_state;
                primitive_query->initial_poses_ids = initial_arrangement;
                primitive_query->target_poses_ids = target_arrangement;
                primitive->link_query(primitive_query);
                primitive->resolve_query();
                mcr_stats = primitive->get_statistics()->as<mcr_test_statistics_t > ();
                return;
            }

            const statistics_t* mcr_t::get_statistics()
            {
                PRX_DEBUG_COLOR("statistics file: " << statistics_file, PRX_TEXT_CYAN);
                std::string dir, file;
                boost::tie(dir, file) = reverse_split_path(statistics_file);
                PRX_DEBUG_COLOR("dir: " << dir << "     file:" << file, PRX_TEXT_GREEN);
                boost::filesystem::path output_dir(dir);
                if( !boost::filesystem::exists(output_dir) )
                {
                    boost::filesystem::create_directory(output_dir);
                }
                std::ofstream fout(statistics_file.c_str());
                PRX_DEBUG_COLOR("Opened the file: " << statistics_file, PRX_TEXT_GREEN);
                PRX_ASSERT(fout.is_open());
                fout << mcr_stats->get_statistics();
                fout << std::endl;
                fout.close();


                return mcr_stats;
            }
        }
    }
}

