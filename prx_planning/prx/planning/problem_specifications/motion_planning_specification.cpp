/**
 * @file motion_planning_specification.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/planning/problem_specifications/motion_planning_specification.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"

#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/modules/heuristic_search/constraints_astar_search.hpp"

#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(prx::plan::motion_planning_specification_t, prx::plan::specification_t)

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace plan
    {

        motion_planning_specification_t::motion_planning_specification_t()
        {
            validity_checker = NULL;
            sampler = NULL;
            metric = NULL;
            local_planner = NULL;
            astar = NULL;
        }

        motion_planning_specification_t::~motion_planning_specification_t()
        {
            clear();
        }

        void motion_planning_specification_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            specification_t::init(reader, template_reader);                       
            PRX_DEBUG_COLOR("Inside motion planning specification init", PRX_TEXT_CYAN);
            
            // TODO: Do not use the template reader. Make your own template reader for each module!
            if( parameters::has_attribute("validity_checker", reader, template_reader) )
            {
                validity_checker = parameters::initialize_from_loader<validity_checker_t > ("prx_planning", reader, "validity_checker", template_reader, "validity_checker");
            }
            else
            {
                PRX_FATAL_S("Missing validity_checker attribute in planning specification!");
            }

            if( parameters::has_attribute("sampler", reader, template_reader) )
            {
                sampler = parameters::initialize_from_loader<sampler_t > ("prx_planning", reader, "sampler", template_reader, "sampler");
            }
            else
            {
                PRX_FATAL_S("Missing sampler attribute in planning specification!");
            }

            if( parameters::has_attribute("local_planner", reader, template_reader) )
            {
                local_planner = parameters::initialize_from_loader<local_planner_t > ("prx_planning", reader, "local_planner", template_reader, "local_planner");
            }
            else
            {
                PRX_FATAL_S("Missing local_planner attribute in planning specification!");
            }

            if( parameters::has_attribute("distance_metric", reader, template_reader) )
            {
                metric = parameters::initialize_from_loader<distance_metric_t > ("prx_utilities", reader, "distance_metric", template_reader, "distance_metric");
            }
            else
            {
                PRX_FATAL_S("Missing distance_metric attribute in planning specification!");
            }

            if( parameters::has_attribute("heuristic_search", reader, template_reader) )
            {
                astar = parameters::initialize_from_loader<constraints_astar_search_t> ("prx_planning", reader, "heuristic_search", template_reader, "heuristic_search");
            }
            else
            {
                PRX_WARN_S("Missing A*, heuristic search algorithm in planning specification!");
            }
            
            if( parameters::has_attribute("seeds", reader, template_reader) )
            {

                foreach(const parameter_reader_t* reader, parameters::get_list("seeds", reader, template_reader))
                {
                    seeds_vec.push_back(reader->get_attribute_as< std::vector<double> >("state"));
                    valid_seeds.push_back(true);
                }
            }
            else
            {
                PRX_WARN_S("No seed states will be added for motion planning problem specification.");
            }
            
            
        }

        void motion_planning_specification_t::setup(world_model_t * const model)
        {
            specification_t::setup( model );
            
            if(sampler!=NULL)
            {
                sampler->link_world_model(model);
            }

            if( validity_checker != NULL )
            {
                validity_checker->link_model(model);
            }

            if( metric != NULL )
            {
                //PRX_DEBUG_COLOR("Giving metric " << metric << " space: " << state_space->get_space_name() << "(" << state_space->get_dimension() << ")", PRX_TEXT_BROWN);
                metric->link_space(state_space);
                if(validity_checker != NULL)
                {
                    validity_checker->link_distance_function(metric->distance_function);
                }

                if(astar != NULL)
                {
                    astar->link_distance_function(metric->distance_function);
                }
                else
                {
                    PRX_ERROR_S("Heuristic not linked to A Star. Best First Search behavior expected.");
                }
            }

            if(local_planner != NULL)
            {
                local_planner->link_model(model);
                local_planner->link_sampler(sampler);
                local_planner->link_metric(metric);

            }            
        }

        void motion_planning_specification_t::clear()
        {
            specification_t::clear();
            clear_seeds();
        }

        void motion_planning_specification_t::clear_seeds()
        {
            seeds_vec.clear();
            valid_seeds.clear();

            if( state_space != NULL && seeds.size() != 0 )
            {

                foreach(state_t * state, seeds)
                {
//                    state_space->free_point(state);
                }
            }
            seeds.clear();
        }

        void motion_planning_specification_t::link_spaces(const util::space_t* new_state_space, const util::space_t* new_control_space)
        {            
            if( this->state_space != NULL && seeds.size() != 0 )
            {
                foreach(state_t * state, seeds)
                {
                    this->state_space->free_point(state);
                }
            }
            seeds.clear();            

            specification_t::link_spaces(new_state_space, new_control_space);
            seeds.resize(seeds_vec.size());
            for( size_t i = 0; i < seeds_vec.size(); ++i )
            {
                seeds[i] = state_space->alloc_point();
                new_state_space->copy_vector_to_point(seeds_vec[i], seeds[i]);
            }
        }

        void motion_planning_specification_t::set_seeds_vec(const std::vector< std::vector< double > >& s_vec)
        {
            clear_seeds();
            size_t size = s_vec.size();
            seeds_vec.resize(size);
            valid_seeds.resize(size);

            for( size_t i = 0; i < size; ++i )
            {
                seeds_vec[i] = s_vec[i];
                valid_seeds[i] = true;
            }

            if( state_space != NULL )
            {
                seeds.resize(size);
                for( size_t i = 0; i < size; ++i )
                {
                    seeds[i] = state_space->alloc_point();
                    state_space->copy_vector_to_point(seeds_vec[i], seeds[i]);
                }
            }
        }

        void motion_planning_specification_t::set_seeds(const std::vector<state_t*>& in_seeds)
        {
            clear_seeds();

            if( state_space != NULL )
            {
                size_t size = in_seeds.size();
                seeds.resize(size);
                seeds_vec.resize(size);
                valid_seeds.resize(size);
                for( size_t i = 0; i < size; ++i )
                {
                    seeds_vec[i].resize(state_space->get_dimension());
                    state_space->copy_point_to_vector(in_seeds[i], seeds_vec[i]);
                    valid_seeds[i] = true;
                    seeds[i] = state_space->clone_point(in_seeds[i]);
                }
            }
            else
                PRX_WARN_S("Setting seeds without state space in specification of problem! Seeds will not be added!");

        }

        void motion_planning_specification_t::add_seed(const state_t* state)
        {
            if( state_space != NULL )
            {
                seeds.push_back(state_space->clone_point(state));
                seeds_vec.push_back(std::vector<double>(state_space->get_dimension()));
                state_space->copy_point_to_vector(state, seeds_vec.back());
                valid_seeds.push_back(true);
            }
            else
                PRX_WARN_S("Adding seed without state space in specification of problem! The seed will not be added!");

        }

        std::vector<state_t*>& motion_planning_specification_t::get_seeds()
        {
            return seeds;
        }

        std::vector<bool>& motion_planning_specification_t::get_validity()
        {
            return valid_seeds;
        }
    }
}
