/**
 * @file grasping_planner.hpp
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

#pragma once

#ifndef PRX_GRASPING_PLANNER_HPP
#define	PRX_GRASPING_PLANNER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/heuristic_search/constraints.hpp"

#include "prx/simulation/plan.hpp"
#include "prx/simulation/trajectory.hpp"

#include "prx/planning/planner.hpp"
#include "planning/specifications/grasping_specification.hpp"
#include "prx/planning/queries/query.hpp"

#include "planning/manipulation_world_model.hpp"
#include "planning/modules/grasp.hpp"
#include "planning/modules/grasp_generator.hpp"
#include "planning/queries/grasping_query.hpp"
#include "prx/planning/task_planners/task_planner.hpp"

#include "planning/modules/planner_info.hpp"


namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
            /**
             * Manipulation task planner. Computes the path for moving an object from an
             * initial to a target position.
             *
             * @authors Andrew Dobson, Andrew Kimmel, Rahul Shome
             */
            class grasping_planner_t : public plan::task_planner_t
            {

              public:

                grasping_planner_t();
                virtual ~grasping_planner_t();

                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                /**
                 * @copydoc planner_t::reset()
                 */
                virtual void reset();

                /**
                 * @copydoc planner_t::link_world_model()
                 */
                virtual void link_world_model(plan::world_model_t * const model);

                /**
                 * @copydoc planner_t::get_statistics()
                 */
                virtual const util::statistics_t* get_statistics();

                /**
                 * @copydoc planner_t::link_specification(specification_t*)
                 */
                virtual void link_specification(plan::specification_t* new_spec);

                /**
                 * @copydoc planner_t::link_query()
                 */
                virtual void link_query(plan::query_t* new_query);

                /**
                 * @copydoc planner_t::setup()
                 */
                virtual void setup();

                /**
                 * @copydoc planner_t::execute()
                 */
                virtual bool execute();


                /**
                 * @copydoc planner_t::succeeded() const
                 */
                virtual bool succeeded() const;

                /**
                 * @copydoc planner_t::resolve_query()
                 */
                virtual void resolve_query();

                /** @copydoc planner_t::set_param(const std::string& , const std::string& , const boost::any& ) */
                virtual void set_param(const std::string& path, const std::string& parameter_name, const boost::any& value);

                virtual int nr_grasps(std::string context_name,movable_body_plant_t* object);


              protected:
                virtual void initialize_grasp_evaluation(grasp_data_t* data, util::config_t& object_pose, const grasp_t* grasp, sim::state_t* obj_state, util::config_t& grasping_config);

                virtual bool evaluate_the_grasp(const grasp_t* grasp);

                /** @brief Extracts the grasping configs **/
                virtual void extract_grasping_configs(const grasp_t* grasp);
                
                /** @brief Computes valid IK grasp states **/
                virtual bool compute_grasping_states( util::config_t& object_pose, const grasp_t* grasp, sim::state_t* obj_state, std::vector<util::config_t>& grasping_configs );
                
                /** @brief Computes valid IK grasp plans **/
                virtual bool compute_grasping_plans( const std::vector<util::config_t>& grasping_configs );

                /** @brief Computes plans for the grasp data (both IK and planning). */
                virtual bool compute_plans( grasp_data_t* grasp_data, unsigned state_index, const util::config_t& grasping_config );

                virtual bool evaluate_the_query();

                /** @brief Removes failed entries from the grasping data of the query. */
                virtual void remove_unsolved_grasp_data_entries( const std::vector< bool >& found_solutions );

                /**
                 * @brief Checks if the given relative configuration is valid for all the given states of the object.
                 * 
                 * @details Checks if the given relative configuration is valid for all the given states of the object.
                 * For the default gasping planner this is always true because we consider the entire data base for possible 
                 * grasps.
                 * 
                 * @param config The relative configuration that we want to check.
                 * @return True if the relative configuration exists in all the possible grasp of the specific state. 
                 */
                virtual bool valid_config_for_states(const grasp_t* grasp);

                /** @copydoc planner_t::set_param(const std::string&, const boost::any&) */
                virtual void set_param(const std::string& parameter_name, const boost::any& value);

                virtual void state_to_config(util::config_t& config, sim::state_t* state);

                manipulation_world_model_t* manipulation_model;

                /** @brief Pointer to the current problem specification */
                grasping_specification_t* grasp_specification;            

                grasping_query_t* grasping_query;

                /** @brief A map for the different types */
                util::hash_t<std::string, std::vector<grasp_t> >  grasps;

                util::config_t retraction_config;

                std::string pracsys_path;
                std::vector<std::pair<std::string,std::string> > data_folders;     
                util::hash_t<std::string, grasp_generator_t* >  grasp_generators;           
              
                manipulation_context_info_t* manipulator_info;
                sim::state_t* original_object_state;
                sim::state_t* tmp_state, *original_manipulation_state;
                sim::plan_t tmp_plan;
                sim::trajectory_t tmp_path;

                util::config_t ee_local_config;

                util::constraints_t* new_constraints;

                int init_ik_fail;
                int init_ik_in_col;
                int reach_ik_steer;
                int reach_with_obj;
                int reach_path_inv;
                int retract_with_obj;
                int retract_ik_steer;
                int retract_path_inv;
                int retract_state_inv;
                int succesfull_grasp;
            };
        }
    }
}


#endif
