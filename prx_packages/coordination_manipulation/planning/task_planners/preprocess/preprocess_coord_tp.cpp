/**
 * @file naive_coord_manip_tp.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */


#include "../../../../manipulation/simulation/systems/plants/movable_body_plant.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/planning/motion_planners/prm_star/prm_star_statistics.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/bvp_local_planner.hpp"
#include "prx/utilities/goals/goal_state.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/utilities/distance_metrics/linear_metric/linear_distance_metric.hpp"
#include "planning/task_planners/preprocess/preprocess_coord_tp.hpp"
#include "prx/planning/planner.hpp"

#include <algorithm> 
#include <boost/range/adaptor/map.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::coordination_manipulation::preprocess_coordination_tp_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace coordination_manipulation
        {
            preprocess_coordination_tp_t::preprocess_coordination_tp_t()
            {
                PRX_PRINT("Creation of the PREPROCESS manipulation task planner", PRX_TEXT_RED);
                global_start_state = NULL;
                left_arm_control_space = NULL;
                left_arm_state_space = NULL;
                right_arm_control_space = NULL;
                right_arm_state_space = NULL;
                preprocess_mp = NULL;
                preprocess_query = NULL;
                manip = NULL;
                manipulation_model = NULL;
                check_all_trajectories = true;
                approximate_collisions = false;
                
                start_index = 20;
                end_index = 100;
            }

            preprocess_coordination_tp_t::~preprocess_coordination_tp_t()
            {
//                //Make sure to free the safe states.
//                for( unsigned i=0; i<safe_states.size(); ++i )
//                {
//                    manipulator_state_spaces[i]->free_point(safe_states[i]);
//                    safe_states[i] = NULL;
//                }
                //Free the global start state
                model->use_context("full_space");
                model->get_state_space()->free_point( global_start_state );
                
                left_arm_state_space->free_point(left_arm_safe_state);
                right_arm_state_space->free_point(right_arm_safe_state);
            }

            void preprocess_coordination_tp_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
            {
                //I don't think there are any additional parameters to load here.
                task_planner_t::init(reader, template_reader);
                
                /** Make sure we know our contexts */
                full_manipulator_context_name = parameters::get_attribute("full_manipulator_context_name", reader, template_reader);
                left_context_name = parameters::get_attribute("left_context_name", reader, template_reader);
                right_context_name = parameters::get_attribute("right_context_name", reader, template_reader);
                
                /** Read number of objects per arm*/
                object_types = parameters::get_attribute_as<std::vector< std::string > >("object_types", reader, template_reader);
                if (parameters::has_attribute("num_trajectories_per_object_type", reader, template_reader))
                {
                    num_trajectories_per_object_type = parameters::get_attribute_as<std::vector<int> >("num_trajectories_per_object_type", reader, template_reader);
                    check_all_trajectories = false;
                }
                approximate_collisions = parameters::get_attribute_as<bool>("approximate_collisions", reader, template_reader, false);

                experiment = parameters::get_attribute_as<std::string>("experiment", reader, template_reader);
                trajectories_directory = parameters::get_attribute_as<std::string>("trajectories_directory", reader, template_reader);
                
                /** Find coordination prm */
                foreach(planner_t* planner, planners | boost::adaptors::map_values)
                {
                    preprocess_coordination_mp_t* test = dynamic_cast<preprocess_coordination_mp_t*>(planner);
                    if (test != NULL)
                        preprocess_mp = test;
                }
                
                PRX_ASSERT(preprocess_mp != NULL);
            }

            void preprocess_coordination_tp_t::setup()
            {
                
                //Now that we have the entire schtick initialized, collect info:
                //Gather all of the manipulators and cup, set them in their order
                find_plants();

                //Then, find start state information and such
                store_start_state();

                if (!approximate_collisions)
                {
                    model->use_context("imaginary_cup_left");
                    left_cup_space = model->get_state_space();
                    model->use_context("imaginary_cup_right");
                    right_cup_space = model->get_state_space();


                    model->use_context("left_armcup");
                    left_armcup_space = model->get_state_space();
                    model->use_context("right_armcup");
                    right_armcup_space = model->get_state_space();

                    model->use_context("full_space");
                }
  
            }

            void preprocess_coordination_tp_t::find_plants()
            {
                manipulation_model->use_context("full_space");
                std::vector< plant_t* > all_plants;
                manipulation_model->get_system_graph().get_plants(all_plants);
                
                
                foreach(plant_t* plant, all_plants)
                {
                    //Grab manipulators
                    if( dynamic_cast<manipulator_t*>(plant) != NULL )
                    {
                        manip = dynamic_cast<manipulator_t*>(plant);
                    }
                
                }
                PRX_ASSERT(manip != NULL);

            }

            void preprocess_coordination_tp_t::store_start_state()
            {
                manipulation_model->use_context(full_manipulator_context_name);
                global_start_state = manipulation_model->get_state_space()->alloc_point();
                full_state_space = manipulation_model->get_state_space();
                full_control_space = manipulation_model->get_control_space();
                
                manipulation_model->use_context(left_context_name);
                left_arm_state_space = manipulation_model->get_current_manipulation_info()->full_arm_state_space;
                left_arm_control_space = manipulation_model->get_current_manipulation_info()->full_arm_control_space;
                left_arm_safe_state = left_arm_state_space->alloc_point();
                left_arm_full_plan.link_control_space(left_arm_control_space);
                left_arm_full_plan.link_state_space(left_arm_state_space);
                current_left_trajectory.link_space(left_arm_state_space);
                
                manipulation_model->use_context(right_context_name);
                right_arm_state_space = manipulation_model->get_current_manipulation_info()->full_arm_state_space;
                right_arm_control_space = manipulation_model->get_current_manipulation_info()->full_arm_control_space;
                right_arm_safe_state = right_arm_state_space->alloc_point();
                right_arm_full_plan.link_control_space(right_arm_control_space);
                right_arm_full_plan.link_state_space(right_arm_state_space);
                current_right_trajectory.link_space(right_arm_state_space);
                
                manipulation_model->use_context(full_manipulator_context_name);

            }

            void preprocess_coordination_tp_t::link_world_model( world_model_t* const model )
            {
                task_planner_t::link_world_model(model);
                manipulation_model = dynamic_cast<manipulation_world_model_t*>(model);
                PRX_ASSERT(manipulation_model != NULL);
            }

            const statistics_t* preprocess_coordination_tp_t::get_statistics()
            {
                return NULL;
            }

            void preprocess_coordination_tp_t::link_specification( specification_t* new_spec )
            {
                //Until I know what is special about the specification...
                task_planner_t::link_specification( new_spec );
            }

            void preprocess_coordination_tp_t::link_query( query_t* new_query )
            {
                //Until I know what is special about this query...
                task_planner_t::link_query( new_query );
                
                preprocess_query = dynamic_cast<preprocess_mp_query_t*>(new_query);
                preprocess_query->experiment = experiment;
                preprocess_query->approximate_collision_check = approximate_collisions;
                PRX_ASSERT(preprocess_query != NULL);
            }

            bool preprocess_coordination_tp_t::execute()
            {

                PRX_PRINT("// =======================================================", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("// =======================================================", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("// =======================================================", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("// ================......===========......================", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("// ==================.....=========.....==================", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("// ===================.....=======.....===================", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("// ====================.....=====.....====================", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("// =====================.....===.....=====================", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("// ======================.....=.....======================", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("// =======================.........=======================", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("// ========================.......========================", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("// =========================.....=========================", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("// ========================.......========================", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("// =========....==========.........==========....=========", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("// =======........=======.....=.....=======........=======", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("// ======...====...=====.....===.....=====...====...======", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("// ======..........====.....=====.....====..........======", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("// ======..===========.....=======.....===..==============", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("// ======....==....==.....=========.....==....==....======", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("// =======........=......===========......=........=======", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("// =======================================================", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("// =======================================================", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("// =======================================================\n", PRX_TEXT_LIGHTGRAY);

                // = = = DEBUG = = =
                PRX_PRINT("Begining Preprocess Coordination Manip Execution.", PRX_TEXT_GREEN);
                
                char* w = std::getenv("PRACSYS_PATH");
                std::string dir(w);
                dir += trajectories_directory;
                boost::filesystem::path output_dir(dir);
                if( !boost::filesystem::exists(output_dir) )
                {
                    PRX_FATAL_S ("Trajectories directory does not exist!::: " << dir);
                }

                
                init_preprocess_mp();
                computation_timer.reset();
                
                for (unsigned object_type_index = 0; object_type_index < object_types.size(); ++object_type_index)
                {
                    std::string right_object = object_types[object_type_index];
                    int right_max = num_trajectories_per_object_type[object_type_index];
                    for (unsigned other_object_index = 0; other_object_index < object_types.size(); ++other_object_index)
                    {
                        std::string left_object = object_types[other_object_index];
                        int left_max = num_trajectories_per_object_type[other_object_index];
                        for (unsigned right_trajectory_index = 0; right_trajectory_index < right_max; ++right_trajectory_index)
                        {
                            std::stringstream ss;
                            ss << dir << "/" << right_object << "/right" << right_trajectory_index << ".traj";
                            std::ifstream fin;
                            fin.open(ss.str().c_str());
                            PRX_ASSERT(fin.is_open());
                            current_right_trajectory.read_from_stream(fin);
                            fin.close();
                            for (unsigned left_trajectory_index = 0; left_trajectory_index < left_max; ++left_trajectory_index)
                            {
        //                        if (left_plan_index != right_plan_index)
                                {
                                    std::stringstream ss2;
                                    ss2 << dir << "/" << left_object << "/left" << left_trajectory_index << ".traj";
                                    std::ifstream fin2;
                                    fin2.open(ss2.str().c_str());
                                    PRX_ASSERT(fin2.is_open());
                                    current_left_trajectory.read_from_stream(fin2);
                                    fin2.close();
                                    preprocess_query->link_arm_trajectories(&current_left_trajectory, &current_right_trajectory);
                                    preprocess_query->define_coordination_problem(right_trajectory_index, left_trajectory_index, 0,0,left_object, right_object, experiment);
                                    preprocess_query->setup();
                                    preprocess_mp->reset();
                                    preprocess_mp->setup();

                                    try
                                    {
                                        preprocess_mp->execute();
                                    }
                                    catch( stopping_criteria_t::stopping_criteria_satisfied e )
                                    {

                                        PRX_PRINT("Execution was successful! Resolving query...", PRX_TEXT_GREEN);
                                        preprocess_mp->resolve_query();
                                    }
                                }
                            }
                        }
                    }
                }
                

//                PRX_PRINT("FINISHED PRECOMPUTATION TIME: " << computation_timer.measure(), PRX_TEXT_CYAN);
//                
                // = = =
                resolve_query();

                return true;
            }
            
    
            

            void preprocess_coordination_tp_t::resolve_query()
            {
                PRX_PRINT("Beginning NBaive Coordination Resolve Query", PRX_TEXT_RED);
                std::ofstream fout;
                fout.open("planning_results.txt", std::ofstream::app);
//                fout << algorithm << " " << experiment << " " << preprocess_query->plan.length() << " " << computation_timer.measure() << " " << std::endl;
                fout.close();
                
            }


            void preprocess_coordination_tp_t::reset()
            {
                //Uh... not sure what to do here?
                task_planner_t::reset();
            }

            bool preprocess_coordination_tp_t::succeeded() const
            {
                //Perhaps return based on whether there is a path?
                return false;
            }

            void preprocess_coordination_tp_t::init_preprocess_mp()
            {
                /** Set up specification */
                input_specification->link_spaces(full_state_space, full_control_space);

                /** Set up query */
                preprocess_query->link_left_arm(left_arm_state_space, left_arm_control_space);
                preprocess_query->link_right_arm(right_arm_state_space, right_arm_control_space);
                preprocess_query->plan.link_control_space(full_control_space);
                preprocess_query->plan.link_state_space(full_state_space);
                preprocess_query->path.link_space(full_state_space);

                if (!approximate_collisions)
                {
                    preprocess_query->link_imaginary_cups(left_cup_space,right_cup_space);
                    preprocess_query->link_armcups(left_armcup_space, right_armcup_space);
                }

                /** Link specification, query, and call setup */
                input_specification->setup(model);
                preprocess_mp->link_specification(input_specification);
                preprocess_mp->link_query(preprocess_query);
            }

        }
    }
}

