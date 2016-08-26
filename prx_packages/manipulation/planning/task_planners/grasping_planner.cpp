/**
 * @file grasping_planner.cpp
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

#include "planning/task_planners/grasping_planner.hpp"

#include "simulation/workspace_trajectory.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"

#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"

#include "planning/manipulation_world_model.hpp"
#include "planning/queries/grasping_query.hpp"
#include "planning/specifications/grasping_specification.hpp"
#include "planning/modules/object_constraints_checker.hpp"
#include "prx/utilities/goals/multiple_goal_states.hpp"

#include <boost/range/adaptor/map.hpp>
#include <boost/filesystem.hpp>
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::grasping_planner_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace manipulation
        {

            void operator >> (const YAML::Node& node, std::vector<double>& config) 
            {
                config.resize( node.size() );
                for( unsigned i=0; i < config.size(); ++i )
                {
                    config[i] = node[i].as<double>();
                }
            }
            void operator >> (const YAML::Node& node, util::config_t& config) 
            {
                std::vector<double> vec;
                node["relative_config"] >> vec;
                config.set_position(vec[0],vec[1],vec[2]);
                config.set_xyzw_orientation(vec[3],vec[4],vec[5],vec[6]);
            }

            void operator >> (const YAML::Node& node, int& mode) 
            {
                mode = node["mode"].as<int>();
            }
            
            grasping_planner_t::grasping_planner_t()
            {
                char* w = std::getenv("PRACSYS_PATH");
                std::string filename(w);
                pracsys_path = filename;

                original_object_state = NULL;
            }

            grasping_planner_t::~grasping_planner_t()
            {
                if (validity_checker != NULL)
                    validity_checker->free_constraint(new_constraints);
            }

            void grasping_planner_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                planner_t::init(reader, template_reader);
                PRX_DEBUG_COLOR("Initializing Grasping planner ...", PRX_TEXT_CYAN);

                if(parameters::has_attribute("data_folders", reader, NULL))
                {
                    parameter_reader_t::reader_map_t planner_map = reader->get_map("data_folders");

                    foreach(const parameter_reader_t::reader_map_t::value_type key_value, planner_map)
                    {
                        data_folders.push_back(std::make_pair(key_value.first,pracsys_path +"/"+ parameters::get_attribute("data_folders/"+key_value.first,reader,NULL)));
                        PRX_DEBUG_COLOR( data_folders.back().first << " " << data_folders.back().second, PRX_TEXT_LIGHTGRAY );
                    }
                    PRX_DEBUG_COLOR( "Loaded a total of " << data_folders.size() << " data folders.", PRX_TEXT_GREEN);
                }
                if(parameters::has_attribute("grasp_generators", reader, NULL))
                {
                    parameter_reader_t::reader_map_t planner_map = reader->get_map("grasp_generators");

                    foreach(const parameter_reader_t::reader_map_t::value_type key_value, planner_map)
                    {

                        std::string template_name;
                        const parameter_reader_t* child_template_reader = NULL;

                        if( key_value.second->has_attribute("template") )
                        {
                            template_name = key_value.second->get_attribute("template");
                            child_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + template_name, global_storage);
                        }

                        grasp_generators[key_value.first] = parameters::initialize_from_loader<packages::manipulation::grasp_generator_t> ("prx_planning", key_value.second, "", child_template_reader, "");
                        PRX_DEBUG_COLOR( "Created grasp evaluator for context: " << key_value.first, PRX_TEXT_LIGHTGRAY );


                        if( child_template_reader != NULL )
                        {
                            delete child_template_reader;
                        }
                    }
                }
                else
                {
                    PRX_FATAL_S ("No grasp database specified. For manual generation: no default end effector normal defined!");
                }
                init_ik_fail=0;
                init_ik_in_col=0;
                reach_ik_steer=0;
                reach_with_obj=0;
                reach_path_inv=0;
                retract_with_obj=0;
                retract_ik_steer=0;
                retract_path_inv=0;
                retract_state_inv=0;
                succesfull_grasp=0;
                PRX_DEBUG_COLOR("Grasping planner Initialized...", PRX_TEXT_CYAN);

            }

            void grasping_planner_t::reset()
            {
            }

            void grasping_planner_t::link_world_model(world_model_t * const model)
            {
                manipulation_model = dynamic_cast<manipulation_world_model_t*>(model);
                if( validity_checker != NULL )
                {
                    validity_checker->link_model(model);
                }

                foreach(grasp_generator_t* generator, grasp_generators | boost::adaptors::map_values)
                {
                    generator->link_world_model(model);
                }
            }

            const statistics_t* grasping_planner_t::get_statistics()
            {
                PRX_WARN_S("Get statistics for grasping planner is not implemented!");
                return new statistics_t();
            }

            void grasping_planner_t::link_specification(specification_t* new_spec)
            {
                grasp_specification = dynamic_cast< grasping_specification_t* >( new_spec );
                if( grasp_specification == NULL )
                {
                    PRX_FATAL_S("Grasping planner requires a grasp_specification to be linked!");
                }
                
                //Setup our stuff from the specification
                validity_checker = grasp_specification->validity_checker;
                new_constraints = validity_checker->alloc_constraint();
            }

            void grasping_planner_t::link_query(query_t* new_query)
            {
                grasping_query = dynamic_cast<grasping_query_t*>(new_query);
                if(grasping_query == NULL)
                {
                    PRX_FATAL_S("The grasping planner operates only over a grasping query as input!");
                }
            }

            void grasping_planner_t::setup()
            {
                PRX_DEBUG_COLOR("Setup grasping planner ...", PRX_TEXT_CYAN);
                std::vector<movable_body_plant_t* > objects;
                manipulation_model->get_objects(objects);

                foreach(movable_body_plant_t* object, objects)
                {
                    std::string object_type =  object->get_object_type();

                    for (std::vector<std::pair<std::string,std::string> >::iterator iter = data_folders.begin(); iter != data_folders.end(); ++iter)
                    {
                        std::string data_folder = iter->second;
                        std::string hash_string = object_type+iter->first;
                        std::string dir = data_folder + object_type + ".yaml"; //the yaml file with the list of configurations
                        if(grasps[hash_string].size() == 0 && boost::filesystem::exists( dir ))
                        {
                            YAML::Node doc = YAML::LoadFile(dir);
                            for(unsigned i=0;i<doc.size();i++)
                            {
                                config_t config;
                                std::vector<double> vec;
                                doc[i]["relative_config"] >> vec;
                                config.set_position(vec[0],vec[1],vec[2]);
                                config.set_xyzw_orientation(vec[3],vec[4],vec[5],vec[6]);
                                int release_mode = 1;
                                if(doc[i]["release_mode"])
                                    release_mode = doc[i]["release_mode"].as<int>();
                                int grasp_mode = doc[i]["grasp_mode"].as<int>();
                                grasps[ hash_string ].push_back(grasp_t(config,release_mode,grasp_mode));
                            }
                            PRX_DEBUG_COLOR("Number of Grasps for "<<hash_string<<": "<<grasps[ hash_string ].size(), PRX_TEXT_BLUE)
                        }
                    }
                }
            }

            bool grasping_planner_t::succeeded() const
            {
                return true;
            }

            bool grasping_planner_t::execute()
            {
                return true;
            }            

            void grasping_planner_t::resolve_query()
            {
                 PRX_PRINT("Grasping planner :: Resolving query", PRX_TEXT_LIGHTGRAY );
                //First off, begin by assuming we have not found a grasp
                grasping_query->found_grasp = false;
                //We need to setup the constriant checker to know things
                if( validity_checker != NULL )
                {
                    //TODO: Is there some way we could not have the specific validity checker setup stuff in here?
                    object_constraints_checker_t* object_checker = dynamic_cast< object_constraints_checker_t* >( validity_checker );
                    if( object_checker != NULL )
                    {
                        object_checker->setup_checker( grasping_query->object->get_pathname() );
                    }
                }
                
                double original_mode = manipulation_model->get_current_grasping_mode();
                original_object_state = grasping_query->object->get_state_space()->alloc_point();    


                original_manipulation_state = manipulation_model->get_state_space()->alloc_point();        
                manipulator_info = manipulation_model->get_current_manipulation_info();
                manipulation_model->get_end_effector_local_config(ee_local_config);

                tmp_state = grasping_query->state_space->alloc_point();
                tmp_path.link_space(grasping_query->state_space);
                tmp_plan.link_control_space(grasping_query->control_space);


                PRX_DEBUG_COLOR(" ----Grasping TP :: With " << grasping_query->object_states.size() << " object states with suggested_grasp : " << ((grasping_query->suggested_grasp == NULL)?"NULL":"YES"), PRX_TEXT_CYAN);

                //If we have a suggested grasp to use
                if(grasping_query->suggested_grasp != NULL)
                {
                    //Determine if the suggested grasp exists in database for each object pose in the query
                    if(valid_config_for_states(grasping_query->suggested_grasp))
                    {
                        //Actually evaluate the grasp for validity
                        grasping_query->found_grasp = evaluate_the_grasp( grasping_query->suggested_grasp );

                        // append_to_stat_file((grasping_query->found_grasp?"\n     Successful grasp": "\n      "+grasping_query->reason_for_failure));
                            
                    }
                }
                else
                {
                    grasping_query->found_grasp = evaluate_the_query();
                }

                //Restore the object state
                grasping_query->object->get_state_space()->copy_from_point( original_object_state );
                
                //if( manipulation_model->get_current_grasping_mode() != original_mode )
                {
                    grasping_query->object->get_state_space()->copy_from_point(original_object_state);
                    manipulation_model->push_state(original_manipulation_state);
                    manipulation_model->engage_grasp(tmp_plan, original_mode, false);
                }
                
                grasping_query->state_space->free_point(tmp_state);
                grasping_query->object->get_state_space()->free_point( original_object_state );
                manipulation_model->get_state_space()->free_point( original_manipulation_state );
                tmp_plan.clear();
                tmp_path.clear();
                return;
            }

            bool grasping_planner_t::evaluate_the_grasp(const grasp_t* grasp )
            {
                PRX_ASSERT(grasp != NULL);

                grasping_query->object->get_state_space()->copy_to_point(original_object_state);
                config_t object_pose;

                /** We must check grasp for a valid IK state*/
                if(grasping_query->grasping_mode == GRASPING_CONFIGURATIONS)
                {
                    extract_grasping_configs(grasp);
                    return true;
                }
                /** We must also check the corresponding task_mode plan */
                else if(grasping_query->grasping_mode == GRASPING_PLANS)
                {
                    // Create a grasping configs vector that has the same size as the object states vector
                    std::vector<config_t> grasping_configs(grasping_query->object_states.size());
                    bool have_states = compute_grasping_states( object_pose, grasp, original_object_state, grasping_configs );

                    //if we didn't get a state
                    if( !have_states )
                    {
                        //No point in even looking for plans
                        return false;
                    }
                    
                    //Otherwise, we have states which work, so see if we can get plans
                    return compute_grasping_plans( grasping_configs );
                }
            }

            void grasping_planner_t::initialize_grasp_evaluation(grasp_data_t* data, config_t& object_pose, const grasp_t* grasp, state_t* obj_state, config_t& grasping_config)
            {
                //convert the space_point_t to a config_t
                state_to_config(object_pose, obj_state);

                //Compute the grasping configuration                
                grasping_config = ee_local_config;
                // PRX_PRINT("1: "<<grasping_config,PRX_TEXT_RED);
                grasping_config.relative_to_global( grasp->relative_config );
                // PRX_PRINT("2: "<<grasping_config,PRX_TEXT_RED);
                grasping_config.relative_to_global( object_pose );
                // PRX_PRINT("3: "<<grasping_config,PRX_TEXT_RED);


                grasping_query->object->get_state_space()->copy_from_point(obj_state);

                // Enforce that the manipulator is using the release mode specified by the grasp data
                double first_mode = manipulation_model->get_current_grasping_mode();
                if(fabs(first_mode - grasp->release_mode)>0.001)
                {
                    // TODO: Make sure we do not need to push state here first
                    manipulation_model->engage_grasp(tmp_plan, grasp->release_mode, false);
                }

                data->setup(grasping_query->state_space, grasping_query->control_space, validity_checker);
                //using the releasing state as tmp variable in order to get the initial state of the manipulator.
                manipulator_info->full_arm_state_space->copy_to_point(data->releasing_state);
            }

            void grasping_planner_t::extract_grasping_configs(const grasp_t* grasp)
            {
                int state_index = 0;
                foreach(state_t* obj_state, grasping_query->object_states)
                {
                    grasp_data_t* data = new grasp_data_t();
                    *data->relative_grasp = *grasp;
                    grasping_query->set_data(state_index, data);
                    ++state_index;
                }
            }
            
            bool grasping_planner_t::compute_grasping_states( config_t& object_pose, const grasp_t* grasp, state_t* obj_state, std::vector<config_t>& grasping_configs )
            {

                int state_index = 0;
                //For each of the object states we are evaluating for
                foreach(state_t* obj_state, grasping_query->object_states)
                {
                    //Set up a data for this potential grasp.
                    grasp_data_t* data = new grasp_data_t();
                    *data->relative_grasp = *grasp;
                    
                    //Do some initialization for the data
                    initialize_grasp_evaluation( data, object_pose, grasp, obj_state, grasping_configs[state_index] );

                    // // PRX_INFO_S(data->relative_grasp->relative_config<<" grasping_configs[state_index] = "<<grasping_configs[state_index]);

                    // bool successIK = false;
                    // //If we have an IK solution for this configuration
                    // //Add next line to reinitialize rand before calling IK. 
                    // // srand(1);
                    // if(manipulation_model->IK( data->releasing_state, data->releasing_state, grasping_configs[state_index], false ))  
                    // {
                    //     std::stringstream ss;
                    //     ss<<manipulator_info->full_arm_state_space->print_point(data->releasing_state,3);
                    //     // append_to_stat_file("\nGrasp: "+ss.str());

                    //     //And the resulting state is valid
                    //     new_constraints->clear();
                    //     if( validity_checker->validate_and_generate_constraints(new_constraints, data->releasing_state) )
                    //     {
                    //         //Good to go, we can use this one
                    //         successIK = true;
                    //         if( grasping_query->astar_mode == STANDARD_SEARCH || grasping_query->astar_mode == LAZY_SEARCH  )
                    //         {
                    //             successIK = !( grasping_query->active_constraints->has_intersection( new_constraints ) );
                    //         }
                            
                    //         if( successIK )
                    //         {
                    //             //Also need to fill in the grasping state
                    //             manipulator_info->full_arm_state_space->copy_from_point( data->releasing_state );
                    //             manipulation_model->engage_grasp(tmp_plan, grasp->grasping_mode, false);
                    //             manipulator_info->full_arm_state_space->copy_to_point( data->grasping_state );
                    //         }
                    //         else
                    //         {
                    //             grasping_query->reason_for_failure = "Initial IK invalid : in collision (non-empty constraints)";
                    //             init_ik_in_col++;                                
                    //         }
                    //     }
                    //     else
                    //     {
                    //         PRX_DEBUG_COLOR ("Releasing state: " <<manipulator_info->full_arm_state_space->print_point(data->releasing_state), PRX_TEXT_RED);
                    //         grasping_query->reason_for_failure = "Initial IK invalid : in collision";
                    //         init_ik_in_col++;
                    //     }
                    // }
                    // else
                    // {
                    //     grasping_query->reason_for_failure = "Initial IK failed";
                    //     init_ik_fail++;
                    // }
                    
                    
                    // //If there is not a collision-free and successful IK 
                    // if( !successIK )
                    // {
                    //     //Free the data
                    //     delete data;
                    //     //We need to clear out all previous successful states, so people don't accidentally use them
                    //     for(int i = state_index-1; i >= 0; --i)
                    //     {
                    //         grasping_query->remove_last_data_from(i);
                    //     }
                    //     //Report failure for this grasp
                    //     return false;
                    // }
                    
                    //If we made it through, add the IK information for that state index, move on...
                    grasping_query->set_data(state_index, data);
                    state_index++;
                }
                //If it worked for all the object states, report success
                return true;
            }
            
            bool grasping_planner_t::compute_grasping_plans( const std::vector<config_t>& grasping_configs )
            {
                unsigned state_index = 0;
                //We know we have grasping states already in the query, so for each of those object poses
                foreach( std::vector< grasp_data_t* >& pose_grasp_data, grasping_query->grasp_data )
                {
                    //== IK ==
                    // if( !compute_plans( pose_grasp_data.back(), state_index, grasping_configs[state_index] ) )
                    // {
                    //     //We need to clear out all previous successful states, so people don't accidentally use them
                    //     for(int i = grasping_query->grasp_data.size()-1; i >= 0; --i)
                    //     {
                    //         grasping_query->remove_last_data_from(i);
                    //     }
                    //     return false;
                    // }
                    
                    // //== Planning ==
                    // //If we have been given goals for planning to
                    // if( !pose_grasp_data[grasp_index]->planning_goals.empty() )
                    // {
                    //     if( !compute_plans( pose_grasp_data[grasp_index], state_index, false ) )
                    //     {
                    //         //Report failure
                    //         return false;
                    //     }
                    // }
                    
                    ++state_index;
                }
                
                return true;
            }
            
            //This function can set the reason for IK failure
            bool grasping_planner_t::compute_plans( grasp_data_t* grasp_data, unsigned state_index, const config_t& grasping_config )
            {
                //A simple renaming/alias for our tmp_path
                trajectory_t& grasped_retraction_path = tmp_path;
                trajectory_t grasped_reaching_path = grasped_retraction_path;
                trajectory_t& open_retraction_path = tmp_path;
                trajectory_t open_reaching_path = grasped_retraction_path;
                
                workspace_trajectory_t ee_traj;
                
                //First, let's compute the retraction config if we are doing IK stuff

                retraction_config = grasping_query->retraction_config;
                retraction_config.relative_to_global( grasping_config );

                //====================================================
                //= Planning with the Object being grasped
                //====================================================

                //If we need to retract or reach with the object,
                // We can skip this object retraction plan only if we are solving PICK without RETRACT
                if( !( grasping_query->task_mode == TASK_PICK && !grasping_query->manipulator_retracts ) )
                {
                    //Move the object back to its original state according to the query
                    grasping_query->object->get_state_space()->copy_from_point( grasping_query->object_states[ state_index ] );
                    //Move the manipulator to the released state
                    manipulation_model->push_state( grasp_data->releasing_state );
                    //Engage grasp, so the manipulator will be moving that object
                    manipulation_model->engage_grasp( tmp_plan, grasp_data->relative_grasp->grasping_mode, false );


                    //PRX_PRINT ("Manipulation world model state: " << manipulation_model->get_full_state_space()->print_memory(3), PRX_TEXT_MAGENTA);
   
                    //PRX_PRINT ("VS. Grasping state: " << manipulator_info->full_arm_state_space->print_point(grasp_data->grasping_state,3), PRX_TEXT_MAGENTA);

                    // Clear the tmp plan for use
                    tmp_plan.clear();

                    if( !manipulation_model->jac_steering( tmp_plan, ee_traj, grasp_data->retracted_closed_state, grasp_data->grasping_state, retraction_config ) )
                    {
                        grasping_query->reason_for_failure = "Retracting (Grasping) Plan failed during IK_steering";
                        return false;
                    }

                    //Propagate the retraction plan from the grasping state (getting the grasping retraction)
                    grasped_retraction_path.clear();
                    manipulation_model->propagate_plan( grasp_data->grasping_state, tmp_plan, grasped_retraction_path );
                }
                
                //First, Retracting with the object; we have to validity check the retraction IK trajectory with the object
                if( (grasping_query->task_mode == TASK_PICK && grasping_query->manipulator_retracts) || grasping_query->task_mode == TASK_PICK_AND_MOVE || (grasping_query->task_mode == TASK_PICK_AND_PLACE && state_index == 0) )
                {
                    //Validate and generate constraints over the resulting trajectory
                    if( !validity_checker->validate_and_generate_constraints( grasp_data->closed_retracting_constraints, grasped_retraction_path ) )
                    {
                        //If that path ends up not being valid for whatever reason, report failure
                        grasping_query->reason_for_failure = "Retracting (Grasping) Plan was in collision during IK_steering";
                        return false;
                    }
                    else
                    {
                        //TODO: Make sure we do the intersection test (non-MCR) or compute the active set intersection (MCR)

                        grasp_data->retracting_plan = tmp_plan;
                        // PRX_PRINT("GRASP PLANNER: Succesful closed retract: ", PRX_TEXT_GREEN);
                        // PRX_PRINT(grasped_retraction_path.print(3), PRX_TEXT_CYAN);
                        // PRX_PRINT(grasp_data->retracting_plan.print(3), PRX_TEXT_LIGHTGRAY);
                    }
                }
                
                //Second, Reaching with the object, Compute and validate the reaching trajectory with the object
                if( (grasping_query->task_mode == TASK_PLACE && grasping_query->manipulator_retracts) || (grasping_query->task_mode == TASK_PICK_AND_PLACE && state_index == 1) || grasping_query->task_mode == TASK_PLACE)
                {
                    //Reverse the retraction trajectory to make it a baseline for the reaching trajectory
                    grasped_reaching_path.reverse_trajectory( grasped_retraction_path );
                    //Re-steer over all of the pairs of states in the reaching trajectory to get a reaching plan
                    grasp_data->reaching_plan.clear();
                    for( unsigned i=0; i < grasped_reaching_path.size() -1; ++i )
                    {
                        manipulation_model->steering_function( grasped_reaching_path[i], grasped_reaching_path[i+1], grasp_data->reaching_plan );
                    }
                    //Repropagate the reaching plan to get an updated/refined/true reaching trajectory
                    manipulation_model->propagate_plan( grasp_data->retracted_closed_state, grasp_data->reaching_plan, grasped_reaching_path );
                    
                    //Validate and generate constraints over the resulting reaching trajectory
                    if( !validity_checker->validate_and_generate_constraints( grasp_data->closed_reaching_constraints, grasped_reaching_path ) )
                    {
                        //If that path ended up not being valid, we must also report failure here
                        grasping_query->reason_for_failure = "Reaching (Grasping) Plan was in collision during IK_steering";
                        return false;
                    }
                }
                
                //====================================================
                //= Planning without the Object (always need released)
                //====================================================
                
                if( grasping_query->task_mode != TASK_PLACE )
                {
                    //Move the object to it's original state
                    grasping_query->object->get_state_space()->copy_from_point( grasping_query->object_states[ state_index ] );
                    //Move the manipulator to the released state
                    manipulation_model->push_state( grasp_data->releasing_state );
                    //Disengage grasp, so the manipulator is not holding the object
                    manipulation_model->engage_grasp( tmp_plan, grasp_data->relative_grasp->release_mode, false );


                    //PRX_PRINT ("Manipulation world model state: " << manipulation_model->get_full_state_space()->print_memory(3), PRX_TEXT_CYAN);

                    //PRX_PRINT ("VS. Releasing state: " << manipulator_info->full_arm_state_space->print_point(grasp_data->releasing_state,3), PRX_TEXT_CYAN);

                    //Do the actual planning and see if that works
                    tmp_plan.clear();
                    if( !manipulation_model->jac_steering( tmp_plan, ee_traj, grasp_data->retracted_open_state, grasp_data->releasing_state, retraction_config ) )
                    {
                        grasping_query->reason_for_failure = "Retracting (Open) Plan failed during IK_steering";
                        return false;                    
                    }                
                    //Propagate the open retraction plan from the releasing state (getting the released retraction)
                    open_retraction_path.clear();
                    manipulation_model->propagate_plan( grasp_data->releasing_state, tmp_plan, open_retraction_path );
                }
                
                //Third, Retracting without the object; validity check the retraction IK trajectory without the object
                if( (grasping_query->task_mode == TASK_PLACE && grasping_query->manipulator_retracts) || (grasping_query->task_mode == TASK_PICK_AND_PLACE && state_index == 1) )
                {
                    //Validate and generate constraints over the resulting trajectory
                    if( !validity_checker->validate_and_generate_constraints( grasp_data->open_retracting_constraints, open_retraction_path ) )
                    {
                        //If that path ends up not being valid for whatever reason, report failure
                        grasping_query->reason_for_failure = "Retracting (Open) Plan was in collision during IK_steering";
                        return false;
                    }
                    grasp_data->retracting_plan = tmp_plan;
                }

                //Fourth, Reaching without the object; compute and validate the reaching trajectory without the object
                if( grasping_query->task_mode == TASK_PICK || grasping_query->task_mode == TASK_PICK_AND_MOVE || (grasping_query->task_mode == TASK_PICK_AND_PLACE && state_index == 0) )
                {
                    //Reverse the retraction trajectory to make it a baseline for the reaching trajectory
                    open_reaching_path.reverse_trajectory( open_retraction_path );
                    //Re-steer over all of the pairs of states in the reaching trajectory to get a reaching plan
                    grasp_data->reaching_plan.clear();
                    for( unsigned i=0; i < open_reaching_path.size() -1; ++i )
                    {
                        manipulation_model->steering_function( open_reaching_path[i], open_reaching_path[i+1], grasp_data->reaching_plan );
                    }
                    //Repropagate the reaching plan to get an updated/refined/true reaching trajectory
                    manipulation_model->propagate_plan( grasp_data->retracted_open_state, grasp_data->reaching_plan, open_reaching_path );
                    
                    //Validate and generate constraints over the resulting reaching trajectory
                    if( !validity_checker->validate_and_generate_constraints( grasp_data->open_reaching_constraints, open_reaching_path ) )
                    {
                        //If that path ended up not being valid, we must also report failure here
                        grasping_query->reason_for_failure = "Reaching (Open) Plan was in collision during IK_steering";
                        return false;
                    }
                    else
                    {
                        // PRX_PRINT("GRASP PLANNER: Succesful open reach: ", PRX_TEXT_GREEN);
                        // PRX_PRINT(open_reaching_path.print(3), PRX_TEXT_CYAN);
                        // PRX_PRINT(grasp_data->reaching_plan.print(3), PRX_TEXT_LIGHTGRAY);
                    }
                }
                
                return true;
            }
            
            
            bool grasping_planner_t::evaluate_the_query()
            {
                PRX_PRINT("Current manipulation context: "<<manipulation_model->get_current_manipulation_info()->full_arm_context_name ,PRX_TEXT_RED);
                // PRX_PRINT("Current manipulation state: "<<manipulation_model->get_state_space()->print_memory(2),PRX_TEXT_RED);
                PRX_PRINT("Grasping query object : " << grasping_query->object->get_object_type(), PRX_TEXT_BLUE);
                bool found_solution = false;
                unsigned grasp_counter = 0;
                unsigned successes = 0;

                // //If we are reusing information and looking for plans
                // if( grasping_query->grasping_mode == GRASPING_PLANS && grasping_query->reuse_grasping_collision_checks )
                // {
                //     //If we have truly been given some data, then we need to iterate over all grasps, and pose 0 should have all of the grasps
                //     if( !grasping_query->grasp_data.empty() && !grasping_query->grasp_data[0].empty() )
                //     {
                //         //Then, for each grasp
                //         std::vector< bool > retrieved_solutions;
                //         for( unsigned i=0; i < grasping_query->grasp_data[0].size(); ++i )
                //         {
                //             //Evaluate the already existing grasp data for index i
                //             retrieved_solutions.push_back( evaluate_the_grasp_data( i ) );
                //             found_solution = retrieved_solutions.back() || found_solution;
                //         }
                //         //Now, remove all of the grasps for which we did not find a solution
                //         remove_unsolved_grasp_data_entries( retrieved_solutions );
                //     }
                //     else
                //     {
                //         PRX_WARN_S("Grasping planner was told to reuse grasp information, but none was found in the query!");
                //     }
                // }
                // //Otherwise, we are doing any other query
                // else

                // Original grasp planning code
                // {
                //     //We have to iterate over all of the grasps in the database and test them all
                //     foreach( grasp_t grasp, grasps[ grasping_query->object->get_object_type() + manipulation_model->get_current_manipulation_info()->full_arm_context_name ] )
                //     {
                //         PRX_DEBUG_COLOR("Grasp counter : " << grasp_counter, PRX_TEXT_GREEN);
                //         //PRX_DEBUG_COLOR ("Before world model state: " << manipulation_model->get_full_state_space()->print_memory(3), PRX_TEXT_MAGENTA);
                //         bool temp_solution = evaluate_the_grasp( &grasp );
                //         found_solution = temp_solution || found_solution;
                //         successes+=temp_solution;
                //         if (!temp_solution)
                //         {

                //             //PRX_DEBUG_COLOR("Failure: " << grasping_query->reason_for_failure, PRX_TEXT_RED);
                //         }
                //         //PRX_DEBUG_COLOR ("AFTER world model state: " << manipulation_model->get_full_state_space()->print_memory(3), PRX_TEXT_LIGHTGRAY);
                //         ++grasp_counter;
                //     }
                // }

                std::string context = manipulation_model->get_current_manipulation_info()->full_arm_context_name;
                std::string hash_string = grasping_query->object->get_object_type() + context;

                // Check if we have a loaded grasping database for this object + arm_context
                if (grasps.find(hash_string) != grasps.end())
                {
                    //We have to iterate over all of the grasps in the database and test them all
                    foreach( grasp_t grasp, grasps[hash_string] )
                    {
                        PRX_DEBUG_COLOR("Grasp counter : " << grasp_counter, PRX_TEXT_GREEN);
                        //PRX_DEBUG_COLOR ("Before world model state: " << manipulation_model->get_full_state_space()->print_memory(3), PRX_TEXT_MAGENTA);
                        bool temp_solution = evaluate_the_grasp( &grasp );
                        found_solution = temp_solution || found_solution;
                        successes+=temp_solution;
                        if (!temp_solution)
                        {

                            //PRX_DEBUG_COLOR("Failure: " << grasping_query->reason_for_failure, PRX_TEXT_RED);
                        }
                        //PRX_DEBUG_COLOR ("AFTER world model state: " << manipulation_model->get_full_state_space()->print_memory(3), PRX_TEXT_LIGHTGRAY);
                        ++grasp_counter;
                    }
                    PRX_INFO_S("Grasping Planner Successes: "<<successes<<"/"<<grasp_counter);
                }
                else if (grasp_generators.find(context) != grasp_generators.end())
                {
                    PRX_DEBUG_COLOR("Found grasp generator for arm context: " <<context, PRX_TEXT_BROWN );

                    geometry_t* ee_geom = manipulator_info->manipulator->get_end_effector_geometry(manipulator_info->end_effector_index);

                    std::vector<grasp_t> grasps;
                    if (!grasp_generators[context]->get_grasps(grasps, grasping_query->object->get_object_type()))
                    {
                        grasps = grasp_generators[context]->compute_grasps(ee_geom, ee_local_config, grasping_query->object);
                        append_to_stat_file("GRAP STATS:\n" + grasp_generators[context]->get_stat_string());
                    }

                    for( auto grasp : grasps )
                    {
                        PRX_DEBUG_COLOR("Grasp counter : " << grasp_counter, PRX_TEXT_GREEN);
                        //PRX_DEBUG_COLOR ("Before world model state: " << manipulation_model->get_full_state_space()->print_memory(3), PRX_TEXT_MAGENTA);
                        bool temp_solution = evaluate_the_grasp( &grasp );
                        found_solution = temp_solution || found_solution;
                        successes+=temp_solution;
                        if (!temp_solution)
                        {

                            //PRX_DEBUG_COLOR("Failure: " << grasping_query->reason_for_failure, PRX_TEXT_RED);
                        }
                        //PRX_DEBUG_COLOR ("AFTER world model state: " << manipulation_model->get_full_state_space()->print_memory(3), PRX_TEXT_LIGHTGRAY);
                        ++grasp_counter;
                    }
                    PRX_INFO_S("Grasping Planner Successes: "<<successes<<"/"<<grasp_counter);
                    PRX_PRINT ("FULL STATE: " << manipulation_model->get_full_state_space()->print_memory(4), PRX_TEXT_RED);
                }
                else
                {
                    PRX_FATAL_S ("No grasp database or grasp generator specified for arm: " << context << " and object: " << grasping_query->object->get_object_type());
                }

                
                return found_solution;
            }
            
            void grasping_planner_t::remove_unsolved_grasp_data_entries( const std::vector< bool >& found_solutions )
            {
                for( int i = found_solutions.size()-1; i >= 0; --i )
                {
                    //If we didn't find a solution for this grasp
                    if( !found_solutions[i] )
                    {
                        //For each object pose
                        for( unsigned j=0; j<grasping_query->grasp_data.size(); ++j )
                        {
                            //Remove this grasp from the data
                            grasping_query->grasp_data[j].erase( grasping_query->grasp_data[j].begin() + i );
                        }
                    }
                }
            }

            bool grasping_planner_t::valid_config_for_states(const grasp_t* grasp)
            {
                return true;
            }

            int grasping_planner_t::nr_grasps( std::string context_name,movable_body_plant_t* object)
            {
                return grasps[object->get_object_type()+context_name].size();
            }

            void grasping_planner_t::set_param(const std::string& path, const std::string& parameter_name, const boost::any& value)
            {
                std::string name;
                std::string subpath;
                boost::tie(name, subpath) = split_path(path);

                if( !name.empty() )
                    PRX_FATAL_S("Error! You are trying to set a parameter to a planner under grasping planner, where it is empty.");
                    
                set_param(parameter_name, value);
                
            }

            void grasping_planner_t::set_param(const std::string& parameter_name, const boost::any& value)
            {
                planner_t::set_param(parameter_name, value);
            }

            void grasping_planner_t::state_to_config(config_t& config, state_t* state)
            {
                PRX_ASSERT(state->memory.size() == 7);
                config.set_position(state->memory[0], state->memory[1], state->memory[2]);
                config.set_orientation(state->memory[3], state->memory[4], state->memory[5], state->memory[6]);
            }
        }
    }
}
