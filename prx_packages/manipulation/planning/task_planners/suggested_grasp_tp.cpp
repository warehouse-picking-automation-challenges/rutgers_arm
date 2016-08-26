/**
 * @file suggested_grasp_tp.cpp
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

#include "planning/task_planners/suggested_grasp_tp.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/heuristic_search/object_collision_constraints.hpp"

#include "planning/specifications/simple_pap_specification.hpp"

#include <boost/range/adaptor/map.hpp>
#include <pluginlib/class_list_macros.h>

#include "planning/task_planners/manipulation_tp.hpp"
#include "utilities/definitions/manip_defs.hpp" 
#include "prx/utilities/definitions/sys_clock.hpp"
#include "planning/task_planners/grasp_rrt_tp.hpp"

#include <string>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::suggested_grasp_tp_t ,prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace manipulation
        {

            suggested_grasp_tp_t::suggested_grasp_tp_t()
            {

            }

            suggested_grasp_tp_t::~suggested_grasp_tp_t()
            {

            }

            void suggested_grasp_tp_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                PRX_DEBUG_COLOR("Initializing suggested_grasp_tp_t task planner ...", PRX_TEXT_CYAN);
                task_planner_t::init(reader,template_reader);                
                full_manipulator_context_name = parameters::get_attribute("full_manipulator_context_name", reader, template_reader);
                manipulation_context_name = parameters::get_attribute("manipulation_context_name", reader, template_reader);
                object_target_vec = parameters::get_attribute_as<std::vector<double> >("object_target", reader, template_reader);
                manipulation_task_planner_name = parameters::get_attribute("manipulation_task_planner_name", reader, template_reader);
                object_name = parameters::get_attribute("object_name", reader, template_reader);
            }

            void suggested_grasp_tp_t::link_world_model(world_model_t * const model)
            {
                task_planner_t::link_world_model(model);
                manipulation_model = dynamic_cast<manipulation_world_model_t*>(model);
                if(manipulation_model == NULL)
                    PRX_FATAL_S("The pick and place task planner can work only with manipulation world model!");
            }

            void suggested_grasp_tp_t::link_query(query_t* new_query)
            {
                task_planner_t::link_query(new_query);
                in_query = static_cast<motion_planning_query_t*>(new_query);
            }

            void suggested_grasp_tp_t::setup()
            {
                manipulation_model->use_context(full_manipulator_context_name);
                manip_initial_state = manipulation_model->get_state_space()->alloc_point();

                foreach(planner_t* planner, planners | boost::adaptors::map_values)
                {
                    PRX_DEBUG_COLOR("Simple P&P is seting up planner: " << planner->get_name(), PRX_TEXT_CYAN);
                    planner->setup();
                    output_specifications[planner->get_name()]->setup( manipulation_model );
                    planner->link_specification( output_specifications[planner->get_name()] );
                }
            }

            bool suggested_grasp_tp_t::execute()
            {
                foreach(planner_t* planner, planners | boost::adaptors::map_values)
                {                    
                    PRX_PRINT("Simple P&P executing planner: " << planner->get_name(), PRX_TEXT_GREEN);
                    planner->execute();
                }
                return true;
            }

            bool suggested_grasp_tp_t::succeeded() const
            {
                return true;
            }

            const util::statistics_t* suggested_grasp_tp_t::get_statistics()
            {
                return NULL;
            }

            void suggested_grasp_tp_t::resolve_query()
            {
                config_t retract_config;
                retract_config.set_position(0.0,0.0,-0.03);
                retract_config.set_orientation(0,0,0,1);
                PRX_DEBUG_COLOR("Resolve query from suggested_grasp_tp ...  context:" << manipulation_context_name,PRX_TEXT_GREEN);
                manipulation_model->use_context(full_manipulator_context_name);
                manipulation_model->get_state_space()->copy_from_point(manip_initial_state);  

                manipulation_model->use_context(manipulation_context_name);
                std::vector<movable_body_plant_t* > objects;
                manipulation_model->get_objects(objects);

                movable_body_plant_t* move_object = NULL;
                std::string name, subpath;
                for(unsigned i = 0; i<objects.size() && move_object == NULL; ++i)
                {
                    boost::tie(subpath, name) = reverse_split_path(objects[i]->get_pathname());
                    PRX_DEBUG_COLOR("name : " << name << "  subpath:" << subpath, PRX_TEXT_GREEN);
                    if(name == object_name)
                        move_object = objects[i];
                }

                const space_t* object_space = move_object->get_state_space();

                manipulator = manipulation_model->get_current_manipulation_info()->manipulator;                
                state_t* initial_state = manipulation_model->get_state_space()->alloc_point();

                state_t* initial_object = object_space->alloc_point();
                state_t* target_object = object_space->alloc_point();
                object_space->copy_vector_to_point(object_target_vec, target_object);

                manipulation_query = dynamic_cast< manipulation_query_t* >( output_queries[ manipulation_task_planner_name ] );
                manipulation_query->smooth_paths = false;
                //Have to at least do a dummy setup and link so that the manipulation task planner sets up context information correctly
                
                // manipulation_query->setup_pick( manipulation_context_name, false, GRASP_GREEDY, move_object, 1, retract_config, initial_state, initial_object );
                // manipulation_query->setup_pick_via_config_and_move( manipulation_context_name, GRASP_GREEDY, move_object, 1, retract_config, initial_state, initial_state, initial_object );                
                //manipulation_query->setup_pick_and_place( manipulation_context_name,  true, GRASP_GREEDY, move_object, 1, retract_config, initial_state, initial_object, target_object);
                manipulation_query->setup_pick_and_move( manipulation_context_name, GRASP_GREEDY, move_object, 1, retract_config, initial_state, initial_state, initial_object );
                

                simple_pap_specification_t* pap_spec = dynamic_cast< simple_pap_specification_t* >(input_specification);
                manipulation_query->path_constraints = pap_spec->validity_checker->alloc_constraint();
                constraints_t* valid_constraints = pap_spec->validity_checker->alloc_constraint();
                
                //Need to push in an index for each of the movable bodies in the world?
                object_collision_constraints_t* occ = dynamic_cast< object_collision_constraints_t* >( valid_constraints );
                if( occ != NULL )
                {
                    PRX_PRINT("SUGGESTED TP: Cast to Object Collision Constraints worked!", PRX_TEXT_GREEN);
                    for( unsigned i=0; i<objects.size(); ++i )
                    {
                        occ->constraints.insert( i );
                    }
                }
                else
                {
                    PRX_PRINT("SUGGESTED TP: Cast to Object Collision Constraints DID NOT WORK!", PRX_TEXT_RED);                    
                }
                manipulation_query->set_valid_constraints( valid_constraints );
                
                //Here is the dummy link
                planners[manipulation_task_planner_name]->link_query(manipulation_query);

                
                //Object states needed to "resolve_grasp_query"
                std::vector< sim::state_t* > object_states;
                object_states.push_back(initial_object);                
                object_states.push_back(target_object);
                util::sys_clock_t _clock;
                _clock.reset();
                //Get the set of grasps from the grasping planner
                 manipulation_tp_t* manip_tp = dynamic_cast< manipulation_tp_t* >(planners[manipulation_task_planner_name]);
                //grasp_rrt_tp_t* manip_tp = dynamic_cast< grasp_rrt_tp_t* >(planners[manipulation_task_planner_name]);
                std::vector< grasp_t > grasps_to_suggest = manip_tp->get_grasps(object_states);
                PRX_INFO_S("\n\nNUMBER OF GRASPS TO SUGGEST::: "<<grasps_to_suggest.size());
                state_t* full_initial_state = manipulation_model->get_state_space()->alloc_point();
                trajectory_t full_trajectory;
                full_trajectory.link_space(manipulation_model->get_state_space());
                state_t* full_intermediate_state = manipulation_model->get_state_space()->alloc_point();
                //Loop through the grasps and suggest them one by one

                planners[manipulation_task_planner_name]->resolve_query();
                PRX_PRINT ("------------------STATS--------------: ", PRX_TEXT_RED);
                PRX_PRINT ("Total time: " << _clock.measure(), PRX_TEXT_RED);
                PRX_PRINT ("Path Length: " << manipulation_query->plan.length(), PRX_TEXT_RED);

                // return;


                
                // util::sys_clock_t _clock;
                // util::sys_clock_t _total_clock;
                // _total_clock.reset();
                // double successes = 0;
                // double tries = 0;
                // double total_path_quality = 0;
                // double total_success_time = 0;
                // double total_failure_time = 0;


                // for( unsigned i = 0; i< grasps_to_suggest.size() && ros::ok() ; ++i)
                // {
                //     tries++;
                //     PRX_PRINT(grasps_to_suggest[i].relative_config.print(), PRX_TEXT_RED);
                //     manipulation_query->clear();
                //     std::stringstream output_text;
                //     output_text<<"\nGrasp [" << i << "] : "<<grasps_to_suggest[i].relative_config.print();
                //     append_to_stat_file( output_text.str() );
                //     _clock.reset();
                    
                //     //manipulation_query->setup_pick_and_place( manipulation_context_name,  true, GRASP_SUGGESTED, move_object, 1, retract_config, initial_state, initial_object, target_object, &(grasps_to_suggest[i]));
                //     manipulation_query->setup_pick_and_move( manipulation_context_name, GRASP_SUGGESTED, move_object, 1, retract_config, initial_state, initial_state, initial_object, &(grasps_to_suggest[i]) );
                //     // manipulation_query->setup_pick( manipulation_context_name, true, GRASP_SUGGESTED, move_object, 1, retract_config, initial_state, initial_object, &(grasps_to_suggest[i]) );
                    
                //     manipulation_query->set_valid_constraints( valid_constraints );
                //     planners[manipulation_task_planner_name]->link_query(manipulation_query);
                //     planners[manipulation_task_planner_name]->resolve_query();
                //     double current_path_quality = manipulation_query->plan.length();
                //     // To get an usable plan and return the first time a suggestion works. 
                //     if(manipulation_query->found_path)
                //     {   
                //         // //Move from the retracted configuration back to the start configuration of the arm, to simulate a complete APC type task
                //         // manipulation_model->propagate_plan(full_initial_state, manipulation_query->plan, full_trajectory);
                //         // manipulation_model->get_state_space()->copy_to_point(full_intermediate_state);
                //         // PRX_PRINT("\n\n\n\nGoing to move from "<<manipulation_model->get_state_space()->print_point(full_intermediate_state, 3)<<" to "<<manipulation_model->get_state_space()->print_point(initial_state, 3)<<"\n\n\n", PRX_TEXT_RED);
                //         // manipulation_query->setup_move(manipulation_context_name, full_intermediate_state, initial_state);
                //         // planners[manipulation_task_planner_name]->link_query(manipulation_query);
                //         // planners[manipulation_task_planner_name]->resolve_query();
                //         if(manipulation_query->found_path)
                //         {
                //             successes++;
                //             double current_success_time = _clock.measure();
                //             total_success_time+=current_success_time;
                //             total_path_quality+=current_path_quality;
                //             append_to_stat_file("\n  ######## Successful Pick and Place");
                //             append_to_stat_file( "\n  Time taken: " + boost::lexical_cast<std::string>(current_success_time) );
                //             append_to_stat_file( "\n  Total Time taken: " + boost::lexical_cast<std::string>(_total_clock.measure()) );
                //             break;
                //         }
                //         else
                //         {
                //             total_failure_time+=_clock.measure();
                //             append_to_stat_file( "\n  Move Failed from target pose." );
                //         }
                        
                //     }
                //     else
                //     {
                //         total_failure_time+=_clock.measure();
                //     }

                //     append_to_stat_file( "\n  Time taken: " + boost::lexical_cast<std::string>(_clock.measure()) );
                //     append_to_stat_file( "\n  Total Time taken: " + boost::lexical_cast<std::string>(_total_clock.measure()) );
                // }

                // append_to_stat_file("\n*************************************************************************");
                // append_to_stat_file("\nGrasps / Database     : "+boost::lexical_cast<std::string>(manip_tp->_grasp_success_count)+" / "+boost::lexical_cast<std::string>(tries)+" = "+boost::lexical_cast<std::string>(manip_tp->_grasp_success_count/(double)tries));
                // append_to_stat_file("\nPlans  / Grasps       : "+boost::lexical_cast<std::string>(successes)+" / "+boost::lexical_cast<std::string>(manip_tp->_grasp_success_count)+" = "+boost::lexical_cast<std::string>((double)successes/(double)(manip_tp->_grasp_success_count)));
                // append_to_stat_file("\nPlans  / Database     : "+boost::lexical_cast<std::string>(successes)+" / "+boost::lexical_cast<std::string>(tries)+" = "+boost::lexical_cast<std::string>((double)successes/tries));
                // append_to_stat_file("\nSuccess time(Grasps)  : "+boost::lexical_cast<std::string>(manip_tp->_grasp_success_time/(double)manip_tp->_grasp_success_count));
                // append_to_stat_file("\nSuccess time(Plans)   : "+boost::lexical_cast<std::string>(manip_tp->_mp_success_time/(double)manip_tp->_mp_success_count));
                // append_to_stat_file("\nSuccess time(Database): "+boost::lexical_cast<std::string>(total_success_time/(double)successes));
                // append_to_stat_file("\nPath cost(Plans)      : "+boost::lexical_cast<std::string>(total_path_quality/(double)successes));
                // append_to_stat_file("\nFailure time(Grasps)  : "+boost::lexical_cast<std::string>(manip_tp->_grasp_failure_time/(double)(manip_tp->_grasp_failure_count)));
                // append_to_stat_file("\nFailure time(Plans)   : "+boost::lexical_cast<std::string>(manip_tp->_mp_failure_time/(double)(manip_tp->_mp_failure_count)));
                // append_to_stat_file("\nFailure time(Database): "+boost::lexical_cast<std::string>(total_failure_time/(double)(tries-successes)));
                // append_to_stat_file("\n*************************************************************************");


                //################################### End of Suggested Queries ####################################


                //############################## Another set of Suggested Queries #################################
                //The following code is to rerun the suggested query loop to check for changes from IK seed differences
                // PRX_INFO_S("\n\n\n\nSECOND TRY NUMBER OF GRASPS TO SUGGEST::: "<<grasps_to_suggest.size());
                // for( unsigned i = 0; i< grasps_to_suggest.size(); ++i)
                // {
                //     PRX_ERROR_S(grasps_to_suggest[i]->relative_config.print());
                //     manipulation_query->setup(manipulation_context_name, manipulation_query_t::PRX_PICK_AND_PLACE, manipulation_query_t::PRX_GRASP_SUGGESTED, move_object, 1, retract_config, initial_state, initial_state, NULL, initial_object, target_object, grasps_to_suggest[i]);
                //     planners[manipulation_task_planner_name]->link_query(manipulation_query);
                //     planners[manipulation_task_planner_name]->resolve_query();
                // }
                //################################### End of Suggested Queries ####################################


                //############################ Helper Code in manipulation_tp_t::resolve_grasp_query ##############
                // Place this code instead of grasping_query->clear() in the resolve_grasp_query function. Otherwise, code tends to crash horribly.
                // for(int i=0;i<grasping_query->grasp_data.size();++i)
                // {
                //     grasping_query->grasp_data[i].clear();
                // }
                //################################## End of helper code ###########################################


                //################## A Possible Way to Get Grasps as a function in manipulation_tp_t ##############
                // std::vector<grasp_t* > manipulation_tp_t::get_grasps(std::vector<state_t* > object_states, int open_mode)
                // {
                //     resolve_grasp_query(object_states, grasping_query_t::PRX_GRASP_WITHOUT_COLLISION, open_mode, NULL);
                //     std::vector<grasp_t* > grasps_to_suggest;
                //     for(int i=0;i<grasping_query->grasp_data[0].size();++i)
                //     {
                //         grasps_to_suggest.push_back(grasping_query->grasp_data[0][i]->relative_grasp);
                //     }
                //     return grasps_to_suggest;
                // }
                //##################################### End of function to get grasps #############################


                //REPORT the final plan
                in_query->link_spaces(manipulator->get_state_space(), manipulator->get_control_space());
                manipulation_model->convert_plan(in_query->plan, manipulator->get_control_space(), manipulation_query->plan, manipulation_model->get_control_space());
                // manipulation_model->convert_plan(in_query->plan, manipulator->get_control_space(), query2->plan, manipulation_model->get_control_space());

                manipulation_model->use_context(full_manipulator_context_name);
                
                delete valid_constraints;
            }
        }
    }
}

