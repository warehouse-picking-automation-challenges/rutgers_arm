/**
 * @file case_eval_task_planner.cpp
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

#include "planning/case_eval_task_planner.hpp"
#include "planning/specifications/manipulation_specification.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "utilities/definitions/manip_defs.hpp"

#include <boost/range/adaptor/map.hpp>
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>

#include <time.h>

#define MAX_TRIES 9999999999

PLUGINLIB_EXPORT_CLASS(prx::packages::apc::case_eval_task_planner_t ,prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        using namespace manipulation;
        namespace apc
        {

            case_eval_task_planner_t::case_eval_task_planner_t()
            {

            }

            case_eval_task_planner_t::~case_eval_task_planner_t()
            {

            }

            void case_eval_task_planner_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                PRX_INFO_S("Initializing case_eval_task_planner_t task planner ...");
                task_planner_t::init(reader,template_reader);
                left_context_name = parameters::get_attribute("left_context_name", reader, template_reader);
                right_context_name = parameters::get_attribute("right_context_name", reader, template_reader);
                manipulation_task_planner_name = parameters::get_attribute("manipulation_task_planner_name", reader, template_reader);

                std::vector<const parameter_reader_t*> readers = parameters::get_list("object_poses", reader, template_reader);
                std::string geom_name;

                foreach(const parameter_reader_t* r, readers)
                {
                    object_poses.push_back(r->get_attribute_as< std::vector<double> > ("pose"));
                }

                manip_planner = dynamic_cast<manipulation_tp_t*>(planners[manipulation_task_planner_name]);

                // profile = new examination_profile_t();
                // profile->focus.push_back(.8-gripper_offset);
                // profile->focus.push_back(0);
                // profile->focus.push_back(1.45);
                // profile->base_viewpoint.set(0,1,0,1);
                // profile->base_viewpoint.normalize();
                // profile->offsets.push_back(profile->base_viewpoint);
                // profile->offsets.push_back(profile->base_viewpoint);
                // profile->distance = .3;
                // camera_positions['B'] = profile;

                // profile = new examination_profile_t();
                // profile->focus.push_back(.8-gripper_offset);
                // profile->focus.push_back(0);
                // profile->focus.push_back(1.20);
                // profile->base_viewpoint.set(0,1,0,1);
                // profile->base_viewpoint.normalize();
                // profile->offsets.push_back(profile->base_viewpoint);
                // profile->offsets.push_back(profile->base_viewpoint);
                // profile->distance = .3;
                // camera_positions['E'] = profile;

                left_arm_order_bin  = {-1.57,1.661659,0.677508,0,-1.120185,0,-0.165669,0,0};
                right_arm_order_bin = { 1.57,1.661659,0.677508,0,-1.120185,0,-0.165669,0};
            }

            void case_eval_task_planner_t::link_world_model(world_model_t * const model)
            {
                task_planner_t::link_world_model(model);
                manipulation_model = dynamic_cast<manipulation_world_model_t*>(model);
                if(manipulation_model == NULL)
                    PRX_FATAL_S("The manipulation task planner can work only with manipulation world model!");
            }

            void case_eval_task_planner_t::link_query(query_t* new_query)
            {
                task_planner_t::link_query(new_query);
                // in_query = static_cast<apc_task_query_t*>(new_query);
            }

            void case_eval_task_planner_t::setup()
            {
                output_specifications[manip_planner->get_name()]->setup( manipulation_model );
                manip_planner->link_specification(output_specifications[manipulation_task_planner_name]);
                foreach(planner_t* planner, planners | boost::adaptors::map_values)
                {
                    PRX_INFO_S("setup planner: " << planner->get_name());
                    planner->setup();
                }
            }

            bool case_eval_task_planner_t::execute()
            {
                foreach(planner_t* planner, planners | boost::adaptors::map_values)
                {                    
                    PRX_INFO_S("execute planner: " << planner->get_name());
                    planner->execute();
                    // planner->update_visualization();
                }
                left_manipulation_query = new manipulation_query_t();
                right_manipulation_query = new manipulation_query_t();

                manipulation_model->use_context(left_context_name);
                manipulator = manipulation_model->get_current_manipulation_info()->manipulator;
                full_manipulator_state_space = manipulation_model->get_current_manipulation_info()->full_manipulator_state_space;
                full_manipulator_control_space = manipulation_model->get_current_manipulation_info()->full_manipulator_control_space;

                manipulation_specification_t* manip_spec = dynamic_cast<manipulation_specification_t*>(output_specifications[manipulation_task_planner_name]);
                left_manipulation_query->path_constraints = manip_spec->validity_checker->alloc_constraint();
                left_manipulation_query->set_valid_constraints(manip_spec->validity_checker->alloc_constraint());
                left_manipulation_query->set_search_mode(LAZY_SEARCH);
                right_manipulation_query->path_constraints = manip_spec->validity_checker->alloc_constraint();
                right_manipulation_query->set_valid_constraints(manip_spec->validity_checker->alloc_constraint());
                right_manipulation_query->set_search_mode(LAZY_SEARCH);


                std::vector<movable_body_plant_t* > objects;
                manipulation_model->get_objects(objects);

                movable_body_plant_t* chosen_object = objects[0];
                PRX_INFO_S("Case Evaluation for Object: "<<chosen_object->get_object_type());
                std::vector<std::string> hands;
                hands.push_back("left");
                hands.push_back("right");
                //setup for the case experiments and data gathering
                //foreach pose in poses
                int pose_index = 0;

                std::stringstream ss;
                ss<<"\nPoses: \n";
                foreach(std::vector<double> pose, object_poses)
                {
                    foreach(double val, pose)
                        ss<<val<<" , ";
                    ss<<"\n";
                }
                append_to_stat_file("\n"+ss.str()+"\n\n\n");
                foreach(std::vector<double> pose, object_poses)
                {
                    PRX_PRINT("New Pose!",PRX_TEXT_RED);
                    //   reset manipulation tp stats
                    //   set object in that pose
                    //   foreach hand in hands
                    foreach(std::string hand, hands)
                    {
                        chosen_object->get_state_space()->set_from_vector(pose);
                        //     call evaluate_pose(object,hand)
                        PRX_PRINT("Eval pose "<<hand,PRX_TEXT_GREEN);
                        evaluate_pose(chosen_object,hand,pose_index);
                        //     check results, append release command to plans, output plans
                    }
                    pose_index++;
                }



                return true;
            }

            bool case_eval_task_planner_t::succeeded() const
            {
                return true;
            }

            const util::statistics_t* case_eval_task_planner_t::get_statistics()
            {
                return NULL;
            }

            bool case_eval_task_planner_t::evaluate_pose(movable_body_plant_t* object,std::string hand, int pose_index)
            {
                config_t retract_config;
                std::string manipulation_context_name;
                if(hand=="left")
                {
                    retract_config.set_position(0,0,-.03);
                    retract_config.set_orientation(0,0,0,1);
                    manipulation_context_name = left_context_name;
                    manipulation_model->use_context(left_context_name);
                    manipulation_query = left_manipulation_query;
                }
                else
                {
                    retract_config.set_position(0,0,-.07);
                    retract_config.set_orientation(0,0,0,1);
                    manipulation_context_name = right_context_name;
                    manipulation_model->use_context(right_context_name);
                    manipulation_query = right_manipulation_query;
                }


                state_t* initial_state = manipulation_model->get_state_space()->alloc_point();
                state_t* result_state = manipulation_model->get_state_space()->alloc_point();
                const space_t* object_space = object->get_state_space();
                state_t* stored_object_state = object_space->alloc_point();
                state_t* initial_object = object_space->alloc_point();
                PRX_PRINT("\n\n\n Initial Object Pose::::: "<<object_space->print_point(initial_object, 3)<<"\n\n\n", PRX_TEXT_RED);

                bool success = false;
                int previous_grasp_mode = manipulation_model->get_current_grasping_mode();


                std::vector<state_t*> object_states;
                object_states.push_back(stored_object_state);


                manipulation_query->setup_pick( manipulation_context_name, true, GRASP_GREEDY, object, previous_grasp_mode, retract_config, initial_state, initial_object );
                manip_planner->link_query(manipulation_query);
                /////////////From suggested grasp tp
                std::vector< grasp_t > grasps_to_suggest = manip_planner->get_grasps(object_states);
                PRX_INFO_S("\n\nNUMBER OF GRASPS TO SUGGEST::: "<<grasps_to_suggest.size());
                state_t* full_intermediate_state = manipulation_model->get_state_space()->alloc_point();
                //Loop through the grasps and suggest them one by one

                util::sys_clock_t _clock;
                util::sys_clock_t _total_clock;
                _total_clock.reset();
                double successes = 0;
                double tries = 0;
                double total_path_quality = 0;
                double total_success_time = 0;
                double total_failure_time = 0;
                std::vector<plan_t*> output_plans;

                for( unsigned i = 0; i< grasps_to_suggest.size() && ros::ok() ; ++i)
                {
                    tries++;
                    PRX_PRINT(grasps_to_suggest[i].relative_config.print(), PRX_TEXT_RED);
                    manipulation_query->clear();
                    std::stringstream output_text;
                    output_text<<"\nGrasp [" << i << "] : "<<grasps_to_suggest[i].relative_config.print();
                    append_to_stat_file( output_text.str() );
                    _clock.reset();
                    
                    //manipulation_query->setup_pick_and_place( manipulation_context_name,  true, GRASP_SUGGESTED, move_object, 1, retract_config, initial_state, initial_object, target_object, &(grasps_to_suggest[i]));
                    manipulation_query->setup_pick_and_move( manipulation_context_name, GRASP_SUGGESTED, object, previous_grasp_mode, 
                        retract_config, initial_state, initial_state, initial_object, &(grasps_to_suggest[i]) );
                    // manipulation_query->setup_pick( manipulation_context_name, true, GRASP_SUGGESTED, move_object, 1, retract_config, initial_state, initial_object, &(grasps_to_suggest[i]) );
                    
                    // manipulation_query->set_valid_constraints( valid_constraints );
                    manip_planner->link_query(manipulation_query);
                    manip_planner->resolve_query();
                    double current_path_quality = manipulation_query->plan.length();
                    // To get an usable plan and return the first time a suggestion works. 
                    if(manipulation_query->found_path)
                    {   
                        // //Move from the retracted configuration back to the start configuration of the arm, to simulate a complete APC type task
                        // manipulation_model->propagate_plan(full_initial_state, manipulation_query->plan, full_trajectory);
                        // manipulation_model->get_state_space()->copy_to_point(full_intermediate_state);
                        // PRX_PRINT("\n\n\n\nGoing to move from "<<manipulation_model->get_state_space()->print_point(full_intermediate_state, 3)<<" to "<<manipulation_model->get_state_space()->print_point(initial_state, 3)<<"\n\n\n", PRX_TEXT_RED);
                        // manipulation_query->setup_move(manipulation_context_name, full_intermediate_state, initial_state);
                        // planners[manipulation_task_planner_name]->link_query(manipulation_query);
                        // planners[manipulation_task_planner_name]->resolve_query();
                        if(manipulation_query->found_path)
                        {
                            successes++;
                            double current_success_time = _clock.measure();
                            total_success_time+=current_success_time;
                            total_path_quality+=current_path_quality;
                            append_to_stat_file("\n  ######## Successful Pick and Place");
                            append_to_stat_file( "\n  Time taken: " + boost::lexical_cast<std::string>(current_success_time) );
                            append_to_stat_file( "\n  Total Time taken: " + boost::lexical_cast<std::string>(_total_clock.measure()) );
                            manipulation_model->engage_grasp( manipulation_query->plan, previous_grasp_mode, false);
                            unsigned max_num_plans = 2;
                            if(output_plans.size()<max_num_plans)
                            {
                                output_plans.push_back(new plan_t());
                                (*output_plans.back() ) = manipulation_query->plan;
                            }
                            else if(output_plans.back()->length() > manipulation_query->plan.length())
                            {
                                (*output_plans.back() ) = manipulation_query->plan;
                            }
                            //swap up
                            bool done=false;
                            for(int b_iter=output_plans.size()-1;b_iter>=1 && !done;b_iter--)
                            {
                                if(output_plans[b_iter]->length() < output_plans[b_iter-1]->length())
                                {
                                    std::swap(output_plans[b_iter],output_plans[b_iter-1]);
                                }
                                else
                                    done = true;
                            } 


                            // break;
                        }
                        else
                        {
                            total_failure_time+=_clock.measure();
                            append_to_stat_file( "\n  Move Failed from target pose." );
                        }
                        
                    }
                    else
                    {
                        total_failure_time+=_clock.measure();
                    }

                    append_to_stat_file( "\n  Time taken: " + boost::lexical_cast<std::string>(_clock.measure()) );
                    append_to_stat_file( "\n  Total Time taken: " + boost::lexical_cast<std::string>(_total_clock.measure()) );
                }

                append_to_stat_file("\n***CASEDATA**********************************************************************");
                std::stringstream ss;
                foreach(double val, object_poses[pose_index])
                    ss<<val<<", ";
                append_to_stat_file("\nObject: "+object->get_object_type()+" Hand: "+hand+" Pose: "+boost::lexical_cast<std::string>(pose_index) + " config: " + ss.str());
                append_to_stat_file("\n"+boost::lexical_cast<std::string>(successes)+"/"
                                        +boost::lexical_cast<std::string>(manip_planner->_grasp_success_count)+"/"
                                        +boost::lexical_cast<std::string>(tries)+"/"
                                        +boost::lexical_cast<std::string>(manip_planner->_grasp_success_time/(double)manip_planner->_grasp_success_count)+"/"
                                        +boost::lexical_cast<std::string>(manip_planner->_mp_success_time/(double)manip_planner->_mp_success_count)+"/"
                                        +boost::lexical_cast<std::string>(total_success_time/(double)successes)+"/"
                                        +boost::lexical_cast<std::string>(total_path_quality/(double)successes)+"/"
                                        +boost::lexical_cast<std::string>(manip_planner->_grasp_failure_time/(double)(manip_planner->_grasp_failure_count))+"/"
                                        +boost::lexical_cast<std::string>(manip_planner->_mp_failure_time/(double)(manip_planner->_mp_failure_count))+"/"
                                        +boost::lexical_cast<std::string>(total_failure_time/(double)(tries-successes)));
                append_to_stat_file("\nGrasps / Database     : "+boost::lexical_cast<std::string>(manip_planner->_grasp_success_count)+" / "+boost::lexical_cast<std::string>(tries)+" = "+boost::lexical_cast<std::string>(manip_planner->_grasp_success_count/(double)tries));
                append_to_stat_file("\nPlans  / Grasps       : "+boost::lexical_cast<std::string>(successes)+" / "+boost::lexical_cast<std::string>(manip_planner->_grasp_success_count)+" = "+boost::lexical_cast<std::string>((double)successes/(double)(manip_planner->_grasp_success_count)));
                append_to_stat_file("\nPlans  / Database     : "+boost::lexical_cast<std::string>(successes)+" / "+boost::lexical_cast<std::string>(tries)+" = "+boost::lexical_cast<std::string>((double)successes/tries));
                append_to_stat_file("\nSuccess time(Grasps)  : "+boost::lexical_cast<std::string>(manip_planner->_grasp_success_time/(double)manip_planner->_grasp_success_count));
                append_to_stat_file("\nSuccess time(Plans)   : "+boost::lexical_cast<std::string>(manip_planner->_mp_success_time/(double)manip_planner->_mp_success_count));
                append_to_stat_file("\nSuccess time(Database): "+boost::lexical_cast<std::string>(total_success_time/(double)successes));
                append_to_stat_file("\nPath cost(Plans)      : "+boost::lexical_cast<std::string>(total_path_quality/(double)successes));
                append_to_stat_file("\nFailure time(Grasps)  : "+boost::lexical_cast<std::string>(manip_planner->_grasp_failure_time/(double)(manip_planner->_grasp_failure_count)));
                append_to_stat_file("\nFailure time(Plans)   : "+boost::lexical_cast<std::string>(manip_planner->_mp_failure_time/(double)(manip_planner->_mp_failure_count)));
                append_to_stat_file("\nFailure time(Database): "+boost::lexical_cast<std::string>(total_failure_time/(double)(tries-successes)));
                append_to_stat_file("\n*************************************************************************");

                manip_planner->_grasp_success_count = 0;
                manip_planner->_grasp_success_time = 0;
                manip_planner->_mp_success_time = 0;
                manip_planner->_mp_success_count = 0;
                manip_planner->_grasp_failure_time = 0;
                manip_planner->_grasp_failure_count = 0;
                manip_planner->_mp_failure_time = 0;
                manip_planner->_mp_failure_count = 0;

                plan_t output_full_plan(manipulator->get_control_space());
                //output the plans
                int index=0;
                foreach(plan_t* plan, output_plans)
                {
                    output_full_plan.clear();
                    //convert the plans
                    manipulation_model->convert_plan(output_full_plan, manipulator->get_control_space(), *plan, manipulation_model->get_control_space());
                    delete plan;
                    char* w = std::getenv("PRACSYS_PATH");
                    std::string file_prefix(w);
                    file_prefix+="/prx_output/";
                    std::stringstream sstr;
                    sstr<<object->get_object_type()<<"_"<<hand<<"_"<<pose_index<<"_"<<index<<".plan";
                    file_prefix+=sstr.str();
                    output_full_plan.save_to_file(file_prefix);
                    index++;
                }

                //cleanup
                manipulation_model->get_state_space()->free_point(initial_state);
                manipulation_model->get_state_space()->free_point(result_state);
                object_space->free_point(initial_object);
                object_space->free_point(stored_object_state);
                return success;
            }

            void case_eval_task_planner_t::resolve_query()
            {

            }
        }
    }
}

