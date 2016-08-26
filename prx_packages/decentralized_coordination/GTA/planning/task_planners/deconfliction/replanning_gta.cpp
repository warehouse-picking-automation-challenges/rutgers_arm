/**
 * @file gta_planner.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Andrew Kimmel, Andrew Dobson, Athanasios Krontiris, Zakary Littlefield,  Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "planning/task_planners/deconfliction/replanning_gta.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/definitions/random.hpp"

#include "prx/utilities/distance_metrics/linear_metric/linear_distance_metric.hpp"
#include "prx/planning/modules/stopping_criteria/element/goal_criterion.hpp"
#include "prx/planning/modules/stopping_criteria/element/iteration_criterion.hpp"
#include "prx/planning/modules/stopping_criteria/element/timed_criterion.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/utilities/goals/radial_goal_region.hpp"
#include "prx/planning/problem_specifications/specification.hpp"
#include "planning/task_planners/deconfliction/replanning_gta_query.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <boost/range/adaptor/map.hpp> //adaptors

#include <numeric>
#include <set>

PLUGINLIB_EXPORT_CLASS( prx::packages::gta::replanning_gta_planner_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace plan;
    using namespace sim;
    using namespace plan::comm;
    using namespace packages::homotopies;
    
    namespace packages
    {
        namespace gta
        {

        /*
         *
         * 
         */
        replanning_gta_planner_t::replanning_gta_planner_t()
        {
            gta_query = NULL;
            my_query = NULL;
            selected_strategy = NULL;
            predicted_state = temp_state = NULL;
            goal_metric = new linear_distance_metric_t();
            color_map = {"white","red","orange","yellow","green","blue","black"};
            iteration_counter = 0;
            interaction_factor = 2.0;
            total_strategies = 0;
            greedy_ratio = 0;
            once = false;
        }

        /*
         * 
         * 
         */
        replanning_gta_planner_t::~replanning_gta_planner_t()
        {
            delete goal_metric;
            delete my_stopping_criteria;

            if (my_goal_criterion)
            {
                delete my_goal_criterion;
            }
            delete my_query;

            gta_query->state_space->free_point(temp_state);
            gta_query->state_space->free_point(predicted_state);
            gta_query->state_space->free_point(last_valid_state);
        }

        /*
         * 
         * 
         */
        void replanning_gta_planner_t::init(const parameter_reader_t* reader,const parameter_reader_t* template_reader)
        {
            task_planner_t::init(reader,template_reader);   

            visualize_actions = parameters::get_attribute_as<bool>("visualize_actions",reader,template_reader,false);
                        
            reconnection_radius = parameters::get_attribute_as<double>("reconnection_radius", reader, template_reader, 1.0);
            velocity = parameters::get_attribute_as<double>("velocity", reader, template_reader);
            disk_radius = parameters::get_attribute_as<double>("disk_radius", reader, template_reader);
            horizon_factor = parameters::get_attribute_as<double>("horizon_factor", reader, template_reader, 3);
            num_greedy_actions = parameters::get_attribute_as<unsigned>("num_greedy_actions", reader, template_reader);
            use_safety =  parameters::get_attribute_as<bool>("use_safety", reader, template_reader);
            consistent =  parameters::get_attribute_as<bool>("consistent", reader, template_reader);
            deterministic =  parameters::get_attribute_as<bool>("deterministic", reader, template_reader);
            waiting = parameters::get_attribute_as<bool>("waiting",reader,template_reader); 
            sensing_radius = parameters::get_attribute_as<double>("sensing_radius", reader, template_reader, 0.0);
            
            angle_threshold = parameters::get_attribute_as<double>("angle_threshold", reader, template_reader, .78539);
            interaction_factor = parameters::get_attribute_as<double>("interaction_factor",reader,template_reader,2.0);
            eta = parameters::get_attribute_as<double>("eta",reader,template_reader,0.042); 

            // Depending on what our type is, have different setups

            // Set my action generator

            h_graph_action_generator = dynamic_cast<h_graph_planner_t*>(planners["h_graph_planner"]);
            my_motion_planner = h_graph_action_generator;
            PRX_ASSERT(h_graph_action_generator != NULL);

            if (num_greedy_actions == 0 && !use_safety)
            {
                PRX_FATAL_S ("Must have greedy or safety");
            }
            
            if (consistent && !use_safety)
            {
                PRX_FATAL_S ("Cannot be consistent without minimum conflict");
            }
            
            if (deterministic && consistent)
            {
                PRX_FATAL_S ("Cannot be deterministic and consistent!");
            }
            PRX_DEBUG_S ("Creating stopping criteria");
            my_stopping_criteria = new stopping_criteria_t();
            my_goal_criterion = new iteration_criterion_t();
            dynamic_cast<iteration_criterion_t*>(my_goal_criterion)->set_max_iterations(10000);//->compute_statistics();
            my_stopping_criteria->add_criterion(my_goal_criterion);

            PRX_DEBUG_COLOR("Finished init", PRX_TEXT_BROWN);

        }

        /*
         * 
         * 
         */
        void replanning_gta_planner_t::setup()
        {
            if (gta_query == NULL)
            {
                PRX_FATAL_S("Hey! You either forgot to link me a valid gta query or your query has been corrupted!");
            }
            // Single shot
            if (my_query == NULL)
            {
                my_query = new motion_planning_query_t();
                temp_state = gta_query->state_space->alloc_point();
                my_query_start = gta_query->state_space->alloc_point();
                my_query->set_start(my_query_start);
                PRX_DEBUG_S ("Creating motion planning query");

                PRX_DEBUG_S ("My graph stopping criterion set");
                my_goal_criterion = new iteration_criterion_t();
                dynamic_cast<iteration_criterion_t*>(my_goal_criterion)->set_max_iterations(10000);//->compute_statistics();
                my_stopping_criteria->add_criterion(my_goal_criterion);
                //this->input_specification->link_spaces(gta_query->state_space, gta_query->control_space);
                
                this->input_specification->set_stopping_criterion(my_stopping_criteria);
                
                my_motion_planner->link_specification(input_specification);
            }
            // Memory is allocated once, action set is computed once
            if (!once)
            {
                resize_and_allocate();
                window = 2*(planning_duration/ simulation::simulation_step);
                delta = velocity*planning_duration;
                
                radial_goal_region_t* new_goal = new radial_goal_region_t();
                std::vector<double> goal_points;

                for (size_t j = 0; j < gta_query->state_space->get_dimension(); j++)
                {
                    goal_points.push_back( gta_query->get_goal()->get_goal_points().front()->at(j));
                }
                new_goal->set_goal(goal_metric, goal_points, reconnection_radius);
                new_goal->link_space(gta_query->state_space);
                my_query->set_goal(new_goal);
                my_query->link_spaces(gta_query->state_space, gta_query->control_space);
//                    my_query->set_start(gta_query->get_start_state());
                gta_query->state_space->copy_point(my_query->get_start_state(), gta_query->get_start_state());
                // Query motion planner
                my_motion_planner->link_query(my_query);
                my_motion_planner->reset();
                my_motion_planner->setup();
                if (my_motion_planner->can_deserialize())
                {
                    my_motion_planner->deserialize();
                }
                else
                {
                    PRX_FATAL_S ("Motion planner must be able to deserialize!");
                }
                PRX_DEBUG_S ("The start state: " << gta_query->state_space->print_point(my_query->get_start_state(),3));
  
                
                // Set up strategy vector
                if (consistent)
                {
                    total_strategies++;
                    strategy_vector.push_back(action_t());
                    strategy_vector.back().action = &consistent_strategy_path;
                    strategy_vector.back().action_plan = &consistent_strategy_plan;
                }
                if (num_greedy_actions > 0)
                {
                    total_strategies++;
                    strategy_vector.push_back(action_t());
                    strategy_vector.back().action = &greedy_strategy_path;
                    strategy_vector.back().action_plan = &greedy_strategy_plan;
                }

                if(use_safety)
                {
                    total_strategies++;
                    strategy_vector.push_back(action_t());
                    strategy_vector.back().action = &safety_strategy_path;
                    strategy_vector.back().action_plan = &safety_strategy_plan;
                }
            
                weight_vector.resize(total_strategies, 1);
                
                once = true;
                PRX_DEBUG_S ("Once is now true");
            }
            
            PRX_DEBUG_S ("SETUP DONE");

        }
        
        bool replanning_gta_planner_t::execute()
        {
//            PRX_WARN_S ("ITERATION NUMBER: " << straight_up_counter);
//            for (size_t my_action_index = 0; my_action_index < my_action_set.size(); my_action_index++)
//            {
//                PRX_ERROR_S ("PLAN NUMBER: " << my_action_index << "| LENGTH: " << my_action_set[my_action_index].action_plan->length() << "| Previous Cost: " << my_action_set[my_action_index].action_cost);
//                foreach (plan_step_t step, *my_action_set[my_action_index].action_plan)
//                {
//                    PRX_WARN_S ("Plan step: " << gta_query->control_space->print_point(step.control) << "\n with duration: " << step.duration);
//                }
//            }
            comp_timer.reset();
            iteration_counter++;
            return strategy_selection();
        }
        

        const statistics_t* replanning_gta_planner_t::get_statistics()
        {
            return planners[planner_names[0]]->get_statistics();
        }

        bool replanning_gta_planner_t::succeeded() const
        {
            return false;
        //    return planners[planner_names[0]]->succeeded();
        }

        void replanning_gta_planner_t::link_query(query_t* in_query)
        {
        //    //this will overwrite any query read from input
        //    if(output_queries.find(planner_names[0]) != output_queries.end())
        //        delete output_queries[planner_names[0]];
        //    output_queries[planner_names[0]] = in_query;
            replanning_gta_query_t* test_query = dynamic_cast<replanning_gta_query_t*>(in_query);
            if (test_query == NULL)
            {
                PRX_FATAL_S ("Nope! This query is not GTA approved");
            }
            else
            {
                PRX_INFO_S ("Good job - GTA query accepted!");
                gta_query = test_query;
            }


        }


        void replanning_gta_planner_t::resolve_query()
        {
            // Check if we solved things
            size_t index = planning_duration / simulation::simulation_step;

            // Set my plan to the selected stategy plan and path
            gta_query->plan.link_control_space(gta_query->control_space);
            gta_query->plan = *(selected_strategy->action_plan);
            gta_query->path = *(selected_strategy->action);
            
            
            // Augment my plan to have the correct duration
            gta_query->plan.augment_plan(this->planning_duration, false);
            // Find my end state

            if (index < gta_query->path.size() )
                gta_query->plan.copy_end_state (gta_query->path[index]);
            else
                gta_query->plan.copy_end_state (*gta_query->path.end());
//            PRX_PRINT ("selected plan: " << selected_strategy->action_plan->print(), PRX_TEXT_LIGHTGRAY);
            PRX_PRINT ("Length: " << selected_strategy->action->length(), PRX_TEXT_RED);
            PRX_PRINT ("Total planning time: " << comp_timer.measure(), PRX_TEXT_MAGENTA);
        }
        
        void replanning_gta_planner_t::set_planning_duration(double new_duration)
        {
            PRX_INFO_S ("Set planning duration: " << new_duration);

            planning_duration = new_duration;

        }
        
                
        
        
        void replanning_gta_planner_t::resize_and_allocate()
        {
            k_greedy_paths.resize(num_greedy_actions);
            k_greedy_plans.resize(num_greedy_actions);
            
            for (unsigned i = 0; i < num_greedy_actions; i++)
            {
                k_greedy_paths[i].link_space(gta_query->state_space);
                k_greedy_plans[i].link_state_space(gta_query->state_space);
                k_greedy_plans[i].link_control_space(gta_query->control_space);
            }
            
            safety_strategy_path.link_space(gta_query->state_space);
            safety_strategy_plan.link_state_space(gta_query->state_space);
            safety_strategy_plan.link_control_space(gta_query->control_space);
            
            consistent_strategy_path.link_space(gta_query->state_space);
            consistent_strategy_plan.link_state_space(gta_query->state_space);
            consistent_strategy_plan.link_control_space(gta_query->control_space);
            
            previous_selected_path.link_space(gta_query->state_space);
            previous_selected_plan.link_state_space(gta_query->state_space);
            previous_selected_plan.link_control_space(gta_query->control_space);
            previous_selected_action.action = &previous_selected_path;
            previous_selected_action.action_plan = &previous_selected_plan;
            
            predicted_state = gta_query->state_space->alloc_point();
            temp_state = gta_query->state_space->alloc_point();
            last_valid_state = gta_query->state_space->alloc_point();
//            my_action_index.resize(action_limit);
//            my_plan_index.resize(action_limit);
//            my_previous_plan_memory.resize(action_limit);
//            replaced_actions_memory.resize(action_limit);
//            replaced_plans_memory.resize(action_limit);
//            h_signatures.resize(action_limit);
//            previous_h_signatures.resize(action_limit);
//            index_list.resize(action_limit);
//            for(int i = 0; i < action_limit; i++)
//            {
//                my_previous_plan_memory[i].link_state_space(gta_query->state_space);
//                my_previous_plan_memory[i].link_control_space(gta_query->control_space);
//                replaced_actions_memory[i].link_space(gta_query->state_space);
//                replaced_plans_memory[i].link_state_space(gta_query->state_space);
//                replaced_plans_memory[i].link_control_space(gta_query->control_space);
//            }
//            dummy_plan.link_control_space(gta_query->control_space);
//            dummy_plan.link_state_space(gta_query->state_space);
//            wait_plan.link_control_space(gta_query->control_space);
//            wait_plan.link_state_space(gta_query->state_space);
//            wait_path.link_space(gta_query->state_space);
//
//            exploratory_actions_memory.resize(max_exploratory_actions);
//            exploratory_plans_memory.resize(max_exploratory_actions);
//            
//            for (int i = 0; i < max_exploratory_actions; i++)
//            {
//                exploratory_actions_memory[i].link_space(gta_query->state_space);
//                exploratory_plans_memory[i].link_state_space(gta_query->state_space);
//                exploratory_plans_memory[i].link_control_space(gta_query->control_space);
//
//            }
//            
//            
//            
//            greedy_plans_memory.resize(k_greedy_actions);
//            greedy_trajectory_memory.resize(k_greedy_actions);
//            for (unsigned i = 0; i < k_greedy_actions; i++)
//            {
//                greedy_plans_memory[i].link_state_space(gta_query->state_space);
//                greedy_plans_memory[i].link_control_space(gta_query->control_space);
//                greedy_trajectory_memory[i].link_space(gta_query->state_space);
//            }
//            
//            safety_plans_memory.resize(num_safety_actions);
//            safety_trajectory_memory.resize(num_safety_actions);
//            for (unsigned i = 0; i < num_safety_actions; i++)
//            {
//                safety_plans_memory[i].link_state_space(gta_query->state_space);
//                safety_plans_memory[i].link_control_space(gta_query->control_space);
//                safety_trajectory_memory[i].link_space(gta_query->state_space);
//            }
        }
        
        
        void replanning_gta_planner_t::update_vis_info() const
        {
        //    PRX_ERROR_S ("Visualizing actions!");
            std::vector<geometry_info_t> geoms;
            std::vector<config_t> configs;    
            hash_t<std::string, std::vector<double> > map_params;
            std::vector<double> params;
            int count;

            std::string my_actions_name = ros::this_node::getName() + "gta_actions/";
            std::string visualization_body = "simulator/disk/body";
            std::string plant_path = "world_model/simulator/disk";
            if(visualize_actions)
            {

                count = 0;        
                std::vector<std::string> system_names;
                system_names.push_back(visualization_body);
                int random_height = 10;

                if (sensing_radius > 0)
                {
                    std::string name = my_actions_name + "/sensing_radius";
                    params.clear();  
 
                    map_params.clear();
                    params.push_back(sensing_radius);
                    params.push_back(1.5);


                    geoms.push_back(geometry_info_t(visualization_body, name, PRX_CYLINDER, params, "green"));
                    configs.push_back(config_t(vector_t(gta_query->my_current_state->at(0),gta_query->my_current_state->at(1), 0), quaternion_t(0,0,0,1)));  

                }
                std::string name = my_actions_name + int_to_str(count) + "/edge_" + int_to_str(count);
                params.clear();  
                foreach(state_t* state, greedy_strategy_path)
                {
                    map_params.clear();
                    ((visualization_comm_t*)vis_comm)->compute_configs(state,system_names,map_params);       
                    params.insert(params.end(),map_params[visualization_body].begin(),map_params[visualization_body].end());  
                    params.back() += random_height;//this is so the solution will be above 

                }
                if (params.size() > 0)
                {

                    geoms.push_back(geometry_info_t(visualization_body, name, PRX_LINESTRIP, params, "red"));
                    configs.push_back(config_t());  


                    count++;
                }
                
                unsigned greedy_counter = 0;
                foreach(trajectory_t traj, k_greedy_paths)
                {
                    if (selected_greedy_index != greedy_counter && k_greedy_paths.size() > 1)
                    {
                
                        name = my_actions_name + int_to_str(count) + "/edge_" + int_to_str(count);
                        params.clear();  

                        foreach(state_t* state, traj)
                        {
                            map_params.clear();
                            ((visualization_comm_t*)vis_comm)->compute_configs(state,system_names,map_params);       
                            params.insert(params.end(),map_params[visualization_body].begin(),map_params[visualization_body].end());  
                            params.back() += random_height;//this is so the solution will be above 

                        }
                        if (params.size() > 0)
                        {

                            geoms.push_back(geometry_info_t(visualization_body, name, PRX_LINESTRIP, params, "black"));
                            configs.push_back(config_t());  


                            count++;
                        }
                    }
                    greedy_counter++;
                }

                name = my_actions_name + int_to_str(count) + "/edge_" + int_to_str(count);
                params.clear();  

                foreach(state_t* state, safety_strategy_path)
                {
                    map_params.clear();
                    ((visualization_comm_t*)vis_comm)->compute_configs(state,system_names,map_params);       
                    params.insert(params.end(),map_params[visualization_body].begin(),map_params[visualization_body].end());  
                    params.back() += random_height;//this is so the solution will be above 

                }
                if (params.size() > 0)
                {

                    geoms.push_back(geometry_info_t(visualization_body, name, PRX_LINESTRIP, params, "green"));
                    configs.push_back(config_t());  


                    count++;
                }
                

                name = my_actions_name + int_to_str(count) + "/edge_" + int_to_str(count);
                params.clear();  

                foreach(state_t* state, consistent_strategy_path)
                {
                    map_params.clear();
                    ((visualization_comm_t*)vis_comm)->compute_configs(state,system_names,map_params);       
                    params.insert(params.end(),map_params[visualization_body].begin(),map_params[visualization_body].end());  
                    params.back() += random_height;//this is so the solution will be above 

                }
                if (params.size() > 0)
                {

                    geoms.push_back(geometry_info_t(visualization_body, name, PRX_LINESTRIP, params, "blue"));
                    configs.push_back(config_t());  


                    count++;
                }
                
                name = my_actions_name + int_to_str(count) + "/edge_" + int_to_str(count);
                params.clear();  
                PRX_PRINT ("VIS INFO:::: Length: " << selected_strategy->action->length(), PRX_TEXT_GREEN );
                foreach(state_t* state, *(selected_strategy->action))
                {
                    map_params.clear();
                    ((visualization_comm_t*)vis_comm)->compute_configs(state,system_names,map_params);       
                    params.insert(params.end(),map_params[visualization_body].begin(),map_params[visualization_body].end());  
                    params.back() += 2*random_height;//this is so the solution will be above 

                }
                if (params.size() > 0)
                {

                    geoms.push_back(geometry_info_t(visualization_body, name, PRX_LINESTRIP, params, "off_white"));
                    configs.push_back(config_t());  


                    count++;
                }
                

                ((visualization_comm_t*)vis_comm)->visualization_geom_map[my_actions_name] = geoms;
                ((visualization_comm_t*)vis_comm)->visualization_configs_map[my_actions_name] = configs;   

                geoms.clear();

            }
        }
 
        bool replanning_gta_planner_t::strategy_selection()
        {
            bool pre_condition_set = false;
            
            if(!compute_predicted_state())
            {
                reconnect_action(selected_strategy, last_valid_state);
                return true;
//                return false;
            }

            // Check waiting pre
            if (waiting && iteration_counter > 1)
            {
                pre_condition_set = pre_waiting_check();
            }
            
            // Build neighbor set
            
            std::vector<state_t*> current_n_states;
            gta_query->distance_limited_neighborhood(current_n_states, sensing_radius);

            
            // Generate strategies
            unsigned strategy_start_index = 0;
            if (consistent)
            {
                generate_consistent(strategy_start_index, current_n_states);
                strategy_start_index++;
            }
            if (num_greedy_actions > 0)
            {
                generate_greedy(strategy_start_index, current_n_states);
                strategy_start_index ++;
            }
            
            if (use_safety)
            {
                generate_safe(strategy_start_index, current_n_states);
                strategy_start_index++;
            }
            

            // Select strategy
            if (num_greedy_actions == 0)
            {
                if(!consistent || iteration_counter > 1)
                {
                    selected_strategy = &strategy_vector[0];
                }
                else
                {
                    selected_strategy = &strategy_vector[1];
                }

                // Check if consistent strategy is in conflict
                if (consistent && compute_interaction_ratio(strategy_vector[0].action, current_n_states) > PRX_ZERO_CHECK)
                {
                    // Use min. conflict
                    selected_strategy = &strategy_vector[1];
                }
                
            }
            else if (!use_safety)
            {
                /** K-Best **/
                if (deterministic)
                {
                    selected_strategy = &strategy_vector[0];
                }
                /** K-Rand **/
                else
                {
                    std::vector<unsigned> valid_strategy_indices;
                    for(unsigned i = 0; i < strategy_vector.size(); i++)
                    {
                        if (strategy_vector[i].action_cost == strategy_vector[i].original_cost)
                        {
                            valid_strategy_indices.push_back(i);
                        }
                    }
                    
                    int random_greedy_strategy;
                    if (valid_strategy_indices.empty())
                        random_greedy_strategy = util::uniform_int_random(0, strategy_vector.size()-1);
                    else
                        random_greedy_strategy = valid_strategy_indices[util::uniform_int_random(0, valid_strategy_indices.size()-1)];
                    selected_strategy = &strategy_vector[random_greedy_strategy];
                }
            }
            else if (deterministic)
            {
                if (strategy_vector[0].action_cost < strategy_vector[1].action_cost)
                {
                    selected_strategy = &strategy_vector[0];
                    greedy_ratio++;
                }
                else
                {
                    selected_strategy = &strategy_vector[1];
                }
            }
            else
            {
                selected_strategy = polynomial_weights();
            }
            
            // Check post waiting
            if (waiting && pre_condition_set && iteration_counter > 1)
            {
                if (selected_strategy->action->size() > 0)
                    post_waiting_check();
                else
                {
                    PRX_PRINT ("Warning: Action size < 0!", PRX_TEXT_MAGENTA);
                }
            }

            PRX_PRINT ("Total time: " << comp_timer.measure(), PRX_TEXT_GREEN);
            PRX_PRINT ("Length: " << selected_strategy->action->length(), PRX_TEXT_RED);
            double selection_ratio = ((double)((double)greedy_ratio/(double)iteration_counter));
            PRX_PRINT ("Greedy ratio: " << selection_ratio, PRX_TEXT_MAGENTA);
            PRX_PRINT ("Safety ratio: " << (1.0 - selection_ratio), PRX_TEXT_MAGENTA);
            return true;

        }

        bool replanning_gta_planner_t::compute_predicted_state()
        {
            gta_query->state_space->copy_point(temp_state, gta_query->my_current_state);
            model->propagate_plan(temp_state,gta_query->plan, predicted_state);
            PRX_DEBUG_COLOR ("RECONNECT: WENT FROM: " << gta_query->state_space->print_point(temp_state) << " TO: " << gta_query->state_space->print_point(predicted_state), PRX_TEXT_RED);
            if( !input_specification->validity_checker->is_valid(predicted_state) )
            {
                PRX_WARN_S ("NOT VALID POINT!");
                gta_query->state_space->copy_point(predicted_state, gta_query->my_current_state);
                if( !input_specification->validity_checker->is_valid(gta_query->my_current_state) )
                {
                    PRX_ERROR_S ("CURRENT STATE NOT VALID POINT!");
                    return false;
                }
            }
            
            gta_query->state_space->copy_point(last_valid_state, gta_query->my_current_state);
            
            return true;
        }
        
        bool replanning_gta_planner_t::pre_waiting_check()
        {
            // Check previous action
            if (selected_strategy->action->size() == 0)
                return false;
            
            previous_selected_path = *(selected_strategy->action);
            previous_selected_plan = *(selected_strategy->action_plan);
            
            // Compute direction vector from it

            
            // Reconnect to action
            reconnect_action(&previous_selected_action, predicted_state);
            
            // Set start and end point
            std::vector<double> start, end; 
            for (unsigned i = 0; i < gta_query->state_space->get_dimension(); i++)
            {
                start.push_back(previous_selected_action.action->at(0)->at(i));
            }
            
            a.set(start);
            
            PRX_DEBUG_COLOR ("Prev start point: " << a, PRX_TEXT_BLUE);
            
            int actual_window = window;
            if (actual_window > previous_selected_action.action->size())
                actual_window = previous_selected_action.action->size() - 1;
            
//            PRX_PRINT ("Window is: " << actual_window, PRX_TEXT_BROWN);
//            PRX_PRINT ("TRAJECTORY: " << prev_action->action->print(), PRX_TEXT_LIGHTGRAY);
            for (unsigned i = 0; i < gta_query->state_space->get_dimension(); i++)
            {
                end.push_back(previous_selected_action.action->at(actual_window)->at(i));
            }
            
            b.set(end);
            
            PRX_DEBUG_COLOR ("Prev end point: " << b, PRX_TEXT_BLUE);
            
            prev_selected_vector = b - a;
            
            PRX_DEBUG_COLOR ("Prev vector: " << prev_selected_vector, PRX_TEXT_RED);
            
            return true;
            
        }        
        
        bool replanning_gta_planner_t::post_waiting_check()
        {
            // Get Current selected action
            if (selected_strategy->action->size() == 0)
                return false;
            
            // Compute direction vector from it

            // Set start and end point
            std::vector<double> start, end; 
            for (unsigned i = 0; i < gta_query->state_space->get_dimension(); i++)
            {
                start.push_back(selected_strategy->action->at(0)->at(i));
            }
            
            a.set(start);
            PRX_DEBUG_COLOR ("Current start point: " << a, PRX_TEXT_BLUE);
            
            int actual_window = window;
            if (actual_window > selected_strategy->action->size())
                actual_window = selected_strategy->action->size() - 1;
            for (unsigned i = 0; i < gta_query->state_space->get_dimension(); i++)
            {
                end.push_back(selected_strategy->action->at(actual_window)->at(i));
            }
            
            b.set(end);
            PRX_DEBUG_COLOR ("Current end point: " << b, PRX_TEXT_BLUE);
            current_selected_vector = b - a;
            PRX_DEBUG_COLOR ("Current vector: " << current_selected_vector, PRX_TEXT_RED);
            // Get the angle between the two
            
            double angle = current_selected_vector.get_angle_between(prev_selected_vector);
            PRX_DEBUG_COLOR( "Angle between: " << angle, PRX_TEXT_LIGHTGRAY);
            if (angle > angle_threshold)
            {
                PRX_DEBUG_COLOR ("WILL WAIT!", PRX_TEXT_MAGENTA);

                std::vector<double> vals;
                vals.push_back(predicted_state->at(0));
                vals.push_back(predicted_state->at(1));
                gta_query->control_space->set_from_vector(vals, previous_selected_action.action_plan->at(0).control);
                previous_selected_action.action_plan->at(0).duration = planning_duration;
                previous_selected_action.action->chop(1);
                
                selected_strategy = &previous_selected_action;
            }
        }   
        
                
        void replanning_gta_planner_t::generate_consistent(unsigned start_action_index , const std::vector<state_t*>& neighbor_states)
        {
            PRX_DEBUG_COLOR ("Generate consistent", PRX_TEXT_BROWN);
            if (selected_strategy)
            {
                consistent_strategy_plan = *(selected_strategy->action_plan);
                consistent_strategy_path = *(selected_strategy->action);

                // Reconnect to action

                if (consistent_strategy_path.size() > 0)
                {
    //                PRX_PRINT("ACTION SIZE NOT ZERO!" , PRX_TEXT_BLUE);
    //                PRX_PRINT ("ACTION PRINTU: " << current_action->action->print(), PRX_TEXT_GREEN);
                    reconnect_action(&strategy_vector[start_action_index], predicted_state);
                    strategy_vector[start_action_index].original_cost = strategy_vector[start_action_index].action->length();
                    strategy_vector[start_action_index].action_cost = strategy_vector[start_action_index].original_cost 
                            * (1 + interaction_factor*compute_interaction_ratio(&consistent_strategy_path, neighbor_states));
                }
                else
                {
                    PRX_FATAL_S ("Consistent strategy path size of 0");
                }
            }

        }
        
                
        void replanning_gta_planner_t::generate_greedy(unsigned start_action_index, const std::vector<sim::state_t*>& neighbor_states)
        {
    
            radial_goal_region_t* new_goal = new radial_goal_region_t();
            std::vector<double> goal_points;

            for (size_t j = 0; j < gta_query->state_space->get_dimension(); j++)
            {
                goal_points.push_back( gta_query->get_goal()->get_goal_points().front()->at(j));
//                PRX_ERROR_S ("Pushed onto goal: " << goal_points.back());
            }
            new_goal->set_goal(goal_metric, goal_points, reconnection_radius);
            new_goal->link_space(gta_query->state_space);
            my_query->set_goal(new_goal);
            gta_query->state_space->copy_point(my_query->get_start_state(), predicted_state);
            double goal_dist = gta_query->state_space->distance(gta_query->my_current_state, gta_query->get_goal()->get_goal_points().front());
           
            std::vector<state_t*> current_n_states;
            if (num_greedy_actions > 1 && goal_dist > 2.1*disk_radius)
            {
                PRX_DEBUG_COLOR("Find k actions", PRX_TEXT_RED);
                h_graph_action_generator->magical_resolve_query( num_greedy_actions,  current_n_states, disk_radius*(1.05), k_greedy_plans, k_greedy_paths);
                // Compute the best of the k-greedy actions -> num_greedy_actions
                double best_cost = PRX_INFINITY, new_cost, original_cost;
                unsigned best_index = -1;

                // Compute scaled costs
                for (unsigned i=0; i < num_greedy_actions; i++)
                {
                    original_cost = k_greedy_paths[i].length();
                    new_cost = original_cost * (1 + interaction_factor*compute_interaction_ratio(&k_greedy_paths[i], neighbor_states));
                    PRX_DEBUG_COLOR ("Original: " << original_cost << " , New cost: " << new_cost, PRX_TEXT_BROWN);
                    if (new_cost < best_cost)
                    {
                        best_cost = new_cost;
                        best_index = i;
                    }
                }

                greedy_strategy_path = k_greedy_paths[best_index];
                greedy_strategy_plan = k_greedy_plans[best_index];
                selected_greedy_index = best_index;
                PRX_DEBUG_COLOR ("Best index is: " << best_index, PRX_TEXT_CYAN);
                strategy_vector[start_action_index].original_cost = greedy_strategy_path.length();
                strategy_vector[start_action_index].action_cost = best_cost;
            }
            else
            {
                PRX_DEBUG_COLOR("ONLY FIND 1 action", PRX_TEXT_RED);
                h_graph_action_generator->invalidate_and_resolve_query_mk2(current_n_states, disk_radius*(1.05), horizon_factor);
                if (my_query->path.size() > 0)
                {
                    k_greedy_plans[0] = my_query->plan;
                    k_greedy_paths[0] = my_query->path;
                    greedy_strategy_path = k_greedy_paths[0];
                    greedy_strategy_plan = k_greedy_plans[0];
                    strategy_vector[start_action_index].original_cost = greedy_strategy_path.length();
                    strategy_vector[start_action_index].action_cost = strategy_vector[start_action_index].original_cost *(1 + interaction_factor*compute_interaction_ratio(&greedy_strategy_path, neighbor_states));
                }
                else
                {
                    reconnect_action (&strategy_vector[start_action_index], predicted_state);
                    strategy_vector[start_action_index].original_cost = strategy_vector[start_action_index].action->length();
                    strategy_vector[start_action_index].action_cost = strategy_vector[start_action_index].original_cost * (1 + interaction_factor*compute_interaction_ratio(strategy_vector[start_action_index].action, neighbor_states));
                }
            }
            

            
        }
        void replanning_gta_planner_t::generate_safe(unsigned start_action_index, const std::vector<state_t*>& neighbor_states)
        {

            radial_goal_region_t* new_goal = new radial_goal_region_t();
            std::vector<double> goal_points;

            for (size_t j = 0; j < gta_query->state_space->get_dimension(); j++)
            {
                goal_points.push_back( gta_query->get_goal()->get_goal_points().front()->at(j));
//                PRX_ERROR_S ("Pushed onto goal: " << goal_points.back());
            }
            new_goal->set_goal(goal_metric, goal_points, reconnection_radius);
            new_goal->link_space(gta_query->state_space);
            my_query->set_goal(new_goal);
            gta_query->state_space->copy_point(my_query->get_start_state(), predicted_state);
            h_graph_action_generator->invalidate_and_resolve_query_mk2(neighbor_states, disk_radius*(1.05), horizon_factor);
            
            safety_strategy_plan = my_query->plan;
            safety_strategy_path = my_query->path;
            
            double original_cost = safety_strategy_path.length();
            strategy_vector[start_action_index].original_cost = original_cost;
            strategy_vector[start_action_index].action_cost = original_cost * (1 + interaction_factor*compute_interaction_ratio(&safety_strategy_path, neighbor_states));
            PRX_DEBUG_COLOR ("Original: " << strategy_vector[start_action_index].original_cost << " , New cost: " << strategy_vector[start_action_index].action_cost, PRX_TEXT_CYAN);
        }
        
        action_t* replanning_gta_planner_t::polynomial_weights()
        {
            PRX_DEBUG_COLOR ("Running PWB", PRX_TEXT_LIGHTGRAY);
            if (iteration_counter < 2)
            {
                return &strategy_vector[1];
            }
            PRX_DEBUG_COLOR ("C_i computation time", PRX_TEXT_CYAN);
            // Compute C(i,t) for each action

            double max_loss, min_loss;
            min_loss = max_loss = std::floor((strategy_vector[0].action_cost)+0.5);// - my_action_set[0].previous_action_cost);
            for (unsigned i = 0; i < total_strategies; i++)
            {

                double d_i = std::floor(strategy_vector[i].action_cost + 0.5);// - my_action_set[i].previous_action_cost + 0.5;
                if (d_i > max_loss)
                    max_loss = d_i;
                if (d_i < min_loss)
                    min_loss = d_i;
                strategy_vector[i].l_i  = d_i;
                PRX_DEBUG_COLOR ("D_i (loss): " << d_i, PRX_TEXT_MAGENTA);
            }
            
            if( std::fabs(min_loss - max_loss) < 1.1 )
            {
                min_loss = max_loss;
            }
            
            PRX_DEBUG_COLOR ("Max loss: " << max_loss << ", min loss: " << min_loss << " at step : " << iteration_counter, PRX_TEXT_CYAN);
            
            double denom = max_loss - min_loss;
            if (denom <= PRX_ZERO_CHECK )
            {
                for (unsigned i=0; i < total_strategies; i++)
                {
                    strategy_vector[i].l_i = 0;
                }
            }
            else
            {
                // Scale and store the loss
                for (unsigned i=0; i < total_strategies; i++)
                {
//                    PRX_PRINT ("Pre scale l_i: " << my_action_set[i].l_i, PRX_TEXT_CYAN);
                    strategy_vector[i].l_i = ((strategy_vector[i].l_i - min_loss)/(denom));
                    PRX_DEBUG_COLOR ("After scale l_i: " << strategy_vector[i].l_i, PRX_TEXT_CYAN);

                    strategy_vector[i].w_i *= (1-eta*strategy_vector[i].l_i);
                    weight_vector[i] = strategy_vector[i].w_i;
                }
            }

            // Apply PW
            // Probabilistically select action

            double sum = 0;
            for( unsigned i = 0; i < weight_vector.size(); i++ )
            {
                sum += weight_vector[i];
            }
            for( unsigned i = 0; i < weight_vector.size(); i++ )
            {
                PRX_DEBUG_COLOR ("Probability of " << i << " is: " << weight_vector[i]/sum, PRX_TEXT_MAGENTA);
            }
            
            unsigned minmax_index;
            minmax_index = roll_weighted_die(weight_vector);

            PRX_DEBUG_COLOR ("Selected action: " << minmax_index, PRX_TEXT_RED);
            if (minmax_index == 0)
                greedy_ratio++;

            return &strategy_vector[minmax_index];
            
        }
        
        
        /*
         * 
         * 
         */
        bool replanning_gta_planner_t::reconnect_action(action_t* to_connect_action, sim::state_t* connect_state)
        {
            /** We must take the ground truth, x(t), simulate the plan
             *  that was just passed, to obtain a predicted state x'(t+1).
             *  Then the objective of this function is to reconnect the actions
             *  to x'(t+1) */
//                prm_action_generator->shortcut = false;
            sys_clock_t time1, time2;
            unsigned int actual_window = window;

            time1.reset();
            my_query->plan.clear();
    //        my_query->plan.state_space = gta_query->state_space;
    //        my_query->plan.input_control_space = gta_query->control_space;

            my_query->path.link_space(gta_query->state_space);
            my_query->path.clear();
    //        PRX_INFO_S ("Reconnecting using action: " << my_action_index);
            // Start at the last state, try to reconnect
            trajectory_t* traj = to_connect_action->action;
            if (actual_window >= traj->size())
            {

                actual_window = traj->size() - 1;
                    PRX_DEBUG_S ("Traj size: " << traj->size() << "... So, Going from window: " << window << " to: " << actual_window);
            }
            radial_goal_region_t* new_goal = new radial_goal_region_t();
            std::vector<double> goal_points;

            for (size_t j = 0; j < gta_query->state_space->get_dimension(); j++)
            {
                goal_points.push_back(traj->at(actual_window)->at(j));
    //            PRX_INFO_S ("Pushed onto goal: " << goal_points.back());
            }
            PRX_DEBUG_COLOR ("Window : " << actual_window << "\nAttempting to reconnect from: " << gta_query->state_space->print_point(connect_state) << "\n to state: " << gta_query->state_space->print_point((*traj)[actual_window]), PRX_TEXT_RED);

            gta_query->state_space->copy_point(my_query->get_start_state(), connect_state);
//                    my_query->set_start(temp_state);
            new_goal->set_goal(goal_metric, goal_points, reconnection_radius);
            new_goal->link_space(gta_query->state_space);
            my_query->set_goal(new_goal);


            h_graph_action_generator->approximate_resolve_query();


            // Append trajectory

//                    PRX_WARN_S ("Time to resolve query: " << time1.measure());
            time2.reset();
            // Then we added the leftover states

            // We append the remaining trajectory states onto the resolved query path
            for (unsigned prev_index = actual_window + 1; prev_index < traj->size(); prev_index++)
            {
                my_query->path.copy_onto_back((*traj)[prev_index]);
    //            PRX_PRINT ("PUSH Traj state: " << gta_query->state_space->print_point((*traj)[prev_index]), PRX_TEXT_BLUE);
            }

            bool cut_plan = true;

            double reconnection_time = actual_window * simulation::simulation_step;
//                if (my_action_index == prev_minmax)
                reconnection_time *= 2;

            double budget = reconnection_time;
            int cutoff_point = -1;
            double current_duration = 0.0;
    //                PRX_WARN_S ("Reconnection time: " << reconnection_time << " and reconnection index: " << actual_window );
    //                PRX_DEBUG_S ("Is this the actual state? : " << gta_query->state_space->print_point((*to_connect_action->action)[actual_window]));
    //                PRX_INFO_S ("Starting budget: " << budget);
            while (budget > 0 )
            {
                cutoff_point++;
                if (cutoff_point >= to_connect_action->action_plan->size())
                {
                    cut_plan = false;
                    cutoff_point--;
//                        PRX_PRINT("Reached the end of plan with budget: " << budget, PRX_TEXT_BLUE);
                    break;
                }
                budget -= to_connect_action->action_plan->at(cutoff_point).duration;
                current_duration += to_connect_action->action_plan->at(cutoff_point).duration;
    //                    PRX_DEBUG_S ("Budget time: " << budget);

            }
//                if (!cut_plan)
//                {
////                            PRX_LOG_ERROR("Oh my goodness!");
//                    PRX_ERROR_S ("OH my goodness?!");
//                    my_motion_planner->resolve_query();
//                    *traj = my_query->path;
//                    *(to_connect_action->action_plan) = my_query->plan;
//                }
//                else
            {
                current_duration -= to_connect_action->action_plan->at(cutoff_point).duration;

                for (int counter = 0; counter < cutoff_point; counter++)
                {
                    to_connect_action->action_plan->pop_front();
                }
                // Append planning steps until we reach the cutoff point
                to_connect_action->action_plan->at(0).duration = std::fabs(budget);//reconnection_time - current_duration;

                if (cutoff_point != 0)
                {
                    for (int step_index =  my_query->plan.size() -1; step_index >= 0; step_index--)
                    {
                        to_connect_action->action_plan->copy_onto_front(my_query->plan[step_index].control,my_query->plan[step_index].duration );
                    }
                }

                //            // Finally we update our action
                *traj = my_query->path;

            }
        }


   
        double replanning_gta_planner_t::compute_interaction_ratio(sim::trajectory_t* executed_path, const std::vector<sim::state_t*>& neighbor_states)
        {
//            PRX_PRINT ("New action", PRX_TEXT_RED);
            // The final interaction ratio
            double interaction_ratio = 0.0;
//            PRX_PRINT ("GTA QUERY GOAL: " << gta_query->state_space->print_point(gta_query->get_goal()->get_goal_points().front()), PRX_TEXT_BLUE);
            
            // Determines the discretization of the trajectory
            int start_check = window;//2*(std::ceil(planning_duration/simulation::simulation_step));
//            PRX_PRINT ("Start check: " << start_check, PRX_TEXT_GREEN );
            int resolution = 1;
            if (executed_path->size() >= start_check)
            {
//                PRX_PRINT ("Start check state: " << gta_query->state_space->print_point(executed_path->at(start_check)), PRX_TEXT_BROWN);
                foreach( state_t* neighbor, neighbor_states)
                {
                    if (gta_query->state_space->distance(neighbor, gta_query->get_goal()->get_goal_points().front()) >= 2 *(disk_radius*1.05))
                    {
//                        PRX_PRINT ("Comparing neighbor: " << gta_query->state_space->print_point(neighbor), PRX_TEXT_LIGHTGRAY);
                        // Find the closest state in the trajectory to the neighbor state
                        double min_distance = PRX_INFINITY;
                        int min_index = 0;
                        for (unsigned i = start_check; i < executed_path->size(); i+=resolution)
                        {
        //                    PRX_PRINT (" traj: " << gta_query->state_space->print_point(executed_path->at(i)), PRX_TEXT_BLUE);
                            double dist = gta_query->state_space->distance(executed_path->at(i), neighbor);
                            if ( dist < min_distance )
                            {
                                min_distance = dist;
                                min_index = i;
                            }
                        }
//                        PRX_PRINT ("Min distance: " << min_distance << " with min_index: " << min_index, PRX_TEXT_CYAN);

                        // Find if the closest two states intersect
                        if (min_distance <= 2*disk_radius)
                        {
                            double ratio = (double)(executed_path->size() - min_index) / (double)executed_path->size();
//                            PRX_PRINT ("COLLISION AT MY STATE: " << gta_query->state_space->print_point(executed_path->at(min_index)) << " vs neighbor: " << gta_query->state_space->print_point(neighbor), PRX_TEXT_MAGENTA);
//                            PRX_PRINT ("INTERACT WOOT WOOT ---- : " << ratio, PRX_TEXT_RED);
                            interaction_ratio += ratio;
                        }
                    }
                }
            }
//            else
//            {
//                PRX_ERROR_S ("Resolution: " << start_check << " vs traj size: " << executed_path->size());
//            }
//            
//            PRX_PRINT ("Interaction ratio: " << interaction_ratio, PRX_TEXT_GREEN);
            return interaction_ratio;
        }
        
    }
    }

}


