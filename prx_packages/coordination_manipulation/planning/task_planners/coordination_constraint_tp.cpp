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


#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "planning/task_planners/coordination_constraint_tp.hpp"

#include <algorithm> 
#include <fstream>
#include <boost/range/adaptor/map.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::coordination_manipulation::coordination_constraint_tp_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace coordination_manipulation
        {
            coordination_constraint_tp_t::coordination_constraint_tp_t()
            {
                naive = incremental = batch = false;
                constraint_aware_astar = NULL;
                precomputed_constraints = NULL;
                multi_constraint_astar = NULL;
                
                /** Initialize variables pertaining to running multiple experiments */
                number_of_experiments = current_experiment = 0;
                min_solution_time = min_computation_time = 100000.0;
                max_solution_time = max_computation_time = 0.0;
            }

            coordination_constraint_tp_t::~coordination_constraint_tp_t()
            {
                PRX_PRINT ("\n\n Coordination constraint tp destructor \n\n", PRX_TEXT_RED);
                if (use_preprocessed_constraints)
                {
                    delete precomputed_constraints;
                    delete constraint_aware_astar;
                }
            }

            void coordination_constraint_tp_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
            {
                naive = parameters::get_attribute_as<bool>("naive", reader, template_reader, false);
                incremental = parameters::get_attribute_as<bool>("incremental", reader, template_reader, false);
                batch = parameters::get_attribute_as<bool>("batch", reader, template_reader, false);
                number_of_experiments = parameters::get_attribute_as<unsigned>("number_of_experiments", reader, template_reader, 0);
                total_new_experiments = parameters::get_attribute_as<unsigned>("total_new_experiments", reader, template_reader, 1);
                
                        
                if (naive || incremental || batch)
                {
                    use_preprocessed_constraints = true;
                    if (naive)
                    {
                        //scheduling_type = RANDOM_SCHEDULE;
                        algorithm = "[NaiveAlg]";
                    }
                    else if (batch)
                    {
                        //scheduling_type = RANDOM_SCHEDULE;
                        algorithm = "[BatchAlg]";
                    }
                    else
                    {
                        scheduling_type = INCREMENTAL_SCHEDULE;
                        algorithm = "[IncrementalAlg]";
                    }
                }
                
                //I don't think there are any additional parameters to load here.
                task_planner_t::init(reader, template_reader);
                
                /** Make sure we know our contexts */
                full_manipulator_context_name = parameters::get_attribute("full_manipulator_context_name", reader, template_reader);
                left_context_name = parameters::get_attribute("left_context_name", reader, template_reader);
                right_context_name = parameters::get_attribute("right_context_name", reader, template_reader);
                
                /** Read number of objects per arm*/
                object_types = parameters::get_attribute_as<std::vector< std::string > >("object_types", reader, template_reader);
                if (parameters::has_attribute("num_plans_per_object_type", reader, template_reader))
                {
                    num_plans_per_object_type = parameters::get_attribute_as<std::vector<int> >("num_plans_per_object_type", reader, template_reader);
                }
                
                /** Get type of scheduling and coordination */
                std::string schedule_alg = parameters::get_attribute_as<std::string>("scheduling_type", reader, template_reader, "MANUAL");
                if (schedule_alg == "RANDOM")
                {
                    scheduling_type = RANDOM_SCHEDULE;
                    algorithm += "[RandomSchedule]";
                }
                else if(schedule_alg =="INCREMENTAL")
                {
                    scheduling_type = INCREMENTAL_SCHEDULE;
                    algorithm += "[IncrementalSchedule]";
                }
                else if(schedule_alg =="MINCONF")
                {
                    scheduling_type = MINCONF_SCHEDULE;
                    algorithm += "[MinconfSchedule]";
                }
                else
                {
                    scheduling_type = MANUAL_SCHEDULE;
                    algorithm += "[ManualSchedule]";
                }
                
                
                std::string coordination_alg = parameters::get_attribute_as<std::string>("coordination_type", reader, template_reader, "NONE");
                if (coordination_alg == "PRM")
                {
                    coordination_type = PRM_COORDINATION;
                    //algorithm += "[PRM_COORDINATION]";
                }
                else if (coordination_alg == "PRM_MAX")
                {
                    coordination_type = PRM_MAX_VEL_COORDINATION;
                    //algorithm += "[PRM_MAX_VEL_COORDINATION]";
                }
                else if (coordination_alg == "GRID")
                {
                    coordination_type = GRID_COORDINATION;
                    //algorithm += "[GRID_COORDINATION]";
                }
                else if (coordination_alg == "GRID_MAX")
                {
                    coordination_type = GRID_MAX_VEL_COORDINATION;
                    //algorithm += "[GRID_MAX_VEL_COORDINATION]";
                }
                else
                {
                    coordination_type = NONE_COORDINATION;
                    //algorithm += "[NONE_COORDINATION]";
                }
                
                
                std::string selection_alg = parameters::get_attribute_as<std::string>("selection_type", reader, template_reader, "NONE");
                if (selection_alg == "LONGEST")
                {
                    selection_type = LONGEST_SELECTION;
                    algorithm += "[LongestSelection]";
                }
                else if (selection_alg == "SHORTEST")
                {
                    selection_type = SHORTEST_SELECTION;
                    algorithm += "[ShortestSelection]";
                }
                else if (selection_alg == "RANDOM")
                {
                    selection_type = RANDOM_SELECTION;
                    algorithm += "[RandomSelection]";
                }
                else if (selection_alg == "MINCONF")
                {
                    selection_type = MINCONF_SELECTION;
                    algorithm += "[MinConfSelection]";
                }
                else
                {
                    selection_type = NONE_SELECTION;
                }
                
                
                std::string minimization_alg = parameters::get_attribute_as<std::string>("minimization_type", reader, template_reader, "NONE");
                if (minimization_alg == "MASTER_ONLY")
                {
                    minimization_type = MASTER_ONLY_MINIMIZATION;
                    algorithm += "[MASTER_ONLY_MINIMIZATION]";
                }
                else if (minimization_alg == "JOINT")
                {
                    minimization_type = JOINT_MINIMIZATION;
                    algorithm += "[JOINT_MINIMIZATION]";
                }
                else
                {
                    minimization_type = NONE_MINIMIZATION;
                }
                
                /** Read number of objects per arm*/
                experiment = parameters::get_attribute_as<std::string> ("experiment", reader, template_reader);
                plans_directory = parameters::get_attribute_as<std::string>("plans_directory", reader, template_reader);
                results_file =  parameters::get_attribute_as<std::string>("results_file", reader, template_reader, "planning_results.txt");
//                PRX_PRINT("Num left arm tasks: " << num_left_plans, PRX_TEXT_BLUE);
//                PRX_PRINT("Num left arm tasks: " << num_right_plans, PRX_TEXT_RED);
                
                /** Resize Plans Vectors - can't read in plans yet until spaces are linked */
                for(unsigned i = 0; i < object_types.size(); ++i)
                {
                    std::vector<sim::plan_t> left_plans(num_plans_per_object_type[i]);
                    std::vector<sim::plan_t> right_plans(num_plans_per_object_type[i]);
                    left_arm_plans[object_types[i]] = left_plans;
                    right_arm_plans[object_types[i]] = right_plans;
                }
                
                /** Read in task assignments */
                left_arm_tasks = parameters::get_attribute_as<std::vector<unsigned> >("left_arm_tasks", reader, template_reader);
                right_arm_tasks = parameters::get_attribute_as<std::vector<unsigned> >("right_arm_tasks", reader, template_reader);
                /** Read in corresponding object assignments */
                left_arm_objects = parameters::get_attribute_as<std::vector<std::string> >("left_arm_objects", reader, template_reader);
                right_arm_objects = parameters::get_attribute_as<std::vector<std::string> >("right_arm_objects", reader, template_reader);
                
                std::string coordination_bias = parameters::get_attribute_as<std::string>("bias_type", reader, template_reader, "JOINT");
                
                if (coordination_bias == "JOINT")
                {
                    bias_alg = coordination_constraint_astar_t::JOINT_BIAS;
                    algorithm += "[JointBias]";
                }
                else if (coordination_bias == "RIGHT")
                {
                    bias_alg = coordination_constraint_astar_t::BIAS_RIGHT;
                    algorithm += "[RightBias]";
                }
                else if (coordination_bias == "LEFT")
                {
                    bias_alg = coordination_constraint_astar_t::BIAS_LEFT;
                    algorithm += "[LeftBias]";
                }
                else if (coordination_bias == "ANY")
                {
                    bias_alg = coordination_constraint_astar_t::ANY_BIAS;
                    algorithm += "[AnyBias]";
                }
                
                if (use_preprocessed_constraints)
                {
                    constraints_directory = parameters::get_attribute_as<std::string>("constraints_directory", reader, template_reader);
                    precomputed_constraints = new coordination_constraints_t();
                    if (batch)
                    {
                        multi_constraint_astar = new multi_constraint_astar_t();
                    }
                    else 
                    {
                        constraint_aware_astar = new coordination_constraint_astar_t();
                    }
                }
                
                num_left_tasks_to_assign = left_arm_tasks.size();
                num_right_tasks_to_assign = right_arm_tasks.size();
                
                original_left_tasks = left_arm_tasks;
                original_right_tasks = right_arm_tasks;
                
                original_left_objects = left_arm_objects;
                original_right_objects = right_arm_objects;
                
                number_of_tasks = left_arm_tasks.size();
                
                PRX_DEBUG_COLOR("End init", PRX_TEXT_MAGENTA);
            }

            void coordination_constraint_tp_t::link_world_model( world_model_t* const model )
            {
                task_planner_t::link_world_model(model);
                manipulation_model = dynamic_cast<manipulation_world_model_t*>(model);
                PRX_ASSERT(manipulation_model != NULL);
                
                PRX_DEBUG_COLOR("END link world model", PRX_TEXT_CYAN);
            }

            void coordination_constraint_tp_t::setup()
            {
                PRX_DEBUG_COLOR("Begin setup", PRX_TEXT_CYAN);
                //Now that we have the entire schtick initialized, collect info:
                //Gather all of the manipulators and cup, set them in their order
                find_plants();

                //Then, find start state information and such
                store_start_state();
                
                fully_coordinated_plan.link_control_space(full_control_space);
                
                if (use_preprocessed_constraints)
                {
                    // Deserialize constraints
                    char* w = std::getenv("PRACSYS_PATH");
                    std::string dir(w);
                    dir += (constraints_directory);
                    boost::filesystem::path output_dir(dir);
                    if( !boost::filesystem::exists(output_dir) )
                    {
                        PRX_FATAL_S("Dir: " << dir << " does not exist!");
                    }
                    if(!precomputed_constraints->deserialize_constraints(object_types,num_plans_per_object_type, dir, experiment))
                    {
                        PRX_FATAL_S ("Failed to deserialize constraints!");
                    }
                    else
                    {
                        PRX_WARN_S ("Successfully loaded constraints!");
                    }
                    
                    if (batch)
                    {
                        PRX_WARN_S ("Batch mode GOOOO!");
                        multi_constraint_astar->link_constraints(precomputed_constraints);
                        multi_constraint_astar->link_task_plans(&right_arm_plans, &left_arm_plans);
                    }
                    else
                    {
                        PRX_WARN_S ("Constraint aware astar GOOOO!");
                        constraint_aware_astar->link_constraints(precomputed_constraints);
                    }
//                    PRX_ASSERT(false);
                }
                
                PRX_DEBUG_COLOR("END setup", PRX_TEXT_MAGENTA);
            }


            void coordination_constraint_tp_t::link_query( query_t* new_query )
            {
                PRX_DEBUG_COLOR("BEGIN Link Query", PRX_TEXT_MAGENTA);
                //Until I know what is special about this query...
                task_planner_t::link_query( new_query );
                mp_query = dynamic_cast<motion_planning_query_t*>(new_query);
                if (mp_query == NULL)
                {
                    PRX_FATAL_S ("Must have motion planning query!");
                }
                
                //TODO: Fix this?
                //mp_query->link_spaces(full_state_space, full_control_space);
                
//                if(!use_preprocessed_constraints)
//                {
//                    prm_query = dynamic_cast<coordination_prm_query_t*>(new_query);
//
//                    PRX_ASSERT(prm_query != NULL);
//                }
                
                PRX_DEBUG_COLOR("END Link Query", PRX_TEXT_MAGENTA);
            }

            // =======================================================
            // =======================================================
            // =======================================================
            // ================......===========......================
            // ==================.....=========.....==================
            // ===================.....=======.....===================
            // ====================.....=====.....====================
            // =====================.....===.....=====================
            // ======================.....=.....======================
            // =======================.........=======================
            // ========================.......========================
            // =========================.....=========================
            // ========================.......========================
            // =========....==========.........==========....=========
            // =======........=======.....=.....=======........=======
            // ======...====...=====.....===.....=====...====...======
            // ======..........====.....=====.....====..........======
            // ======..===========.....=======.....===..==============
            // ======....==....==.....=========.....==....==....======
            // =======........=......===========......=........=======
            // =======================================================
            // =======================================================
            // =======================================================

            bool coordination_constraint_tp_t::execute()
            {
                PRX_DEBUG_COLOR("Begin execute!", PRX_TEXT_CYAN);
                if (!use_preprocessed_constraints)
                {
//                    return naive_coord_manip_tp_t::execute();
                    return false;
                }
                else
                {

                    unsigned new_experiment_counter = 0;
                    
                    do
                    {
                        PRX_PRINT("---- NEW EXPERIMENT-------------", PRX_TEXT_MAGENTA);
                    
                        do
                        {
                            /** Clear plans and reset statistics */
                            left_arm_full_plan.clear();
                            right_arm_full_plan.clear();
                            mp_query->plan.clear();
                            // = = = DEBUG = = =
                            PRX_PRINT("----Begining COORDINATION CONSTRAINT!!!!!!! Execution.-------------", PRX_TEXT_BLUE);
                            computation_timer.reset();

                            assign_tasks();

                            if (scheduling_type != INCREMENTAL_SCHEDULE || !incremental)
                            {
                                construct_full_individual_plans();
                            }

                            coordinate_paths();

                            save_statistics();

                            current_experiment++;

                            get_old_tasks();
                        }
                        while(current_experiment < number_of_experiments);

                        get_new_tasks();
                        write_statistics();
                        original_left_tasks = left_arm_tasks;
                        original_right_tasks = right_arm_tasks;
                        
                        original_left_objects = left_arm_objects;
                        original_right_objects = right_arm_objects;
                        
                        min_solution_time = min_computation_time = 100000.0;
                        max_solution_time = max_computation_time = 0.0;
                        solution_time.clear(); computation_time.clear();
                        current_experiment = 0;
                        
                        new_experiment_counter++;
                    }
                    while(new_experiment_counter < total_new_experiments);

                }
                
                // = = =
                resolve_query();

                return true;
            }
            
            void coordination_constraint_tp_t::get_old_tasks()
            {
                PRX_DEBUG_COLOR("Assigning old tasks: ", PRX_TEXT_CYAN);
                print_tasks();
                left_arm_tasks = original_left_tasks;
                right_arm_tasks = original_right_tasks;
                left_arm_objects = original_left_objects;
                right_arm_objects = original_right_objects;
                print_tasks();
            }
            
            void coordination_constraint_tp_t::get_new_tasks()
            {
                left_arm_tasks.clear();
                right_arm_tasks.clear();
                left_arm_objects.clear();
                right_arm_objects.clear();
                
                int new_left_task, new_right_task;
                
                for(unsigned i = 0; i < num_left_tasks_to_assign; i++)
                {
                    int object = std::rand() % object_types.size();
                    
                    new_left_task = std::rand() % num_plans_per_object_type[object];
                    
                    left_arm_tasks.push_back(new_left_task);
                    left_arm_objects.push_back(object_types[object]);
                }
                
                for(unsigned i = 0; i < num_right_tasks_to_assign; i++)
                {
                    int object = std::rand() % object_types.size();
                    
                    new_right_task = std::rand() % num_plans_per_object_type[object];
                    
                    right_arm_tasks.push_back(new_right_task);
                    right_arm_objects.push_back(object_types[object]);
                }
                
                PRX_PRINT ("NEW EXPERIMENT!", PRX_TEXT_BLUE);
                print_tasks();
            }
            
            void coordination_constraint_tp_t::assign_tasks()
            {
                PRX_DEBUG_COLOR("// =======================================================", PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("// ======================TASK ASSIGNMENT==================", PRX_TEXT_CYAN);
                PRX_DEBUG_COLOR("// =======================================================", PRX_TEXT_CYAN);
                print_tasks();
                if (scheduling_type == RANDOM_SCHEDULE)
                {
                    PRX_PRINT("---- RANDOM ASSIGNMENT ----", PRX_TEXT_CYAN);
                    random_assignment();

                }
                else if (scheduling_type == INCREMENTAL_SCHEDULE)
                {
                    PRX_PRINT("---- INCREMENTAL ASSIGNMENT ----", PRX_TEXT_CYAN);
                    incremental_assignment();

                }
                else if (scheduling_type == MANUAL_SCHEDULE)
                {
                    PRX_PRINT("---- MANUAL ASSIGNMENT ----", PRX_TEXT_CYAN);

                }
                else if (scheduling_type == MINCONF_SCHEDULE)
                {
                    PRX_PRINT("---- MIN CONF ASSIGNMENT ----", PRX_TEXT_CYAN);
                    min_conf_assignment();

                }
                PRX_DEBUG_COLOR("AFTER ASSIGNMENT!", PRX_TEXT_GREEN);
                print_tasks();
            }
            
            void coordination_constraint_tp_t::min_conf_assignment()
            {
                std::vector<unsigned> new_right_tasks, new_left_tasks;
                std::vector<std::string> new_right_objects, new_left_objects;
                
                right_arm_tasks = original_right_tasks;
                left_arm_tasks = original_left_tasks;
                
                left_arm_objects = original_left_objects;
                right_arm_objects = original_right_objects;
                
                while (new_right_tasks.size() < number_of_tasks)
                {
                    unsigned smallest_conflict_area = PRX_INFINITY;
                    unsigned smallest_right, smallest_left;
                    std::string smallest_right_object, smallest_left_object;
                    unsigned current_right, current_left;
                    std::string current_right_object, current_left_object;
                    for (unsigned right_index = 0; right_index < right_arm_tasks.size(); right_index++)
                    {
                        current_right = right_arm_tasks[right_index];
                        current_right_object = right_arm_objects[right_index];
                        for (unsigned left_index = 0; left_index < left_arm_tasks.size(); left_index++)
                        {
                            current_left = left_arm_tasks[left_index];
                            current_left_object = left_arm_objects[left_index];
                            unsigned test_conflict_area = precomputed_constraints->estimate_conflict_area(current_right, current_left, current_right_object, current_left_object);
                            
                            if (test_conflict_area < smallest_conflict_area)
                            {
                                smallest_conflict_area = test_conflict_area;
                                smallest_right = current_right;
                                smallest_left = current_left;
                                smallest_right_object = current_right_object;
                                smallest_left_object = current_left_object;
                            }
                        }
                    }
                    
                    new_right_tasks.push_back(smallest_right);
                    new_left_tasks.push_back(smallest_left);
                    
                    new_right_objects.push_back(smallest_right_object);
                    new_left_objects.push_back(smallest_left_object);
                    
                    remove_task(smallest_right, BAXTER_RIGHT_ARM);
                    remove_task(smallest_left, BAXTER_LEFT_ARM);
                }
                
                right_arm_tasks = new_right_tasks;
                left_arm_tasks = new_left_tasks;
                
            }
            
            void coordination_constraint_tp_t::incremental_assignment()
            {
                left_arm_full_plan.clear();
                right_arm_full_plan.clear();
                PRX_PRINT("// =======================================================", PRX_TEXT_BLUE);
                PRX_PRINT("// ================== CONSTRAINT Individual Full Paths================", PRX_TEXT_BLUE);
                PRX_PRINT("// =======================================================", PRX_TEXT_BLUE);

                unsigned left_counter = 0, right_counter = 0;
                unsigned total_left_tasks = left_arm_tasks.size(), total_right_tasks = right_arm_tasks.size();
                unsigned current_left_object, current_right_object;
                std::string current_left_type, current_right_type;
                unsigned start_left_step = 0, start_right_step = 0;
                unsigned goal_left_step, goal_right_step;
                std::deque<unsigned> left_solution, right_solution;
                
                unsigned master_arm,  slave_arm;
                bool left_arm_finished = false, right_arm_finished = false;
                bool need_new_master = true;
                while (left_counter < total_left_tasks && right_counter < total_right_tasks)
                {
                    PRX_PRINT ("Current LEFT dropped objects: " << left_counter << " and total LEFT tasks: " << total_left_tasks, PRX_TEXT_BLUE);
                    PRX_PRINT ("Current RIGHT dropped objects: " << right_counter << " and total RIGHT tasks: " << total_right_tasks, PRX_TEXT_RED);
                    left_arm_finished = false;
                    right_arm_finished = false;
                    
                    if (need_new_master)
                    {
                        PRX_PRINT ("--------------Need new master!", PRX_TEXT_GREEN);
                        /** Get prioritized task and arm */
                        if (!get_master_and_task_assignment(master_arm, slave_arm, goal_right_step, goal_left_step, current_right_object, current_left_object, current_right_type, current_left_type))
                        {
                            PRX_FATAL_S ("Invalid starting point for scheduling!");
                        }
                        /** Remove task from set */
                        start_left_step = 0; start_right_step = 0;
                        need_new_master = false;
                    }
                    
                    
                    PRX_PRINT ("CURRENT left start: " << start_left_step << ", CURRENT right start: " << start_right_step, PRX_TEXT_MAGENTA);
                    
                    PRX_PRINT ("CURRENT left goal: " << goal_left_step << ", CURRENT right goal: " << goal_right_step, PRX_TEXT_MAGENTA);
                    
                    solve_slave_task_assignment(slave_arm, start_right_step, start_left_step, goal_right_step, goal_left_step, current_right_object, current_left_object, current_right_type, current_left_type, right_solution, left_solution);
                    
                    int prev_left_index = -1, prev_right_index = -1;
                    unsigned last_right_index, last_left_index;
                    PRX_PRINT ("Checking right solution", PRX_TEXT_RED);
                    for(unsigned i = 0; i < right_solution.size(); i++)
                    {
                        last_right_index = right_solution[i];
                        
                        if (last_right_index == prev_right_index)
                        {
                            right_arm_full_plan.copy_onto_back(right_zero_control, simulation::simulation_step);
                        }
                        else
                        {
                            right_arm_full_plan.copy_onto_back(right_arm_plans[current_right_type][current_right_object].consume_control(simulation::simulation_step), simulation::simulation_step);
                        }
                        
                        prev_right_index = last_right_index;

                    }
                    PRX_PRINT ("Last right step: " << last_right_index, PRX_TEXT_RED);

                    if (last_right_index == goal_right_step)
                    {
                        right_arm_finished = true;
                        remove_task(current_right_object, BAXTER_RIGHT_ARM);

                        right_counter++;
                        start_right_step = 0;
                    }
                    else
                    {
                        start_right_step = last_right_index;
                        master_arm = BAXTER_RIGHT_ARM;
                        slave_arm = BAXTER_LEFT_ARM;
                    }
                    PRX_PRINT ("Checking left solution", PRX_TEXT_BLUE);
                    for(unsigned i = 0; i < left_solution.size(); i++)
                    {
                        last_left_index = left_solution[i];
                        
                        if (last_left_index == prev_left_index)
                        {
                            left_arm_full_plan.copy_onto_back(left_zero_control, simulation::simulation_step);
                        }
                        else
                        {
                            left_arm_full_plan.copy_onto_back(left_arm_plans[current_left_type][current_left_object].consume_control(simulation::simulation_step), simulation::simulation_step);
                        }
                        
                        prev_left_index = last_left_index;

                    }
                    PRX_PRINT ("Last left step: " << last_left_index, PRX_TEXT_BLUE);

                    if (last_left_index == goal_left_step)
                    {
                        left_arm_finished = true;
                        remove_task(current_left_object, BAXTER_LEFT_ARM);
                        
                        left_counter++;
                        start_left_step = 0;
                    }
                    else
                    {
                        start_left_step = last_left_index;
                        master_arm = BAXTER_LEFT_ARM;
                        slave_arm = BAXTER_RIGHT_ARM;
                    }
                    
                    PRX_PRINT ("New left start: " << start_left_step << ", new right start: " << start_right_step, PRX_TEXT_GREEN);
                    
                    PRX_PRINT ("New left goal: " << goal_left_step << ", new right goal: " << goal_right_step, PRX_TEXT_GREEN);
                    
                    need_new_master = (left_arm_finished && right_arm_finished);
                }
                
                if (!need_new_master)
                {
                    PRX_PRINT("Some plans leftover!", PRX_TEXT_RED);
                    
                    /** Copy remaining plan over */
                    if (!left_arm_finished)
                    {
                        for(unsigned i = start_left_step; i <= goal_left_step; i++)
                        {
                            left_arm_full_plan.copy_onto_back(left_arm_plans[current_left_type][current_left_object].consume_control(simulation::simulation_step), simulation::simulation_step);
                        }
                        remove_task(current_left_object, BAXTER_LEFT_ARM);
                        
                        for(unsigned i = 0; i < left_arm_tasks.size(); i++)
                        {
                            start_left_step = 0;
                            current_left_object = left_arm_tasks[i];
                            current_left_type = left_arm_objects[i];
                            goal_left_step = left_arm_plans[current_left_type][current_left_object].size() - 1;
                            for(unsigned i = start_left_step; i <= goal_left_step; i++)
                            {
                                left_arm_full_plan.copy_onto_back(left_arm_plans[current_left_type][current_left_object].consume_control(simulation::simulation_step), simulation::simulation_step);
                            }
                        }
                        
                    }
                    else
                    {
                        for(unsigned i = start_right_step; i <= goal_right_step; i++)
                        {
                            right_arm_full_plan.copy_onto_back(right_arm_plans[current_right_type][current_right_object].consume_control(simulation::simulation_step), simulation::simulation_step);
                        }
                        remove_task(current_right_object, BAXTER_RIGHT_ARM);
                        
                        for(unsigned i = 0; i < right_arm_tasks.size(); i++)
                        {
                            start_right_step = 0;
                            current_right_object = right_arm_tasks[i];
                            current_right_type = right_arm_objects[i];
                            goal_right_step = right_arm_plans[current_right_type][current_right_object].size() - 1;
                            for(unsigned i = start_right_step; i <= goal_right_step; i++)
                            {
                                right_arm_full_plan.copy_onto_back(right_arm_plans[current_right_type][current_right_object].consume_control(simulation::simulation_step), simulation::simulation_step);
                            }
                        }
                        
                    }
                }
            }
            
            void coordination_constraint_tp_t::random_assignment()
            {
                /** Random task assignment */
                std::vector<std::pair<unsigned, std::string> > left_task_shuffler;
                
                for (unsigned i = 0; i < left_arm_tasks.size(); ++i)
                {
                    left_task_shuffler.push_back(std::make_pair<unsigned, std::string>(left_arm_tasks[i], left_arm_objects[i]));
                }
                
                std::random_shuffle(left_task_shuffler.begin(), left_task_shuffler.end());
                left_arm_tasks.clear(); left_arm_objects.clear();
                
                for (unsigned i = 0; i < left_task_shuffler.size(); ++i)
                {
                    left_arm_tasks.push_back(left_task_shuffler[i].first);
                    left_arm_objects.push_back(left_task_shuffler[i].second);
                }
                
                std::vector<std::pair<unsigned, std::string> > right_task_shuffler;
                
                for (unsigned i = 0; i < right_arm_tasks.size(); ++i)
                {
                    right_task_shuffler.push_back(std::make_pair<unsigned, std::string>(right_arm_tasks[i], right_arm_objects[i]));
                }
                
                std::random_shuffle(right_task_shuffler.begin(), right_task_shuffler.end());
                right_arm_tasks.clear(); right_arm_objects.clear();
                
                for (unsigned i = 0; i < right_task_shuffler.size(); ++i)
                {
                    right_arm_tasks.push_back(right_task_shuffler[i].first);
                    right_arm_objects.push_back(right_task_shuffler[i].second);
                }
                
            }

            void coordination_constraint_tp_t::solve_slave_task_assignment(unsigned slave_arm, unsigned start_right_step, unsigned start_left_step, unsigned& goal_right_step, unsigned& goal_left_step, unsigned& rob_index, unsigned& lob_index, std::string& right_type, std::string& left_type, std::deque<unsigned>& right_arm_solution_vertices, std::deque<unsigned>& left_arm_solution_vertices)
            {
                right_arm_solution_vertices.clear();
                left_arm_solution_vertices.clear();
                
                unsigned best_lob = 1000, best_rob = 1000;
                unsigned best_rgoal_step = 1000, best_lgoal_step = 1000;
                std::string best_right_type, best_left_type;
                std::deque<unsigned> best_right_solution, best_left_solution;
                
                unsigned best_count = 100000;
                if (slave_arm == BAXTER_LEFT_ARM)
                {
                    for (unsigned i = 0; i < left_arm_tasks.size(); i++)
                    {
                        lob_index = left_arm_tasks[i];
                        left_type = left_arm_objects[i];
                        goal_left_step = left_arm_plans[left_type][lob_index].size() - 1;
                        start_left_step = 0;
                        constraint_aware_astar->set_coordination_problem(rob_index, lob_index, right_type, left_type, bias_alg);
                        constraint_aware_astar->set_start_and_goal(start_right_step, start_left_step, goal_right_step, goal_left_step);
                        if(constraint_aware_astar->solve())
                        {
                        
                            constraint_aware_astar->get_solution_path(right_arm_solution_vertices, left_arm_solution_vertices);
                            unsigned comparison = right_arm_solution_vertices.size() + left_arm_solution_vertices.size();
                            if (comparison <  best_count)
                            {
                                best_count = comparison;
                                best_right_solution = right_arm_solution_vertices;
                                best_left_solution = left_arm_solution_vertices;
                                best_lob = lob_index;
                                best_left_type = left_type;
                                best_lgoal_step = goal_left_step;
                            }
                                
                        }
                        else
                        {
                            PRX_FATAL_S("Unsolvable instance?!");
                        }
                    }
                    
                    right_arm_solution_vertices = best_right_solution;
                    left_arm_solution_vertices = best_left_solution;
                    lob_index = best_lob;
                    left_type = best_left_type;
                    goal_left_step = best_lgoal_step;
                    
                }
                else if (slave_arm == BAXTER_RIGHT_ARM)
                {
                    for (unsigned i = 0; i < right_arm_tasks.size(); i++)
                    {
                        rob_index = right_arm_tasks[i];
                        right_type = right_arm_objects[i];
                        goal_right_step = right_arm_plans[right_type][rob_index].size() - 1;
                        start_right_step = 0;
                        constraint_aware_astar->set_coordination_problem(rob_index, lob_index, right_type, left_type, bias_alg);
                        constraint_aware_astar->set_start_and_goal(start_right_step, start_left_step, goal_right_step, goal_left_step);
                        if(constraint_aware_astar->solve())
                        {
                            constraint_aware_astar->get_solution_path(right_arm_solution_vertices, left_arm_solution_vertices);
                            unsigned comparison = right_arm_solution_vertices.size() + left_arm_solution_vertices.size();
                            if (comparison <  best_count)
                            {
                                best_count = comparison;
                                best_right_solution = right_arm_solution_vertices;
                                best_left_solution = left_arm_solution_vertices;
                                best_rob = rob_index;
                                best_right_type = right_type;
                                best_rgoal_step = goal_right_step;
                            }
                                
                        }
                        else
                        {
                            PRX_FATAL_S("Unsolvable instance?!");
                        }
                    }
                    
                    right_arm_solution_vertices = best_right_solution;
                    left_arm_solution_vertices = best_left_solution;
                    rob_index = best_rob;
                    right_type = best_right_type;
                    goal_right_step = best_rgoal_step;
                    
                }
                else
                {
                    PRX_FATAL_S ("Invalid arm!");
                }

            }
            
            void coordination_constraint_tp_t::remove_task(unsigned task, unsigned arm)
            {
                if (arm == BAXTER_LEFT_ARM)
                {
                    completed_left_arm_tasks.push_back(task);
                    PRX_PRINT("LEFT ARM Before erase", PRX_TEXT_BLUE);
                    print_tasks();
                    std::vector<std::string>::iterator object_iterator;
                    bool found = false; unsigned i = 0;
                    for (object_iterator = left_arm_objects.begin(); object_iterator != left_arm_objects.end(); object_iterator++)
                    {
                        if (left_arm_tasks[i] == task)
                        {
                            found = true;
                            break;
                        }
                    }
                    PRX_ASSERT(found);
                    left_arm_tasks.erase(std::find(left_arm_tasks.begin(),left_arm_tasks.end(), task));
                    left_arm_objects.erase(object_iterator);
                    PRX_PRINT("LEFT ARM  After erase", PRX_TEXT_BLUE);
                    print_tasks();
                }
                else if (arm == BAXTER_RIGHT_ARM)
                {
                    completed_right_arm_tasks.push_back(task);
                    PRX_PRINT("RIGHT ARM Before erase", PRX_TEXT_RED);
                    print_tasks();
                    right_arm_tasks.erase(std::find(right_arm_tasks.begin(),right_arm_tasks.end(), task));
                    PRX_PRINT("RIGHT ARM  After erase", PRX_TEXT_RED);
                    print_tasks();
                }
                else
                {
                    PRX_FATAL_S ("Unknown arm specified: " << arm);
                }
            }
            
            bool coordination_constraint_tp_t::get_master_and_task_assignment(unsigned& master_arm, unsigned& slave_arm, unsigned& goal_right_step, unsigned& goal_left_step, unsigned& rob_index, unsigned& lob_index, std::string& right_type, std::string& left_type)
            {
                if (left_arm_tasks.size() == 0 && right_arm_tasks.size() == 0)
                {
                    master_arm = BAXTER_LEFT_ARM;
                    slave_arm = BAXTER_RIGHT_ARM;
                    return false;
                    
                }
                if (selection_type == NONE_SELECTION)
                {
                    PRX_FATAL_S ("Somehow called selection from an algorithm type that does not select!");
                }
                else if (selection_type == SHORTEST_SELECTION)
                {
                    unsigned shortest_left_index, shortest_right_index;
                    std::string shortest_left_type, shortest_right_type;
                    double shortest_left_plan = 1000000.0, shortest_right_plan = 1000000.0;
                    
                    for (unsigned i = 0; i < left_arm_tasks.size(); ++i)
                    {
                        unsigned current_index = left_arm_tasks[i];
                        std::string current_type = left_arm_objects[i];
                        unsigned plan_index = current_index;
                        double current_size = left_arm_plans[current_type][plan_index].length();
                        
                        if( current_size < shortest_left_plan)
                        {
                            shortest_left_plan = current_size; 
                            shortest_left_index = current_index;
                            shortest_left_type = current_type;
                        }
                    }
                    
                    for (unsigned i = 0; i < right_arm_tasks.size(); ++i)
                    {
                        unsigned current_index = right_arm_tasks[i];
                        std::string current_type = right_arm_objects[i];
                        unsigned plan_index = current_index;
                        double current_size = right_arm_plans[current_type][plan_index].length();
                        
                        if( current_size < shortest_right_plan)
                        {
                            shortest_right_plan = current_size; 
                            shortest_right_index = current_index;
                            shortest_right_type = current_type;
                        }
                    }
                    
                    if ( shortest_left_plan < shortest_right_plan)
                    {
                        lob_index = shortest_left_index;
                        left_type = shortest_left_type;
                        goal_left_step = left_arm_plans[left_type][lob_index].size() - 1;
                        master_arm = BAXTER_LEFT_ARM;
                        slave_arm = BAXTER_RIGHT_ARM;
                    }
                    else
                    {
                        rob_index = shortest_right_index;
                        right_type = shortest_right_type;
                        goal_right_step = right_arm_plans[right_type][rob_index].size() - 1;
                        master_arm = BAXTER_RIGHT_ARM;
                        slave_arm = BAXTER_LEFT_ARM;
                    }
                    
                }
                else if (selection_type == LONGEST_SELECTION)
                {
                    unsigned longest_left_index, longest_right_index;
                    std::string longest_left_type, longest_right_type;
                    double longest_left_plan = 0.0, longest_right_plan = 0.0;
                    
                    for (unsigned i = 0; i < left_arm_tasks.size(); ++i)
                    {
                        unsigned current_index = left_arm_tasks[i];
                        std::string current_type = left_arm_objects[i];
                        unsigned plan_index = current_index;
                        double current_size = left_arm_plans[current_type][plan_index].length();
                        
                        if( current_size > longest_left_plan)
                        {
                            longest_left_plan = current_size; 
                            longest_left_index = current_index;
                            longest_left_type = current_type;
                        }
                    }
                    
                    for (unsigned i = 0; i < right_arm_tasks.size(); ++i)
                    {
                        unsigned current_index = right_arm_tasks[i];
                        std::string current_type = right_arm_objects[i];
                        unsigned plan_index = current_index;
                        double current_size = right_arm_plans[current_type][plan_index].length();
                        
                        if( current_size > longest_right_plan)
                        {
                            longest_right_plan = current_size; 
                            longest_right_type = current_type;
                            longest_right_index = current_index;
                        }
                    }
                    
                    if ( longest_left_plan > longest_right_plan)
                    {
                        lob_index = longest_left_index;
                        left_type = longest_left_type;
                        goal_left_step = left_arm_plans[left_type][lob_index].size() - 1;
                        master_arm = BAXTER_LEFT_ARM;
                        slave_arm = BAXTER_RIGHT_ARM;
                    }
                    else
                    {
                        rob_index = longest_right_index;
                        right_type = longest_right_type;
                        goal_right_step = right_arm_plans[right_type][rob_index].size() - 1;
                        master_arm = BAXTER_RIGHT_ARM;
                        slave_arm = BAXTER_LEFT_ARM;
                    }
                    
                }
                else if (selection_type == MINCONF_SELECTION)
                {
                    unsigned smallest_conflict_area = PRX_INFINITY;
                    unsigned smallest_right, smallest_left;
                    std::string smallest_right_type, smallest_left_type;
                    unsigned current_right, current_left;
                    std::string current_right_type, current_left_type;

                    for (unsigned right_index = 0; right_index < right_arm_tasks.size(); right_index++)
                    {
                        current_right = right_arm_tasks[right_index];
                        current_right_type = right_arm_objects[right_index];
                        for (unsigned left_index = 0; left_index < left_arm_tasks.size(); left_index++)
                        {
                            current_left = left_arm_tasks[left_index];
                            current_left_type = left_arm_objects[left_index];
                            unsigned test_conflict_area = precomputed_constraints->estimate_conflict_area(current_right, current_left, current_right_type, current_left_type);
                            
                            if (test_conflict_area < smallest_conflict_area)
                            {
                                smallest_conflict_area = test_conflict_area;
                                smallest_right = current_right;
                                smallest_left = current_left;
                                smallest_left_type = current_left_type;
                                smallest_right_type = current_right_type;
                            }
                        }
                    }
                    rob_index = smallest_right;
                    right_type = smallest_right_type;
                    goal_right_step = right_arm_plans[right_type][rob_index].size() - 1;
                    master_arm = BAXTER_RIGHT_ARM;
                    slave_arm = BAXTER_LEFT_ARM;
                }
                else if (selection_type == RANDOM_SELECTION)
                {
                    random_assignment();
                    double select_arm = uniform_random();

                    if (left_arm_tasks.size() == 0 || select_arm < 0.5)
                    {
                        rob_index = right_arm_tasks[0];
                        right_type = right_arm_objects[0];
                        goal_right_step = right_arm_plans[right_type][rob_index].size() - 1;
                        master_arm = BAXTER_RIGHT_ARM;
                        slave_arm = BAXTER_LEFT_ARM;
                    }
                    else if (right_arm_tasks.size() == 0 || select_arm >= 0.5)
                    {
                        lob_index = left_arm_tasks[0];
                        left_type = left_arm_objects[0];
                        goal_left_step = left_arm_plans[left_type][lob_index].size() - 1;
                        master_arm = BAXTER_LEFT_ARM;
                        slave_arm = BAXTER_RIGHT_ARM;
                    }
                    else
                    {
                        PRX_FATAL_S("Invalid condition");
                    }
                }
                
                return true;
            }

            void coordination_constraint_tp_t::coordinate_paths()
            {

                PRX_PRINT("// =========================================================================", PRX_TEXT_MAGENTA);
                PRX_PRINT("// =============================== CONSTRAINT PATH COORDINATION=========================", PRX_TEXT_MAGENTA);
                PRX_PRINT("// =========================================================================", PRX_TEXT_MAGENTA);

                if (naive || incremental)
                {
                    PRX_ASSERT(left_arm_full_plan.size() > 0);
                    PRX_ASSERT(right_arm_full_plan.size() > 0);

                    construct_full_joined_plan();
                }
                else if (batch)
                {
                    mp_query->plan = fully_coordinated_plan;
//                        PRX_DEBUG_COLOR ("MP QUERY PLAN" << mp_query->plan.print(), PRX_TEXT_CYAN);
//                        ((motion_planning_query_t*)input_query)->plan = all_arms_plan;
                }
                
            }
            
            
            void coordination_constraint_tp_t::construct_full_individual_plans()
            {

                if (batch)
                {
                    multi_constraint_astar->link_tasks(&right_arm_tasks, &left_arm_tasks, &right_arm_objects, &left_arm_objects);
                    multi_constraint_astar->set_coordination_bias();
                    multi_constraint_astar->solve();
                    multi_constraint_astar->get_solution_plan(right_arm_control_space, left_arm_control_space, full_control_space, fully_coordinated_plan);
//                        ((motion_planning_query_t*)input_query)->plan = all_arms_plan;
                    PRX_PRINT ("Found solution with size: " << fully_coordinated_plan.size(), PRX_TEXT_LIGHTGRAY);
                }
                else if (naive)
                {
                    unsigned left_counter = 0, right_counter = 0;
                    unsigned current_left_object, current_right_object;
                    std::string current_left_type, current_right_type;

                    left_arm_full_plan.clear();
                    right_arm_full_plan.clear();

                    while (left_counter < left_arm_tasks.size() || right_counter < right_arm_tasks.size())
                    {
                        if (right_counter < right_arm_tasks.size())
                        {
                            current_right_object = right_arm_tasks[right_counter];
                            current_right_type = right_arm_objects[right_counter];

                            for(unsigned i = 0; i < right_arm_plans[current_right_type][current_right_object].size(); i++)
                            {
                                double duration = right_arm_plans[current_right_type][current_right_object][i].duration;
                                right_arm_full_plan.copy_onto_back(right_arm_plans[current_right_type][current_right_object][i].control, duration);
                                left_arm_full_plan.copy_onto_back(left_zero_control, duration);

                            }

                            right_counter++;
                        }

                        if (left_counter < left_arm_tasks.size())
                        {
                            current_left_object = left_arm_tasks[left_counter];
                            current_left_type = left_arm_objects[left_counter];

                            for(unsigned i = 0; i < left_arm_plans[current_left_type][current_left_object].size(); i++)
                            {
                                double duration = left_arm_plans[current_left_type][current_left_object][i].duration;
                                left_arm_full_plan.copy_onto_back(left_arm_plans[current_left_type][current_left_object][i].control, duration);
                                right_arm_full_plan.copy_onto_back(right_zero_control, duration);

                            }

                            left_counter++;
                        }
                    }
                }                  
            }
            
            void coordination_constraint_tp_t::construct_full_joined_plan()
            {
                PRX_DEBUG_COLOR("Construct full joined plan!", PRX_TEXT_CYAN);
                /** Make sure size of plans are equivalent*/
                unsigned left_size= left_arm_full_plan.size();
                unsigned right_size = right_arm_full_plan.size();
                
                if (left_size < right_size)
                {
                    control_t* ctrl = left_arm_full_plan.back().control;
                    double dur = left_arm_full_plan.back().duration;
                    for(unsigned i = 0; i < right_size - left_size; i++)
                    {
                        left_arm_full_plan.copy_onto_back(ctrl, dur);
                    }
                }
                else if (left_size > right_size)
                {
                    control_t* ctrl = right_arm_full_plan.back().control;
                    double dur = right_arm_full_plan.back().duration;
                    for(unsigned i = 0; i < left_size - right_size; i++)
                    {
                        right_arm_full_plan.copy_onto_back(ctrl, dur);
                    }
                }
                
                PRX_ASSERT(left_arm_full_plan.size() == right_arm_full_plan.size());
                /** Create full plan */
                model->use_context("full_space");
                
                ((motion_planning_query_t*)input_query)->link_spaces(model->get_state_space(), model->get_control_space());
                control_t* new_control = model->get_control_space()->alloc_point();
                for(unsigned i = 0; i < left_arm_full_plan.size(); i++)
                {
                    double dur = left_arm_full_plan[i].duration;
                    left_arm_control_space->copy_from_point(left_arm_full_plan[i].control);
                    right_arm_control_space->copy_from_point(right_arm_full_plan[i].control);
                    model->get_control_space()->copy_to_point(new_control);
                    ((motion_planning_query_t*)input_query)->plan.copy_onto_back(new_control,dur);
                    // potentially delete control here
                }
                model->get_control_space()->free_point(new_control);
            }
            
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================

            void coordination_constraint_tp_t::resolve_query()
            {
                
                model->use_context("full_space");
//                naive_coord_manip_tp_t::resolve_query();
                PRX_PRINT("Coordination Constraint Resolve Query", PRX_TEXT_RED);
                
                PRX_DEBUG_COLOR ("PLAN: " << mp_query->plan.print(), PRX_TEXT_GREEN);
//                
//                std::stringstream left_tasks_string, right_tasks_string;
//
//                left_tasks_string << "COMPLETED Left tasks: [";
//                right_tasks_string << "COMPLETED Right tasks: [";
//
//                foreach(unsigned index, completed_left_arm_tasks)
//                {
//                    left_tasks_string << index << ",";
//                }
//                left_tasks_string << "]";
//                foreach(unsigned index, completed_right_arm_tasks)
//                {
//                    right_tasks_string << index << ",";
//                }
//                right_tasks_string << "]";
//                PRX_PRINT(left_tasks_string.str(), PRX_TEXT_BLUE);
//                PRX_PRINT(right_tasks_string.str(), PRX_TEXT_RED);
                
//                model->use_context("full_space");
//                prm_query->plan.clear();
//                
//                control_t* new_control = full_control_space->alloc_point();
//                
//                for(unsigned i = 0; i < prm_query->path.size();i++)
//                {
//                    full_state_space->copy_from_point(prm_query->path[i]);
//                    both_arm_state_space->copy_to_point(new_control);
//                    prm_query->plan.copy_onto_back(new_control, simulation::simulation_step);
//                }
            }
            
            void coordination_constraint_tp_t::save_statistics()
            {
                double new_solution_time = mp_query->plan.length();
                double new_computation_time = computation_timer.measure();
                
                solution_time.push_back(new_solution_time);
                computation_time.push_back(new_computation_time);
                
                if (new_solution_time < min_solution_time)
                    min_solution_time = new_solution_time;
                
                if (new_solution_time > max_solution_time)
                    max_solution_time = new_solution_time;
                
                if (new_computation_time < min_computation_time)
                    min_computation_time = new_computation_time;
                
                if (new_computation_time > max_computation_time)
                    max_computation_time = new_computation_time;
            
            }
            
            void coordination_constraint_tp_t::write_statistics()
            {
                
                model->use_context("full_space");
//                naive_coord_manip_tp_t::resolve_query();
                
                /** Calculate averages */
                
                double average_solution_time = 0.0, average_computation_time = 0.0;
                double solution_standard_deviation = 0.0, computation_standard_deviation = 0.0;
                
                /** Average solution time **/
                for(unsigned i = 0; i < solution_time.size(); i++)
                {
                    PRX_DEBUG_COLOR("Solution time: " << solution_time[i], PRX_TEXT_GREEN);
                    average_solution_time += solution_time[i];
                }
                average_solution_time /= solution_time.size();
                /** Calculate standard deviation */
                for(unsigned i = 0; i < solution_time.size(); i++)
                {
                    solution_standard_deviation += std::pow(solution_time[i] - average_solution_time, 2);
                }
                
                solution_standard_deviation = std::sqrt((solution_standard_deviation/solution_time.size()));
                
                /** Average computation time **/
                for(unsigned i = 0; i < computation_time.size(); i++)
                {
                    PRX_DEBUG_COLOR("computation time: " << computation_time[i], PRX_TEXT_GREEN);
                    average_computation_time += computation_time[i];
                }
                average_computation_time /= computation_time.size();
                /** Calculate standard deviation */
                for(unsigned i = 0; i < computation_time.size(); i++)
                {
                    computation_standard_deviation += std::pow(computation_time[i] - average_computation_time, 2);
                }
                
                computation_standard_deviation = std::sqrt((computation_standard_deviation/computation_time.size()));
                
                /** Save stats: sample_size, solution_time(min, max, average, std_dev), then comp time*/
                std::ofstream fout;
                fout.open(results_file.c_str(), std::ofstream::app);
                fout << algorithm << " " << experiment << " " << number_of_tasks << " " << (current_experiment) << " " 
                        << min_solution_time << " " << max_solution_time << " " << average_solution_time << " " << solution_standard_deviation << " "
                        << min_computation_time << " " << max_computation_time << " " << average_computation_time << " " << computation_standard_deviation << std::endl;
                
                fout.close();
            }

            void coordination_constraint_tp_t::find_plants()
            {
                model->use_context("full_space");
                std::vector< plant_t* > all_plants;
                model->get_system_graph().get_plants(all_plants);
                
                
                foreach(plant_t* plant, all_plants)
                {
                    //Grab manipulators
                    if( dynamic_cast<manipulator_t*>(plant) != NULL )
                    {
                        manip = dynamic_cast<manipulator_t*>(plant);
                    }
                    //And the movable bodies
                    else if( dynamic_cast<movable_body_plant_t*>(plant) != NULL )
                    {
                        objects.push_back( static_cast<movable_body_plant_t*>(plant) );
                    }
                    //Otherwise, we should throw a warning that there is an unexpected plant here
                    else
                        PRX_WARN_S("Unexpected non-manipulator or non-movable body plant in the system tree.");
                
                }
                PRX_ASSERT(manip != NULL);

                // = = = DEBUG = = =
//                PRX_PRINT("Many Arm manipulation found plants: ", PRX_TEXT_CYAN);
//                for(unsigned i=0; i<manipulators.size(); ++i)
//                {
//                    PRX_PRINT("Manipulator: " << manipulators[i]->get_pathname(), PRX_TEXT_LIGHTGRAY);
//                }
                for(unsigned i=0; i<objects.size(); ++i)
                {
                    PRX_PRINT("Object: " << objects[i]->get_pathname(), PRX_TEXT_LIGHTGRAY);
                }
                // = = =
            }
            
            const statistics_t* coordination_constraint_tp_t::get_statistics()
            {
                return NULL;
            }
            
            bool coordination_constraint_tp_t::succeeded() const
            {
                //Perhaps return based on whether there is a path?
                return false;
            }
            
            void coordination_constraint_tp_t::print_tasks()
            {
                
                std::stringstream left_tasks_string, right_tasks_string;

                left_tasks_string << "Left tasks: [";
                right_tasks_string << "Right tasks: [";

                foreach(unsigned index, left_arm_tasks)
                {
                    left_tasks_string << index << ",";
                }
                left_tasks_string << "]";
                foreach(unsigned index, right_arm_tasks)
                {
                    right_tasks_string << index << ",";
                }
                right_tasks_string << "]";
                PRX_PRINT(left_tasks_string.str(), PRX_TEXT_BLUE);
                PRX_PRINT(right_tasks_string.str(), PRX_TEXT_RED);
            }
            
            void coordination_constraint_tp_t::store_start_state()
            {
                
                manipulation_model->use_context(full_manipulator_context_name);
                global_start_state = manipulation_model->get_state_space()->alloc_point();
                full_state_space = manipulation_model->get_state_space();
                full_control_space = manipulation_model->get_control_space();
                
                manipulation_model->use_context(left_context_name);
                left_arm_state_space = manipulation_model->get_current_manipulation_info()->full_arm_state_space;
                left_arm_control_space = manipulation_model->get_current_manipulation_info()->full_arm_control_space;
                left_arm_safe_state = left_arm_state_space->alloc_point();
                left_zero_control = left_arm_control_space->alloc_point();
                left_arm_control_space->zero(left_zero_control);
                left_arm_full_plan.link_control_space(left_arm_control_space);
                left_arm_full_plan.link_state_space(left_arm_state_space);
                
                manipulation_model->use_context(right_context_name);
                right_arm_state_space = manipulation_model->get_current_manipulation_info()->full_arm_state_space;
                right_arm_control_space = manipulation_model->get_current_manipulation_info()->full_arm_control_space;
                right_arm_safe_state = right_arm_state_space->alloc_point();
                right_zero_control = right_arm_control_space->alloc_point();
                right_arm_control_space->zero(right_zero_control);
                right_arm_full_plan.link_control_space(right_arm_control_space);
                right_arm_full_plan.link_state_space(right_arm_state_space);
                
                manipulation_model->use_context(full_manipulator_context_name);
                
                PRX_PRINT("Full start: " << full_state_space->print_point(global_start_state), PRX_TEXT_MAGENTA);
                //First, let's make sure we have the global start state saved.
        
                all_arms_plan.link_control_space(full_control_space);
                all_arms_plan.link_state_space(full_state_space);
                
                char* w = std::getenv("PRACSYS_PATH");
                std::string dir(w);
                dir += plans_directory;
                boost::filesystem::path output_dir(dir);
                if( !boost::filesystem::exists(output_dir) )
                {
                    PRX_FATAL_S ("Plans directory does not exist!::: " << dir);
                }

                for (unsigned i = 0; i < object_types.size(); ++i)
                {
                    std::string current_object = object_types[i];
                    
                    for(unsigned j = 0; j < num_plans_per_object_type[i]; ++j)
                    {

                        std::stringstream ss;
                        ss << dir << current_object << "/left" << j << ".plan";
                        left_arm_plans[current_object][j].link_control_space(left_arm_control_space);
                        left_arm_plans[current_object][j].link_state_space(left_arm_state_space);                
                        std::ifstream input_stream;
                        input_stream.open(ss.str().c_str());
                        PRX_PRINT("Attempting to open: " << ss.str(), PRX_TEXT_BLUE);
                        left_arm_plans[current_object][j].read_from_stream(input_stream);
                        input_stream.close();
                    }

                    for(unsigned j = 0; j < num_plans_per_object_type[i]; ++j)
                    {
                        std::stringstream ss;
                        ss << dir << current_object << "/right" << j << ".plan";
                        right_arm_plans[current_object][j].link_control_space(right_arm_control_space);
                        right_arm_plans[current_object][j].link_state_space(right_arm_state_space);                
                        std::ifstream input_stream;
                        input_stream.open(ss.str().c_str());
                        PRX_PRINT("Attempting to open: " << ss.str(), PRX_TEXT_BLUE);
                        right_arm_plans[current_object][j].read_from_stream(input_stream);
                        input_stream.close();
                    }
                }
                
            }

        }
    }
}

