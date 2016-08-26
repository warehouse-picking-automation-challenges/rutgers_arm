///**
// * @file naive_coord_manip_tp.cpp
// *
// * @copyright Software License Agreement (BSD License)
// * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick
// * All Rights Reserved.
// * For a full description see the file named LICENSE.
// *
// * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
// *
// * Email: pracsys@googlegroups.com
// */
//
//#include "../../../manipulation/simulation/systems/plants/movable_body_plant.hpp"
//
//#include "prx/utilities/definitions/string_manip.hpp"
//#include "prx/planning/motion_planners/prm_star/prm_star_statistics.hpp"
//#include "prx/planning/motion_planners/motion_planner.hpp"
//#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
//#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
//#include "prx/planning/modules/local_planners/bvp_local_planner.hpp"
//#include "prx/utilities/goals/goal_state.hpp"
//#include "prx/utilities/goals/goal.hpp"
//#include "prx/utilities/distance_metrics/linear_metric/linear_distance_metric.hpp"
//#include "planning/task_planners/naive_coord_manip_tp.hpp"
//
//#include <algorithm> 
//#include <boost/range/adaptor/map.hpp>
//#include <pluginlib/class_list_macros.h>
//
//PLUGINLIB_EXPORT_CLASS(prx::packages::coordination_manipulation::naive_coord_manip_tp_t, prx::plan::planner_t)
//
//namespace prx
//{
//    using namespace util;
//    using namespace sim;
//    using namespace plan;
//
//    namespace packages
//    {
//        namespace coordination_manipulation
//        {
//            naive_coord_manip_tp_t::naive_coord_manip_tp_t()
//            {
//                PRX_PRINT("Creation of the naive coordination manipulation task planner", PRX_TEXT_RED);
//                global_start_state = NULL;
//                left_arm_control_space = NULL;
//                left_arm_state_space = NULL;
//                right_arm_control_space = NULL;
//                right_arm_state_space = NULL;
//                left_arm = NULL;
//                right_arm = NULL;
//                algorithm = "";
//            }
//
//            naive_coord_manip_tp_t::~naive_coord_manip_tp_t()
//            {
//                PRX_PRINT ("\n\n Coordination NAIVE tp destructor \n\n", PRX_TEXT_RED);
////                //Make sure to free the safe states.
////                for( unsigned i=0; i<safe_states.size(); ++i )
////                {
////                    manipulator_state_spaces[i]->free_point(safe_states[i]);
////                    safe_states[i] = NULL;
////                }
//                //Free the global start state
//                model->use_context("full_space");
//                model->get_state_space()->free_point( global_start_state );
//                
//                left_arm_state_space->free_point(left_arm_safe_state);
//                right_arm_state_space->free_point(right_arm_safe_state);
//            }
//
//            void naive_coord_manip_tp_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
//            {
//                //I don't think there are any additional parameters to load here.
//                task_planner_t::init(reader, template_reader);
//                
//                /** Get type of scheduling and coordination */
//                std::string schedule_alg = parameters::get_attribute_as<std::string>("scheduling_type", reader, template_reader, "MANUAL");
//                if (schedule_alg == "RANDOM")
//                {
//                    scheduling_type = RANDOM_SCHEDULE;
//                    algorithm += "[RandomSchedule]";
//                }
//                else if(schedule_alg =="INCREMENTAL")
//                {
//                    scheduling_type = INCREMENTAL_SCHEDULE;
//                    algorithm += "[IncrementalSchedule]";
//                }
//                else if(schedule_alg =="MINCONF")
//                {
//                    scheduling_type = MINCONF_SCHEDULE;
//                    algorithm += "[MinconfSchedule]";
//                }
//                else
//                {
//                    scheduling_type = MANUAL_SCHEDULE;
//                    algorithm += "[ManualSchedule]";
//                }
//                
//                
//                std::string coordination_alg = parameters::get_attribute_as<std::string>("coordination_type", reader, template_reader, "NONE");
//                if (coordination_alg == "PRM")
//                {
//                    coordination_type = PRM_COORDINATION;
//                    //algorithm += "[PRM_COORDINATION]";
//                }
//                else if (coordination_alg == "PRM_MAX")
//                {
//                    coordination_type = PRM_MAX_VEL_COORDINATION;
//                    //algorithm += "[PRM_MAX_VEL_COORDINATION]";
//                }
//                else if (coordination_alg == "GRID")
//                {
//                    coordination_type = GRID_COORDINATION;
//                    //algorithm += "[GRID_COORDINATION]";
//                }
//                else if (coordination_alg == "GRID_MAX")
//                {
//                    coordination_type = GRID_MAX_VEL_COORDINATION;
//                    //algorithm += "[GRID_MAX_VEL_COORDINATION]";
//                }
//                else
//                {
//                    coordination_type = NONE_COORDINATION;
//                    //algorithm += "[NONE_COORDINATION]";
//                }
//                
//                
//                std::string selection_alg = parameters::get_attribute_as<std::string>("selection_type", reader, template_reader, "NONE");
//                if (selection_alg == "LONGEST")
//                {
//                    selection_type = LONGEST_SELECTION;
//                    algorithm += "[LongestSelection]";
//                }
//                else if (selection_alg == "SHORTEST")
//                {
//                    selection_type = SHORTEST_SELECTION;
//                    algorithm += "[ShortestSelection]";
//                }
//                else if (selection_alg == "RANDOM")
//                {
//                    selection_type = RANDOM_SELECTION;
//                    algorithm += "[RandomSelection]";
//                }
//                else if (selection_alg == "MINCONF")
//                {
//                    selection_type = MINCONF_SELECTION;
//                    algorithm += "[MinConfSelection]";
//                }
//                else
//                {
//                    selection_type = NONE_SELECTION;
//                }
//                
//                
//                std::string minimization_alg = parameters::get_attribute_as<std::string>("minimization_type", reader, template_reader, "NONE");
//                if (minimization_alg == "MASTER_ONLY")
//                {
//                    minimization_type = MASTER_ONLY_MINIMIZATION;
//                    algorithm += "[MASTER_ONLY_MINIMIZATION]";
//                }
//                else if (minimization_alg == "JOINT")
//                {
//                    minimization_type = JOINT_MINIMIZATION;
//                    algorithm += "[JOINT_MINIMIZATION]";
//                }
//                else
//                {
//                    minimization_type = NONE_MINIMIZATION;
//                }
//                
//                /** Read number of objects per arm*/
//                num_left_plans = parameters::get_attribute_as<unsigned>("num_left_arm_plans", reader, template_reader);
//                num_right_plans = parameters::get_attribute_as<unsigned>("num_right_arm_plans", reader, template_reader);
//                experiment = parameters::get_attribute_as<std::string> ("experiment", reader, template_reader);
//                plans_directory = parameters::get_attribute_as<std::string>("plans_directory", reader, template_reader);
//                results_file =  parameters::get_attribute_as<std::string>("results_file", reader, template_reader, "planning_results.txt");
//                PRX_PRINT("Num left arm tasks: " << num_left_plans, PRX_TEXT_BLUE);
//                PRX_PRINT("Num left arm tasks: " << num_right_plans, PRX_TEXT_RED);
//                
//                /** Resize Plans Vectors - can't read in plans yet until spaces are linked */
//                left_arm_plans.resize(num_left_plans);
//                right_arm_plans.resize(num_right_plans);
//                
//                /** Read in task assignments */
//                left_arm_tasks = parameters::get_attribute_as<std::vector<unsigned> >("left_arm_tasks", reader, template_reader);
//                right_arm_tasks = parameters::get_attribute_as<std::vector<unsigned> >("right_arm_tasks", reader, template_reader);
//                
//                
//                /** Find coordination prm */
//                foreach(planner_t* planner, planners | boost::adaptors::map_values)
//                {
//                    coordination_prm_t* test = dynamic_cast<coordination_prm_t*>(planner);
//                    if (test != NULL)
//                        coordination_prm = test;
//                }
//                
//                PRX_ASSERT(coordination_prm != NULL);
//                
//                print_tasks();
//            }
//
//            void naive_coord_manip_tp_t::setup()
//            {
//                
//                //Now that we have the entire schtick initialized, collect info:
//                //Gather all of the manipulators and cup, set them in their order
//                find_plants();
//
//                //Then, find start state information and such
//                store_start_state();
//                
//                incremental_partial_plan.link_control_space(left_arm_control_space);
//                incremental_coordinated_plan.link_control_space(full_control_space);
//                fully_coordinated_plan.link_control_space(full_control_space);
//  
//            }
//
//            void naive_coord_manip_tp_t::find_plants()
//            {
//                model->use_context("full_space");
//                std::vector< plant_t* > all_plants;
//                model->get_system_graph().get_plants(all_plants);
//                
//                
//                foreach(plant_t* plant, all_plants)
//                {
//                    //Grab manipulators
//                    if( dynamic_cast<baxter_arm_t*>(plant) != NULL )
//                    {
//                        if (dynamic_cast<baxter_arm_t*>(plant)->is_left_handed() && left_arm == NULL)
//                        {
//                            left_arm = dynamic_cast<baxter_arm_t*>(plant);
//                        }
//                        else if (right_arm == NULL)
//                        {
//                            right_arm = dynamic_cast<baxter_arm_t*>(plant);
//                        }
//                        else
//                        {
//                            PRX_FATAL_S("Only two arms supported")
//                        }
//                    }
//                    //And the movable bodies
//                    else if( dynamic_cast<movable_body_plant_t*>(plant) != NULL )
//                    {
//                        objects.push_back( static_cast<movable_body_plant_t*>(plant) );
//                    }
//                    //Otherwise, we should throw a warning that there is an unexpected plant here
//                    else
//                        PRX_WARN_S("Unexpected non-manipulator or non-movable body plant in the system tree.");
//                
//                }
//                PRX_ASSERT(left_arm != NULL);
//                PRX_ASSERT(right_arm != NULL);
//
//                // = = = DEBUG = = =
////                PRX_PRINT("Many Arm manipulation found plants: ", PRX_TEXT_CYAN);
////                for(unsigned i=0; i<manipulators.size(); ++i)
////                {
////                    PRX_PRINT("Manipulator: " << manipulators[i]->get_pathname(), PRX_TEXT_LIGHTGRAY);
////                }
//                for(unsigned i=0; i<objects.size(); ++i)
//                {
//                    PRX_PRINT("Object: " << objects[i]->get_pathname(), PRX_TEXT_LIGHTGRAY);
//                }
//                // = = =
//            }
//
//            void naive_coord_manip_tp_t::store_start_state()
//            {
//                model->use_context("both_arms");
//                both_arm_control_space = model->get_control_space();
//                both_arm_state_space = model->get_state_space();
//                
//                model->use_context("full_space");
//                full_state_space = model->get_state_space();
//                full_control_space = model->get_control_space();
//                global_start_state = full_state_space->alloc_point();
//                PRX_PRINT("Full start: " << full_state_space->print_point(global_start_state), PRX_TEXT_MAGENTA);
//                //First, let's make sure we have the global start state saved.
//        
//                all_arms_plan.link_control_space(full_control_space);
//                all_arms_plan.link_state_space(full_state_space);
//                
//                char* w = std::getenv("PRACSYS_PATH");
//                std::string dir(w);
//                dir += plans_directory;
//                boost::filesystem::path output_dir(dir);
//                if( !boost::filesystem::exists(output_dir) )
//                {
//                    PRX_FATAL_S ("Plans directory does not exist!::: " << dir);
//                }
//                
//                std::string left_path = reverse_split_path(left_arm->get_pathname()).second;
//                PRX_PRINT("Left arm context: " << left_path, PRX_TEXT_BLUE);
//                model->use_context(left_path);
//                left_arm_control_space = model->get_control_space();
//                left_arm_state_space = model->get_state_space();
//                left_arm_safe_state = left_arm_state_space->alloc_point();
//                left_arm_full_plan.link_control_space(left_arm_control_space);
//                left_arm_full_plan.link_state_space(left_arm_state_space);
//                
//                for(unsigned i = 0; i < left_arm_plans.size(); i++)
//                {
//
//                    std::stringstream ss;
//                    ss << dir << "left/Object" << (i+1) << "_plan.txt";
//                    left_arm_plans[i].link_control_space(left_arm_control_space);
//                    left_arm_plans[i].link_state_space(left_arm_state_space);                
//                    std::ifstream input_stream;
//                    input_stream.open(ss.str().c_str());
//                    PRX_PRINT("Attempting to open: " << ss.str(), PRX_TEXT_BLUE);
//                    left_arm_plans[i].read_from_stream(input_stream);
//                    input_stream.close();
//                }
//                
//                std::string right_path = reverse_split_path(right_arm->get_pathname()).second;
//                PRX_PRINT("Right arm context: " << right_path, PRX_TEXT_RED);
//                model->use_context(right_path);
//                right_arm_control_space = model->get_control_space();
//                right_arm_state_space = model->get_state_space();
//                right_arm_safe_state = right_arm_state_space->alloc_point();
//                right_arm_full_plan.link_control_space(right_arm_control_space);
//                right_arm_full_plan.link_state_space(right_arm_state_space);
//                left_arm_safe_state = left_arm_state_space->alloc_point();
//                for(unsigned i = 0; i < right_arm_plans.size(); i++)
//                {
//                    std::stringstream ss;
//                    ss << dir << "right/Object" << (i+1) << "_plan.txt";
//                    right_arm_plans[i].link_control_space(right_arm_control_space);
//                    right_arm_plans[i].link_state_space(right_arm_state_space);                
//                    std::ifstream input_stream;
//                    input_stream.open(ss.str().c_str());
//                    PRX_PRINT("Attempting to open: " << ss.str(), PRX_TEXT_RED);
//                    right_arm_plans[i].read_from_stream(input_stream);
//                    input_stream.close();
//                }
////                //-----
////                //- Save the safe states
////                //-----
////                for( unsigned i=0; i<manipulators.size(); ++i)
////                {
////                    //Switch to its context
////                    model->use_context(move_context_names[i]);
////                    //Get it's space
////                    space_t* state_space = model->get_state_space();
////                    manipulator_state_spaces.push_back(state_space);
////
////                    //Allocate the point, this is the safe state.
////                    safe_states.push_back( state_space->alloc_point() );
////                    state_space->copy_to_point(safe_states.back());
////
////                    // = = = Debug = = =
////                    PRX_PRINT("Storing safe state [" << i << "]: " << state_space->print_point(safe_states.back(), 4), PRX_TEXT_CYAN );
////                    // = = =
////                }
//            }
//
//            void naive_coord_manip_tp_t::link_world_model( world_model_t* const model )
//            {
//                //Again, I don't think anything special has to happen here
//                task_planner_t::link_world_model(model);
//            }
//
//            const statistics_t* naive_coord_manip_tp_t::get_statistics()
//            {
//                return NULL;
//            }
//
//            void naive_coord_manip_tp_t::link_specification( specification_t* new_spec )
//            {
//                //Until I know what is special about the specification...
//                task_planner_t::link_specification( new_spec );
//            }
//
//            void naive_coord_manip_tp_t::link_query( query_t* new_query )
//            {
//                //Until I know what is special about this query...
//                task_planner_t::link_query( new_query );
//                
//                prm_query = dynamic_cast<coordination_prm_query_t*>(new_query);
//                
//                PRX_ASSERT(prm_query != NULL);
//            }
//
//            // =======================================================
//            // =======================================================
//            // =======================================================
//            // ================......===========......================
//            // ==================.....=========.....==================
//            // ===================.....=======.....===================
//            // ====================.....=====.....====================
//            // =====================.....===.....=====================
//            // ======================.....=.....======================
//            // =======================.........=======================
//            // ========================.......========================
//            // =========================.....=========================
//            // ========================.......========================
//            // =========....==========.........==========....=========
//            // =======........=======.....=.....=======........=======
//            // ======...====...=====.....===.....=====...====...======
//            // ======..........====.....=====.....====..........======
//            // ======..===========.....=======.....===..==============
//            // ======....==....==.....=========.....==....==....======
//            // =======........=......===========......=........=======
//            // =======================================================
//            // =======================================================
//            // =======================================================
//
//            bool naive_coord_manip_tp_t::execute()
//            {
//
//                PRX_PRINT("// =======================================================", PRX_TEXT_LIGHTGRAY);
//                PRX_PRINT("// =======================================================", PRX_TEXT_LIGHTGRAY);
//                PRX_PRINT("// =======================================================", PRX_TEXT_LIGHTGRAY);
//                PRX_PRINT("// ================......===========......================", PRX_TEXT_LIGHTGRAY);
//                PRX_PRINT("// ==================.....=========.....==================", PRX_TEXT_LIGHTGRAY);
//                PRX_PRINT("// ===================.....=======.....===================", PRX_TEXT_LIGHTGRAY);
//                PRX_PRINT("// ====================.....=====.....====================", PRX_TEXT_LIGHTGRAY);
//                PRX_PRINT("// =====================.....===.....=====================", PRX_TEXT_LIGHTGRAY);
//                PRX_PRINT("// ======================.....=.....======================", PRX_TEXT_LIGHTGRAY);
//                PRX_PRINT("// =======================.........=======================", PRX_TEXT_LIGHTGRAY);
//                PRX_PRINT("// ========================.......========================", PRX_TEXT_LIGHTGRAY);
//                PRX_PRINT("// =========================.....=========================", PRX_TEXT_LIGHTGRAY);
//                PRX_PRINT("// ========================.......========================", PRX_TEXT_LIGHTGRAY);
//                PRX_PRINT("// =========....==========.........==========....=========", PRX_TEXT_LIGHTGRAY);
//                PRX_PRINT("// =======........=======.....=.....=======........=======", PRX_TEXT_LIGHTGRAY);
//                PRX_PRINT("// ======...====...=====.....===.....=====...====...======", PRX_TEXT_LIGHTGRAY);
//                PRX_PRINT("// ======..........====.....=====.....====..........======", PRX_TEXT_LIGHTGRAY);
//                PRX_PRINT("// ======..===========.....=======.....===..==============", PRX_TEXT_LIGHTGRAY);
//                PRX_PRINT("// ======....==....==.....=========.....==....==....======", PRX_TEXT_LIGHTGRAY);
//                PRX_PRINT("// =======........=......===========......=........=======", PRX_TEXT_LIGHTGRAY);
//                PRX_PRINT("// =======================================================", PRX_TEXT_LIGHTGRAY);
//                PRX_PRINT("// =======================================================", PRX_TEXT_LIGHTGRAY);
//                PRX_PRINT("// =======================================================\n", PRX_TEXT_LIGHTGRAY);
//
//                // = = = DEBUG = = =
//                PRX_PRINT("Begining Multi-Arm Manipulation Execution.", PRX_TEXT_GREEN);
//                computation_timer.reset();
//
//                assign_tasks();
//                
//                if (scheduling_type != INCREMENTAL_SCHEDULE)
//                {
//                    construct_full_individual_plans();
//
//                    coordinate_paths();
//                }
//                
//                // = = =
//                resolve_query();
//
//                return true;
//            }
//            
//            void naive_coord_manip_tp_t::assign_tasks()
//            {
//                PRX_PRINT("// =======================================================", PRX_TEXT_CYAN);
//                PRX_PRINT("// ======================TASK ASSIGNMENT==================", PRX_TEXT_CYAN);
//                PRX_PRINT("// =======================================================", PRX_TEXT_CYAN);
//                
//                if (scheduling_type == RANDOM_SCHEDULE)
//                {
//                    PRX_PRINT("---- RANDOM ASSIGNMENT ----", PRX_TEXT_CYAN);
//                    random_assignment();
//
//                }
//                else if (scheduling_type == INCREMENTAL_SCHEDULE)
//                {
//                    PRX_PRINT("---- INCREMENTAL ASSIGNMENT ----", PRX_TEXT_CYAN);
//                    prm_query->incremental_assignment = true;
//                    init_coordination_prm();
//                    incremental_assignment();
//                    
//                }
//                else if (scheduling_type == MANUAL_SCHEDULE)
//                {
//                    PRX_PRINT("---- MANUAL ASSIGNMENT ----", PRX_TEXT_CYAN);
//                    
//                }
//                else if (scheduling_type == MINCONF_SCHEDULE)
//                {
//                    min_conf_assignment();
//                    
//                }
//            }
//            
//            void naive_coord_manip_tp_t::random_assignment()
//            {
//                /** Random task assignment */
//                std::random_shuffle(left_arm_tasks.begin(), left_arm_tasks.end());
//                std::random_shuffle(right_arm_tasks.begin(), right_arm_tasks.end());
//                
//                print_tasks();
//            }
//            
//            void naive_coord_manip_tp_t::min_conf_assignment()
//            {
//                PRX_FATAL_S("Not implemented");
//                
//            }
//            
//            void naive_coord_manip_tp_t::incremental_assignment()
//            {
//                unsigned master_arm, master_task;
//                unsigned slave_arm,  slave_task;
//                bool get_new_master = false;
//                unsigned remaining_slave_tasks = 0;
//                
//                left_arm_full_plan.clear();
//                right_arm_full_plan.clear();
//                
//                /** Get prioritized task and arm */
//                if (!get_master_and_task_assignment(master_task, master_arm, slave_arm, remaining_slave_tasks))
//                {
//                    PRX_FATAL_S ("Invalid starting point for scheduling!");
//                }
//                /** Add task to query */
//                add_task_to_query(master_task,master_arm);
//                /** Remove task from set */
//                remove_task(master_task, master_arm);
//                
//                /** Begin loop: solve until one arm is finished with all tasks */
//                PRX_ASSERT(remaining_slave_tasks > 0);
//                bool further_coordination_required = true;
//                while(remaining_slave_tasks > 0)
//                {
//                    PRX_PRINT( "=======================================================", PRX_TEXT_MAGENTA);
//                    PRX_PRINT( "=======================================================", PRX_TEXT_MAGENTA);
//                    PRX_PRINT( "=======================================================", PRX_TEXT_MAGENTA);
//                    PRX_PRINT( "=======================================================", PRX_TEXT_MAGENTA);
//                    PRX_PRINT( "=======================================================", PRX_TEXT_MAGENTA);
//                    PRX_PRINT( "===========================--- ... ---=================", PRX_TEXT_MAGENTA);
//                    PRX_PRINT( "==========================--- ..... ----===============", PRX_TEXT_MAGENTA);
//                    PRX_PRINT( "=========================----- ... -------=============", PRX_TEXT_MAGENTA);
//                    PRX_PRINT( "========================------========------===========", PRX_TEXT_MAGENTA);
//                    PRX_PRINT( "=======================------===========--==  =========", PRX_TEXT_MAGENTA);
//                    PRX_PRINT( "======================-------============  ===  =======", PRX_TEXT_MAGENTA);
//                    PRX_PRINT( "==============---- .... ----===============  ==========", PRX_TEXT_MAGENTA);
//                    PRX_PRINT( "=============---- ...... ---===========================", PRX_TEXT_MAGENTA);
//                    PRX_PRINT( "============----- ...... --============================", PRX_TEXT_MAGENTA);
//                    PRX_PRINT( "===========------- .... ===============================", PRX_TEXT_MAGENTA);
//                    PRX_PRINT( "==========--------=====================================", PRX_TEXT_MAGENTA);
//                    PRX_PRINT( "==========--------=====================================", PRX_TEXT_MAGENTA);
//                    PRX_PRINT( "==========--------=====================================", PRX_TEXT_MAGENTA);
//                    PRX_PRINT( "===========-----=======================================", PRX_TEXT_MAGENTA);
//                    PRX_PRINT( "=======================================================", PRX_TEXT_MAGENTA);
//                    PRX_PRINT( "=======================================================", PRX_TEXT_MAGENTA);
//                    PRX_PRINT( "=======================================================", PRX_TEXT_MAGENTA);
//                    PRX_PRINT( "=======================================================", PRX_TEXT_MAGENTA);
//                    
//                    /** Find the best slave task assignment.*/
//                    incremental_partial_plan.clear();
//                    incremental_coordinated_plan.clear();
//                    solve_slave_task_assignment(slave_arm, incremental_coordinated_plan, incremental_partial_plan, slave_task);
//                    
//                    /** Add solution to fully coordinated plan*/
//                    PRX_DEBUG_COLOR("BEFORE Fully coordinated plan\n" << fully_coordinated_plan.print(), PRX_TEXT_LIGHTGRAY);
//                    fully_coordinated_plan += incremental_coordinated_plan;
//                    PRX_DEBUG_COLOR("AFTER Fully coordinated plan\n" << fully_coordinated_plan.print(), PRX_TEXT_RED);
//                    
//                    /** Remove task from set */
//                    remove_task(slave_task, slave_arm);
//
//                    /** Add partial plan to the master arm plan */
//                    
//                    /** If both arms finished, resolve initial arm*/
//                    if (prm_query->left_finished && prm_query->right_finished)
//                    {
//                        PRX_PRINT("Both arms finished simultaneously!!", PRX_TEXT_LIGHTGRAY);
//                        PRX_ASSERT(incremental_partial_plan.size() == 0);
//                        PRX_PRINT("------------->>>>>>>>>> GET NEW MASTER <<<<<<<<<<<<<<<<---------------", PRX_TEXT_RED)
//                        further_coordination_required = false;
//                        if( get_master_and_task_assignment(master_task, master_arm, slave_arm, remaining_slave_tasks) )
//                        {
//                            left_arm_full_plan.clear();
//                            right_arm_full_plan.clear();
//                            add_task_to_query(master_task,master_arm);
//                            remove_task(master_task, master_arm);
//                            further_coordination_required = true;
//                        }
//                        
//                    }
//                    else if (!prm_query->left_finished && prm_query->right_finished)
//                    {
//                        PRX_PRINT("<<<<<<<<<+------------- LEFT MASTER ", PRX_TEXT_BLUE);
//                        left_arm_full_plan = incremental_partial_plan;
//                        right_arm_full_plan.clear();
//                        master_arm = BAXTER_LEFT_ARM;
//                        slave_arm = BAXTER_RIGHT_ARM;
//                        remaining_slave_tasks = right_arm_tasks.size();
//                        further_coordination_required = true;
//                    }
//                    else if (prm_query->left_finished && !prm_query->right_finished)
//                    {
//                        PRX_PRINT("RIGHT MASTER ----------------+>>>>>>>>> ", PRX_TEXT_BLUE);
//                        right_arm_full_plan = incremental_partial_plan;
//                        left_arm_full_plan.clear();
//                        master_arm = BAXTER_RIGHT_ARM;
//                        slave_arm = BAXTER_LEFT_ARM;
//                        remaining_slave_tasks = left_arm_tasks.size();
//                        further_coordination_required = true;
//                    }
//                    else
//                    {
//                        PRX_FATAL_S ("THIS CANNOT HAPPPEEEENNN!");
//                    }
////                    PRX_WARN_S ("Press 'c' to play plan");
////                    char c;
////                    std:: cin >> c;
////                    if (c == 'c')
////                    {
////                        prm_query->plan = fully_coordinated_plan;
////                        return;
////                    }
//                    
//                }
//                if(further_coordination_required)
//                {
//                    PRX_PRINT("Further coordinatoin required!", PRX_TEXT_GREEN);
//    //                /** One arm has finished all tasks, finish plan for the other arm */
//                    bool partial_right_arm_plan = (right_arm_full_plan.size() > 0);
//                    bool partial_left_arm_plan = (left_arm_full_plan.size() > 0);
//
//                    bool remainder_right_arm_tasks = (right_arm_tasks.size() > 0);
//                    bool remainder_left_arm_tasks  = (left_arm_tasks.size()  > 0);
//
//                    if (master_arm ==  BAXTER_LEFT_ARM)
//                    {
//                        prm_query->plan.clear();
//                        if (remainder_left_arm_tasks)
//                        {
//                            PRX_PRINT("Left arm is not finished, adding remaining tasks", PRX_TEXT_BLUE);
//
//                            for(unsigned i = 0; i < left_arm_tasks.size(); i++)
//                            {
//                                unsigned plan_index = left_arm_tasks[i] - 1;
//                                left_arm_full_plan += left_arm_plans[plan_index];
//                            }
//                        }
//                        if(partial_right_arm_plan)
//                        {
//                            PRX_WARN_S ("This shouldn't happen...");
//                            PRX_PRINT("Right arm has partial plan still. Coordinating motions", PRX_TEXT_BLUE);
//                            prm_query->incremental_assignment = false;
//                            prm_query->setup();
//                            coordination_prm->reset();
//                            coordination_prm->setup();
//                            try
//                            {
//                                coordination_prm->execute();
//                            }
//                            catch( stopping_criteria_t::stopping_criteria_satisfied e )
//                            {
//
//                                PRX_PRINT("Execution was successful! Stats: " << coordination_prm->get_statistics()->get_statistics() << "\n Resolving query...", PRX_TEXT_GREEN);
//                                coordination_prm->resolve_query();
//                                fully_coordinated_plan += prm_query->plan;
//                            }
//                        }
//                        else
//                        {
//                            PRX_PRINT("Right arm is finished. Constructing joint plan", PRX_TEXT_BLUE);
//                            right_arm_full_plan.copy_onto_back(right_arm_safe_state, simulation::simulation_step);
//                            construct_full_joined_plan();
//                            fully_coordinated_plan += prm_query->plan;
//                        }
//                    }
//                    else if (master_arm ==  BAXTER_RIGHT_ARM)
//                    {
//                        prm_query->plan.clear();
//                        if (remainder_right_arm_tasks)
//                        {
//                            PRX_PRINT("right arm is not finished, adding remaining tasks", PRX_TEXT_BLUE);
//
//                            for(unsigned i = 0; i < right_arm_tasks.size(); i++)
//                            {
//                                unsigned plan_index = right_arm_tasks[i] - 1;
//                                right_arm_full_plan += right_arm_plans[plan_index];
//                            }
//                        }
//                        if(partial_left_arm_plan)
//                        {
//                            PRX_WARN_S ("This shouldn't happen...");
//                            PRX_PRINT("left arm has partial plan still. Coordinating motions", PRX_TEXT_BLUE);
//                            prm_query->incremental_assignment = false;
//                            prm_query->setup();
//                            coordination_prm->reset();
//                            coordination_prm->setup();
//                            try
//                            {
//                                coordination_prm->execute();
//                            }
//                            catch( stopping_criteria_t::stopping_criteria_satisfied e )
//                            {
//
//                                PRX_PRINT("Execution was successful! Stats: " << coordination_prm->get_statistics()->get_statistics() << "\n Resolving query...", PRX_TEXT_GREEN);
//                                coordination_prm->resolve_query();
//                                fully_coordinated_plan += prm_query->plan;
//                            }
//                        }
//                        else
//                        {
//                            PRX_PRINT("left arm is finished. Constructing joint plan", PRX_TEXT_BLUE);
//                            left_arm_full_plan.copy_onto_back(left_arm_safe_state, simulation::simulation_step);
//                            construct_full_joined_plan();
//                            fully_coordinated_plan += prm_query->plan;
//                        }
//
//                    }
//                }
//                prm_query->plan = fully_coordinated_plan;
//                
////                /** Left arm is not finished, but right arm is finished */
////                if (no_right_arm_tasks && (remainder_left_arm_tasks) && right_arm_full_plan.size() > 0)
////                {
////                    PRX_PRINT("Left arm is not finished, but right arm is finished", PRX_TEXT_BLUE);
////                    
////                    for(unsigned i = 0; i < left_arm_tasks.size(); i++)
////                    {
////                        unsigned plan_index = left_arm_tasks[i] - 1;
////                        left_arm_full_plan += left_arm_plans[plan_index];
////                    }
//////                    if (right_arm_full_plan.size() == 0)
//////                    {
//////                        right_arm_full_plan.copy_onto_back(right_arm_safe_state, simulation::simulation_step);
//////                    }
////                    prm_query->setup();
////                    coordination_prm->reset();
////                    coordination_prm->setup();
////                    try
////                    {
////                        coordination_prm->execute();
////                    }
////                    catch( stopping_criteria_t::stopping_criteria_satisfied e )
////                    {
////
////                        PRX_PRINT("Execution was successful! Stats: " << coordination_prm->get_statistics()->get_statistics() << "\n Resolving query...", PRX_TEXT_GREEN);
////                        coordination_prm->resolve_query();
////                    }
////                    fully_coordinated_plan += prm_query->plan;
////
////                }
////                /** Right arm is not finished, but left arm is finished */
////                else if (no_left_arm_tasks && (remainder_right_arm_tasks) && left_arm_full_plan.size() > 0)
////                {
////                    PRX_PRINT("Right arm is not finished, but left arm is finished", PRX_TEXT_RED);
////                    
////                    for(unsigned i = 0; i < right_arm_tasks.size(); i++)
////                    {
////                        unsigned plan_index = right_arm_tasks[i] - 1;
////                        right_arm_full_plan += right_arm_plans[plan_index];
////                    }
//////                    if (left_arm_full_plan.size() == 0)
//////                    {
//////                        left_arm_full_plan.copy_onto_back(left_arm_safe_state, simulation::simulation_step);
//////                    }
////                    prm_query->setup();
////                    coordination_prm->reset();
////                    coordination_prm->setup();
////                    try
////                    {
////                        coordination_prm->execute();
////                    }
////                    catch( stopping_criteria_t::stopping_criteria_satisfied e )
////                    {
////
////                        PRX_PRINT("Execution was successful! Stats: " << coordination_prm->get_statistics()->get_statistics() << "\n Resolving query...", PRX_TEXT_GREEN);
////                        coordination_prm->resolve_query();
////                    }
////                    fully_coordinated_plan += prm_query->plan;
////
////                }
////                /** If both arms finished, resolve initial arm*/
////                if (prm_query->left_finished && prm_query->right_finished)
////                {
////                    PRX_PRINT("FINAL: Both arms finished simultaneously!!", PRX_TEXT_LIGHTGRAY);
////                    left_arm_full_plan.clear();
////                    right_arm_full_plan.clear();
////
////                }
////                else if (!prm_query->left_finished && prm_query->right_finished)
////                {
////                    PRX_PRINT("FINAL: RIGHT FINISHED !!", PRX_TEXT_LIGHTGRAY);
////                    left_arm_full_plan = prm_query->partial_plan;
////                    prm_query->plan.clear();
////                    right_arm_full_plan.clear();
////                    right_arm_full_plan.copy_onto_back(right_arm_safe_state, simulation::simulation_step);
////                    construct_full_joined_plan();
////                    fully_coordinated_plan += prm_query->plan;
////                }
////                else if (prm_query->left_finished && !prm_query->right_finished)
////                {
////                    PRX_PRINT("FINAL: LEFT FINISHED!!", PRX_TEXT_LIGHTGRAY);
////                    right_arm_full_plan = prm_query->partial_plan;
////                    prm_query->plan.clear();
////                    left_arm_full_plan.clear();
////                    left_arm_full_plan.copy_onto_back(left_arm_safe_state, simulation::simulation_step);
////                    construct_full_joined_plan();
////                    fully_coordinated_plan += prm_query->plan;
////                    
////                }
////                else
////                {
////                    PRX_FATAL_S ("This cannot happen!");
////                }
////                
////                /** Save plan, resolve query is done */
////                prm_query->plan = fully_coordinated_plan;
//                
//                
//                
//            }
//            void naive_coord_manip_tp_t::solve_slave_task_assignment(unsigned slave_arm, plan_t& best_coordinated_plan, sim::plan_t& best_partial_plan, unsigned& next_task)
//            {
//                bool best_left_finished, best_right_finished;
//                if (slave_arm == BAXTER_LEFT_ARM)
//                {
//                    PRX_ASSERT(left_arm_tasks.size() > 0);
//                    PRX_PRINT("SLAVE LEFT ARM", PRX_TEXT_MAGENTA);
//                    double best_solution = 10000.0;
//                    unsigned best_task;
//                    for(unsigned i = 0; i < left_arm_tasks.size(); i++)
//                    {
//                        unsigned current_task = left_arm_tasks[i];
//                        unsigned plan_index = current_task - 1;
//
//                        PRX_PRINT("// =======================================================", PRX_TEXT_CYAN);
//                        PRX_PRINT("// COORDINATING TASK:" << current_task << "  ==================", PRX_TEXT_CYAN);
//                        PRX_PRINT("// =======================================================", PRX_TEXT_CYAN);
//                        left_arm_full_plan = left_arm_plans[plan_index];
//                        prm_query->setup();
//                        coordination_prm->reset();
//                        coordination_prm->setup();
//                        try
//                        {
//                            coordination_prm->execute();
//                        }
//                        catch( stopping_criteria_t::stopping_criteria_satisfied e )
//                        {
//
//                            PRX_PRINT("Execution was successful! Stats: " << coordination_prm->get_statistics()->get_statistics() << "\n Resolving query...", PRX_TEXT_GREEN);
//                            coordination_prm->resolve_query();
//                        }
//                        
//                        double solution_length = prm_query->plan.length();
//                        
//                        if (minimization_type == JOINT_MINIMIZATION)
//                            solution_length += prm_query->partial_plan.length();
//                        
//                        if ( solution_length < best_solution)
//                        {
//                            best_solution = solution_length;
//                            best_task = current_task;
//                            best_coordinated_plan = prm_query->plan;
//                            best_partial_plan = prm_query->partial_plan;
//                            best_left_finished = prm_query->left_finished;
//                            best_right_finished = prm_query->right_finished;
//                        }
//                        
//                    }
//                    prm_query->left_finished =  best_left_finished;
//                    prm_query->right_finished = best_right_finished;
//                    next_task = best_task;
//                }
//                else if (slave_arm == BAXTER_RIGHT_ARM)
//                {
//                    PRX_ASSERT(right_arm_tasks.size() > 0);
//                    PRX_PRINT("SLAVE RIGHT ARM", PRX_TEXT_MAGENTA);
//                    double best_solution = 10000.0;
//                    unsigned best_task;
//                    for(unsigned i = 0; i < right_arm_tasks.size(); i++)
//                    {
//                        unsigned current_task = right_arm_tasks[i];
//                        unsigned plan_index = current_task - 1;
//                        
//                        PRX_PRINT("// =======================================================", PRX_TEXT_CYAN);
//                        PRX_PRINT("// COORDINATING TASK:" << current_task << "  ==================", PRX_TEXT_CYAN);
//                        PRX_PRINT("// =======================================================", PRX_TEXT_CYAN);
//                        
//                        right_arm_full_plan = right_arm_plans[plan_index];
//                        prm_query->setup();
//                        coordination_prm->reset();
//                        coordination_prm->setup();
//                        try
//                        {
//                            coordination_prm->execute();
//                        }
//                        catch( stopping_criteria_t::stopping_criteria_satisfied e )
//                        {
//
//                            PRX_PRINT("Execution was successful! Stats: " << coordination_prm->get_statistics()->get_statistics() << "\n Resolving query...", PRX_TEXT_GREEN);
//                            coordination_prm->resolve_query();
//                        }
//                        
//                        double solution_length = prm_query->plan.length();
//                        
//                        if (minimization_type == JOINT_MINIMIZATION)
//                            solution_length += prm_query->partial_plan.length();
//                        
//                        if ( solution_length < best_solution)
//                        {
//                            best_solution = solution_length;
//                            best_task = current_task;
//                            best_coordinated_plan = prm_query->plan;
//                            best_partial_plan = prm_query->partial_plan;
//                            best_left_finished = prm_query->left_finished;
//                            best_right_finished = prm_query->right_finished;
//                        }
//                        
//                    }
//                    prm_query->left_finished =  best_left_finished;
//                    prm_query->right_finished = best_right_finished;
//                    next_task = best_task;
//                }
//                else
//                {
//                    PRX_FATAL_S("In a bad state");
//                }
//            }
//            
//            void naive_coord_manip_tp_t::remove_task(unsigned task, unsigned arm)
//            {
//                if (arm == BAXTER_LEFT_ARM)
//                {
//                    completed_left_arm_tasks.push_back(task);
//                    PRX_PRINT("LEFT ARM Before erase", PRX_TEXT_BLUE);
//                    print_tasks();
//                    left_arm_tasks.erase(std::remove(left_arm_tasks.begin(), left_arm_tasks.end(), task), left_arm_tasks.end());
//                    PRX_PRINT("LEFT ARM  After erase", PRX_TEXT_BLUE);
//                    print_tasks();
//                }
//                else if (arm == BAXTER_RIGHT_ARM)
//                {
//                    completed_right_arm_tasks.push_back(task);
//                    PRX_PRINT("RIGHT ARM Before erase", PRX_TEXT_RED);
//                    print_tasks();
//                    right_arm_tasks.erase(std::remove(right_arm_tasks.begin(), right_arm_tasks.end(), task), right_arm_tasks.end()); 
//                    PRX_PRINT("RIGHT ARM  After erase", PRX_TEXT_RED);
//                    print_tasks();
//                }
//                else
//                {
//                    PRX_FATAL_S ("Unknown arm specified: " << arm);
//                }
//            }
//            
//            void naive_coord_manip_tp_t::add_task_to_query(unsigned task, unsigned arm)
//            {
//                if (arm == BAXTER_LEFT_ARM)
//                {
//                    left_arm_full_plan += left_arm_plans[task-1]; 
//                }
//                else if (arm == BAXTER_RIGHT_ARM)
//                {
//                    right_arm_full_plan += right_arm_plans[task-1];   
//                }
//                else
//                {
//                    PRX_FATAL_S ("Unknown arm specified: " << arm);
//                }
//            }
//            
//            bool naive_coord_manip_tp_t::get_master_and_task_assignment(unsigned& master_task, unsigned& master_arm, unsigned& slave_arm, unsigned& remaining_tasks)
//            {
//                if (left_arm_tasks.size() == 0 && right_arm_tasks.size() == 0)
//                {
//                        master_task = -1;
//                        master_arm = BAXTER_LEFT_ARM;
//                        slave_arm = BAXTER_RIGHT_ARM;
//                        remaining_tasks = 0;
//                        return false;
//                    
//                }
//                if (selection_type == NONE_SELECTION)
//                {
//                    PRX_FATAL_S ("Somehow called selection from an algorithm type that does not select!");
//                }
//                else if (selection_type == SHORTEST_SELECTION)
//                {
//                    unsigned shortest_left_index, shortest_right_index;
//                    double shortest_left_plan = 1000000.0, shortest_right_plan = 1000000.0;
//                    
//                    for (unsigned i = 0; i < left_arm_tasks.size(); ++i)
//                    {
//                        unsigned current_index = left_arm_tasks[i];
//                        unsigned plan_index = current_index - 1;
//                        double current_size = left_arm_plans[plan_index].length();
//                        
//                        if( current_size < shortest_left_plan)
//                        {
//                            shortest_left_plan = current_size; 
//                            shortest_left_index = current_index;
//                        }
//                    }
//                    
//                    for (unsigned i = 0; i < right_arm_tasks.size(); ++i)
//                    {
//                        unsigned current_index = right_arm_tasks[i];
//                        unsigned plan_index = current_index - 1;
//                        double current_size = right_arm_plans[plan_index].length();
//                        
//                        if( current_size < shortest_right_plan)
//                        {
//                            shortest_right_plan = current_size; 
//                            shortest_right_index = current_index;
//                        }
//                    }
//                    
//                    if ( shortest_left_plan < shortest_right_plan)
//                    {
//                        master_task = shortest_left_index;
//                        master_arm = BAXTER_LEFT_ARM;
//                        slave_arm = BAXTER_RIGHT_ARM;
//                        remaining_tasks = right_arm_tasks.size();
//                    }
//                    else
//                    {
//                        master_task = shortest_right_index;
//                        master_arm = BAXTER_RIGHT_ARM;
//                        slave_arm = BAXTER_LEFT_ARM;
//                        remaining_tasks = left_arm_tasks.size();
//                    }
//                    
//                }
//                else if (selection_type == LONGEST_SELECTION)
//                {
//                    unsigned longest_left_index, longest_right_index;
//                    double longest_left_plan = 0.0, longest_right_plan = 0.0;
//                    
//                    for (unsigned i = 0; i < left_arm_tasks.size(); ++i)
//                    {
//                        unsigned current_index = left_arm_tasks[i];
//                        unsigned plan_index = current_index - 1;
//                        double current_size = left_arm_plans[plan_index].length();
//                        
//                        if( current_size > longest_left_plan)
//                        {
//                            longest_left_plan = current_size; 
//                            longest_left_index = current_index;
//                        }
//                    }
//                    
//                    for (unsigned i = 0; i < right_arm_tasks.size(); ++i)
//                    {
//                        unsigned current_index = right_arm_tasks[i];
//                        unsigned plan_index = current_index - 1;
//                        double current_size = right_arm_plans[plan_index].length();
//                        
//                        if( current_size > longest_right_plan)
//                        {
//                            longest_right_plan = current_size; 
//                            longest_right_index = current_index;
//                        }
//                    }
//                    
//                    if ( longest_left_plan > longest_right_plan)
//                    {
//                        master_task = longest_left_index;
//                        master_arm = BAXTER_LEFT_ARM;
//                        slave_arm = BAXTER_RIGHT_ARM;
//                        remaining_tasks = right_arm_tasks.size();
//                    }
//                    else
//                    {
//                        master_task = longest_right_index;
//                        master_arm = BAXTER_RIGHT_ARM;
//                        slave_arm = BAXTER_LEFT_ARM;
//                        remaining_tasks = left_arm_tasks.size();
//                    }
//                    
//                }
//                
//                return true;
//            }
//
//            void naive_coord_manip_tp_t::coordinate_paths()
//            {
//                PRX_PRINT("// =========================================================================", PRX_TEXT_MAGENTA);
//                PRX_PRINT("// ===============================PATH COORDINATION=========================", PRX_TEXT_MAGENTA);
//                PRX_PRINT("// =========================================================================", PRX_TEXT_MAGENTA);
//                
//                PRX_ASSERT(left_arm_full_plan.size() > 0);
//                PRX_ASSERT(right_arm_full_plan.size() > 0);
//                
//                if (coordination_type == NONE_COORDINATION)
//                {
//                    PRX_PRINT("---- NO COORDINATION ----", PRX_TEXT_MAGENTA);
//                    // Empty
//                    construct_full_joined_plan();
//                }
//                else if (coordination_type == PRM_COORDINATION || coordination_type == PRM_MAX_VEL_COORDINATION)
//                {
//                    PRX_PRINT("---- PRM COORDINATION ----", PRX_TEXT_MAGENTA);
//                    init_coordination_prm();
//                    coordination_prm->setup();
//                    try
//                    {
//                        coordination_prm->execute();
//                    }
//                    catch( stopping_criteria_t::stopping_criteria_satisfied e )
//                    {
//                        
//                        PRX_PRINT("Execution was successful! Stats: " << coordination_prm->get_statistics()->get_statistics() << "\n Resolving query...", PRX_TEXT_GREEN);
//                        coordination_prm->resolve_query();
//                    }
//                }
//            }
//            
//            void naive_coord_manip_tp_t::init_coordination_prm()
//            {
//                /** Set up specification */
//                input_specification->link_spaces(full_state_space, full_control_space);
//
//                /** Set up query */
//                prm_query->link_left_arm(left_arm, left_arm_state_space, left_arm_control_space, &left_arm_full_plan);
//                prm_query->link_right_arm(right_arm, right_arm_state_space, right_arm_control_space, &right_arm_full_plan);
//                prm_query->plan.link_control_space(full_control_space);
//                prm_query->plan.link_state_space(full_state_space);
//                prm_query->path.link_space(full_state_space);
//
//                model->use_context("imaginary_cup_left");
//                space_t* left_cup_space = model->get_state_space();
//                model->use_context("imaginary_cup_right");
//                space_t* right_cup_space = model->get_state_space();
//                prm_query->link_imaginary_cups(left_cup_space, right_cup_space);
//
//
//                model->use_context("left_armcup");
//                space_t* left_armcup = model->get_state_space();
//                model->use_context("right_armcup");
//                space_t* right_armcup = model->get_state_space();
//                prm_query->link_armcups(left_armcup, right_armcup);
//
//                model->use_context("full_space");
//                prm_query->setup();
//
//                if (coordination_type == PRM_MAX_VEL_COORDINATION)
//                    prm_query->max_velocity_bias = true;
//
//
//                /** Link specification, query, and call setup */
//                input_specification->setup(model);
//                coordination_prm->link_specification(input_specification);
//                coordination_prm->link_query(prm_query);
//            }
//            
//            void naive_coord_manip_tp_t::construct_full_individual_plans()
//            {
//                PRX_PRINT("// =======================================================", PRX_TEXT_BLUE);
//                PRX_PRINT("// ==================Individual Full Paths================", PRX_TEXT_BLUE);
//                PRX_PRINT("// =======================================================", PRX_TEXT_BLUE);
//                /** Create left arm joined plan */
//                for (unsigned i = 0; i < left_arm_tasks.size(); i++)
//                {
//                    unsigned plan_index = left_arm_tasks[i] -1;
//                    left_arm_full_plan += left_arm_plans[plan_index];
//                    
//                }
//                
//                
//                /** Create right arm joined plan */
//                for (unsigned i = 0; i < right_arm_tasks.size(); i++)
//                {
//                    unsigned plan_index = right_arm_tasks[i] -1;
//                    right_arm_full_plan += right_arm_plans[plan_index]; 
//                }
//                                
//               
//            }
//            
//            void naive_coord_manip_tp_t::construct_full_joined_plan()
//            {
//                /** Make sure size of plans are equivalent*/
//                unsigned left_size= left_arm_full_plan.size();
//                unsigned right_size = right_arm_full_plan.size();
//                
//                if (left_size < right_size)
//                {
//                    control_t* ctrl = left_arm_full_plan.back().control;
//                    double dur = left_arm_full_plan.back().duration;
//                    for(unsigned i = 0; i < right_size - left_size; i++)
//                    {
//                        left_arm_full_plan.copy_onto_back(ctrl, dur);
//                    }
//                }
//                else if (left_size > right_size)
//                {
//                    control_t* ctrl = right_arm_full_plan.back().control;
//                    double dur = right_arm_full_plan.back().duration;
//                    for(unsigned i = 0; i < left_size - right_size; i++)
//                    {
//                        right_arm_full_plan.copy_onto_back(ctrl, dur);
//                    }
//                }
//                
//                PRX_ASSERT(left_arm_full_plan.size() == right_arm_full_plan.size());
//                /** Create full plan */
//                model->use_context("full_space");
//                
//                ((motion_planning_query_t*)input_query)->link_spaces(model->get_state_space(), model->get_control_space());
//                control_t* new_control = model->get_control_space()->alloc_point();
//                for(unsigned i = 0; i < left_arm_full_plan.size(); i++)
//                {
//                    double dur = left_arm_full_plan[i].duration;
//                    left_arm_control_space->copy_from_point(left_arm_full_plan[i].control);
//                    right_arm_control_space->copy_from_point(right_arm_full_plan[i].control);
//                    model->get_control_space()->copy_to_point(new_control);
//                    ((motion_planning_query_t*)input_query)->plan.copy_onto_back(new_control,dur);
//                    // potentially delete control here
//                }
//                model->get_control_space()->free_point(new_control);
//            }
//            
//            // =======================================================
//            // =======================================================
//            // =======================================================
//            // =======================================================
//            // =======================================================
//            // =======================================================
//            // =======================================================
//            // =======================================================
//            // =======================================================
//            // =======================================================
//            // =======================================================
//            // =======================================================
//            // =======================================================
//            // =======================================================
//            // =======================================================
//            // =======================================================
//            // =======================================================
//            // =======================================================
//            // =======================================================
//            // =======================================================
//            // =======================================================
//            // =======================================================
//            // =======================================================
//
//            void naive_coord_manip_tp_t::resolve_query()
//            {
//                PRX_PRINT("Beginning NBaive Coordination Resolve Query", PRX_TEXT_RED);
//                std::ofstream fout;
//                fout.open(results_file.c_str(), std::ofstream::app);
//                fout << algorithm << " " << experiment << " " << prm_query->plan.length() << " " << computation_timer.measure() << " " << std::endl;
//                fout.close();
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
//                
////                model->use_context("full_space");
////                prm_query->plan.clear();
////                
////                control_t* new_control = full_control_space->alloc_point();
////                
////                for(unsigned i = 0; i < prm_query->path.size();i++)
////                {
////                    full_state_space->copy_from_point(prm_query->path[i]);
////                    both_arm_state_space->copy_to_point(new_control);
////                    prm_query->plan.copy_onto_back(new_control, simulation::simulation_step);
////                }
//            }
//
//
//
//
//            void naive_coord_manip_tp_t::print_full_state()
//            {
//                std::string old_context = model->get_current_context();
//                model->use_context("full_space");
//                PRX_PRINT("Full State: " << model->get_state_space()->print_memory(3), PRX_TEXT_CYAN);
//                PRX_PRINT("Full Ctrl.: " << model->get_control_space()->print_memory(3), PRX_TEXT_LIGHTGRAY);
//                model->use_context(old_context);
//            }
//
//
//            void naive_coord_manip_tp_t::reset()
//            {
//                //Uh... not sure what to do here?
//                task_planner_t::reset();
//            }
//
//            bool naive_coord_manip_tp_t::succeeded() const
//            {
//                //Perhaps return based on whether there is a path?
//                return false;
//            }
//
//            void naive_coord_manip_tp_t::set_param( const std::string& path, const std::string& parameter_name, const boost::any& value )
//            {
//                //Should be no parameters to set for this TP
//                task_planner_t::set_param(path, parameter_name, value);
//            }
//            
//            void naive_coord_manip_tp_t::print_tasks()
//            {
//                
//                std::stringstream left_tasks_string, right_tasks_string;
//
//                left_tasks_string << "Left tasks: [";
//                right_tasks_string << "Right tasks: [";
//
//                foreach(unsigned index, left_arm_tasks)
//                {
//                    left_tasks_string << index << ",";
//                }
//                left_tasks_string << "]";
//                foreach(unsigned index, right_arm_tasks)
//                {
//                    right_tasks_string << index << ",";
//                }
//                right_tasks_string << "]";
//                PRX_PRINT(left_tasks_string.str(), PRX_TEXT_BLUE);
//                PRX_PRINT(right_tasks_string.str(), PRX_TEXT_RED);
//            }
//
//
//        }
//    }
//}
//
