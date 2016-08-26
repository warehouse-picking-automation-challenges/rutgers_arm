/**
 * @file multi_modal_forward_tp.cppp
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

#include "../../../manipulation/planning/problem_specifications/manipulation_specification.hpp"
#include "../../../manipulation/planning/modules/samplers/manip_sampler.hpp"
#include "../../../manipulation/simulation/plants/movable_body_plant.hpp"
#include "../../../manipulation/planning/modules/samplers/manip_sampler.hpp"
#include "../../../manipulation/simulation/manipulator_simulator.hpp"

#include "../../../baxter/simulation/plants/baxter_arm.hpp"

#include "planning/task_planners/multi_modal_forward_tp.hpp"
#include "planning/modules/hand_off_sampler.hpp"

#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/goals/goal_state.hpp"
#include "prx/utilities/distance_metrics/linear_metric/linear_distance_metric.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/modules/local_planners/bvp_local_planner.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"

#include <pluginlib/class_list_macros.h>

 PLUGINLIB_EXPORT_CLASS(prx::packages::multi_arm::multi_modal_forward_tp_t, prx::plan::planner_t)

 namespace prx
 {
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace multi_arm
        {

            multi_modal_forward_tp_t::multi_modal_forward_tp_t()
            {
                reached_goal = false;
                successful_transfer = false;
                do_preprocessing = true;
                delayed_planning = false;
                expansions = 0;
                pause_frames = 5;
                bias_low_level_samples = false;
            }

            multi_modal_forward_tp_t::~multi_modal_forward_tp_t()
            {
                for( unsigned i=0; i<pre_alloced_points.size(); ++i )
                {
                    global_state_space->free_point( pre_alloced_points[i] );
                }
                global_state_space->free_point( global_start_state );
                global_state_space->free_point( tree_sample_point );

                for( unsigned i=0; i<move_seeds.size(); ++i )
                {
                    for(unsigned j=0; j<move_seeds[i]->size(); ++j)
                    {
                        move_spaces[i]->free_point( (*(move_seeds[i]))[j] );
                    }
                    delete move_spaces[i];
                }

                for( unsigned i=0; i<transfer_seeds.size(); ++i )
                {
                    for( unsigned n=0; n<transfer_seeds[i].size(); ++n )
                    {
                        for(unsigned j=0; j<transfer_seeds[i][n]->size(); ++j)
                        {
                            transfer_spaces[i]->free_point( (*(transfer_seeds[i][n]))[j] );
                        }
                    }
                    delete transfer_spaces[i];
                }

                object_space->free_point( object_state );

                for( unsigned i=0; i<safe_states.size(); ++i )
                {
                    move_spaces[i]->free_point( safe_states[i] );
                }

                for( unsigned i=0; i<planner_queries.size(); ++i )
                {
                    delete planner_queries[i];
                }

                for( unsigned i=0; i<transfer_queries.size(); ++i )
                {
                    for( unsigned j=0; j<transfer_queries[i].size(); ++j )
                    {
                        delete transfer_queries[i][j];
                    }
                }

                for( unsigned i=0; i<move_specifications.size(); ++i )
                {
                    delete move_specifications[i]->metric;
                    delete move_specifications[i]->local_planner;
                    delete move_specifications[i];
                }

                for( unsigned i=0; i<transfer_specifications.size(); ++i )
                {
                    for( unsigned j=0; j<transfer_specifications[i].size(); ++j )
                    {
                        delete transfer_specifications[i][j]->metric;
                        delete transfer_specifications[i][j]->local_planner;
                        delete transfer_specifications[i][j];
                    }
                }

                delete grasp_sampler;
            }

            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // ===========      High-Level Operations      ===========
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================

            void multi_modal_forward_tp_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
            {
                PRX_PRINT("===================", PRX_TEXT_GREEN);
                PRX_PRINT(" = Many-Arm Init = ", PRX_TEXT_GREEN);
                PRX_PRINT("===================", PRX_TEXT_GREEN);

                task_planner_t::init(reader, template_reader);
                //Get whether we are serializing or deserializing
                grasp_retraction_distance = parameters::get_attribute_as<double>("retract_distance", reader, template_reader, 0.05 );

                //If we know we should be building the graph, we must also read spatial interaction information
                stable_sig_filename = parameters::get_attribute_as<std::string>("stable_sig_filename", reader, template_reader, "");
                handoff_sig_filename = parameters::get_attribute_as<std::string>("handoff_sig_filename", reader, template_reader, "");
                automaton_filename = parameters::get_attribute_as<std::string>("automaton_file", reader, template_reader);

                //Figuring out some metaparameters
                object_k = parameters::get_attribute_as<unsigned>("object_k", reader, template_reader, 1);
                max_points = parameters::get_attribute_as<unsigned>("max_points", reader, template_reader, 200);
                double zdiff = parameters::get_attribute_as<double>("grasp_z", reader, template_reader, 0.075);

                use_automaton_states = parameters::get_attribute_as<bool>("use_automaton", reader, template_reader, true);
                discrete_search = parameters::get_attribute_as<bool>("discrete_search", reader, template_reader, true);
                delayed_planning = parameters::get_attribute_as<bool>("delay_planning", reader, template_reader, true);

                order_weight = parameters::get_attribute_as<double>("order_weight", reader, template_reader, 2.0);
                valence_weight = parameters::get_attribute_as<double>("valence_weight", reader, template_reader, 2.0);
                heuristic_weight = parameters::get_attribute_as<double>("heuristic_weight", reader, template_reader, 2.5);
                selection_weight = parameters::get_attribute_as<double>("selection_weight", reader, template_reader, 1.5);
                arms_weight = parameters::get_attribute_as<double>("arms_weight", reader, template_reader, 2.2);

                approach_attempts = parameters::get_attribute_as<unsigned>("approach_attempts", reader, template_reader, 3);
                pause_frames = parameters::get_attribute_as<unsigned>("pause_frames", reader, template_reader, 5);

                bias_low_level_samples = parameters::get_attribute_as<bool>("sample_bias", reader, template_reader, true);

                do_preprocessing = parameters::get_attribute_as<bool>("preprocess", reader, template_reader, false);
                number_of_samples = parameters::get_attribute_as<unsigned>("pose_samples", reader, template_reader, 2);
                num_grasp_samples = parameters::get_attribute_as<unsigned>("grasp_samples", reader, template_reader, 3);

                //Read in the manipulator mappings from input
                std::vector<const parameter_reader_t*> man_map_reader = reader->get_list("manipulator_mapping");
                foreach(const parameter_reader_t* item_reader, man_map_reader)
                {
                    std::string name = item_reader->get_attribute_as< std::string >("system");
                    manipulator_map[name] = item_reader->get_attribute_as< unsigned >("index");

                    PRX_DEBUG_COLOR("Read in mapping: " << name << " : " << manipulator_map[name], PRX_TEXT_BROWN);
                }

                //Let us reuse that grasp z stuff?
                grasp_zs.resize(3);
                grasp_zs[0] = zdiff;
                grasp_zs[1] = 0;
                grasp_zs[2] = -zdiff;

                //We can at least allocate the handoff sampler
                grasp_sampler = new hand_off_sampler_t();
            }

            void multi_modal_forward_tp_t::setup()
            {
                PRX_PRINT("====================", PRX_TEXT_MAGENTA);
                PRX_PRINT(" = Many-Arm Setup = ", PRX_TEXT_MAGENTA);
                PRX_PRINT("====================", PRX_TEXT_MAGENTA);
                //Now that we have the entire schtick initialized, collect info:
                //Gather all of the manipulators and cup, set them in their order
                find_plants();

                //Then, find start state information and such
                store_start_state();

                //Then, let's setup the sampler
                setup_grasp_sampler();

                //Finally, load up the automaton
                if( do_preprocessing )
                {
                    find_planning_waypoints();
                }
                else
                {
                    deserialize_automaton();
                }
            }

            void multi_modal_forward_tp_t::resolve_query()
            {
                PRX_DEBUG_COLOR("Forward Search Resolving query:", PRX_TEXT_GREEN );

                //We need to approach and depart the graph
                clock.reset();
                connect_query_points();
                connect_time = clock.measure_reset();

                //Also, let's get the original distance of the object
                original_object_distance = object_cost_to_go( global_start_state );

                //There are two ways we can try to search
                if( discrete_search )
                    resolve_discrete();
                else
                    resolve_est();

                //Then, if resolution has happened, report some statistics
                report_statistics();
            }

            bool multi_modal_forward_tp_t::execute()
            {
                PRX_PRINT("=================================", PRX_TEXT_CYAN);
                PRX_PRINT(" = Begining Many-Arm execute() = ", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("=================================", PRX_TEXT_CYAN);

                model->use_context( object_context_name );
                motion_planner_t* motion_planner = NULL;

                //For each manipulator, we need to build two graphs
                for( unsigned i=0; i<manipulators.size(); ++i )
                {
                    //-------------------------
                    //Prepare Move Planner.
                    //-------------------------
                    //First, let's put everything away where it's supposed to be
                    go_to_start();
                    //And, make sure that there are no stray controls...
                    set_zero_control();
                    //Use the move context for this arm
                    model->use_context( move_ignoring_context_names[i] );
                    motion_planning_specification_t* graph_specification = move_specifications[i];
                    motion_planner = move_planners[i];

                    ungrasping_sampler->link_info( manipulators[i], move_spaces[i], object_space, transfer_spaces[i] );

                    //Make sure the stopping criterion appropriately links to the planner
                    graph_specification->get_stopping_criterion()->reset();
                    graph_specification->get_stopping_criterion()->link_motion_planner( motion_planner );
                    //Make sure we have the correct spaces linked in
                    graph_specification->link_spaces( model->get_state_space(), model->get_control_space() );
                    //Clear out any seeds which might have been there
                    graph_specification->clear_seeds();
                    //Give it some states to use (from the preprocessing stuffs)
                    graph_specification->set_seeds( *move_seeds[i] );
                    graph_specification->setup(model);
                    //Link in the specification...
                    motion_planner->link_specification(graph_specification);
                    PRX_DEBUG_COLOR("Starting Move Roadmap Construction: " << i << " : " << manipulators[i]->get_pathname(), PRX_TEXT_CYAN);
                    motion_planner->setup();

                    //First, let's put everything away where it's supposed to be
                    go_to_start();
                    //And, make sure that there are no stray controls...
                    set_zero_control();

                    if( do_preprocessing )
                    {
                        PRX_DEBUG_COLOR(":: From state: " << global_state_space->print_memory(3), PRX_TEXT_LIGHTGRAY);
                        try
                        {
                            motion_planner->execute();
                        }
                        catch( stopping_criteria_t::stopping_criteria_satisfied e )
                        {
                            PRX_DEBUG_COLOR("Finished Move Roadmap Construction: " << i, PRX_TEXT_GREEN);
                        }
                    }
                    else
                    {
                        motion_planner->deserialize();
                    }

                    //-------------------------
                    //Prepare Transfer Planners
                    //-------------------------
                    //Alrighty, so for each transfer planner (each grasp)
                    for( unsigned j=0; j<transfer_planners.size(); ++j )
                    {
                        //First, let's put everything away where it's supposed to be
                        go_to_start();

                        //Use the move context for this arm
                        model->use_context( transfer_ignoring_context_names[i] );
                        graph_specification = transfer_specifications[j][i];

                        //Get the right planner for the job
                        motion_planner = transfer_planners[j][i];

                        //Also, the manipulation sampler needs the appropriate information
                        manipulation_sampler->link_info( manipulators[i], move_spaces[i], object_space, transfer_spaces[i] );
                        manipulation_sampler->set_grasp_z( grasp_zs[j] );

                        //Make sure the stopping criterion appropriately links to the planner
                        graph_specification->get_stopping_criterion()->reset();
                        graph_specification->get_stopping_criterion()->link_motion_planner( motion_planner );
                        //Make sure we have the correct spaces linked in
                        graph_specification->link_spaces( model->get_state_space(), model->get_control_space() );
                        //Clear out any seeds which might have been there
                        graph_specification->clear_seeds();
                        //Give it some states to use (from the preprocessing stuffs)
                        graph_specification->set_seeds( *(transfer_seeds[j][i]) );
                        graph_specification->setup( model );
                        //Link in the specification...
                        motion_planner->link_specification( graph_specification );
                        PRX_DEBUG_COLOR("Starting Transfer Roadmap Construction: " << i << " : " << manipulators[i]->get_pathname() << "   for grasp: " << j, PRX_TEXT_MAGENTA);
                        motion_planner->setup();

                        //First, let's put everything away where it's supposed to be
                        go_to_start();

                        if( do_preprocessing )
                        {
                            // PRX_DEBUG_COLOR(":: From state: " << global_state_space->print_memory(3), PRX_TEXT_LIGHTGRAY);
                            try
                            {
                                motion_planner->execute();
                            }
                            catch( stopping_criteria_t::stopping_criteria_satisfied e )
                            {
                                PRX_DEBUG_COLOR("Finished Transfer Roadmap Construction: " << i, PRX_TEXT_RED);
                            }
                        }
                        else
                        {
                            motion_planner->deserialize();
                        }
                    }
                }

                if( do_preprocessing )
                    post_process_graphs();
                else
                    resolve_query();

                return true;
            }


            void multi_modal_forward_tp_t::find_planning_waypoints()
            {
                PRX_PRINT("========================", PRX_TEXT_BROWN);
                PRX_PRINT(" = PREPROCESSING MODE = ", PRX_TEXT_BROWN);
                PRX_PRINT("========================", PRX_TEXT_BROWN);

                //DEBUG: Let's skip the first couple of steps and load an automaton instead to speed up the debugging loop.
                // deserialize_automaton();

                //Build the automaton
                construct_automaton();

                //Generate Low-Level states
                sample_low_level_states();

                //MOAR DEBUG: print the automaton at this point with the node corrections
                // add_edge_records_to_nodes();
                // print_automaton();
                // PRX_FATAL_S("DEBUG SON");

                //Generate the planner seeds
                generate_seeds_from_automaton();
            }
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // ========    Specific High-Level Operations    =========
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================

            void multi_modal_forward_tp_t::resolve_discrete()
            {
                //Need the start in the open set here
                open_set.insert( tree.get_vertex_as< search_node_t >( start_vertex ) );

                PRX_DEBUG_COLOR(" -> Resolve via GMAM: " << open_set.peek_min(), PRX_TEXT_GREEN);

                while( !reached_goal && clock.measure() < 900 )
                {
                    //What's our min node?
                    search_node_t* min_node = open_set.peek_min();
                    PRX_DEBUG_COLOR("Min (" << min_node << ") has values: " << min_node->cost << "(" << min_node->heuristic << ")", PRX_TEXT_LIGHTGRAY);
                    //Try to extend
                    extend( get_min_open_set() );
                    //Inflate its heuristics
                    if( min_node->auto_state != NULL )
                    {
                        min_node->heuristic = (1.0 + min_node->expansions) * heuristics[ get_heuristic_index( min_node->auto_state ) ];
                    }
                    else
                    {
                        //Well, so there's no automaton node, meaning this must be an approach, so just use something reasonably large?
                        min_node->heuristic = (1.0 + min_node->expansions) * 2;
                    }
                    //And makes ure to update the thing...
                    open_set.update( min_node, min_node );
                }

                if( reached_goal )
                {
                    trace_path( goal_vertex );
                }
                else
                {
                    PRX_DEBUG_COLOR("Unable to find a goal. ", PRX_TEXT_RED);
                    //We should touch a file which says we timed out
                    generate_timeout_file();
                }
            }

            void multi_modal_forward_tp_t::resolve_est()
            {
                PRX_DEBUG_COLOR(" -> Resolve via EST.", PRX_TEXT_CYAN);

                while( !reached_goal && clock.measure() < 900 )
                {
                    extend( select_est_no_heuristic() );
                }

                if( reached_goal )
                {
                    trace_path( goal_vertex );
                }
                else
                {
                    PRX_DEBUG_COLOR("Unable to find a goal. ", PRX_TEXT_RED);
                    //We should touch a file which says we timed out
                    generate_timeout_file();
                }
            }

            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // ========           Setup Functions           ==========
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================

            void multi_modal_forward_tp_t::find_plants()
            {
                PRX_PRINT("===================", PRX_TEXT_BLUE);
                PRX_PRINT(" =  Find Plants  = ", PRX_TEXT_BLUE);
                PRX_PRINT("===================", PRX_TEXT_BLUE);

                //First, get the simulator
                manip_sim = dynamic_cast<manipulation_simulator_t*>( model->get_simulator() );
                PRX_ASSERT( manip_sim != NULL );

                model->use_context("full_space");
                std::vector< plant_t* > all_plants;
                model->get_system_graph().get_plants(all_plants);

                final_plan.link_control_space( model->get_control_space() );

                foreach(plant_t* plant, all_plants)
                {
                    //Grab manipulators
                    if( dynamic_cast<manipulator_plant_t*>(plant) != NULL )
                    {
                        manipulators.push_back( static_cast<manipulator_plant_t*>(plant) );
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

                //All of these methods are for a single object...
                PRX_ASSERT( objects.size() == 1 );

                // Need to set up the contexts...
                context_flags true_flags(true, true);
                context_flags active_flags(true, false);

                char tmpstring[100];
                //Set things up for each manipulator
                for( unsigned i=0; i<manipulators.size(); ++i )
                {
                    //New mapping each time... maybe inefficient, but should work
                    util::hash_t<std::string, context_flags> mappings;

                    //First, movement context
                    //We need a name for the context
                    sprintf(tmpstring, "movement_%u", i);
                    // std::string context_name = tmpstring;
                    move_context_names.push_back(tmpstring);

                    //Now, we need: arm (t,t), other arms (t,f), and object (t,f)
                    mappings[manipulators[i]->get_pathname()] = true_flags;
                    model->create_new_planning_context( move_context_names.back(), mappings, active_flags );

                    //Second, the transfer context
                    sprintf(tmpstring, "transfer_%u", i);
                    transfer_context_names.push_back(tmpstring);

                    //This context is: arm(t,t), other (t,f), and object (t,t)
                    mappings[objects[0]->get_pathname()] = true_flags;
                    model->create_new_planning_context( transfer_context_names.back(), mappings, active_flags );

                    //Need preproc trans context: arm(t,t), other arms (f,f), and object (t,t)
                    sprintf(tmpstring, "transfer_ignore_%u", i);
                    transfer_ignoring_context_names.push_back(tmpstring);
                    model->create_new_planning_context( transfer_ignoring_context_names.back(), mappings ); //false, false

                    //Need preproc move context: arm(t,t), other arms (f,f), and object (t,f)
                    mappings[objects[0]->get_pathname()] = active_flags;

                    sprintf(tmpstring, "movement_ignore_%u", i);
                    move_ignoring_context_names.push_back(tmpstring);
                    model->create_new_planning_context( move_ignoring_context_names.back(), mappings ); //false, false
                }

                // Need a context for all of the manipulators being planned for (for final path)
                util::hash_t<std::string, context_flags> map;
                map[objects[0]->get_pathname()] = context_flags(false, false);
                all_manips_context_name = "all_manipulators";
                model->create_new_planning_context( all_manips_context_name, map, true_flags );

                // Also need a context which contains only the object, so we can generate poses and grasps
                object_context_name = "the_object";
                map[objects[0]->get_pathname()] = true_flags;
                model->create_new_planning_context( object_context_name, map );
                model->use_context( object_context_name );
                //Make sure to also grab the object space.
                object_space = model->get_state_space();
                //and get a point from it
                object_state = object_space->alloc_point();
                object_goal_state = object_space->alloc_point();

                //This will be the control space for the final plan
                model->use_context(all_manips_context_name);

                //Store the cast pointer to our manipulation specification
                manip_spec = dynamic_cast< manipulation_specification_t* >(input_specification);
                PRX_ASSERT(manip_spec != NULL);
                //Also, make sure to grab his super special sampler.
                manipulation_sampler = manip_spec->manip_sampler;
                manipulation_sampler->set_transfer_mode();
                //Also need to get the normal sampler cast (so hack, but whatev)
                ungrasping_sampler = dynamic_cast< manip_sampler_t* >( manip_spec->sampler );
                PRX_ASSERT( ungrasping_sampler != NULL );

                //Some local variables for setting this all up
                char tmpname[200];
                std::string handedness = "";
                std::string planner_name = "";
                motion_planner_t* motion_planner;

                //Set up the queries and specifications
                for(unsigned i=0; i<manipulators.size(); ++i)
                {
                    // PRX_DEBUG_COLOR("Building Queries and Specifications for manip: " << i, PRX_TEXT_GREEN );

                    //Because we know we are only dealing with Baxters, we can get handedness.
                    baxter_arm_t* bax_manip = dynamic_cast< baxter_arm_t* >( manipulators[i] );
                    PRX_ASSERT( bax_manip != NULL );
                    if( bax_manip->is_left_handed() )
                    {
                        handedness = "left";
                    }
                    else
                    {
                        handedness = "right";
                    }

                    // MOVE planner

                    //Get the moving motion planner
                    model->use_context( move_context_names[i] );

                    //Get the right planner for the job
                    sprintf( tmpname, "planner_%d_%s_move", (i/2)+1, handedness.c_str() );
                    planner_name = tmpname;
                    motion_planner = dynamic_cast<motion_planner_t*>(planners[planner_name]);
                    if(motion_planner == NULL)
                    {
                        PRX_FATAL_S("No such motion planner called: " << planner_name);
                    }
                    move_planners.push_back(motion_planner);

                    //Make sure to link the model up to planner
                    motion_planner->link_world_model(model);


                    // - - - - - -
                    // - Create the transit Query
                    // - - -
                    //Use the context for manipulator i
                    model->use_context( move_context_names[i] );

                    //Create a new motion planning query
                    motion_planning_query_t* query = new motion_planning_query_t();
                    //If we are in the online mode, make the query recheck collisions
                    // query->q_collision_type = motion_planning_query_t::PRX_NO_COLLISIONS;
                    query->q_collision_type = motion_planning_query_t::PRX_LAZY_COLLISIONS;

                    //Don't forget to link the spaces to the query...
                    query->link_spaces( model->get_state_space(), model->get_control_space() );
                    //Create the Goal class
                    goal_state_t* goal = new goal_state_t();
                    // PRX_DEBUG_COLOR("After building goal: " << model->get_current_context(), PRX_TEXT_LIGHTGRAY );
                    //The goal also needs a distance metric because deprecation and shit
                    distance_metric_t* query_metric = new linear_distance_metric_t();
                    // PRX_DEBUG_COLOR("After building metric: " << model->get_current_context(), PRX_TEXT_LIGHTGRAY );
                    query_metric->link_space( model->get_state_space() );
                    goal->link_metric( query_metric );
                    goal->link_space( model->get_state_space() );
                    // PRX_DEBUG_COLOR("Linking things to the goal: " << model->get_current_context(), PRX_TEXT_LIGHTGRAY );
                    //Set the goal for this query
                    query->set_goal(goal);
                    //Finally, add this query to the list of queries
                    planner_queries.push_back(query);

                    // - - - - - -
                    // - Create the transit specification
                    // - - -
                    //Use the context for manipulator i
                    model->use_context( move_context_names[i] );

                    //Create a new motion planning specification
                    motion_planning_specification_t* spec = new motion_planning_specification_t();
                    // PRX_DEBUG_COLOR("Building Move Specification: " << model->get_current_context(), PRX_TEXT_CYAN );
                    //Point to the specification's stuff
                    spec->local_planner = new bvp_local_planner_t( 1.0 ); //SO SUPER HACKY... at least make this a parameter maybe?
                    spec->validity_checker = input_specification->validity_checker;
                    spec->validity_checker->link_model(model);
                    spec->sampler = ungrasping_sampler;
                    spec->metric = new linear_distance_metric_t(); //Hacky, need better things
                    // PRX_DEBUG_COLOR("After building metric: " << model->get_current_context(), PRX_TEXT_LIGHTGRAY );
                    spec->metric->link_space(model->get_state_space());
                    //PRX_DEBUG_COLOR("Linking metric statespace: " << model->get_state_space()->get_space_name(), PRX_TEXT_MAGENTA)
                    spec->set_stopping_criterion( input_specification->get_stopping_criterion() );
                    spec->link_spaces( model->get_state_space(), model->get_control_space() );
                    spec->clear_seeds();
                    spec->setup(model);
                    //Add this to the moving things
                    move_specifications.push_back(spec);
                    spec = NULL;

                    //TRANSFER planners
                    transfer_planners.resize( grasp_zs.size() );
                    transfer_queries.resize( grasp_zs.size() );
                    transfer_specifications.resize( grasp_zs.size() );

                    for( unsigned j=0; j<grasp_zs.size(); ++j )
                    {
                        //Get the transfer motion planner
                        model->use_context( transfer_context_names[i] );

                        //Get the right planner for the job
                        sprintf( tmpname, "planner_%d_%s_transfer_%d", (i/2)+1, handedness.c_str(), j );
                        planner_name = tmpname;
                        motion_planner = dynamic_cast<motion_planner_t*>(planners[planner_name]);
                        if(motion_planner == NULL)
                        {
                            PRX_FATAL_S("No such motion planner called: " << planner_name);
                        }
                        transfer_planners[j].push_back(motion_planner);

                        //Make sure to link the model up to planner
                        motion_planner->link_world_model(model);

                        // - - - - - -
                        // - Create the transfer Query
                        // - - -
                        // PRX_DEBUG_COLOR("Building query using context: " << model->get_current_context(), PRX_TEXT_CYAN );
                        //Create a new motion planning query
                        query = new motion_planning_query_t();
                        //If we are in the online mode, make the query recheck collisions
                        query->q_collision_type = motion_planning_query_t::PRX_LAZY_COLLISIONS;

                        //Don't forget to link the spaces to the query...
                        query->link_spaces( model->get_state_space(), model->get_control_space() );
                        //Create the Goal class
                        goal = new goal_state_t();
                        // PRX_DEBUG_COLOR("After building goal: " << model->get_current_context(), PRX_TEXT_LIGHTGRAY );
                        //The goal also needs a distance metric because deprecation and shit
                        query_metric = new linear_distance_metric_t();
                        // PRX_DEBUG_COLOR("After building metric: " << model->get_current_context(), PRX_TEXT_LIGHTGRAY );
                        query_metric->link_space( model->get_state_space() );
                        goal->link_metric( query_metric );
                        goal->link_space( model->get_state_space() );
                        // PRX_DEBUG_COLOR("Linking things to the goal: " << model->get_current_context(), PRX_TEXT_LIGHTGRAY );
                        //Set the goal for this query
                        query->set_goal(goal);
                        //Finally, add this query to the list of queries
                        transfer_queries[j].push_back(query);

                        // - - - - - -
                        // - Create the transfer specification
                        // - - -
                        //Also need a transfer specification
                        spec = new motion_planning_specification_t();

                        spec->local_planner = new bvp_local_planner_t( 1.0 );
                        spec->validity_checker = input_specification->validity_checker;
                        spec->validity_checker->link_model(model);
                        spec->sampler = manipulation_sampler;
                        spec->metric = new linear_distance_metric_t();

                        spec->metric->link_space(model->get_state_space());
                        spec->set_stopping_criterion( input_specification->get_stopping_criterion() );
                        spec->link_spaces( model->get_state_space(), model->get_control_space() );
                        // PRX_DEBUG_COLOR("After Linking Spaces: " << model->get_current_context(), PRX_TEXT_LIGHTGRAY );
                        spec->setup(model);
                        //Add this to the transfer specs
                        transfer_specifications[j].push_back(spec);

                    }

                    //For the sake of safety here... perhaps we need to switch over to the other context?
                    model->use_context( move_context_names[i] );
                }

                //Alright, let's allocate some seed vectors so that way the planning can actually use them
                move_seeds.resize( move_planners.size() );
                transfer_seeds.resize( grasp_zs.size() );
                for( unsigned i=0; i<move_seeds.size(); ++i )
                {
                    move_seeds[i] = new std::vector< space_point_t* >();
                }
                for( unsigned j=0; j<transfer_seeds.size(); ++j )
                {
                    transfer_seeds[j].resize( transfer_planners[j].size() );
                    for( unsigned i=0; i<transfer_seeds[j].size(); ++i )
                    {
                        transfer_seeds[j][i] = new std::vector< space_point_t* >();
                    }
                }

            }

            void multi_modal_forward_tp_t::store_start_state()
            {
                PRX_PRINT("==========================", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT(" = Storing Start States = ", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("==========================", PRX_TEXT_LIGHTGRAY);
                //First, let's make sure we have the global start state saved.
                model->use_context("full_space");
                global_state_space = model->get_state_space();
                global_control_space = model->get_control_space();
                global_start_state = global_state_space->alloc_point();
                tree_sample_point = global_state_space->alloc_point();

                //First, let's increase the number of points
                tree.pre_alloc_memory< search_node_t, rrt_edge_t >(max_points);
                start_vertex = tree.add_vertex< search_node_t, rrt_edge_t > ();
                tree[start_vertex]->point = global_state_space->clone_point( global_start_state );
                tree.get_vertex_as< search_node_t >(start_vertex)->auto_state = NULL;
                tree_vis.push_back( start_vertex );
                // metric->add_point(tree[start_vertex]);
                point_number = 0;
                if( pre_alloced_points.empty() )
                {
                    for( unsigned i = 0; i < max_points; i++ )
                    {
                        pre_alloced_points.push_back( global_state_space->alloc_point() );
                    }
                }

                //--------
                //- Collect all of the spaces
                //--------
                for( unsigned i=0; i<manipulators.size(); ++i )
                {
                    model->use_context( move_context_names[i] );
                    move_spaces.push_back( model->get_state_space() );
                    model->use_context( transfer_context_names[i] );
                    transfer_spaces.push_back( model->get_state_space() );
                }

                //-----
                //- Save the safe states
                //-----
                for( unsigned i=0; i<manipulators.size(); ++i)
                {
                    //Allocate the point, this is the safe state.
                    safe_states.push_back( move_spaces[i]->alloc_point() );
                    PRX_DEBUG_COLOR("Safe " << i << ":: " << move_spaces[i]->print_point( safe_states.back(), 3 ), PRX_TEXT_LIGHTGRAY);
                }

                //--------
                //- Load the SIGs
                //--------
                //Switch over to the object context, since the SIGs read object points..
                model->use_context( object_context_name );
                //Since we should have already called find_plants() at this point, we can set up the SIG mapping
                for(unsigned i=0; i<manipulators.size(); ++i)
                {
                    SIG_map.push_back( manipulator_map[manipulators[i]->get_pathname()] );
                    // PRX_DEBUG_COLOR("[" << i << "]: " << SIG_map.back(), PRX_TEXT_LIGHTGRAY)
                }

                //Then, perform deserialization of the SIGs.
                //Deserialize the SIGs: first get the right file directories
                char* w = std::getenv("PRACSYS_PATH");
                std::string dir(w);
                dir += ("/prx_input/");
                std::string file = dir + stable_sig_filename;
                std::string file2 = dir + handoff_sig_filename;

                //Then, actually deserialize
                std::ifstream fin;
                if( !SIGs.deserialize< sig_node_t, sig_edge_t >(file, fin, model->get_state_space()) )
                {
                    PRX_FATAL_S("Error deserializing stable grasp SIG");
                }
                std::ifstream fin2;
                if( !SIGh.deserialize< sig_node_t, sig_edge_t >(file2, fin2, model->get_state_space()) )
                {
                    PRX_FATAL_S("Error deserializing handoff SIG");
                }

                //Now that the SIGs have already been read, we can set up the index mapping.
                SIGs_map.resize(manipulators.size());
                foreach(undirected_vertex_index_t vi, boost::vertices(SIGs.graph))
                {
                    //Get the actual node
                    undirected_node_t* node = SIGs[vi];
                    //Get it's node id
                    unsigned id = node->node_id;
                    //Now, need to search through our global SIG map
                    for(unsigned i=0; i<SIG_map.size(); ++i)
                    {
                        //If this manip index maps to the node id
                        if( SIG_map[i] == id )
                        {
                            //It's the one we have been looking for, set up the map
                            SIGs_map[i] = vi;
                        }
                    }
                }
                //Also need to set up SIGh
                SIGh_map.resize(manipulators.size());
                foreach(undirected_vertex_index_t vi, boost::vertices(SIGh.graph))
                {
                    //Get the actual node
                    undirected_node_t* node = SIGh[vi];
                    //Get it's node id
                    unsigned id = node->node_id;
                    //Now, need to search through our global SIG map
                    for(unsigned i=0; i<SIG_map.size(); ++i)
                    {
                        //If this manip index maps to the node id
                        if( SIG_map[i] == id )
                        {
                            //It's the one we have been looking for, set up the map
                            SIGh_map[i] = vi;
                        }
                    }
                }
            }

            void multi_modal_forward_tp_t::generate_seeds_from_automaton()
            {
                PRX_PRINT("======================", PRX_TEXT_BLUE);
                PRX_PRINT(" = Generating Seeds = ", PRX_TEXT_BLUE);
                PRX_PRINT("======================", PRX_TEXT_BLUE);
                //Go over each node in the high-level graph...
                for( unsigned i=0; i<all_vis.size(); ++i )
                {
                    //GET THE NODE
                    automaton_node_t* node = automaton.get_vertex_as<automaton_node_t>(all_vis[i]);
                    // PRX_DEBUG_COLOR("Finding seeds in node: " << node->node_id, PRX_TEXT_LIGHTGRAY);

                    //Let's get ALL of its records
                    const std::vector< pose_record_t >& records = node->get_records();

                    //For all of those valid records
                    for( unsigned k=0; k<node->get_num_records(); ++k )
                    {
                        //Let's concentrate on this record
                        const pose_record_t& record = records[k];

                        //For each one of its grasps
                        for( unsigned g=0; g<record.manipulator_indices.size(); ++g )
                        {
                            //Figure out the manipulator which has interesting information here
                            unsigned manip_index = record.manipulator_indices[g];
                            //Add the move seed
                            move_seeds[manip_index]->push_back( record.retracted[g] );
                            //Add the transfer seed
                            transfer_seeds[record.grasp_ids[g]][manip_index]->push_back( record.states[g] );
                        }
                    }
                }

                //Also going to have to go over every edge too and grab their seeds.
                for( unsigned i=0; i<edge_indices.size(); ++i )
                {
                    automaton_edge_t* edge = automaton.get_edge_as<automaton_edge_t>( edge_indices[i] );
                    const std::vector< pose_record_t >& records = edge->get_records();

                    //For all of those records
                    for( unsigned k=0; k<edge->get_num_records(); ++k )
                    {
                        //Get this record
                        const pose_record_t& record = records[k];

                        //For each of its grasps
                        for( unsigned g=0; g<record.manipulator_indices.size(); ++g )
                        {
                            //Get the index
                            unsigned manip_index = record.manipulator_indices[g];
                            //And add the seeds
                            move_seeds[manip_index]->push_back( record.retracted[g] );
                            transfer_seeds[record.grasp_ids[g]][manip_index]->push_back( record.states[g] );
                        }
                    }
                }
            }

            void multi_modal_forward_tp_t::setup_grasp_sampler()
            {
                PRX_PRINT("==================================", PRX_TEXT_MAGENTA);
                PRX_PRINT(" = Setting up the Grasp Sampler = ", PRX_TEXT_RED);
                PRX_PRINT("==================================", PRX_TEXT_MAGENTA);

                //Alright, we need to link ALL the things
                grasp_sampler->link_manipulators( manipulators );
                grasp_sampler->link_object_space( object_space );
                grasp_sampler->link_move_names( move_context_names );
                grasp_sampler->link_transfer_names( transfer_context_names );
                grasp_sampler->link_grasp_zs( grasp_zs );
                grasp_sampler->link_start_state( global_start_state );
                grasp_sampler->link_manipulation_sampler( manipulation_sampler );
                grasp_sampler->link_validity_checker( validity_checker );
                grasp_sampler->link_local_planner( local_planner );
                grasp_sampler->link_specifications( move_specifications );
                grasp_sampler->link_world_model( model );
                //Then, just to verify, verify
                grasp_sampler->verify();
            }

            void multi_modal_forward_tp_t::sample_low_level_states()
            {
                PRX_PRINT("===============================", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT(" = Sampling Low-Level States = ", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("===============================", PRX_TEXT_LIGHTGRAY);
                //We will need an object point to do all of our fancy pants sampling stuff
                space_point_t* object_pt = object_space->alloc_point();

                // = = = = = = = = = = = = =
                // = = = = = = = = = = = = =
                // = = = = = = = = = = = = =

                bool pose_success;
                unsigned successful_poses;

                unsigned grasp_failures = 0;
                unsigned total_attempts = 0;

                //Let us do the sampling o' th' Stable states
                for( unsigned i=0; i<stable_pose_vis.size(); ++i )
                {
                    PRX_DEBUG_COLOR("============================", PRX_TEXT_GREEN);
                    PRX_DEBUG_COLOR(" Processing stable pose : " << i, PRX_TEXT_GREEN);
                    PRX_DEBUG_COLOR("============================", PRX_TEXT_GREEN);

                    grasp_failures = 0;
                    total_attempts = 0;

                    //Get the node associated with this stable pose
                    automaton_node_t* node = automaton.get_vertex_as<automaton_node_t>( stable_pose_vis[i] );
                    //Retrieve the bounds
                    const std::vector< bounds_t >& bounds = node->get_bounds();

                    //Also, should find out what manipulator we are actually dealing with here
                    unsigned manip_index = node->get_arms()[0][0];

                    //For sanity's sake, put everyone at the start
                    go_to_start();

                    //First, we have had no success
                    successful_poses = 0;

                    //Let's try throwing samples until we get enough successful poses
                    while( successful_poses < number_of_samples )
                    {
                        //Begin by sampling a pose for the object given our bounds
                        object_pt->memory[0] = bounds[0].uniform_random_bounds();
                        object_pt->memory[1] = bounds[1].uniform_random_bounds();
                        object_pt->memory[2] = bounds[2].uniform_random_bounds();
                        object_pt->memory[3] = 0; object_pt->memory[4] = 0; object_pt->memory[5] = 0; object_pt->memory[6] = 1;

                        // PRX_DEBUG_COLOR("==========================================================", PRX_TEXT_CYAN);
                        // PRX_DEBUG_COLOR("Pose: " << object_pt->memory[0] << " " << object_pt->memory[1] << " " << object_pt->memory[2], PRX_TEXT_LIGHTGRAY);
                        // PRX_DEBUG_COLOR("==========================================================", PRX_TEXT_CYAN);

                        //Also assume this pose has failed
                        pose_success = false;

                        //For some number of different grasps
                        for(unsigned p=0; p<num_grasp_samples; ++p)
                        {
                            //For sanity's sake, put everyone at the start
                            go_to_start();

                            //Keep track of how well it goes
                            ++total_attempts;

                            //First, tell the grasp sampler what grasp to use
                            unsigned grasp_id = p%3;
                            manipulation_sampler->set_grasp_z( grasp_zs[grasp_id] );

                            // PRX_DEBUG_COLOR("Trying a grasp with id: " << grasp_id, PRX_TEXT_BLUE);

                            if( grasp_sampler->sample_grasp( manip_index, object_pt, grasp_retraction_distance ) )
                            {
                                // PRX_DEBUG_COLOR("SUCCEEDED", PRX_TEXT_GREEN);
                                //Alright, now get that grasp information
                                const grasp_data_t* grasp_data = grasp_sampler->get_first_grasp();

                                //We need to add a record to the node with the information we were able to discover.
                                unsigned record_index = node->add_record( object_space->clone_point( object_pt ), successful_poses );

                                //Because we found a successful grasp, report this pose as a success
                                pose_success = true;

                                //Now, for this case, this record only requires a single grasp.
                                node->add_grasp_to_record( record_index, manip_index, grasp_id, transfer_spaces[manip_index]->clone_point( grasp_data->grasped ), move_spaces[manip_index]->clone_point( grasp_data->released ), move_spaces[manip_index]->clone_point( grasp_data->retracted ), new plan_t( *(grasp_data->retract_plan) ), new plan_t( *(grasp_data->approach_plan) ) );
                            }
                            else
                            {
                                // PRX_DEBUG_COLOR("FAILED", PRX_TEXT_RED);
                                // PRX_DEBUG_COLOR("Sim state: " << global_state_space->print_memory(3), PRX_TEXT_LIGHTGRAY);
                                ++grasp_failures;
                            }
                        }
                        if( pose_success )
                        {
                            ++successful_poses;
                            // PRX_DEBUG_COLOR("^ Pose success!", PRX_TEXT_GREEN);
                        }
                    }
                    TEXT_COLOR color = PRX_TEXT_LIGHTGRAY;
                    double fail_rate = ((double)grasp_failures)/((double)total_attempts);
                    if( fail_rate >= 0.5 )
                        color = PRX_TEXT_BROWN;
                    if( fail_rate >= 0.99 )
                        color = PRX_TEXT_RED;
                    PRX_DEBUG_COLOR("Failed Samples: [" << grasp_failures << "/" << total_attempts << "]", color );
                }

                // = = = = = = = = = = = = =
                // = = = = = = = = = = = = =
                // = = = = = = = = = = = = =

                for( unsigned i=0; i<stable_pose_vis.size(); ++i )
                {
                    for( unsigned k=i+1; k<stable_pose_vis.size(); ++k )
                    {
                        grasp_failures = 0;
                        total_attempts = 0;

                        //Get the first node
                        automaton_node_t* first_node = automaton.get_vertex_as<automaton_node_t>( stable_pose_vis[i] );
                        automaton_node_t* second_node = automaton.get_vertex_as<automaton_node_t>( stable_pose_vis[k] );

                        //Get the edge between them
                        automaton_edge_t* edge = automaton.get_edge_as<automaton_edge_t>( stable_pose_vis[i], stable_pose_vis[k] );

                        //If there is no edge between them, then forget about it
                        if( edge == NULL )
                            continue;

                        PRX_DEBUG_COLOR("==================================", PRX_TEXT_BROWN);
                        PRX_DEBUG_COLOR(" Processing stable-stable : " << i << " - " << k, PRX_TEXT_BROWN);
                        PRX_DEBUG_COLOR("==================================", PRX_TEXT_BROWN);

                        //Again, for sanity's sake, put everyone at the start
                        go_to_start();

                        //Retrieve the bounds
                        const std::vector< bounds_t >& bounds = edge->get_bounds();

                        //Get the manipulator indices
                        unsigned first_index = first_node->get_arms()[0][0];
                        unsigned second_index = second_node->get_arms()[0][0];

                        //First, we have had no success
                        successful_poses = 0;

                        //Let's try throwing samples until we get enough successful poses
                        while( successful_poses < number_of_samples )
                        {
                            //Begin by sampling a pose for the object given our bounds
                            object_pt->memory[0] = bounds[0].uniform_random_bounds();
                            object_pt->memory[1] = bounds[1].uniform_random_bounds();
                            object_pt->memory[2] = bounds[2].uniform_random_bounds();
                            object_pt->memory[3] = 0; object_pt->memory[4] = 0; object_pt->memory[5] = 0; object_pt->memory[6] = 1;

                            // PRX_DEBUG_COLOR("Pose: " << object_pt->memory[0] << " " << object_pt->memory[1] << " " << object_pt->memory[2], PRX_TEXT_LIGHTGRAY);

                            //Put everyone at the start
                            go_to_start();

                            //Begin by assuming the pose fails
                            pose_success = false;

                            //For some number of different grasps
                            for(unsigned p=0; p<num_grasp_samples; ++p)
                            {
                                //Put everyone at the start
                                go_to_start();

                                //Keep track of how well it goes
                                ++total_attempts;

                                //First, tell the grasp sampler what grasp to use
                                unsigned grasp_id = p%3;
                                manipulation_sampler->set_grasp_z( grasp_zs[grasp_id] );

                                //Then, see if there is a valid grasp for the first arm
                                if( grasp_sampler->sample_grasp( first_index, object_pt, grasp_retraction_distance ) )
                                {
                                    //things are going well, swap role of primary and secondary grasp stuff
                                    grasp_sampler->swap_grasp_data();

                                    //Then, put the arms in the safe place
                                    go_to_start();

                                    //Then, use some grasp... don't need to alternate so much
                                    unsigned second_grasp_id = (p/2)%3;
                                    manipulation_sampler->set_grasp_z( grasp_zs[second_grasp_id] );

                                    set_zero_control();

                                    //Now, see if the other arm has a valid grasp
                                    if( grasp_sampler->sample_grasp( second_index, object_pt, grasp_retraction_distance ) )
                                    {
                                        //Both of the grasps are winners, so swap the data back
                                        grasp_sampler->swap_grasp_data();

                                        //Alright, now get the grasp informations
                                        const grasp_data_t* first_data = grasp_sampler->get_first_grasp();
                                        const grasp_data_t* second_data = grasp_sampler->get_second_grasp();

                                        //We need to add a record to the node with the information we were able to discover.
                                        unsigned record_index = edge->add_record( object_space->clone_point( object_pt ), successful_poses );

                                        //We succeeded!
                                        pose_success = true;

                                        //Now, for this case, this record only requires a single grasp.
                                        edge->add_grasp_to_record( record_index, first_index, grasp_id, transfer_spaces[first_index]->clone_point( first_data->grasped ), move_spaces[first_index]->clone_point( first_data->released ), move_spaces[first_index]->clone_point( first_data->retracted ), new plan_t( *(first_data->retract_plan) ), new plan_t( *(first_data->approach_plan) ) );
                                        edge->add_grasp_to_record( record_index, second_index, second_grasp_id, transfer_spaces[second_index]->clone_point( second_data->grasped ), move_spaces[second_index]->clone_point( second_data->released ), move_spaces[second_index]->clone_point( second_data->retracted ), new plan_t( *(second_data->retract_plan) ), new plan_t( *(second_data->approach_plan) ) );
                                    }
                                    else
                                    {
                                        ++grasp_failures;
                                    }
                                }
                                else
                                {
                                    ++grasp_failures;
                                }
                            }
                            if( pose_success )
                            {
                                ++successful_poses;
                                // PRX_DEBUG_COLOR("^ Pose success!", PRX_TEXT_GREEN);
                            }
                        }
                        TEXT_COLOR color = PRX_TEXT_LIGHTGRAY;
                        double fail_rate = ((double)grasp_failures)/((double)total_attempts);
                        if( fail_rate >= 0.5 )
                            color = PRX_TEXT_BROWN;
                        if( fail_rate >= 0.99 )
                            color = PRX_TEXT_RED;
                        PRX_DEBUG_COLOR("Failed Samples: [" << grasp_failures << "/" << total_attempts << "]", color );
                    }
                }

                // = = = = = = = = = = = = =
                // = = = = = = = = = = = = =
                // = = = = = = = = = = = = =

                //Let us do the sampling o' th' Handoff states
                for( unsigned i=0; i<handoff_pose_vis.size(); ++i )
                {
                    PRX_DEBUG_COLOR("==============================", PRX_TEXT_RED);
                    PRX_DEBUG_COLOR(" Processing Handoff state : " << i, PRX_TEXT_RED);
                    PRX_DEBUG_COLOR("==============================", PRX_TEXT_RED);

                    //Put everybody back at the start
                    go_to_start();

                    //Statistics
                    grasp_failures = 0;
                    total_attempts = 0;

                    //Get the node associated with this handoff pose
                    automaton_node_t* node = automaton.get_vertex_as<automaton_node_t>( handoff_pose_vis[i] );
                    //Retrieve the bounds
                    const std::vector< bounds_t >& bounds = node->get_bounds();
                    //Also, this is for k=1, get the arms we care about
                    unsigned manip_index = node->get_arms()[0][0];
                    unsigned second_index = node->get_arms()[1][0];

                    //First, we have had no success
                    successful_poses = 0;

                    //Let's try throwing samples until we get enough successful poses
                    while( successful_poses < number_of_samples )
                    {
                        //Begin by sampling a pose for the object given our bounds
                        object_pt->memory[0] = bounds[0].uniform_random_bounds();
                        object_pt->memory[1] = bounds[1].uniform_random_bounds();
                        object_pt->memory[2] = bounds[2].uniform_random_bounds();
                        object_pt->memory[3] = 0; object_pt->memory[4] = 0; object_pt->memory[5] = 0; object_pt->memory[6] = 1;

                        // PRX_DEBUG_COLOR("Pose: " << object_pt->memory[0] << " " << object_pt->memory[1] << " " << object_pt->memory[2], PRX_TEXT_LIGHTGRAY);

                        //Put everyone at the start
                        go_to_start();

                        //This pose fails until proven otherwise
                        pose_success = false;

                        //Then, for as many different kinds of grasps we want to try
                        for(unsigned p=0; p<num_grasp_samples; ++p)
                        {
                            //Put everyone at the start
                            go_to_start();

                            //Keep track of how well things are going
                            ++total_attempts;

                            if( grasp_sampler->sample_handoff( manip_index, second_index, p, object_pt, grasp_retraction_distance ) )
                            {
                                //GREATEST SUCCESS!!!!
                                //Alright, now get that grasp information
                                const grasp_data_t* first_data = grasp_sampler->get_first_grasp();
                                const grasp_data_t* second_data = grasp_sampler->get_second_grasp();

                                //We need to add a record to the node with the information we were able to discover.
                                unsigned record_index = node->add_record( object_space->clone_point( object_pt ), successful_poses );

                                //Indeed, we are successful
                                pose_success = true;

                                //Then, add both grasps to the record.
                                node->add_grasp_to_record( record_index, manip_index, first_data->grasp_id, transfer_spaces[manip_index]->clone_point( first_data->grasped ), move_spaces[manip_index]->clone_point( first_data->released ), move_spaces[manip_index]->clone_point( first_data->retracted ), new plan_t( *(first_data->retract_plan) ), new plan_t( *(first_data->approach_plan) ) );
                                node->add_grasp_to_record( record_index, second_index, second_data->grasp_id, transfer_spaces[manip_index]->clone_point( second_data->grasped ), move_spaces[manip_index]->clone_point( second_data->released ), move_spaces[manip_index]->clone_point( second_data->retracted ), new plan_t( *(second_data->retract_plan) ), new plan_t( *(second_data->approach_plan) ) );

                                //Print what we got: DEBUG
                                // if( i == 4 && successful_poses == 1 )
                                // {
                                //     PRX_DEBUG_COLOR("Generated a handoff:", PRX_TEXT_RED);
                                //     PRX_DEBUG_COLOR("Object: " << object_space->print_point( object_pt, 3 ), PRX_TEXT_CYAN );
                                //     PRX_DEBUG_COLOR("[" << manip_index << "]: Grasp: " << transfer_spaces[manip_index]->print_point( first_data->grasped, 3 ), PRX_TEXT_LIGHTGRAY );
                                //     PRX_DEBUG_COLOR("[" << manip_index << "]: Release: " << move_spaces[manip_index]->print_point( first_data->released, 3 ), PRX_TEXT_LIGHTGRAY );
                                //     PRX_DEBUG_COLOR("[" << manip_index << "]: Retract: " << move_spaces[manip_index]->print_point( first_data->retracted, 3 ), PRX_TEXT_LIGHTGRAY );
                                //     PRX_DEBUG_COLOR("[" << second_index << "]: Grasp: " << transfer_spaces[second_index]->print_point( second_data->grasped, 3 ), PRX_TEXT_LIGHTGRAY );
                                //     PRX_DEBUG_COLOR("[" << second_index << "]: Release: " << move_spaces[second_index]->print_point( second_data->released, 3 ), PRX_TEXT_LIGHTGRAY );
                                //     PRX_DEBUG_COLOR("[" << second_index << "]: Retract: " << move_spaces[second_index]->print_point( second_data->retracted, 3 ), PRX_TEXT_LIGHTGRAY );
                                // }
                            }
                            else
                            {
                                ++grasp_failures;
                            }
                        }
                        if( pose_success )
                        {
                            ++successful_poses;
                            // PRX_DEBUG_COLOR("^ Pose success!", PRX_TEXT_GREEN);
                        }
                    }
                    TEXT_COLOR color = PRX_TEXT_LIGHTGRAY;
                    double fail_rate = ((double)grasp_failures)/((double)total_attempts);
                    if( fail_rate >= 0.5 )
                        color = PRX_TEXT_BROWN;
                    if( fail_rate >= 0.99 )
                        color = PRX_TEXT_RED;

                    PRX_DEBUG_COLOR("Failed Samples: [" << grasp_failures << "/" << total_attempts << "]", color );
                }

                //Again at this stage, just go ahead and serialize
                serialize_automaton();
            }

            void multi_modal_forward_tp_t::connect_query_points()
            {
                PRX_PRINT("==========================", PRX_TEXT_CYAN);
                PRX_PRINT(" = Connect Query Points = ", PRX_TEXT_BLUE);
                PRX_PRINT("==========================", PRX_TEXT_CYAN);

                //First, we need to connect up the known start
                global_state_space->copy_from_point( global_start_state );
                //Get the object pose from of the start
                object_space->copy_to_point( object_state );

                unsigned approaches = 0;
                //For each stable pose state, try to do this
                for( unsigned s=0; s<stable_pose_vis.size(); ++s )
                {
                    PRX_DEBUG_COLOR("Attempting connection to stable node :: " << s, PRX_TEXT_CYAN );
                    //Go get the node
                    automaton_node_t* node = automaton.get_vertex_as< automaton_node_t >( stable_pose_vis[s] );
                    //Since we know we're doing k=1, just get the arm
                    unsigned manip_index = node->get_arms()[0][0];

                    for( unsigned i=0; i<approach_attempts; ++i )
                    {
                        //Also, let's come up with a grasp id
                        unsigned grasp_id = i%3;
                        //And set up the manipulation sampler to use the right grasp z
                        manipulation_sampler->set_grasp_z( grasp_zs[grasp_id] );

                        //For sanity's sake, just always go to the start... we are just trying to sample
                        go_to_start();

                        if( grasp_sampler->sample_grasp( manip_index, object_state, grasp_retraction_distance ) )
                        {
                            ++approaches;

                            //Alright, now get that grasp information
                            const grasp_data_t* grasp_data = grasp_sampler->get_first_grasp();

                            // PRX_DEBUG_COLOR("Found approach grasp  : " << transfer_spaces[manip_index]->print_point( grasp_data->grasped, 3 ), PRX_TEXT_RED );
                            // PRX_DEBUG_COLOR("Found approach release: " << move_spaces[manip_index]->print_point( grasp_data->released, 3 ), PRX_TEXT_BROWN );
                            // PRX_DEBUG_COLOR("Found approach retract: " << move_spaces[manip_index]->print_point( grasp_data->retracted, 3 ), PRX_TEXT_GREEN );

                            //We need to add a record to the node with the information we were able to discover.
                            unsigned record_index = node->add_record( object_space->clone_point( object_state ), PRX_INFINITY ); //TODO: Make this not dumb

                            //Now, for this case, this record only requires a single grasp.
                            node->add_grasp_to_record( record_index, manip_index, grasp_id, transfer_spaces[manip_index]->clone_point( grasp_data->grasped ), move_spaces[manip_index]->clone_point( grasp_data->released ), move_spaces[manip_index]->clone_point( grasp_data->retracted ), new plan_t( *(grasp_data->retract_plan) ), new plan_t( *(grasp_data->approach_plan) ) );

                            //Now, get that record
                            const pose_record_t& record = node->get_records()[record_index];
                            //And add seeds for PRM: NOTE: we know there is only one grasp in this record, so we are okay to just index with 0
                            move_seeds[manip_index]->push_back( record.retracted[0] );
                            transfer_seeds[grasp_id][manip_index]->push_back( record.states[0] );
                            // PRX_DEBUG_COLOR("Added transfer seed: [" << grasp_id << "][" << manip_index << "]", PRX_TEXT_LIGHTGRAY);
                        }
                    }
                }
                PRX_DEBUG_COLOR("Found " << approaches << " approach grasps.", PRX_TEXT_GREEN);
                if( approaches == 0 )
                {
                    PRX_FATAL_S("Unable to connect to the starting location");
                }

                //Get the query in motion planning query form
                motion_planning_query_t* mp_query = dynamic_cast<motion_planning_query_t*>(input_query);
                PRX_ASSERT(mp_query != NULL);
                //PRX oddity: need to link the space?
                mp_query->get_goal()->link_space( global_state_space );
                //First, we need to connect up the goal
                global_state_space->copy_from_point( mp_query->get_goal()->get_goal_points()[0] );
                //Get the object pose from of the start
                object_space->copy_to_point( object_state );
                object_space->copy_to_point( object_goal_state );

                approaches = 0;
                //For each stable pose state, try to do this
                for( unsigned s=0; s<stable_pose_vis.size(); ++s )
                {
                    PRX_DEBUG_COLOR("Attempting connection to stable node :: " << s, PRX_TEXT_CYAN );
                    //Go get the node
                    automaton_node_t* node = automaton.get_vertex_as< automaton_node_t >( stable_pose_vis[s] );
                    //Since we know we're doing k=1, just get the arm
                    unsigned manip_index = node->get_arms()[0][0];

                    bool is_goal = false;
                    for( unsigned i=0; i<approach_attempts; ++i )
                    {
                        //Also, let's come up with a grasp id
                        unsigned grasp_id = i%3;
                        //And set up the manipulation sampler to use the right grasp z
                        manipulation_sampler->set_grasp_z( grasp_zs[grasp_id] );

                        //For sanity's sake, just always go to the start... we are just trying to sample
                        go_to_start();

                        if( grasp_sampler->sample_grasp( manip_index, object_state, grasp_retraction_distance ) )
                        {
                            ++approaches;
                            is_goal = true;

                            //Alright, now get that grasp information
                            const grasp_data_t* grasp_data = grasp_sampler->get_first_grasp();

                            //We need to add a record to the node with the information we were able to discover.
                            unsigned record_index = node->add_record( object_space->clone_point( object_state ), PRX_INFINITY+1 ); //TODO: Make this not dumb

                            //Now, for this case, this record only requires a single grasp.
                            node->add_grasp_to_record( record_index, manip_index, grasp_id, transfer_spaces[manip_index]->clone_point( grasp_data->grasped ), move_spaces[manip_index]->clone_point( grasp_data->released ), move_spaces[manip_index]->clone_point( grasp_data->retracted ), new plan_t( *(grasp_data->retract_plan) ), new plan_t( *(grasp_data->approach_plan) ) );

                            //Now, get that record
                            const pose_record_t& record = node->get_records()[record_index];
                            //And add seeds for PRM: NOTE: we know there is only one grasp in this record, so we are okay to just index with 0
                            move_seeds[manip_index]->push_back( record.retracted[0] );
                            transfer_seeds[grasp_id][manip_index]->push_back( record.states[0] );
                            // PRX_DEBUG_COLOR("Added transfer seed: [" << grasp_id << "][" << manip_index << "]", PRX_TEXT_LIGHTGRAY);
                        }
                    }
                    if( is_goal )
                    {
                        goal_vertices.push_back( stable_pose_vis[s] );
                    }
                }
                PRX_DEBUG_COLOR("Found " << approaches << " departure grasps.", PRX_TEXT_GREEN);
                if( approaches == 0 )
                {
                    PRX_FATAL_S("Unable to connect to the goal location");
                }

                //Alright, before adding seeds, we should be able to compute some Heurisitics
                compute_heuristics();

                //Then, add all the seeds we came up with to the roadmap
                add_seeds_to_roadmaps();
            }

            void multi_modal_forward_tp_t::add_seeds_to_roadmaps()
            {
                PRX_DEBUG_COLOR("Adding seeds to PRM* graphs...", PRX_TEXT_RED);
                //Now, since we have added new seeds, need PRM* to connect these up.
                for( unsigned i=0; i<move_planners.size(); ++i )
                {
                    PRX_DEBUG_COLOR("Move: " << i, PRX_TEXT_CYAN);
                    model->use_context( move_context_names[i] );
                    move_specifications[i]->set_seeds( *move_seeds[i] );
                    go_to_start();
                    move_planners[i]->setup();
                }
                for( unsigned j=0; j<transfer_planners.size(); ++j )
                {
                    for( unsigned i=0; i<transfer_planners[j].size(); ++i )
                    {
                        PRX_DEBUG_COLOR("[" << j << "] Transfer: " << i, PRX_TEXT_MAGENTA);
                        model->use_context( transfer_context_names[i] );
                        transfer_specifications[j][i]->set_seeds( *transfer_seeds[j][i] );
                        go_to_start();
                        transfer_planners[j][i]->setup();
                    }
                }
            }

            void multi_modal_forward_tp_t::construct_automaton()
            {
                PRX_PRINT("=====================================", PRX_TEXT_RED);
                PRX_PRINT("=  Generating the Automaton! (" << manipulators.size() << " " << object_k << ")  =", PRX_TEXT_BROWN);
                PRX_PRINT("=====================================", PRX_TEXT_RED);

                //We need to know how many stable states there are here
                unsigned nr_stable_states = combination( manipulators.size(), object_k );
                //PRX_DEBUG_COLOR("Nr states: " << nr_stable_states, PRX_TEXT_BROWN);

                //Let's get some combinations
                std::vector< unsigned > arm_set;
                arm_set.assign( manipulators.size(), 1 );
                std::vector< std::vector< unsigned > > arm_combos = generate_combinations(object_k, arm_set);

                //Generate the stable pose states
                for( unsigned i=0; i<nr_stable_states; ++i )
                {
                    //Set up some bounds for tracking interaction
                    std::vector<bounds_t> bounds;
                    //Now, we have to tell the node what arms it is responsible for
                    arm_set.clear();
                    //Generate the arm set corresponding to this combination
                    for(unsigned k=0; k<arm_combos[i].size(); ++k)
                    {
                        if(arm_combos[i][k] == 1)
                            arm_set.push_back(k);
                    }
                    //Begin by checking if this arm combination can even interact
                    if( have_interaction( arm_set, bounds, SIGs, SIGs_map ) )
                    {
                        //First, actually add the node
                        stable_pose_vis.push_back( automaton.add_vertex<automaton_node_t>() );
                        all_vis.push_back( stable_pose_vis.back() );
                        //Get the node which corresponds to this stable pose
                        automaton_node_t* node = automaton.get_vertex_as<automaton_node_t>(stable_pose_vis.back());
                        //Ensure we link the spaces to this node
                        node->link_spaces( move_spaces, transfer_spaces, object_space );
                        //Then, also get the bounds into this node for sampling
                        node->set_bounds( bounds );
                        //Then, tell the arm that this is its arm set
                        node->add_arm_set( arm_set );
                        //Then, let's just make the edge
                        undirected_edge_index_t ei = automaton.add_edge<automaton_edge_t>( stable_pose_vis.back(), stable_pose_vis.back(), 0 );
                        edge_indices.push_back(ei);
                        //Now, let's get it as an automaton edge so we can access our pretty pretty functions
                        automaton_edge_t* edge = automaton.get_edge_as<automaton_edge_t>( ei );
                        //Make sure to link in the spaces to the edge to so no crashy
                        edge->link_spaces( move_spaces, transfer_spaces, object_space );
                    }
                }
                //Actual number of generated stable poses may be even less, so reduce
                nr_stable_states = stable_pose_vis.size();
                //Then, for each stable pose state
                for( unsigned i=0; i<nr_stable_states; ++i )
                {
                    //For every other stable pose state
                    for( unsigned j=i+1; j<nr_stable_states; ++j)
                    {
                        //Some bounds for the THINGS
                        std::vector< bounds_t > bounds;

                        //Let's get the vertex indices
                        const undirected_vertex_index_t& u = stable_pose_vis[i];
                        const undirected_vertex_index_t& v = stable_pose_vis[j];
                        //And their corresponding nodes
                        automaton_node_t* node_u = automaton.get_vertex_as<automaton_node_t>(u);
                        automaton_node_t* node_v = automaton.get_vertex_as<automaton_node_t>(v);
                        //If poses share an edge in SIG_s
                        if( shared_surface(u, v, bounds) )
                        {
                            //Add an edge between the stable pose states
                            undirected_edge_index_t ei = automaton.add_edge<automaton_edge_t>( u, v, 0 );
                            edge_indices.push_back(ei);
                            //The edge planning mode should be all move
                            automaton_edge_t* edge = automaton.get_edge_as<automaton_edge_t>( ei );
                            //Link in the spaces
                            edge->link_spaces( move_spaces, transfer_spaces, object_space );
                            //Also get the bounds into this edge
                            edge->set_bounds( bounds );
                        }
                        //If poses share an edge in SIG_h
                        if( shared_space(u, v, bounds) )
                        {
                            //Generate a hand-off state
                            handoff_pose_vis.push_back( automaton.add_vertex<automaton_node_t>() );
                            all_vis.push_back( handoff_pose_vis.back() );
                            //Make sure to put the right information into this thing
                            automaton_node_t* node = automaton.get_vertex_as<automaton_node_t>(handoff_pose_vis.back());
                            //Then, link the spaces to this node
                            node->link_spaces( move_spaces, transfer_spaces, object_space );
                            //Also add the sampling bounds
                            node->set_bounds( bounds );
                            //And add the arm sets
                            node->add_arm_set( node_u->get_arms()[0] );
                            node->add_arm_set( node_v->get_arms()[0] );
                            //Add edges from stable poses to this handoff pose
                            undirected_edge_index_t ei_u = automaton.add_edge<automaton_edge_t>( u, handoff_pose_vis.back(), 0 );
                            edge_indices.push_back(ei_u);
                            undirected_edge_index_t ei_v = automaton.add_edge<automaton_edge_t>( v, handoff_pose_vis.back(), 0 );
                            edge_indices.push_back(ei_v);
                            //First the edge to u
                            automaton_edge_t* edge_u = automaton.get_edge_as<automaton_edge_t>(ei_u);
                            //Link 'dem spaces
                            edge_u->link_spaces( move_spaces, transfer_spaces, object_space );
                            //First the edge to v
                            automaton_edge_t* edge_v = automaton.get_edge_as<automaton_edge_t>(ei_v);
                            //Link spaces
                            edge_v->link_spaces( move_spaces, transfer_spaces, object_space );
                        }
                    }
                }
                //Now, for each handoff state
                for( unsigned i=0; i<handoff_pose_vis.size(); ++i )
                {
                    //Get the vertex index for this handoff state
                    const undirected_vertex_index_t& x = handoff_pose_vis[i];
                    automaton_node_t* node_x = automaton.get_vertex_as< automaton_node_t >( x );
                    //Add the two self transitions for "aerial" regrasps
                    undirected_edge_index_t s_ei = automaton.add_edge<automaton_edge_t>( x, x, 0 );
                    edge_indices.push_back(s_ei);
                    automaton_edge_t* self_edge = automaton.get_edge_as<automaton_edge_t>(s_ei);
                    self_edge->link_spaces( move_spaces, transfer_spaces, object_space );

                    //For each other handoff state
                    for( unsigned j=i+1; j<handoff_pose_vis.size(); ++j  )
                    {
                        //Get the nodes which correspond to these vertex indices
                        const undirected_vertex_index_t y = handoff_pose_vis[j];
                        automaton_node_t* node_y = automaton.get_vertex_as< automaton_node_t >( y );
                        //If the states have common arms
                        if( have_common_arms( node_x, node_y ) )
                        {
                            //Add an edge between these arms
                            undirected_edge_index_t ei = automaton.add_edge<automaton_edge_t>( x, y, 0 );
                            edge_indices.push_back(ei);
                            automaton_edge_t* edge = automaton.get_edge_as<automaton_edge_t>(ei);
                            //Link spaces
                            edge->link_spaces( move_spaces, transfer_spaces, object_space );
                        }
                    }
                }

                //At each point in the process, serialize the automaton so as to at least keep partial work
                serialize_automaton();
            }

            void multi_modal_forward_tp_t::post_process_graphs()
            {
                PRX_PRINT("============================", PRX_TEXT_CYAN);
                PRX_PRINT(" = Post-Processing Graphs = ", PRX_TEXT_LIGHTGRAY);
                PRX_PRINT("============================", PRX_TEXT_CYAN);

                // ===================================================
                // =======...=======...=======....====================
                // ======....======.===.====...==...==================
                // ========..=======.=======..====..==================
                // ========..=======..===.=======...==================
                // ========..=====..==.=..=====...====================
                // ========..=====..===..====...======================
                // =====........===.....=.==........==================
                // ===================================================

                PRX_DEBUG_COLOR("\n\n\n PROCESSING CASES 1&2: STABLE SELF-TRANSITION \n\n", PRX_TEXT_GREEN);

                //Have to process each and every valid stable pose
                for( unsigned i=0; i<stable_pose_vis.size(); ++i )
                {
                    //Some variables for computing the average path length
                    double regrasp_length = 0.0;
                    unsigned regrasp_paths = 0;
                    double transfer_length = 0.0;
                    unsigned transfer_paths = 0;
                    //Let's begin by gettan' node
                    automaton_node_t* node = automaton.get_vertex_as<automaton_node_t>(stable_pose_vis[i]);
                    //Now get its records
                    const std::vector< pose_record_t >& records = node->get_records();
                    unsigned nr_records = node->get_num_records();
                    //For each such record
                    for( unsigned r=0; r<nr_records; ++r )
                    {
                        const pose_record_t& first_record = records[r];
                        //For each OTHER record
                        for( unsigned t=r+1; t<nr_records; ++t )
                        {
                            const pose_record_t& second_record = records[t];

                            //Make sure to put all of the manipulators in the safe state.
                            go_to_start();

                            // - - - - - - - - -
                            // - Regrasps
                            // - - - - - - - - -
                            //If they have the same object pose
                            if( first_record.pose_id == second_record.pose_id )
                            {
                                //AND they also use different grasps && TODO: Do we have a better way of doing this?
                                if( !is_same_grasp( first_record.manipulator_indices[0], first_record, second_record ) )
                                {
                                    //Then move the object to the pose location
                                    set_object_pose_from_record( first_record );
                                    //Then, for each manipulator which is part of this stable grasp state
                                    for( unsigned n=0; n<first_record.manipulator_indices.size(); ++n )
                                    {
                                        //Get the manipulator's index
                                        unsigned manip_index = first_record.manipulator_indices[n];
                                        //Then, make a plan for this manipulator
                                        if( plan_move( manip_index, first_record.retracted[n], second_record.retracted[n] ) )
                                        {
                                            //And if the plan succeeded, make it part of the average.
                                            regrasp_length += saved_plan.length();
                                            ++regrasp_paths;
                                        }
                                    }
                                }
                            }
                            // - - - - - - - - -
                            // - Re-Placing
                            // - - - - - - - - -
                            //If they come from different object poses
                            else
                            {
                                //AND, they also use the same grasps && TODO: Do we have a better way of doing this?
                                if( is_same_grasp( first_record.manipulator_indices[0], first_record, second_record ) )
                                {
                                    //Set the object pose to be where the first state is
                                    set_object_pose_from_record( first_record );
                                    //Then, for each manipulator which is part of this stable grasp state
                                    for( unsigned n=0; n<first_record.manipulator_indices.size(); ++n )
                                    {
                                        //Get the manipulator's index
                                        unsigned manip_index = first_record.manipulator_indices[n];
                                        //Then, make a plan for this manipulator
                                        if( plan_transfer( manip_index, first_record.grasp_ids[n], first_record.states[n], second_record.states[n] ) )
                                        {
                                            //And if the plan succeeded, make it part of the average.
                                            transfer_length += saved_plan.length();
                                            ++transfer_paths;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    //Now, for this stable pose, we have the averages we were seeking
                    regrasp_length = regrasp_length / ((double)regrasp_paths);
                    transfer_length = transfer_length / ((double)transfer_paths);
                    PRX_DEBUG_COLOR("Computed Stable Pose (" << i << ") self-transition average path costs: ", PRX_TEXT_GREEN);
                    PRX_DEBUG_COLOR("Regrasp length: " << regrasp_length << "   [" << regrasp_paths << "]", PRX_TEXT_LIGHTGRAY);
                    PRX_DEBUG_COLOR("Transfer length: " << transfer_length << "   [" << transfer_paths << "]", PRX_TEXT_LIGHTGRAY);

                    //First, get the edge so we can put the thing with the thing.
                    automaton_edge_t* edge = automaton.get_edge_as<automaton_edge_t>( stable_pose_vis[i], stable_pose_vis[i] );
                    if( transfer_paths == 0 )
                        edge->set_weight( PRX_INFINITY );
                    else
                        edge->set_weight( PRX_MAXIMUM(regrasp_length, transfer_length) );
                }

                // ===================================================
                // ======......=======================================
                // =====...===..======================================
                // ===========..======================================
                // =========...=======================================
                // ===========..======================================
                // =====...===..======================================
                // ======......=======================================
                // ===================================================

                PRX_DEBUG_COLOR("\n\n\n PROCESSING CASE 3: STABLE-STABLE \n\n", PRX_TEXT_CYAN);

                // - Case 3: Stable pose to Stable pose
                //For each stable pose
                for( unsigned i=0; i<stable_pose_vis.size(); ++i )
                {
                    //Grab the node for future checks
                    automaton_node_t* first_node = automaton.get_vertex_as<automaton_node_t>(stable_pose_vis[i]);
                    //For each other stable pose
                    for( unsigned j=i+1; j<stable_pose_vis.size(); ++j )
                    {
                        //Get some variables here
                        double stable_stable_length = 0.0;
                        unsigned stable_stable_paths = 0;
                        //Get this node as well
                        automaton_node_t* second_node = automaton.get_vertex_as<automaton_node_t>(stable_pose_vis[j]);
                        //If they share an edge
                        automaton_edge_t* common_edge = automaton.get_edge_as<automaton_edge_t>( stable_pose_vis[i], stable_pose_vis[j] );
                        if( common_edge != NULL )
                        {
                            //Then we need to test paths which are on the records
                            const std::vector< pose_record_t >& records = common_edge->get_records();
                            unsigned nr_records = records.size();
                            //For each such record
                            for( unsigned r=0; r<nr_records; ++r )
                            {
                                //Get the storage
                                const pose_record_t& record = records[r];
                                //Get some variables heah: first assume there are no paths
                                std::vector< double > path_lengths;
                                //First, assume the problems have no solution
                                path_lengths.assign( record.manipulator_indices.size(), PRX_INFINITY+1 );
                                //Keep track of if any of the paths failed
                                bool failed = false;
                                //Then, for each grasp in the record
                                for( unsigned t=0; t<record.manipulator_indices.size() && !failed; ++t )
                                {
                                    //Make sure to put the manipulators in their start positions
                                    go_to_start();
                                    //Then, make sure that the object is placed where we expect to be reaching it
                                    set_object_pose_from_record( record );
                                    //What manipulator is it?
                                    unsigned manip_index = record.manipulator_indices[t];
                                    //If it is not a common manipulator, plan is from safe to the retracted position
                                    if( !index_is_common_manipulator( manip_index, first_node->get_arms()[0], second_node->get_arms()[0] ) )
                                    {
                                        //Run a movement plan.
                                        if( plan_move( manip_index, safe_states[manip_index], record.retracted[t] ) )
                                        {
                                            //If it succeeded, store the length
                                            path_lengths[t] = saved_plan.length();
                                        }
                                        else
                                        {
                                            //Otherwise the plan failed, which means this grasp doesn't work, sadly
                                            failed = true;
                                        }
                                    }
                                    //It is a common manipulator, so we must plan a regrasp for it... o_O
                                    else
                                    {
                                        //For now, let's just assume it regrasps the same point, and thus does not move
                                        //TODO: Actually compare against all other records here with the same pose and different grasps.
                                        path_lengths[t] = 0;
                                    }
                                }
                                //Now, if it failed at any point, we shall ignore this record, otherwise
                                if( !failed )
                                {
                                    //Find the maximum lenth path
                                    double max_length = 0.0;
                                    for( unsigned q=0; q<path_lengths.size(); ++q )
                                    {
                                        if( path_lengths[q] > max_length )
                                        {
                                            max_length = path_lengths[q];
                                        }
                                    }
                                    //Then, this maximum length must be added to the average...
                                    stable_stable_length += max_length;
                                    ++stable_stable_paths;
                                }
                            }
                            stable_stable_length = stable_stable_length/((double)stable_stable_paths);
                            //Let's actually get this average
                            PRX_DEBUG_COLOR("Computed Stable-Stable average (" << i << ")<->(" << j << "): ", PRX_TEXT_CYAN);
                            PRX_DEBUG_COLOR(":: " << stable_stable_length << "   [" << stable_stable_paths << "]", PRX_TEXT_LIGHTGRAY);
                            if( stable_stable_paths == 0 )
                                common_edge->set_weight( PRX_INFINITY );
                            else
                                common_edge->set_weight( stable_stable_length );
                        }
                    }
                }

                // ===================================================
                // =====..====..======================================
                // =====..====..======================================
                // =====..====..======================================
                // =====........======================================
                // ===========..======================================
                // ===========..======================================
                // ===========..======================================
                // ===================================================

                PRX_DEBUG_COLOR("\n\n\n PROCESSING CASE 4: STABLE-HANDOFF \n\n", PRX_TEXT_BROWN);

                //For each stable pose
                for( unsigned i=0; i<stable_pose_vis.size(); ++i )
                {
                    //Get this node
                    automaton_node_t* first_node = automaton.get_vertex_as<automaton_node_t>( stable_pose_vis[i] );
                    //For each handoff state
                    for( unsigned j=0; j<handoff_pose_vis.size(); ++j )
                    {
                        //Get this node as well
                        automaton_node_t* second_node = automaton.get_vertex_as<automaton_node_t>( handoff_pose_vis[j] );
                        //If there is an edge between this stable state and the handoff
                        automaton_edge_t* edge = automaton.get_edge_as<automaton_edge_t>( stable_pose_vis[i], handoff_pose_vis[j] );

                        ////Starting here, we are considering only a single edge: between a single stable state and handoff state

                        if( edge != NULL )
                        {
                            // PRX_DEBUG_COLOR("Processing: " << first_node->node_id << " <-> " << second_node->node_id, PRX_TEXT_BROWN);

                            //We have to do transfer planning for all arms which are part of the stable pose, and move planning for the others.
                            const std::vector< unsigned >& transfer_arms = first_node->get_arms()[0];
                            std::vector< unsigned > move_arms;
                            arm_union_setminus( move_arms, second_node->get_arms(), transfer_arms );

                            //Figure out how many records are on each high-level node
                            unsigned nr_first_records = first_node->get_num_records();
                            unsigned nr_second_records = second_node->get_num_records();

                            //Then retrieve the record data
                            const std::vector< pose_record_t >& first_records = first_node->get_records();
                            const std::vector< pose_record_t >& second_records = second_node->get_records();

                            //TODO: we'll have to be super special awesome for k>1
                            //First, get what should be the single transfer arm
                            unsigned first_manip = transfer_arms[0];
                            //Then, the move arms should just be one arm as well
                            unsigned second_manip = move_arms[0];

                            //Some variables for remembering the things
                            double first_leg_length = 0.0;
                            double second_leg_length = 0.0;

                            double stable_handoff_length = 0.0;
                            unsigned stable_handoff_paths = 0;

                            // PRX_DEBUG_COLOR("Transfer Arm: " << transfer_arms[0], PRX_TEXT_LIGHTGRAY);
                            // PRX_DEBUG_COLOR("Movement Arm: " << move_arms[0], PRX_TEXT_LIGHTGRAY);

                            //Need to do some processing for each pair of records
                            for( unsigned r=0; r<nr_first_records; ++r )
                            {
                                const pose_record_t& first_record = first_records[r];

                                //FIRST, we should find out which grasp in the first record the first manipulator corresponds to
                                unsigned first_transfer_index = get_grasp_index(first_manip, first_record);

                                for( unsigned t=0; t<nr_second_records; ++t )
                                {
                                    const pose_record_t& second_record = second_records[t];

                                    //Figure out appropriate indices
                                    unsigned second_transfer_index = get_grasp_index( first_manip, second_record );
                                    unsigned second_move_index = get_grasp_index( second_manip, second_record );

                                    //Put everything in place
                                    go_to_start();
                                    set_object_pose_from_record( first_record );

                                    //Then, if this record uses the same grasp for the transfer arm
                                    if( first_record.grasp_ids[first_transfer_index] == second_record.grasp_ids[second_transfer_index] )
                                    {
                                        //Try the transfer
                                        if( plan_transfer( first_manip, first_record.grasp_ids[first_transfer_index], first_record.states[first_transfer_index], second_record.states[second_transfer_index] ) )
                                        {
                                            //If there is a plan here, temporarily hold on to its length.
                                            first_leg_length = saved_plan.length();

                                            //Also, friggin'... put the first manipulator where it's planned to, and make sure people don't move
                                            transfer_spaces[ first_manip ]->copy_from_point( second_record.states[second_transfer_index] );
                                            set_zero_control();

                                            //Then, try the move
                                            if( plan_move( second_manip, safe_states[second_manip], second_record.released[second_move_index] ) )
                                            {
                                                //Store this path's length
                                                second_leg_length = saved_plan.length();

                                                //Now, this will actually count towards our average.
                                                stable_handoff_length += PRX_MAXIMUM( second_leg_length, first_leg_length );
                                                ++stable_handoff_paths;
                                            }
                                        }
                                    }
                                }
                            }

                            //Alright, so now, we're done getting the value over each pair of records between these two states which meet the criteria.  Average
                            stable_handoff_length = stable_handoff_length / ((double)stable_handoff_paths);
                            PRX_DEBUG_COLOR("Computed Stable-Handoff Average: (" << first_node->node_id << " <-> " << second_node->node_id << "): ", PRX_TEXT_BROWN );
                            PRX_DEBUG_COLOR(":: " << stable_handoff_length << "    [" << stable_handoff_paths << "]", PRX_TEXT_LIGHTGRAY);
                            if( stable_handoff_paths == 0 )
                                edge->set_weight( PRX_INFINITY );
                            else
                                edge->set_weight( stable_handoff_length );
                        }
                    }
                }

                // ===================================================
                // =====........======================================
                // =====..============================================
                // =====..============================================
                // =====.......=======================================
                // ===========..======================================
                // =====..====..======================================
                // ======......=======================================
                // ===================================================

                PRX_DEBUG_COLOR("\n\n\n PROCESSING CASE 5: HANDOFF-HANDOFF \n\n", PRX_TEXT_RED);

                //For each Handoff state
                for( unsigned i=0; i<handoff_pose_vis.size(); ++i )
                {
                    //Get this node
                    automaton_node_t* first_node = automaton.get_vertex_as<automaton_node_t>( handoff_pose_vis[i] );
                    //For each other Handoff state
                    for( unsigned j=i+1; j<handoff_pose_vis.size(); ++j )
                    {
                        //Get this node as well
                        automaton_node_t* second_node = automaton.get_vertex_as<automaton_node_t>( handoff_pose_vis[j] );
                        //If there is an edge between this stable state and the handoff
                        automaton_edge_t* edge = automaton.get_edge_as<automaton_edge_t>( handoff_pose_vis[i], handoff_pose_vis[j] );

                        ////Starting here, we are considering only a single edge: between a single stable state and handoff state
                        if( edge != NULL )
                        {
                            // PRX_DEBUG_COLOR("Processing HO-HO: " << first_node->node_id << " <-> " << second_node->node_id, PRX_TEXT_RED);

                            //We have to do transfer planning for all arms in the common arm set.
                            const std::vector< unsigned >& transfer_arms = get_common_arms( first_node, second_node );
                            std::vector< unsigned > move_arms;
                            non_common_arm_set( move_arms, first_node, second_node );

                            // = = = DEBUG = = = : Okay, really need to see that these are correct.. probably messed it up badly.
                            // Since we are doing k=1, these should be tiny sets
                            // PRX_DEBUG_COLOR("Common (transfer) arm: " << transfer_arms[0], PRX_TEXT_LIGHTGRAY);
                            // PRX_DEBUG_COLOR("Exclusive (move) arms: " << move_arms[0] << " : " << move_arms[1], PRX_TEXT_LIGHTGRAY);
                            // = = =

                            //Figure out how many records are on each high-level node
                            unsigned nr_first_records = first_node->get_num_records();
                            unsigned nr_second_records = second_node->get_num_records();

                            //Then retrieve the record data
                            const std::vector< pose_record_t >& first_records = first_node->get_records();
                            const std::vector< pose_record_t >& second_records = second_node->get_records();

                            //TODO: we'll have to be super special awesome for k>1
                            //First, get what should be the single transfer arm
                            unsigned first_manip = transfer_arms[0];

                            //Some variables for remembering the things
                            double first_leg_length = 0.0;
                            double second_leg_length = 0.0;
                            double third_leg_length = 0.0;

                            double handoff_handoff_length = 0.0;
                            unsigned handoff_handoff_paths = 0;

                            //Need to do some processing for each pair of records
                            for( unsigned r=0; r<nr_first_records; ++r )
                            {
                                const pose_record_t& first_record = first_records[r];
                                for( unsigned t=0; t<nr_second_records; ++t )
                                {
                                    const pose_record_t& second_record = second_records[t];
                                    //Alright, now we have the records: See if we can plan the transfer path

                                    //The two records must have a common grasp for the transfer manipulator
                                    if( is_same_grasp( first_manip, first_record, second_record ) )
                                    {
                                        //And if it is the same grasp, let's plan.
                                        //Make sure to put the manipulators in their start positions
                                        go_to_start();
                                        //Then, make sure that the object is placed where we expect to be moving it from
                                        set_object_pose_from_record( first_record );

                                        unsigned first_transfer_index = get_grasp_index(first_manip, first_record);
                                        unsigned second_transfer_index = get_grasp_index(first_manip, second_record);
                                        //Check to see if we have a plan
                                        if( plan_transfer( first_manip, first_record.grasp_ids[first_transfer_index], first_record.states[first_transfer_index], second_record.states[second_transfer_index], true ) )
                                        {
                                            //If we recieved a plan, we must also get a plan for EVERY moving planner.

                                            //For now though, save the length of this path
                                            first_leg_length = saved_plan.length();

                                            //I REALLY need to fix this at some point, but we know we are getting these in the order of from first then from second record.
                                            unsigned second_manip = move_arms[0];

                                            //Put the object back where it started for this plan.
                                            set_object_pose_from_record( first_record );
                                            //Also, put the transfer arm back where it belongs for the first record
                                            set_arm_grasping_from_record( first_manip, first_record );
                                            set_zero_control();

                                            //Plan...
                                            if( plan_move( second_manip, safe_states[second_manip], first_record.retracted[ get_grasp_index(second_manip, first_record) ] ) )
                                            {
                                                //Shweet, got a plan, save its length
                                                second_leg_length = saved_plan.length();

                                                //Now, we have yet another manipulator
                                                unsigned third_manip = move_arms[1];

                                                //Put the object where this guy will be going to touch it
                                                set_object_pose_from_record( second_record );
                                                //Also put the transfer arm there with it
                                                set_arm_grasping_from_record( first_manip, second_record );
                                                //Okay, well, try putting the second manipulator back into the safe position
                                                go_to_safe( second_manip );
                                                set_zero_control();

                                                //Plan it up baby, for the win...
                                                if( plan_move( third_manip, safe_states[third_manip], second_record.retracted[ get_grasp_index(third_manip, second_record) ] ) )
                                                {
                                                    //EPICAL success, need to get the max of these three path lengths
                                                    third_leg_length = saved_plan.length();

                                                    // PRX_DEBUG_COLOR("Found a path from arm " << second_manip << " to " << third_manip, PRX_TEXT_BROWN);

                                                    third_leg_length = PRX_MAXIMUM( third_leg_length, second_leg_length );
                                                    first_leg_length = PRX_MAXIMUM( third_leg_length, first_leg_length );
                                                    //Now that we have the LONGEST path, need to add this to the average.

                                                    handoff_handoff_length += first_leg_length;
                                                    ++handoff_handoff_paths;
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                            //Alright, so now, we're done getting the value over each pair of records between these two states which meet the criteria.  Average
                            handoff_handoff_length = handoff_handoff_length / ((double)handoff_handoff_paths);
                            PRX_DEBUG_COLOR("Computed Handoff-Handoff Average: (" << first_node->node_id << " <-> " << second_node->node_id << "): ", PRX_TEXT_RED );
                            PRX_DEBUG_COLOR(":: " << handoff_handoff_length << "    [" << handoff_handoff_paths << "]", PRX_TEXT_LIGHTGRAY);
                            if( handoff_handoff_paths == 0 )
                                edge->set_weight( PRX_INFINITY );
                            else
                                edge->set_weight( handoff_handoff_length );
                        }
                    }
                }

                // ===================================================
                // =======.....=======================================
                // ======..===..======================================
                // =====..============================================
                // =====..=....=======================================
                // =====...==...======================================
                // =====...==...======================================
                // ======......=======================================
                // ===================================================

                PRX_DEBUG_COLOR("\n\n\n PROCESSING CASE 6: HANDOFF SELF-TRANSITION \n\n", PRX_TEXT_MAGENTA);

                //For each handoff high-level state
                for( unsigned i=0; i<handoff_pose_vis.size(); ++i )
                {
                    //Get the node
                    automaton_node_t* node = automaton.get_vertex_as<automaton_node_t>(handoff_pose_vis[i]);

                    //Get the records on this node
                    const std::vector< pose_record_t >& records = node->get_records();
                    //Also figure out how many records there are
                    unsigned nr_records = node->get_num_records();

                    //Some variables for computing the average
                    double self_handoff_length = 0.0;
                    unsigned self_handoff_paths = 0;

                    //Now, for each pair of records on this node
                    for( unsigned r=0; r<nr_records; ++r )
                    {
                        for( unsigned t=r+1; t<nr_records; ++t )
                        {
                            //Let's get references to the two records
                            const pose_record_t& first_record = records[r];
                            const pose_record_t& second_record = records[t];

                            //For now, let's just figure out what happens for k=1
                            const std::vector< std::vector< unsigned > >& arms = node->get_arms();

                            //Keep a running maximum for the longest path we end up finding
                            double transfer_length = 0.0;
                            double longest_path = 0.0;

                            //There are actually two separate types of plans to consider: TM and MT
                            for( unsigned v=0; v<2; ++v )
                            {
                                unsigned transfer_arm = v%2;
                                unsigned move_arm = (v+1)%2;

                                unsigned transfer_manip = arms[transfer_arm][0];
                                unsigned move_manip = arms[move_arm][0];

                                //GOD, SO BROKEN
                                //Then, Make sure we get the transfer indices for each of the records
                                unsigned transfer_first_index = get_grasp_index( transfer_manip, first_record );
                                unsigned transfer_second_index = get_grasp_index( transfer_manip, second_record );
                                //Also get the move indices for each record
                                unsigned move_first_index = get_grasp_index( move_manip, first_record );
                                unsigned move_second_index = get_grasp_index( move_manip, second_record );

                                //If the records are for different poses,
                                if( first_record.pose_id != second_record.pose_id )
                                {
                                    //And if the transfer arm has the same grasp in both records
                                    if( first_record.grasp_ids[transfer_first_index] == second_record.grasp_ids[transfer_second_index] )
                                    {
                                        //Then, plan a transfer for this arm
                                        //Make sure to put the manipulators in their start positions
                                        go_to_start();
                                        //Then, make sure that the object is placed where we expect to be reaching it
                                        set_object_pose_from_record( first_record );

                                        //Now, check to see if there is a plan
                                        if( plan_transfer( transfer_manip, first_record.grasp_ids[transfer_first_index], first_record.states[transfer_first_index], second_record.states[transfer_second_index], true ) )
                                        {
                                            //We have a plan, so record its length
                                            transfer_length = saved_plan.length();

                                            //Now, make sure we keep this arm in the transferred state, but move the object
                                            set_object_pose_from_record( first_record );
                                            set_arm_grasping_from_record( transfer_manip, first_record );
                                            set_zero_control();

                                            //First, ensure that move arm is actually changing grasp, or there was no point in following this edge
                                            if( first_record.grasp_ids[move_first_index] != second_record.grasp_ids[move_second_index] )
                                            {
                                                //There was a plan found, need to try moving the other arm
                                                if( plan_move( move_manip, first_record.retracted[move_first_index], safe_states[move_manip] ) )
                                                {
                                                    //Get the length for this leg of the move arm's trip
                                                    double move_lengths = saved_plan.length();
                                                    //Then, set things up to be moving to the second record location
                                                    set_object_pose_from_record( second_record );
                                                    set_arm_grasping_from_record( transfer_manip, second_record );
                                                    set_zero_control();

                                                    //Then, plan the second trip for the moving arm
                                                    if( plan_move(move_manip, safe_states[move_manip], second_record.retracted[move_second_index] ) )
                                                    {
                                                        move_lengths += saved_plan.length();
                                                        //If the plan worked, update the running max-length path for self-transition
                                                        double tmp_max = PRX_MAXIMUM( transfer_length, move_lengths );
                                                        longest_path = PRX_MAXIMUM( tmp_max, longest_path );
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                            //If we got a nonzero longest path
                            if( longest_path > 0.0 )
                            {
                                //Add this to our running average
                                self_handoff_length += longest_path;
                                //And make sure we increase the number of paths
                                ++self_handoff_paths;
                            }
                        }
                    }
                    //Now, after running all pairs of records, report the average
                    self_handoff_length = self_handoff_length/((double)self_handoff_paths);
                    PRX_DEBUG_COLOR("Computed Handoff (" << i << ") self-transition average path costs: ", PRX_TEXT_MAGENTA);
                    PRX_DEBUG_COLOR(":: " << self_handoff_length << "   [" << self_handoff_paths << "]", PRX_TEXT_LIGHTGRAY);
                    automaton_edge_t* edge = automaton.get_edge_as<automaton_edge_t>( handoff_pose_vis[i], handoff_pose_vis[i] );
                    if( self_handoff_paths == 0 )
                        edge->set_weight( PRX_INFINITY );
                    else
                        edge->set_weight( self_handoff_length );
                }

                // ===================================================
                // =============.....=================================
                // ============..===..===============.================
                // ===========..=====================.================
                // =========......=======...==.===.=.....=============
                // ======..===..========.===.=.===.==.================
                // =====....=..=========.===.=.===.==.==.=============
                // ======.....===========...===...====..==============
                // ===================================================

                //Serialize the low-level motion planning graphs
                serialize_motion_graphs();

                //Alright, first thing's first, we need to remove extraneous edges which we cannot follow due to infinite cost
                std::vector< undirected_edge_index_t > edges_to_remove;
                for( unsigned i=0; i<edge_indices.size(); ++i )
                {
                    automaton_edge_t* edge = automaton.get_edge_as< automaton_edge_t >(edge_indices[i]);
                    if( edge->get_weight() >= PRX_INFINITY-1 )
                    {
                        //This edge has no viable transitions, mark it for removal.
                        edges_to_remove.push_back( edge_indices[i] );
                    }
                }
                //Then, for all the edge indices indicated for removal
                for( unsigned i=0; i<edges_to_remove.size(); ++i )
                {
                    automaton.remove_edge( edges_to_remove[i] );
                }

                //Fix edge indices so that it only contains things not in the removed edges
                std::vector< undirected_edge_index_t > retained_edges;
                for( unsigned i=0; i<edge_indices.size(); ++i )
                {
                    //If it is not a removed edge
                    if( std::find( edges_to_remove.begin(), edges_to_remove.end(), edge_indices[i] ) == edges_to_remove.end() )
                    {
                        //It must be retained
                        retained_edges.push_back( edge_indices[i] );
                    }
                }
                //Then, make sure our indices are set
                edge_indices = retained_edges;


                //Let's go ahead and print the automaton we will be serializing out.
                // print_automaton();

                //Then serialize it
                serialize_automaton();
            }

            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // ========       General Search Functions      ==========
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================

            bool multi_modal_forward_tp_t::plan_top( unsigned record_index, automaton_node_t* source, automaton_node_t* target )
            {
                //Before we begin make sure we clear out the plan storage
                final_plan.clear();
                saved_plan.clear();

                //Before we do complex things, make sure our source and target are both in the automaton
                if( source == NULL )
                {
                    // PRX_DEBUG_COLOR("Case: APPROACH", PRX_TEXT_BROWN);
                    //We are moving to a stable pose from nothing grasping
                    PRX_ASSERT( target != NULL );
                    //Since the target must be a stable pose, we just need a plan to any grasp that corresponds to the current object pose...
                    object_space->copy_to_point( object_state );
                    //Get a record that satisfies this stuff
                    const pose_record_t* target_record = get_approach_record( object_state, target );
                    //Then, if there is something we can approach
                    if( target_record != NULL )
                    {
                        //Get the manip index which we are planning for
                        unsigned manip_index = target->get_arms()[0][0];
                        //Plan to that record
                        // PRX_DEBUG_COLOR("Plan move ALPHA", PRX_TEXT_CYAN);
                        unsigned grasp_index = get_grasp_index(manip_index, *target_record);
                        //Go to the start state
                        go_to_start();
                        //And plan
                        bool result = plan_move( manip_index, safe_states[manip_index], target_record->retracted[ grasp_index ] );
                        //Alright, and in case the plan succeeded, append in the approach
                        saved_plan = *(target_record->approach_plans[ get_grasp_index(manip_index, *target_record) ] );
                        append_plan( manip_index );
                        append_grasp( manip_index, 5, 1.0 );

                        //Make sure we put the manipulator at its goal
                        transfer_spaces[ manip_index ]->copy_from_point( target_record->states[ grasp_index ] );
                        //Make sure the object hasn't moved
                        object_space->copy_from_point( target_record->object_pose );

                        return result;
                    }
                    //Otherwise, nothing doing here, report failure
                    return false;
                }
                else if( target == NULL )
                {
                    // PRX_DEBUG_COLOR("Case: RETRACT", PRX_TEXT_RED);
                    //We must be going from a stable grasp to no grasping
                    PRX_ASSERT( source->get_arms().size() == 1 );

                    //Alright, need the source record here
                    const pose_record_t& source_record = source->get_records()[record_index];
                    unsigned manip_index = source->get_arms()[0][0];

                    //My assumption here is that we just plan to the safe states
                    // PRX_DEBUG_COLOR("Plan move BETA", PRX_TEXT_RED);
                    unsigned grasp_index = get_grasp_index( manip_index, source_record );
                    //Go to the friggin' start
                    go_to_start();
                    //Also, need to put the object in the way
                    object_space->copy_from_point( source_record.object_pose );

                    //Then plan
                    bool result = plan_move( manip_index, source_record.retracted[ grasp_index ], safe_states[manip_index] );
                    //Okah, so a challenge, eh?  Need to PRE-pend the retraction
                    saved_plan = *( source_record.retraction_plans[ get_grasp_index( manip_index, source_record ) ] );
                    prepend_plan( manip_index );
                    prepend_grasp( manip_index, 5, 0.0 );

                    //Put the arm where it planned to
                    go_to_safe( manip_index );
                    //Let's ensure the object is where we want it
                    object_space->copy_from_point( source_record.object_pose );

                    return result;
                }

                //First, retrieve the record that we are planning from
                const pose_record_t& source_record = source->get_records()[record_index];

                //Alright, we have to figure out what high-level planning to do from here
                bool source_stable = source->get_arms().size() == 1 ? true : false;
                bool target_stable = target->get_arms().size() == 1 ? true : false;

                //If we are starting from a stable grasp pose
                if(source_stable)
                {
                    //And we are going to a stabe pose
                    if(target_stable)
                    {
                        //CASE::  S => S
                        // PRX_DEBUG_COLOR("Stable-Stable:  " << source->get_arms()[0][0] << " <-> " << target->get_arms()[0][0], PRX_TEXT_BROWN );
                        return plan_stable_stable( source_record, source, target );
                    }
                    //Or we are going to a handoff
                    else
                    {
                        return plan_stable_handoff( source_record, source, target, false );
                    }
                }
                //Or, if we are starting from a handoff state
                else
                {
                    //And we are going to a stabe pose
                    if(target_stable)
                    {
                        return plan_stable_handoff( source_record, source, target, true );
                    }
                    //Or we are going to a handoff
                    else
                    {
                        return plan_handoff_handoff( source_record, source, target );
                    }
                }
            }

            bool multi_modal_forward_tp_t::plan_stable_stable( const pose_record_t& source_record, automaton_node_t* source, automaton_node_t* target )
            {
                // PRX_DEBUG_COLOR("Case: stable-stable", PRX_TEXT_MAGENTA);
                //First of all, if these are the same nodes, we are doing a regrasp, so we need to require a different grasp
                bool enforce_new_grasp = false;
                bool transfer = false;
                if( source == target )
                {
                    enforce_new_grasp = true;
                    // if( successful_transfer )
                    // {
                        transfer = uniform_int_random(0,7) <= 1 ? false : true;
                    // }
                    // else
                    // {
                    //     transfer = true;
                    // }
                }

                //Then, get the manipulator indices for the problem
                unsigned first_index = source->get_arms()[0][0];
                unsigned second_index = target->get_arms()[0][0];

                //Let's make sure we have appropriate indices
                unsigned first_source_index = get_grasp_index( first_index, source_record );
                //Then, let's get some valid record from the target
                const pose_record_t* target_record = get_s_s_record( target, source_record, enforce_new_grasp, transfer );

                //If such a record to plan to was found
                if( target_record != NULL )
                {
                    //Get the index for the second grasp
                    unsigned second_target_index = get_grasp_index( second_index, *target_record );

                    //First of all, if we are planning a single-arm transfer
                    if( transfer )
                    {
                        // PRX_DEBUG_COLOR("Planning a Transfer", PRX_TEXT_BROWN);
                        //So, make the plan
                        bool result = plan_transfer( first_index, source_record.grasp_ids[ first_source_index ], source_record.states[ first_source_index ], target_record->states[second_target_index] );

                        if( result )
                        {
                            PRX_DEBUG_COLOR("[" << point_number+1 << "]: Created transfer manip: " << first_index << "   through grasp: " << source_record.grasp_ids[first_source_index], PRX_TEXT_BROWN);
                            PRX_DEBUG_COLOR("Object put at: " << object_space->print_point( target_record->object_pose, 3 ), PRX_TEXT_LIGHTGRAY );
                            successful_transfer = true;
                        }

                        //Make sure we leave the arm where it belongs
                        transfer_spaces[ first_index ]->copy_from_point( target_record->states[second_target_index] );
                        //Then, make sure we leave the object where it was transferred
                        object_space->copy_from_point( target_record->object_pose );

                        return result;
                    }
                    //Otherwise, we are planning two move operations here
                    else
                    {
                        // PRX_DEBUG_COLOR("Planning re-grasp or table handoff", PRX_TEXT_GREEN);
                        // PRX_DEBUG_COLOR("Plan move DELTA", PRX_TEXT_BROWN);
                        //Begin by taking the first arm back to its safe place
                        if( plan_move( first_index, source_record.retracted[ first_source_index ], safe_states[first_index] ) )
                        {
                            //Prepend the retraction
                            saved_plan = *( source_record.retraction_plans[ first_source_index ] );
                            prepend_plan( first_index );
                            prepend_grasp( first_index, 5, 0.0 );

                            //So here, we also need to put the first manipulator at his goal position, because we don't know where planning left him.
                            go_to_safe( first_index );

                            //And if that worked out, also see if we can move the second arm to its grasp
                            bool result = plan_move( second_index, safe_states[second_index], target_record->retracted[ second_target_index ] );
                            //Also need to append the approach
                            saved_plan = *( target_record->approach_plans[ second_target_index ] );
                            append_plan( second_index );
                            append_grasp( second_index, 5, 1.0 );

                            //Place the arm where it planned to
                            transfer_spaces[ second_index ]->copy_from_point( target_record->states[ second_target_index ] );
                            //Make sure we keep the object in place
                            object_space->copy_from_point( target_record->object_pose );

                            //DEBUG: Let's make sure we are making reasonable plans
                            if( result )
                            {
                                if( first_index != second_index )
                                {
                                    PRX_DEBUG_COLOR("[" << point_number+1 << "]: Created M-M plan for arms: " << first_index << " : " << second_index, PRX_TEXT_CYAN);
                                }
                                else
                                {
                                    PRX_DEBUG_COLOR("[" << point_number+1 << "]: Created Regrasp for arm: " << first_index, PRX_TEXT_MAGENTA );
                                }
                            }

                            return result;
                        }
                        //Our very first plan failed... geeze, report failure
                        return false;
                    }
                }
                //Nothing we could plan to, sadly, so abort out.
                return false;
            }

            bool multi_modal_forward_tp_t::plan_stable_handoff( const pose_record_t& source_record, automaton_node_t* source, automaton_node_t* target, bool to_stable )
            {
                // PRX_DEBUG_COLOR("Case: stable-handoff", PRX_TEXT_MAGENTA);
                //Need indices for what arms we are planning for
                unsigned transfer_index = 0;
                unsigned move_index = 0;

                //If we are planning toward a stable grasp, we first MOVE the other arm, then TRANSFER
                if( to_stable )
                {
                    //The transfer arm then must be the one on the target
                    transfer_index = target->get_arms()[0][0];
                    //We also need to know the one that is moving
                    if( source->get_arms()[0][0] == transfer_index )
                        move_index = source->get_arms()[1][0];
                    else
                        move_index = source->get_arms()[0][0];

                    //Get the transfer arm's index in the source record
                    unsigned transfer_source_index = get_grasp_index( transfer_index, source_record );

                    //We need to get a record where the transfer arm has the same grasp id
                    const pose_record_t* target_record = get_h_s_record( target, source_record, transfer_index );

                    //Now, assuming there is something which has the same grasp id
                    if( target_record != NULL )
                    {
                        //Get some of the other important indices
                        unsigned move_source_index = get_grasp_index( move_index, source_record );
                        unsigned transfer_target_index = get_grasp_index( transfer_index, *target_record );

                        //Before we begin, make sure the transfer arm is at the source record position
                        transfer_spaces[ transfer_index ]->copy_from_point( source_record.states[ transfer_source_index ] );

                        //First, we need to try to move things
                        if( plan_move( move_index, source_record.retracted[ move_source_index ], safe_states[ move_index ] ) )
                        {
                            //The plan succeeded, so prepend the necessary things to retract away
                            saved_plan = *( source_record.retraction_plans[ move_source_index ] );
                            prepend_plan( move_index );
                            prepend_grasp( move_index, 5, 0.0 );

                            //Then, tuck this manipulator away at the safe state, as planning could have technically left him anywhere.
                            go_to_safe( move_index );

                            //Now, attempt to transfer the object to the stable pose
                            bool result = plan_transfer( transfer_index, source_record.grasp_ids[ transfer_source_index ], source_record.states[ transfer_source_index ], target_record->states[ transfer_target_index ] );

                            if( result )
                            {
                                PRX_DEBUG_COLOR("[" << point_number+1 << "]: Created HO>S path: " << move_index << " > " << transfer_index, PRX_TEXT_GREEN);
                                PRX_DEBUG_COLOR("Object put at: " << object_space->print_point( target_record->object_pose, 3 ), PRX_TEXT_LIGHTGRAY );
                            }

                            //Then, leave the state where we planned to
                            transfer_spaces[ transfer_index ]->copy_from_point( target_record->states[ transfer_target_index ] );

                            //And report our success
                            return result;
                        }
                    }
                }
                //Otherwise, we TRANSFER first, then MOVE. (to handoff)
                else
                {
                    //The transfer arm then must be the one on the source
                    transfer_index = source->get_arms()[0][0];
                    //We also need to know the one that is moving
                    if( target->get_arms()[0][0] == transfer_index )
                        move_index = target->get_arms()[1][0];
                    else
                        move_index = target->get_arms()[0][0];

                    //Get the transfer arm's index in the source record
                    unsigned transfer_source_index = get_grasp_index( transfer_index, source_record );

                    //We need to get a record where the transfer arm has the same grasp id
                    const pose_record_t* target_record = get_h_s_record( target, source_record, transfer_index );

                    //Now, assuming there is something which has the same grasp id
                    if( target_record != NULL )
                    {
                        //Get some of the other important indices
                        unsigned move_target_index = get_grasp_index( move_index, *target_record );
                        unsigned transfer_target_index = get_grasp_index( transfer_index, *target_record );

                        //First, attempt the transfer
                        if( plan_transfer( transfer_index, source_record.grasp_ids[ transfer_source_index ], source_record.states[ transfer_source_index ], target_record->states[ transfer_target_index ] ) )
                        {
                            //Then, leave the state where we planned to
                            transfer_spaces[ transfer_index ]->copy_from_point( target_record->states[ transfer_target_index ] );

                            //Now, attempt to move the other arm to meet the handoff
                            bool result = plan_move( move_index, safe_states[ move_index ], target_record->retracted[ move_target_index ] );

                            //Let's append an approach to the object
                            saved_plan = *( target_record->approach_plans[ move_target_index ] );
                            append_plan( move_index );
                            append_grasp( move_index, 5, 1.0 );

                            if( result )
                            {
                                PRX_DEBUG_COLOR("[" << point_number+1 << "]: Created S>HO path: " << transfer_index << " > " << move_index, PRX_TEXT_BLUE);
                            }
                            // else
                            // {
                            //     PRX_DEBUG_COLOR("It's the planning that fails...?", PRX_TEXT_RED);
                            // }

                            //And report our success
                            return result;
                        }
                    }
                    // else
                    // {
                    //     PRX_DEBUG_COLOR("> Target Record is NULL", PRX_TEXT_RED);
                    // }
                }

                return false;
            }

            bool multi_modal_forward_tp_t::plan_handoff_handoff( const pose_record_t& source_record, automaton_node_t* source, automaton_node_t* target )
            {
                // PRX_DEBUG_COLOR("Case: handoff-handoff", PRX_TEXT_MAGENTA);
                //Need indices for what arms we are planning for
                unsigned transfer_index = 0;
                unsigned move_one_index = 0;
                unsigned move_two_index = 0;

                //The transfer arm is just the first (only) arm that the two handoff nodes have in common
                transfer_index = get_common_arms( source, target )[0];

                //Then, the first move arm is whatever one is not the common
                if( source->get_arms()[0][0] == transfer_index )
                    move_one_index = source->get_arms()[1][0];
                else
                    move_one_index = source->get_arms()[0][0];

                //Also get the second move arm in the same fashion
                if( target->get_arms()[0][0] == transfer_index )
                    move_two_index = target->get_arms()[1][0];
                else
                    move_two_index = target->get_arms()[0][0];

                //Alright, since we have the source record, we can get the indices in the source record
                unsigned transfer_source_index = get_grasp_index( transfer_index, source_record );
                unsigned move_one_source_index = get_grasp_index( move_one_index, source_record );

                //We need to get a record where the transfer arm has the same grasp id
                const pose_record_t* target_record = get_h_h_record( target, source_record, transfer_index, source == target );

                //Now, assuming there is something which works
                if( target_record != NULL )
                {
                    //Get some of the other important indices
                    unsigned transfer_target_index = get_grasp_index( transfer_index, *target_record );
                    unsigned move_two_target_index = get_grasp_index( move_two_index, *target_record );

                    //So, before we move, we need to make sure that the transfer arm has been placed at the source record
                    transfer_spaces[ transfer_index ]->copy_from_point( source_record.states[ transfer_source_index ] );


                    // PRX_DEBUG_COLOR("First", PRX_TEXT_BROWN);
                    //First, we need to try to move away the first arm
                    if( plan_move( move_one_index, source_record.retracted[ move_one_source_index ], safe_states[ move_one_index ] ) )
                    {
                        //The plan succeeded, so prepend the necessary things to retract away
                        saved_plan = *( source_record.retraction_plans[ move_one_source_index ] );
                        prepend_plan( move_one_index );
                        prepend_grasp( move_one_index, 5, 0.0 );

                        //Then, tuck this manipulator away at the safe state, as planning could have technically left him anywhere.
                        go_to_safe( move_one_index );

                        //Now, attempt to transfer the object to the stable pose
                        if( plan_transfer( transfer_index, source_record.grasp_ids[ transfer_source_index ], source_record.states[ transfer_source_index ], target_record->states[ transfer_target_index ] ) )
                        {
                            //Then, leave the state where we planned to
                            transfer_spaces[ transfer_index ]->copy_from_point( target_record->states[ transfer_target_index ] );
                            //Fortunately, transfers don't require additional plan information, so... huzzah!


                            // PRX_DEBUG_COLOR("Second", PRX_TEXT_BROWN);
                            //Finally, let's try moving the last arm from its safe state to the new handoff position
                            bool result = plan_move( move_two_index, safe_states[move_two_index], target_record->retracted[ move_two_target_index ] );

                            //Then, if this went through, we'll also need to append the approach & grasp of the object
                            saved_plan = *( target_record->approach_plans[ move_two_target_index ] );
                            append_plan( move_two_index );
                            append_grasp( move_two_index, 5, 1.0 );

                            //Also should ensure we leave this arm in the final grasping state, since that's where it would be after the grasp
                            transfer_spaces[ move_two_index ]->copy_from_point( target_record->states[ move_two_target_index ] );

                            if(result)
                            {
                                PRX_DEBUG_COLOR("[" << point_number+1 << "]: Made HO-HO path: " << move_one_index << " <(\'_\'<) " << transfer_index << " (>\'_\')> " << move_two_index, PRX_TEXT_RED);
                            }
                            //And report our success
                            return result;
                        }
                    }
                }
                return false;
            }

            const pose_record_t* multi_modal_forward_tp_t::get_approach_record( space_point_t* object_pose, automaton_node_t* target )
            {
                //Get all the records we might be going to
                const std::vector< pose_record_t >& records = target->get_records();
                std::vector< unsigned > candidtate_records;

                // PRX_DEBUG_COLOR("Testing for object pose: " << object_space->print_point(object_pose, 3), PRX_TEXT_CYAN );

                for( unsigned i=0; i<target->get_num_records(); ++i )
                {
                    const pose_record_t& test_record = records[i];
                    //Then, if the record contains the indicated pose
                    //And hasn't been previously discovered
                    if( is_same_pose( object_pose, test_record.object_pose ) && (!test_record.discovered || discrete_search) )
                    {
                        //This is a candidate
                        candidtate_records.push_back( i );
                    }
                }

                //Now, if we have any candidates
                if( candidtate_records.size() != 0 )
                {
                    //Choose a record at random
                    unsigned chosen_index = uniform_int_random(0, candidtate_records.size()-1);
                    //Remember what I chose
                    reached_record_index = candidtate_records[chosen_index];
                    reached_record = const_cast< pose_record_t* >( &records[ reached_record_index ] );
                    //And return the record
                    return &records[ reached_record_index ];
                }
                //Otherwise, we had no candidates, so there is no record to return
                return NULL;
            }

            const pose_record_t* multi_modal_forward_tp_t::get_s_s_record( automaton_node_t* target, const pose_record_t& source_record, bool new_grasp, bool transfer )
            {
                //Okay, first I guess let's just get the records from the target
                const std::vector< pose_record_t >& records = target->get_records();
                //We should also use a list of candidates and select one randomly for things (?)
                std::vector< unsigned > candidtate_records;

                //First of all, if we are enforcing a new grasp, then the target and source must be the same
                if( new_grasp )
                {
                    //So we know what arm we are dealing with
                    unsigned manip_index = target->get_arms()[0][0];
                    //BUT, if we are transferring,
                    if( transfer )
                    {
                        bool bias = false;
                        if( bias_low_level_samples )
                        {
                            bias = uniform_int_random(0, 3) == 0 ? true : false;
                        }
                        //We in fact need the SAME grasp, but a different object pose, so for each record
                        for( unsigned i=0; i<target->get_num_records(); ++i )
                        {
                            const pose_record_t& test_record = records[i];
                            //if they have the same grasp,
                            //But different poses,
                            //and if we are doing discrete search, or the record has not already been reached
                            if( (is_same_grasp( manip_index, source_record, test_record )) && ( source_record.pose_id != test_record.pose_id ) && candidate_record_check(source_record, test_record) )
                            {
                                //If we are biased and the record corresponds to a goal position.
                                if( bias )
                                {
                                    if( (object_distance( test_record.object_pose, object_goal_state ) < PRX_DISTANCE_CHECK) )
                                    {
                                        candidtate_records.push_back( i );
                                    }
                                }
                                else
                                {
                                    //Success, this guy is a candidate for expansion
                                    candidtate_records.push_back( i );
                                }
                            }
                        }
                    }
                    //otherwise, continue as planned, need the same pose but different grasp
                    else
                    {
                        //For each record
                        for( unsigned i=0; i<target->get_num_records(); ++i )
                        {
                            const pose_record_t& test_record = records[i];
                            //if they DON'T have the same grasp,
                            //and they have the same pose,
                            //and if we're not doing discrete search, but the record has been discovered
                            if( !(is_same_grasp( manip_index, source_record, test_record )) && ( source_record.pose_id == test_record.pose_id ) && candidate_record_check(source_record, test_record) )
                            {
                                //Success, this guy is a candidate for expansion
                                candidtate_records.push_back( i );
                            }
                        }
                    }
                }
                //Otherwise, we are planning between two different stable poses
                else
                {
                    //Alright then, for each record
                    for( unsigned i=0; i<target->get_num_records(); ++i )
                    {
                        //Get the actual record
                        const pose_record_t& test_record = records[i];
                        //If the two records have the same object pose
                        // And it has not been discovered
                        if( is_same_pose( source_record, test_record ) && candidate_record_check(source_record, test_record) )
                        {
                            //Then it's a candidate!
                            candidtate_records.push_back( i );
                        }
                    }
                }

                //Now, if we have any candidates
                if( candidtate_records.size() != 0 )
                {
                    //Choose a record at random
                    unsigned chosen_index = uniform_int_random(0, candidtate_records.size()-1);
                    //Remember what I chose
                    reached_record_index = candidtate_records[chosen_index];
                    reached_record = const_cast< pose_record_t* >( &records[ reached_record_index ] );
                    //And set the record
                    return &records[ reached_record_index ];
                }
                //Otherwise, we had no candidates, so there is no record to return
                return NULL;
            }

            const pose_record_t* multi_modal_forward_tp_t::get_h_s_record( automaton_node_t* target, const pose_record_t& source_record, unsigned transfer_index )
            {
                //Okay, first I guess let's just get the records from the target
                const std::vector< pose_record_t >& records = target->get_records();
                //We should also use a list of candidates and select one randomly for things
                std::vector< unsigned > candidtate_records;

                //Alright then, for each record
                for( unsigned i=0; i<target->get_num_records(); ++i )
                {
                    //Get the actual record
                    const pose_record_t& test_record = records[i];
                    //If the two records have the same grasp,
                    //and the record hasn't been discovered
                    if( is_same_grasp( transfer_index, source_record, test_record ) && candidate_record_check(source_record, test_record) )
                    {
                        //Then it's a candidate!
                        candidtate_records.push_back( i );
                    }
                }

                //Now, if we have any candidates
                if( candidtate_records.size() != 0 )
                {
                    //Choose a record at random
                    unsigned chosen_index = uniform_int_random(0, candidtate_records.size()-1);
                    //Remember what I chose
                    reached_record_index = candidtate_records[chosen_index];
                    reached_record = const_cast< pose_record_t* >( &records[ reached_record_index ] );
                    //And set the record
                    return &records[ reached_record_index ];
                }

                return NULL;
            }

            const pose_record_t* multi_modal_forward_tp_t::get_h_h_record( automaton_node_t* target, const pose_record_t& source_record, unsigned transfer_index, bool same_node )
            {
                //Okay, first I guess let's just get the records from the target
                const std::vector< pose_record_t >& records = target->get_records();
                //We should also use a list of candidates and select one randomly for things
                std::vector< unsigned > candidtate_records;

                //Alright then, for each record
                for( unsigned i=0; i<target->get_num_records(); ++i )
                {
                    //Get the actual record
                    const pose_record_t& test_record = records[i];
                    //If the two records have the same grasp
                    if( is_same_grasp( transfer_index, source_record, test_record ) )
                    {
                        //Now, if it is the same node
                        if( same_node )
                        {
                            //Then we must additionally ensure the records have different poses associated with them
                            //And of course that the record has not been discovered
                            if( !(is_same_pose( source_record, test_record )) && candidate_record_check(source_record, test_record) )
                            {
                                //Only then is it a candidate
                                candidtate_records.push_back( i );
                            }
                        }
                        //Else, if it is some other handoff node that we haven't discovered
                        else if( candidate_record_check(source_record, test_record))
                        {
                            //Then it's a candidate!
                            candidtate_records.push_back( i );
                        }
                    }
                }

                //Now, if we have any candidates
                if( candidtate_records.size() != 0 )
                {
                    //Choose a record at random
                    unsigned chosen_index = uniform_int_random(0, candidtate_records.size()-1);
                    //Remember what I chose
                    reached_record_index = candidtate_records[chosen_index];
                    reached_record = const_cast< pose_record_t* >( &records[ reached_record_index ] );
                    //And set the record
                    return &records[ reached_record_index ];
                }

                return NULL;
            }

            bool multi_modal_forward_tp_t::candidate_record_check( const pose_record_t& source_record, const pose_record_t& test_record )
            {
                //So first, if it has not been discovered, always go for it
                if( !test_record.discovered )
                    return true;

                //Alright, so we need the heuristic value for the specific high-level edge which would connect these records
                automaton_edge_t* edge = automaton.get_edge_as< automaton_edge_t >( source_record.search_node->auto_state->index, test_record.search_node->auto_state->index );

                //Now, if we are doing discrete search, we check against heuristics
                if( discrete_search && source_record.search_node->cost + edge->get_weight() < test_record.search_node->cost )
                    return true;
                return false;
            }

            bool multi_modal_forward_tp_t::plan_move(int manip_index, util::space_point_t* start, util::space_point_t* goal, bool ignore_others, bool verbose)
            {
                if( verbose )
                {
                    PRX_DEBUG_COLOR("========================", PRX_TEXT_CYAN);
                    PRX_DEBUG_COLOR(" = Planning a move... = ", PRX_TEXT_GREEN);
                    PRX_DEBUG_COLOR("========================", PRX_TEXT_CYAN);
                }

                //Set the context to the appropriate one
                if( ignore_others )
                    model->use_context( move_ignoring_context_names[manip_index] );
                else
                    model->use_context( move_context_names[manip_index] );

                //Set up the sampler to work with the correct arm
                ungrasping_sampler->link_info( manipulators[manip_index], move_spaces[manip_index], object_space, transfer_spaces[manip_index] );

                //Then, just plan
                bool success = plan(start, goal, move_specifications[manip_index], planner_queries[manip_index], move_planners[manip_index], verbose);

                if( success )
                {
                    // PRX_DEBUG_COLOR("=========================", PRX_TEXT_GREEN);
                    // PRX_DEBUG_COLOR(" =   Successful Move   = ", PRX_TEXT_GREEN);
                    // PRX_DEBUG_COLOR("=========================", PRX_TEXT_GREEN);
                    // PRX_DEBUG_COLOR("Start  : " << move_spaces[manip_index]->print_point( start, 16 ), PRX_TEXT_RED );
                    // PRX_DEBUG_COLOR("Goal   : " << move_spaces[manip_index]->print_point( goal, 16 ), PRX_TEXT_CYAN);
                    // PRX_DEBUG_COLOR("Reached: " << move_spaces[manip_index]->print_point( planner_queries[manip_index]->path.back(), 16 ), PRX_TEXT_LIGHTGRAY );

                    //Let's just go ahead and append it in already, eh?
                    append_plan( manip_index );
                    //Also, ensure that we leave whatever manipulator we are planning for at the goal location for future planning calls.
                    move_spaces[manip_index]->copy_from_point( saved_plan.back().control );
                }

                return success;
            }

            bool multi_modal_forward_tp_t::plan_transfer(int manip_index, unsigned grasp_id, util::space_point_t* start, util::space_point_t* goal, bool ignore_others, bool verbose)
            {
                if( verbose )
                {
                    PRX_DEBUG_COLOR("============================", PRX_TEXT_BROWN);
                    PRX_DEBUG_COLOR(" = Planning a transfer... = ", PRX_TEXT_RED);
                    PRX_DEBUG_COLOR("============================", PRX_TEXT_BROWN);
                }

                //Set the context to be a transfer one
                if( ignore_others )
                    model->use_context( transfer_ignoring_context_names[manip_index] );
                else
                    model->use_context( transfer_context_names[manip_index] );

                //Set up the manipulation sampler to be working with the appropriate arm for the planning
                manipulation_sampler->link_info( manipulators[manip_index], move_spaces[manip_index], object_space, transfer_spaces[manip_index] );

                //Then, just plan
                bool success = plan(start, goal, transfer_specifications[grasp_id][manip_index], transfer_queries[grasp_id][manip_index], transfer_planners[grasp_id][manip_index], verbose);

                //If we actually got a plan
                if( success )
                {
                    //Let's just go ahead and append it in already, eh?
                    append_plan( manip_index );
                    //Also, ensure that we leave whatever manipulator we are planning for at the goal location for future planning calls.
                    move_spaces[manip_index]->copy_from_point( saved_plan.back().control );
                }

                return success;
            }

            bool multi_modal_forward_tp_t::plan(util::space_point_t* start, util::space_point_t* goal, motion_planning_specification_t* spec, motion_planning_query_t* query, motion_planner_t* planner, bool verbose)
            {
                //Make sure to set up the saved plan
                saved_plan.clear();
                saved_plan.link_control_space( model->get_control_space() );

                if( verbose )
                {
                    PRX_DEBUG_COLOR("Planning in context: " << model->get_current_context(), PRX_TEXT_BROWN);
                    PRX_DEBUG_COLOR("Plan start: " << model->get_state_space()->print_point(start, 3), PRX_TEXT_GREEN );
                    PRX_DEBUG_COLOR("Plan goal : " << model->get_state_space()->print_point(goal, 3), PRX_TEXT_MAGENTA );
                }

                //First, generate the query to pass to the planner
                query->clear();
                query->link_start(start);
                goal_state_t* cast_goal = dynamic_cast< goal_state_t* >(query->get_goal());
                cast_goal->set_goal_state( goal );
                planner->link_query( query );

                //Alright, print RIGHT before we call resolve
                if( verbose )
                {
                    PRX_DEBUG_COLOR("RIGHT BEFORE RESOLVING QUERY", PRX_TEXT_RED);
                    PRX_DEBUG_COLOR("State: " << global_state_space->print_memory( 3 ), PRX_TEXT_LIGHTGRAY);
                }

                //Then try the plan
                planner->resolve_query();

                //If a path was found, report success,
                if(query->plan.size() != 0)
                {
                    //Store the plan
                    saved_plan = query->plan;
                    return true;
                }

                //Otherwise, report failure,
                return false;
            }

            double multi_modal_forward_tp_t::object_cost_to_go( space_point_t* state )
            {
                //Alright, first, remember what context we were in
                std::string old_context = model->get_current_context();

                //Then, remember what state we were in.
                model->use_context("full_space");
                space_point_t* stored_state = global_state_space->alloc_point();

                //Now, let's get the object's coordinates
                global_state_space->copy_from_point( state );
                double point_x = (*object_space)[0];
                double point_y = (*object_space)[1];
                double point_z = (*object_space)[2];

                //Then, let's get the object's goal coordinates
                //Get the query in motion planning query form
                motion_planning_query_t* mp_query = dynamic_cast<motion_planning_query_t*>(input_query);
                mp_query->get_goal()->link_space( global_state_space );
                global_state_space->copy_from_point( mp_query->get_goal()->get_goal_points()[0] );
                double goal_x = (*object_space)[0];
                double goal_y = (*object_space)[1];
                double goal_z = (*object_space)[2];

                //Make sure we restore the state
                global_state_space->copy_from_point( stored_state );
                //And clean up after ourselves
                global_state_space->free_point( stored_state );

                //Restore the context
                model->use_context(old_context);

                //Then, now that we have the information we want, compute Euclidean distance
                return pow( (goal_x - point_x)*(goal_x - point_x) + (goal_y - point_y)*(goal_y - point_y) + (goal_z - point_z)*(goal_z - point_z) , 0.5 );
            }

            double multi_modal_forward_tp_t::object_distance( util::space_point_t* from, util::space_point_t* to )
            {
                double point_x = from->memory[0];
                double point_y = from->memory[1];
                double point_z = from->memory[2];

                double goal_x = from->memory[0];
                double goal_y = from->memory[1];
                double goal_z = from->memory[2];

                return pow( (goal_x - point_x)*(goal_x - point_x) + (goal_y - point_y)*(goal_y - point_y) + (goal_z - point_z)*(goal_z - point_z) , 0.5 );
            }

            void multi_modal_forward_tp_t::trace_path( tree_vertex_index_t target )
            {
                PRX_DEBUG_COLOR("Tracing a path: ", PRX_TEXT_GREEN);
                model->use_context( "full_space" );
                tree_vertex_index_t new_v = target;
                search_node_t* v = tree.get_vertex_as< search_node_t >( target );
                //Get a cast query
                motion_planning_query_t* cast_query = dynamic_cast< motion_planning_query_t* >(input_query);
                PRX_ASSERT( cast_query != NULL );
                // double cost = PRX_INFINITY;

                path_transitions = 0;
                while( v->get_parent() != new_v )
                {
                    tree_edge_index_t e = tree.edge( v->get_parent(), new_v);
                    rrt_edge_t* edge = tree.get_edge_as< rrt_edge_t >( e );
                    ++path_transitions;

                    plan_t hold_plan(edge->plan);
                    hold_plan += cast_query->plan;
                    cast_query->plan = hold_plan;
                    new_v = v->get_parent();
                    v = tree.get_vertex_as< search_node_t >( new_v );
                }
                //TODO: should fix this so we can get the trajectories as well
                // local_planner->propagate(tree[start_vertex]->point, cast_query->plan, cast_query->path);
            }

            bool multi_modal_forward_tp_t::state_satisfies_goal( space_point_t* state )
            {
                motion_planning_query_t* mp_query = dynamic_cast<motion_planning_query_t*>(input_query);
                if( global_state_space->distance( state, mp_query->get_goal()->get_goal_points()[0] ) + object_cost_to_go( state ) <= PRX_DISTANCE_CHECK )
                    return true;
                return false;
            }


            bool multi_modal_forward_tp_t::extend( const tree_vertex_index_t& tree_node )
            {
                // PRX_DEBUG_COLOR("===============", PRX_TEXT_CYAN);
                // PRX_DEBUG_COLOR(" = In Extend = ", PRX_TEXT_LIGHTGRAY);
                // PRX_DEBUG_COLOR("===============", PRX_TEXT_CYAN);

                //First, let's just get the high-level node this tree node is in
                search_node_t* search_node = tree.get_vertex_as< search_node_t >( tree_node );
                automaton_node_t* auto_node = search_node->auto_state;

                search_node->expansions++;
                ++expansions;
                bool successful_expand = false;

                //If the automaton node is null, this means we're in a no grasping situation, so iterate instead over stable grasp states
                if( auto_node == NULL )
                {
                    // PRX_DEBUG_COLOR("No node, must approach.", PRX_TEXT_CYAN );
                    //For all of the stable poses
                    foreach( const undirected_vertex_index_t& vi, stable_pose_vis )
                    {
                        //Get the automaton node with all the goodies.
                        automaton_node_t* adj_node = automaton.get_vertex_as< automaton_node_t >( vi );
                        // PRX_DEBUG_COLOR("> Branch:", PRX_TEXT_LIGHTGRAY);
                        if( generate_branch( auto_node, search_node, adj_node ) )
                            successful_expand = true;
                    }
                }
                else
                {
                    // PRX_DEBUG_COLOR("Have a node, must check.", PRX_TEXT_MAGENTA );
                    //Need to keep a list of indices we have visited
                    std::vector< undirected_vertex_index_t > visited;
                    bool do_extend = true;
                    //For each adjacent node
                    foreach( const undirected_vertex_index_t& vi, boost::adjacent_vertices( auto_node->index, automaton.graph ) )
                    {
                        //Begin by assuming we will extend this direction
                        do_extend = true;
                        //Then, search through everything that has been visited
                        for( unsigned i=0; i<visited.size() && do_extend; ++i )
                        {
                            //If it has been visited
                            if( vi == visited[i] )
                            {
                                //Don't do the extend
                                do_extend = false;
                            }
                        }
                        if( do_extend )
                        {
                            //Add this vertex index to the list of things we have visited
                            visited.push_back( vi );
                            //Get the automaton node with all the goodies.
                            automaton_node_t* adj_node = automaton.get_vertex_as< automaton_node_t >( vi );
                            // PRX_DEBUG_COLOR("> Branch: " << vi << " => " << adj_node->get_arms()[0][0], PRX_TEXT_LIGHTGRAY);
                            if( generate_branch( auto_node, search_node, adj_node ) )
                                successful_expand = true;
                        }
                    }
                    //Then, a special case: we have to try reaching a state where nobody grasps
                    // now IF this extension is happening from a stable pose, and it is time to
                    // depart (i.e. object is at the goal)
                    if( auto_node->get_arms().size() == 1 && object_cost_to_go( search_node->point ) <= PRX_DISTANCE_CHECK )
                    {
                        // PRX_DEBUG_COLOR("> Special Branch: ", PRX_TEXT_RED);
                        if( generate_branch( auto_node, search_node, NULL ) )
                            successful_expand = true;
                        // PRX_DEBUG_COLOR(">>", PRX_TEXT_RED);
                    }
                }
                return successful_expand;
            }

            bool multi_modal_forward_tp_t::generate_branch( automaton_node_t* auto_node, search_node_t* search_node, automaton_node_t* adj_node )
            {
                //If we are planning during extension
                if( !delayed_planning )
                {
                    //Since we already know where we are planning from, set the state.
                    global_state_space->copy_from_point( search_node->point );
                    //Then, perform the planning, and see what we find
                    if( plan_top( search_node->record_index, auto_node, adj_node ) )
                    {
                        //If we're doing discrete search
                        if( discrete_search )
                        {
                            //Then, do the branching logic for discrete search
                            branch_discrete( search_node, adj_node );
                        }
                        //If we are doing forward search, simply expand the tree
                        else
                        {
                            //Add this new node to the tree with the right plans
                            add_node_to_tree( search_node->get_index(), adj_node );
                        }
                        return true;
                    }
                }
                else
                {
                    //Just add this new node to the tree...?  I don't even know man.
                }
                return false;
            }

            void multi_modal_forward_tp_t::add_node_to_tree( const tree_vertex_index_t& parent, automaton_node_t* adj_node )
            {
                // First, just add it to the tree
                tree_vertex_index_t v = tree.add_vertex< search_node_t, rrt_edge_t >();
                tree_vis.push_back( v );

                //Need to copy the state that we ultimately will end up at into the tree points
                global_state_space->copy_to_point( pre_alloced_points[point_number] );
                tree[v]->point = pre_alloced_points[point_number];

                //Next, check if it is a goal
                if( state_satisfies_goal( tree[v]->point ) )
                {
                    //Need to store the goal vertex index
                    goal_vertex = v;
                    //and report success
                    reached_goal = true;
                    //Also, get that time
                    search_time = clock.measure();
                }

                //Currently not using a metric, so let's not bother
                // metric->add_point(tree[v]);

                //Get the search node
                search_node_t* search_node = tree.get_vertex_as< search_node_t >(v);

                //Figure out the index for the heurisitcs
                if( adj_node != NULL )
                {
                    search_node->heuristic = heuristics[ get_heuristic_index( adj_node ) ];
                }
                //Otherwise, we're departing, so we must be winning
                else
                {
                    search_node->heuristic = 0;
                }

                search_node->cost = tree.get_vertex_as< search_node_t >(parent)->cost + final_plan.length();
                search_node->auto_state = adj_node;
                search_node->record_index = reached_record_index;

                //Then, if this search node has the goal position for the object
                if( object_distance( reached_record->object_pose, object_goal_state ) < PRX_DISTANCE_CHECK )
                {
                    search_node->at_goal = true;
                }

                //So, we have the reached record index, so set that record to discovered
                if( adj_node != NULL )
                {
                    adj_node->discover_record( reached_record_index );
                    //And since we must have reached a record, let it know what the search node it belongs to is
                    reached_record->search_node = search_node;
                }

                //Finally, let's add the plan to the edge... will likely need to be more complex
                tree_edge_index_t e = tree.add_edge< rrt_edge_t >(parent, v);
                tree.get_edge_as< rrt_edge_t >(e)->plan = final_plan;
                ++point_number;

                // PRX_DEBUG_COLOR("Added node to the tree [" << point_number << "]", PRX_TEXT_LIGHTGRAY);
                // PRX_DEBUG_COLOR("State > " << global_state_space->print_point( pre_alloced_points[point_number-1], 3 ), PRX_TEXT_LIGHTGRAY );

                if(point_number == max_points)
                {
                    for(unsigned i=0;i<max_points;i++)
                    {
                        pre_alloced_points.push_back(global_state_space->alloc_point());
                    }
                    max_points*=2;
                }

            }

            void multi_modal_forward_tp_t::branch_discrete( search_node_t* search_node, automaton_node_t* adj_node )
            {
                // PRX_DEBUG_COLOR("Branching discrete: " << adj_node, PRX_TEXT_LIGHTGRAY);
                //If we are branching to a node of some sort
                if( adj_node != NULL )
                {
                    //If the record is already in the tree, we just need to potentially update ALL the things
                    if( reached_record->discovered )
                    {
                        // PRX_DEBUG_COLOR("We think this record was previously discovered.", PRX_TEXT_GREEN);
                        //First, let's figure out how good the path even is
                        double new_cost = search_node->cost + final_plan.length();

                        //Then, we have to get the search node (&cost) of the node which is already in the tree
                        search_node_t* reached_node = reached_record->search_node;
                        double old_cost = reached_node->cost;

                        // PRX_DEBUG_COLOR("Found cost: " << new_cost << "   compared to old cost: " << old_cost, PRX_TEXT_LIGHTGRAY);
                        //Then, if the cost is better
                        if( new_cost < old_cost )
                        {
                            //Basically, need to rewire the tree
                            tree.transplant( reached_node->get_index(), search_node->get_index() );
                            PRX_DEBUG_COLOR("Rewire reduced cost: " << old_cost << " -> " << new_cost << "   : " << old_cost - new_cost, PRX_TEXT_LIGHTGRAY);

                            //And update costs & queue for all of the children
                            reduce_branch_cost( reached_node->get_index(), old_cost - new_cost );
                        }
                    }
                    //Otherwise, we just need to add the node to the tree and update the queue
                    else
                    {
                        // PRX_DEBUG_COLOR("It's a brand-new node!", PRX_TEXT_BROWN);
                        //Add it
                        add_node_to_tree( search_node->get_index(), adj_node );
                        //Now, this should be a unique state in the search, so just add it
                        open_set.insert( tree.get_vertex_as< search_node_t >( tree_vis.back() ) );
                    }
                }
                //Otherwise, we must be departing the graph
                else
                {
                    //So, we're attempting to reach the goal, so, just add the node and be done?
                    add_node_to_tree( search_node->get_index(), adj_node );
                }
            }

            void multi_modal_forward_tp_t::reduce_branch_cost(tree_vertex_index_t v, double amount)
            {
                search_node_t* node = tree.get_vertex_as< search_node_t >(v);
                node->cost -= amount;
                //Then, update the node with its new cost
                open_set.update( node, node );
                foreach(tree_vertex_index_t child, tree[v]->get_children())
                {
                    reduce_branch_cost( child, amount );
                }
            }


            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // ========           Discrete Search           ==========
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================

            tree_vertex_index_t multi_modal_forward_tp_t::get_min_open_set()
            {
                return open_set.peek_min()->get_index();
            }

            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // ========           Forward Search            ==========
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================

            tree_vertex_index_t multi_modal_forward_tp_t::select_est_no_heuristic()
            {
                double total_weight = 0;
                double max_children = 0;
                double max_selection = 0;
                double max_order = 0;
                tree_vertex_index_t selected_node = start_vertex;
                std::vector<double> ind_weights;

                // PRX_DEBUG_COLOR("Selecting out of " << tree_vis.size() << " nodes.", PRX_TEXT_GREEN);
                for( unsigned i=0; i<tree_vis.size(); ++i )
                {
                    search_node_t* node = tree.get_vertex_as< search_node_t >( tree_vis[i] );
                    max_children = PRX_MAXIMUM( max_children, (double)node->get_children().size() );
                    max_selection = PRX_MAXIMUM( max_selection, node->expansions );
                }
                max_order = tree_vis.size();
                for( unsigned i=0; i<tree_vis.size(); ++i )
                {
                    search_node_t* node = tree.get_vertex_as< search_node_t >( tree_vis[i] );

                    double order = ((double)(i+1))/max_order;
                    double children = ((double)(node->get_children().size()+1)+0.00001)/(max_children+1.0);
                    double selections = (1.0+node->expansions)/(1.0+max_selection);

                    ind_weights.push_back( pow( order, order_weight )
                        / ( pow( children, valence_weight ) *
                            pow( selections, selection_weight ) ) );
                    total_weight+=ind_weights.back();
                    // PRX_DEBUG_COLOR("w[" << i << "]: " << ind_weights.back(), PRX_TEXT_LIGHTGRAY);
                }
                double random_number = uniform_random()*total_weight;
                for( unsigned i=0; i<tree_vis.size(); ++i )
                {
                    search_node_t* node = tree.get_vertex_as< search_node_t >( tree_vis[i] );
                    random_number-=ind_weights[i];
                    if(random_number <= 0.0)
                    {
                        // if( node->auto_state != NULL )
                        // {
                        //     PRX_DEBUG_COLOR("Est selcted: " << i << "(" << node->auto_state->get_arms()[0][0] << ") :: " << node->record_index, PRX_TEXT_LIGHTGRAY);
                        // }
                        // else
                        // {
                        //     PRX_DEBUG_COLOR("Est selcted: " << i << "(root) :: " << node->record_index, PRX_TEXT_LIGHTGRAY);
                        // }
                        selected_node = node->get_index();
                        break;
                    }
                }
                return selected_node;
            }

            tree_vertex_index_t multi_modal_forward_tp_t::select_est()
            {
                double total_weight = 0;
                double max_children = 0;
                double max_selection = 0;
                double max_order = 0;
                double max_heuristic = 0;
                tree_vertex_index_t selected_node = start_vertex;
                std::vector<double> ind_weights;

                // PRX_DEBUG_COLOR("Selecting out of " << tree_vis.size() << " nodes.", PRX_TEXT_GREEN);
                for( unsigned i=0; i<tree_vis.size(); ++i )
                {
                    search_node_t* node = tree.get_vertex_as< search_node_t >( tree_vis[i] );
                    max_children = PRX_MAXIMUM( max_children, (double)node->get_children().size() );
                    max_heuristic = PRX_MAXIMUM( max_heuristic, object_cost_to_go( node->point ) );
                    max_selection = PRX_MAXIMUM( max_selection, node->expansions );
                }
                max_order = tree_vis.size();
                for( unsigned i=0; i<tree_vis.size(); ++i )
                {
                    search_node_t* node = tree.get_vertex_as< search_node_t >( tree_vis[i] );

                    double arms = 1.0;
                    if( node->auto_state != NULL )
                    {
                        arms = node->auto_state->get_arms().size();
                    }

                    ind_weights.push_back(pow( ((double)(i+1))/max_order, order_weight )
                        /(pow( ((double)(node->get_children().size()+1)+0.00001)/(max_children+1.0), valence_weight ) *
                        (pow( (object_cost_to_go(node->point)+0.00001)/max_heuristic, heuristic_weight )) *
                        (pow( (1.0+node->expansions)/(1.0+max_selection), selection_weight )) *
                        (pow( arms/2.0, arms_weight )) ));
                    total_weight+=ind_weights.back();
                    // PRX_DEBUG_COLOR("w[" << i << "]: " << ind_weights.back(), PRX_TEXT_LIGHTGRAY);
                }
                double random_number = uniform_random()*total_weight;
                for( unsigned i=0; i<tree_vis.size(); ++i )
                {
                    search_node_t* node = tree.get_vertex_as< search_node_t >( tree_vis[i] );
                    random_number-=ind_weights[i];
                    if(random_number <= 0.0)
                    {
                        // PRX_DEBUG_COLOR("Est selcted: " << i, PRX_TEXT_LIGHTGRAY);
                        selected_node = node->get_index();
                        break;
                    }
                }
                return selected_node;
            }

            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // ========              Heuristics             ==========
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================

            void multi_modal_forward_tp_t::compute_heuristics()
            {
                //Well, for our purposes, we can probably represent the problem with a matrix (?)
                size_t nr_nodes = all_vis.size();
                //Then, assume everything can't reach the goals
                heuristics.assign( nr_nodes, PRX_INFINITY );

                //Need a closed set
                std::vector< bool > closed;
                closed.assign( nr_nodes, false );

                //And an open set... a deque should be fine?
                std::deque< unsigned > open;

                //For each of our goals, we can assign a heuristic of 0
                for( unsigned i=0; i<goal_vertices.size(); ++i )
                {
                    //We need an index for this index... if that makes any sense
                    unsigned from_index = 0;
                    for( unsigned k=0; k<all_vis.size(); ++k )
                    {
                        if( all_vis[k] == goal_vertices[i] )
                        {
                            from_index = k;
                        }
                    }

                    //This is a goal, so it reaches with as much effort as a self-transition?
                    automaton_edge_t* self_edge = automaton.get_edge_as< automaton_edge_t >( goal_vertices[i], goal_vertices[i] );
                    heuristics[ from_index ] = self_edge->get_weight();

                    //clear out closed
                    closed.assign( nr_nodes, false );

                    //Set up open set
                    open.clear();
                    open.push_front( from_index );

                    //While there are still things to search
                    while( !open.empty() )
                    {
                        //Expand the thing from the open set
                        unsigned examined = open.back();
                        open.pop_back();
                        closed[examined] = true;

                        //Also, get its vertex index
                        undirected_vertex_index_t ext_index = all_vis[examined];

                        //We need to get its neighbors, holy crud
                        foreach( const undirected_vertex_index_t& vi, boost::adjacent_vertices( ext_index, automaton.graph ) )
                        {
                            //If it is not the same node, then we can compute things
                            if( vi != ext_index )
                            {
                                //We need an index for this index... if that makes any sense
                                unsigned neigh_index = 0;
                                for( unsigned k=0; k<all_vis.size(); ++k )
                                {
                                    if( all_vis[k] == vi )
                                    {
                                        neigh_index = k;
                                    }
                                }

                                //Need the edge between
                                automaton_edge_t* edge = automaton.get_edge_as< automaton_edge_t >( ext_index, vi );

                                //Get its weight
                                double edge_weight = edge->get_weight();

                                //Then, if the current heuristic is too large
                                if( heuristics[neigh_index] > heuristics[examined] + edge_weight )
                                {
                                    //Update it
                                    heuristics[neigh_index] = heuristics[examined] + edge_weight;

                                    //Then, if we haven't seen it before
                                    if( !closed[neigh_index] )
                                    {
                                        //Put it in the open
                                        open.push_front( neigh_index );
                                    }
                                }
                            }
                        }
                    }
                }
            }

            unsigned multi_modal_forward_tp_t::get_heuristic_index( automaton_node_t* adj_node )
            {
                for( unsigned i=0; i<all_vis.size(); ++i )
                {
                    if( all_vis[i] == adj_node->index )
                    {
                        return i;
                    }
                }
                PRX_FATAL_S("Error: could not find node heuristic.");
            }

            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // ========             I/O Functions           ==========
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================

            void multi_modal_forward_tp_t::serialize_motion_graphs()
            {
                //Serialize all of the move planners
                for( unsigned i=0; i<move_planners.size(); ++i )
                {
                    move_planners[i]->serialize();
                }
                //Serialize all of the transfer planners
                for( unsigned j=0; j<transfer_planners.size(); ++j )
                {
                    for( unsigned i=0; i<transfer_planners[j].size(); ++i )
                    {
                        transfer_planners[j][i]->serialize();
                    }
                }
            }

            void multi_modal_forward_tp_t::serialize_automaton()
            {
                char* w = std::getenv("PRACSYS_PATH");
                std::string dir(w);
                std::stringstream s1;
                s1 << "/prx_output/";
                dir += (s1.str());
                boost::filesystem::path output_dir(dir);
                if( !boost::filesystem::exists(output_dir) )
                {
                    boost::filesystem::create_directories(output_dir);
                }
                dir += "automaton.txt";
                std::ofstream fout;
                fout.open(dir.c_str());
                automaton.serialize( fout, object_space );
                fout.close();
            }

            void multi_modal_forward_tp_t::deserialize_automaton()
            {
                //DEserialize the automaton
                char* w = std::getenv("PRACSYS_PATH");
                std::string dir(w);
                std::stringstream s1;
                s1 << "/prx_input/";
                dir += (s1.str());
                boost::filesystem::path output_dir(dir);
                dir += "automaton.txt";

                //Seems weird, but need to get the right control space here?
                model->use_context( move_context_names[0] );
                space_t* control_space = model->get_control_space();

                PRX_PRINT("===========================", PRX_TEXT_RED);
                PRX_PRINT(" = Loading the automaton = ", PRX_TEXT_MAGENTA);
                PRX_PRINT("===========================", PRX_TEXT_RED);

                std::ifstream fin;
                if( automaton.deserialize<automaton_node_t, automaton_edge_t>( dir, fin, move_spaces, transfer_spaces, object_space, control_space ) )
                {
                    PRX_DEBUG_COLOR("Successfully read in automaton:", PRX_TEXT_GREEN);
                }
                else
                {
                    PRX_FATAL_S("Error reading automaton: aborting.");
                }
                fin.close();

                //Alright, now the real issue is that we have to reacquire all of the indices
                foreach( undirected_vertex_index_t v, boost::vertices(automaton.graph) )
                {
                    //Store the vertex index
                    all_vis.push_back( v );
                    //Then, let's get the node
                    automaton_node_t* node = automaton.get_vertex_as< automaton_node_t >( v );
                    //If it has more than one set of arms, it must be a handoff node
                    if( node->get_arms().size() > 1 )
                    {
                        handoff_pose_vis.push_back( v );
                    }
                    else
                    {
                        stable_pose_vis.push_back( v );
                    }
                }
                //Then, gather the edge indices
                foreach( undirected_edge_index_t e, boost::edges(automaton.graph) )
                {
                    edge_indices.push_back( e );
                }

                //Make sure we have all the edge records on the nodes
                add_edge_records_to_nodes();
            }

            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // ========          Utilitiy Functions          =========
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================

            void multi_modal_forward_tp_t::report_statistics()
            {
                motion_planning_query_t* cast_query = dynamic_cast< motion_planning_query_t* >( input_query );

                if( !reached_goal )
                {
                    PRX_DEBUG_COLOR("Planner did not reach goal, no stats to show.", PRX_TEXT_RED);
                    return;
                }

                PRX_DEBUG_COLOR("Reporting Statistics", PRX_TEXT_GREEN);

                char* w = std::getenv("PRACSYS_PATH");
                std::string dir(w);
                dir += ("/prx_output/mm_forward/");
                boost::filesystem::path output_dir(dir);
                if( !boost::filesystem::exists(output_dir) )
                {
                    boost::filesystem::create_directory(output_dir);
                }
                std::string outdir = dir;

                std::string filename = outdir + ros::this_node::getName() + "_";
                filename += "statistics.txt";
                std::ofstream fout;
                fout.open(filename.c_str());
                //Here is where we puke statistics
                fout << "C " << connect_time << std::endl;
                fout << "S " << search_time << std::endl;
                fout << "V " << point_number << std::endl;
                fout << "E " << expansions << std::endl;
                fout << "L " << cast_query->plan.length() << std::endl;
                fout << "T " << path_transitions << std::endl;
                fout << "D " << original_object_distance << std::endl;
                //No more puke
                fout.close();
                PRX_DEBUG_COLOR("Output the statistics...", PRX_TEXT_LIGHTGRAY);

                std::string directory(w);
                std::stringstream s1;
                s1 << "/prx_output/published_plans/";
                directory += (s1.str());
                boost::filesystem::path output_directory(directory);
                if( !boost::filesystem::exists(output_directory) )
                {
                    boost::filesystem::create_directories(output_directory);
                }
                directory += ros::this_node::getName() + "_plan.txt";

                cast_query->plan.save_to_file(directory);
                PRX_DEBUG_COLOR("As well as the plan...", PRX_TEXT_LIGHTGRAY);
            }

            void multi_modal_forward_tp_t::generate_timeout_file()
            {
                char* w = std::getenv("PRACSYS_PATH");
                std::string dir(w);
                dir += ("/prx_output/mm_forward/");
                boost::filesystem::path output_dir(dir);
                if( !boost::filesystem::exists(output_dir) )
                {
                    boost::filesystem::create_directory(output_dir);
                }
                std::string outdir = dir;

                std::string filename = outdir + ros::this_node::getName() + "_";
                filename += "timeout.txt";
                std::ofstream fout;
                fout.open(filename.c_str());
                //Here is where we puke statistics
                fout << "Sadness" << std::endl;
                //No more puke
                fout.close();
            }

            void multi_modal_forward_tp_t::go_to_safe( unsigned manip_index )
            {
                move_spaces[manip_index]->copy_from_point( safe_states[manip_index] );

                //Should also do this for the arm
                std::string old_context = model->get_current_context();
                model->use_context( move_context_names[manip_index] );

                model->get_control_space()->copy_from_point( safe_states[manip_index] );

                //Restore
                model->use_context( old_context );
            }

            void multi_modal_forward_tp_t::go_to_start()
            {
                //Alright, Just copy in the memory
                global_state_space->copy_from_point( global_start_state );
                set_zero_control();
            }

            void multi_modal_forward_tp_t::set_zero_control()
            {
                //Make sure we don't mess with contexts
                std::string old_context = model->get_current_context();

                //Go to the context of all the manipulators
                std::vector< double > vec;
                model->use_context(all_manips_context_name);
                model->get_state_space()->copy_to_vector( vec );
                model->get_control_space()->set_from_vector( vec );

                model->use_context( old_context );
            }

            void multi_modal_forward_tp_t::print_automaton()
            {
                PRX_DEBUG_COLOR("// =======================================================", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("// =======================================================", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("// =======================================================", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("// =======================================================", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("// ===......====......====..====..========================", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("// ===..====..==..====..==..====..========================", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("// ===..====..==..====..===..==..=========================", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("// ===......====......=======..===========================", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("// ===..========..====..===..==..=========================", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("// ===..========..====..==..====..========================", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("// ===..========..====..==..====..==........==============", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("// =======================================================", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("// =======================================================", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("// ===......====........==......====..====..====....======", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("// ===..===...==..========..====..==..====..==...==...====", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("// ===..====..==..========..====..==..====..==..====..====", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("// ===..====..==......====......====..====..==..==========", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("// ===..====..==..========..====..==..====..==..==....====", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("// ===..===...==..========..====..==...==...==...==...====", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("// ===......====........==......======....======....=.====", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("// =======================================================", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("// =======================================================", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("// =======================================================", PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("// =======================================================\n", PRX_TEXT_LIGHTGRAY);

                // Let's print up some statistics
                PRX_DEBUG_COLOR("Total Number of Nodes: " << boost::num_vertices(automaton.graph), PRX_TEXT_GREEN );
                PRX_DEBUG_COLOR("Total Number of Edges: " << boost::num_edges(automaton.graph), PRX_TEXT_GREEN );
                //Let's count our actual number of transitions we believe we can follow
                PRX_DEBUG_COLOR("Stable : " << stable_pose_vis.size(), PRX_TEXT_LIGHTGRAY);
                PRX_DEBUG_COLOR("Handoff: " << handoff_pose_vis.size(), PRX_TEXT_LIGHTGRAY);

                PRX_DEBUG_COLOR("===================", PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR(" = Stable States = ", PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR("===================", PRX_TEXT_GREEN);
                for (unsigned i = 0; i < stable_pose_vis.size(); ++i)
                {
                    automaton_node_t* node = automaton.get_vertex_as<automaton_node_t>(stable_pose_vis[i]);
                    PRX_DEBUG_COLOR("=> Node: " << node->node_id, PRX_TEXT_CYAN);
                    PRX_DEBUG_COLOR("=  Arms  =", PRX_TEXT_GREEN);
                    foreach( const std::vector<unsigned>& arms, node->get_arms() )
                    {
                        PRX_DEBUG_COLOR("==========", PRX_TEXT_GREEN);
                        for(unsigned k=0; k<arms.size(); ++k)
                        {
                            PRX_DEBUG_COLOR(":: " << arms[k], PRX_TEXT_LIGHTGRAY)
                        }
                    }

                    PRX_DEBUG_COLOR("= Bounds =", PRX_TEXT_RED);
                    PRX_DEBUG_COLOR("==========", PRX_TEXT_RED);
                    foreach( const bounds_t& bounds, node->get_bounds() )
                    {
                        PRX_DEBUG_COLOR(":: [" << bounds.get_lower_bound() << " , " << bounds.get_upper_bound() << "]", PRX_TEXT_LIGHTGRAY);
                    }

                    const std::vector< pose_record_t >& records = node->get_records();
                    PRX_DEBUG_COLOR("= Records = " << node->get_num_records(), PRX_TEXT_BROWN);
                    PRX_DEBUG_COLOR("===========", PRX_TEXT_BROWN);
                    for( unsigned k=0; k<node->get_num_records(); ++k )
                    {
                        const pose_record_t& record = records[k];
                        PRX_DEBUG_COLOR("Object Position: " << object_space->print_point(record.object_pose, 3) << "  (" << record.pose_id << ")", PRX_TEXT_BROWN );
                        PRX_DEBUG_COLOR("= Grasps = " << record.manipulator_indices.size(), PRX_TEXT_BLUE);
                        for( unsigned t=0; t<record.manipulator_indices.size(); ++t )
                        {
                            unsigned manip_index = record.manipulator_indices[t];
                            PRX_DEBUG_COLOR("========== (type: " << record.grasp_ids[t] << ")", PRX_TEXT_BLUE);
                            PRX_DEBUG_COLOR("Manip: " << manip_index, PRX_TEXT_CYAN);
                            PRX_DEBUG_COLOR("Grasped: " << transfer_spaces[manip_index]->print_point(record.states[t], 3) ,PRX_TEXT_LIGHTGRAY);
                            PRX_DEBUG_COLOR("Released: " << move_spaces[manip_index]->print_point(record.released[t], 3) ,PRX_TEXT_LIGHTGRAY);
                            PRX_DEBUG_COLOR("Retracted: " << move_spaces[manip_index]->print_point(record.retracted[t], 3) ,PRX_TEXT_LIGHTGRAY);
                            PRX_DEBUG_COLOR("Retraction Plan size: " << record.retraction_plans[t]->size(), PRX_TEXT_LIGHTGRAY);
                            PRX_DEBUG_COLOR("Approach Plan size: " << record.approach_plans[t]->size(), PRX_TEXT_LIGHTGRAY);
                        }
                    }
                }

                PRX_DEBUG_COLOR("====================", PRX_TEXT_MAGENTA);
                PRX_DEBUG_COLOR(" = Handoff States = ", PRX_TEXT_MAGENTA);
                PRX_DEBUG_COLOR("====================", PRX_TEXT_MAGENTA);
                for (unsigned i = 0; i < handoff_pose_vis.size(); ++i)
                {
                    automaton_node_t* node = automaton.get_vertex_as<automaton_node_t>(handoff_pose_vis[i]);
                    PRX_DEBUG_COLOR("Node: " << node->node_id, PRX_TEXT_CYAN);
                    PRX_DEBUG_COLOR("=  Arms  =", PRX_TEXT_GREEN);
                    foreach( const std::vector<unsigned>& arms, node->get_arms() )
                    {
                        PRX_DEBUG_COLOR("==========", PRX_TEXT_GREEN);
                        for(unsigned k=0; k<arms.size(); ++k)
                        {
                            PRX_DEBUG_COLOR(":: " << arms[k], PRX_TEXT_LIGHTGRAY)
                        }
                    }

                    PRX_DEBUG_COLOR("= Bounds =", PRX_TEXT_RED);
                    PRX_DEBUG_COLOR("==========", PRX_TEXT_RED);
                    foreach( const bounds_t& bounds, node->get_bounds() )
                    {
                        PRX_DEBUG_COLOR(":: [" << bounds.get_lower_bound() << " , " << bounds.get_upper_bound() << "]", PRX_TEXT_LIGHTGRAY);
                    }

                    const std::vector< pose_record_t >& records = node->get_records();
                    PRX_DEBUG_COLOR("= Records = " << node->get_num_records(), PRX_TEXT_BROWN);
                    PRX_DEBUG_COLOR("===========", PRX_TEXT_BROWN);
                    for( unsigned k=0; k<node->get_num_records(); ++k )
                    {
                        const pose_record_t& record = records[k];
                        PRX_DEBUG_COLOR("Object Position: " << object_space->print_point(record.object_pose, 3) << "  (" << record.pose_id << ")", PRX_TEXT_BROWN );
                        PRX_DEBUG_COLOR("= Grasps = " << record.manipulator_indices.size(), PRX_TEXT_BLUE);
                        for( unsigned t=0; t<record.manipulator_indices.size(); ++t )
                        {
                            unsigned manip_index = record.manipulator_indices[t];
                            PRX_DEBUG_COLOR("========== (type: " << record.grasp_ids[t] << ")", PRX_TEXT_BLUE);
                            PRX_DEBUG_COLOR("Manip: " << manip_index, PRX_TEXT_CYAN);
                            PRX_DEBUG_COLOR("Grasped: " << transfer_spaces[manip_index]->print_point(record.states[t], 3) ,PRX_TEXT_LIGHTGRAY);
                            PRX_DEBUG_COLOR("Released: " << move_spaces[manip_index]->print_point(record.released[t], 3) ,PRX_TEXT_LIGHTGRAY);
                            PRX_DEBUG_COLOR("Retracted: " << move_spaces[manip_index]->print_point(record.retracted[t], 3) ,PRX_TEXT_LIGHTGRAY);
                            PRX_DEBUG_COLOR("Retraction Plan size: " << record.retraction_plans[t]->size(), PRX_TEXT_LIGHTGRAY);
                            PRX_DEBUG_COLOR("Approach Plan size: " << record.approach_plans[t]->size(), PRX_TEXT_LIGHTGRAY);
                        }
                    }
                }

                PRX_DEBUG_COLOR("=================", PRX_TEXT_BLUE);
                PRX_DEBUG_COLOR(" =    Edges    = ", PRX_TEXT_BLUE);
                PRX_DEBUG_COLOR("=================", PRX_TEXT_BLUE);
                for(unsigned i=0; i<edge_indices.size(); ++i)
                {
                    //Which edge?
                    automaton_edge_t* edge = automaton.get_edge_as<automaton_edge_t>(edge_indices[i]);
                    if( edge == NULL )
                    {
                        continue;
                    }
                    PRX_DEBUG_COLOR("Edge: " << i, PRX_TEXT_RED);
                    //Also need to know what it is connected to
                    PRX_DEBUG_COLOR(":: " << edge->source_vertex << " <-> " << edge->target_vertex, PRX_TEXT_BLUE );

                    const std::vector< pose_record_t >& records = edge->get_records();
                    PRX_DEBUG_COLOR("= Records = " << edge->get_num_records(), PRX_TEXT_BROWN);
                    PRX_DEBUG_COLOR("===========", PRX_TEXT_BROWN);
                    for( unsigned k=0; k<edge->get_num_records(); ++k )
                    {
                        const pose_record_t& record = records[k];
                        PRX_DEBUG_COLOR("Object Position: " << object_space->print_point(record.object_pose, 3) << "  (" << record.pose_id << ")", PRX_TEXT_BROWN );
                        PRX_DEBUG_COLOR("= Grasps = " << record.manipulator_indices.size(), PRX_TEXT_BLUE);
                        for( unsigned t=0; t<record.manipulator_indices.size(); ++t )
                        {
                            unsigned manip_index = record.manipulator_indices[t];
                            PRX_DEBUG_COLOR("========== (type: " << record.grasp_ids[t] << ")", PRX_TEXT_BLUE);
                            PRX_DEBUG_COLOR("Manip: " << manip_index, PRX_TEXT_CYAN);
                            PRX_DEBUG_COLOR("Grasped: " << transfer_spaces[manip_index]->print_point(record.states[t], 3) ,PRX_TEXT_LIGHTGRAY);
                            PRX_DEBUG_COLOR("Released: " << move_spaces[manip_index]->print_point(record.released[t], 3) ,PRX_TEXT_LIGHTGRAY);
                            PRX_DEBUG_COLOR("Retracted: " << move_spaces[manip_index]->print_point(record.retracted[t], 3) ,PRX_TEXT_LIGHTGRAY);
                            PRX_DEBUG_COLOR("Retraction Plan size: " << record.retraction_plans[t]->size(), PRX_TEXT_LIGHTGRAY);
                            PRX_DEBUG_COLOR("Approach Plan size: " << record.approach_plans[t]->size(), PRX_TEXT_LIGHTGRAY);
                        }
                    }
                }
            }

            void multi_modal_forward_tp_t::add_edge_records_to_nodes()
            {
                PRX_DEBUG_COLOR("==================================", PRX_TEXT_BROWN);
                PRX_DEBUG_COLOR(" = Adding edge records to nodes = ", PRX_TEXT_RED);
                PRX_DEBUG_COLOR("==================================", PRX_TEXT_BROWN);

                //Alright, as a workaround, any records which are on edges should be replicated in corresponding nodes
                for( unsigned i=0; i<edge_indices.size(); ++i )
                {
                    //Begin by getting the information
                    automaton_edge_t* edge = automaton.get_edge_as< automaton_edge_t >( edge_indices[i] );
                    automaton_node_t* source = automaton.get_vertex_as< automaton_node_t >( all_vis[edge->source_vertex] );
                    automaton_node_t* target = automaton.get_vertex_as< automaton_node_t >( all_vis[edge->target_vertex] );
                    //Then, get the records on the edge
                    const std::vector< pose_record_t >& records = edge->get_records();

                    //Need some information
                    unsigned source_pose_id = 0;
                    unsigned target_pose_id = 0;

                    //Find the largest source pose ID
                    const std::vector< pose_record_t >& source_records = source->get_records();
                    for( unsigned k=0; k<source->get_num_records(); ++k )
                    {
                        source_pose_id = PRX_MAXIMUM( source_pose_id, source_records[k].pose_id );
                    }
                    ++source_pose_id;

                    //Find the largest target pose ID
                    const std::vector< pose_record_t >& target_records = target->get_records();
                    for( unsigned k=0; k<target->get_num_records(); ++k )
                    {
                        target_pose_id = PRX_MAXIMUM( target_pose_id, target_records[k].pose_id );
                    }
                    ++target_pose_id;

                    //Then, for each one of those records
                    for( unsigned k=0; k<edge->get_num_records(); ++k )
                    {
                        //For this particular edge record
                        const pose_record_t& record = records[k];

                        //Figure out what arm the source deals with
                        unsigned source_index = source->get_arms()[0][0];

                        unsigned new_source_record_id = source->add_record( record.object_pose, source_pose_id + record.pose_id );

                        // PRX_DEBUG_COLOR("Add record to [" << source_index << "]: " << record.object_pose, PRX_TEXT_LIGHTGRAY);

                        //Now, for each grasp on the edge which corresponds to the source node
                        for( unsigned j=0; j<record.manipulator_indices.size(); ++j )
                        {
                            if( record.manipulator_indices[j] == source_index )
                            {
                                //Add this grasp to the source
                                source->add_grasp_to_record( new_source_record_id, source_index, record.grasp_ids[j], record.states[j], record.released[j], record.retracted[j], record.retraction_plans[j], record.approach_plans[j] );
                            }
                        }

                        //Figure out what arm the target deals with
                        unsigned target_index = target->get_arms()[0][0];

                        unsigned new_target_record_id = target->add_record( record.object_pose, target_pose_id + record.pose_id );

                        // PRX_DEBUG_COLOR("Add record to [" << target_index << "]: " << record.object_pose, PRX_TEXT_LIGHTGRAY);
                        //Now, for each grasp on the edge which corresponds to the target node
                        for( unsigned j=0; j<record.manipulator_indices.size(); ++j )
                        {
                            if( record.manipulator_indices[j] == target_index )
                            {
                                //Add this grasp to the target
                                target->add_grasp_to_record( new_target_record_id, target_index, record.grasp_ids[j], record.states[j], record.released[j], record.retracted[j], record.retraction_plans[j], record.approach_plans[j] );
                            }
                        }
                    }

                    edge->clear_records_no_free();
                }
            }

            void multi_modal_forward_tp_t::append_plan(int manip_index, unsigned replications)
            {
                // PRX_DEBUG_COLOR("Appending plan (" << saved_plan.size() << ")", PRX_TEXT_GREEN);
                //Assuming someone actually saved things into the temporary plan, let's add it to the final plan
                if( saved_plan.size() == 0 )
                {
                    PRX_WARN_S("No plan was saved... nothing will be appended.");
                    return;
                }

                //Make sure we are careful about contexts
                std::string old_context = model->get_current_context();

                //Switch to the full space
                model->use_context( "full_space" );
                space_t* full_control_space = model->get_control_space();

                //Control point we'll end up using
                space_point_t* full_control = full_control_space->alloc_point();

                //Get the control space for that manipulator
                model->use_context( move_context_names[manip_index] );
                space_t* manipulator_space = model->get_control_space();

                for(unsigned k=0; k<replications; ++k)
                {
                    //For each control step in the saved plan,
                    for(unsigned i=0; i<saved_plan.size(); ++i)
                    {
                        //Set the plan step to that manipulator's control space
                        manipulator_space->copy_from_point( saved_plan[i].control );
                        //Then, derp, make sure we update the full control point
                        full_control_space->copy_to_point( full_control );
                        //Copy full control space into our point
                        final_plan.copy_onto_back( full_control, saved_plan[i].duration );
                    }
                    //Aesthetics: append extra copies of the last control
                    for( unsigned i=0; i<pause_frames; ++i )
                    {
                        final_plan.copy_onto_back( full_control, sim::simulation::simulation_step );
                    }
                }

                //I need a point to save the final state in
                model->use_context( all_manips_context_name );
                //At this point, we at least have the manipulator state, but the object...
                model->get_state_space()->copy_from_point( full_control );

                //Free the point
                full_control_space->free_point( full_control );

                //restore the context
                model->use_context(old_context);
            }

            void multi_modal_forward_tp_t::prepend_plan(int manip_index, unsigned replications)
            {
                //Make sure we are careful about contexts
                std::string old_context = model->get_current_context();

                //Switch to the full space
                model->use_context( "full_space" );
                space_t* full_control_space = model->get_control_space();

                //Control point we'll end up using
                space_point_t* full_control = full_control_space->alloc_point();

                //Get the control space for that manipulator
                model->use_context( move_context_names[manip_index] );
                space_t* manipulator_space = model->get_control_space();

                //Generate a copy of the final plan
                plan_t final_plan_copy( final_plan );
                //Clear out what we know as the final plan
                final_plan.clear();

                //Then, put all the stuff into the "final plan"
                for(unsigned k=0; k<replications; ++k)
                {
                    //For each control step in the saved plan,
                    for(unsigned i=0; i<saved_plan.size(); ++i)
                    {
                        //Set the plan step to that manipulator's control space
                        manipulator_space->copy_from_point( saved_plan[i].control );
                        //Then, derp, make sure we update the full control point
                        full_control_space->copy_to_point( full_control );
                        //Copy full control space into our point
                        final_plan.copy_onto_back( full_control, saved_plan[i].duration );
                    }
                    //Aesthetics: append extra copies of the last control
                    for( unsigned i=0; i<pause_frames; ++i )
                    {
                        final_plan.copy_onto_back( full_control, sim::simulation::simulation_step );
                    }
                }

                //Then, append the previous final plan information
                final_plan += final_plan_copy;

                //I need a point to save the final state in
                model->use_context( all_manips_context_name );
                //At this point, we at least have the manipulator state, but the object...
                model->get_state_space()->copy_from_point( full_control );

                //Free the point
                full_control_space->free_point( full_control );

                //restore the context
                model->use_context(old_context);
            }

            void multi_modal_forward_tp_t::append_grasp(int manip_index, unsigned replications, double grasping)
            {
                // PRX_DEBUG_COLOR("Appending a grasp WITH:: " << grasping, PRX_TEXT_GREEN);
                //Make sure we are careful about contexts
                std::string old_context = model->get_current_context();

                //Switch to the full space
                model->use_context( "full_space" );
                space_t* full_control_space = model->get_control_space();

                //Now, set the controls to be the END of the plan
                full_control_space->copy_from_point( final_plan.back().control );

                //Control point we'll end up using
                space_point_t* full_control = full_control_space->alloc_point();

                //Get the control space for that manipulator
                model->use_context( move_context_names[manip_index] );
                space_t* manipulator_space = model->get_control_space();
                space_point_t* manip_control = manipulator_space->alloc_point();

                //Get the manipulator space point
                manipulator_space->copy_to_point( manip_control );
                //Set the grasping
                manip_control->memory.back() = grasping;
                manipulator_space->copy_from_point( manip_control );
                //Then, copy this over to the full point
                full_control_space->copy_to_point( full_control );

                // PRX_DEBUG_COLOR("Appending grasping:", PRX_TEXT_CYAN);
                //Then, for as many steps as we want
                for( unsigned k=0; k<replications; ++k )
                {
                    //Just put this into the plan
                    final_plan.copy_onto_back( full_control, sim::simulation::simulation_step );
                    // PRX_DEBUG_COLOR(":: " << full_control_space->print_point( full_control, 2 ), PRX_TEXT_LIGHTGRAY );
                }

                //I need a point to save the final state in
                model->use_context( all_manips_context_name );
                //At this point, we at least have the manipulator state, but the object...
                model->get_state_space()->copy_from_point( full_control );

                //Free the points
                full_control_space->free_point( full_control );
                manipulator_space->free_point( manip_control );

                //restore the context
                model->use_context(old_context);
            }

            void multi_modal_forward_tp_t::prepend_grasp(int manip_index, unsigned replications, double grasping)
            {
                // PRX_DEBUG_COLOR("Prepending a grasp WITH:: " << grasping, PRX_TEXT_GREEN);
                //Make sure we are careful about contexts
                std::string old_context = model->get_current_context();

                //Switch to the full space
                model->use_context( "full_space" );
                space_t* full_control_space = model->get_control_space();

                //Now, set the controls to be the FRONT of the plan
                full_control_space->copy_from_point( final_plan[0].control );

                //Control point we'll end up using
                space_point_t* full_control = full_control_space->alloc_point();

                //Get the control space for that manipulator
                model->use_context( move_context_names[manip_index] );
                space_t* manipulator_space = model->get_control_space();
                space_point_t* manip_control = manipulator_space->alloc_point();

                //Get the manipulator space point
                manipulator_space->copy_to_point( manip_control );
                //Set the grasping
                manip_control->memory.back() = grasping;
                manipulator_space->copy_from_point( manip_control );
                //Then, copy this over to the full point
                full_control_space->copy_to_point( full_control );

                // PRX_DEBUG_COLOR("Prepending grasping:", PRX_TEXT_CYAN);
                //Then, for as many steps as we want
                for( unsigned k=0; k<replications; ++k )
                {
                    //Just put this into the plan
                    final_plan.copy_onto_front( full_control, sim::simulation::simulation_step );
                    // PRX_DEBUG_COLOR(":: " << full_control_space->print_point( full_control, 2 ), PRX_TEXT_LIGHTGRAY );
                }

                //I need a point to save the final state in
                model->use_context( all_manips_context_name );
                //At this point, we at least have the manipulator state, but the object...
                model->get_state_space()->copy_from_point( full_control );

                //Free the points
                full_control_space->free_point( full_control );
                manipulator_space->free_point( manip_control );

                //restore the context
                model->use_context(old_context);
            }

            bool multi_modal_forward_tp_t::is_same_grasp( unsigned manip_index, const pose_record_t& first_record, const pose_record_t& second_record )
            {
                unsigned findex = get_grasp_index( manip_index, first_record );
                unsigned sindex = get_grasp_index( manip_index, second_record );

                //Now, tell if they have the same grasp
                return first_record.grasp_ids[findex] == second_record.grasp_ids[sindex];
            }

            bool multi_modal_forward_tp_t::is_same_pose( const pose_record_t& first_record, const pose_record_t& second_record )
            {
                //We have to run under the assumption that these records could have come from different high-level states
                return is_same_pose( first_record.object_pose, second_record.object_pose );
            }

            bool multi_modal_forward_tp_t::is_same_pose( const util::space_point_t* first_pose, const util::space_point_t* second_pose, double epsilon )
            {
                for(unsigned i=0; i<first_pose->memory.size(); ++i)
                {
                    if( fabs(first_pose->memory[i] - second_pose->memory[i]) > epsilon )
                    {
                        return false;
                    }
                }
                return true;
            }

            unsigned multi_modal_forward_tp_t::get_grasp_index( unsigned manip_index, const pose_record_t& record )
            {
                for( unsigned i=0; i<record.manipulator_indices.size(); ++i )
                {
                    if( manip_index == record.manipulator_indices[i] )
                    {
                        return i;
                    }
                }
                PRX_FATAL_S("Requesting grasp index of a manipulator which is not part of this record!");
            }

            const std::vector< unsigned >& multi_modal_forward_tp_t::get_common_arms( automaton_node_t* node_a, automaton_node_t* node_b )
            {
                foreach( const std::vector< unsigned >& arms_a, node_a->get_arms() )
                {
                    foreach( const std::vector< unsigned >& arms_b, node_b->get_arms() )
                    {
                        if( arms_a == arms_b )
                        {
                            return arms_a;
                        }
                    }
                }
                PRX_FATAL_S("Requested common arms between two nodes which have no such common set!");
            }

            void multi_modal_forward_tp_t::non_common_arm_set( std::vector< unsigned >& result, automaton_node_t* node_a, automaton_node_t* node_b )
            {
                result.resize(0);

                //Okay, inefficient, but whatever, get the common arm set
                const std::vector< unsigned >& common_arms = get_common_arms( node_a, node_b );

                //Also get the individual arm sets
                const std::vector< std::vector< unsigned > >& first_arms = node_a->get_arms();
                const std::vector< std::vector< unsigned > >& second_arms = node_b->get_arms();

                //Now, go over the first set
                for( unsigned i=0; i<first_arms.size(); ++i )
                {
                    for( unsigned j=0; j<first_arms[i].size(); ++j )
                    {
                        //Get the arm
                        unsigned arm = first_arms[i][j];
                        bool in_common = false;
                        //Now, search to see if it is in the common set
                        for( unsigned k=0; k<common_arms.size() && !in_common; ++k )
                        {
                            if( arm == common_arms[k] )
                            {
                                in_common = true;
                            }
                        }
                        //Then if it is not in the set of common arms, it must be in the non-common set
                        if( !in_common )
                        {
                            result.push_back(arm);
                        }
                    }
                }

                //Same thing for the second set5
                for( unsigned i=0; i<second_arms.size(); ++i )
                {
                    for( unsigned j=0; j<second_arms[i].size(); ++j )
                    {
                        //Get the arm
                        unsigned arm = second_arms[i][j];
                        bool in_common = false;
                        //Now, search to see if it is in the common set
                        for( unsigned k=0; k<common_arms.size() && !in_common; ++k )
                        {
                            if( arm == common_arms[k] )
                            {
                                in_common = true;
                            }
                        }
                        //Then if it is not in the set of common arms, it must be in the non-common set
                        if( !in_common )
                        {
                            result.push_back(arm);
                        }
                    }
                }
            }

            bool multi_modal_forward_tp_t::have_common_arms( automaton_node_t* node_a, automaton_node_t* node_b )
            {
                foreach( const std::vector< unsigned >& arms_a, node_a->get_arms() )
                {
                    foreach( const std::vector< unsigned >& arms_b, node_b->get_arms() )
                    {
                        if( arms_a == arms_b )
                        {
                            return true;
                        }
                    }
                }
                return false;
            }

            bool multi_modal_forward_tp_t::index_is_common_manipulator( unsigned index, const std::vector< unsigned >& arms_a, const std::vector< unsigned >& arms_b )
            {
                //Variables
                bool found_in_first = false;

                //Try to find it in the first set (linear search because I'm a moob)
                for( unsigned i=0; i<arms_a.size() && !found_in_first; ++i )
                {
                    if( index == arms_a[i] )
                        found_in_first = true;
                }
                //If it is in the first, ensure it is in the second as well
                if(found_in_first)
                {
                    //Let's find it in the second
                    for( unsigned i=0; i<arms_b.size(); ++i )
                    {
                        if( index == arms_b[i] )
                            return true;
                    }
                }
                return false;
            }

            //Needed for Case 4
            void multi_modal_forward_tp_t::arm_union_setminus( std::vector< unsigned >& result, const std::vector< std::vector< unsigned > >& lhs, const std::vector< unsigned >& rhs )
            {
                //Clear out the result
                result.clear();

                //Then, union all the unsigneds in the lhs
                std::vector< unsigned > arm_union = lhs[0];
                //For each other set in the LHS
                for( unsigned i=1; i<lhs.size(); ++i )
                {
                    //Iterate over its arms
                    for( unsigned j=0; j<lhs[i].size(); ++j )
                    {
                        bool in_union = false;
                        unsigned arm = lhs[i][j];
                        //Then, for each arm in the union
                        for( unsigned k=0; k<arm_union.size() && !in_union; ++k )
                        {
                            //And if it is in the union
                            if( arm == arm_union[k] )
                            {
                                in_union = true;
                            }
                        }
                        //If it not in the union
                        if( !in_union )
                        {
                            //Put it in there.
                            arm_union.push_back(arm);
                        }
                    }
                }

                //For each arm in the lhs
                for( unsigned i=0; i<arm_union.size(); ++i )
                {
                    bool in_rhs = false;
                    //Look for this arm in the rhs
                    for( unsigned j=0; j<rhs.size() && !in_rhs; ++j )
                    {
                        if( arm_union[i] == rhs[j] )
                            in_rhs = true;
                    }
                    //If it is not in the right hand side, it goes into the result
                    if( !in_rhs )
                    {
                        result.push_back(arm_union[i]);
                    }
                }
            }

            bool multi_modal_forward_tp_t::have_interaction( const std::vector<unsigned>& arms, std::vector<bounds_t>& bounds, const undirected_graph_t& SIG, const std::vector< undirected_vertex_index_t >& map )
            {
                //IF there are no arms, there's also no interaction
                if(arms.size() == 0)
                    return false;

                //First, there must at least exist an edge between each arm in question... if even a single edge is absent, no interaction.
                for( unsigned i=0; i<arms.size(); ++i )
                {
                    for( unsigned k=i+1; k<arms.size(); ++k )
                    {
                        if( SIG.get_edge_as<sig_edge_t>(map[arms[i]], map[arms[k]]) == NULL )
                        {
                            return false;
                        }
                    }
                }

                //Let's also take care of the special case of n = k = 1
                if( arms.size() == 1 )
                {
                    //Retrieve its bounds
                    sig_node_t* node = SIG.get_vertex_as<sig_node_t>( map[arms[0]] );
                    bounds = node->get_bounds();
                    //And report there is interaction... with itself...?
                    return true;
                }
                bounds.resize(3);
                bounds[0].set_bounds(-PRX_INFINITY, PRX_INFINITY);
                bounds[1].set_bounds(-PRX_INFINITY, PRX_INFINITY);
                bounds[2].set_bounds(-PRX_INFINITY, PRX_INFINITY);

                //Now we know there is the POSSIBILITY of an interaction, we have to find the final bounds
                for( unsigned i=0; i<arms.size(); ++i )
                {
                    for( unsigned j=i+1; j<arms.size(); ++j )
                    {
                        //get the edge
                        sig_edge_t* edge = SIG.get_edge_as<sig_edge_t>(map[arms[i]], map[arms[j]]);
                        //And intersect the bounds to get the new bounds
                        util::bounds::intersect( bounds, bounds, edge->get_bounds() );
                    }
                }

                //Now that the bounds have been all intersected, verify their validity
                try
                {
                    util::bounds::verify( bounds );
                }
                catch( std::runtime_error e )
                {
                    //The bounds do not validate, report failure
                    return false;
                }

                return true;
            }


            bool multi_modal_forward_tp_t::shared_surface( undirected_vertex_index_t a, undirected_vertex_index_t b, std::vector< bounds_t >& bounds )
            {
                //Need to gather all of the arms between these two stable pose states
                std::vector< unsigned > shared_arms;
                //get the GDMF nodes.
                automaton_node_t* node_a = automaton.get_vertex_as<automaton_node_t>(a);
                automaton_node_t* node_b = automaton.get_vertex_as<automaton_node_t>(b);

                // PRX_DEBUG_COLOR("Checking for a shared surface between: " << node_a->node_id << " <> " << node_b->node_id, PRX_TEXT_BLUE );

                //First of all, we will get A's arms... since this should only be called on stable pose states, we are fine just getting the first (only) set
                shared_arms = node_a->get_arms()[0];
                foreach( const unsigned& arm, node_b->get_arms()[0] )
                {
                    bool in_structure = false;
                    for( unsigned i=0; i<shared_arms.size() && !in_structure; ++i )
                    {
                        if( shared_arms[i] == arm )
                        {
                            in_structure = true;
                        }
                    }
                    if(!in_structure)
                    {
                        shared_arms.push_back(arm);
                    }
                }

                // PRX_DEBUG_COLOR("Arms share: ", PRX_TEXT_CYAN);
                // for(unsigned i=0; i<shared_arms.size(); ++i)
                // {
                //     PRX_DEBUG_COLOR(":: " << shared_arms[i], PRX_TEXT_LIGHTGRAY);
                // }

                //Then, just verify that the arms can all interact
                return have_interaction(shared_arms, bounds, SIGs, SIGs_map );
            }

            bool multi_modal_forward_tp_t::shared_space( undirected_vertex_index_t a, undirected_vertex_index_t b, std::vector< bounds_t >& bounds )
            {
                //Again, just gather all of the arms between these two stable poses
                std::vector< unsigned > shared_arms;
                //get the GDMF nodes.
                automaton_node_t* node_a = automaton.get_vertex_as<automaton_node_t>(a);
                automaton_node_t* node_b = automaton.get_vertex_as<automaton_node_t>(b);

                //PRX_DEBUG_COLOR("Checking for a shared space near: " << node_a->node_id << " <> " << node_b->node_id, PRX_TEXT_RED );

                //First of all, we will get A's arms... since this should only be called on stable pose states, we are fine just getting the first (only) set
                shared_arms = node_a->get_arms()[0];
                foreach( const unsigned& arm, node_b->get_arms()[0] )
                {
                    bool in_structure = false;
                    for( unsigned i=0; i<shared_arms.size() && !in_structure; ++i )
                    {
                        if( shared_arms[i] == arm )
                        {
                            in_structure = true;
                        }
                    }
                    if(!in_structure)
                    {
                        shared_arms.push_back(arm);
                    }
                }

                // PRX_DEBUG_COLOR("Arms share: ", PRX_TEXT_CYAN);
                // for(unsigned i=0; i<shared_arms.size(); ++i)
                // {
                //     PRX_DEBUG_COLOR(":: " << shared_arms[i], PRX_TEXT_LIGHTGRAY);
                // }

                //Verify that they can interact.
                return have_interaction(shared_arms, bounds, SIGh, SIGh_map);
            }

            void multi_modal_forward_tp_t::set_object_pose_from_record( const pose_record_t& record )
            {
                object_space->copy_from_point( record.object_pose );
            }

            void multi_modal_forward_tp_t::set_arm_grasping_from_record( unsigned manip_index, const pose_record_t& record )
            {
                //First, make sure to get the correct grasping index for the record
                unsigned grasp_index = get_grasp_index( manip_index, record );
                //Then, just set the things
                transfer_spaces[manip_index]->copy_from_point(record.states[grasp_index]);
            }

            void multi_modal_forward_tp_t::set_arm_released_from_record( unsigned manip_index, const pose_record_t& record )
            {
                //First, make sure to get the correct grasping index for the record
                unsigned grasp_index = get_grasp_index( manip_index, record );
                //Then, just set the things
                move_spaces[manip_index]->copy_from_point(record.released[grasp_index]);
            }

            void multi_modal_forward_tp_t::set_arm_retracted_from_record( unsigned manip_index, const pose_record_t& record )
            {
                //First, make sure to get the correct grasping index for the record
                unsigned grasp_index = get_grasp_index( manip_index, record );
                //Then, just set the things
                move_spaces[manip_index]->copy_from_point(record.retracted[grasp_index]);
            }

            std::vector< std::vector< unsigned > > multi_modal_forward_tp_t::generate_combinations(unsigned in_k, const std::vector< unsigned >& max_cc)
            {
                std::vector< std::vector< unsigned > > ret;
                std::vector< std::vector< unsigned > > hold;

                //Error handling: If someone gives us an empty set of cc's.
                if( max_cc.size() == 0 )
                    return ret;

                //If we still have connected components to try to fill
                if( max_cc.size() > 1 )
                {
                    unsigned max_nr = PRX_MINIMUM(in_k, max_cc.back());
                    for( unsigned i = 0; i <= max_nr; ++i )
                    {
                        hold = generate_combinations(in_k - i, std::vector< unsigned >(max_cc.begin(), max_cc.end() - 1));
                        if( !hold.empty() )
                        {

                            foreach(std::vector< unsigned > u, hold)
                            {
                                u.push_back(i);
                                ret.resize(ret.size() + 1);
                                ret.back() = u;
                            }
                        }
                    }
                }
                //We have only one component to fill.
                else
                {
                    //If we're still supposed to assign some pebbles.
                    if( in_k > 0 )
                    {
                        //Now, if there's enough room to put the remaining pebbles
                        if( in_k <= max_cc[0] )
                        {
                                //Assign that many pebbles
                            ret.resize(1);
                            ret[0].push_back(in_k);
                            return ret;
                        }
                        //Otherwise, we can't make the assignment, return empty.
                        return ret;
                    }
                    //Otherwise, we have succesfully assigned all of the pebbles
                    else
                    {
                        //And, we didn't have to do any work here, so we report 0 new assignments.
                        ret.resize(1);
                        ret[0].push_back(0);
                        return ret;
                    }
                }

                return ret;
            }

            const util::statistics_t* multi_modal_forward_tp_t::get_statistics()
            {
                return NULL;
            }

            bool multi_modal_forward_tp_t::succeeded() const
            {
                return false;
            }

        }
    }
}
