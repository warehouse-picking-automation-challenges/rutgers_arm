/**
 * @file preprocess_manipulation_tp.cpp
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

#include "planning/task_planners/preprocess_manipulation_tp.hpp"
#include "planning/motion_planners/manipulation_mp.hpp"
#include "planning/problem_specifications/manipulation_mp_specification.hpp"
#include "planning/problem_specifications/rearrangement_manipulation_specification.hpp"
#include "planning/modules/system_name_validity_checker.hpp"

#include "simulation/manipulator_simulator.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/math/configurations/bounds.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/spaces/space.hpp"

#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/motion_planners/prm_star/prm_star_statistics.hpp"

//Includes from manipulation package
#include "../../../baxter/simulation/plants/manipulator.hpp"
#include "../../../manipulation/planning/modules/pose.hpp"
#include "../../../manipulation/simulation/plants/movable_body_plant.hpp"
#include "../../../manipulation/planning/modules/samplers/manip_sampler.hpp"
#include "planning/problem_specifications/manipulation_specification.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <boost/range/adaptor/map.hpp>
#include <vector>
#include <bits/stl_list.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::rearrangement_manipulation::preprocess_manipulation_tp_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        using namespace baxter;
        using namespace manipulation;

        namespace rearrangement_manipulation
        {

            preprocess_manipulation_tp_t::preprocess_manipulation_tp_t()
            {
                poses_set_length = 0;

                stable_pose_memory.push_back(new double());
                stable_pose_memory.push_back(new double());
                stable_pose_space = new space_t("XY", stable_pose_memory);
                extra_pose_memory.push_back(new double());
                extra_pose_memory.push_back(new double());
                extra_pose_memory.push_back(new double());
                extra_pose_space = new space_t("XYZ", extra_pose_memory);

                _manipulator = NULL;
                _object = NULL;

                char* w = std::getenv("PRACSYS_PATH");

                prx_output_dir = std::string(w) + "/prx_output/rearrangement_graphs/";
                prx_input_dir = std::string(w) + "/prx_input/rearrangement_graphs/";

                rpg_graph_time = 0;

                manip_sampler = NULL;
                system_name_validity_checker = NULL;

                relative_configuration.set_position(0, 0, 0);
                relative_configuration.set_orientation(0, -0.707106, 0, 0.707106);
            }

            preprocess_manipulation_tp_t::~preprocess_manipulation_tp_t()
            {
                manip_state_space->free_point(manip_state);
                manip_state_space->free_point(safe_state);
                manip_state_space->free_point(released_point);
                manip_state_space->free_point(retracted_point);
                manip_state_space->free_point(manip_grasping_point);

                mo_space->free_point(grasped_point);
                mo_space->free_point(grasped_retracted_point);

                object_state_space->free_point(object_state);
                object_state_space->free_point(object_moved_state);                
                full_collision_objects_space->free_point(full_collision_check_state);

                stable_pose_space->free_point(stable_pose_state);
                extra_pose_space->free_point(extra_pose_state);

                manip_control_space->free_point(manip_ctrl);
                manip_control_space->free_point(safe_control);

                tmp_path.clear();
                transit_reaching_plan.clear();
                transfer_reaching_plan.clear();
                transit_retract_plan.clear();
                transfer_retract_plan.clear();

                poses_set.clear();
            }

            void preprocess_manipulation_tp_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                task_planner_t::init(reader, template_reader);
                PRX_INFO_S("Initializing Manipulation task planner ...");

                pc_name_manipulator_only = parameters::get_attribute("pc_name_manipulator_only", reader, template_reader, "pc_name_manipulator_only");
                pc_name_object_only = "pc_name_object_only";
                pc_name_objects = "pc_name_objects";
                pc_name_manipulator_with_object = "pc_name_manipulator_with_object";
                pc_name_grasp_checks = "pc_name_grasp_checks";
                pc_name_transit = "pc_name_transit";
                pc_name_transfer = "pc_name_transfer";

                if( parameters::has_attribute("transit_motion_planner_name", reader, template_reader) )
                {
                    transit_motion_planner_name = parameters::get_attribute("transit_motion_planner_name", reader, template_reader);
                }
                else
                {
                    PRX_FATAL_S("transit_motion_planner_name is missing!");
                }

                if( parameters::has_attribute("transfer_motion_planner_name", reader, template_reader) )
                {
                    transfer_motion_planner_name = parameters::get_attribute("transfer_motion_planner_name", reader, template_reader);
                }
                else
                {
                    PRX_FATAL_S("transfer_motion_planner_name is missing!");
                }

                if( parameters::has_attribute("transit_manipulation_mp_name", reader, template_reader) )
                {
                    transit_manipulation_mp_name = parameters::get_attribute("transit_manipulation_mp_name", reader, template_reader);
                }
                else
                {
                    PRX_FATAL_S("transit_manipulation_mp_name is missing!");
                }

                if( parameters::has_attribute("transfer_manipulation_mp_name", reader, template_reader) )
                {
                    transfer_manipulation_mp_name = parameters::get_attribute("transfer_manipulation_mp_name", reader, template_reader);
                }
                else
                {
                    PRX_FATAL_S("transfer_manipulation_mp_name is missing!");
                }

                const parameter_reader_t* specification_template_reader = NULL;
                if( parameters::has_attribute("transit_manipulation_specification", reader, template_reader) )
                {
                    PRX_DEBUG_COLOR("transit_manipulation_specification", PRX_TEXT_GREEN);
                    if( parameters::has_attribute("transit_manipulation_specification/template", reader, template_reader) )
                    {
                        specification_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + reader->get_attribute("transit_manipulation_specification/template"));
                    }
                    output_specifications[transit_manipulation_mp_name] = parameters::initialize_from_loader<specification_t > ("prx_planning", reader, "transit_manipulation_specification", specification_template_reader, "");

                    if( specification_template_reader != NULL )
                    {
                        delete specification_template_reader;
                        specification_template_reader = NULL;
                    }
                }
                else
                {
                    PRX_FATAL_S("Missing transit manipulation specification!!!");
                }

                if( parameters::has_attribute("transfer_manipulation_specification", reader, template_reader) )
                {
                    PRX_DEBUG_COLOR("transfer_manipulation_specification", PRX_TEXT_GREEN);
                    if( parameters::has_attribute("transfer_manipulation_specification/template", reader, template_reader) )
                    {
                        specification_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + reader->get_attribute("transfer_manipulation_specification/template"));
                    }
                    output_specifications[transfer_manipulation_mp_name] = parameters::initialize_from_loader<specification_t > ("prx_planning", reader, "transfer_manipulation_specification", specification_template_reader, "");

                    if( specification_template_reader != NULL )
                    {
                        delete specification_template_reader;
                        specification_template_reader = NULL;
                    }
                }
                else
                {
                    PRX_FATAL_S("Missing transfer manipulation specification!!!");
                }

                if( parameters::has_attribute("transit_motion_planner_specification", reader, template_reader) )
                {
                    PRX_DEBUG_COLOR("transit_motion_planner_specification", PRX_TEXT_GREEN);
                    if( parameters::has_attribute("transit_motion_planner_specification/template", reader, template_reader) )
                    {
                        specification_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + reader->get_attribute("transit_motion_planner_specification/template"));
                    }
                    output_specifications[transit_motion_planner_name] = parameters::initialize_from_loader<specification_t > ("prx_planning", reader, "transit_motion_planner_specification", specification_template_reader, "");

                    if( specification_template_reader != NULL )
                    {
                        delete specification_template_reader;
                        specification_template_reader = NULL;
                    }
                }
                else
                {
                    PRX_FATAL_S("Missing transit motion planner's specification!!!");
                }

                if( parameters::has_attribute("transfer_motion_planner_specification", reader, template_reader) )
                {
                    PRX_DEBUG_COLOR("transfer_motion_planner_specification", PRX_TEXT_GREEN);
                    if( parameters::has_attribute("transfer_motion_planner_specification/template", reader, template_reader) )
                    {
                        specification_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + reader->get_attribute("transfer_motion_planner_specification/template"));
                    }
                    output_specifications[transfer_motion_planner_name] = parameters::initialize_from_loader<specification_t > ("prx_planning", reader, "transfer_motion_planner_specification", specification_template_reader, "");

                    if( specification_template_reader != NULL )
                    {
                        delete specification_template_reader;
                        specification_template_reader = NULL;
                    }
                }
                else
                {
                    PRX_FATAL_S("Missing transfer motion planner's specification!!!");
                }

                if( reader->has_attribute("stable_pose_space") )
                    stable_pose_space->init(reader->get_child("stable_pose_space").get());
                else if( template_reader != NULL )
                    stable_pose_space->init(template_reader->get_child("stable_pose_space").get());
                else
                    PRX_FATAL_S("Missing stable pose space for rearrangement rearrangement task planner!");

                if( reader->has_attribute("extra_pose_space") )
                    extra_pose_space->init(reader->get_child("extra_pose_space").get());
                else if( template_reader != NULL )
                    extra_pose_space->init(template_reader->get_child("extra_pose_space").get());
                else
                    extra_pose_space = NULL;

                if( parameters::has_attribute("manip_sampler", reader, template_reader) )
                    manip_sampler = static_cast<manip_sampler_t*>(parameters::initialize_from_loader<sampler_t > ("prx_planning", reader, "manip_sampler", template_reader, "manip_sampler"));
                else
                    PRX_FATAL_S("Missing sampler attribute for manipulation task planner!");

                if( parameters::has_attribute("manip_validity_checker", reader, template_reader) )
                {
                    system_name_validity_checker = dynamic_cast<system_name_validity_checker_t*>(parameters::initialize_from_loader<validity_checker_t > ("prx_planning", reader, "manip_validity_checker", template_reader, "manip_validity_checker"));
                    if( system_name_validity_checker == NULL )
                        PRX_FATAL_S("Preprocess manipulation task planner initialize a validity_checker that it is not system_name_validity_checker!");
                }
                else
                    PRX_FATAL_S("Missing system_name_validity_checker attribute for preprocess manipulation task planner!");

                transit_graph_file = parameters::get_attribute("transit_graph_file", reader, template_reader, "transit_graph.txt");
                transfer_graph_file = parameters::get_attribute("transfer_graph_file", reader, template_reader, "transfer_graph.txt");
                informed_transit_graph_file = parameters::get_attribute("informed_transit_graph_file", reader, template_reader, "informed_transit_graph.txt");
                informed_transfer_graph_file = parameters::get_attribute("informed_transfer_graph_file", reader, template_reader, "informed_transfer_graph.txt");
                poses_file = parameters::get_attribute("poses_file", reader, template_reader, "poses.txt");
                poses_constraints_file = parameters::get_attribute("poses_constraints_file", reader, template_reader, "poses_constraints_file.txt");
                store_poses_file = parameters::get_attribute("store_poses_file", reader, template_reader, "store_poses.txt");

                if( parameters::has_attribute("fix_poses", reader, template_reader) )
                {

                    foreach(const parameter_reader_t* s_reader, parameters::get_list("fix_poses", reader, template_reader))
                    {

                        fix_poses.push_back(s_reader->get_attribute_as<std::vector<double> >("pose"));
                    }
                }

                if( parameters::has_attribute("cheating_poses", reader, template_reader) )
                {

                    foreach(const parameter_reader_t* s_reader, parameters::get_list("cheating_poses", reader, template_reader))
                    {

                        cheating_poses.push_back(s_reader->get_attribute_as<std::vector<double> >("pose"));
                    }
                }

                min_theta = parameters::get_attribute_as<double>("min_theta", reader, template_reader, 1.5);
                max_theta = parameters::get_attribute_as<double>("max_theta", reader, template_reader, 4.7);

                object_name = parameters::get_attribute("object_name", reader, template_reader, "simulator/cup");

                add_grasped_seed = parameters::get_attribute_as<bool>("add_grasped_seed", reader, template_reader, true);

            }

            void preprocess_manipulation_tp_t::reset()
            {
                PRX_FATAL_S("Reset is not implemented for preprocess manipulation tp!");
            }

            void preprocess_manipulation_tp_t::link_specification(specification_t* new_spec)
            {
                task_planner_t::link_specification(new_spec);
                specs = static_cast<rearrangement_manipulation_specification_t*>(new_spec);
                retraction_config.set_position(0, 0, -specs->retract_distance);
                retraction_config.set_xyzw_orientation(0, 0, 0, 1);

            }

            void preprocess_manipulation_tp_t::setup()
            {
                PRX_DEBUG_COLOR("Setup preprocess_manipulation_tp ...", PRX_TEXT_CYAN);

                detect_plants();

                if( _manipulator == NULL )
                    PRX_FATAL_S("You need at least one manipulator for the project to work!");
                if( _object == NULL )
                    PRX_FATAL_S("You need at least a movable object for the rearrangement project!");

                context_flags true_flags(true, true);
                context_flags active_flags(true, false);
                context_flags inactive_flags(false, false);

                util::hash_t<std::string, context_flags> mappings;
                mappings[_manipulator->get_pathname()] = true_flags;
                mappings[object_name] = inactive_flags;
                model->create_new_planning_context(pc_name_transit, mappings, active_flags);

                mappings[object_name] = active_flags;
                model->create_new_planning_context(pc_name_grasp_checks, mappings);

                mappings[object_name].plannable = true;
                model->create_new_planning_context(pc_name_manipulator_with_object, mappings);

                model->create_new_planning_context(pc_name_transfer, mappings, active_flags);


                mappings[_manipulator->get_pathname()] = inactive_flags;
                model->create_new_planning_context(pc_name_object_only, mappings);

                model->create_new_planning_context(pc_name_objects, mappings, active_flags);

                //Initializing the spaces.
                model->use_context(pc_name_manipulator_with_object);
                mo_space = model->get_state_space();

                model->use_context(pc_name_object_only);
                object_state_space = model->get_state_space();


                model->use_context(pc_name_manipulator_only);
                manip_state_space = model->get_state_space();
                manip_control_space = model->get_control_space();

                model->use_context(pc_name_objects);
                full_collision_objects_space = model->get_active_space();

                //Allocating the helping state/control point variables.
                manip_state = manip_state_space->alloc_point();
                safe_state = manip_state_space->alloc_point();
                released_point = manip_state_space->alloc_point();
                retracted_point = manip_state_space->alloc_point();
                manip_grasping_point = manip_state_space->alloc_point();
                grasped_point = mo_space->alloc_point();
                grasped_retracted_point = mo_space->alloc_point();
                object_state = object_state_space->alloc_point();
                object_moved_state = object_state_space->alloc_point();
                stable_pose_state = stable_pose_space->alloc_point();
                extra_pose_state = extra_pose_space->alloc_point();
                full_collision_check_state = full_collision_objects_space->alloc_point();

                manip_ctrl = manip_control_space->alloc_point();
                safe_control = manip_control_space->alloc_point();

                manip_control_vec.resize(manip_control_space->get_dimension());

                tmp_path.link_space(manip_state_space);                
                transit_reaching_plan.link_control_space(manip_control_space);
                transfer_reaching_plan.link_control_space(manip_control_space);
                transit_retract_plan.link_control_space(manip_control_space);
                transfer_retract_plan.link_control_space(manip_control_space);

                poses_set_length = 0;

                manip_sampler->link_info(_manipulator, manip_state_space, object_state_space, mo_space);

                input_specification->link_spaces(manip_state_space, manip_control_space);
                input_specification->setup(model);
                manip_state_space->set_from_vector(specs->safe_position, safe_state);
                manip_control_space->set_from_vector(specs->safe_position, safe_control);

                system_name_validity_checker->setup_checker(_manipulator, _object->get_pathname());

                graph_specification = dynamic_cast<motion_planning_specification_t*>(output_specifications[transit_motion_planner_name]);                
                dynamic_cast<manip_sampler_t*>(graph_specification->sampler)->link_info(_manipulator, manip_state_space, object_state_space, mo_space);
                graph_specification->link_spaces(manip_state_space, manip_control_space);
                graph_specification->setup(model);
                graph_specification->get_stopping_criterion()->link_motion_planner((motion_planner_t*)planners[transit_motion_planner_name]);

                graph_specification = dynamic_cast<motion_planning_specification_t*>(output_specifications[transfer_motion_planner_name]);
                dynamic_cast<manip_sampler_t*>(graph_specification->sampler)->link_info(_manipulator, manip_state_space, object_state_space, mo_space);
                graph_specification->link_spaces(mo_space, manip_control_space);
                graph_specification->setup(model);
                graph_specification->get_stopping_criterion()->link_motion_planner((motion_planner_t*)planners[transfer_motion_planner_name]);

                manipulation_specification = dynamic_cast<manipulation_mp_specification_t*>(output_specifications[transit_manipulation_mp_name]);
                manipulation_specification->validity_checker = system_name_validity_checker;
                dynamic_cast<manip_sampler_t*>(manipulation_specification->sampler)->link_info(_manipulator, manip_state_space, object_state_space, mo_space);
                PRX_ASSERT(!dynamic_cast<manip_sampler_t*>(manipulation_specification->sampler)->is_transfer_mode());
                manipulation_specification->_manipulator = _manipulator;
                //The object state space is being used for the collision checking because this is the only active
                //object during the transit state.                
                manipulation_specification->object_space = object_state_space;
                manipulation_specification->full_collision_object_space = full_collision_objects_space;
                manipulation_specification->graph_deserialization_file = prx_output_dir + transit_graph_file;
                manipulation_specification->serialization_file = prx_input_dir + informed_transit_graph_file;

                manipulation_specification->link_spaces(manip_state_space, manip_control_space);
                manipulation_specification->setup(model);
                PRX_ASSERT(manipulation_specification->is_builder);
                PRX_ASSERT(!manipulation_specification->transfer_mode);

                manipulation_specification = dynamic_cast<manipulation_mp_specification_t*>(output_specifications[transfer_manipulation_mp_name]);
                manipulation_specification->validity_checker = system_name_validity_checker;
                dynamic_cast<manip_sampler_t*>(manipulation_specification->sampler)->link_info(_manipulator, manip_state_space, object_state_space, mo_space);
                PRX_ASSERT(dynamic_cast<manip_sampler_t*>(manipulation_specification->sampler)->is_transfer_mode());
                manipulation_specification->_manipulator = _manipulator;
                manipulation_specification->object_space = object_state_space;
                manipulation_specification->full_collision_object_space = full_collision_objects_space;
                manipulation_specification->graph_deserialization_file = prx_output_dir + transfer_graph_file;
                manipulation_specification->serialization_file = prx_input_dir + informed_transfer_graph_file;
                PRX_ASSERT(manipulation_specification->is_builder);
                PRX_ASSERT(manipulation_specification->transfer_mode);
                manipulation_specification->link_spaces(mo_space, manip_control_space);
                manipulation_specification->setup(model);
            }

            bool preprocess_manipulation_tp_t::execute()
            {
                const prm_star_statistics_t* statistics;

                //////////////////////////////////////
                //           Create Poses.          //
                //////////////////////////////////////
                if( cheating_poses.size() != 0 )
                    compute_cheating_seeds();

                PRX_DEBUG_COLOR("Going to compute extra poses", PRX_TEXT_GREEN);
                compute_extra_poses();

                //Compute the random poses.
                PRX_DEBUG_COLOR("Going to generate the fixed poses plus compute random poses", PRX_TEXT_GREEN);
                create_poses_set();

                /////////////////////////////////////////////////////////
                //    Build Graphs from Common Motion Planner.         //
                /////////////////////////////////////////////////////////
                //Building the Transit graph.
                PRX_DEBUG_COLOR("Building the Transit graph.", PRX_TEXT_BROWN);
                model->use_context(pc_name_manipulator_only);
                graph_specification = dynamic_cast<motion_planning_specification_t*>(output_specifications[transit_motion_planner_name]);
                graph_specification->clear();

                for( unsigned i = 0; i < poses_set_length; ++i )
                {

                    if(add_grasped_seed)
                    {
                        foreach(state_t* state, poses_set[i].ungrasped_set)
                        {
                            graph_specification->add_seed(state);
                        }
                    }

                    foreach(state_t* state, poses_set[i].retracted_set)
                    {
                        graph_specification->add_seed(state);
                    }

                }
                if( cheating_transit_seeds.size() != 0 )
                {

                    foreach(state_t* state, cheating_transit_seeds)
                    {
                        graph_specification->add_seed(state);
                    }
                }
                graph_specification->add_seed(safe_state);

                motion_planner_t* motion_planner;
                motion_planner = dynamic_cast<motion_planner_t*>(planners[transit_motion_planner_name]);
                motion_planner->link_specification(graph_specification);
                motion_planner->set_param("", "serialize_flag", true);
                motion_planner->set_param("", "serialization_file", "rearrangement_graphs/" + transit_graph_file);
                motion_planner->setup();

                
                try
                {
                    motion_planner->execute();
                }
                catch( stopping_criteria_t::stopping_criteria_satisfied e )
                {
                    statistics = dynamic_cast<const prm_star_statistics_t*>(motion_planner->get_statistics());
                    PRX_DEBUG_COLOR("Transit Graph statistics : |seed| : " << graph_specification->get_seeds().size() << "    (|V|,|E|,|cc|): " << statistics->get_statistics(), PRX_TEXT_GREEN);

                }

                motion_planner->serialize();
                motion_planner->update_visualization();


                //Building the Transfer graph.
                PRX_DEBUG_COLOR("Building the Transfer graph.", PRX_TEXT_BROWN);
                model->use_context(pc_name_manipulator_with_object);
                graph_specification = dynamic_cast<motion_planning_specification_t*>(output_specifications[transfer_motion_planner_name]);
                graph_specification->clear();
                for( unsigned i = 0; i < poses_set_length; ++i )
                {
                    if(add_grasped_seed)
                    {
                        foreach(state_t* state, poses_set[i].grasped_set)
                        {
                            graph_specification->add_seed(state);
                        }
                    }

                    foreach(state_t* state, poses_set[i].grasped_retracted_set)
                    {
                        graph_specification->add_seed(state);
                        // PRX_DEBUG_COLOR("grasped_retract: " << mo_space->print_point(state, 8), PRX_TEXT_BROWN);
                    }
                    //                    PRX_ASSERT(false);
                }
                if( cheating_transfer_seeds.size() != 0 )
                {

                    foreach(state_t* state, cheating_transfer_seeds)
                    {
                        graph_specification->add_seed(state);
                    }
                }

                motion_planner = dynamic_cast<motion_planner_t*>(planners[transfer_motion_planner_name]);
                motion_planner->link_specification(graph_specification);
                motion_planner->set_param("", "serialize_flag", true);
                motion_planner->set_param("", "serialization_file", "rearrangement_graphs/" + transfer_graph_file);
                motion_planner->setup();

                try
                {
                    motion_planner->execute();
                }
                catch( stopping_criteria_t::stopping_criteria_satisfied e )
                {
                    statistics = dynamic_cast<const prm_star_statistics_t*>(motion_planner->get_statistics());
                    PRX_DEBUG_COLOR("Transfer Graph statistics : |seed| : " << graph_specification->get_seeds().size() << "    (|V|,|E|,|cc|): " << statistics->get_statistics(), PRX_TEXT_GREEN);

                }
                motion_planner->serialize();
                motion_planner->update_visualization();


                //////////////////////////////////////////
                //           Build Object Map.          //
                //////////////////////////////////////////

                hash_t<std::string, unsigned> object_map;
                unsigned pl = 0;
                unsigned size = object_state_space->get_dimension();

                //                for( unsigned i = 0; i < pose_seeds.size(); ++i )
                //                    for( unsigned j = 0; j < size; ++j, ++pl )
                //                        full_state_vec[pl] = pose_seeds[i].second->at(j);
                //                full_collision_objects_space->set_from_vector(full_state_vec);

                model->use_context(pc_name_objects);

                std::vector<double> object_vec(object_state_space->get_dimension());
                for( unsigned i = 0; i < pose_seeds.size(); ++i )
                {
                    std::vector<double> full_state_vec(full_collision_objects_space->get_dimension());
                    unsigned pl = i*size;
                    for( unsigned j = 0; j < size; ++j )
                        full_state_vec[pl + j] = pose_seeds[i].second->at(j);
                    full_collision_objects_space->set_from_vector(full_state_vec);
                    //                    PRX_DEBUG_COLOR("full size: " << full_collision_objects_space->get_dimension(), PRX_TEXT_BROWN);
                    //                    PRX_DEBUG_COLOR(full_collision_objects_space->print_memory(10), PRX_TEXT_CYAN);
                    object_state_space->copy_point_to_vector(pose_seeds[i].second, object_vec);
                    object_vec[0] += 0.0001;
                    object_state_space->set_from_vector(object_vec);
                    std::string collide_object = system_name_validity_checker->collides_with();
                    PRX_DEBUG_COLOR("Collides with : " << collide_object, PRX_TEXT_GREEN);
                    PRX_DEBUG_COLOR(object_state_space->print_memory(10), PRX_TEXT_BROWN);
                    PRX_ASSERT(collide_object != "");
                    object_map[collide_object] = pose_seeds[i].first;
                }
                system_name_validity_checker->link_map(object_map);

                foreach(std::string name, object_map | boost::adaptors::map_keys)
                {
                    PRX_DEBUG_COLOR("map: " << name << "      id: " << object_map[name], PRX_TEXT_LIGHTGRAY);
                }

                std::vector<double> full_state_vec(full_collision_objects_space->get_dimension());
                for( unsigned i = 0; i < pose_seeds.size(); ++i )
                    for( unsigned j = 0; j < size; ++j, ++pl )
                        full_state_vec[pl] = pose_seeds[i].second->at(j);
                full_collision_objects_space->set_from_vector(full_state_vec);

                //////////////////////////////////////////
                //       Inform the Transit graph.      //
                //////////////////////////////////////////

                PRX_DEBUG_COLOR("Inform the Transit graph.", PRX_TEXT_BROWN);
                manipulation_specification = dynamic_cast<manipulation_mp_specification_t*>(output_specifications[transit_manipulation_mp_name]);
                manip_mp = dynamic_cast<manipulation_mp_t*>(planners[transit_manipulation_mp_name]);
                PRX_ASSERT(manip_mp != NULL);

                model->use_context(pc_name_transit);
                manipulation_specification->link_poses(&poses_set);
                manipulation_specification->link_poses_seeds(&pose_seeds);
                manip_state_space->copy_from_point(safe_state);

                manip_mp->link_specification(manipulation_specification);
                manip_mp->setup();
                manip_mp->execute();
                manip_mp->update_visualization();


                //////////////////////////////////////////
                //       Inform the Transfer graph.     //
                //////////////////////////////////////////

                PRX_DEBUG_COLOR("Inform the Transfer graph.", PRX_TEXT_BROWN);

                //The validity checker is already informed from above
                model->use_context(pc_name_transfer);
                PRX_DEBUG_COLOR("planning : " << model->get_state_space()->get_dimension() << "    active: " << model->get_active_space()->get_dimension() << "     inactive: " << model->get_inactive_space()->get_space_name(), PRX_TEXT_CYAN);

                model->use_context(pc_name_transfer);

                manipulation_specification = dynamic_cast<manipulation_mp_specification_t*>(output_specifications[transfer_manipulation_mp_name]);
                manip_mp = dynamic_cast<manipulation_mp_t*>(planners[transfer_manipulation_mp_name]);
                PRX_ASSERT(manip_mp != NULL);
                manipulation_specification->link_poses(&poses_set);
                manipulation_specification->link_poses_seeds(&pose_seeds);
                manipulation_specification->poses_constraints.resize(poses_set_length);
                manipulation_specification->manip_state_space = manip_state_space;
                manipulation_specification->safe_state = safe_state;
                manip_state_space->copy_from_point(safe_state);

                manip_mp->link_specification(manipulation_specification);
                manip_mp->setup();
                manip_mp->execute();
                manip_mp->update_visualization();

                /////////////////////////////////
                //       Inform the Poses.     //
                /////////////////////////////////
                for( unsigned i = 0; i < poses_set_length; ++i )
                {
                    poses_set[i].constraints = manipulation_specification->poses_constraints[poses_set[i].pose_id];
                    PRX_DEBUG_COLOR(i << " ) constraints: poseId:" << poses_set[i].pose_id << "  constraints: " << print(poses_set[i].constraints), PRX_TEXT_CYAN);
                }

                serialize();

                return true;
            }

            const statistics_t* preprocess_manipulation_tp_t::get_statistics()
            {
                return NULL;
            }

            bool preprocess_manipulation_tp_t::succeeded() const
            {
                return true;
            }

            void preprocess_manipulation_tp_t::resolve_query() { }

            bool preprocess_manipulation_tp_t::serialize()
            {
                std::string file = prx_input_dir + poses_file;
                std::string c_poses_file = prx_input_dir + poses_constraints_file;
                std::string poses_file = prx_output_dir + store_poses_file;
                PRX_DEBUG_COLOR(" Inside serialize preprocess manipulation, saving to file: " << file, PRX_TEXT_CYAN);
                std::ofstream fout(file.c_str());
                std::ofstream foutcposes(c_poses_file.c_str());
                std::ofstream foutposes(poses_file.c_str());
                PRX_ASSERT(fout.is_open());

                unsigned valid_poses = 0;

                foreach(pose_t pose, poses_set)
                {
                    if( pose.grasped_set.size() > 0 )
                        ++valid_poses;
                }
                PRX_ASSERT(valid_poses == poses_set_length);

                fout << valid_poses << std::endl;
                foutcposes << valid_poses << std::endl;

                foreach(pose_t pose, poses_set)
                {
                    if( pose.grasped_set.size() > 0 )
                    {
                        //For the informed poses
                        pose.serialize(fout, manip_state_space, mo_space, object_state_space);
                        //For the poses and the constraints.
                        pose.serialize(foutcposes, object_state_space, 10);
                        //For the stored poses. Only the state of the pose in this file. 
                        foutposes << object_state_space->print_point(pose.state, 10) << std::endl;
                    }
                    else
                    {
                        PRX_ASSERT(false);
                    }
                }
                fout.close();
                foutposes.close();

                return true;
            }

            bool preprocess_manipulation_tp_t::deserialize()
            {
                std::string file = prx_output_dir + poses_file;
                PRX_DEBUG_COLOR(" Inside deserialize preprocess manipulation, saving to file: " << file, PRX_TEXT_CYAN);
                std::ifstream fin(file.c_str());
                PRX_ASSERT(fin.is_open());
                fin >> poses_set_length;

                poses_set.resize(poses_set_length);
                for( unsigned i = 0; i < poses_set_length; ++i )
                {
                    poses_set[i].deserialize(fin, manip_state_space, manip_control_space, mo_space, object_state_space);
                }
                return true;
            }

            bool preprocess_manipulation_tp_t::detect_plants()
            {
                model->use_context("full_space");
                std::vector< plant_t* > all_plants;
                model->get_system_graph().get_plants(all_plants);

                foreach(plant_t* plant, all_plants)
                {
                    if( _manipulator == NULL && dynamic_cast<manipulator_plant_t*>(plant) != NULL )
                        _manipulator = static_cast<manipulator_plant_t*>(plant);

                    if( _object == NULL && plant->get_pathname() == object_name )
                        _object = static_cast<movable_body_plant_t*>(plant);


                    if( _manipulator != NULL && _object != NULL )
                    {
                        PRX_DEBUG_COLOR("object : " << _object->get_pathname() << "     _manipulator: " << _manipulator->get_pathname(), PRX_TEXT_CYAN);
                        return true;
                    }
                }
                return false;
            }

            void preprocess_manipulation_tp_t::create_poses_set()
            {
                model->use_context(pc_name_object_only);
                //poses from the specification correspond to the initial position of the cups.

                //Start first with the fixed poses.
                for( unsigned i = 0; i < fix_poses.size(); ++poses_set_length, ++i )
                {
                    pose_t pose;
                    pose.state = object_state_space->alloc_point();
                    object_state_space->copy_vector_to_point(fix_poses[i], pose.state);
                    PRX_DEBUG_COLOR("Goint to test FIXED pose: " << object_state_space->print_point(pose.state, 6) << "  grasps:" << specs->max_different_grasps << "    tries:" << specs->max_tries, PRX_TEXT_GREEN);
                    compute_posible_grasps(pose, specs->max_different_grasps, specs->max_tries);
                    if( pose.grasped_set.size() == 0 )
                    {
                        PRX_DEBUG_COLOR(object_state_space->print_point(pose.state, 5), PRX_TEXT_RED);
                        PRX_FATAL_S("One of the fixed poses/possible critical cannot be grasped!");
                    }
                    pose.pose_id = poses_set_length;
                    poses_set.push_back(pose);
                    pose_seeds.push_back(std::make_pair(poses_set_length, pose.state));
                }

                quaternion_t object_orient = specs->object_orientation;
                quaternion_t theta_orient;

                //                for( unsigned i = 0; i < specs->num_poses; ++i )
                unsigned sampled_poses = 0;
                unsigned sampled_tries = 0;
                while( sampled_poses < specs->num_poses && sampled_tries < specs->max_random_failures )
                {
                    if( specs->boxes_env )
                    {
                        double theta = uniform_random(-PRX_PI, PRX_PI);
                        theta_orient.set_from_euler(theta, 0, 0);
                        object_orient = specs->object_orientation * theta_orient;
                        //                        PRX_DEBUG_COLOR("quat (" << theta << "): " << object_orient.get_x() << " , " << object_orient.get_y() << " , " << object_orient.get_z() << " , " << object_orient.get_w(),PRX_TEXT_GREEN);
                    }

                    pose_t pose;
                    pose.state = object_state_space->alloc_point();
                    pose.state->memory[2] = specs->z_on_table;
                    pose.state->memory[3] = object_orient.get_x();
                    pose.state->memory[4] = object_orient.get_y();
                    pose.state->memory[5] = object_orient.get_z();
                    pose.state->memory[6] = object_orient.get_w();

                    do
                    {
                        sampler->sample(stable_pose_space, stable_pose_state);
                        if( stable_pose_state != NULL )
                        {
                            pose.state->memory[0] = stable_pose_state->memory[0];
                            pose.state->memory[1] = stable_pose_state->memory[1];
                        }
                    }
                    while( stable_pose_state == NULL || !validity_checker->is_valid(pose.state) || similar_pose(pose.state) != -1 );

                    compute_posible_grasps(pose, specs->max_different_grasps, specs->max_tries);
                    if( pose.grasped_set.size() > 0 )
                    {
                        PRX_DEBUG_COLOR("GOT pose: " << object_state_space->print_point(pose.state, 6), PRX_TEXT_CYAN);
                        pose.pose_id = poses_set_length;
                        poses_set.push_back(pose);
                        pose_seeds.push_back(std::make_pair(poses_set_length, pose.state));
                        poses_set_length++;
                        sampled_poses++;
                        sampled_tries = 0;
                    }
                    else
                    {
                        PRX_DEBUG_COLOR("FAILED pose: " << object_state_space->print_point(pose.state, 6), PRX_TEXT_RED);
                        pose.clear(object_state_space, manip_state_space, mo_space);
                        sampled_tries++;
                    }
                }

                if( sampled_poses < specs->num_poses )
                    PRX_FATAL_S("We couldn't sample " << specs->num_poses << " poses!");

                PRX_ASSERT(poses_set_length == specs->num_poses + fix_poses.size());
            }

            void preprocess_manipulation_tp_t::compute_posible_grasps(pose_t& pose, int number_of_grasps, int max_tries)
            {
                double step = 0;
                if(number_of_grasps > 1)
                    step = (max_theta - min_theta)/(number_of_grasps - 1);
                
                for( int i = 0; i < number_of_grasps; ++i )
                {
                    double theta = min_theta + i*step;
                    if( get_grasp_at_theta(grasped_point, pose.state, max_tries, theta) || get_grasp(grasped_point, pose.state, max_tries))
                    {
                        pose.grasped_set.push_back(mo_space->clone_point(grasped_point));
                        pose.ungrasped_set.push_back(manip_state_space->clone_point(released_point));
                        pose.retracted_set.push_back(manip_state_space->clone_point(retracted_point));

                        manip_state_space->copy_from_point(manip_grasping_point);
                        object_state_space->copy_from_point(object_moved_state);
                        mo_space->copy_to_point(grasped_retracted_point);
                        pose.grasped_retracted_set.push_back(mo_space->clone_point(grasped_retracted_point));

                        PRX_DEBUG_COLOR("===========================================================",PRX_TEXT_BROWN);
                        PRX_DEBUG_COLOR("grasped point             : " << mo_space->print_point(pose.grasped_set.back(),5), PRX_TEXT_LIGHTGRAY);
                        PRX_DEBUG_COLOR("grasped_retracted point   : " << mo_space->print_point(pose.grasped_retracted_set.back(),5), PRX_TEXT_LIGHTGRAY);
                        PRX_DEBUG_COLOR("ungrasped point           : " << manip_state_space->print_point(pose.ungrasped_set.back(),5), PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("ungrasped retracted point : " << manip_state_space->print_point(pose.retracted_set.back(),5), PRX_TEXT_CYAN);
                        PRX_DEBUG_COLOR("Reaching: \n" << transit_reaching_plan.print(), PRX_TEXT_LIGHTGRAY);
                        PRX_DEBUG_COLOR("Retract: \n" << transfer_retract_plan.print(), PRX_TEXT_LIGHTGRAY);
                        PRX_DEBUG_COLOR("===========================================================",PRX_TEXT_BROWN);

                        pose.transit_reaching_plans.push_back(transit_reaching_plan);
                        pose.transit_retracting_plans.push_back(transit_retract_plan);
                        pose.transfer_reaching_plans.push_back(transfer_reaching_plan);
                        pose.transfer_retracting_plans.push_back(transfer_retract_plan);
                    }
                }
            }

            // void preprocess_manipulation_tp_t::compute_posible_grasps2(pose_t& pose, int number_of_grasps, int max_tries)
            // {
            //     double step = 0;
            //     if(number_of_grasps > 1)
            //         step = (max_theta - min_theta)/(number_of_grasps - 1);
                
            //     for( int i = 0; i < number_of_grasps; ++i )
            //     {
            //         double theta = min_theta + i*step;
            //         if( get_grasp_at_theta(grasped_point, pose.state, max_tries, theta) || get_grasp(grasped_point, pose.state, max_tries))
            //         {
            //             pose.grasped_set.push_back(mo_space->clone_point(grasped_point));
            //             pose.ungrasped_set.push_back(manip_state_space->clone_point(released_point));

            //             PRX_ASSERT(manip_state_space->equal_points(released_point, retract_path[0]));
            //             pose.retracted_set.push_back(manip_state_space->clone_point(retract_path.back()));

            //             manip_state_space->copy_point_to_vector(pose.retracted_set.back(), manip_control_vec);
            //             manip_control_vec.back() = 1;
            //             manip_state_space->set_from_vector(manip_control_vec);

            //             config_t effector_config;
            //             std::vector< double > config_vector(8);
            //             _manipulator->get_end_effector_configuration(effector_config);

            //             config_t tmp_config = relative_configuration;
            //             tmp_config.relative_to_global(effector_config);
            //             tmp_config.normalize_orientation();

            //             tmp_config.copy_to_vector(config_vector);
            //             object_state_space->set_from_vector(config_vector);

            //             mo_space->copy_to_point(grasped_point);
            //             pose.grasped_retracted_set.push_back(mo_space->clone_point(grasped_point));

            //             //That is ok because state == control for rigid bodies. 
            //             manip_state_space->copy_point_to_vector(released_point, manip_control_vec);
            //             manip_control_space->copy_vector_to_point(manip_control_vec, manip_ctrl);

            //             retract_plan.reverse();
            //             retract_plan.copy_onto_back(manip_ctrl, retract_plan[0].duration);
            //             pose.reaching_plans.push_back(retract_plan);
            //             pose.paths.push_back(retract_path);
            //         }
            //     }
            // }

            void preprocess_manipulation_tp_t::compute_cheating_seeds()
            {
                for( unsigned c = 0; c < cheating_poses.size(); ++c )
                {
                    object_state_space->copy_vector_to_point(cheating_poses[c], object_state);
                    for( unsigned i = 0; i < specs->max_different_grasps; ++i )
                    {
                        if( get_grasp(grasped_point, object_state, specs->max_tries, false) )
                        {
                            cheating_transfer_seeds.push_back(mo_space->clone_point(grasped_point));
                            cheating_transit_seeds.push_back(manip_state_space->clone_point(released_point));
                        }
                    }
                }

                PRX_DEBUG_COLOR("Cheating... Transit: " << cheating_transit_seeds.size() << "    Transfer: " << cheating_transfer_seeds.size(), PRX_TEXT_LIGHTGRAY);
            }

            void preprocess_manipulation_tp_t::compute_extra_poses()
            {
                model->use_context(pc_name_object_only);
                for( unsigned i = 0; i < specs->num_extra_poses; ++i )
                {
                    object_state->memory[3] = specs->object_orientation.get_x();
                    object_state->memory[4] = specs->object_orientation.get_y();
                    object_state->memory[5] = specs->object_orientation.get_z();
                    object_state->memory[6] = specs->object_orientation.get_w();
                    do
                    {
                        sampler->sample(extra_pose_space, extra_pose_state);
                        if( extra_pose_state != NULL )
                        {
                            object_state->memory[0] = extra_pose_state->memory[0];
                            object_state->memory[1] = extra_pose_state->memory[1];
                            object_state->memory[2] = extra_pose_state->memory[2];
                        }
                    }
                    while( extra_pose_state == NULL || !validity_checker->is_valid(object_state) );

                    if( get_grasp(grasped_point, object_state, specs->max_tries, false) )
                    {
                        PRX_ASSERT(released_point->memory.back() == 0);
                        cheating_transfer_seeds.push_back(mo_space->clone_point(grasped_point));
                        cheating_transit_seeds.push_back(manip_state_space->clone_point(released_point));
                    }

                }
                PRX_DEBUG_COLOR("EXTRA... Transit: " << cheating_transit_seeds.size() << "    Transfer: " << cheating_transfer_seeds.size(), PRX_TEXT_LIGHTGRAY);
                //                PRX_ASSERT(false);
            }

            bool preprocess_manipulation_tp_t::get_grasp(sim::state_t* point, const sim::state_t* pose_state, int max_tries, bool full_motion)
            {

                int tries = -1;
                do
                {
                    ++tries;
                    if( tries >= max_tries )
                        return false;

                }
                while( !manip_sampler->sample_near_object(point, pose_state) || !is_valid_grasp(point, full_motion) );
                return true;
            }

            bool preprocess_manipulation_tp_t::get_grasp_at_theta(sim::state_t* point, const sim::state_t* pose_state, int max_tries, double theta, bool full_motion)
            {
                return manip_sampler->sample_near_object_with_theta(point, pose_state, theta) && is_valid_grasp(point, full_motion);            
            }

            bool preprocess_manipulation_tp_t::is_valid_grasp(state_t* point, bool full_motion)
            {
                //The retracted end_effector configuration
                tmp_end_config = retraction_config;

                std::string old_context = model->get_current_context();                            

                //Argument point is in mo_space so we will check first the Grasped sets. 
                model->use_context(pc_name_manipulator_with_object);
                PRX_DEBUG_COLOR("going to check grasp: " << model->get_state_space()->print_point(point,5),PRX_TEXT_CYAN);                
                if( validity_checker->is_valid(point) )
                {
                    //point has to be a grasped point that correspond to both manipulator configuration
                    //and the pose of the object that the manipulator grasping.
                    mo_space->copy_from_point(point);
                    manip_state_space->copy_to_point(released_point);
                    object_state_space->copy_to_point(object_state);
                    //we need that to set the manipulator at the grasping position so as to compute_relative_configuration
                    manip_state_space->copy_to_point(manip_grasping_point); 
                    
                    released_point->memory.back() = 0;
                    model->use_context(pc_name_grasp_checks);
                    PRX_DEBUG_COLOR("The point is valid and this is the released point: " << manip_state_space->print_point(released_point, 5), PRX_TEXT_CYAN);                    
                    if( validity_checker->is_valid(released_point) )
                    {
                        PRX_DEBUG_COLOR("Its valid point now full_motion? " << full_motion, PRX_TEXT_GREEN);
                        if( full_motion )
                        {
                            manip_state_space->copy_from_point(manip_grasping_point);
                            //Because the manipulator should still be in the released state, this should be fine?
                            _manipulator->get_end_effector_configuration(tmp_start_config);
                            tmp_end_config.relative_to_global(tmp_start_config);

                            transit_reaching_plan.clear();
                            transfer_reaching_plan.clear();
                            transit_retract_plan.clear();
                            transfer_retract_plan.clear();

                            ((manipulation_simulator_t*)model->get_simulator())->compute_relative_config();
                            //Checking the retracted path when the manipulator is grasping an object.
                            PRX_DEBUG_COLOR("************** TRANSFER RETRACT PLAN *****************" , PRX_TEXT_BROWN);
                            if( valid_move(transfer_retract_plan, manip_grasping_point, tmp_start_config, tmp_end_config, true))
                            {

                                //Keep the new position of the object in order to build the grasped retracted point
                                object_state_space->copy_to_point(object_moved_state);

                                //We are going to use the retracted point as grasped point to check the transfer reaching plan and we will fix it later.
                                manip_state_space->copy_point(retracted_point, tmp_path.back());
                                manip_state_space->copy_from_point(retracted_point);                                
                                ((manipulation_simulator_t*)model->get_simulator())->compute_relative_config();
                                PRX_ASSERT(retracted_point->memory.back() == 1 && manip_grasping_point->memory.back() == 1);
                                PRX_DEBUG_COLOR("************** TRANSFER REACHING PLAN *****************" , PRX_TEXT_MAGENTA);
                                if( valid_move(transfer_reaching_plan, retracted_point, manip_grasping_point, tmp_end_config, tmp_start_config, true ) )
                                {
#ifndef NDEBUG
                                    state_t* obj_state = object_state_space->alloc_point();
                                    object_state_space->copy_to_point(obj_state);
                                    PRX_ASSERT(!object_state_space->equal_points(obj_state, object_moved_state));
                                    PRX_ASSERT(object_state_space->equal_points(obj_state, object_state, PRX_DISTANCE_CHECK*10));
                                    object_state_space->free_point(obj_state);
#endif                                    
                                    
                                    //manip_grasping point has to have the retracted grasped state. Retracted point is now at that stage
                                    manip_state_space->copy_point(manip_grasping_point,retracted_point);
                                    //Fix the retracted point to be the correct one
                                    retracted_point->memory.back() = 0;
                                    PRX_ASSERT(retracted_point->memory.back() == 0 && manip_grasping_point->memory.back() == 1);
                                    //return the object and the manipulator at their original position in order to test the transit plans. 
                                    object_state_space->copy_from_point(object_state);
                                    manip_state_space->copy_from_point(released_point);
                                    ((manipulation_simulator_t*)model->get_simulator())->compute_relative_config();
                                    PRX_DEBUG_COLOR("************** BOTH TRANSIT PLANS *****************" , PRX_TEXT_BLUE);
                                    if( valid_move(transit_retract_plan, released_point, tmp_start_config, tmp_end_config) && valid_move(transit_reaching_plan, retracted_point, released_point, tmp_end_config, tmp_start_config ))
                                    {
#ifndef NDEBUG
                                        state_t* obj_state = object_state_space->alloc_point();
                                        object_state_space->copy_to_point(obj_state);
                                        PRX_ASSERT(object_state_space->equal_points(obj_state, object_state,PRX_DISTANCE_CHECK));
                                        object_state_space->free_point(obj_state);
#endif                                         
                                        model->use_context(old_context);
                                        PRX_DEBUG_COLOR("----------------------- GOOD POSE ------------------------", PRX_TEXT_CYAN);
                                        return true;
                                    }
                                }
                            }
                        }
                        else //not in full motion
                        {
                            PRX_DEBUG_COLOR("Good point no full motion!", PRX_TEXT_GREEN);
                            model->use_context(old_context);
                            return true;
                        }
                    }
                }
                PRX_DEBUG_COLOR("Bad point", PRX_TEXT_RED);
                model->use_context(old_context);
                return false;
            }

            // bool preprocess_manipulation_tp_t::is_valid_grasp(state_t* point, bool full_motion)
            // {
            //     tmp_end_config = retraction_config;

            //     std::string old_context = model->get_current_context();
            //     model->use_context(pc_name_manipulator_with_object);
            //     PRX_DEBUG_COLOR("going to check grasp: " << model->get_state_space()->print_point(point,5),PRX_TEXT_CYAN);
            //     if( validity_checker->is_valid(point) )
            //     {

            //         //point has to be a grasped point that correspond to both manipulator configuration
            //         //and the pose of the object that the manipulator grasping.
            //         mo_space->copy_from_point(point);
            //         manip_state_space->copy_to_point(released_point);
            //         manip_state_space->copy_to_point(manip_grasping_point);                    
            //         object_state_space->copy_to_point(object_state);

            //         released_point->memory.back() = 0;
            //         model->use_context(pc_name_grasp_checks);
            //         PRX_DEBUG_COLOR("The point is valid and this is the released point: " << manip_state_space->print_point(released_point, 5), PRX_TEXT_CYAN);
            //         if( validity_checker->is_valid(released_point) )
            //         {
            //             PRX_DEBUG_COLOR("Its valid point now full_motion? " << full_motion, PRX_TEXT_GREEN);
            //             if( full_motion )
            //             {
            //                 //Because the manipulator should still be in the grasped state, this should be fine?
            //                 _manipulator->get_end_effector_configuration(tmp_start_config);
            //                 tmp_end_config.relative_to_global(tmp_start_config);

            //                 retract_path.clear();
            //                 tmp_path.clear();
            //                 retract_plan.clear();
            //                 reach_plan.clear();
            //                 manip_state_space->copy_from_point(manip_grasping_point);
            //                 ((manipulation_simulator_t*)model->get_simulator())->compute_relative_config();
            //                 if( valid_move(retract_plan, retract_path, manip_grasping_point, tmp_start_config, tmp_end_config, true))
            //                 {
            //                     //Keep the new position of the object and return the object in the old position in order to 
            //                     //compute the reaching plan/path.
            //                     PRX_DEBUG_COLOR("Object before: " << object_state_space->print_point(object_state,5),PRX_TEXT_GREEN);
            //                     PRX_DEBUG_COLOR("Object after : " << object_state_space->print_memory(5),PRX_TEXT_BROWN);
            //                     object_state_space->copy_to_point(object_moved_state);
            //                     object_state_space->copy_from_point(object_state);
            //                     manip_state_space->copy_point(retracted_point, retract_path.back());
            //                     retracted_point->memory.back() = 0;
            //                     PRX_DEBUG_COLOR("retracted point: " << manip_state_space->print_point(retracted_point,5),PRX_TEXT_CYAN);
            //                     manip_state_space->copy_from_point(retracted_point);
            //                     ((manipulation_simulator_t*)model->get_simulator())->compute_relative_config();
            //                     PRX_DEBUG_COLOR("Manip before reaching: " << manip_state_space->print_memory(5),PRX_TEXT_LIGHTGRAY);
            //                     if( valid_move(reach_plan, tmp_path, retracted_point, tmp_end_config, tmp_start_config ) )
            //                     {
            //                         PRX_DEBUG_COLOR("Object before: " << object_state_space->print_point(object_state,5),PRX_TEXT_GREEN);
            //                         PRX_DEBUG_COLOR("Object after : " << object_state_space->print_memory(5),PRX_TEXT_BROWN);
            //                         PRX_DEBUG_COLOR("Good point yes full motion!", PRX_TEXT_GREEN);                                
            //                         model->use_context(old_context);
            //                         // PRX_ASSERT(false);
            //                         return true;
            //                     }
            //                 }
            //             }
            //             else
            //             {
            //                 PRX_DEBUG_COLOR("Good point no full motion!", PRX_TEXT_GREEN);
            //                 model->use_context(old_context);
            //                 return true;
            //             }
            //         }
            //     }
            //     PRX_DEBUG_COLOR("Bad point", PRX_TEXT_RED);
            //     model->use_context(old_context);
            //     return false;
            // }

            bool preprocess_manipulation_tp_t::valid_move(plan_t& plan, const state_t* start, config_t & start_config, config_t & goal_config, bool set_grasping)
            {
                PRX_DEBUG_COLOR("start config: " << start_config.print(), PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR("end config  : " << goal_config.print(), PRX_TEXT_BROWN);
                if( _manipulator->IK_steering(start_config, goal_config, plan, set_grasping) )
                {
                    tmp_path.clear();
                    local_planner->propagate(start, plan, tmp_path);
                    PRX_DEBUG_COLOR("plan for valid move: \n" << plan.print(10), PRX_TEXT_LIGHTGRAY);
                    PRX_DEBUG_COLOR("path for valid move: \n" << tmp_path.print(10), PRX_TEXT_BLUE);
                    if( tmp_path.size() != 0 && validity_checker->is_valid(tmp_path) )
                        return true;
                }
                PRX_DEBUG_COLOR("Failed IK steering for valid move", PRX_TEXT_RED);
                return false;
            }

            bool preprocess_manipulation_tp_t::valid_move(plan_t& plan, const state_t* start, const state_t* goal, config_t & start_config, config_t & goal_config, bool set_grasping)
            {
                PRX_DEBUG_COLOR("start config: " << start_config.print(), PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR("end config  : " << goal_config.print(), PRX_TEXT_BROWN);
                if( _manipulator->IK_steering(start_config, goal_config, goal, plan, set_grasping) )
                {
                    tmp_path.clear();
                    local_planner->propagate(start, plan, tmp_path);
                    PRX_DEBUG_COLOR("plan for valid move: \n" << plan.print(10), PRX_TEXT_LIGHTGRAY);
                    PRX_DEBUG_COLOR("path for valid move: \n" << tmp_path.print(10), PRX_TEXT_BLUE);
                    if( tmp_path.size() != 0 && validity_checker->is_valid(tmp_path) )
                        return true;
                }
                PRX_DEBUG_COLOR("Failed IK steering for valid move", PRX_TEXT_RED);
                return false;
            }

            bool preprocess_manipulation_tp_t::valid_move(plan_t& plan, trajectory_t& path, const state_t* manip_start, const state_t* start, config_t & goal_config)
            {
                //                PRX_DEBUG_COLOR("Valid_Move to " << goal_config.print(), PRX_TEXT_BLUE);
                config_t start_config;
                manip_state_space->copy_from_point(manip_start);
                _manipulator->get_end_effector_configuration(start_config);
                PRX_DEBUG_COLOR("start config: " << start_config.print(), PRX_TEXT_GREEN);
                PRX_DEBUG_COLOR("end config  : " << goal_config.print(), PRX_TEXT_BROWN);
                if( _manipulator->IK_steering(start_config, goal_config, plan) )
                {
                    //                    PRX_DEBUG_COLOR("Going to propagate : " << manip_state_space->serialize_point(manip_start, 5), PRX_TEXT_BROWN);
                    local_planner->propagate(start, plan, path);
                    PRX_DEBUG_COLOR("path for valid move: " << path.print(), PRX_TEXT_BLUE);
                    if( path.size() != 0 && validity_checker->is_valid(path) )
                        return true;
                }
                return false;
            }

            int preprocess_manipulation_tp_t::similar_pose(state_t * pose)
            {
                for( unsigned i = 0; i < poses_set_length; ++i )
                    if( poses_set[i].equal(object_state_space, pose) )
                        return i;

                return -1;
            }

            std::string preprocess_manipulation_tp_t::print(const std::set<unsigned>& set)
            {
                std::stringstream output(std::stringstream::out);

                foreach(unsigned c, set)
                {
                    output << c << " , ";
                }
                return output.str();
            }
        }
    }
}