/**
 * @file manipulation_tp.cpp
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

#include "planning/task_planners/rearrangement_primitive.hpp"


#include "prx/utilities/goals/multiple_goal_states.hpp"

#include "../../../manipulation/planning/modules/samplers/manip_sampler.hpp"
#include "../../../rearrangement_manipulation/planning/modules/system_name_validity_checker.hpp"
#include "../../../rearrangement_manipulation/planning/modules/obstacle_aware_astar.hpp"
#include "../../../rearrangement_manipulation/planning/problem_specifications/manipulator_specification.hpp"
#include "../../../rearrangement_manipulation/planning/problem_specifications/manipulation_mp_specification.hpp"
#include "../../../rearrangement_manipulation/planning/queries/manipulator_mp_query.hpp"


namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        using namespace manipulation;
        using namespace rearrangement_manipulation;

        namespace labeled_rearrangement_manipulation
        {

            rearrangement_primitive_t::rearrangement_primitive_t()
            {
                poses_set_length = 0;
                time_ends = false;

                _manipulator = NULL;
                _object = NULL;

                transit_specification = NULL;
                transfer_specification = NULL;

                char* w = std::getenv("PRACSYS_PATH");
                prx_output_dir = std::string(w) + "/prx_output/rearrangement_graphs/";
                prx_input_dir = std::string(w) + "/prx_input/rearrangement_graphs/";
                
                stats = new rearrangement_primitive_statistics_t();
            }

            rearrangement_primitive_t::~rearrangement_primitive_t() { }

            void rearrangement_primitive_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                task_planner_t::init(reader, template_reader);
                PRX_INFO_S("Initializing rearrangement_primitive_t ...");

                pc_name_manipulator_only = parameters::get_attribute("pc_name_manipulator_only", reader, template_reader, "pc_name_manipulator_only");
                pc_name_object_only = parameters::get_attribute("pc_name_object_only", reader, template_reader, "pc_name_object_only");
                pc_name_manipulator_with_object = parameters::get_attribute("pc_name_manipulator_with_object", reader, template_reader, "pc_name_manipulator_with_object");
                pc_name_manipulator_with_active_object = parameters::get_attribute("pc_name_manipulator_with_active_object", reader, template_reader, "pc_name_manipulator_with_active_object");
                pc_name_transit_inform = parameters::get_attribute("pc_name_transit_inform", reader, template_reader, "pc_name_transit_inform");
                pc_name_transfer_inform = parameters::get_attribute("pc_name_transfer_inform", reader, template_reader, "pc_name_transfer_inform");
                pc_name_all_objects = parameters::get_attribute("pc_name_all_objects", reader, template_reader, "pc_name_all_objects");
                pc_name_grasp_planning = parameters::get_attribute("pc_name_grasp_planning", reader, template_reader, "pc_name_grasp_planning");


                //================================//
                //  For Manipulator Task Planner  //
                //================================//

                if( parameters::has_attribute("manipulation_tp_name", reader, template_reader) )
                    manipulation_tp_name = parameters::get_attribute("manipulation_tp_name", reader, template_reader);
                else
                    PRX_FATAL_S("The name of the manipulation task planner is not specified!");

                const parameter_reader_t* specification_template_reader = NULL;
                if( parameters::has_attribute("manip_specification", reader, template_reader) )
                {
                    if( parameters::has_attribute("manip_specification/template", reader, template_reader) )
                    {
                        specification_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + reader->get_attribute("manip_specification/template"));
                    }
                    output_specifications[manipulation_tp_name] = parameters::initialize_from_loader<specification_t > ("prx_planning", reader, "manip_specification", specification_template_reader, "");

                    if( specification_template_reader != NULL )
                    {
                        delete specification_template_reader;
                        specification_template_reader = NULL;
                    }
                }
                else
                {
                    PRX_FATAL_S("Missing manipulation specification!!!");
                }

                const parameter_reader_t* query_template_reader = NULL;
                std::string element = "transfer_query";
                if( parameters::has_attribute(element, reader, template_reader) )
                {
                    if( parameters::has_attribute(element + "/template", reader, template_reader) )
                    {
                        query_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + reader->get_attribute(element + "/template"));
                    }
                    transfer_query = dynamic_cast<manipulator_query_t*>(parameters::initialize_from_loader<query_t > ("prx_planning", reader, element, query_template_reader, ""));
                    if( query_template_reader != NULL )
                    {
                        delete query_template_reader;
                        query_template_reader = NULL;
                    }
                }
                else
                {
                    PRX_FATAL_S("Missing " << element << " query!");
                }

                element = "transit_query";
                if( parameters::has_attribute(element, reader, template_reader) )
                {
                    if( parameters::has_attribute(element + "/template", reader, template_reader) )
                    {
                        query_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + reader->get_attribute(element + "/template"));
                    }
                    transit_query = dynamic_cast<manipulator_query_t*>(parameters::initialize_from_loader<query_t > ("prx_planning", reader, element, query_template_reader, ""));
                    if( query_template_reader != NULL )
                    {
                        delete query_template_reader;
                        query_template_reader = NULL;
                    }
                }
                else
                {
                    PRX_FATAL_S("Missing " << element << " query!");
                }

                //=============================//
                //  Tools for Motion Planners  //
                //=============================//

                element = "planners_specifications/transit";
                if( parameters::has_attribute(element, reader, template_reader) )
                {
                    if( parameters::has_attribute(element + "/template", reader, template_reader) )
                    {
                        specification_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + reader->get_attribute(element + "/template"));
                    }
                    transit_specification = dynamic_cast<manipulation_mp_specification_t*>(parameters::initialize_from_loader<specification_t > ("prx_planning", reader, element, specification_template_reader, ""));
                    if( transit_specification == NULL )
                        PRX_FATAL_S("The transit specification has to be manipulation_mp_specification_t");

                    if( specification_template_reader != NULL )
                    {
                        delete specification_template_reader;
                        specification_template_reader = NULL;
                    }
                }
                else
                {
                    PRX_FATAL_S("Missing transit planner's specification!!!");
                }

                element = "planners_specifications/transfer";
                if( parameters::has_attribute(element, reader, template_reader) )
                {
                    if( parameters::has_attribute(element + "/template", reader, template_reader) )
                    {
                        specification_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + reader->get_attribute(element + "/template"));
                    }
                    transfer_specification = dynamic_cast<manipulation_mp_specification_t*>(parameters::initialize_from_loader<specification_t > ("prx_planning", reader, element, specification_template_reader, ""));
                    if( transfer_specification == NULL )
                        PRX_FATAL_S("The transfer specification has to be manipulation_mp_specification_t");

                    if( specification_template_reader != NULL )
                    {
                        delete specification_template_reader;
                        specification_template_reader = NULL;
                    }
                }
                else
                {
                    PRX_FATAL_S("Missing transfer planner's specification!!!");
                }


                if( parameters::has_attribute("manip_sampler", reader, template_reader) )
                    manip_sampler = static_cast<manip_sampler_t*>(parameters::initialize_from_loader<sampler_t > ("prx_planning", reader, "manip_sampler", template_reader, "manip_sampler"));
                else
                    PRX_FATAL_S("Missing sampler attribute for manipulation task planner!");

                if( parameters::has_attribute("manip_validity_checker", reader, template_reader) )
                {
                    system_name_validity_checker = dynamic_cast<system_name_validity_checker_t*>(parameters::initialize_from_loader<validity_checker_t > ("prx_planning", reader, "manip_validity_checker", template_reader, "manip_validity_checker"));
                    if( system_name_validity_checker == NULL )
                        PRX_FATAL_S("Rearrangement manipulation task planner initialize a validity_checker that it is not system_name_validity_checker!");
                }
                else
                    PRX_FATAL_S("Missing system_name_validity_checker attribute for rearrangement task planner!");
            }

            void rearrangement_primitive_t::link_specification(specification_t* new_spec)
            {
                task_planner_t::link_specification(new_spec);
                in_specs = static_cast<rearrangement_primitive_specification_t*>(new_spec);
                k_objects = in_specs->k_objects;
            }

            void rearrangement_primitive_t::link_query(query_t* new_query)
            {
                task_planner_t::link_query(new_query);
                in_query = dynamic_cast<rearrangement_query_t*>(new_query);

            }

            void rearrangement_primitive_t::setup()
            {
                PRX_DEBUG_COLOR("Setup rearrangement_primitive_t ...", PRX_TEXT_CYAN);

                detect_plants();

                if( _manipulator == NULL )
                    PRX_FATAL_S("You need at least one manipulator for the project to work!");
                if( _object == NULL )
                    PRX_FATAL_S("You need at least two movable object for the rearrangement project!");

                PRX_DEBUG_COLOR("manipulator :  " << _manipulator->get_pathname(), PRX_TEXT_RED);
                PRX_DEBUG_COLOR("object :  " << _object->get_pathname(), PRX_TEXT_RED);

                //===============================//
                // Create the planning contexts. //
                //===============================//
                context_flags true_flags(true, true);
                context_flags active_flags(true, false);

                util::hash_t<std::string, context_flags> mappings;
                mappings[_manipulator->get_pathname()] = true_flags;

                model->create_new_planning_context(pc_name_all_objects, mappings, active_flags);

                mappings[_object->get_pathname()] = active_flags;
                model->create_new_planning_context(pc_name_manipulator_with_active_object, mappings);
                model->create_new_planning_context(pc_name_transit_inform, mappings);

                mappings[_object->get_pathname()].plannable = true;
                model->create_new_planning_context(pc_name_manipulator_with_object, mappings);

                model->create_new_planning_context(pc_name_grasp_planning, mappings, active_flags);

                mappings[_manipulator->get_pathname()].set(false, false);
                model->create_new_planning_context(pc_name_object_only, mappings);

                mappings[_manipulator->get_pathname()].set(true, true);
                model->create_new_planning_context(pc_name_transfer_inform, mappings);


                //==========================//
                // Initializing the spaces. //
                //==========================//
                model->use_context(pc_name_grasp_planning);
                mo_space = model->get_state_space();
                other_objects_space = model->get_active_space();

                model->use_context(pc_name_all_objects);
                all_object_space = model->get_active_space();
                manip_state_space = model->get_state_space();
                manip_control_space = model->get_control_space();

                model->use_context(pc_name_object_only);
                object_state_space = model->get_state_space();

                //Allocating the helping state/control point variables.
                manip_state = manip_state_space->alloc_point();
                safe_state = manip_state_space->alloc_point();
                object_state = object_state_space->alloc_point();
                real_initial_state = all_object_space->alloc_point();

                PRX_ASSERT(manip_control_space != NULL);
                manip_ctrl = manip_control_space->alloc_point();
                safe_control = manip_control_space->alloc_point();

                manip_state_space->set_from_vector(in_specs->safe_position, safe_state);
                manip_control_space->set_from_vector(in_specs->safe_position, safe_control);

                system_name_validity_checker->setup_checker(_manipulator, _object->get_pathname());
                manip_sampler->link_info(_manipulator, manip_state_space, object_state_space, mo_space);

                //                in_specs->link_spaces(manip_state_space, manip_control_space);
                in_specs->setup(model);

                //===========//
                // Poses Set //
                //===========//               
                std::string file = prx_input_dir + in_specs->poses_file;
                PRX_DEBUG_COLOR("The file for the poses : " << file, PRX_TEXT_GREEN);
                std::ifstream fin(file.c_str());
                PRX_ASSERT(fin.is_open());
                fin >> poses_set_length;

                poses_set.resize(poses_set_length);
                seed_poses.resize(poses_set_length);
                for( unsigned i = 0; i < poses_set_length; ++i )
                {
                    poses_set[i].deserialize(fin, manip_state_space, manip_control_space, mo_space, object_state_space);
                    poses_set[i].pose_id = i;
                    seed_poses[i] = std::make_pair(i, poses_set[i].state);
                }

                //=======================//
                // Transit Specification //
                //=======================//
                transit_specification->validity_checker = system_name_validity_checker;
                transit_specification->sampler = manip_sampler;
                transit_specification->_manipulator = _manipulator;
                //The object state space is being used for the collision checking because this is the only active
                //object during the transit state.
                transit_specification->object_space = object_state_space;
                transit_specification->deserialization_file = prx_input_dir + in_specs->transit_graph_file;
                transit_specification->manip_state_space = manip_state_space;
                transit_specification->link_spaces(manip_state_space, manip_control_space);
                transit_specification->valid_constraints = &transit_constraints;
                transit_specification->extra_starting_states = &transit_extra_start_states;
                transit_astar = dynamic_cast<obstacle_aware_astar_t*>(transit_specification->astar);
                transit_specification->setup(model);

                transit_specification->add_seed(safe_state);
                transit_specification->link_poses_seeds(&seed_poses);
                transit_specification->link_query_poses_seeds(&query_poses);

                //========================//
                // Transfer Specification //
                //========================//
                transfer_specification->validity_checker = system_name_validity_checker;
                transfer_specification->sampler = manip_sampler;
                transfer_specification->_manipulator = _manipulator;
                transfer_specification->object_space = object_state_space;
                transfer_specification->deserialization_file = prx_input_dir + in_specs->transfer_graph_file;
                transfer_specification->manip_state_space = manip_state_space;
                transfer_specification->link_spaces(mo_space, manip_control_space);
                transfer_specification->valid_constraints = &transfer_constraints;
                transfer_specification->extra_starting_states = &transfer_extra_start_states;
                transfer_astar = dynamic_cast<obstacle_aware_astar_t*>(transfer_specification->astar);
                transfer_specification->setup(model);

                transfer_specification->link_poses_seeds(&seed_poses);
                transfer_specification->link_query_poses_seeds(&query_poses);

                //============================//
                // Manipulation Specification //
                //============================//
                manip_specs = dynamic_cast<manipulator_specification_t*>(output_specifications[manipulation_tp_name]);
                manip_specs->validity_checker = system_name_validity_checker;
                manip_specs->local_planner = in_specs->local_planner;
                manip_specs->sampler = manip_sampler;
                manip_specs->manip_sampler = manip_sampler;
                manip_specs->link_spaces(manip_state_space, manip_control_space);
                manip_specs->link_extra_spaces(mo_space, manip_control_space);
                manip_specs->pc_name_manipulator_only = pc_name_manipulator_only;
                manip_specs->pc_name_object_only = pc_name_object_only;
                manip_specs->pc_name_manipulator_with_object = pc_name_manipulator_with_object;
                manip_specs->pc_name_manipulator_with_active_object = pc_name_manipulator_with_active_object;
                manip_specs->pc_name_transit_inform = pc_name_transit_inform;
                manip_specs->pc_name_transfer_inform = pc_name_transfer_inform;
                //TODO: This has to be the pebble graph setup with k+b objects.
                manip_specs->pc_name_transit_planning = pc_name_all_objects;
                //TODO: This has to be the pebble graph setup with k+b-1 objects on place and 1 in the manipulator.
                manip_specs->pc_name_transfer_planning = pc_name_grasp_planning;
                manip_specs->safe_position = in_specs->safe_position;
                manip_specs->manipulator = _manipulator;
                manip_specs->object = _object;
                manip_specs->setup(model);


                manip_tp = dynamic_cast<manipulator_tp_t*>(planners[manipulation_tp_name]);

                transit_astar->link_constraints(&transit_obstacles, &transit_constraints, &transit_avoid);
                transfer_astar->link_constraints(&transfer_obstacles, &transfer_constraints, &transfer_avoid);

                //======================//
                // Manipulation queries //
                //======================//
                transit_query->link_spaces(manip_state_space, manip_control_space);
                transit_query->mode = manipulator_query_t::PRX_TRANSIT;
                transit_query->q_collision_type = motion_planning_query_t::PRX_ACTIVE_COLLISIONS_REUSE_EDGES;
                transit_query->q_type = motion_planning_query_t::PRX_ADD_QUERY_POINTS_COLLISIONS;
                transit_query_goal = dynamic_cast<multiple_goal_states_t*>(transit_query->get_goal());

                transfer_query->link_spaces(mo_space, manip_control_space);
                transfer_query->mode = manipulator_query_t::PRX_TRANSFER;
                transfer_query->q_collision_type = motion_planning_query_t::PRX_ACTIVE_COLLISIONS_REUSE_EDGES;
                transfer_query->q_type = motion_planning_query_t::PRX_ADD_QUERY_POINTS_COLLISIONS;
                transfer_query_goal = dynamic_cast<multiple_goal_states_t*>(transfer_query->get_goal());
            }
            
            void rearrangement_primitive_t::resolve_query()
            {
                time_ends = false;
                statistics_clock.reset();

                if( solve() )
                {
                    in_query->got_solution = true;
                    stats->computation_time = statistics_clock.measure();
                    stats->num_of_moved_objects = in_query->path_sequence.size();
                    stats->found_path = true;
                    PRX_PRINT("------------------------", PRX_TEXT_BROWN);
                    PRX_PRINT("------ FOUND PATH ------", PRX_TEXT_RED);
                    PRX_PRINT("------------------------", PRX_TEXT_BROWN);
                    return;
                }
                in_query->got_solution = false;
                stats->computation_time = statistics_clock.measure();
                stats->found_path = false;                
                return;
            }

            const statistics_t* rearrangement_primitive_t::get_statistics()
            {
                stats->objects_no = k_objects;
                return stats;
            }

            bool rearrangement_primitive_t::succeeded() const
            {
                return true;
            }

            bool rearrangement_primitive_t::execute()
            {
                manip_specs->transit_graph_specification = transit_specification;
                manip_specs->transfer_graph_specification = transfer_specification;

                manip_tp->link_specification(manip_specs);
                manip_tp->setup();
                manip_tp->execute();
                return true;
            }                        

            bool rearrangement_primitive_t::detect_plants()
            {
                model->use_context("full_space");
                std::vector< plant_t* > all_plants;
                model->get_system_graph().get_plants(all_plants);

                foreach(plant_t* plant, all_plants)
                {
                    if( _manipulator == NULL && dynamic_cast<manipulator_plant_t*>(plant) != NULL )
                        _manipulator = static_cast<manipulator_plant_t*>(plant);

                    if( _object == NULL && dynamic_cast<movable_body_plant_t*>(plant) != NULL )
                        _object = static_cast<movable_body_plant_t*>(plant);

                    if( _manipulator != NULL && _object != NULL )
                        return true;
                }
                return false;
            }

            std::string rearrangement_primitive_t::print(const std::vector<unsigned>& arrangement)
            {

                std::stringstream output(std::stringstream::out);

                foreach(unsigned i, arrangement)
                {
                    output << i << " , ";
                }
                return output.str();

            }

            std::string rearrangement_primitive_t::print(const std::vector<double>& arrangement)
            {

                std::stringstream output(std::stringstream::out);

                foreach(double i, arrangement)
                {
                    output << i << " , ";
                }
                return output.str();

            }

            std::string rearrangement_primitive_t::print(const std::deque<unsigned>& arrangement)
            {
                std::stringstream output(std::stringstream::out);

                foreach(unsigned i, arrangement)
                {
                    output << i << " , ";
                }
                return output.str();
            }

            std::string rearrangement_primitive_t::print(const std::set<unsigned>& constraints)
            {
                std::stringstream output(std::stringstream::out);

                foreach(unsigned i, constraints)
                {
                    output << i << " , ";
                }
                return output.str();
            }
            
            std::string rearrangement_primitive_t::print(const std::vector<pose_t*>& free_poses)
            {
                std::stringstream output(std::stringstream::out);
                
                foreach(pose_t* p, free_poses)
                {
                    output << p->pose_id << " , ";
                }
                return output.str();
            }
        }
    }
}