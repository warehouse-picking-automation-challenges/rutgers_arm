/**
 * @file rearrangement_search_algorithm.cpp
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

#include "planning/task_planners/rearrangement_search_algorithm.hpp"
#include "planning/task_planners/rearrangement_primitive.hpp"
#include "planning/task_planners/rearrangement_path_planner.hpp"
#include "planning/problem_specifications/rearrangement_search_specification.hpp"
#include "planning/problem_specifications/rearrangement_primitive_specification.hpp"
#include "planning/queries/rearrangement_search_query.hpp"
#include "planning/queries/rearrangement_query.hpp"
#include "planning/modules/path_part.hpp"


#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/goals/multiple_goal_states.hpp"
#include "prx/simulation/plan.hpp"
#include "prx/planning/modules/heuristic_search/astar_module.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"

#include <vector>
#include <numeric>
#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/filesystem.hpp>
#include <set>

PLUGINLIB_EXPORT_CLASS(prx::packages::labeled_rearrangement_manipulation::rearrangement_search_algorithm_t, prx::plan::planner_t)

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

            rearrangement_search_algorithm_t::rearrangement_search_algorithm_t()
            {
                char* w = std::getenv("PRACSYS_PATH");
                prx_output_dir = std::string(w) + "/prx_output/rearrangement_graphs/";
                prx_input_dir = std::string(w) + "/prx_input/rearrangement_graphs/";
                stats = new rearrangement_search_statistics_t();
            }

            rearrangement_search_algorithm_t::~rearrangement_search_algorithm_t() { }

            void rearrangement_search_algorithm_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                task_planner_t::init(reader, template_reader);
                PRX_INFO_S("Initializing rearrangement_search_algorithm_t ...");

                //=================//
                //  For Primitive  //
                //=================//
                const parameter_reader_t* tmp_reader = NULL;
                std::string element = "primitive_specification";
                if( parameters::has_attribute(element, reader, template_reader) )
                {
                    if( parameters::has_attribute(element + "/template", reader, template_reader) )
                    {
                        tmp_reader = new parameter_reader_t(ros::this_node::getName() + "/" + reader->get_attribute(element + "/template"));
                    }
                    primitive_specs = dynamic_cast<rearrangement_primitive_specification_t*>(parameters::initialize_from_loader<specification_t > ("prx_planning", reader, element, tmp_reader, ""));
                    if( primitive_specs == NULL )
                        PRX_FATAL_S("The primitive specification must be rearrangement_primitive_specification_t type!");
                    if( tmp_reader != NULL )
                    {
                        delete tmp_reader;
                        tmp_reader = NULL;
                    }
                }
                else
                {
                    PRX_FATAL_S("Missing " << element << " query!");
                }

                element = "rearrangement_query";
                if( parameters::has_attribute(element, reader, template_reader) )
                {
                    if( parameters::has_attribute(element + "/template", reader, template_reader) )
                    {
                        tmp_reader = new parameter_reader_t(ros::this_node::getName() + "/" + reader->get_attribute(element + "/template"));
                    }
                    primitive_query = dynamic_cast<rearrangement_query_t*>(parameters::initialize_from_loader<query_t > ("prx_planning", reader, element, tmp_reader, ""));
                    if( primitive_query == NULL )
                        PRX_FATAL_S("The primitive query must be rearrangement_query_t type!");
                    if( tmp_reader != NULL )
                    {
                        delete tmp_reader;
                        tmp_reader = NULL;
                    }
                }
                else
                {
                    PRX_FATAL_S("Missing " << element << " !!!");
                }


                pc_name_manipulator_only = parameters::get_attribute("pc_name_manipulator_only", reader, template_reader, "manipulator_pc");
            }

            void rearrangement_search_algorithm_t::setup()
            {
                
                PRX_INFO_S("Setup rearrangement_search_algorithm_t ...");
                //==============================//
                // Get the manipulator's spaces //
                //==============================//                
                crate_spaces();

                safe_state = manip_state_space->alloc_point();
                safe_control = manip_control_space->alloc_point();
                object_state = object_state_space->alloc_point();

                manip_state_space->set_from_vector(in_specs->safe_position, safe_state);
                manip_control_space->set_from_vector(in_specs->safe_position, safe_control);


                //========================//
                // Initialize graph space //
                //========================//
                std::string space_name;
                for( unsigned i = 0; i < in_specs->k_objects; ++i )
                {
                    graph_space_memory.push_back(new double());
                    graph_space_memory.push_back(new double());
                    space_name += "XY";
                    if( i < in_specs->k_objects - 1 )
                        space_name += "|";
                }
                graph_space = new space_t(space_name, graph_space_memory);
                graph_point = graph_space->alloc_point();
                graph_vec.resize(graph_space->get_dimension());


                //====================================================//
                // Read the poses that will be used in the experiment //
                //====================================================//                
                std::string file = prx_input_dir + in_specs->poses_constraints_file;
                PRX_DEBUG_COLOR("The file for the poses : " << file, PRX_TEXT_GREEN);
                std::ifstream fin(file.c_str());
                PRX_ASSERT(fin.is_open());
                fin >> poses_set_length;

                poses_set.resize(poses_set_length);
                for( unsigned i = 0; i < poses_set_length; ++i )
                {
                    poses_set[i].deserialize(fin, object_state_space);
                    poses_set[i].pose_id = i;
                }


                //====================================//
                // Initialize Primitive Communication //
                //====================================//                
                primitive_specs->k_objects = in_specs->k_objects;
                primitive_specs->safe_position = in_specs->safe_position;
                primitive_specs->transit_graph_file = in_specs->transit_graph_file;
                primitive_specs->transfer_graph_file = in_specs->transfer_graph_file;
                primitive_specs->poses_file = in_specs->poses_file;
                primitive_specs->gather_statistics = in_specs->gather_statistics;
                primitive_specs->validity_checker = in_specs->validity_checker;
                primitive_specs->local_planner = in_specs->local_planner;
                primitive_specs->link_spaces(manip_state_space, manip_control_space);


                primitive = dynamic_cast<rearrangement_primitive_t*>(planners["primitive"]);
                primitive->link_specification(primitive_specs);
                primitive->setup();
                primitive->execute();

                //===================================//
                // Initialize Smoother Communication //
                //===================================//
                path_planner = dynamic_cast<rearrangement_path_planner_t*>(planners["path_planner"]);
                path_planner->link_specification(primitive_specs);
                path_planner->setup();
                path_planner->execute();

                //=========================//
                // Init the rest variables //
                //=========================//
                pose_checked.resize(poses_set_length);
                initial_arrangement.resize(in_specs->k_objects);
                target_arrangement.resize(in_specs->k_objects);
                ids.resize(in_specs->k_objects);
                for( unsigned i = 0; i < in_specs->k_objects; ++i )
                    ids[i] = i;
                metric->link_space(graph_space);

                //================//
                // For Statistics //
                //================//
                statistics_file = prx_output_dir + "Statistics/Single_" + in_specs->statistics_file;
                print_all = false;
                stats->objects_no = in_specs->k_objects;
            }

            const statistics_t* rearrangement_search_algorithm_t::get_statistics()
            {
                if( in_specs->gather_statistics )
                {
                    PRX_DEBUG_COLOR("statistics file: " << statistics_file, PRX_TEXT_CYAN);
                    std::string dir, file;
                    boost::tie(dir, file) = reverse_split_path(statistics_file);
                    PRX_DEBUG_COLOR("dir: " << dir << "     file:" << file, PRX_TEXT_GREEN);
                    boost::filesystem::path output_dir(dir);
                    if( !boost::filesystem::exists(output_dir) )
                    {
                        boost::filesystem::create_directory(output_dir);
                    }

                    std::ofstream fout(statistics_file.c_str());
                    PRX_DEBUG_COLOR("Opened the file: " << statistics_file, PRX_TEXT_GREEN);
                    PRX_ASSERT(fout.is_open());
                    stats->average(print_all);
                    fout << stats->print_statistics(print_all);
                    fout << std::endl;
                    fout.close();
                }

                return stats;
            }

            void rearrangement_search_algorithm_t::link_specification(specification_t* new_spec)
            {
                task_planner_t::link_specification(new_spec);
                in_specs = static_cast<rearrangement_search_specification_t*>(new_spec);
            }

            void rearrangement_search_algorithm_t::link_query(query_t* new_query)
            {
                task_planner_t::link_query(new_query);
                in_query = dynamic_cast<rearrangement_search_query_t*>(new_query);
            }

            bool rearrangement_search_algorithm_t::execute()
            {
                resolve_query();
                return true;
            }

            bool rearrangement_search_algorithm_t::succeeded() const
            {
                return true;
            }

            void rearrangement_search_algorithm_t::resolve_query()
            {
                PRX_DEBUG_COLOR("Resolve_query for rearrangement_search_algorithm_t", PRX_TEXT_CYAN);
                statistics_clock.reset();
                int pose_index;
                //============================//
                //         Initial Node       //
                //============================//
                for( unsigned i = 0; i < in_specs->k_objects; ++i )
                {
                    object_state_space->copy_vector_to_point(in_query->initial_poses[i], object_state);
                    if( (pose_index = detect_pose(object_state)) != -1 )
                    {
                        initial_arrangement[i] = pose_index;
                    }
                    else
                    {
                        PRX_FATAL_S("Initial pose is not included in the informed poses! " << object_state_space->print_point(object_state, 8));
                    }
                }

                //============================//
                //         Target Node        //
                //============================//
                for( unsigned i = 0; i < in_specs->k_objects; ++i )
                {
                    object_state_space->copy_vector_to_point(in_query->target_poses[i], object_state);
                    if( (pose_index = detect_pose(object_state)) != -1 )
                    {
                        target_arrangement[i] = pose_index;
                    }
                    else
                    {
                        PRX_FATAL_S("Initial pose is not included in the informed poses! " << object_state_space->print_point(object_state, 8));
                    }
                }

                //========================================//
                // Gets the query ready for the primitive //
                //========================================//
                primitive_query->link_spaces(manip_state_space, manip_control_space);
                primitive_query->start_state = safe_state;
                primitive_query->initial_poses_ids = initial_arrangement;
                primitive_query->target_poses_ids = target_arrangement;
                primitive_query->time_limit = in_specs->time_limit - statistics_clock.measure();
                primitive_query->accept_partial_solutions = false;
                primitive->link_query(primitive_query);
                primitive->resolve_query();
                stats->sequence_stats.push_back(primitive->get_statistics()->as<rearrangement_primitive_statistics_t > ());
                stats->computation_time = statistics_clock.measure();

                if( primitive_query->found_solution() )
                {
                    stats->found_path = true;
                    primitive_query->time_limit = in_specs->time_limit - statistics_clock.measure();
                    path_planner->link_query(primitive_query);
                    path_planner->resolve_query();
                    in_query->plan = primitive_query->plan;
                }
                stats->planner_stats = path_planner->get_statistics()->as<rearrangement_primitive_statistics_t > ();

                return;
            }

            undirected_vertex_index_t rearrangement_search_algorithm_t::add_node(std::vector<unsigned>& arrangement)
            {
                generate_graph_point(graph_point, arrangement);
                return add_node(arrangement, graph_point);
            }

            undirected_vertex_index_t rearrangement_search_algorithm_t::add_node(std::vector<unsigned>& arrangement, sim::state_t* graph_point)
            {
                undirected_vertex_index_t v = super_graph.add_vertex<rearrangement_node_t > ();
                super_graph.get_vertex_as<rearrangement_node_t > (v)->init(graph_space, graph_point, arrangement);
                return v;
            }

            bool rearrangement_search_algorithm_t::try_connect(std::vector<unsigned>& arrangement)
            {
                return false;
            }

            bool rearrangement_search_algorithm_t::sample_arrangement(std::vector<unsigned>& arrangement)
            {
                unsigned num_checked = 0;
                unsigned index = 0;
                std::fill(pose_checked.begin(), pose_checked.end(), false);
                while( arrangement.size() != in_specs->k_objects && num_checked < poses_set_length )
                {
                    PRX_DEBUG_COLOR("Getting arrangement : " << print(arrangement), PRX_TEXT_CYAN);
                    int failures = -1;
                    do
                    {
                        if( ++failures == in_specs->max_tries )
                            return false;

                        //5 % of the time we are biased towards the goal positions
                        //Poses have the same id as their place in the poses_set.
                        int bias_chance = uniform_int_random(0, 99);
                        if( bias_chance < in_specs->goal_biasing )
                        {
                            if( uniform_int_random(0, 99) >= 50 )
                            {
                                index = target_arrangement[uniform_int_random(0, in_specs->k_objects - 1)];
                            }
                            else
                            {
                                index = initial_arrangement[uniform_int_random(0, in_specs->k_objects - 1)];
                            }
                        }
                        else
                        {
                            index = uniform_int_random(0, poses_set_length - 1);
                        }

                    }
                    while( pose_checked[index] );
                    pose_checked[index] = true;
                    ++num_checked;

                    if( valid_pose(index, arrangement) )
                        arrangement.push_back(index);
                }

                //We can reach here if we check all the possible poses but none of them worked.
                return arrangement.size() == in_specs->k_objects;
            }

            void rearrangement_search_algorithm_t::generate_graph_point(state_t* point, const std::vector<unsigned>& arrangement)
            {
                PRX_ASSERT(arrangement.size() == in_specs->k_objects);
                for( unsigned i = 0; i < in_specs->k_objects; ++i )
                {
                    graph_vec[i * 2] = poses_set[arrangement[i]].state->memory[0];
                    graph_vec[i * 2 + 1] = poses_set[arrangement[i]].state->memory[1];
                }

                graph_space->copy_vector_to_point(graph_vec, point);
            }

            int rearrangement_search_algorithm_t::detect_pose(state_t * pose)
            {
                PRX_DEBUG_COLOR("Detect pose: out of " << poses_set_length << " poses", PRX_TEXT_GREEN);
                for( unsigned i = 0; i < poses_set_length; ++i )
                    if( poses_set[i].equal(object_state_space, pose) )
                    {
                        PRX_DEBUG_COLOR("Found the same pose with id : " << i, PRX_TEXT_GREEN);
                        return i;
                    }

                PRX_DEBUG_COLOR("NO POSE with the same state", PRX_TEXT_RED);
                return -1;
            }

            bool rearrangement_search_algorithm_t::valid_pose(unsigned index, const std::vector<unsigned>& arrangement)
            {

                if( std::find(arrangement.begin(), arrangement.end(), index) != arrangement.end() )
                    return false;

                if( poses_set[index].constraints.size() != 0 )
                    for( unsigned i = 0; i < arrangement.size(); ++i )
                        if( poses_set[index].constraints.count(arrangement[i]) > 0 )
                            return false;
                return true;
            }

            undirected_vertex_index_t rearrangement_search_algorithm_t::node_exists(const std::vector<unsigned>& arrangement)
            {

                foreach(undirected_vertex_index_t v, boost::vertices(super_graph.graph))
                {
                    if( super_graph.get_vertex_as<rearrangement_node_t>(v)->same_arrangement(arrangement) )
                        return v;
                }
                return NULL;
            }

            void rearrangement_search_algorithm_t::update_k(unsigned nr_nodes)
            {
                if( nr_nodes == 0 )
                {
                    k_near = 0;
                }
                else
                {
                    double d = graph_space->get_dimension();
                    double val = (1.0 / d);
                    k_near = PRX_MAXIMUM(std::ceil((log(nr_nodes)*2.7182818284 * (1 + val))), 1);
                }
            }

            void rearrangement_search_algorithm_t::crate_spaces()
            {
                model->use_context(pc_name_manipulator_only);
                manip_state_space = model->get_state_space();
                manip_control_space = model->get_control_space();


                model->use_context("full_space");
                std::vector< plant_t* > all_plants;
                model->get_system_graph().get_plants(all_plants);

                foreach(plant_t* plant, all_plants)
                {
                    if( dynamic_cast<movable_body_plant_t*>(plant) != NULL )
                    {
                        util::hash_t<std::string, context_flags> mappings;
                        mappings[plant->get_pathname()] = context_flags(true, true);
                        std::string pc_name = "object_pc_name";
                        model->create_new_planning_context(pc_name, mappings);
                        model->use_context(pc_name);
                        object_state_space = model->get_state_space();
                        return;
                    }
                }
            }

            std::string rearrangement_search_algorithm_t::print(const std::vector<unsigned>& arrangement)
            {

                std::stringstream output(std::stringstream::out);

                foreach(unsigned i, arrangement)
                {
                    output << i << " , ";
                }
                return output.str();

            }
        }
    }
}

