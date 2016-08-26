/**
 * @file preprocess_coordination_mp.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "planning/motion_planners/preprocess/preprocess_coord_mp.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/modules/heuristic_search/astar_module.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/utilities/definitions/random.hpp"

#include <boost/property_map/vector_property_map.hpp>
#include <boost/graph/connected_components.hpp>

#include <fstream>

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS(prx::packages::coordination_manipulation::preprocess_coordination_mp_t, prx::plan::planner_t)


namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace coordination_manipulation
        {
            preprocess_coordination_mp_t::preprocess_coordination_mp_t() 
            {
                preprocess_query = NULL;
                full_arm_state_space = NULL;
                full_arm_control_space = NULL;
                grid_point = NULL;
            }

            preprocess_coordination_mp_t::~preprocess_coordination_mp_t() 
            {
                if (grid_point != NULL)
                {
                    state_space->free_point(grid_point);
                }
                if (coordination_start_point != NULL)
                {
                    state_space->free_point(coordination_start_point);
                }
                full_arm_state_space->free_point(full_armcup_state);
            }

            void preprocess_coordination_mp_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                PRX_INFO_S("Initializing preprocess prm motion planner ...");
                prm_star_t::init(reader, template_reader);
                
                preprocess_graph = parameters::get_attribute_as<bool>("preprocess_graph", reader, template_reader, false);
                save_graph = parameters::get_attribute_as<bool>("save_graph", reader, template_reader, false);
                save_constraints = parameters::get_attribute_as<bool>("save_constraints", reader, template_reader, true);
//                max_plan_size = parameters::get_attribute_as<int>("max_plan_size", reader, template_reader, 1200);
                
//                max_objects = parameters::get_attribute_as<int>("max_objects", reader, template_reader, 50);

//                vertex_vector.resize(max_plan_size*max_plan_size);
//                valid_vertex.resize(max_plan_size*max_plan_size, false);
                

                
            }
            
//            void  preprocess_coordination_mp_t::initialize_grid()
//            {
//                PRX_PRINT("Initializing grid", PRX_TEXT_MAGENTA);
//                                
//                /** Build maximum sized grid first (2000x2000) */
//                for(int right_step = 0; right_step < max_plan_size; right_step++)
//                {
//                    PRX_PRINT("INITIAL Right step: " << right_step, PRX_TEXT_BLUE);
//                    grid_point->at(0) = right_step;
//                    for(int left_step = 0; left_step < max_plan_size; left_step++)
//                    {
//                        grid_point->at(1) = left_step;
//                        vertex_vector[right_step*max_plan_size + left_step] = add_node(grid_point).second;
//                    }
//                }
//                
//                /** Build edges */
//                int north, south, east, west;
//                int NE, NW, SE, SW;
//                undirected_vertex_index_t v;
//                for(int right_step = 0; right_step < max_plan_size; right_step++)
//                {
//                    for(int left_step = 0; left_step < max_plan_size; left_step++)
//                    {
//                        v = vertex_vector[right_step*max_plan_size + left_step];
//                        // Check if we can add southern neighbors
//                        if (left_step > 0)
//                        {
//                            // Add south
//                            south = right_step*max_plan_size + (left_step-1);
//                            add_edge(v, vertex_vector[south], 1);
//                            
//                            // Check if we can add southeast
//                            if (right_step < max_plan_size - 1)
//                            {
//                                SE = (right_step+1)*max_plan_size + (left_step-1);
//                                add_edge(v, vertex_vector[SE], 1.41421356237);
//                            }
//                            // Check if we can add southwest
//                            if (right_step > 0)
//                            {
//                                SW = (right_step-1)*max_plan_size + (left_step-1);
//                                add_edge(v, vertex_vector[SW], 1.41421356237);
//                            }
//                                
//                        }
//                        
//                        //Check if we can add northern neighbors
//                        if (left_step < max_plan_size -1)
//                        {
//                            north = (right_step)*max_plan_size + (left_step+1);
//                            add_edge(v, vertex_vector[north], 1);
//                            // Check if we can add northeast
//                            if (right_step < max_plan_size - 1)
//                            {
//                                NE = (right_step+1)*max_plan_size + (left_step+1);
//                                add_edge(v, vertex_vector[NE], 1.41421356237);
//                                
//                            }
//                            // Check if we can add northwest
//                            if (right_step > 0)
//                            {
//                                NW = (right_step-1)*max_plan_size + (left_step+1);
//                                add_edge(v, vertex_vector[NW], 1.41421356237);
//                                
//                            }
//                                
//                        }
//                        
//                        // Check if we can add east
//                        if (right_step < max_plan_size - 1)
//                        {
//                            // Add east
//                            east = (right_step+1)*max_plan_size + (left_step);
//                            add_edge(v, vertex_vector[east], 1);
//
//                        }
//                        // Check if we can add west
//                        if (right_step > 0)
//                        {
//                            // Add west
//                            west = (right_step-1)*max_plan_size + (left_step);
//                            add_edge(v, vertex_vector[west], 1);
//                        }
//                    }
//                }
//                PRX_PRINT("FINISHED Initializing grid", PRX_TEXT_CYAN);
//            }
            
            std::pair<bool, util::undirected_vertex_index_t> preprocess_coordination_mp_t::add_node(const space_point_t* n_state)
            {

                v_new = graph.add_vertex< undirected_node_t > ();
                undirected_node_t* node = graph.get_vertex_as< undirected_node_t > (v_new);
                node->index = v_new;
                node->point = state_space->clone_point(n_state);
                node->node_id = num_vertices;
                num_vertices++;
                metric->add_point(graph[v_new]);
                
                return std::make_pair(true, v_new);
            }
            
            util::undirected_edge_index_t preprocess_coordination_mp_t::add_edge(util::undirected_vertex_index_t v, util::undirected_vertex_index_t u, double dist)
            {
                undirected_edge_index_t e;
                e = graph.add_edge< undirected_edge_t > (v, u, dist);
                num_edges++;
                return e;
            }
            
            void preprocess_coordination_mp_t::link_specification(specification_t* new_spec)
            {
                motion_planner_t::link_specification(new_spec);

                full_arm_state_space = input_specification->state_space;
                full_armcup_state = full_arm_state_space->alloc_point();
                full_arm_control_space = input_specification->control_space;

            }
            
            void preprocess_coordination_mp_t::link_query(query_t* new_query)
            {
                if (input_query != NULL)
                {
                    PRX_FATAL_S ("DO not link query more than once!");
                }
                input_query = (motion_planning_query_t*)new_query;
                PRX_ASSERT(input_specification != NULL);
//                input_specification->get_stopping_criterion()->link_goal(input_query->get_goal());
                
                preprocess_query = dynamic_cast<preprocess_mp_query_t*>(new_query);
                PRX_ASSERT(preprocess_query != NULL);
                
//                PRX_ASSERT(preprocess_query->left_plan->size() == preprocess_query->right_plan->size());
//                PRX_ASSERT(preprocess_query->left_plan->length() == preprocess_query->right_plan->length());
                
                /** Extract information necessary */
                
                state_space = preprocess_query->preprocess_space;
                metric->link_space(state_space);
                
                
//                if (!preprocess_graph)
//                {
//                    coordination_start_point = state_space->alloc_point();
//                    grid_point = state_space->alloc_point();
//                    initialize_grid();
//                    initialize_coordination_matrix();
//                }

            }

            void preprocess_coordination_mp_t::setup()
            {
                left_arm_size = preprocess_query->left_trajectory->size();
                right_arm_size = preprocess_query->right_trajectory->size();
                right_start_index = preprocess_query->right_start_index;
                left_start_index = preprocess_query->left_start_index;
                approximate_collisions = preprocess_query->approximate_collision_check;
                valid_vertex.clear();
                valid_vertex.resize(right_arm_size);
                for(unsigned i = 0; i < valid_vertex.size(); i++)
                {
                    valid_vertex[i].resize(left_arm_size, false);
                }
//                if (!preprocess_graph)
//                {
//                    coordination_start_point->at(0) = right_start_index;
//                    coordination_start_point->at(1) = left_start_index;
//                }
                computation_timer.reset();
                
            }
            
            void preprocess_coordination_mp_t::reset()
            {
                PRX_ASSERT(preprocess_query != NULL);
                if (preprocess_graph)
                {
//                    for(unsigned i = 0; i < valid_vertex.size(); i++)
//                    {
//                        for(unsigned j = 0; j < valid_vertex[i].size(); j++)
//                        {
//                            valid_vertex[i][j] = false;
//                        }
//                    }
                    num_vertices = 0;
                }

            }
            
            bool preprocess_coordination_mp_t::execute()
            {
                PRX_PRINT ("EXECUTEin preprocess coordination MP", PRX_TEXT_LIGHTGRAY);
                
                
                if (preprocess_graph)
                {
                    PRX_PRINT("Preprocessing graph!", PRX_TEXT_RED);
                    for(int right_step = right_start_index; right_step < right_arm_size; right_step++)
                    {
//                        PRX_PRINT("EXECUTE Right step: " << right_step, PRX_TEXT_BLUE);

                        for(int left_step = left_start_index; left_step < left_arm_size; left_step++)
                        {
//                            undirected_vertex_index_t v = vertex_vector[right_step*left_arm_size + left_step];
                            if (valid_point(right_step, left_step))
                            {
                                valid_vertex[right_step][left_step] = true;
                                num_vertices++;
                            }
                            else
                            {
                                valid_vertex[right_step][left_step] = false;
                            }
                        }
                    }
                    
                    std::stringstream file;
                    if (save_graph)
                    {
                        file << "[COORDINATION_GRAPH][";
                    }
                    else if (save_constraints)
                    {
                        file << "[COORDINATION_CONSTRAINTS][";
                    }
                    file << preprocess_query->experiment <<"]_R" << preprocess_query->right_object << preprocess_query->right_object_index << 
                            "_L" << preprocess_query->left_object << preprocess_query->left_object_index << ".txt";
                    serialization_file = file.str();
                    serialize();
                }

                PRX_PRINT ("THIS ITERATION TIME: " << computation_timer.measure(), PRX_TEXT_MAGENTA);
                
            }


//            void preprocess_coordination_mp_t::connect_node(undirected_vertex_index_t v, double rad)
//            {
//                std::vector< const abstract_node_t* > neighbors;
//                neighbors = metric->radius_query(graph[v], rad);
//
//                link_node_to_neighbors(v, neighbors);
//
//                //add the generated node to the metric module. Add node is not doing it
//                //so as the nearest neighbor will not return itself.
//                if( metric->has_point(graph[v]) )
//                {
//                    PRX_WARN_S("Metric already has the point! Skipped add");
//                }
//                else
//                    metric->add_point(graph[v]);
//            }
            
            void preprocess_coordination_mp_t::step()
            {
                PRX_PRINT ("Step in preprocess coordination MP", PRX_TEXT_LIGHTGRAY);
                
            }
            
            
            void preprocess_coordination_mp_t::resolve_query()
            {
                PRX_PRINT ("Resolve query in preprocess coordination MP", PRX_TEXT_LIGHTGRAY);
//                if (!preprocess_graph)
//                {
//                    PRX_PRINT ("Resolve preprocess coordination query", PRX_TEXT_RED);
//                    
//                    /** Find start vertex */
//                    undirected_vertex_t start;
//                    const undirected_node_t* node = metric->single_query(undirected_node_t);
//                    if( node != NULL && metric->distance_function(coordination_start_point, node->point) <= PRX_DISTANCE_CHECK)
//                    {
//                        // PRX_DEBUG_COLOR("The point is already in the graph : " << state_space->print_point(n_state, 4), PRX_TEXT_BROWN);
//                        start = node->index;
//                    }
//                    else
//                    {
//                        PRX_FATAL_S ("Start is not in graph!" << coordination_start_point->at(0) << " : " << coordination_start_point->at(1));
//                    }
//                }
//                else
//                {
//                    PRX_ERROR_S ("Cannot resolve queries when mp is in preprocess mode!");
//                }
            }
            
        
            
            bool preprocess_coordination_mp_t::valid_point(state_t* point_to_check)
            {
                unsigned right_point = point_to_check->at(0);
                unsigned left_point = point_to_check->at(1);
                
                return valid_point(right_point, left_point);
            }
            
            bool preprocess_coordination_mp_t::valid_point(unsigned right_point,unsigned left_point)
            {
                state_t* left_state = preprocess_query->left_trajectory->at(left_point);
                state_t* right_state = preprocess_query->right_trajectory->at(right_point);
                
                // Get Left Arm With Cup
                get_armcup_state(left_state, preprocess_query->left_cup_safe_state, preprocess_query->left_state_space,
                        preprocess_query->left_cup_space, preprocess_query->left_armcup_space, preprocess_query->left_armcup_state);
                
                // Get Right Arm With Cup
                get_armcup_state(right_state, preprocess_query->right_cup_safe_state, preprocess_query->right_state_space,
                        preprocess_query->right_cup_space, preprocess_query->right_armcup_space, preprocess_query->right_armcup_state);
                
                // Get combined state
                full_arm_state_space->copy_to_point(full_armcup_state);
                bool valid = validity_checker->is_valid(full_armcup_state);
//                if (!valid)
//                {
//                    PRX_ERROR_S ("NOT VALID! POINT: " << right_point << " , " << left_point);
//                }
//                if (valid)
//                {
//                    PRX_WARN_S("WAS VALID! POINT: " << right_point << " , " << left_point);
//                }
                return valid;
            }

            void preprocess_coordination_mp_t::link_node_to_neighbors(undirected_vertex_index_t v, const std::vector< const abstract_node_t* >& neighbors)
            {
 
            }
            
            void preprocess_coordination_mp_t::get_armcup_state(state_t* arm_start_state, const state_t* cup_safe_state, const space_t* arm_state_space, 
                                                               const space_t* cup_space, const space_t* armcup_space, state_t* armcup_state)
            {
 
                arm_state_space->copy_from_point(arm_start_state);
                
                //TODO: FIX this to actually compute accurate collisions
                if (!approximate_collisions)
                {
//                    if (arm_start_state->at(grasp_control_index) == arm_open)
//                    {
//                        cup_space->copy_from_point(cup_safe_state);
//                    }
//                    else if (arm_start_state->at(grasp_control_index) == arm_closed)
//                    {
//                        cup_space->set_from_vector(config_vector);
//                    }
//                    else
//                    {
//                        PRX_FATAL_S("Something bad happened- apparently we are in a state of halfway grasp???");
//                    }
                }

            }
            
            
            bool preprocess_coordination_mp_t::serialize()
            {
                PRX_INFO_S(" Inside PRM serialization now, saving to file: " << serialization_file);
                char* w = std::getenv("PRACSYS_PATH");
                std::string dir(w);
                dir += ("/prx_output/coordination_constraints/");
                boost::filesystem::path output_dir(dir);
                if( !boost::filesystem::exists(output_dir) )
                {
                    boost::filesystem::create_directories(output_dir);
                }
                std::string file = dir + serialization_file;
                PRX_INFO_S("Filename is: " << file);
                std::ofstream fout;
                fout.open(file.c_str());
                PRX_ASSERT(fout.is_open());
                
                undirected_vertex_index_t v;
                PRX_ERROR_S("Inside graph serialize");
                
                if (save_constraints)
                {
                    fout << preprocess_query->experiment << std::endl;
                    fout << preprocess_query->right_object << " " << preprocess_query->left_object << std::endl;
                    fout << preprocess_query->right_object_index << " " << preprocess_query->left_object_index << std::endl;
                    fout << right_arm_size << " " << left_arm_size << std::endl;
                    fout << num_vertices << std::endl;
                    for(int right_step = 0; right_step < right_arm_size; right_step++)
                    {

                        for(int left_step = 0; left_step < left_arm_size; left_step++)
                        {
                            if (valid_vertex[right_step][left_step])
                            {
                                fout << right_step << " " << left_step << std::endl;

                            }
                        }
                    }
                }
                else if (save_graph)
                {
                    PRX_ERROR_S ("Not implemented yet!");
                }
                
//                int north, south, east, west;
//                int NE, NW, SE, SW;
//                
//                std::vector< undirected_edge_t* > valid_edges;
//                undirected_edge_t* e;
//                
//                for(int right_step = 0; right_step < right_arm_size; right_step++)
//                {
//
//                    for(int left_step = 0; left_step < left_arm_size; left_step++)
//                    {
//                        if (valid_vertex[right_step*max_plan_size + left_step])
//                        {
//                            v = vertex_vector[right_step*max_plan_size + left_step];
//                            graph[v]->serialize(fout, state_space);
//                            fout << std::endl;
//                            
//                            // Check if we can add southern neighbors
//                            if (left_step > 0)
//                            {
//                                // Add south
//                                south = right_step*max_plan_size + (left_step-1);
//                                if (valid_vertex[south])
//                                {
//                                    e = graph.get_edge_as<undirected_edge_t>(v,vertex_vector[south]);
//                                    valid_edges.push_back(e);
//                                }
//
//                                // Check if we can add southeast
//                                if (right_step < right_arm_size - 1)
//                                {
//                                    SE = (right_step+1)*max_plan_size + (left_step-1);
//                                    if (valid_vertex[SE])
//                                    {
//                                        e = graph.get_edge_as<undirected_edge_t>(v,vertex_vector[SE]);
//                                        valid_edges.push_back(e);
//                                    }
//                                }
//                                // Check if we can add southwest
//                                if (right_step > 0)
//                                {
//                                    SW = (right_step-1)*max_plan_size + (left_step-1);
//                                    if (valid_vertex[SW])
//                                    {
//                                        e = graph.get_edge_as<undirected_edge_t>(v,vertex_vector[SW]);
//                                        valid_edges.push_back(e);
//                                    }
//                                }
//
//                            }
//
//                            //Check if we can add northern neighbors
//                            if (left_step < right_arm_size -1)
//                            {
//                                north = (right_step)*max_plan_size + (left_step+1);
//                                if (valid_vertex[north])
//                                {
//                                    e = graph.get_edge_as<undirected_edge_t>(v,vertex_vector[north]);
//                                    valid_edges.push_back(e);
//                                }
//                                // Check if we can add northeast
//                                if (right_step < right_arm_size - 1)
//                                {
//                                    NE = (right_step+1)*max_plan_size + (left_step+1);
//                                    if (valid_vertex[NE])
//                                    {
//                                        e = graph.get_edge_as<undirected_edge_t>(v,vertex_vector[NE]);
//                                        valid_edges.push_back(e);
//                                    }
//
//                                }
//                                // Check if we can add northwest
//                                if (right_step > 0)
//                                {
//                                    NW = (right_step-1)*max_plan_size + (left_step+1);
//                                    if (valid_vertex[NW])
//                                    {
//                                        e = graph.get_edge_as<undirected_edge_t>(v,vertex_vector[NW]);
//                                        valid_edges.push_back(e);
//                                    }
//
//                                }
//
//                            }
//
//                            // Check if we can add east
//                            if (right_step < right_arm_size - 1)
//                            {
//                                // Add east
//                                east = (right_step+1)*max_plan_size + (left_step);
//                                if (valid_vertex[east])
//                                {
//                                    e = graph.get_edge_as<undirected_edge_t>(v,vertex_vector[east]);
//                                    valid_edges.push_back(e);
//                                }
//
//                            }
//                            // Check if we can add west
//                            if (right_step > 0)
//                            {
//                                // Add west
//                                west = (right_step-1)*max_plan_size + (left_step);
//                                if (valid_vertex[west])
//                                {
//                                    e = graph.get_edge_as<undirected_edge_t>(v,vertex_vector[west]);
//                                    valid_edges.push_back(e);
//                                }
//                            }
//                        
//                        }
//                    }
//                    
//                }
//                fout << valid_edges.size() << std::endl;
//
//                foreach(undirected_edge_t* e, valid_edges)
//                {
//                    e->serialize(fout);
//                    fout << std::endl;
//                }
                
                fout.close();
                return true;

            }

            bool preprocess_coordination_mp_t::deserialize()
            {
                PRX_ERROR_S ("Deserialize has no functionality here! Call deserialize_coordination_graphs instead!");
//                PRX_INFO_S(" Inside PRM deserialization now, opening file: " << deserialization_file);
//                char* w = std::getenv("PRACSYS_PATH");
//                std::string dir(w);
////                dir += ("/prx_input/");
//                std::string file = dir + deserialization_file;
//                PRX_INFO_S("File directory is: " << file);
//                //            fin.open(file.c_str());
//                //            PRX_ASSERT(fin.is_open());
//                std::ifstream fin;
//                if( !graph.deserialize<prm_star_node_t, prm_star_edge_t > (file, fin, state_space) )
//                {
//                    PRX_FATAL_S("File could not deserialize!");
//                    return false;
//                }
//                int counter = 0;
//                //    int blah;
//
//                foreach(undirected_edge_index_t e, boost::edges(graph.graph))
//                {
//                    //                boost::source<>()
//                    //                graph.get_edge_as<prm_star_edge_t > (e).
//                    //                graph.set_weight()
//                    double dist = metric->distance_function(graph[boost::source(e, graph.graph)]->point, graph[boost::target(e, graph.graph)]->point);
//                    graph.set_weight(e, dist);
//                    graph.get_edge_as<prm_star_edge_t > (e)->id = counter;
//                    graph.get_edge_as<prm_star_edge_t > (e)->plan.link_control_space(this->control_space);
//                    graph.get_edge_as<prm_star_edge_t > (e)->plan.link_state_space(this->state_space);
//                    //graph.get_edge_as<prm_star_edge_t > (e)->plan.read_from_stream(fin);
//                    counter++;
//                }
//
//                double val_mu = (double)boost::num_edges(graph.graph) / (double)boost::num_vertices(graph.graph);
//                double diff;
//                double val_dev = 0.0;
//
//                foreach(undirected_vertex_index_t nd, boost::vertices(graph.graph))
//                {
//                    PRX_DEBUG_S("Added to metric: " << state_space->print_point(graph.graph[nd].node->point));
//                    metric->add_point(graph[nd]);
//                    PRX_DEBUG_S("Metric now has: " << metric->get_nr_points() << " points");
//                    diff = (double)boost::out_degree(nd, graph.graph) - val_mu;
//                    val_dev += diff * diff;
//                }
//                val_dev = sqrt(val_dev);
//                update_k(boost::num_vertices(graph.graph));
//                fin.close();
//
//                PRX_PRINT("Deserialized roadmap with " << boost::num_vertices(graph.graph) << " vertices ... " << boost::num_edges(graph.graph) << " edges.", PRX_TEXT_MAGENTA);
//                PRX_PRINT("Average Valence: " << val_mu << "   Valence Deviation: " << val_dev, PRX_TEXT_GREEN);
//
//                return true;
            }
            
            void preprocess_coordination_mp_t::valid_random_sample()
            {
                PRX_PRINT ("Valid random sample has no functionality here!", PRX_TEXT_RED);
                
            }
            
            void preprocess_coordination_mp_t::update_k(unsigned nr_nodes)
            {
                PRX_PRINT ("Update K has no functionality here!", PRX_TEXT_RED);
                
            }



        }        

    }
}



