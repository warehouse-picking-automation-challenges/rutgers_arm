/**
 * @file irs.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2011, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Athanasios Krontiris, Andrew Dobson, Andrew Kimmel, Zakary Littlefield, James Marble, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/utilities/boost/boost_wrappers.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"

#include "planning/motion_planners/tsm_prm_star.hpp"
#include "planning/specifications/tsm_prm_specification.hpp"
#include "utilities/distance_metrics/task_space_metric.hpp"
#include "utilities/distance_metrics/motion_planning_task_space_node.hpp"
#include "planning/modules/grasp_evaluator.hpp"

#include <boost/property_map/vector_property_map.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/assign/list_of.hpp>

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::tsm_prm_star_t, prx::plan::planner_t)


namespace prx
{
    using namespace util;
    using namespace plan;
    using namespace sim;

    namespace packages
    {

        namespace manipulation
        {

            tsm_prm_star_t::tsm_prm_star_t() 
            {

                std::vector<double*> state_dim = boost::assign::list_of(&_x)(&_y)(&_z)(&_qx)(&_qy)(&_qz)(&_qw);
                end_effector_space = new space_t("SE3", state_dim);
            }

            tsm_prm_star_t::~tsm_prm_star_t() 
            {
                delete end_effector_space; 
            }

            void tsm_prm_star_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                PRX_INFO_S("Initializing TSM PRM motion planner ...");
                prm_star_t::init(reader, template_reader);
            }

            void tsm_prm_star_t::link_specification(specification_t* new_spec)
            {
                prm_star_t::link_specification( new_spec );

                tsm_prm_specification = dynamic_cast< tsm_prm_specification_t* > (new_spec);
                PRX_ASSERT(tsm_prm_specification != NULL);
            }


            std::pair<bool, util::undirected_vertex_index_t> tsm_prm_star_t::add_node(space_point_t* n_state)
            {
                if( metric->get_nr_points() > 0 )
                {
                    abstract_node_t* inner_node = metric->single_query(n_state)->as< task_space_node_t >()->motion_planning_node;
                    PRX_ASSERT(inner_node != NULL);
                    motion_planning_task_space_node_t<prm_star_node_t>* msts_node = dynamic_cast<motion_planning_task_space_node_t<prm_star_node_t>* >(inner_node);
                    PRX_ASSERT(msts_node != NULL);
                    //TODO: removed near query type
                    if( msts_node != NULL && ( metric->distance_function(n_state, msts_node->point) <= similarity_threshold) )
                    {
                        return std::make_pair(false, msts_node->index);
                    }
                }

                v_new = graph.add_vertex<motion_planning_task_space_node_t<prm_star_node_t> > ();
                num_vertices++;

                config_t ee_conf;
                tsm_prm_specification->grasp_evaluator->FK(n_state, ee_conf);

                task_space_node_t* ts_node = new task_space_node_t(state_space, n_state, config_to_state(ee_conf), ee_conf);
                motion_planning_task_space_node_t<prm_star_node_t>* msts_node = graph.get_vertex_as< motion_planning_task_space_node_t<prm_star_node_t> >(v_new);
                ts_node->motion_planning_node = msts_node;
                msts_node->task_space_node = ts_node;
                msts_node->init_node(state_space, n_state, validity_checker->alloc_constraint());

                if( delta_prm )
                    connect_node(v_new, r_n);
                else
                    connect_node(v_new);

                return std::make_pair(true, v_new);
            }


            void tsm_prm_star_t::step()
            {
                if (ros::ok())
                {
                    prm_star_t::step();
                }
            }

            void tsm_prm_star_t::connect_node(undirected_vertex_index_t v)
            {
                std::vector<const abstract_node_t*> neighbors;
                task_space_node_t* query_node = graph[v]->as< motion_planning_task_space_node_t<prm_star_node_t> >()->task_space_node;
                PRX_ASSERT(query_node != NULL);
                neighbors = metric->multi_query(query_node, k);

                link_node_to_neighbors(v, neighbors);

                //add the generated node to the metric module. Add node is not doing it
                //so as the nearest neighbor will not return itself.
                if( metric->has_point(graph[v]->as< motion_planning_task_space_node_t<prm_star_node_t> >()->task_space_node) )
                {
                    PRX_WARN_S("Metric already has the point! Skipped add");
                }
                else
                    metric->add_point(graph[v]->as< motion_planning_task_space_node_t<prm_star_node_t> >()->task_space_node);
            }

            void tsm_prm_star_t::connect_node(undirected_vertex_index_t v, double rad)
            {
                std::vector< const abstract_node_t* > neighbors;
                task_space_node_t* query_node = graph[v]->as< motion_planning_task_space_node_t<prm_star_node_t> >()->task_space_node;
                PRX_ASSERT(query_node != NULL);
                neighbors = metric->radius_query(query_node, rad);

                link_node_to_neighbors(v, neighbors);

                //add the generated node to the metric module. Add node is not doing it
                //so as the nearest neighbor will not return itself.
                if( metric->has_point(graph[v]->as< motion_planning_task_space_node_t<prm_star_node_t> >()->task_space_node) )
                {
                    PRX_WARN_S("Metric already has the point! Skipped add");
                }
                else
                    metric->add_point(graph[v]->as< motion_planning_task_space_node_t<prm_star_node_t> >()->task_space_node);
            }


            void tsm_prm_star_t::link_node_to_neighbors(undirected_vertex_index_t v, const std::vector< const abstract_node_t* >& neighbors)
            {
                const task_space_node_t* neighbor_node;
                const motion_planning_task_space_node_t<prm_star_node_t>* neighbor_msts_node;

                // PRX_DEBUG_COLOR("Linking node: " << v, PRX_TEXT_GREEN);
                // PRX_DEBUG_COLOR("Linking to neighbors: " << neighbors.size(), PRX_TEXT_RED);
                motion_planning_task_space_node_t<prm_star_node_t>* msts_node = graph.get_vertex_as< motion_planning_task_space_node_t<prm_star_node_t> >(v);

                new_path.clear();
                for( size_t i = 0; i < neighbors.size(); i++ )
                {
                    neighbor_node = neighbors[i]->as< task_space_node_t >();
                    neighbor_msts_node = neighbor_node->motion_planning_node->as< motion_planning_task_space_node_t<prm_star_node_t> >();
                    new_plan.clear();

                    local_planner->steer(msts_node->task_space_node->point, neighbor_node->point, new_plan, new_path);

                    //If the path is valid
                    if( new_plan.size() != 0 && is_valid_trajectory(new_path) )
                    {
                        //Add the edge to the graph
                        double dist = metric->distance_function(graph[v]->point, neighbor_node->point);
                        undirected_edge_index_t e = graph.add_edge< prm_star_edge_t > (v, neighbor_msts_node->index, dist);
                        graph.get_edge_as<prm_star_edge_t >(e)->id = num_edges;
                        graph.get_edge_as<prm_star_edge_t >(e)->constraints = validity_checker->alloc_constraint();
                        num_edges++;
                        if( visualize_graph )
                            graph.get_edge_as< prm_star_edge_t > (e)->path = new_path;
                    }

                    new_path.clear();
                }
            }

            void tsm_prm_star_t::remove_vertex( undirected_vertex_index_t v )
            {
                foreach(undirected_vertex_index_t u, boost::adjacent_vertices(v, graph.graph))
                {
                    undirected_edge_index_t e = boost::edge(v, u, graph.graph).first;
                    prm_star_edge_t* edge = graph.get_edge_as< prm_star_edge_t >(e);
                    edge->clear(control_space);
                }

                metric->remove_point(graph.get_vertex_as< motion_planning_task_space_node_t<prm_star_node_t> >(v)->task_space_node);
                graph.clear_and_remove_vertex(v);
                num_vertices--;
            }

            bool tsm_prm_star_t::serialize()
            {
                // PRX_INFO_S(" Inside PRM serialization now, saving to file: " << serialization_file);
                char* w = std::getenv("PRACSYS_PATH");
                std::string dir(w);
                dir += ("/prx_roadmaps/");
                std::string file = dir + serialization_file;
                PRX_DEBUG_COLOR("PRM* serializing to file: " << file, PRX_TEXT_LIGHTGRAY);
                std::ofstream fout;
                fout.open(file.c_str());
                PRX_ASSERT(fout.is_open());
                graph.serialize(fout, state_space);

                if( serialize_plan )
                {

                    foreach(undirected_edge_index_t e, boost::edges(graph.graph))
                    {
                        if( graph.get_edge_as<prm_star_edge_t > (e)->plan.size() > 0 )
                        {
                            graph.get_edge_as<prm_star_edge_t > (e)->plan.save_to_stream(fout);
                        }
                        else
                        {
                            local_planner->steer(graph[boost::source(e, graph.graph)]->point, graph[boost::target(e, graph.graph)]->point, new_plan, new_path);
                            new_plan.save_to_stream(fout);
                        }
                    }
                }
                fout.close();
                return true;

            }

            bool tsm_prm_star_t::deserialize()
            {
                PRX_INFO_S(" Inside TSM PRM deserialization now, opening file: " << deserialization_file);
                char* w = std::getenv("PRACSYS_PATH");
                std::string dir(w);
                dir += ("/prx_roadmaps/");
                std::string file = dir + deserialization_file;
                PRX_INFO_S("File directory is: " << file);
                std::ifstream fin;
                if( !graph.deserialize<motion_planning_task_space_node_t<prm_star_node_t>, prm_star_edge_t > (file, fin, state_space) )
                {
                    PRX_FATAL_S("File could not deserialize!");
                    return false;
                }
                int counter = 0;

                hash_t<unsigned, undirected_vertex_index_t> node_map;
                hash_t<unsigned, undirected_edge_index_t> edge_map;

                foreach(undirected_edge_index_t e, boost::edges(graph.graph))
                {
                    if( update_weights_on_deserialize )
                    {
                        double dist = metric->distance_function(graph[boost::source(e, graph.graph)]->point, graph[boost::target(e, graph.graph)]->point);
                        graph.set_weight(e, dist);
                    }
                    graph.get_edge_as<prm_star_edge_t > (e)->id = counter;
                    graph.get_edge_as<prm_star_edge_t > (e)->plan.link_control_space(this->control_space);
                    graph.get_edge_as<prm_star_edge_t > (e)->plan.link_state_space(this->state_space);
                    counter++;
                    //added to make sure that constraint classes are allocated upon deserialize: ZL 2/17
                    graph[e]->constraints = validity_checker->alloc_constraint();
                    edge_map[graph[e]->edge_id] = e;
                }

                double val_mu = (double)boost::num_edges(graph.graph) / (double)boost::num_vertices(graph.graph);
                double diff;
                double val_dev = 0.0;

                foreach(undirected_vertex_index_t nd, boost::vertices(graph.graph))
                {
                    PRX_DEBUG_S("Added to metric: " << state_space->print_point(graph.graph[nd].node->point));
                    metric->add_point(graph[nd]->as< motion_planning_task_space_node_t<prm_star_node_t> >()->task_space_node);
                    PRX_DEBUG_S("Metric now has: " << metric->get_nr_points() << " points");
                    diff = (double)boost::out_degree(nd, graph.graph) - val_mu;
                    val_dev += diff * diff;
                    //added to make sure that constraint classes are allocated upon deserialize: ZL 2/17
                    graph[nd]->constraints = validity_checker->alloc_constraint();
                    node_map[graph[nd]->node_id] = nd;
                }
                val_dev = sqrt(val_dev);
                num_vertices = boost::num_vertices(graph.graph);
                num_edges = boost::num_edges(graph.graph);

                update_k(num_vertices);

                for (int i = 0; i < num_vertices; ++i)
                {
                    unsigned id;
                    fin>>id;
                    graph[node_map[id]]->constraints->deserialize(fin);
                }

                for (int i = 0; i < num_edges; ++i)
                {
                    unsigned id;
                    fin>>id;
                    graph[edge_map[id]]->constraints->deserialize(fin);
                }
                fin.close();

                // PRX_PRINT("Deserialized roadmap with " << boost::num_vertices(graph.graph) << " vertices ... " << boost::num_edges(graph.graph) << " edges.", PRX_TEXT_MAGENTA);
                // PRX_PRINT("Average Valence: " << val_mu << "   Valence Deviation: " << val_dev, PRX_TEXT_GREEN);

                return true;
            }

            state_t* tsm_prm_star_t::config_to_state(config_t conf)
            {
                std::vector<double> conf_vec;
                conf_vec.push_back(conf.get_position()[0]);
                conf_vec.push_back(conf.get_position()[1]);
                conf_vec.push_back(conf.get_position()[2]);
                conf_vec.push_back(conf.get_orientation()[0]);
                conf_vec.push_back(conf.get_orientation()[1]);
                conf_vec.push_back(conf.get_orientation()[2]);
                conf_vec.push_back(conf.get_orientation()[3]);
                state_t* conf_point = end_effector_space->alloc_point();
                end_effector_space->set_from_vector(conf_vec, conf_point);
                return conf_point;
            }
            
        }
    }
}



