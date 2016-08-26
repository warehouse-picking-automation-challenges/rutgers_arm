/**
 * @file bfs_tree_planner.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "planning/bfs_tree_planner.hpp"
#include "prx/planning/motion_planners/rrt/rrt_statistics.hpp"

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/utilities/goals/radial_goal_region.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/communication/visualization_comm.hpp"


#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <boost/graph/subgraph.hpp>

PLUGINLIB_EXPORT_CLASS( prx::plan::bfs_tree_planner_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace plan
    {

        bfs_tree_planner_t::bfs_tree_planner_t()
        {
            iteration_count = 0;
            color_map = {"red","blue","green","yellow","black","white"};
            solution_number = 0;
            state_space = control_space = NULL;
        }

        bfs_tree_planner_t::~bfs_tree_planner_t()
        {
            state_space->free_point(sample_point);
            //    delete statistics;
            for( unsigned i = 0; i < max_points; i++ )
            {
                state_space->free_point(pre_alloced_points[i]);
            }
            reset();
        }

        void bfs_tree_planner_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            motion_planner_t::init(reader, template_reader);

            visualize_tree = parameters::get_attribute_as<bool>("visualize_tree", reader, template_reader, true);
            visualize_solution = parameters::get_attribute_as<bool>("visualize_solution", reader, template_reader, true);
            visualization_tree_name = parameters::get_attribute_as<std::string > ("visualization_tree_name", reader, template_reader, ros::this_node::getName() + "/rrt/graph");
            visualization_solution_name = parameters::get_attribute_as<std::string > ("visualization_solution_name", reader, template_reader, ros::this_node::getName() + "/rrt/solutions");
            graph_color = parameters::get_attribute_as<std::string > ("graph_color", reader, template_reader, "white");
            solution_color = parameters::get_attribute_as<std::string > ("solution_color", reader, template_reader, "green");
            visualization_body = parameters::get_attribute_as<std::string > ("visualization_body", reader, template_reader, "");
            radius_solution = parameters::get_attribute_as<bool>("radius_solution", reader, template_reader, false);
            collision_checking = parameters::get_attribute_as<bool>("collision_checking", reader, template_reader, true);
            max_points = parameters::get_attribute_as<int>("max_points", reader, template_reader, 100000);
            num_branches = parameters::get_attribute_as<unsigned>("branches", reader, template_reader, 20);
            goal_bias_rate = parameters::get_attribute_as<double>("goal_bias_rate", reader, template_reader, 0.0);
            use_boost_random = parameters::get_attribute_as<bool>("use_boost_random", reader, template_reader, false);
        }

        void bfs_tree_planner_t::reset()
        {
            if( metric != NULL )
            {
                if( metric->get_nr_points() != 0 )
                {
                    metric->clear();
                }
            }
            tree.clear();
            iteration_count = 0;
            point_number = 0;
        }

        void bfs_tree_planner_t::setup()
        {
            states_to_check.clear();
            trajectory.link_space(state_space);
            trajectory.resize(500);
            tree.pre_alloc_memory<rrt_node_t, rrt_edge_t > (max_points);

            // Generate a random point
            util::space_point_t* rand_space_point;
            rand_space_point = state_space->alloc_point();
            srand(time(NULL));
            state_space->uniform_sample(rand_space_point);
            std::vector<double> rsp_mem = rand_space_point->memory;
            // std::cout << "rsp: ";
            // for (std::vector<double>::const_iterator i = rsp_mem.begin(); i != rsp_mem.end(); i++)
            //     std::cout << *i << " ";
            // std::cout << std::endl;

            // Random space point instead of input_specification
            // input_specification->get_seeds()[0]
            states_to_check.push_back(state_space->clone_point(rand_space_point));
            sample_point = state_space->alloc_point();
            start_vertex = tree.add_vertex<rrt_node_t, rrt_edge_t > ();

            // Random space point instead of input_specification
            tree[start_vertex]->point = state_space->clone_point(rand_space_point);
            PRX_DEBUG_S("Start state: " << state_space->print_point(tree[start_vertex]->point, 3));
            metric->add_point(tree[start_vertex]);
            clock.reset();
            if( pre_alloced_points.empty() )
            {
                for( unsigned i = 0; i < max_points; i++ )
                {
                    pre_alloced_points.push_back(state_space->alloc_point());
                }
            }
            point_number = 0;
            input_specification->get_stopping_criterion()->reset();
            vertex_queue.push(start_vertex);

            best_goal = start_vertex;

            char* w = std::getenv("PRACSYS_PATH");
            std::string dir(w);
            dir += ("/prx_output/");
            file_name = dir + "data_output_50Hz.txt";

        }

        void bfs_tree_planner_t::link_specification(specification_t* new_spec)
        {
            const space_t* old_state_space = state_space;
            const space_t* old_control_space = control_space;

            motion_planner_t::link_specification( new_spec );

            //Now, if we've changed state space
            if( old_state_space != NULL && old_state_space != state_space )
            {
                //Free the old points, and allocate new ones
                for( unsigned i=0; i<pre_alloced_points.size(); ++i )
                {
                    old_state_space->free_point(pre_alloced_points[i]);
                    pre_alloced_points[i] = state_space->alloc_point();
                }
            }
            //TODO: should probably do something for the plan and the old space too?
        }


        void bfs_tree_planner_t::step()
        {
            // //physics sim test
            // {
            //     //propagate toward the sampled point
            //     plan_t my_plan;
            //     my_plan.link_control_space(control_space);
            //     my_plan.clear();trajectory.clear();
            //     //    plan.link_state_space(state_space);
            //     local_planner->steer(tree[start_vertex]->point, sample_point, my_plan, trajectory, false);

            //     my_plan.back().duration = 2;

            //     local_planner->propagate_step(tree[start_vertex]->point, my_plan, sample_point);

            //     std::ofstream fout;
            //     fout.open(file_name.c_str(),std::ios::app);

            //     fout<<state_space->print_point(sample_point,4)<<std::endl;

            //     fout.close();
            //     iteration_count++;
            //     PRX_STATUS_S(iteration_count<<"/1000");

            //     return;
            // }

            //PRX_DEBUG_COLOR("RRT Step " << iteration_count << " : " << point_number, PRX_TEXT_LIGHTGRAY);
            //sample a state
            //get the nearest state

            // Sample random state here and loop
            
            state_space->uniform_sample(sample_point);
            tree_vertex_index_t nearest = nearest_vertex(sample_point);
            plan_t plan;
            plan.link_control_space(control_space);

            std::cout << "Start State: ";
            for (std::vector<double>::const_iterator i = sample_point->memory.begin(); i != sample_point->memory.end(); i++)
                std::cout << *i << " ";
            std::cout << std::endl;
            
            for(unsigned outer=0; outer < num_branches; outer++)
            {
                // Otherwise, our dice said to randomly sample
                sampler->sample(state_space, sample_point);

                //propagate toward the sampled point
                plan.clear();
                //    plan.link_state_space(state_space);
                // PRX_INFO_S(state_space->get_space_name()<<" "<<state_space->print_point(tree[nearest]->point));
                local_planner->steer(tree[nearest]->point, sample_point, plan, trajectory, false);
                //check if the trajectory is valid
                // || (validity_checker->is_valid(trajectory) && trajectory.size() > 1)
                if( !collision_checking || (validity_checker->is_valid(trajectory) && trajectory.size() > 1 ))
                {
                    tree_vertex_index_t v = tree.add_vertex<rrt_node_t, rrt_edge_t > ();
                    state_space->copy_point(pre_alloced_points[point_number], trajectory[trajectory.size() - 1]);
                    tree[v]->point = pre_alloced_points[point_number]; 
                    tree.get_vertex_as<rrt_node_t>(v)->cost = tree.get_vertex_as<rrt_node_t>(nearest)->cost + validity_checker->trajectory_cost(trajectory);

                    if(get_vertex(v)->cost > get_vertex(best_goal)->cost)
                    {
                        best_goal = v;
                    }

                    //output the data
                    if(plan.size()>1)
                    {
                        std::ofstream fout;
                        fout.open(file_name.c_str(),std::ios::app);
                        int skip_count = 0;
                        fout<<state_space->serialize_point(trajectory[0])<<std::endl;
                        foreach(plan_step_t& step, plan)
                        {
                            fout<<control_space->serialize_point(step.control)<<std::endl;
                            fout<<step.duration<<std::endl;
                            skip_count += step.duration/simulation::simulation_step;
                            fout<<state_space->serialize_point(trajectory[skip_count])<<std::endl;
                        }
                        fout<<std::endl;
                        fout.close();
                    }
                    else
                    {
                        std::ofstream fout;
                        fout.open(file_name.c_str(),std::ios::app);
                        int skip_count = 0;
                        int skip_number = 1.0/simulation::simulation_step;
                        skip_number /=50;
                        fout<<state_space->serialize_point(trajectory[0])<<std::endl;
                        for(unsigned i = skip_number;i < trajectory.size();i += skip_number)
                        {
                            fout<<control_space->serialize_point(plan.back().control)<<std::endl;
                            fout<<.02<<std::endl;
                            fout<<state_space->serialize_point(trajectory[i])<<std::endl;
                        }
                        fout<<std::endl;

                        fout.close();
                    }



                    state_space->copy_point(states_to_check[0], tree[v]->point);

                    tree_edge_index_t e = tree.add_edge<rrt_edge_t > (nearest, v);
                    get_edge(e)->plan = plan;
                    if( visualize_tree )
                    {
                        get_edge(e)->trajectory = trajectory;
                    }

                    point_number++;
                    vertex_queue.push(v);
                    if(point_number == max_points)
                    {
                        pre_alloced_points.resize(max_points+max_points/10);

                        for(unsigned i=max_points;i<max_points+max_points/10;i++)
                        {
                            pre_alloced_points[i] = (state_space->alloc_point());
                        }
                        max_points+=max_points/10;
                    }
                }
                // PRX_INFO_S(iteration_count<<" "<<outer);
            }   
            iteration_count++;
            // PRX_INFO_S(iteration_count);
        }

        bool bfs_tree_planner_t::succeeded() const
        {
            if( input_specification->get_stopping_criterion()->satisfied() )
                return true;
            return false;
        }

        void bfs_tree_planner_t::resolve_query()
        {
            // solution_number++;
            // unsigned goals_size; 
            // std::vector<space_point_t*> goals = input_query->get_goal()->get_goal_points(goals_size);

            std::vector<tree_vertex_index_t> v_goals;
            // v_goals.clear();

            // input_query->clear();
            // radial_goal_region_t* region = dynamic_cast<radial_goal_region_t*>(input_query->get_goal());

            // for(unsigned i=0; i<goals_size; ++i)
            // {
            //     std::vector<const abstract_node_t*> source = metric->radius_query(goals[i], region->get_radius());

            //     foreach(const abstract_node_t* node, source)
            //     {
            //         v_goals.push_back(((const tree_node_t*)node)->get_index());
            //     }
            // }

            // //At this point, the list of vertices that are candidate solutions have been found.
            // //  Now, depending on the type of query requested, perform collision checking on the paths.

            // // PRX_WARN_S("RRT is resolving query without doing collision checking.");
            tree_vertex_index_t new_v;
            // double cost = PRX_INFINITY;
            // bool found = false;

            // foreach(tree_vertex_index_t v, v_goals)
            // {
            //     if( input_query->get_goal()->satisfied(tree[v]->point) && get_vertex(v)->cost < cost )
            //     {
            //         found = true;
            //         cost = get_vertex(v)->cost;
            //         new_v = v;
            //     }
            // }
            // v_goals.clear();
            v_goals.push_back(best_goal);
            // if( !found )
            //     return;
            new_v = v_goals[0];

            input_query->plan.clear();
            input_query->path.clear();

            while( get_vertex(new_v)->get_parent() != new_v )
            {
                tree_edge_index_t e = tree.edge(get_vertex(new_v)->get_parent(), new_v);
                rrt_edge_t* edge = get_edge(e);

                plan_t hold_plan(edge->plan);
                hold_plan += input_query->plan;
                input_query->plan = hold_plan;
                new_v = get_vertex(new_v)->get_parent();
                // PRX_PRINT(input_query->plan.print(),PRX_TEXT_CYAN);
            }
            local_planner->propagate(tree[start_vertex]->point, input_query->plan, input_query->path);
        }

        void bfs_tree_planner_t::update_vis_info() const
        {
            std::vector<geometry_info_t> geoms;
            std::vector<config_t> configs;
            hash_t<std::string, std::vector<double> > map_params;
            std::vector<double> params;
            PRX_WARN_S("Vis sol name: " << visualization_solution_name << " , tree name: " << visualization_tree_name);

            int count;
            if( visualize_tree )
            {
                PRX_WARN_S("Visualizing tree! " << visualization_body);
                count = 0;
                std::vector<std::string> system_names;
                system_names.push_back(visualization_body);

                foreach(tree_edge_t* e, tree.edges())
                {
                    std::string name = visualization_tree_name + "/edge_" + int_to_str(count);
                    params.clear();

                    foreach(state_t* state, get_edge(e->get_index())->trajectory)
                    {
                        map_params.clear();
                        ((comm::visualization_comm_t*)comm::vis_comm)->compute_configs(state, system_names, map_params);
                        params.insert(params.end(), map_params[system_names[0]].begin(), map_params[system_names[0]].end());
                        //HACK HACK HACK
                        // params.push_back(state->at(0));
                        // params.push_back(state->at(1));
                        // params.push_back(state->at(2));

                    }

                    geoms.push_back(geometry_info_t(visualization_body, name, PRX_LINESTRIP, params, graph_color));
                    configs.push_back(config_t());

                    count++;
                }
                ((comm::visualization_comm_t*)comm::vis_comm)->visualization_geom_map[visualization_tree_name] = geoms;
                ((comm::visualization_comm_t*)comm::vis_comm)->visualization_configs_map[visualization_tree_name] = configs;
                geoms.clear();
                configs.clear();
            }


            if( visualize_solution )
            {
                PRX_WARN_S("Visualizing solution! " << visualization_body);
                std::vector<std::string> system_names;
                system_names.push_back(visualization_body);
                if( input_query->path.size() < 2 )
                {
                    //            return;
                }
                else
                {
                    params.clear();
                    for( size_t i = 0; i < input_query->path.size(); i++ )
                    {
                        map_params.clear();
                        //HACK HACK HACK
                        params.push_back(input_query->path[i]->at(0));
                        params.push_back(input_query->path[i]->at(1));
                        params.push_back(input_query->path[i]->at(2));
                        // ((comm::visualization_comm_t*)comm::vis_comm)->compute_configs(input_query->path[i], system_names, map_params);
                        // params.insert(params.end(), map_params[visualization_body].begin(), map_params[visualization_body].end());
                        //params.back() += 3; //this is so the solution will be above
                    }

                    std::string name = visualization_solution_name + "/" + visualization_body + "/path";//_" + int_to_str(solution_number);
                    geoms.push_back(geometry_info_t(visualization_body, name, PRX_LINESTRIP, params, color_map[solution_number % color_map.size()]));
                    configs.push_back(config_t());
                    ((comm::visualization_comm_t*)comm::vis_comm)->visualization_geom_map[visualization_solution_name] = geoms;
                    ((comm::visualization_comm_t*)comm::vis_comm)->visualization_configs_map[visualization_solution_name] = configs;
                    geoms.clear();
                    configs.clear();
                }

            }
        }

        rrt_node_t* bfs_tree_planner_t::get_vertex(tree_vertex_index_t v) const
        {
            return tree.get_vertex_as<rrt_node_t > (v);
        }

        rrt_edge_t* bfs_tree_planner_t::get_edge(tree_edge_index_t e) const
        {
            return tree.get_edge_as<rrt_edge_t > (e);
        }

        tree_vertex_index_t bfs_tree_planner_t::nearest_vertex(const state_t* state)
        {
            if(vertex_queue.empty())
                return start_vertex;
            tree_vertex_index_t v = vertex_queue.front();
            vertex_queue.pop();
            return v;
        }

        bool bfs_tree_planner_t::serialize()
        {
            PRX_WARN_S("Serialize in RRT is unimplemented");
            return false;
        }

        bool bfs_tree_planner_t::deserialize()
        {
            PRX_WARN_S("Deserialize in RRT is unimplemented");
            return false;
        }

        const statistics_t* bfs_tree_planner_t::get_statistics()
        {
            statistics = new rrt_statistics_t();
            statistics->as<rrt_statistics_t > ()->num_vertices = tree.num_vertices();
            statistics->as<rrt_statistics_t > ()->solution_quality = validity_checker->trajectory_cost(input_query->path);
            statistics->time = clock.measure();
            statistics->steps = iteration_count;

            return statistics;
        }


        void bfs_tree_planner_t::prune_trajectory(sim::plan_t& new_plan, sim::trajectory_t& new_trajectory, sim::state_t* new_end_state)
        {
            unsigned num_states = new_trajectory.size();
            for(unsigned i=1;i<num_states;i++)
            {
                if(input_query->get_goal()->satisfied(new_trajectory[i]))
                {
                    if(new_end_state!=NULL)
                        state_space->copy_point(new_end_state,new_trajectory[i]);
                    double duration = i*simulation::simulation_step;
                    new_plan.trim(duration);
                    new_trajectory.chop(i+1);
                    break;
                }
            }
        }

    }
}
