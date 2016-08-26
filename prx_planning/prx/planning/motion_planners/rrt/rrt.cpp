/**
 * @file rrt.cpp
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

#include "prx/planning/motion_planners/rrt/rrt.hpp"
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
#include "prx/planning/motion_planners/motion_planner.hpp"


#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <boost/graph/subgraph.hpp>

PLUGINLIB_EXPORT_CLASS( prx::plan::rrt_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace plan
    {

        rrt_t::rrt_t()
        {
            iteration_count = 0;
            color_map = {"red","blue","green","yellow","black"};
            solution_number = 0;
            state_space = control_space = NULL;
            satisfied_goal_vertex.clear();
            max_edges = 0;
            max_goals = 0;
        }

        rrt_t::~rrt_t()
        {
            state_space->free_point(sample_point);
            //    delete statistics;
            for( unsigned i = 0; i < max_points; i++ )
            {
                state_space->free_point(pre_alloced_points[i]);
            }
            reset();
        }

        void rrt_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            motion_planner_t::init(reader, template_reader);

            visualize_tree = parameters::get_attribute_as<bool>("visualize_tree", reader, template_reader, true);
            visualize_solution = parameters::get_attribute_as<bool>("visualize_solution", reader, template_reader, true);
            visualization_tree_name = parameters::get_attribute_as<std::string > ("visualization_tree_name", reader, template_reader, ros::this_node::getName() + "/rrt/graph");
            visualization_solution_name = parameters::get_attribute_as<std::string > ("visualization_solution_name", reader, template_reader, ros::this_node::getName() + "/rrt/solutions");
            graph_color = parameters::get_attribute_as<std::string > ("graph_color", reader, template_reader, "white");
            solution_color = parameters::get_attribute_as<std::string > ("solution_color", reader, template_reader, "green");
            visualization_body = "";
            if( parameters::has_attribute("visualization_bodies", reader, template_reader) )
                visualization_body = (parameters::get_attribute_as<std::vector<std::string> >("visualization_bodies", reader, template_reader))[0];
            visualization_body = parameters::get_attribute_as<std::string > ("visualization_body", reader, template_reader, visualization_body);
            radius_solution = parameters::get_attribute_as<bool>("radius_solution", reader, template_reader, false);
            collision_checking = parameters::get_attribute_as<bool>("collision_checking", reader, template_reader, true);
            max_points = parameters::get_attribute_as<int>("max_points", reader, template_reader, 100000);
            goal_bias_rate = parameters::get_attribute_as<double>("goal_bias_rate", reader, template_reader, 0.0);
            use_boost_random = parameters::get_attribute_as<bool>("use_boost_random", reader, template_reader, false);
            smooth_solution = parameters::get_attribute_as<bool>("smooth_solution", reader, template_reader, false);
        }

        void rrt_t::reset()
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
            satisfied_goal_vertex.clear();
            states_to_check.clear();
            input_specification->get_stopping_criterion()->reset();
        }

        void rrt_t::setup()
        {
            states_to_check.clear();
            trajectory.link_space(state_space);
            trajectory.resize(500);
            tree.pre_alloc_memory<rrt_node_t, rrt_edge_t > (max_points);
            // states_to_check.push_back(state_space->clone_point(input_specification->get_seeds()[0]));
            sample_point = state_space->alloc_point();
            // start_vertex = tree.add_vertex<rrt_node_t, rrt_edge_t > ();
            // tree[start_vertex]->point = state_space->clone_point(input_specification->get_seeds()[0]);
            // PRX_DEBUG_S("Start state: " << state_space->print_point(tree[start_vertex]->point, 3));
            // metric->add_point(tree[start_vertex]);
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
            tmp_constraint = input_specification->validity_checker->alloc_constraint();


            char* w = std::getenv("PRACSYS_PATH");
            std::string dir(w);
            dir += ("/prx_output/");
            int hz = 1.0/simulation::simulation_step;
            file_name = dir + int_to_str(hz)+"Hz.txt";

        }

        void rrt_t::link_specification(specification_t* new_spec)
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

        void rrt_t::link_query(query_t* new_query)
        {
            motion_planner_t::link_query(new_query);
            reset();
            if(input_query->search_mode == LAZY_SEARCH)
            {
                PRX_ERROR_S("RRT is not supposed to be invoked with Lazy Search. Reverting the mode to STANDARD_SEARCH");
                PRX_ERROR_S("ZL How the heck am I supposed to be able to run without calling the generate constraints if I can't run a lazy \"search\" in the tree?");
                input_query->search_mode = LAZY_SEARCH;
            }
        }

        void rrt_t::random_sample()
        {
            if( roll_weighted_die({goal_bias_rate,1 - goal_bias_rate}, use_boost_random) == 0 )
            {
                // This will ONLY check the first goal.
                // TODO: Multiple goals?
                unsigned num_of_goals = 1;
                std::vector<state_t*> goal_points = input_query->get_goal()->get_goal_points(num_of_goals);
                if(num_of_goals == 1)
                {
                    state_space->copy_point(sample_point, input_query->get_goal()->get_first_goal_point());
                }
                else if (num_of_goals > 1)
                {
                    state_space->copy_point(sample_point, goal_points[uniform_int_random(0,num_of_goals-1)]);
                }
                else
                {
                    PRX_ERROR_S("No Goal Points to sample from!");
                    sampler->sample(state_space, sample_point);
                }

                
            }
            else
            {
                    // Otherwise, our dice said to randomly sample
                sampler->sample(state_space, sample_point);
            }
        }

        void rrt_t::increment_point_number()
        {
            point_number++;
            //Increase the preallocated buffer
            if(point_number == max_points)
            {
                for(unsigned i=0;i<max_points;i++)
                {
                    pre_alloced_points.push_back(state_space->alloc_point());
                }
                max_points*=2;
            }
        }

        bool rrt_t::check_path_for_collisions_and_constraints(constraints_t* constraints, trajectory_t& path_to_evaluate)
        {
            //Check if the path is valid and whether any of the constraints on the path are valid
            if(input_query->search_mode==LAZY_SEARCH)
            {
                //TODO ZL This is added here because I don't want tree planners to deal with constraints (at least not in this way)
                return validity_checker->is_valid(path_to_evaluate);
            }
            else if(input_query->search_mode==STANDARD_SEARCH)
            {
                return validity_checker->validate_and_generate_constraints(constraints, path_to_evaluate) && !constraints->has_intersection(input_query->active_constraints);
            }
            else if(input_query->search_mode==TRUSTED_MCR || input_query->search_mode==UNTRUSTED_MCR)
            {
                return validity_checker->validate_and_generate_constraints(constraints, path_to_evaluate);
            }
            PRX_FATAL_S("Running RRT in an invalid search mode");
        }

        std::pair<bool, tree_vertex_index_t> rrt_t::add_node(space_point_t* n_state)
        {
            //get the nearest state on the tree, to the randomly sampled point
            tree_vertex_index_t nearest = nearest_vertex(n_state);
            tree_vertex_index_t v;
            //If the sampled point is already in the tree
            if( state_space->equal_points(n_state,tree[nearest]->point))
            {
                return std::make_pair(false, nearest);
            }

            //propagate toward the sampled point
            plan_t plan;
            plan.link_control_space(control_space);
            local_planner->steer(tree[nearest]->point, n_state, plan, trajectory, false);
            prune_trajectory(plan,trajectory);
            //Get pruned connected state
            state_space->copy_point(n_state, trajectory[trajectory.size() - 1]);
            //Check collisions over the trajectory
            // if( !collision_checking || (validity_checker->is_valid(trajectory) && trajectory.size() > 1) )
            if( !collision_checking || ( trajectory.size() > 1 && check_path_for_collisions_and_constraints(tmp_constraint, trajectory) ) )
            {
                //*******************Add a new vertex ******************
                v = tree.add_vertex<rrt_node_t, rrt_edge_t > ();
                state_space->copy_point(pre_alloced_points[point_number], n_state);
                tree[v]->point = pre_alloced_points[point_number];
                metric->add_point(tree[v]);
                tree.get_vertex_as<rrt_node_t>(v)->cost = tree.get_vertex_as<rrt_node_t>(nearest)->cost + validity_checker->trajectory_cost(trajectory);
                state_space->copy_point(states_to_check[0], tree[v]->point);
                //******************Added a new vertex *****************

                //********************Add a new edge *******************
                tree_edge_index_t e = tree.add_edge<rrt_edge_t > (nearest, v);
                get_edge(e)->plan = plan;
                if( visualize_tree )
                {
                    get_edge(e)->trajectory = trajectory;
                }
                //*******************Added a new edge ******************

                increment_point_number();
                return std::make_pair(true, v);

            }
            return std::make_pair(false, v);
        }


        bool rrt_t::execute()
        {
            //Do nothing since there is no preprocessing for rrt planners
            throw stopping_criteria_t::stopping_criteria_satisfied(" Stopping criteria is satisfied ");
            return true;
        }
            

        void rrt_t::step()
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

            // We roll a weighted die to find out whether to bias towards the goal.
            // If you want replicatable results, set the second parameter to false.
            // if( roll_weighted_die({goal_bias_rate,1 - goal_bias_rate), use_boost_random) == 0 )
            // {
            //     // This will ONLY check the first goal.
            //     // TODO: Multiple goals?
            //     unsigned num_of_goals = 1;
            //     std::vector<state_t*> goal_points = input_query->get_goal()->get_goal_points(num_of_goals);
            //     if(num_of_goals == 1)
            //     {
            //         state_space->copy_point(sample_point, input_query->get_goal()->get_first_goal_point());
            //     }
            //     else if (num_of_goals > 1)
            //     {
            //         state_space->copy_point(sample_point, goal_points[uniform_int_random(0,num_of_goals-1)]);
            //     }
            //     else
            //     {
            //         PRX_ERROR_S("No Goal Points to sample from!");
            //         sampler->sample(state_space, sample_point);
            //     }

                
            // }
            // else
            // {
            //         // Otherwise, our dice said to randomly sample
            //     sampler->sample(state_space, sample_point);
            // }

            // //get the nearest state
            // tree_vertex_index_t nearest = nearest_vertex(sample_point);
            // if( state_space->equal_points(sample_point,tree[nearest]->point))
            // {
            //     iteration_count++;
            //     return;
            // }
            // //propagate toward the sampled point
            // plan_t plan;
            // plan.link_control_space(control_space);
            // //    plan.link_state_space(state_space);
            // local_planner->steer(tree[nearest]->point, sample_point, plan, trajectory, false);
            // //check if the trajectory is valid
            // if( !collision_checking || (validity_checker->is_valid(trajectory) && trajectory.size() > 1) )
            // {
            //     tree_vertex_index_t v = tree.add_vertex<rrt_node_t, rrt_edge_t > ();
            //     prune_trajectory(plan,trajectory);
            //     state_space->copy_point(pre_alloced_points[point_number], trajectory[trajectory.size() - 1]);
            //     tree[v]->point = pre_alloced_points[point_number]; //state_space->clone_point(traj.states.back());
            //     metric->add_point(tree[v]);
            //     tree.get_vertex_as<rrt_node_t>(v)->cost = tree.get_vertex_as<rrt_node_t>(nearest)->cost + validity_checker->trajectory_cost(trajectory);

            //     //PRX_DEBUG_COLOR("[RRT] Added point: " << state_space->print_point(tree[v]->point, 3), PRX_TEXT_LIGHTGRAY);

            //     state_space->copy_point(states_to_check[0], tree[v]->point);

            //     tree_edge_index_t e = tree.add_edge<rrt_edge_t > (nearest, v);
            //     get_edge(e)->plan = plan;

            //     if( visualize_tree )
            //     {
            //         get_edge(e)->trajectory = trajectory;
            //     }
            //     point_number++;
            //     if(point_number == max_points)
            //     {
            //         for(unsigned i=0;i<max_points;i++)
            //         {
            //             pre_alloced_points.push_back(state_space->alloc_point());
            //         }
            //         max_points*=2;
            //     }
            // }
            // iteration_count++;
            // PRX_STATUS("RRT Step ["<<iteration_count<<"] ... "<<point_number, PRX_TEXT_BROWN);



            
            iteration_count++;
            //Sample a point at random
            random_sample();

            //Vertex index of added vertex
            tree_vertex_index_t added_vertex;
            //Check whether the sampled point could be added to the tree
            bool add_check;
            boost::tie(add_check, added_vertex) = add_node(sample_point);

            //If a new node was added to the tree
            if(add_check)
            {
                //Check if the added point satisfies the goal
                //PRX_ASSERT(input_query->get_goal()->satisfied(states_to_check[0]) == input_query->get_goal()->satisfied(tree[added_vertex]->point));
                //PRX_DEBUG_COLOR(input_query->get_goal() << " : " << state_space->print_point(states_to_check[0], 3), PRX_TEXT_LIGHTGRAY);
                if(input_query->get_goal()->satisfied(tree[added_vertex]->point))
                {
                    //PRX_PRINT ("Goal satisfied?", PRX_TEXT_BROWN);
                    //The cost to the last satisfied goal is worse than the current satisfied goal
                    if(satisfied_goal_vertex.empty() || get_vertex(satisfied_goal_vertex[0])->cost > get_vertex(added_vertex)->cost)
                    {
                        //PRX_PRINT ("Added satisfied goal!", PRX_TEXT_CYAN);

                        //Current satisfied goal is the least cost goal discovered till now
                        satisfied_goal_vertex.clear();
                        satisfied_goal_vertex.push_back(added_vertex);
                    }

                }
            }


            PRX_STATUS("RRT Step ["<<iteration_count<<"] ... "<<point_number, PRX_TEXT_BROWN);
        }

        bool rrt_t::succeeded() const
        {
            if( input_specification->get_stopping_criterion()->satisfied() )
                return true;
            return false;
        }

        void rrt_t::retrace_path(plan_t& plan, tree_vertex_index_t new_v, tree_vertex_index_t root_v)
        {
            if(smooth_solution)
            {
                std::vector<tree_vertex_index_t> v_path;
                v_path.push_back(new_v);
                while( get_vertex(new_v)->get_parent() != new_v )
                {
                    new_v = get_vertex(new_v)->get_parent();
                    v_path.insert(v_path.begin(),new_v);
                }

                PRX_WARN_S("\n\nRRT Path Smoothing:::");
                trajectory_t smooth_traj(state_space);
                plan_t smooth_plan(control_space);
                constraints_t* tmp_constraint = validity_checker->alloc_constraint();
                for(int i=0; i<v_path.size()-1; ++i)
                {
                    bool smoothed = false;
                    for(int j=v_path.size()-1; j>i+1; j--)
                    {
                        smooth_plan.clear();
                        smooth_traj.clear();
                        local_planner->steer(tree[v_path[i]]->point, tree[v_path[j]]->point, smooth_plan, smooth_traj, true);
                        if(!collision_checking || ( smooth_traj.size() > 1 && check_path_for_collisions_and_constraints(tmp_constraint, smooth_traj) ) )
                        {
                            PRX_WARN_S("Smoothing path: "<<i<<" <-> "<<j);
                            plan+=smooth_plan;
                            smoothed = true;
                            i=j-1;
                            break;
                        }
                        else
                        {
                            PRX_ERROR_S("NOT Smoothing path: "<<i<<" <-> "<<j);
                        }
                    }

                    if(!smoothed)
                    {
                        PRX_ERROR_S("Tree edge: "<<i<<" <-> "<<i+1);
                        tree_edge_index_t e = tree.edge(v_path[i], v_path[i+1]);
                        rrt_edge_t* edge = get_edge(e);

                        plan+=edge->plan;
                    }
                }
            }
            else
            {
                bool retraced_till_root = true;
                while( get_vertex(new_v)->get_parent() != new_v )
                {
                    tree_edge_index_t e = tree.edge(get_vertex(new_v)->get_parent(), new_v);
                    rrt_edge_t* edge = get_edge(e);

                    plan_t hold_plan(edge->plan);
                    hold_plan += plan;
                    plan = hold_plan;

                    //If finding a path to an intermediate node and not the root of the tree
                    if(new_v == root_v)
                    {   
                        retraced_till_root = false;
                        return;
                    }

                    new_v = get_vertex(new_v)->get_parent();
                }

                //If passed in a root vertex but reached the root of the tree.
                if(retraced_till_root && start_vertex != root_v)
                {
                    PRX_ERROR_S("Tried to retrace to a specified vertex but ended up retracing to the root of the tree.");
                }
            }
        }

        void rrt_t::resolve_query()
        {
            // // solution_number++;
            // unsigned goals_size; 
            // std::vector<space_point_t*> goals = input_query->get_goal()->get_goal_points(goals_size);

            // std::vector<tree_vertex_index_t> v_goals;
            // v_goals.clear();

            // input_query->clear();
            // if( radius_solution )
            // {
            //     radial_goal_region_t* region = dynamic_cast<radial_goal_region_t*>(input_query->get_goal());

            //     for(unsigned i=0; i<goals_size; ++i)
            //     {
            //         std::vector<const abstract_node_t*> source = metric->radius_query(goals[i], region->get_radius());

            //         foreach(const abstract_node_t* node, source)
            //         {
            //             v_goals.push_back(((const tree_node_t*)node)->get_index());
            //         }
            //     }
            // }
            // else
            // {

            //     for(unsigned i=0; i<goals_size; ++i)
            //     {
            //         tree_vertex_index_t source = nearest_vertex(goals[i]);
            //         v_goals.push_back(source);
            //     }
            // }

            // //At this point, the list of vertices that are candidate solutions have been found.
            // //  Now, depending on the type of query requested, perform collision checking on the paths.

            // // PRX_WARN_S("RRT is resolving query without doing collision checking.");
            // tree_vertex_index_t new_v;
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
            // v_goals.push_back(new_v);
            // if( !found )
            //     return;
            // new_v = v_goals[0];

            // while( get_vertex(new_v)->get_parent() != new_v )
            // {
            //     tree_edge_index_t e = tree.edge(get_vertex(new_v)->get_parent(), new_v);
            //     rrt_edge_t* edge = get_edge(e);

            //     plan_t hold_plan(edge->plan);
            //     hold_plan += input_query->plan;
            //     input_query->plan = hold_plan;
            //     new_v = get_vertex(new_v)->get_parent();
            // }
            // local_planner->propagate(tree[start_vertex]->point, input_query->plan, input_query->path);



            solution_number++;
            //*******************Add the start vertex ******************
            if(tree.num_vertices()==0)
            {
                start_vertex = tree.add_vertex<rrt_node_t, rrt_edge_t > ();
                tree[start_vertex]->point = state_space->clone_point(input_query->get_start_state());
                // PRX_WARN_S("Start state: " << state_space->print_point(tree[start_vertex]->point, 3));
                if( !input_specification->validity_checker->is_valid( tree[start_vertex]->point ) )
                {
                    PRX_ERROR_S("RRT start state is NOT VALID: aborting!!");
                    return;
                }
                metric->add_point(tree[start_vertex]);
                states_to_check.clear();
                states_to_check.push_back(state_space->clone_point(tree[start_vertex]->point));
            }
            //******************Added the start vertex *****************

            //*******************Step till stopping criterion succeeds ******************
            try
            {
                do
                {
                    step();

                }while(!succeeded() && ros::ok());
                if(!ros::ok())
                {
                    exit(0);
                }
            }
            catch( stopping_criteria_t::stopping_criteria_satisfied e )                
            {
                //If no goal was satisfied during any of the steps
                if(satisfied_goal_vertex.empty())
                {
                    PRX_DEBUG_COLOR("No Goal Was Satisfied.", PRX_TEXT_CYAN);
                    input_query->found_solution = false;
                }
                else
                {   
                    //Some goal was satisfied
                    input_query->found_solution = true;
                    // PRX_ASSERT(satisfied_goal_vertex.size()==1);
                    tree_vertex_index_t new_v = satisfied_goal_vertex.back();
                    //Retrace a path from the satisfied goal to the start and store the resultant plan in the input query
                    input_query->plan.clear();
                    input_query->path.clear();
                    retrace_path(input_query->plan, new_v, start_vertex);
                    //Propagate and generate the path for the input query
                    local_planner->propagate(tree[start_vertex]->point, input_query->plan, input_query->path);
                    //Generate constraints over the solution path                
                    check_path_for_collisions_and_constraints(tmp_constraint, input_query->path);
                    //TODO: Must put this information on some constraint structure in the query
                    constraints_t* query_path_constraints = validity_checker->alloc_constraint();
                    //Find the constraints on the path that are currently active

                    //TODO ZL Someone else needs to make sure this is correct, because it crashes the RRT when no active constraints are specified
                    if(input_query->active_constraints!=NULL)
                        tmp_constraint->intersect(query_path_constraints, input_query->active_constraints);
                    //Find the cost of the solution path
                    input_query->solution_cost = validity_checker->true_cost(input_query->path,input_query->plan);
                    // PRX_PRINT("Found a solution of length "<<input_query->solution_cost<<" to the goal "<<state_space->print_point(tree[new_v]->point, 3), PRX_TEXT_CYAN);
                    // PRX_PRINT("Solution path: "<<input_query->path.print(3), PRX_TEXT_LIGHTGRAY);
                }
            }
            catch( stopping_criteria_t::interruption_criteria_satisfied e )                
            {
                //If no goal was satisfied during any of the steps
                if(satisfied_goal_vertex.empty())
                {
                    PRX_DEBUG_COLOR("No Goal Was Satisfied.", PRX_TEXT_CYAN);
                    input_query->found_solution = false;
                }
                else
                {   
                    //Some goal was satisfied
                    input_query->found_solution = true;
                    input_query->plan.clear();
                    input_query->path.clear();


                    if(satisfied_goal_vertex.size()>0)
                    {
                        tree_vertex_index_t new_v = satisfied_goal_vertex.back();
                        //Retrace a path from the satisfied goal to the start and store the resultant plan in the input query
                        input_query->plan.clear();
                        input_query->path.clear();
                        retrace_path(input_query->plan, new_v, start_vertex);
                        //Propagate and generate the path for the input query
                        local_planner->propagate(tree[start_vertex]->point, input_query->plan, input_query->path);
                        //Generate constraints over the solution path         
                        check_path_for_collisions_and_constraints(tmp_constraint, input_query->path);
                        //TODO: Must put this information on some constraint structure in the query
                        constraints_t* query_path_constraints = validity_checker->alloc_constraint();
                        //Find the constraints on the path that are currently active
                        //TODO ZL Someone else needs to make sure this is correct, because it crashes the RRT when no active constraints are specified
                        if(input_query->active_constraints!=NULL)
                            tmp_constraint->intersect(query_path_constraints, input_query->active_constraints);
                        //Find the cost of the solution path
                        input_query->solution_cost = validity_checker->true_cost(input_query->path,input_query->plan);
                        // PRX_PRINT("Solution path: "<<input_query->path.print(3), PRX_TEXT_LIGHTGRAY);
                    }
                }
                throw e;
            }

            //If the tree needs to be visualized then update_vis_info needs to be explicitly called here.
            if(visualize_tree || visualize_solution)
            {
                update_vis_info();
                ((comm::visualization_comm_t*)comm::vis_comm)->send_geometries();
            }

        }

        void rrt_t::update_vis_info() const
        {
            if(tree.num_vertices()<=1)
            {
                //If the tree is empy then skip visualization
                return;
            }

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
                        
                        // params.push_back(state->at(0));
                        // params.push_back(state->at(1));
                        // params.push_back(state->at(2));
                        map_params.clear();
                        ((comm::visualization_comm_t*)comm::vis_comm)->compute_configs(state, system_names, map_params);
                        params.insert(params.end(), map_params[system_names[0]].begin(), map_params[system_names[0]].end());

                    }

                    

                    geoms.push_back(geometry_info_t(visualization_body, name, PRX_LINESTRIP, params, graph_color));
                    configs.push_back(config_t());

                    count++;
                }

                for(int i=count; i<=max_edges; ++i)
                {
                    params.clear();
                    params = {0,0,0,0,0,0};
                    geoms.push_back(geometry_info_t(visualization_body, visualization_tree_name + "/edge_" + int_to_str(i), PRX_LINESTRIP, params, graph_color));
                    configs.push_back(config_t());
                }

                //Update the maximum number edges in a generated tree to properly reset visualized edges.
                if(point_number>max_edges)
                {   
                    max_edges = point_number;
                }

                unsigned num_goals;
                std::vector<state_t*> goal_points = input_query->get_goal()->get_goal_points(num_goals);
                goal_points.resize(num_goals);
                int goal_iter;
                for(goal_iter=0; goal_iter<goal_points.size(); ++goal_iter)
                {
                    state_t* state = goal_points[goal_iter];
                    std::string name = visualization_tree_name + "/goal_" + int_to_str(goal_iter);
                    params.clear();
                    params.push_back(0.01);
                    ((comm::visualization_comm_t*)comm::vis_comm)->compute_configs(state, system_names, map_params);
                    geoms.push_back(geometry_info_t(visualization_body, name, PRX_SPHERE, params, "red"));
                    configs.push_back(config_t(vector_t(map_params[visualization_body][0], map_params[visualization_body][1], map_params[visualization_body][2]), quaternion_t()));

                }

                for( ; goal_iter<max_goals; ++goal_iter)
                {
                    std::string name = visualization_tree_name + "/goal_" + int_to_str(goal_iter);
                    geoms.push_back(geometry_info_t(visualization_body, name, PRX_SPHERE, params, "red"));
                    configs.push_back(config_t(vector_t(0, 0, 0), quaternion_t()));
                }

                //Update the maximum number of goals in a generated tree to properly reset visualized goals.
                if(num_goals>max_goals)
                {
                    max_goals = num_goals;
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
                    params.clear();
                    params = {0,0,0,0,0,0};

                    std::string name = visualization_solution_name + "/" + visualization_body + "/path";//_" + int_to_str(solution_number);
                    geoms.push_back(geometry_info_t(visualization_body, name, PRX_LINESTRIP, params, color_map[solution_number % color_map.size()]));
                    configs.push_back(config_t());
                    ((comm::visualization_comm_t*)comm::vis_comm)->visualization_geom_map[visualization_solution_name] = geoms;
                    ((comm::visualization_comm_t*)comm::vis_comm)->visualization_configs_map[visualization_solution_name] = configs;
                    geoms.clear();
                    configs.clear();
                }
                else
                {
                    params.clear();
                    for( size_t i = 0; i < input_query->path.size(); i++ )
                    {
                        map_params.clear();
                        //HACK HACK HACK
                        // params.push_back(input_query->path[i]->at(0));
                        // params.push_back(input_query->path[i]->at(1));
                        // params.push_back(input_query->path[i]->at(2));
                        ((comm::visualization_comm_t*)comm::vis_comm)->compute_configs(input_query->path[i], system_names, map_params);
                        map_params[visualization_body][0]+=0.005;
                        params.insert(params.end(), map_params[visualization_body].begin(), map_params[visualization_body].end());
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

        rrt_node_t* rrt_t::get_vertex(tree_vertex_index_t v) const
        {
            return tree.get_vertex_as<rrt_node_t > (v);
        }

        rrt_edge_t* rrt_t::get_edge(tree_edge_index_t e) const
        {
            return tree.get_edge_as<rrt_edge_t > (e);
        }

        tree_vertex_index_t rrt_t::nearest_vertex(const state_t* state) const
        {
            return metric->single_query(state)->as<rrt_node_t > ()->get_index();
        }

        bool rrt_t::serialize()
        {
            PRX_WARN_S("Serialize in RRT is unimplemented");
            return false;
        }

        bool rrt_t::deserialize()
        {
            PRX_WARN_S("Deserialize in RRT is unimplemented");
            return false;
        }

        const statistics_t* rrt_t::get_statistics()
        {
            statistics = new rrt_statistics_t();
            statistics->as<rrt_statistics_t > ()->num_vertices = tree.num_vertices();
            statistics->as<rrt_statistics_t > ()->solution_quality = validity_checker->trajectory_cost(input_query->path);
            statistics->time = clock.measure();
            statistics->steps = iteration_count;

            return statistics;
        }


        void rrt_t::prune_trajectory(sim::plan_t& new_plan, sim::trajectory_t& new_trajectory, sim::state_t* new_end_state)
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
