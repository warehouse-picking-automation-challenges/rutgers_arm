/**
 * @file esst.cpp
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

#include "planning/esst.hpp"
#include "prx/planning/motion_planners/sst/sst_statistics.hpp"

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "prx/utilities/statistics/svg_image.hpp"
#include "utilities/particle_goal.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>

PLUGINLIB_EXPORT_CLASS( prx::packages::conformant::esst_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;
    namespace packages
    {
        namespace conformant
        {
            esst_t::esst_t() 
            {
                statistics = new sst_statistics_t();
                drain = true;
                radius_nearest = true;
                count_of_failure = 0;
                img_count=0;
                time_elapsed = 0;
                steering = false;
            }

            esst_t::~esst_t() 
            { 
            }

            void esst_t::init(const parameter_reader_t* reader,const parameter_reader_t* template_reader)
            {
                rrt_t::init( reader, template_reader);
                delta_drain = parameters::get_attribute_as<double>("delta_drain",reader,template_reader,0);
                delta_near = parameters::get_attribute_as<double>("delta_near",reader,template_reader,4);
                max_attempts = parameters::get_attribute_as<int>("max_attempts",reader,template_reader,100);
                probability_threshold = parameters::get_attribute_as<double>("probability_threshold",reader,template_reader,.8);
                drain = (delta_drain>PRX_ZERO_CHECK);
                radius_nearest = (delta_near>PRX_ZERO_CHECK);
                est_selection = parameters::get_attribute_as<bool>("est_selection",reader,template_reader,false);
                sample_metric = parameters::initialize_from_loader<distance_metric_t > ("prx_utilities", reader, "sample_metric" ,template_reader,"sample_metric");

                collision_weight = parameters::get_attribute_as<double>("collision_weight",reader,template_reader,0.0);
                path_cost_weight = parameters::get_attribute_as<double>("path_cost_weight",reader,template_reader,1.0);
                selected_weight = parameters::get_attribute_as<int>("selected_weight",reader,template_reader,1);
                cost_weight = parameters::get_attribute_as<int>("cost_weight",reader,template_reader,1);
                order_weight = parameters::get_attribute_as<int>("order_weight",reader,template_reader,1);
                density_weight = parameters::get_attribute_as<int>("density_weight",reader,template_reader,1);
                particle_tree = parameters::get_attribute_as<bool>("particle_tree",reader,template_reader,false);
                num_particles = parameters::get_attribute_as<int>("num_particles",reader,template_reader,50);
            }

            void esst_t::setup()
            {
                point_number=0;

                max_cost=0;
                char* w = std::getenv("PRACSYS_PATH");
                std::string dir(w);
                std::stringstream s1;
                s1<<"/prx_output/"<<this->path<<"/";
                dir += (s1.str());
                boost::filesystem::path output_dir (dir);
                if (!boost::filesystem::exists(output_dir))
                {
                    boost::filesystem::create_directories( output_dir );
                }


                //create the particle space
                particle_space = new particle_space_t(state_space,num_particles);


                output_directory = dir;
                tree.pre_alloc_memory<esst_node_t,sst_edge_t>(max_points);
                particle_trajectory.link_space(particle_space);
                trajectory.link_space(particle_space);
                states_to_check.clear();

                clock.reset();  
                for(unsigned i=0;i<max_points;i++)
                {
                    pre_alloced_points.push_back(particle_space->alloc_point());
                }
                for(unsigned i=0;i<max_points;i++)
                {
                    radial.push_back(new abstract_node_t());
                }
                // trajectory_t testing_trajectory;
                // testing_trajectory.link_space(state_space);
                // testing_trajectory.read_from_file("/Users/zlittlefield/prx/pracsys/prx_output/published_plans/trajectory.txt");
                // unsigned index1 = 0;
                // unsigned index5 = testing_trajectory.size()-1;
                // unsigned index2 = index5/4; 
                // unsigned index3 = index5/2; 
                // unsigned index4 = index5*3/4; 



                // state_t* orig1 = state_space->clone_point(testing_trajectory[index1]);
                // state_t* mod3 = state_space->clone_point(testing_trajectory[index5]);
                // state_t* mod1 = state_space->clone_point(testing_trajectory[index1]);
                // state_t* mod2 = state_space->clone_point(testing_trajectory[index5]);
                // for(unsigned i=0;i<number_of_states;i++)
                // {
                //     orig1->at(i*state_space->get_dimension())   = 0;
                //     orig1->at(i*state_space->get_dimension()+1) = 0;
                //     mod1->at(i*state_space->get_dimension())    = gaussian_random()*2; 
                //     mod1->at(i*state_space->get_dimension()+1)  = gaussian_random()*2; 
                //     mod2->at(i*state_space->get_dimension())    = gaussian_random()*3; 
                //     mod2->at(i*state_space->get_dimension()+1)  = gaussian_random()*3; 
                //     mod3->at(i*state_space->get_dimension())    = gaussian_random()*4; 
                //     mod3->at(i*state_space->get_dimension()+1)  = gaussian_random()*4; 
                // }
                // PRX_INFO_S("0 to 2: "<<metric->distance_function(orig1,mod1));
                // PRX_INFO_S("0 to 3: "<<metric->distance_function(orig1,mod2));
                // PRX_INFO_S("0 to 4: "<<metric->distance_function(orig1,mod3));
                // PRX_INFO_S("2 to 3: "<<metric->distance_function(mod1,mod2));
                // PRX_INFO_S("2 to 4: "<<metric->distance_function(mod1,mod3));
                // PRX_INFO_S("3 to 4: "<<metric->distance_function(mod2,mod3));

                // PRX_INFO_S("BEGIN TO BEGIN%: "<<metric->distance_function(testing_trajectory[index1],testing_trajectory[index1]));
                // PRX_INFO_S("BEGIN TO 25%: "<<metric->distance_function(testing_trajectory[index1],testing_trajectory[index2]));
                // PRX_INFO_S("BEGIN TO 50%: "<<metric->distance_function(testing_trajectory[index1],testing_trajectory[index3]));
                // PRX_INFO_S("BEGIN TO 75%: "<<metric->distance_function(testing_trajectory[index1],testing_trajectory[index4]));
                // PRX_INFO_S("BEGIN TO END: "<<metric->distance_function(testing_trajectory[index1],testing_trajectory[index5]));
                // PRX_INFO_S("25% TO 25%: "<<metric->distance_function(testing_trajectory[index2],testing_trajectory[index2]));
                // PRX_INFO_S("25% TO 50%: "<<metric->distance_function(testing_trajectory[index2],testing_trajectory[index3]));
                // PRX_INFO_S("25% TO 75%: "<<metric->distance_function(testing_trajectory[index2],testing_trajectory[index4]));
                // PRX_INFO_S("25% TO END: "<<metric->distance_function(testing_trajectory[index2],testing_trajectory[index5]));
                // PRX_INFO_S("50% TO 50%: "<<metric->distance_function(testing_trajectory[index3],testing_trajectory[index3]));
                // PRX_INFO_S("50% TO 75%: "<<metric->distance_function(testing_trajectory[index3],testing_trajectory[index4]));
                // PRX_INFO_S("50% TO END: "<<metric->distance_function(testing_trajectory[index3],testing_trajectory[index5]));
                // PRX_INFO_S("75% TO 75%: "<<metric->distance_function(testing_trajectory[index4],testing_trajectory[index4]));
                // PRX_INFO_S("75% TO END: "<<metric->distance_function(testing_trajectory[index4],testing_trajectory[index5]));
                // PRX_INFO_S("END TO END: "<<metric->distance_function(testing_trajectory[index5],testing_trajectory[index5]));
                // exit(0);
                // std::stringstream s;
                // s<<std::setfill('0') << std::setw(8)<<iteration_count<<".txt";
                // std::string dir2 = output_directory+s.str();
                // std::string names_file = output_directory+"names.txt";
                // std::ofstream fout;
                // fout.open(names_file.c_str());
                // fout<<s.str()<<std::endl;
                // fout.close();
                // fout.open(dir2.c_str());
                // fout<< std::fixed << std::showpoint;
                // fout<<std::setprecision(6);
                // std::vector<double> particle;
                // particle.resize(3);
                // particle[2] = 2;
                // for(unsigned i=0;i<state_space->get_dimension();i+=state_space->get_dimension())
                // {
                //     for(unsigned j=0;j<state_space->get_dimension();++j)
                //     {
                //         particle[j] = testing_trajectory[index1]->at(i*state_space->get_dimension()+j);
                //     }
                //     fout<<std::endl<<"Box "<<particle[0]<<" "<<particle[1]<<" "<<particle[2]<<" ";
                //     fout<<".1 "<<0<<" "<<0<<" "<<0;
                // }
                // for(unsigned i=0;i<state_space->get_dimension();i+=state_space->get_dimension())
                // {
                //     for(unsigned j=0;j<state_space->get_dimension();++j)
                //     {
                //         particle[j] = testing_trajectory[index2]->at(i*state_space->get_dimension()+j);
                //     }
                //     fout<<std::endl<<"Box "<<particle[0]<<" "<<particle[1]<<" "<<particle[2]<<" ";
                //     fout<<".1 "<<.2<<" "<<.2<<" "<<.2;
                // }
                // for(unsigned i=0;i<state_space->get_dimension();i+=state_space->get_dimension())
                // {
                //     for(unsigned j=0;j<state_space->get_dimension();++j)
                //     {
                //         particle[j] = testing_trajectory[index3]->at(i*state_space->get_dimension()+j);
                //     }
                //     fout<<std::endl<<"Box "<<particle[0]<<" "<<particle[1]<<" "<<particle[2]<<" ";
                //     fout<<".1 "<<.4<<" "<<.4<<" "<<.4;
                // }
                // for(unsigned i=0;i<state_space->get_dimension();i+=state_space->get_dimension())
                // {
                //     for(unsigned j=0;j<state_space->get_dimension();++j)
                //     {
                //         particle[j] = testing_trajectory[index4]->at(i*state_space->get_dimension()+j);
                //     }
                //     fout<<std::endl<<"Box "<<particle[0]<<" "<<particle[1]<<" "<<particle[2]<<" ";
                //     fout<<".1 "<<.6<<" "<<.6<<" "<<.6;
                // }
                // for(unsigned i=0;i<state_space->get_dimension();i+=state_space->get_dimension())
                // {
                //     for(unsigned j=0;j<state_space->get_dimension();++j)
                //     {
                //         particle[j] = testing_trajectory[index5]->at(i*state_space->get_dimension()+j);
                //     }
                //     fout<<std::endl<<"Box "<<particle[0]<<" "<<particle[1]<<" "<<particle[2]<<" ";
                //     fout<<".1 "<<.8<<" "<<.8<<" "<<.8;
                // }
                // fout.close();
                // exit(0);

            }

            void esst_t::hacked_resolve()
            {
                    // input_query->clear();
                    // unsigned size;
                    // tree_vertex_index_t best_goal = nearest_vertex(input_query->get_goal()->get_goal_points(size)[0]);
                    // input_query->solution_cost = get_vertex(best_goal)->cost;
            }
            void esst_t::resolve_query()
            {
                if(tree.num_vertices()==0)
                {

                    state_t* state = input_query->get_start_state();
                    PRX_WARN_S ("Start state: " << state_space->print_point(state,3));
                    for(unsigned i=0;i<num_particles;i++)
                    {
                        particle_space->copy_from_particle(state,i);
                    }

                    // reinitialize modules
                    local_planner->link_state_space(particle_space);
                    metric->link_space(particle_space);

                    states_to_check.push_back(particle_space->alloc_point());
                    sample_point = particle_space->alloc_point();
                    start_vertex = tree.add_vertex<esst_node_t,sst_edge_t>();
                    get_vertex(start_vertex)->time_stamp = iteration_count;
                    std::vector<bool> init_collisions;
                    init_collisions.resize(num_particles);
                    for(unsigned i=0;i<num_particles;i++)
                    {
                        init_collisions[i] = false;
                    }
                    get_vertex(start_vertex)->collisions = init_collisions;

                    PRX_INFO_S("IN SETUP");

                    tree[start_vertex]->point = particle_space->alloc_point();
                    PRX_WARN_S ("Start state: " << particle_space->print_point(tree[start_vertex]->point,3));
                    metric->add_point(tree[start_vertex]); 
                    // create the sample set
                    sample_graph.pre_alloc_memory<sample_point_t,sst_edge_t>(max_points);
                    sample_point_t* sample;
                    tree_vertex_index_t v = sample_graph.add_vertex<sample_point_t,sst_edge_t>();
                    sample = sample_graph.get_vertex_as<sample_point_t>(v);
                    sample->point = particle_space->alloc_point();
                    PRX_WARN_S ("Start state: " << particle_space->print_point(sample->point,3));
                    PRX_INFO_S("IN SETUP");
                    sample->set = true;
                    sample->memory = start_vertex;
                    sample_metric->link_space(particle_space);
                    sample_metric->add_point(sample_graph[v]);

                    //things for solution gathering
                    best_goal = start_vertex;
                    real_solution = false;
                    sprint_iteration_counter = 0;
                    sprint_count = 0;
                    last_stat = 0;

                    prob_checker = dynamic_cast<esst_validity_checker_t*>(validity_checker);
                    particle_local_planner = dynamic_cast<esst_local_planner_t*>(local_planner);

                    prob_checker->link_particle_space(particle_space);
                    particle_local_planner->link_particle_space(particle_space);
                    
                    input_query->path.link_space(particle_space);
                }

                rrt_t::resolve_query();
                // solution_number++;
                // input_query->clear();
                // tree_vertex_index_t new_v = best_goal;
                // std::deque<tree_vertex_index_t> path_v;
                // while(get_vertex(new_v)->get_parent() != new_v)
                // {                
                //     path_v.push_front(new_v);
                //     tree_edge_index_t e = tree.edge(get_vertex(new_v)->get_parent(),new_v);
                //     rrt_edge_t* edge = get_edge(e);

                //     foreach(plan_step_t step, edge->plan)
                //     {
                //         input_query->plan.copy_onto_front(step.control,step.duration);
                //     }
                //     new_v = get_vertex(new_v)->get_parent();
                // }
                // local_planner->propagate(tree[start_vertex]->point,input_query->plan,input_query->path);
            }

            void esst_t::step()
            {                   
                if(iteration_count==0)
                {
                    dynamic_cast<particle_goal_t*>(input_query->get_goal())->link_particle_space(particle_space);
                }
                //sample a state
                if( !real_solution && roll_weighted_die({.1,1 - .1}, false) == 0 )
                {
                    for(unsigned i=0;i<num_particles;i++)
                    {
                        particle_space->copy_from_particle(input_query->get_goal()->get_first_goal_point(),i,sample_point);
                    }
                }
                else
                {
                    sampler->sample(particle_space,sample_point);
                }
                // sampler->sample(particle_space,sample_point);

                // //get the nearest state  
                tree_vertex_index_t nearest = nearest_vertex(sample_point);
                //propagate toward the sampled point
                plan_t plan;
                plan.link_control_space(control_space);
                state_t* end_state = pre_alloced_points[point_number];
                double prop_length=0;
                int attempts=0;
                particle_trajectory.clear();
                //small optimization where we try to make sure we leave the drain ball.
                // do
                // {
                    //keep the trajectory if collision checking
                    if(collision_checking)
                    {
                        plan.clear();

                        util::sys_clock_t timer;
                        timer.reset();
                        local_planner->steer(tree[nearest]->point,sample_point,plan,particle_trajectory,steering);
                        prop_times.push_back(timer.measure());
                        PRX_INFO_S(prop_times.back());
                        particle_space->copy_point(end_state,particle_trajectory[particle_trajectory.size()-1]);
                        prop_length = plan.length(); //prob_checker->true_cost(particle_trajectory,plan);         
                    }
                    //just keep the end state
                    else
                    {
                        plan.clear();            
                        local_planner->steer(tree[nearest]->point,sample_point,plan,end_state,steering);            
                        prop_length = plan.length();          
                    }
                    attempts++;
                // }
                // while(drain && attempts < max_attempts && ( metric->distance_function(tree[nearest]->point,end_state) < delta_drain || metric->distance_function(tree[nearest]->point,end_state) == PRX_INFINITY));
                count_of_failure += attempts-1;  
                //**** finding sample node
                //check for better things
                sample_point_t* sample;
                double distance_to_nearest_sample;
                tree_vertex_index_t sample_vertex = ((tree_node_t*)(sample_metric->single_query(end_state,&distance_to_nearest_sample)))->get_index();
                sample = sample_graph.get_vertex_as<sample_point_t>(sample_vertex);




                if(prop_times.size()==25)
                {
                    double mean=0;
                    double var=0;
                    foreach(double val, prop_times)
                    {
                        mean+=val;
                    }
                    mean/=25.0;
                    foreach(double val, prop_times)
                    {
                        var+=(mean-val)*(mean-val);
                    }
                    var/=25.0;
                    PRX_WARN_S(mean<<" "<<sqrt(var));

                }



                /////////////////////////////////////////////////////////////////////////////////////////compute the dispersion of the points
                // double dist1,dist2;
                // {
                //     const particle_point_t* p1p = (const particle_point_t*)tree[nearest]->point;
                //     std::vector<double> point1;
                //     unsigned state_size = particle_space->get_point_space()->get_dimension();
                //     point1.resize(state_size);
                //     for(unsigned long j=0;j<state_size;j++)
                //     {
                //         point1[j] = 0;
                //     }
                //     unsigned number_of_states = particle_space->get_num_particles();
                //     for(unsigned long i=0;i<number_of_states;i++)
                //     {
                //         for(unsigned long j=0;j<state_size;j++)
                //         {
                //             point1[j] += p1p->links[i]->memory[j];
                //         }
                //     }
                //     for(unsigned long j=0;j<state_size;j++)
                //     {
                //         point1[j] /= number_of_states;
                //     }
                //     double temp_dist = 0;
                //     dist1 = 0;
                //     for(unsigned long i=0;i<number_of_states;i++)
                //     {
                //         temp_dist = 0;
                //         for(unsigned long j=0;j<state_size;j++)
                //         {
                //             temp_dist+=(point1[j]-p1p->links[i]->memory[j])*(point1[j]-p1p->links[i]->memory[j]);
                //         }
                //         if(dist1<temp_dist)
                //         {
                //             dist1 = temp_dist;
                //         }
                //     }
                //     dist1 = sqrt(dist1);
                // }
                // {
                //     const particle_point_t* p1p = (const particle_point_t*)end_state;
                //     std::vector<double> point1;
                //     unsigned state_size = particle_space->get_point_space()->get_dimension();
                //     point1.resize(state_size);
                //     for(unsigned long j=0;j<state_size;j++)
                //     {
                //         point1[j] = 0;
                //     }
                //     unsigned number_of_states = particle_space->get_num_particles();
                //     for(unsigned long i=0;i<number_of_states;i++)
                //     {
                //         for(unsigned long j=0;j<state_size;j++)
                //         {
                //             point1[j] += p1p->links[i]->memory[j];
                //         }
                //     }
                //     for(unsigned long j=0;j<state_size;j++)
                //     {
                //         point1[j] /= number_of_states;
                //     }
                //     double temp_dist = 0;
                //     dist2 = 0;
                //     for(unsigned long i=0;i<number_of_states;i++)
                //     {
                //         temp_dist = 0;
                //         for(unsigned long j=0;j<state_size;j++)
                //         {
                //             temp_dist+=(point1[j]-p1p->links[i]->memory[j])*(point1[j]-p1p->links[i]->memory[j]);
                //         }
                //         if(dist2<temp_dist)
                //         {
                //             dist2 = temp_dist;
                //         }
                //     }
                //     dist2 = sqrt(dist2);
                // }
                // if(dist1>dist2)
                // {
                //     //output the first point

                //     append_to_stat_file(boost::lexical_cast<std::string>(dist1)+"-->"+boost::lexical_cast<std::string>(dist2)+"\n");
                //     const particle_point_t* p1p = (const particle_point_t*)tree[nearest]->point;
                //     unsigned state_size = particle_space->get_point_space()->get_dimension();
                //     unsigned number_of_states = particle_space->get_num_particles();
                //     for(unsigned long i=0;i<number_of_states;i++)
                //     {
                //         append_to_stat_file(state_space->serialize_point(p1p->links[i])+"\n");
                //     }
                //     append_to_stat_file("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");
                //     const particle_point_t* p2p = (const particle_point_t*)end_state;
                //     for(unsigned long i=0;i<number_of_states;i++)
                //     {
                //         append_to_stat_file(state_space->serialize_point(p2p->links[i])+"\n");
                //     }
                //     append_to_stat_file("********************************************************************\n");
                //     PRX_INFO_S(dist1<<"-->"<<dist2);
                // }

                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

















                //check if the trajectory is valid
                double probability_of_validity;


                prob_checker->set_previous_collisions(get_vertex(nearest)->collisions);
                // if(nearest!=start_vertex)
                // {
                //     sst_edge_t* previous_edge = get_edge(get_vertex(nearest)->get_parent_edge());
                //     probability_of_validity = previous_edge->prob*prob_checker->probability_of_validity(trajectory);//(prob_checker->is_valid(trajectory)
                // }
                // else
                // {
                    probability_of_validity = prob_checker->probability_of_validity(particle_trajectory);//(prob_checker->is_valid(trajectory)
                // }
                double new_cost = get_vertex(nearest)->cost + prop_length*path_cost_weight + (1.0-probability_of_validity)*collision_weight;

                if(!collision_checking || (probability_of_validity >= probability_threshold && particle_trajectory.size()>1))
                {  
                    // PRX_WARN_S(distance_to_nearest_sample<<"  > "<<delta_drain);
                    //if too far from a witness sample
                    if(distance_to_nearest_sample > delta_drain)
                    {
                        //add into list of graph points
                        tree_vertex_index_t v = sample_graph.add_vertex<sample_point_t,sst_edge_t>();
                        sample = sample_graph.get_vertex_as<sample_point_t>(v);
                        sample->point = particle_space->clone_point(end_state);
                        sample_metric->add_point(sample_graph[v]);
                    }
                    //either we made a new witness sample, or this representative is better for the existing witness
                    if((!(sample->set)) || get_vertex(sample->memory)->cost > new_cost )
                    {    
                        // if(nearest!=start_vertex)
                        // {
                        //     std::cout<<std::setprecision(std::numeric_limits<double>::digits10)<<"Nearest to sample:    "<<metric->distance_function(get_vertex(nearest)->point,sample_point)<<std::endl;
                        //     std::cout<<std::setprecision(std::numeric_limits<double>::digits10)<<"Start to sample:      "<<metric->distance_function(get_vertex(start_vertex)->point,sample_point)<<std::endl;
                        //     std::cout<<std::setprecision(std::numeric_limits<double>::digits10)<<"Nearest to new state: "<<metric->distance_function(get_vertex(nearest)->point,end_state)<<std::endl;
                        //     exit(0);
                        // }
                        // PRX_INFO_S(5);
                        tree_vertex_index_t v = tree.add_vertex<esst_node_t,sst_edge_t>();
                        esst_node_t* node = get_vertex(v);
                        node->point = end_state;
                        node->time_stamp = point_number;
                        node->collisions = prob_checker->get_new_collisions();

                        point_number++;
                        node->bridge = true;
                        node->cost = new_cost;   
                        tree_edge_index_t e = tree.add_edge<sst_edge_t>(nearest,v);
                        get_edge(e)->plan = plan;          
                        get_edge(e)->prob = probability_of_validity;    
                        particle_space->copy_point(states_to_check[0],end_state);
                        //check if better goal node
                        // double accepted;
                        if(!real_solution && input_query->get_goal()->satisfied(end_state))
                        {
                            best_goal = v;
                            satisfied_goal_vertex.push_back(v);
                            PRX_WARN_S("First Solution: "<<get_vertex(best_goal)->cost);
                            real_solution = true;
                            // max_cost = get_vertex(best_goal)->cost;
                        }
                        else if(real_solution && input_query->get_goal()->satisfied(end_state) && 
                            get_vertex(best_goal)->cost >//+ prob_checker->heuristic( get_vertex(best_goal)->point,input_query->get_goal()->get_goal_points()[0]) > //
                            get_vertex(v)->cost //+ prob_checker->heuristic( end_state,input_query->get_goal()->get_goal_points()[0])
                                 )
                        {
                            PRX_WARN_S("Next Solution: "<<get_vertex(v)->cost);
                            best_goal = v;
                            satisfied_goal_vertex.push_back(v);
                        }
                        else if (real_solution && input_query->get_goal()->satisfied(end_state))
                        {
                            PRX_WARN_S("Candidate solution was worse: "<<get_vertex(v)->cost);
                        }
                        //check if we need to store trajectories for visualization
                        if(collision_checking && visualize_tree)
                        {
                            get_edge(e)->trajectory = particle_trajectory;
                        }
                        else if(!collision_checking && visualize_tree)
                        {
                            if(particle_trajectory.size()==0)
                                local_planner->propagate(get_vertex(nearest)->point,plan,particle_trajectory);
                            get_edge(e)->trajectory = particle_trajectory;
                        }   

                        // output_edge(e);

                        //optimization for sparsity
                        if(sample->set)
                        {
                            int old_density = get_vertex(sample->memory)->density;
                            if(!get_vertex(sample->memory)->bridge )
                            {
                                metric->remove_point(tree[sample->memory]);
                                get_vertex(sample->memory)->bridge = true;
                            }
                            tree_vertex_index_t iter = sample->memory;
                            while( is_leaf(iter) && get_vertex(iter)->bridge && !is_best_goal(iter))
                            {
                                tree_vertex_index_t next = get_vertex(iter)->get_parent();
                                remove_leaf(iter);
                                iter = next;
                            } 
                            get_vertex(v)->density = old_density;
                        }
                        else
                        {
                            if(density_weight>0.5)
                            {
                                unsigned val = metric->radius_query(get_vertex(v)->point,delta_near, radial);
                                // get_vertex(v)->density = val;
                                std::vector<const abstract_node_t*>::const_iterator iter,iter_end;
                                iter = radial.begin();
                                iter_end = iter;
                                std::advance(iter_end,val);
                                for(iter = radial.begin();iter!=iter_end;iter++)
                                {
                                    tree_vertex_index_t v = ((const tree_node_t*)(*iter))->get_index();
                                    get_vertex(v)->density++ ;
                                }
                            }
                        }
                        sample->set = true;
                        sample->memory = v;

                        //add new node to metric
                        get_vertex(v)->bridge = false;
                        metric->add_point(tree[v]);
                    }
                }  
                iteration_count++;
                PRX_STATUS(iteration_count,PRX_TEXT_BROWN);
                if(point_number == max_points)
                {
                    for(unsigned i=0;i<max_points;i++)
                    {
                        pre_alloced_points.push_back(particle_space->alloc_point());
                        radial.push_back(new abstract_node_t());

                        PRX_INFO_S("sAVING TRAJ");
                    }
                    max_points*=2;
                }
            }
            void esst_t::link_query(query_t* new_query)
            {
                motion_planner_t::link_query(new_query);
                // state_t* state = input_query->get_goal()->get_goal_points()[0];
                // for(unsigned i=state_space->get_dimension();i<state_space->get_dimension();i++)
                //     state->at(i) = state->at(i%state_space->get_dimension());

                // // PRX_INFO_S(metric->distance_function(tree[start_vertex]->point,tree[start_vertex]->point));
                // // PRX_INFO_S(metric->distance_function(state,state));
                // // PRX_INFO_S(metric->distance_function(tree[start_vertex]->point,state));
                // // PRX_INFO_S(metric->distance_function(state,tree[start_vertex]->point));
                // // exit(0);
                // sample_point_t* sample;
                // tree_vertex_index_t v = sample_graph.add_vertex<sample_point_t,sst_edge_t>();
                // sample = sample_graph.get_vertex_as<sample_point_t>(v);
                // sample->point = state_space->clone_point(input_query->get_goal()->get_goal_points()[0]);
                // PRX_WARN_S ("End state: " << state_space->print_point(sample->point,3));
                // sample->set = false;
                // sample_metric->link_space(state_space);
                // sample_metric->add_point(sample_graph[v]);


            }

            void esst_t::remove_subtree(util::tree_vertex_index_t v)
            {
                // bool new_goal=(v==best_goal);
                // if(get_vertex(v)==NULL)
                //     return;
                // if(get_vertex(v)->get_children().size()!=0)
                // {
                //     std::vector<tree_vertex_index_t> vs;
                //     for(std::list<tree_vertex_index_t>::const_iterator i=get_vertex(v)->get_children().begin();
                //         i!=get_vertex(v)->get_children().end();
                //         i++)
                //     {
                //         vs.push_back(*i);
                //     }
                //     foreach(tree_vertex_index_t child, vs)
                //     {
                //         remove_subtree(child);
                //     }
                // }
                // remove_leaf(v);
                // if(new_goal)
                // {
                //     real_solution = false;
                //     best_goal = start_vertex;
                //     foreach(tree_node_t* node, tree.vertices())
                //     {
                //         tree_vertex_index_t iter = node->get_index();
                //         if(!real_solution && input_query->get_goal()->satisfied(get_vertex(iter)->point))
                //         {
                //             best_goal = iter;
                //             real_solution = true;
                //         }
                //         else if(real_solution && get_vertex(best_goal)->cost > get_vertex(iter)->cost && input_query->get_goal()->satisfied(get_vertex(iter)->point) )
                //         {
                //             best_goal = iter;
                //         }
                //     }
                // }
            }

            bool esst_t::is_best_goal(tree_vertex_index_t v)
            {
                tree_vertex_index_t new_v = best_goal;

                while(get_vertex(new_v)->get_parent()!=new_v)
                {
                    if(new_v == v)
                        return true;

                    new_v = get_vertex(new_v)->get_parent();
                }
                return false;

            }


            bool esst_t::is_leaf(tree_vertex_index_t v)
            {
                return (get_vertex(v)->get_children().size() == 0);
            }

            void esst_t::remove_leaf(tree_vertex_index_t v)
            {
                if(get_vertex(v)->get_parent()!=v)
                {
                    rrt_edge_t* edge = get_edge(tree.edge(get_vertex(v)->get_parent(),v));  
                    edge->plan.clear();
                }
                if(!get_vertex(v)->bridge)
                {
                    metric->remove_point(tree[v]);
                }
                tree.remove_vertex(v);
            }    

            inline esst_node_t* esst_t::get_vertex(tree_vertex_index_t v) const
            {
                return tree.get_vertex_as<esst_node_t>(v);
            }
            inline sst_edge_t* esst_t::get_edge(tree_edge_index_t e) const
            {
                return tree.get_edge_as<sst_edge_t>(e);
            }

            tree_vertex_index_t esst_t::nearest_vertex(state_t* state)
            {    
                if(est_selection)
                {

                    // PRX_INFO_S("EST SELECTION");
                    double total_weight = 0;
                    double max_selection = 0;
                    double max_order = 0;
                    double max_cost = 0;
                    double max_density = 0;
                    tree_vertex_index_t selected_node = start_vertex;
                    std::vector<double> ind_weights;
                    foreach(tree_node_t* v, tree.vertices())
                    {
                        esst_node_t* node = dynamic_cast<esst_node_t*>(v);
                        if(!node->bridge)
                        {
                            max_selection = PRX_MAXIMUM(max_selection,node->selected_count);
                            max_order = PRX_MAXIMUM(max_selection,node->time_stamp);
                            max_cost = PRX_MAXIMUM(max_selection,node->cost);
                            max_density = PRX_MAXIMUM(max_density,node->density);
                        }
                    }
                    foreach(tree_node_t* v, tree.vertices())
                    {
                        esst_node_t* node = dynamic_cast<esst_node_t*>(v);
                        if(!node->bridge)
                        {
                            ind_weights.push_back(pow(node->time_stamp/max_order,order_weight)
                                            /(pow(node->selected_count/max_selection,selected_weight)*
                                                pow(node->density/max_density,density_weight)*
                                                    pow((node->cost+.0001)/max_cost,cost_weight)));
                            total_weight+=ind_weights.back();
                        }
                    }
                    double random_number = uniform_random()*total_weight;
                    unsigned index = 0;
                    foreach(tree_node_t* v, tree.vertices())
                    {
                        esst_node_t* node = dynamic_cast<esst_node_t*>(v);
                        if(!node->bridge)
                        {
                            random_number-=ind_weights[index];
                            if(random_number <= 0.0)
                            {
                                node->selected_count++;
                                selected_node = node->get_index();
                                break;
                            }
                            index++;

                        }
                    }
                    return selected_node;
                }
                if(radius_nearest)
                {
                    unsigned val = metric->radius_and_closest_query(state,delta_near, radial);
                    if(val==0)
                    {
                        return ((const tree_node_t*)metric->single_query(state))->get_index();
                    }
                    else
                    {
                        tree_vertex_index_t ret_val;
                        double length = PRX_INFINITY;
                        std::vector<const abstract_node_t*>::const_iterator iter,iter_end;
                        iter = radial.begin();
                        iter_end = iter;
                        std::advance(iter_end,val);
                        for(iter = radial.begin();iter!=iter_end;iter++)
                        {
                            tree_vertex_index_t v = ((const tree_node_t*)(*iter))->get_index();
                            double temp = get_vertex(v)->cost ;
                            if( temp < length)
                            {
                                length = temp;
                                ret_val = v;
                            }
                        }
                        return ret_val;
                    }
                }
                else
                {
                    return ((const tree_node_t*)metric->single_query(state))->get_index();
                }
            }

            using namespace svg;
            void esst_t::update_vis_info() const
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
                        for(unsigned j=0;j<(num_particles>5?5:num_particles);j++)
                        {
                            std::string name = visualization_tree_name + "/edge_" + int_to_str(count);
                            params.clear();

                            foreach(state_t* state, get_edge(e->get_index())->trajectory)
                            {
                                map_params.clear();

                                ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->compute_configs(particle_space->get_particle(j,state), system_names, map_params);
                                params.insert(params.end(), map_params[system_names[0]].begin(), map_params[system_names[0]].end());

                            }
                            geoms.push_back(geometry_info_t(visualization_body, name, PRX_LINESTRIP, params, graph_color));
                            configs.push_back(config_t());

                            count++;
                        }
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
                        ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->compute_configs(state, system_names, map_params);
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

                    ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->visualization_geom_map[visualization_tree_name] = geoms;
                    ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->visualization_configs_map[visualization_tree_name] = configs;

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
                        ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->visualization_geom_map[visualization_solution_name] = geoms;
                        ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->visualization_configs_map[visualization_solution_name] = configs;
                        geoms.clear();
                        configs.clear();
                    }
                    else
                    {
                        params.clear();
                        for(unsigned j=0;j<(num_particles>5?5:num_particles);j++)
                        {
                            for( size_t i = 0; i < input_query->path.size(); i++ )
                            {
                                map_params.clear();
                                //HACK HACK HACK
                                // params.push_back(input_query->path[i]->at(0));
                                // params.push_back(input_query->path[i]->at(1));
                                // params.push_back(input_query->path[i]->at(2));
                                ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->compute_configs(particle_space->get_particle(j,input_query->path[i]), system_names, map_params);
                                // map_params[visualization_body][0]+=0.005;
                                params.insert(params.end(), map_params[visualization_body].begin(), map_params[visualization_body].end());
                                //params.back() += 3; //this is so the solution will be above
                            }
                        }

                        std::string name = visualization_solution_name + "/" + visualization_body + "/path";//_" + int_to_str(solution_number);
                        geoms.push_back(geometry_info_t(visualization_body, name, PRX_LINESTRIP, params, color_map[solution_number % color_map.size()]));
                        configs.push_back(config_t());
                        ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->visualization_geom_map[visualization_solution_name] = geoms;
                        ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->visualization_configs_map[visualization_solution_name] = configs;
                        geoms.clear();
                        configs.clear();
                    }

                }
            }



            void esst_t::output_edge(tree_edge_index_t e)
            {
                double max_cost_val = max_cost;
                if(max_cost_val<0.00001)
                {
                    foreach(tree_node_t* v, tree.vertices())
                    {
                        double temp = get_vertex(v->get_index())->cost;
                        if(temp > max_cost_val)
                            max_cost_val = temp;    
                    }
                    if(real_solution)
                        max_cost = max_cost_val;
                }
                std::stringstream s;
                s<<std::setfill('0') << std::setw(8)<<iteration_count;
                // img_count++;
                hash_t<std::string, std::vector<double> > map_params;
                std::string dir = output_directory+s.str()+"_edge.txt";
                std::string names_file = output_directory+"edge_names.txt";
                std::ofstream fout;
                fout.open(names_file.c_str(),std::ios_base::app);
                fout<<s.str()<<"_edge.txt"<<std::endl;
                fout.close();
                std::ofstream point_out;
                std::string point_name = output_directory+s.str()+"_points.txt";
                point_out.open(point_name.c_str());
                fout.open(dir.c_str());
                fout<< std::fixed << std::showpoint;
                fout<<std::setprecision(6);
                std::vector<double> line_params;
                std::vector<std::string> system_names;
                system_names.push_back(visualization_body);
                sst_edge_t* edge = dynamic_cast<sst_edge_t*>(get_edge(e));
                tree_node_t* v = tree[edge->get_source()];
                tree_node_t* v2 = tree[edge->get_target()];
                // if (!v->bridge && !v2->bridge)
                {
                    map_params.clear();

                    std::vector<double> first;
                    first.resize(state_space->get_dimension());
                    std::vector<double> second;
                    second.resize(state_space->get_dimension());
                    for(unsigned i=0;i<state_space->get_dimension();++i)
                    {
                        first[i]=second[i] = 0;
                    }
                    state_t* state = v->point;
                    state_t* state2 = v2->point;
                    double color_scale = v2->as<esst_node_t>()->cost/max_cost_val;
                    for(unsigned i=0;i<particle_space->get_num_particles();i++)
                    {
                        state_t* p_s = particle_space->get_particle(i,state);
                        state_t* p_s2 = particle_space->get_particle(i,state2);
                        point_out<<state_space->serialize_point(p_s)<<std::endl;
                        for(unsigned j=0;j<state_space->get_dimension();++j)
                        {
                            first[j]+=p_s->at(j);
                            second[j]+=p_s2->at(j);
                            // sample_point->at(index_offset+j) = state2->at(i+j);
                        }
                        // map_params.clear();
                        // ((::prx::plan::comm::visualization_comm_t*)::prx::plan::comm::vis_comm)->compute_configs(sample_point, system_names, map_params);
                        // fout<<std::endl<<"Box "<<map_params[system_names[0]][0]<<" "<<map_params[system_names[0]][1]<<" "<<map_params[system_names[0]][2]<<" ";
                        // fout<<".1 "<<color_scale<<" "<<color_scale<<" "<<color_scale;
                    }

                    point_out.close();
                    edge->plan.save_to_file(output_directory+s.str()+"_plan.txt");

                    for(unsigned j=0;j<state_space->get_dimension();++j)
                    {
                        first[j]/=particle_space->get_num_particles();
                        second[j]/=particle_space->get_num_particles();
                    }

                    double line_width = 1;
                    if(is_best_goal(edge->get_target()))
                    {
                        line_width = 4;
                    }

                    line_params.clear();

                    line_params.push_back(first[0]);
                    line_params.push_back(first[1]);
                    line_params.push_back(first[2]);
                    line_params.push_back(second[0]);
                    line_params.push_back(second[1]);
                    line_params.push_back(second[2]);

                    // for(unsigned j=0;j<state_space->get_dimension();++j)
                    // {
                    //     sample_point->at(index_offset+j) = first[j];
                    // }
                    // map_params.clear();
                    // ((::prx::plan::comm::visualization_comm_t*)::prx::plan::comm::vis_comm)->compute_configs(sample_point, system_names, map_params);
                    // line_params.insert(line_params.end(), map_params[system_names[0]].begin(), map_params[system_names[0]].end());
                    // for(unsigned j=0;j<state_space->get_dimension();++j)
                    // {
                    //     sample_point->at(index_offset+j) = second[j];
                    // }
                    // map_params.clear();
                    // ((::prx::plan::comm::visualization_comm_t*)::prx::plan::comm::vis_comm)->compute_configs(sample_point, system_names, map_params);
                    // line_params.insert(line_params.end(), map_params[system_names[0]].begin(), map_params[system_names[0]].end());

                    fout<<std::endl<<"Line "<<line_params[0]<<" "<<line_params[1]<<" "<<line_params[2]<<" "<<line_params[3]<<" "<<line_params[4]<<" "<<line_params[5]<<" ";
                    fout<<line_width<<" "<<(1.0 - edge->prob)<<" "<<0<<" "<<edge->prob;

                    //fidushifuhsdiufhsiufhsdiufhisudhfisd
                    for(unsigned i=0;i<particle_space->get_num_particles();i++)
                    {
                        state_t* p_s = particle_space->get_particle(i,state);
                        // map_params.clear();
                        // ((::prx::plan::comm::visualization_comm_t*)::prx::plan::comm::vis_comm)->compute_configs(p_s, system_names, map_params);
                        // fout<<std::endl<<"Box "<<map_params[system_names[0]][0]<<" "<<map_params[system_names[0]][1]<<" "<<map_params[system_names[0]][2]<<" ";
                        fout<<std::endl<<"Box "<<p_s->at(0)<<" "<<p_s->at(1)<<" "<<p_s->at(2)<<" ";
                        fout<<".3 "<<color_scale<<" "<<0<<" "<<0;
                    }
                    for(unsigned i=0;i<particle_space->get_num_particles();i++)
                    {
                        state_t* p_s = particle_space->get_particle(i,state2);
                        // map_params.clear();
                        // ((::prx::plan::comm::visualization_comm_t*)::prx::plan::comm::vis_comm)->compute_configs(p_s, system_names, map_params);
                        // fout<<std::endl<<"Box "<<map_params[system_names[0]][0]<<" "<<map_params[system_names[0]][1]<<" "<<map_params[system_names[0]][2]<<" ";
                        fout<<std::endl<<"Box "<<p_s->at(0)<<" "<<p_s->at(1)<<" "<<p_s->at(2)<<" ";
                        fout<<".3 "<<0<<" "<<color_scale<<" "<<0;
                    }
                }
                fout.close();

            }


            const statistics_t* esst_t::get_statistics()
            {      
                time_elapsed+=clock.measure();

                double avg_cost=0;
                int count=0;
                double avg_time_stamp=0;
                double max_time=0;
                foreach(tree_node_t* v, tree.vertices())
                {
                    if (!v->as<esst_node_t>()->bridge)
                    {
                        avg_cost += get_vertex(v->get_index())->cost;
                        count++;
                        if(v->get_index()!=start_vertex)
                        {
                            double temp = iteration_count - get_vertex(v->get_index())->time_stamp;
                            avg_time_stamp+= temp;
                            if(temp > max_time)
                                max_time = temp;
                        }
                    }
                }
                
                avg_cost/=count;
                avg_time_stamp/=count;

                if(particle_tree)
                {
                    double max_cost_val = max_cost;
                    if(max_cost_val<0.00001)
                    {
                        foreach(tree_node_t* v, tree.vertices())
                        {
                            double temp = get_vertex(v->get_index())->cost;
                            if(temp > max_cost_val)
                                max_cost_val = temp;    
                        }
                        if(real_solution)
                            max_cost = max_cost_val;
                    }
                    std::stringstream s;
                    s<<std::setfill('0') << std::setw(8)<<iteration_count<<".txt";
                    // img_count++;
                    hash_t<std::string, std::vector<double> > map_params;
                    std::string dir = output_directory+s.str();
                    std::string names_file = output_directory+"names.txt";
                    std::ofstream fout;
                    fout.open(names_file.c_str(),std::ios_base::app);
                    fout<<s.str()<<std::endl;
                    fout.close();
                    fout.open(dir.c_str());
                    fout<< std::fixed << std::showpoint;
                    fout<<std::setprecision(6);
                    std::vector<double> line_params;
                    std::vector<std::string> system_names;
                    system_names.push_back(visualization_body);
                    foreach(tree_edge_t* e, tree.edges())
                    {
                        sst_edge_t* edge = dynamic_cast<sst_edge_t*>(e);
                        tree_node_t* v = tree[e->get_source()];
                        tree_node_t* v2 = tree[e->get_target()];
                        // if (!v->bridge && !v2->bridge)
                        {
                            count++;
                            map_params.clear();

                            std::vector<double> first;
                            first.resize(state_space->get_dimension());
                            std::vector<double> second;
                            second.resize(state_space->get_dimension());
                            for(unsigned i=0;i<state_space->get_dimension();++i)
                            {
                                first[i]=second[i] = 0;
                            }
                            state_t* state = v->point;
                            state_t* state2 = v2->point;
                            double color_scale = v2->as<esst_node_t>()->cost/max_cost_val;
                            for(unsigned i=0;i<particle_space->get_num_particles();i++)
                            {
                                state_t* p_s = particle_space->get_particle(i,state);
                                state_t* p_s2 = particle_space->get_particle(i,state2);
                                for(unsigned j=0;j<state_space->get_dimension();++j)
                                {
                                    first[j]+=p_s->at(j);
                                    second[j]+=p_s2->at(j);
                                    // sample_point->at(index_offset+j) = state2->at(i+j);
                                }
                                // map_params.clear();
                                // ((::prx::plan::comm::visualization_comm_t*)::prx::plan::comm::vis_comm)->compute_configs(sample_point, system_names, map_params);
                                // fout<<std::endl<<"Box "<<map_params[system_names[0]][0]<<" "<<map_params[system_names[0]][1]<<" "<<map_params[system_names[0]][2]<<" ";
                                // fout<<".1 "<<color_scale<<" "<<color_scale<<" "<<color_scale;
                            }
                            for(unsigned j=0;j<state_space->get_dimension();++j)
                            {
                                first[j]/=particle_space->get_num_particles();
                                second[j]/=particle_space->get_num_particles();
                            }

                            double line_width = 1;
                            if(is_best_goal(e->get_target()))
                            {
                                line_width = 4;
                            }

                            line_params.clear();

                            line_params.push_back(first[0]);
                            line_params.push_back(first[1]);
                            line_params.push_back(first[2]);
                            line_params.push_back(second[0]);
                            line_params.push_back(second[1]);
                            line_params.push_back(second[2]);

                            // for(unsigned j=0;j<state_space->get_dimension();++j)
                            // {
                            //     sample_point->at(index_offset+j) = first[j];
                            // }
                            // map_params.clear();
                            // ((::prx::plan::comm::visualization_comm_t*)::prx::plan::comm::vis_comm)->compute_configs(sample_point, system_names, map_params);
                            // line_params.insert(line_params.end(), map_params[system_names[0]].begin(), map_params[system_names[0]].end());
                            // for(unsigned j=0;j<state_space->get_dimension();++j)
                            // {
                            //     sample_point->at(index_offset+j) = second[j];
                            // }
                            // map_params.clear();
                            // ((::prx::plan::comm::visualization_comm_t*)::prx::plan::comm::vis_comm)->compute_configs(sample_point, system_names, map_params);
                            // line_params.insert(line_params.end(), map_params[system_names[0]].begin(), map_params[system_names[0]].end());

                            fout<<std::endl<<"Line "<<line_params[0]<<" "<<line_params[1]<<" "<<line_params[2]<<" "<<line_params[3]<<" "<<line_params[4]<<" "<<line_params[5]<<" ";
                            fout<<line_width<<" "<<(1.0 - edge->prob)<<" "<<0<<" "<<edge->prob;

                            // //fidushifuhsdiufhsiufhsdiufhisudhfisd
                            // for(unsigned i=0;i<particle_space->get_num_particles();i++)
                            // {
                            //     state_t* p_s = particle_space->get_particle(i,state);
                            //     // map_params.clear();
                            //     // ((::prx::plan::comm::visualization_comm_t*)::prx::plan::comm::vis_comm)->compute_configs(p_s, system_names, map_params);
                            //     // fout<<std::endl<<"Box "<<map_params[system_names[0]][0]<<" "<<map_params[system_names[0]][1]<<" "<<map_params[system_names[0]][2]<<" ";
                            //     fout<<std::endl<<"Box "<<p_s->at(0)<<" "<<p_s->at(1)<<" "<<p_s->at(2)<<" ";
                            //     fout<<".1 "<<color_scale<<" "<<color_scale<<" "<<color_scale;
                            // }

                            fout<<std::endl<<"Box "<<line_params[0]<<" "<<line_params[1]<<" "<<line_params[2]<<" ";
                            fout<<".3 "<<color_scale<<" "<<0<<" "<<0;

                            fout<<std::endl<<"Box "<<line_params[3]<<" "<<line_params[4]<<" "<<line_params[5]<<" ";
                            fout<<".3 "<<0<<" "<<color_scale<<" "<<0;
                        }
                    }
                    fout.close();
                }

                statistics = new sst_statistics_t();
                statistics->as<sst_statistics_t>()->failure_count = count_of_failure;
                statistics->as<sst_statistics_t>()->num_vertices = tree.num_vertices();
                statistics->as<sst_statistics_t>()->solution_quality = get_vertex(best_goal)->cost;
                statistics->as<sst_statistics_t>()->average_cost = avg_cost;
                statistics->as<sst_statistics_t>()->best_near = delta_near;
                statistics->as<sst_statistics_t>()->drain = delta_drain;
                statistics->as<sst_statistics_t>()->avg_lifespan = avg_time_stamp;


                esst_node_t* start_node = get_vertex(start_vertex);
                double avg=0;
                int child_count=0;
                util::sys_clock_t distance_test;

                distance_test.reset();
                foreach(tree_vertex_index_t v, get_vertex(start_vertex)->get_children())
                {
                    if(!get_vertex(start_vertex)->bridge)
                    {
                        avg+=metric->distance_function(start_node->point,get_vertex(v)->point);
                        child_count++;
                    }
                }
                double high_dist = 0;
                foreach(tree_node_t* v, tree.vertices())
                {
                    if(!get_vertex(v->get_index())->bridge)
                    {
                        if(high_dist < metric->distance_function(start_node->point,get_vertex(v->get_index())->point))
                        {
                            high_dist = metric->distance_function(start_node->point,get_vertex(v->get_index())->point);
                        }
                    }
                }
                double average_distance_function = distance_test.measure()/child_count;

                avg/=child_count;
                statistics->as<sst_statistics_t>()->children_of_root = child_count;
                statistics->as<sst_statistics_t>()->avg_distance_from_root = high_dist;
                statistics->time = time_elapsed;
                statistics->steps = iteration_count;
                count=0;

                std::stringstream out(std::stringstream::out);

                out<<"Stats from SST: \n-- Time Elapsed: "<<time_elapsed
                        <<" s \n-- Iteration Count: "<<iteration_count
                        <<" \n-- Number of Vertices: "<<tree.num_vertices()
                        <<" \n-- Solution Length: "<<get_vertex(best_goal)->cost
                        <<" \n-- Average Node Cost: "<<avg_cost
                        <<" \n-- Delta_near: "<<delta_near
                        <<" \n-- Delta_drain: "<<delta_drain
                        <<" \n-- Average Lifespan (iterations): "<<avg_time_stamp
                        <<" \n-- Children of Root: "<<get_vertex(start_vertex)->get_children().size()
                        <<" \n-- Average Valence: "<<avg
                        <<" \n-- Distance from Goal: "<<high_dist
                        <<" \n-- Average Distance Call Time: "<<average_distance_function;
                PRX_INFO_S(out.str());
                clock.reset();
                last_stat = iteration_count;
                // foreach(tree_node_t* node, tree.vertices())
                // {
                //     tree_vertex_index_t v = node->get_index();
                //     if(get_vertex(v)->get_children().size()>0)
                //     {
                //         std::cout<<std::setprecision(std::numeric_limits<double>::digits10)<<metric->distance_function(get_vertex(start_vertex)->point,get_vertex(v)->point)<<std::endl;
                //         PRX_WARN_S( get_vertex(v)->point->at(0)<<" "<< get_vertex(v)->point->at(1)<<" "<< get_vertex(v)->point->at(2)<<" "<< get_vertex(v)->point->at(3)<<" "<< get_vertex(v)->point->at(4));
                //     }
                //     else
                //     {
                //         std::cout<<std::setprecision(std::numeric_limits<double>::digits10)<<metric->distance_function(get_vertex(start_vertex)->point,get_vertex(v)->point)<<std::endl;
                //     }
                // }
                return statistics;
            }
        }
    }
}

/** 

                double max_cost=0;
                foreach(tree_node_t* v, tree.vertices())
                {
                    double temp = get_vertex(v->get_index())->cost;
                    if(temp > max_cost)
                        max_cost = temp;    
                }

                {
                    std::stringstream s;
                    s<<std::setfill('0') << std::setw(8)<<iteration_count<<".svg";
                    // img_count++;
                    std::string dir = output_directory+s.str();
                    Dimensions dimensions(1000, 1000);
                    Document doc(dir, Layout(dimensions, Layout::BottomLeft));

                    // doc<<Rectangle(Point(0,  0 ),1000, 1000, Color::Yellow);
                    foreach(tree_edge_t* e, tree.edges())
                    {
                        sst_edge_t* edge = dynamic_cast<sst_edge_t*>(e);
                        tree_node_t* v = tree[e->get_source()];
                        tree_node_t* v2 = tree[e->get_target()];
                        // if (!v->bridge && !v2->bridge)
                        {
                            double x,y;
                            x = y = 0;
                            double x1,y1;
                            x1 = y1 = 0;
                            state_t* state = v->point;
                            state_t* state2 = v2->point;
                            int color_scale = v2->as<esst_node_t>()->cost/max_cost * 255;
                            for(unsigned i=0;i<state_space->get_dimension();i+=2)
                            {
                                x+=state2->at(i);
                                y+=state2->at(i+1);
                                x1+=state->at(i);
                                y1+=state->at(i+1);
                                doc << Circle(Point(state2->at(i)*50+500,state2->at(i+1)*50+500),3,Color( color_scale, color_scale, color_scale ));
                            }
                            x/=(state_space->get_dimension()/2);
                            y/=(state_space->get_dimension()/2);
                            x1/=(state_space->get_dimension()/2);
                            y1/=(state_space->get_dimension()/2);
                            if(is_best_goal(e->get_target()))
                            {
                                doc << (Polyline(Stroke(4,Color( (1.0 - edge->prob)*255, 0 , edge->prob*255 ))) 
                                                                         << Point(x1*50+500,y1*50+500)
                                                                         << Point(x*50+500,y*50+500));
                            }
                            else
                            {
                                doc << (Polyline(Stroke(1,Color( (1.0 - edge->prob)*255, 0 , edge->prob*255 ))) 
                                                                         << Point(x1*50+500,y1*50+500)
                                                                         << Point(x*50+500,y*50+500));

                            }

                        }
                    }
                    doc<<Rectangle(Point(0*50+500-100,  8*50+500 + 100 ),4*50, 4*50, Color::Red);
                    doc<<Rectangle(Point(0*50+500-100, -9*50+500 + 50  ),4*50, 2*50, Color::Red);
                    doc<<Rectangle(Point(0*50+500-100,  0*50+500 + 200 ),4*50, 8*50, Color::Red);

                    // doc << Rectangle(Point(3*50+500-100, 4*50+500+275), 4*50, 11*50, Color::Red);
                    // doc << Rectangle(Point(-4.5*50+500-175,5*50+500+37.5), 7*50, 1.5*50, Color::Red);
                    // doc << Rectangle(Point(7*50+500-100,4*50+500+25), 4*50, 1*50, Color::Red);
                    // doc << Rectangle(Point(-6*50+500-125,-5*50+500+125), 5*50, 5*50, Color::Red);
                    // doc << Rectangle(Point(7*50+500-100,-6.5*50+500+100), 4*50, 4*50, Color::Red);

                    // foreach(state_t* state, input_query->path)
                    // for(unsigned i=1;i<input_query->path.size();i++)
                    // {
                    //     Polyline solution(Stroke(3, Color::Green));
                    //     state_t* state = input_query->path[i-1];
                    //     state_t* state2 = input_query->path[i];
                    //     double x,y;
                    //     x = y = 0;
                    //     for(unsigned i=0;i<state_space->get_dimension();i+=2)
                    //     {
                    //         x+=state->at(i);
                    //         y+=state->at(i+1);
                    //     }
                    //     x/=(state_space->get_dimension()/2);
                    //     y/=(state_space->get_dimension()/2);
                    //     solution<<Point(x*50+500,y*50+500);


                    //     x = y = 0;
                    //     for(unsigned i=0;i<state_space->get_dimension();i+=2)
                    //     {
                    //         x+=state2->at(i);
                    //         y+=state2->at(i+1);
                    //     }
                    //     x/=(state_space->get_dimension()/2);
                    //     y/=(state_space->get_dimension()/2);
                    //     solution<<Point(x*50+500,y*50+500);

                    //     doc<<solution;
                    // }
                    doc.save();
                }


*/

