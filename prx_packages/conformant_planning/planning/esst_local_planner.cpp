/**
 * @file time_varying_local_planner.cpp
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
#include "planning/esst_local_planner.hpp"

#include "prx/planning/world_model.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "simulation/particle_trajectory.hpp"


#include <pluginlib/class_list_macros.h> 
#include <boost/thread/thread.hpp>
#include <boost/function.hpp>

PLUGINLIB_EXPORT_CLASS( prx::plan::esst_local_planner_t, prx::plan::local_planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace packages::conformant;
    
    namespace plan
    {

        void esst_local_planner_t::init(const parameter_reader_t * const reader, const parameter_reader_t * const template_reader)
        {
            local_planner_t::init(reader, template_reader);
            control_variance = parameters::get_attribute_as<std::vector<double> > ("control_variance", reader, template_reader);
            lower_multiple = parameters::get_attribute_as<int>("lower_multiple", reader, template_reader, 50);
            upper_multiple = parameters::get_attribute_as<int>("upper_multiple", reader, template_reader, 500);

            std::vector<std::string> names = parameters::get_attribute_as<std::vector<std::string> >("parallel_props", reader, template_reader);

            foreach(std::string name, names)
            {
                PRX_INFO_S("/prx/"+name);
                acs.push_back(new actionlib::SimpleActionClient<prx_planning::parallel_propAction>("/prx/"+name, true));
            }
        }

        void esst_local_planner_t::link_particle_space(util::particle_space_t* p_space)
        {
            particle_space = p_space;
            trajs = new sim::trajectory_t[particle_space->get_num_particles()];
            for(int i=0;i<particle_space->get_num_particles();i++)
            {
                trajs[i].link_space(particle_space->get_point_space());
            }
            for(int i=0;i<parallel_models.size()+1;i++)
            {
                noisy_plans.push_back(new plan_t(world_model->get_control_space()));
            }
        }

        void esst_local_planner_t::steer(const state_t* start, const state_t* goal, plan_t& plan, trajectory_t& traj, bool connect)
        {
            if(acs.size()>0)
            {
                traj.resize(2);

                const particle_point_t* particle_point = dynamic_cast<const particle_point_t*>(start);
                int random_number = uniform_int_random(lower_multiple, upper_multiple);
                plan.clear();
                plan.append_onto_back(random_number*simulation::simulation_step);
                sampler->sample(world_model->get_control_space(),plan.back().control);\

                state_t* final_state = world_model->get_state_space()->alloc_point();


                std::vector<bool> clients(acs.size());
                for(int i=0;i<acs.size();i++)
                {
                    clients[i] = false;
                }

                for(int i=0;i<particle_space->get_num_particles()+acs.size();i++)
                {
                    actionlib::SimpleActionClient<prx_planning::parallel_propAction>* ac = acs[i%acs.size()];
                    if(clients[i%acs.size()])
                    {
                        // PRX_WARN_S("Reading: "<<i%acs.size());
                        //wait for the action to return
                        bool finished_before_timeout = ac->waitForResult();


                        world_model->get_full_state_space()->set_from_vector(ac->getResult()->final_state);
                        world_model->get_state_space()->copy_to_point(final_state);
                        

                        particle_trajectory_t& particle_traj = dynamic_cast<particle_trajectory_t&>(traj);

                        if(world_model->is_phys_engine())
                        {
                            particle_traj.collisions.resize(particle_space->get_num_particles());
                            particle_traj.collisions[i-acs.size()]=!(ac->getResult()->collision_free);
                        }

                        particle_space->copy_from_particle(particle_point->links[i-acs.size()],i-acs.size(),traj[0]);
                        particle_space->copy_from_particle(final_state,i-acs.size(),traj[1]);
                        clients[i%acs.size()] = false;
                    }
                    if(!clients[i%acs.size()] && i < particle_space->get_num_particles() )
                    {
                        // PRX_WARN_S("Sending: "<<i%acs.size());
                        ac->waitForServer();
                        // send a goal to the action
                        prx_planning::parallel_propGoal goal;
                        goal.planning_context = world_model->get_current_context();
                        world_model->push_state(particle_point->links[i]);
                        world_model->get_full_state_space()->copy_to_vector(goal.initial_state);
                        world_model->get_control_space()->copy_point_to_vector(plan.back().control,goal.initial_control);
                        goal.duration = plan.back().duration;

                        for(unsigned j=0;j<world_model->get_control_space()->get_dimension();j++)
                        {
                            goal.initial_control[j] += uniform_random(-.5,.5)*control_variance[j];
                        }
                        ac->sendGoal(goal);
                        clients[i%acs.size()] = true;
                    }
                }
            
                // PRX_INFO_S(particle_space->print_point(traj[1]));
                world_model->get_state_space()->free_point(final_state);
                return;
            }
            else
            {
                const particle_point_t* particle_point = dynamic_cast<const particle_point_t*>(start);
                int random_number = uniform_int_random(lower_multiple, upper_multiple);
                plan.clear();
                plan.append_onto_back(random_number*simulation::simulation_step);
                sampler->sample(world_model->get_control_space(),plan.back().control);
                traj.resize(random_number+1);


                noisy_plans[0]->clear();
                noisy_plans[0]->append_onto_back(random_number*simulation::simulation_step);
                parallel_steer_traj(particle_point,&traj,&plan,noisy_plans[0],random_number,world_model,0,particle_space->get_num_particles());
                
                // {
                //     std::vector<boost::thread*> threads;
                //     int num_particles_per_thread = particle_space->get_num_particles()/(parallel_models.size()+1);
                //     for(int i=0;i<parallel_models.size()+1;i++)
                //     {
                //         // PRX_INFO_S("Start Thread: "<<i);
                //         noisy_plans[i]->clear();
                //         noisy_plans[i]->append_onto_back(random_number*simulation::simulation_step);

                //         if(parallel_models.size()==i)
                //         {
                //             boost::thread* new_thread = new boost::thread(&esst_local_planner_t::parallel_steer_traj,this,
                //                                     particle_point,&traj,&plan,noisy_plans[i],random_number,
                //                                     world_model,i*num_particles_per_thread, particle_space->get_num_particles());
                //             threads.push_back(new_thread);//group.add_thread(new_thread);
                //             // parallel_steer_traj(particle_point,&traj,&plan,noisy_plans[i],random_number,
                //             //                         world_model,i*num_particles_per_thread, particle_space->get_num_particles());
                //         }
                //         else
                //         {
                //             boost::thread* new_thread = new boost::thread(&esst_local_planner_t::parallel_steer_traj,this,
                //                                     particle_point,&traj,&plan,noisy_plans[i],random_number,
                //                                     parallel_models[i],i*num_particles_per_thread, (i+1)*num_particles_per_thread);
                //             threads.push_back(new_thread);//group.add_thread(new_thread);
                //             // parallel_steer_traj(particle_point,&traj,&plan,noisy_plans[i],random_number,
                //             //                         parallel_models[i],i*num_particles_per_thread, (i+1)*num_particles_per_thread);
                //         }
                //     }
                //     // group.join_all();
                //     foreach(boost::thread* t, threads)
                //     {
                //         t->join();
                //         delete t;
                //     }
                // }


                if(world_model->is_phys_engine())
                {
                    particle_trajectory_t& particle_traj = dynamic_cast<particle_trajectory_t&>(traj);
                    particle_traj.collisions.resize(particle_space->get_num_particles());
                    for(int i = 0 ;i<particle_space->get_num_particles();i++)
                    {
                        particle_traj.collisions[i]=trajs[i].in_collision();
                    }
                }
            }
        }

        void esst_local_planner_t::steer(const state_t* start, const state_t* goal, plan_t& plan, state_t* result, bool connect)
        {

            const particle_point_t* particle_point = dynamic_cast<const particle_point_t*>(start);
            particle_point_t* particle_result = dynamic_cast< particle_point_t*>(result);
            int random_number = uniform_int_random(lower_multiple, upper_multiple);
            plan.clear();
            noisy_plans[0]->clear();
            plan.append_onto_back(random_number*simulation::simulation_step);
            noisy_plans[0]->append_onto_back(random_number*simulation::simulation_step);
            sampler->sample(world_model->get_control_space(),plan.back().control);
            setup_parallel_models();
            parallel_steer_point(particle_point,particle_result,plan,noisy_plans[0],random_number,world_model,0,particle_space->get_num_particles());
        }

        void esst_local_planner_t::setup_parallel_models()
        {
            std::string current_context = world_model->get_current_context();
            foreach(world_model_t* model, parallel_models)
            {
                model->use_context(current_context);
            }
        }

        void esst_local_planner_t::parallel_steer_point(const particle_point_t* particle_point, particle_point_t* particle_result, sim::plan_t& plan, sim::plan_t* noisy_plan,
                                    int random_number, world_model_t* which_model, int start_index, int end_index)
        {

            for(int i = start_index; i<end_index; i++)
            {
                //create noise
                for(unsigned j=0;j<which_model->get_control_space()->get_dimension();j++)
                {
                    noisy_plan->back().control->memory[j] = plan.back().control->memory[j] + uniform_random(-.5,.5)*control_variance[j];
                }

                which_model->propagate_plan(particle_point->links[i], *noisy_plan, particle_result->links[i]);
            }

        }

        void esst_local_planner_t::parallel_steer_traj(const particle_point_t* particle_point, sim::trajectory_t* traj, sim::plan_t* plan, sim::plan_t* noisy_plan,
                                    int random_number, world_model_t* which_model, int start_index, int end_index)
        {
            // PRX_INFO_S(which_model<<" "<<world_model);
            for(int i = start_index; i<end_index; i++)
            {
                //create noise
                for(unsigned j=0;j<which_model->get_control_space()->get_dimension();j++)
                {
                    noisy_plan->back().control->memory[j] = plan->back().control->memory[j] + uniform_random(-.5,.5)*control_variance[j];
                }

                which_model->propagate_plan(particle_point->links[i], *noisy_plan, trajs[i]);

                boost::lock_guard<boost::mutex> lock(mutex);
                for(int j = 0 ;j<traj->size();j++)
                {
                    particle_space->copy_from_particle(trajs[i][j],i,traj->at(j));
                }
            }

        }

        void esst_local_planner_t::propagate_step(const sim::state_t* start, const sim::plan_t& plan, sim::state_t* state)
        {
            const particle_point_t* particle_point = dynamic_cast<const particle_point_t*>(start);
            particle_point_t* particle_result = dynamic_cast< particle_point_t*>(state);

            for(int i = 0; i<particle_space->get_num_particles(); i++)
            {
                //create noise
                world_model->propagate_plan(particle_point->links[i], plan, particle_result->links[i]);
            }

        }

        void esst_local_planner_t::propagate(const sim::state_t* start, const sim::plan_t& plan, sim::trajectory_t& traj)
        {
            const particle_point_t* particle_point = dynamic_cast<const particle_point_t*>(start);
            PRX_INFO_S(particle_point);

            for(int i = 0; i<particle_space->get_num_particles(); i++)
            {
                //create noise
                world_model->propagate_plan(particle_point->links[i], plan, trajs[i]);
            }
            traj.resize(trajs[0].size());

            for(int j = 0 ;j<traj.size();j++)
            {
                for(int i = 0 ;i<particle_space->get_num_particles();i++)
                {
                    particle_space->copy_from_particle(trajs[i][j],i,traj[j]);
                }
            }
            if(world_model->is_phys_engine())
            {
                // particle_trajectory_t& particle_traj = dynamic_cast<particle_trajectory_t&>(traj);
                // particle_traj.collisions.resize(particle_space->get_num_particles());
                // for(int i = 0 ;i<particle_space->get_num_particles();i++)
                // {
                //     particle_traj.collisions[i]=trajs[i].in_collision();
                // }
            }

        }



    }
}