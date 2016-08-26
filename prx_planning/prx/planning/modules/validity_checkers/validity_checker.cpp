/**
 * @file validity_checker.cpp 
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

#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/utilities/heuristic_search/null_constraints.hpp"

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace plan
    {

        std::deque<std::pair<unsigned,unsigned> > queue;
        pluginlib::ClassLoader<validity_checker_t> validity_checker_t::loader("prx_planning", "prx::plan::validity_checker_t");

        validity_checker_t::validity_checker_t() : world_model(NULL), indices_set(false) { }

        void validity_checker_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader) 
        {
            cost_function_name = parameters::get_attribute_as<std::string>("cost_function",reader,template_reader,"default_uniform");
        }

        bool validity_checker_t::is_valid(const trajectory_t& input)
        {
            //TODO: We need to figure out how to do this in a way that makes everyone a.k.a. KOSTAS happy
            
            // full_space_traj.link_space(world_model->get_full_state_space());
            // full_space_traj.clear();
            // state_t* full_state = world_model->get_full_state_space()->alloc_point();
            // foreach(state_t* s, input)
            // {
            //     world_model->push_state(s);
            //     world_model->get_full_state_space()->copy_to_point(full_state);
            //     full_space_traj.copy_onto_back(full_state);
            // }
            // bool result = world_model->get_simulator()->traj_in_collision(full_space_traj);
            // world_model->get_full_state_space()->free_point(full_state);
            // return !result;

            return validate_and_generate_constraints(NULL, input);
        }
        bool validity_checker_t::is_valid(const trajectory_t& input, unsigned min, unsigned max)
        {
            unsigned index = (max+min)>>1;
            if(!is_valid(input[index]))
                return false;
            else if(max==min)
                return true;
            else if(min!=index)
                return is_valid(input,min,index-1) && is_valid(input,index+1,max);
            else
                return is_valid(input,index+1,max);
        }

        void validity_checker_t::valid_trajectory(const space_t* space, const trajectory_t& input, trajectory_t& result)
        {
            result.clear();
            foreach(state_t* state, input)
            {
                if( !is_valid(state) )
                    return;
                result.copy_onto_back(state);
            }
        }

        void validity_checker_t::link_distance_function(util::distance_t dist)
        {
            if(cost_function_name=="default")
            {
                cost_function_name = "default_uniform";
            }
            cost_function = cost_function_t::get_loader().createUnmanagedInstance("prx_simulation/"+cost_function_name);
            cost_function->link_distance_function(dist);
            state_cost = cost_function->cost_of_state;
            trajectory_cost = cost_function->cost_of_trajectory;
            heuristic = cost_function->heuristic;
            true_cost = cost_function->true_cost;
        }

        constraints_t* validity_checker_t::alloc_constraint() const
        {
            return new null_constraints_t();
        }
            
        void validity_checker_t::free_constraint(const util::constraints_t* constraint_to_delete) const
        {
            delete constraint_to_delete;
        }

        void validity_checker_t::valid_trajectory_prefix(const space_t* space, const trajectory_t& input, trajectory_t& result, unsigned int size)
        {
            unsigned int traj_size = 0;
            result.clear();

            foreach(state_t* state, input)
            {
                if( !is_valid(state) || traj_size == size )
                    return;
                result.copy_onto_back(state);
                traj_size++;
            }
        }

        bool validity_checker_t::validate_and_generate_constraints(constraints_t* constraints, const sim::state_t* state)
        {
            // PRX_WARN_S("Abstract Validity Checker generate constraints function should not be called: Please overload validate_and_generate_constraints()!");
            return is_valid(state);
        }

        bool validity_checker_t::validate_and_generate_constraints(constraints_t* constraints, const trajectory_t& path)
        {
            queue.clear();
            unsigned min = 0;
            unsigned max = indices_set ? state_check_indices.size()-1 : path.size()-1;
            queue.push_back(std::pair<unsigned,unsigned>(min,max));
            while(queue.size()!=0)
            {
                std::pair<unsigned,unsigned > interval = queue.front();
                queue.pop_front();
                min = interval.first;
                max = interval.second;
                
                //Retrieve the appropriate state to check
                unsigned index = (max+min)>>1;
                state_t* st = NULL;
                if( indices_set )
                {
                    st = path[state_check_indices[index]];
                }
                else
                {
                    st = path[index];
                }
                
                if( constraints != NULL )
                {
                    if(!validate_and_generate_constraints( constraints, st ))
                    {
                        constraints->clear();
                        indices_set = false;
                        return false;
                    }
                }
                else
                {
                    if(!is_valid( st ))
                    {
                        indices_set = false;
                        return false;                    
                    }
                }
                if(min!=index)
                {
                    queue.push_back(std::pair<unsigned,unsigned>(min,index-1));
                    queue.push_back(std::pair<unsigned,unsigned>(index+1,max));
                }
                else if(max!=min)
                {
                    queue.push_back(std::pair<unsigned,unsigned>(index+1,max));
                }
            }
            indices_set = false;
            return true;
        }

        void validity_checker_t::link_model(world_model_t* model)
        {
            world_model = model;
        }

        void validity_checker_t::valid_trajectory(trajectory_t& input)
        {
            unsigned int traj_size = 0;
            foreach(state_t* state, input)
            {
                if( !is_valid(state) )
                    break;
                traj_size++;
            }
            input.resize(traj_size);
        }

        void validity_checker_t::get_constraint_names( std::vector< std::string >& names )
        {
            PRX_WARN_S("get_constraint_names is not implemented in base validity_checker!!");
        }
        
        void validity_checker_t::generate_constraint_names( )
        {
            PRX_WARN_S("generate_constraint_names is not implemented in base validity_checker!!");
        }

        void validity_checker_t::clear_constraint_names( )
        {
            PRX_WARN_S("clear_constraint_names is not implemented in base validity_checker!!");
        }


        pluginlib::ClassLoader<validity_checker_t>& validity_checker_t::get_loader()
        {
            return loader;
        }

    }
}

