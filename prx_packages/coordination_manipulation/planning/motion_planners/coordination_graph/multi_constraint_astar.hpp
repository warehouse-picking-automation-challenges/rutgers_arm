/**
 * @file multi_constraint_astar_t.hpp 
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
#pragma once

#ifndef PRX_MULTI_CONSTRAINT_ASTAR_HPP
#define PRX_MULTI_CONSTRAINT_ASTAR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/simulation/control.hpp"
#include "prx/simulation/systems/system.hpp"
#include "planning/motion_planners/coordination_graph/coordination_constraint.hpp"
#include <deque>

namespace prx
{
    namespace packages
    {
        namespace coordination_manipulation
        {
                    
            enum
            {
                ASTAR_COLOR_WHITE, ASTAR_COLOR_GRAY, ASTAR_COLOR_BLACK
            };
            struct multi_constraint_node_t
            {
                double right_plan_step, left_plan_step;
                
                unsigned rob_step, lob_step;
                unsigned rob_index, lob_index;
                std::string right_type_identifier, left_type_identifier;
                
                double f_cost, g_cost;
                
                double edge_cost;
                
                int color;
                
                multi_constraint_node_t* parent_node;
                
                multi_constraint_node_t()
                {
                    right_plan_step = left_plan_step = edge_cost = 0.0;
                    f_cost = g_cost = PRX_INFINITY;
                    parent_node = NULL;
                    color = ASTAR_COLOR_WHITE;
                }
                
                multi_constraint_node_t(double r_step, double l_step, double rob_st, double lob_st, unsigned rob_in, unsigned lob_in, std::string r_type, std::string l_type)
                {
                    right_plan_step = r_step;
                    left_plan_step = l_step;
                    
                    rob_step = rob_st;
                    lob_step = lob_st;
                    
                    rob_index = rob_in;
                    lob_index = lob_in;
                    
                    right_type_identifier = r_type;
                    left_type_identifier = l_type;
                    
                    edge_cost = 0.0;
                    f_cost = g_cost = PRX_INFINITY;
                    parent_node = NULL;
                    color = ASTAR_COLOR_WHITE;
                    
                }
                
                void set(double r_step, double l_step, double rob_st, double lob_st, unsigned rob_in, unsigned lob_in, std::string r_type, std::string l_type)
                {
                    right_plan_step = r_step;
                    left_plan_step = l_step;
                    
                    rob_step = rob_st;
                    lob_step = lob_st;
                    
                    rob_index = rob_in;
                    lob_index = lob_in;
                    
                    right_type_identifier = r_type;
                    left_type_identifier = l_type;
                                 
                }
                
                void initialize()
                {
                    left_plan_step = 0;
                    right_plan_step = 0;
                    rob_step = 0;
                    lob_step = 0;
                    f_cost = g_cost = PRX_INFINITY;
                    edge_cost = 0.0;
                    parent_node = NULL;
                    color = ASTAR_COLOR_WHITE;
                    right_type_identifier = "";
                    left_type_identifier = "";
                }
                
                        
            };
            
            struct multi_constraint_node_pointer_t
            {
                multi_constraint_node_t* ptr;
            };
        }
    }
}



#include <boost/version.hpp>

#if BOOST_VERSION >= 104900
 #include <boost/heap/fibonacci_heap.hpp>
 #define HEAP_EXISTS
 typedef boost::heap::fibonacci_heap< prx::packages::coordination_manipulation::multi_constraint_node_pointer_t, boost::heap::compare<std::greater<prx::packages::coordination_manipulation::multi_constraint_node_pointer_t> > > multi_constraint_heap_def;
#else
 #include <queue>
 #define HEAP_DOES_NOT_EXIST
 typedef std::priority_queue<double,std::vector<prx::packages::coordination_manipulation::multi_constraint_node_pointer_t>,std::greater<prx::packages::coordination_manipulation::multi_constraint_node_pointer_t> > multi_constraint_heap_def;
#endif



namespace prx
{
    namespace packages
    {
        namespace coordination_manipulation
        {
            typedef multi_constraint_heap_def multi_constraint_heap_t;

            inline bool operator < (const multi_constraint_node_pointer_t& x, const multi_constraint_node_pointer_t& y)
            {
                return x.ptr->f_cost < y.ptr->f_cost;
            }
            inline bool operator > (const multi_constraint_node_pointer_t& x, const multi_constraint_node_pointer_t& y)
            {
                return x.ptr->f_cost > y.ptr->f_cost;
            }
            
            
            /**
             * Custome A* that operates over coordination constraint nodes
             * 
             * @author Andrew Kimmel
             * 
             */
            class multi_constraint_astar_t
            {

                public:

                    enum coordination_heuristic_type_t
                    {
                        BIAS_RIGHT, BIAS_LEFT, JOINT_BIAS, ANY_BIAS
                    };

                    multi_constraint_astar_t();
                    virtual ~multi_constraint_astar_t();

//                    virtual bool examine_edge(util::undirected_vertex_index_t start_index, util::undirected_edge_index_t edge, util::undirected_vertex_index_t end_index);
//                    virtual bool examine_vertex(util::undirected_vertex_index_t v);
//                    virtual double heuristic(util::undirected_vertex_index_t current, util::undirected_vertex_index_t goal);
//                    virtual double heuristic(util::undirected_vertex_index_t current, const std::vector<util::undirected_vertex_index_t>& goals);
//                    virtual bool check_if_goal_found(util::undirected_vertex_index_t potential_goal, util::undirected_vertex_index_t actual_goal);
//                    virtual bool solve(util::undirected_vertex_index_t start, const std::vector<util::undirected_vertex_index_t>& goals);
                    
                    virtual void link_constraints(coordination_constraints_t* constraints);
                    virtual void link_task_plans(util::hash_t<std::string, std::vector<sim::plan_t> >* right_plans_map, util::hash_t<std::string, std::vector<sim::plan_t> >* left_plans_map);
                    virtual void link_tasks(std::vector<unsigned>* right_tasks, std::vector<unsigned>* left_tasks, std::vector<std::string>* right_objects, std::vector<std::string>* left_objects);
                    virtual void set_coordination_bias(coordination_heuristic_type_t bias = JOINT_BIAS);
                    
                    //virtual void set_start_and_goal(unsigned right_plan_start, unsigned left_plan_start, unsigned right_plan_goal, unsigned left_plan_goal);
                    //virtual void set_coordination_problem( unsigned rob_index, unsigned lob_index, coordination_heuristic_type_t bias = JOINT_BIAS);

                    virtual bool solve();
                    
                    virtual void get_solution_plan (const util::space_t* right_arm_space, const util::space_t* left_arm_space, util::space_t* joint_arm_space, sim::plan_t& plan_to_fill);
                protected:
                    void clear_node_memory();
                    void define_batch_coordination_problem();
                    bool check_if_goal_found(const multi_constraint_node_t* current_node);
                    void get_valid_neighbors(const multi_constraint_node_t* current_node, std::vector<multi_constraint_node_t*>& neighbor_nodes);
                    double heuristic(const multi_constraint_node_t* current_node);
                    
                    bool get_valid_object_step_index(int step_index, unsigned& ob_step_index, unsigned& object_index, std::string& object_type_identifier, bool right_arm);
                    bool is_in_collision(unsigned rob_step_index, unsigned lob_step_index, unsigned rob_index, unsigned lob_index, std::string right_type, std::string left_type);
                    
                    multi_constraint_heap_t astar_heap;
                    std::vector< std::vector< multi_constraint_node_t* > > node_memory;
                    std::vector< std::vector<int> > colors;
                    multi_constraint_node_t* start_node, *goal_node;
                    
                    unsigned max_right_size, max_left_size;
                    
                    unsigned right_start, left_start;
                    unsigned right_goal, left_goal;
                    unsigned right_object, left_object;
                    std::string right_object_type, left_object_type;
                    
                    coordination_constraints_t* linked_constraints;
                    util::hash_t<std::string, std::vector<sim::plan_t> >* linked_right_plans, *linked_left_plans;
                    std::vector<unsigned>* linked_right_tasks, *linked_left_tasks;
                    std::vector<std::string>* linked_right_objects, *linked_left_objects;
                    
                    coordination_heuristic_type_t bias_type;
                    bool append_actual_goal;
            };
        }

    }
}

#endif