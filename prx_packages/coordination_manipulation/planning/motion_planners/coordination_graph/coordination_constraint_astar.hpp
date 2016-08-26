/**
 * @file coordination_constraint_astar.hpp 
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

#ifndef PRX_COORDINATION_CONSTRAINT_ASTAR_HPP
#define PRX_COORDINATION_CONSTRAINT_ASTAR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "planning/motion_planners/coordination_graph/coordination_constraint.hpp"
#include <deque>

namespace prx
{
    namespace packages
    {
        namespace coordination_manipulation
        {
            struct constraint_node_t
            {
                double right_plan_step, left_plan_step;
                
                double f_cost, g_cost;
                
                double edge_cost;
                
                int* color;
                
                constraint_node_t* parent_node;
                
                constraint_node_t()
                {
                    right_plan_step = left_plan_step = edge_cost = 0.0;
                    f_cost = g_cost = PRX_INFINITY;
                    parent_node = NULL;
                }
                
                constraint_node_t(double r_step, double l_step)
                {
                    right_plan_step = r_step;
                    left_plan_step = l_step;
                    edge_cost = 0.0;
                    f_cost = g_cost = PRX_INFINITY;
                    parent_node = NULL;
                    
                }
                
                void initialize()
                {
                    f_cost = g_cost = PRX_INFINITY;
                    edge_cost = 0.0;
                    parent_node = NULL;
                }
            };
            
            struct constraint_node_pointer_t
            {
                constraint_node_t* ptr;
            };
        }
    }
}



#include <boost/version.hpp>

#if BOOST_VERSION >= 104900
 #include <boost/heap/fibonacci_heap.hpp>
 #define HEAP_EXISTS
 typedef boost::heap::fibonacci_heap< prx::packages::coordination_manipulation::constraint_node_pointer_t, boost::heap::compare<std::greater<prx::packages::coordination_manipulation::constraint_node_pointer_t> > > constraint_heap_def;
#else
 #include <queue>
 #define HEAP_DOES_NOT_EXIST
 typedef std::priority_queue<double,std::vector<prx::packages::coordination_manipulation::constraint_node_pointer_t>,std::greater<prx::packages::coordination_manipulation::constraint_node_pointer_t> > constraint_heap_def;
#endif



namespace prx
{
    namespace packages
    {
        namespace coordination_manipulation
        {
            typedef constraint_heap_def constraint_heap_t;

            inline bool operator < (const constraint_node_pointer_t& x, const constraint_node_pointer_t& y)
            {
                return x.ptr->f_cost < y.ptr->f_cost;
            }
            inline bool operator > (const constraint_node_pointer_t& x, const constraint_node_pointer_t& y)
            {
                return x.ptr->f_cost > y.ptr->f_cost;
            }
            
            
            /**
             * Custome A* that operates over coordination constraint nodes
             * 
             * @author Andrew Kimmel
             * 
             */
            class coordination_constraint_astar_t
            {

                public:

                    enum coordination_heuristic_type_t
                    {
                        BIAS_RIGHT, BIAS_LEFT, JOINT_BIAS, ANY_BIAS
                    };
                    
                    enum
                    {
                        ASTAR_COLOR_WHITE, ASTAR_COLOR_GRAY, ASTAR_COLOR_BLACK
                    };

                    coordination_constraint_astar_t();
                    virtual ~coordination_constraint_astar_t();

//                    virtual bool examine_edge(util::undirected_vertex_index_t start_index, util::undirected_edge_index_t edge, util::undirected_vertex_index_t end_index);
//                    virtual bool examine_vertex(util::undirected_vertex_index_t v);
//                    virtual double heuristic(util::undirected_vertex_index_t current, util::undirected_vertex_index_t goal);
//                    virtual double heuristic(util::undirected_vertex_index_t current, const std::vector<util::undirected_vertex_index_t>& goals);
//                    virtual bool check_if_goal_found(util::undirected_vertex_index_t potential_goal, util::undirected_vertex_index_t actual_goal);
//                    virtual bool solve(util::undirected_vertex_index_t start, const std::vector<util::undirected_vertex_index_t>& goals);
                    
                    virtual void link_constraints(coordination_constraints_t* constraints);
                    
                    virtual void set_start_and_goal(unsigned right_plan_start, unsigned left_plan_start, unsigned right_plan_goal, unsigned left_plan_goal);
                    virtual void set_coordination_problem( unsigned rob_index, unsigned lob_index, std::string right_object_type, std::string left_object_type, coordination_heuristic_type_t bias = JOINT_BIAS);

                    virtual bool solve();
                    
                    virtual void get_solution_path (std::deque<unsigned>& right_solution_indices, std::deque<unsigned>& left_solution_indices);
                protected:
                    
                    bool check_if_goal_found(const constraint_node_t* current_node);
                    void get_valid_neighbors(const constraint_node_t* current_node, std::vector<constraint_node_t*>& neighbor_nodes);
                    double heuristic(const constraint_node_t* current_node);
                    
                    constraint_heap_t astar_heap;
                    std::vector< std::vector<constraint_node_t*> > nodes;
                    std::vector< std::vector<int> > colors;
                    constraint_node_t* start_node, *goal_node;
                    
                    unsigned max_right_size, max_left_size;
                    
                    unsigned right_start, left_start;
                    unsigned right_goal, left_goal;
                    unsigned right_object, left_object;
                    std::string right_type, left_type;
                    coordination_constraints_t* linked_constraints;
                    
                    coordination_heuristic_type_t bias_type;
                    bool append_actual_goal;
            };
        }

    }
}

#endif