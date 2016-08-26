#pragma once
/**
 * @file LQR_tree_planner.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2012, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Zakary Littlefield, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, James Marble, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#ifndef PRX_LQR_TREE_PLANNER_HPP
#define	PRX_LQR_TREE_PLANNER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/motion_planner.hpp"
#include "prx/utilities/graph/directed_graph.hpp"

#ifdef OCTAVE_FOUND
#include "prx/utilities/octave_interface/octave_caller.hpp"
#endif
PRX_START


class lqr_node_t : public directed_node_t
{

  public:
    lqr_node_t()
    {
        parent = NULL;
    }
    virtual ~lqr_node_t(){ }
#ifdef OCTAVE_FOUND
    Matrix K;
    Matrix S;
#endif   
    double radius;
    state_t* state;
    control_t* control;
    directed_vertex_index_t parent;
    
};

class lqr_edge_t : public directed_edge_t
{

  public:

    lqr_edge_t(){ }

    virtual ~lqr_edge_t(){ }
};

struct sim_order
{
  directed_vertex_index_t node;
  double val;
};
struct comparer {
  bool operator() (sim_order i,sim_order j) { return (i.val>j.val);}
};

/**
 * The planning implementation of the LQR-tree approach from Tedrake et al. 
 */
class LQR_tree_planner_t : public motion_planner_t
{

  public:
    LQR_tree_planner_t();
    virtual ~LQR_tree_planner_t();
    virtual void init(const parameter_reader_t* reader);
    virtual void reset();
    virtual bool plan(double t);
    virtual bool achieved_goal() const;


    virtual void get_solution(plan_t& plan, const vector_t&, const vector_t& start);
    virtual std::vector<trajectory_t> get_planning_structure();
    virtual trajectory_t get_solution_trajectory();
    
    
    ///functions specific to LQR-tree
    void give_plant_pointer(plant_t* plant);
    void compute_qr_matrix();
    void retrieve_lqr_tree(std::vector<double>& radii,std::vector<vector_t>& centers,std::vector<vector_t>& controls,std::vector<vector_t>& gains,std::vector<vector_t>& costs);
    void get_sim_order(std::vector<sim_order>& sim_orders,state_t* state);
    bool simulate_forward(std::vector<sim_order>& sim_orders,state_t* state,double t);
    

  protected:
    
#ifdef OCTAVE_FOUND
    double compute_lqr(state_t* in_state, control_t* in_control, Matrix& K, Matrix& S);
    double compute_basin(Matrix& A, Matrix& B, Matrix& K, Matrix& S, state_t* in_state, control_t* in_control);
    Matrix Q,R;
    octave_caller_t caller;
#endif
    bool outside_basin(state_t* state);
    bool inside_basin(state_t* state);
    bool trim_trajectory(trajectory_t& traj);
    directed_vertex_index_t find_parent(state_t* state);
    
    
    directed_graph_t tree;
    directed_vertex_index_t root;
    vector_t q_vector;
    vector_t r_vector;
    plant_t* plant;
    control_t* control;
    unsigned state_size ;
    unsigned control_size ;
    unsigned success_counter;
    unsigned max_successes;

  private:

};

PRX_FINISH

#endif	// PRX_LQR_TREE_PLANNER_HPP

