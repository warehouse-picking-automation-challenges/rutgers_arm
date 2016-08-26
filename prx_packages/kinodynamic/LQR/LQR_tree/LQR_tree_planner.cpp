/**
 * @file LQR_tree_planner.cpp
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

#include "prx/planning/motion_planners/sampling_planners/LQR_tree/LQR_tree_planner.hpp"

#include "prx/utilities/algorithms/graph/boost_wrappers.hpp"

#include <boost/property_map/property_map.hpp> 
#include <fstream>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::LQR_tree_planner_t, prx::motion_planner_t)

PRX_START

LQR_tree_planner_t::LQR_tree_planner_t() 
{
    success_counter = 0;
}

LQR_tree_planner_t::~LQR_tree_planner_t() {
    //nothing to do here
}

void LQR_tree_planner_t::reset() {
    //should be clearing things here
}

bool LQR_tree_planner_t::plan(double t)
{
#ifdef OCTAVE_FOUND
    if( boost::num_vertices(tree.graph) == 0 )
    {
        //perform an initial linearization and compute the LQR
        vector_t zero_vec(control_size);
        zero_vec.zero();
        control = control_space->alloc_point();
        control_space->set_from_vec(control, zero_vec);
        Matrix K(control_size, state_size);
        Matrix S(state_size, state_size);
        double radius = compute_lqr(goal, control, K, S);
        root = tree.add_vertex<lqr_node_t > ();

        lqr_node_t* node = tree.get_vertex_as<lqr_node_t > (root);
        node->radius = 6.28;
        node->K=K;
        node->S=S;
        node->control = control_space->clone_point(control);
        //for nearest query
        node->point = state_space->clone_point(goal);

    }

    //random sample
    state_t* random_sample = state_space->alloc_point();
    sampler->sample(state_space, random_sample);
    
    //determine an order of simualation
    std::vector<sim_order> nodes_to_simulate;
    get_sim_order(nodes_to_simulate,random_sample);    
    //perform the simulation of the nodes from the given state and update any basins that are invalid
    //if nothing was successful, add to the tree
    if( !simulate_forward(nodes_to_simulate,random_sample,t) )
    {        
        //get a trajectory
        sampler->sample(control_space, control);

        trajectory_t traj = local_planner->propagate_step(random_sample, control, t);        
        //make sure the end gets inside the basin
        trim_trajectory(traj);        
        if( traj.states.size() != 0 && validity_checker->is_valid(traj) )
        {
            //find parent node
            directed_vertex_index_t parent = find_parent(traj.states.back());
 
            if(parent != NULL)
            {
                //add node
                PRX_DEBUG_S("Applying the transformations for the matrices");
                for(int i=((int)traj.states.size())-1;i>=0;i--)
                {
                    directed_vertex_index_t new_node = tree.add_vertex<lqr_node_t > ();
                    lqr_node_t* node = tree.get_vertex_as<lqr_node_t > (new_node);
                    node->parent = parent;
                    node->control = control_space->clone_point(control);           
                    node->K = Matrix(control_size,state_size); 
                    node->S = Matrix(state_size,state_size);              
                    //for nearest query
                    node->point = state_space->clone_point(traj.states[i]);  
                    
                    node->radius = compute_lqr(node->point, node->control, node->K, node->S);
                    
                    //create an edge between these two nodes
                    tree.add_edge<lqr_edge_t>(parent,new_node);
                    
                    parent = new_node;
                }
            }
            traj.clear(state_space);
        }
        success_counter = 0;
    }
    
    success_counter++;

    state_space->free_point(random_sample);

    return achieved_goal();
#else
    return true;
#endif
}

bool LQR_tree_planner_t::outside_basin(state_t* state)
{

    foreach(directed_vertex_index_t v, boost::vertices(tree.graph))
    {
        lqr_node_t* node = tree.get_vertex_as<lqr_node_t > (v);

        if( state_space->distance(state, node->point) < node->radius )
        {
            return false;
        }
    }
    return true;
}


directed_vertex_index_t LQR_tree_planner_t::find_parent(state_t* state)
{
    std::deque<directed_vertex_index_t> queue;
    queue.push_back(root);
    
    while( queue.size() > 0 )
    {
        directed_vertex_index_t curr_index = queue.front();
        queue.pop_front();

        lqr_node_t* node = tree.get_vertex_as<lqr_node_t > (curr_index);

        if( state_space->distance(state, node->point) < node->radius )
        {
            return curr_index;
        }
        else
        {
            //get the children nodes and add them to the queue
            foreach(directed_edge_index_t e, boost::out_edges(curr_index, tree.graph))
            {
                queue.push_back(boost::target(e, tree.graph));
            }
        }
    }
    return NULL;
}

bool LQR_tree_planner_t::inside_basin(state_t* state)
{
    return !outside_basin(state);
}

bool LQR_tree_planner_t::trim_trajectory(trajectory_t& traj)
{
    int size = (int)traj.states.size();
    for( int i = size - 1; i >= 0; i-- )
    {
        if( inside_basin(traj.states[i]) )
        {
            break;
        }
        else
        {
            state_t* state = traj.states[i];
            state_space->free_point(state);
            traj.states.pop_back();
        }
    }
    return (traj.states.size());
}

bool LQR_tree_planner_t::achieved_goal() const
{
    return (success_counter >= max_successes);
}

void LQR_tree_planner_t::get_solution(plan_t& plan, const vector_t& goal, const vector_t& start) {
    //not used in LQR_tree_planner
}

std::vector<trajectory_t> LQR_tree_planner_t::get_planning_structure()
{
    //not used in LQR_tree_planner
    std::vector<trajectory_t> temp;
    return temp;
}

trajectory_t LQR_tree_planner_t::get_solution_trajectory()
{
    //not used in LQR_tree_planner
    trajectory_t temp;
    return temp;
}

void LQR_tree_planner_t::init(const parameter_reader_t* reader)
{
    motion_planner_t::init(reader);
    q_vector = reader->get_attribute_as<vector_t > ("Q");
    r_vector = reader->get_attribute_as<vector_t > ("R");
    max_successes = reader->get_attribute_as<unsigned>("max_successes",5000);
}

void LQR_tree_planner_t::give_plant_pointer(plant_t* plant)
{
    this->plant = plant;
}

void LQR_tree_planner_t::get_sim_order(std::vector<sim_order>& sim_orders,state_t* state)
{
#ifdef OCTAVE_FOUND
    const element_iterator_t& st = state_space->get_element_iterator(state);
    
    foreach(directed_vertex_index_t v, boost::vertices(tree.graph))
    {
        lqr_node_t* node = tree.get_vertex_as<lqr_node_t>(v);
        Matrix S = node->S;
        double radius = node->radius;
        sim_order temp;
        Matrix x_bar = Matrix(state_size,1);

        const element_iterator_t& in_st = state_space->get_element_iterator(node->point);
        for(int j=0;j<state_size;j++)
        {
            x_bar(j,0) =  st[j]-in_st[j];
        }
        temp.val = (radius - ((x_bar.transpose())*S*x_bar)(0,0)  );
        temp.node = v;
        if(temp.val > 0)
                sim_orders.push_back(temp);
    }
    
    
    comparer instance;
    std::sort(sim_orders.begin(), sim_orders.end(), instance);
#endif
}

#ifdef OCTAVE_FOUND

double LQR_tree_planner_t::compute_lqr(state_t* in_state, control_t* in_control, Matrix& K, Matrix& S)
{    
    state_t* plant_state = plant->pull_state_space()->alloc_point();
    control_t* plant_control = plant->pull_control_space()->alloc_point();    
    
    const element_iterator_t& st = state_space->get_element_iterator(in_state);
    const element_iterator_t& ct = control_space->get_element_iterator(in_control);
    
    for(unsigned i=0;i<state_size;i++)
    {
        plant->pull_state_space()->set_element(plant_state,i,st[i]);
    }
    for(unsigned i=0;i<control_size;i++)
    {
        plant->pull_control_space()->set_element(plant_control,i,ct[i]);
    }
    
    
    
    vector_t A_vec = plant->get_diff_wrt_state(plant_state, plant_control);
    vector_t B_vec = plant->get_diff_wrt_control(plant_state, plant_control);
    Matrix A(state_size, state_size);
    Matrix B(state_size, control_size);

    for( unsigned i = 0; i < state_size; i++ )
    {
        for( unsigned j = 0; j < state_size; j++ )
        {
            A(i, j) = A_vec[i * state_size + j];
        }
    }
    for( unsigned i = 0; i < state_size; i++ )
    {
        for( unsigned j = 0; j < control_size; j++ )
        {
            B(i, j) = B_vec[i * control_size + j];
        }
    }

    octave_value_list args;
    octave_value_list ret;

    args(0) = A;
    args(1) = B;
    args(2) = Q;
    args(3) = R;

    ret = caller.call_function("LQR_Octave", args);

    Matrix gain(ret(1).matrix_value());
    Matrix cost(ret(0).matrix_value());

    K = gain;
    S = cost;

    plant->pull_state_space()->free_point(plant_state);
    plant->pull_control_space()->free_point(plant_control);
    
    return compute_basin(A, B, K, S, in_state, in_control);
}

double LQR_tree_planner_t::compute_basin(Matrix& A, Matrix& B, Matrix& K, Matrix& S, state_t* in_state, control_t* in_control)
{
    //is meant to be a ridiculous over-approximation
    return 500.0;
}


bool LQR_tree_planner_t::simulate_forward(std::vector<sim_order>& sim_orders,state_t* state,double t)
{
    bool success = false;
        
    state_t* in_state = state_space->alloc_point();
    control_t* in_control = control_space->alloc_point();
    foreach(sim_order s, sim_orders)
    {
        directed_vertex_index_t v = s.node;
        directed_vertex_index_t new_v = v;

        Matrix inner_state(state_size, 1);
        Matrix inner_control(control_size, 1);
        
        state_space->copy_point(in_state,state);
        
        trajectory_t full_trajectory;
        full_trajectory.states.push_back(state_space->clone_point(in_state));
        
        while(v!=NULL)
        {
            //do the simulation in steps
            
            lqr_node_t* node = tree.get_vertex_as<lqr_node_t>(v);
            for( unsigned int i = 0; i < state_size; i++ )
            {
                inner_state(i, 0) = state_space->get_element(in_state, i) - state_space->get_element(node->point,i);
            }

            Matrix gain_matrix(control_size, state_size);

            gain_matrix = node->K;

            inner_control = -1 * gain_matrix * (inner_state);

            for( unsigned int i = 0; i < control_size; i++ )
                control_space->set_element(in_control, i, inner_control(i, 0) + control_space->get_element(node->control,i));

            trajectory_t traj = local_planner->propagate_step(in_state, in_control, sim_step);

            full_trajectory.states.push_back(state_space->clone_point(traj.states.back()));

            state_space->copy_point(in_state,full_trajectory.states.back());     

            traj.clear(state_space);
            
            //for the loop condition
            new_v = v;
            v = node->parent;
            
            
        }
        Matrix goal_cost = tree.get_vertex_as<lqr_node_t>(new_v)->S;
        double goal_radius = tree.get_vertex_as<lqr_node_t>(new_v)->radius;
        for( unsigned int i = 0; i < state_size; i++ )
        {
            inner_state(i, 0) = state_space->get_element(in_state, i) - state_space->get_element(tree.get_vertex_as<lqr_node_t>(new_v)->point,i);
        }
        
        v = s.node;
        if(((inner_state.transpose())*goal_cost*inner_state)(0,0) > goal_radius)
        {
            //this means the basins need to be updated
            unsigned index = 0;
            while(v!=NULL)
            {
                lqr_node_t* node = tree.get_vertex_as<lqr_node_t>(v);
                
                state_space->copy_point(in_state,full_trajectory.states[index]);
                for( unsigned int i = 0; i < state_size; i++ )
                {
                    inner_state(i, 0) = state_space->get_element(in_state, i) - state_space->get_element(node->point,i);
                }

                Matrix cost_matrix(state_size, state_size);

                cost_matrix = node->S;

                double new_radius = ((inner_state.transpose())*cost_matrix*inner_state)(0,0);
 
                if(new_radius < node->radius && node->parent!=NULL)
                {
                    node->radius = new_radius;
                }
                //for the loop condition
                v = node->parent;
                index++;
            }

        }
        else
        {
            success = true;
        }
           
        full_trajectory.clear(state_space);
    }
    state_space->free_point(in_state);
    control_space->free_point(in_control);
    
    
    return success;
}

#endif

void LQR_tree_planner_t::compute_qr_matrix()
{
    state_size = state_space->get_dimension();
    control_size = control_space->get_dimension();
    tree.link_space((space_t*)state_space);
#ifdef OCTAVE_FOUND
    Matrix Qi(state_size, state_size);
    Matrix Ri(control_size, control_size);
    for( unsigned i = 0; i < state_size; i++ )
    {
        for( unsigned j = 0; j < state_size; j++ )
        {
            Qi(i, j) = q_vector[i * state_size + j];
        }
    }
    for( unsigned i = 0; i < control_size; i++ )
    {
        for( unsigned j = 0; j < control_size; j++ )
        {
            Ri(i, j) = r_vector[i * control_size + j];
        }
    }
    Q = Qi;
    R = Ri;
#endif
}

void LQR_tree_planner_t::retrieve_lqr_tree(std::vector<double>& radii, std::vector<vector_t>& centers,std::vector<vector_t>& controls, std::vector<vector_t>& gains,std::vector<vector_t>& costs)
{

    PRX_DEBUG_S("size of graph: "<<boost::num_vertices(tree.graph));

#ifdef OCTAVE_FOUND 
    directed_vertex_index_t v,stored_v;
    

   
    Matrix inner_state(state_size, 1);
    Matrix inner_control(control_size, 1);
    
    double distance = -1;
    foreach(directed_vertex_index_t vertex, boost::vertices(tree.graph))
    {
        lqr_node_t* node = tree.get_vertex_as<lqr_node_t>(vertex);
        for( unsigned int i = 0; i < state_size; i++ )
        {
            inner_state(i, 0) = state_space->get_element(start, i) - state_space->get_element(node->point,i);
        }
        if((inner_state.transpose()*node->S*inner_state)(0,0) > distance)
        {
            v = vertex;
            distance = (inner_state.transpose()*node->S*inner_state)(0,0);
        }
    }
    trajectory_t full_trajectory;
    
    state_t* in_state = state_space->alloc_point();
    control_t* in_control = control_space->alloc_point();
    full_trajectory.states.push_back(state_space->clone_point(start));
    
    state_space->copy_point(in_state,start);
    stored_v = v;
    int counter = 0;
    while(v!=NULL)
    {
        //do the simulation in steps

        lqr_node_t* node = tree.get_vertex_as<lqr_node_t>(v);
        for( unsigned int i = 0; i < state_size; i++ )
        {
            inner_state(i, 0) = state_space->get_element(in_state, i) - state_space->get_element(node->point,i);
        }

        Matrix gain_matrix(control_size, state_size);

        gain_matrix = node->K;

        inner_control = -1 * gain_matrix * (inner_state);

        for( unsigned int i = 0; i < control_size; i++ )
            control_space->set_element(in_control, i, inner_control(i, 0) + control_space->get_element(node->control,i));

        trajectory_t traj = local_planner->propagate_step(in_state, in_control, sim_step);

        full_trajectory.states.push_back(state_space->clone_point(traj.states.back()));

        state_space->copy_point(in_state,full_trajectory.states.back());     

        traj.clear(state_space);

        //for the loop condition
        
        if(node->parent==NULL && counter<50)
            counter++;
        else
            v = node->parent;


    }
    
    unsigned index = 0;
    v = stored_v;
    counter =0;
    while(v!=NULL)
    {
        //get the node
        lqr_node_t* node = tree.get_vertex_as<lqr_node_t > (v);
        //extract the data from the node
        //radius of attraction
        radii.push_back(node->radius);
        vector_t state_vec(state_size);
        vector_t control_vec(control_size);
        vector_t gain_vec(state_size * control_size);
        vector_t cost_vec(state_size * state_size);
        element_iterator_t state_iter = state_space->get_element_iterator(node->point);
        element_iterator_t actual_iter = state_space->get_element_iterator(full_trajectory.states[index]);
        element_iterator_t control_iter = control_space->get_element_iterator(node->control);
        for( unsigned inner = 0; inner < state_space->get_dimension(); inner++ )
        {
            state_vec[inner] = state_iter[inner];
            inner_state(inner,0) = actual_iter[inner]-state_iter[inner];
        }
        centers.push_back(state_vec);
        
        
        inner_control = -1 * node->K * (inner_state);
        
        
        for( unsigned inner = 0; inner < control_space->get_dimension(); inner++ )
        {
            control_vec[inner] = inner_control(inner,0)+control_iter[inner];
        }
        controls.push_back(control_vec);
        for( unsigned c = 0; c < control_size; c++ )
        {
            for( unsigned s = 0; s < state_size; s++ )
            {
                gain_vec[c * state_size + s] = node->K(c, s);
            }
        }
        gains.push_back(gain_vec);
        for( unsigned c = 0; c < state_size; c++ )
        {
            for( unsigned s = 0; s < state_size; s++ )
            {
                cost_vec[c * state_size + s] = node->S(c, s);
            }
        }
        costs.push_back(cost_vec);
        
        if(node->parent==NULL && counter<50)
            counter++;
        else
            v = node->parent;
        
        index++;
        
        
    }
#endif

    return;
}

PRX_FINISH

