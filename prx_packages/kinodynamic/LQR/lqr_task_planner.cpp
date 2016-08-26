/**
 * @file lqr_task_planner.cpp
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

#include "prx/planning/task_planners/lqr_task_planner.hpp"
#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::lqr_task_planner_t, prx::task_planner_t)

PRX_START

lqr_task_planner_t::lqr_task_planner_t() { }

lqr_task_planner_t::~lqr_task_planner_t() { }

void lqr_task_planner_t::init(const parameter_reader_t* reader)
{
#ifdef OCTAVE_FOUND    
    task_planner_t::init(reader);
    if( reader->has_attribute("cycle_time") )
    {
        cycle_time = reader->get_attribute_as<double>("cycle_time");
    }
    else
    {
        PRX_LOG_ERROR("Missing cycle_time attribute in input files.");
    } 
    if( reader->has_attribute("planning_cycles") )
    {
        planning_cycles = reader->get_attribute_as<double>("planning_cycles");
    }
    else
    {
        PRX_LOG_ERROR("Missing cycle_time attribute in input files.");
    }
    
    plant_path = reader->get_attribute_as<std::string>("plant_path");
    controller_path = reader->get_attribute_as<std::string>("lqr_tree_controller");
    planner = dynamic_cast<LQR_tree_planner_t*>(planners[0]);
    plant_t* plant = dynamic_cast<plant_t*>(model->get_system(plant_path).get());
    planner->give_plant_pointer(plant);
    planner->compute_qr_matrix();

    foreach(const parameter_reader_t* state_reader, reader->get_list("states"))
    {
        std::string planner_name = state_reader->get_attribute_as<std::string > ("planner_name");
        model->use_state_space(planner_name);
        start_states[planner_name] = state_reader->get_attribute_as<vector_t > ("start_state");
        goal_states[planner_name] = state_reader->get_attribute_as<vector_t > ("goal_state");
    }
    for( unsigned int i = 0; i < planners.size(); i++ )
    {
        planners[i]->link_goal(goal_states[planner_names[i]]);
        planners[i]->link_start(start_states[planner_names[i]], model->get_initial_state());
    }
#endif    
}

void lqr_task_planner_t::run()
{
#ifdef OCTAVE_FOUND    
    // This function tells the planning_comm_t how to convert a state into a
    // configuration.
    planning_comm_t::state_to_config_t to_config =
            boost::bind(&world_model_t::get_config, model, _1, _2, _3);

    unsigned count = 0;
    do
    {
        count++;
        if(count%100 == 0)
                PRX_INFO_S("LQR CYCLE: "<<count);
        
    }
    while(!planner->plan(cycle_time) && count < planning_cycles);
    
    std::vector<double> radii;
    std::vector<vector_t> centers;
    std::vector<vector_t> controls;
    std::vector<vector_t> gains;
    std::vector<vector_t> costs;
    
    planner->retrieve_lqr_tree(radii,centers,controls,gains,costs);
    communication->request_send_plans_2(controller_path,controls,.017);
    //communication->request_send_lqr(controller_path,radii,centers,controls,gains,costs);
#endif
}

PRX_FINISH

