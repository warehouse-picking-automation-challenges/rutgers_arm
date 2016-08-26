/**
 * @file simple_pap_tp.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "planning/task_planners/simple_pap_tp.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/heuristic_search/object_collision_constraints.hpp"

#include "planning/specifications/simple_pap_specification.hpp"

#include <boost/range/adaptor/map.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::simple_pap_tp_t ,prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace manipulation
        {

            simple_pap_tp_t::simple_pap_tp_t()
            {

            }

            simple_pap_tp_t::~simple_pap_tp_t()
            {

            }

            void simple_pap_tp_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                PRX_DEBUG_COLOR("Initializing simple_pap_tp_t task planner ...", PRX_TEXT_CYAN);
                task_planner_t::init(reader,template_reader);                
                full_manipulator_context_name = parameters::get_attribute("full_manipulator_context_name", reader, template_reader);
                manipulation_context_name = parameters::get_attribute("manipulation_context_name", reader, template_reader);
                object_target_vec = parameters::get_attribute_as<std::vector<double> >("object_target", reader, template_reader);
                manipulation_task_planner_name = parameters::get_attribute("manipulation_task_planner_name", reader, template_reader);
                object_name = parameters::get_attribute("object_name", reader, template_reader);
            }

            void simple_pap_tp_t::link_world_model(world_model_t * const model)
            {
                task_planner_t::link_world_model(model);
                manipulation_model = dynamic_cast<manipulation_world_model_t*>(model);
                if(manipulation_model == NULL)
                    PRX_FATAL_S("The pick and place task planner can work only with manipulation world model!");
            }

            void simple_pap_tp_t::link_query(query_t* new_query)
            {
                task_planner_t::link_query(new_query);
                in_query = static_cast<motion_planning_query_t*>(new_query);
            }

            void simple_pap_tp_t::setup()
            {
                manipulation_model->use_context(full_manipulator_context_name);
                manip_initial_state = manipulation_model->get_state_space()->alloc_point();

                foreach(planner_t* planner, planners | boost::adaptors::map_values)
                {
                    PRX_DEBUG_COLOR("Simple P&P is seting up planner: " << planner->get_name(), PRX_TEXT_CYAN);
                    planner->setup();
                    output_specifications[planner->get_name()]->setup( manipulation_model );
                    planner->link_specification( output_specifications[planner->get_name()] );
                }
            }

            bool simple_pap_tp_t::execute()
            {
                foreach(planner_t* planner, planners | boost::adaptors::map_values)
                {                    
                    PRX_PRINT("Simple P&P executing planner: " << planner->get_name(), PRX_TEXT_GREEN);
                    planner->execute();
                }
                return true;
            }

            bool simple_pap_tp_t::succeeded() const
            {
                return true;
            }

            const util::statistics_t* simple_pap_tp_t::get_statistics()
            {
                return NULL;
            }

            void simple_pap_tp_t::resolve_query()
            {
                config_t retract_config;
                retract_config.set_position(0,0,-.08);
                retract_config.set_orientation(0,0,0,1);
                PRX_DEBUG_COLOR("Resolve query from simple_pap_tp ...  context:" << manipulation_context_name,PRX_TEXT_GREEN);
                manipulation_model->use_context(full_manipulator_context_name);
                manipulation_model->get_state_space()->copy_from_point(manip_initial_state);  

                manipulation_model->use_context(manipulation_context_name);
                std::vector<movable_body_plant_t* > objects;
                manipulation_model->get_objects(objects);

                movable_body_plant_t* move_object = NULL;
                std::string name, subpath;
                for(unsigned i = 0; i<objects.size() && move_object == NULL; ++i)
                {
                    boost::tie(subpath, name) = reverse_split_path(objects[i]->get_pathname());
                    PRX_DEBUG_COLOR("name : " << name << "  subpath:" << subpath, PRX_TEXT_GREEN);
                    if(name == object_name)
                        move_object = objects[i];
                }

                const space_t* object_space = move_object->get_state_space();

                manipulator = manipulation_model->get_current_manipulation_info()->manipulator;                
                state_t* initial_state = manipulation_model->get_state_space()->alloc_point();

                state_t* initial_object = object_space->alloc_point();
                state_t* target_object = object_space->alloc_point();
                object_space->copy_vector_to_point(object_target_vec, target_object);

                // manipulation_query = new manipulation_query_t(manipulation_context_name, manipulation_query_t::PRX_PICK_AND_PLACE, manipulation_query_t::PRX_GRASP_GREEDY, move_object, 1, retract_config, initial_state, initial_state, NULL, initial_object, target_object );
                manipulation_query = dynamic_cast< manipulation_query_t* >( output_queries[ manipulation_task_planner_name ] );
                
                //A simple pick
                //manipulation_query->setup_pick( manipulation_context_name, true, GRASP_GREEDY, move_object, 1, retract_config, initial_state, initial_object );

                //Let's do a pick and place
                // manipulation_query->setup_pick_and_place( manipulation_context_name, true, GRASP_GREEDY, move_object, 1, retract_config, initial_state, initial_object, target_object );
               
                //Do a pick and move
                manipulation_query->setup_pick_and_move( manipulation_context_name, GRASP_GREEDY, move_object, 1, retract_config, initial_state, initial_state, initial_object );

                simple_pap_specification_t* pap_spec = dynamic_cast< simple_pap_specification_t* >(input_specification);
                manipulation_query->path_constraints = pap_spec->validity_checker->alloc_constraint();
                constraints_t* valid_constraints = pap_spec->validity_checker->alloc_constraint();

                //Need to push in an index for each of the movable bodies in the world?
                object_collision_constraints_t* occ = dynamic_cast< object_collision_constraints_t* >( valid_constraints );
                if( occ != NULL )
                {
                    PRX_PRINT("PAP TP: Cast to Object Collision Constraints worked!", PRX_TEXT_GREEN);
                    for( unsigned i=0; i<objects.size(); ++i )
                    {
                        occ->constraints.insert( i );
                    }
                }
                else
                {
                    PRX_PRINT("PAP TP: Cast to Object Collision Constraints DID NOT WORK!", PRX_TEXT_RED);                    
                }
                manipulation_query->set_valid_constraints( valid_constraints );
                
                planners[manipulation_task_planner_name]->link_query(manipulation_query);
                planners[manipulation_task_planner_name]->resolve_query();

                PRX_PRINT( manipulation_model->get_full_state_space()->print_memory( 3 ), PRX_TEXT_GREEN );

                trajectory_t traj;
                traj.link_space(manipulation_model->get_state_space());         
                manipulation_model->propagate_plan(initial_state,  manipulation_query->plan, traj);
                PRX_DEBUG_COLOR("New object state: " << object_space->print_memory(4), PRX_TEXT_CYAN);

                //There is a second query here... commenting to reduce prints and better understand what is going on.
                
                // object_space->copy_from_point(target_object);
                // manipulation_query_t* query2 = new manipulation_query_t(manipulation_context_name, manipulation_query_t::PRX_PICK_AND_PLACE, manipulation_query_t::PRX_GRASP_GREEDY, move_object, 1, retract_config, initial_state, initial_state, NULL, target_object, initial_object );
                // query2->path_constraints = new object_collision_constraints_t();

                // query2->setup_astar( PRX_MCR_EXACT, valid_constraints );

                // planners[manipulation_task_planner_name]->link_query(query2);
                // planners[manipulation_task_planner_name]->resolve_query();

                
                //REPORT the final plan
                in_query->link_spaces(manipulator->get_state_space(), manipulator->get_control_space());
                manipulation_model->convert_plan(in_query->plan, manipulator->get_control_space(), manipulation_query->plan, manipulation_model->get_control_space());
                // manipulation_model->convert_plan(in_query->plan, manipulator->get_control_space(), query2->plan, manipulation_model->get_control_space());

                manipulation_model->use_context(full_manipulator_context_name);
                
                delete valid_constraints;
            }
        }
    }
}

