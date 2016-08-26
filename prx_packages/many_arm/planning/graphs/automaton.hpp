/**
 * @file automaton.hpp
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
#pragma once

#ifndef PRX_MA_AUTOMATON_HPP
#define PRX_MA_AUTOMATON_HPP

#include "planning/graphs/search_tree.hpp"

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"

namespace prx
{
    namespace util
    {
        class space_point_t;
    }

    namespace packages
    {
        namespace multi_arm
        {
            /**
             *
             */
            struct pose_record_t
            {
                util::space_point_t* object_pose; //The actual pose attained by the object
                unsigned pose_id; //An identifier which indicates pose equivalence between records
                bool discovered; //A flag useful while searching to determine if a node has been discovered by search
                search_node_t* search_node; //A pointer to the search node which reaches this record

                std::vector< unsigned > manipulator_indices;
                std::vector< unsigned > grasp_ids;
                std::vector< util::space_point_t* > states;
                std::vector< util::space_point_t* > released;
                std::vector< util::space_point_t* > retracted;
                std::vector< sim::plan_t* > retraction_plans;
                std::vector< sim::plan_t* > approach_plans;
            };

            // =======================================================
            // =====..====..====....====......====........============
            // =====...===..===..==..===..===...==..==================
            // =====..=.==..==..====..==..====..==..==================
            // =====..==.=..==..====..==..====..==......==============
            // =====..===...==..====..==..====..==..==================
            // =====..====..===..==..===..===...==..==================
            // =====..====..====....====......====........============
            // =======================================================

            /**
             *
             */
            class automaton_node_t : public util::undirected_node_t
            {
              public:
                automaton_node_t()
                {
                    num_records = 0;
                    max_records = 30;
                    poses.resize(max_records);
                }

                ~automaton_node_t()
                {
                    //We have to assume that these nodes own their own memory, so have to free it... o_O
                    for( unsigned i=0; i<num_records; ++i )
                    {
                        unsigned manip_index;
                        for( unsigned j=0; j<poses[i].manipulator_indices.size(); ++j )
                        {
                            object_space->free_point(poses[i].object_pose);
                            manip_index = poses[i].manipulator_indices[j];
                            transfer_spaces[manip_index]->free_point( poses[i].states[j] );
                            move_spaces[manip_index]->free_point( poses[i].released[j] );
                            move_spaces[manip_index]->free_point( poses[i].retracted[j] );
                            delete poses[i].retraction_plans[j];
                            delete poses[i].approach_plans[j];
                        }
                    }
                }

                unsigned add_record( util::space_point_t* object_pose, unsigned new_id )
                {
                    //Increment the number of records
                    ++num_records;
                    // PRX_DEBUG_COLOR("Node [" << node_id << "] added a record: " << num_records-1, PRX_TEXT_CYAN);
                    //Then, if the number of records has exceeded our maximum
                    if(num_records >= max_records)
                    {
                        //Resize the vector
                        max_records *= 2;
                        poses.resize(max_records);
                    }
                    //Finally, need to copy in the pose information we have been given
                    poses[num_records-1].object_pose = object_pose;
                    poses[num_records-1].pose_id = new_id;
                    poses[num_records-1].discovered = false;
                    poses[num_records-1].search_node = NULL;
                    //Return the index for this created recrord
                    return num_records-1;
                }

                void add_grasp_to_record( unsigned record, unsigned manip_index, unsigned new_id, util::space_point_t* state, util::space_point_t* released, util::space_point_t* retracted, sim::plan_t* retract_plan, sim::plan_t* approach_plan )
                {
                    PRX_ASSERT(record < num_records);
                    poses[record].manipulator_indices.push_back( manip_index );
                    poses[record].grasp_ids.push_back( new_id );
                    poses[record].states.push_back( state );
                    poses[record].released.push_back( released );
                    poses[record].retracted.push_back( retracted );
                    poses[record].approach_plans.push_back( approach_plan );
                    poses[record].retraction_plans.push_back( retract_plan );
                }

                void discover_record( unsigned record )
                {
                    PRX_ASSERT(record < num_records);
                    poses[record].discovered = true;
                }

                virtual void serialize(std::ofstream& output_stream, const util::space_t* point_space)
                {
                    // PRX_DEBUG_COLOR("Serializing node: " << node_id, PRX_TEXT_GREEN);
                    //Begin by outputting the node id and some sample bounds
                    output_stream << node_id << " " << sample_bounds[0].get_lower_bound() << " " << sample_bounds[0].get_upper_bound() << " " << sample_bounds[1].get_lower_bound() << " " << sample_bounds[1].get_upper_bound() << " " << sample_bounds[2].get_lower_bound() << " " << sample_bounds[2].get_upper_bound() << std::endl;

                    // PRX_DEBUG_COLOR("# of arms: " << arms.size(), PRX_TEXT_BLUE);
                    //Then, output the arm sets for this node
                    output_stream << arms.size() << std::endl;
                    for( unsigned i=0; i<arms.size(); ++i )
                    {
                        output_stream << arms[i].size() << std::endl;
                        for( unsigned j=0; j<arms[i].size(); ++j )
                        {
                            output_stream << arms[i][j] << " ";
                        }
                        output_stream << std::endl;
                    }

                    // PRX_DEBUG_COLOR("Now outputting Records: " << num_records, PRX_TEXT_MAGENTA);
                    //Then, output the pose records for this node
                    output_stream << num_records << std::endl;
                    for( unsigned i=0; i<num_records; ++i )
                    {
                        // PRX_DEBUG_COLOR("Record: " << i, PRX_TEXT_BLUE);
                        const pose_record_t& record = poses[i];
                        //Print out the object grasp point
                        output_stream << object_space->serialize_point( record.object_pose ) << " " << record.pose_id << std::endl;
                        //Now, print out how many grasps are in this thing
                        output_stream << record.manipulator_indices.size() << std::endl;
                        for( unsigned j=0; j<record.manipulator_indices.size(); ++j )
                        {
                            unsigned manip_index = record.manipulator_indices[j];
                            // PRX_DEBUG_COLOR("Record for: " << manip_index, PRX_TEXT_CYAN);
                            //Thank god we have those spaces, eh?
                            output_stream << manip_index << " " << record.grasp_ids[j] << std::endl;
                            //Serialize the grasped states
                            output_stream << transfer_spaces[manip_index]->serialize_point( record.states[j] ) << std::endl;
                            //Serialize the ungrasped states
                            output_stream << move_spaces[manip_index]->serialize_point( record.released[j] ) << std::endl;
                            output_stream << move_spaces[manip_index]->serialize_point( record.retracted[j] ) << std::endl;
                            // PRX_DEBUG_COLOR("Output Approach plan of size: " << record.approach_plans[j]->size(), PRX_TEXT_LIGHTGRAY);
                            //Serialize the approach plan
                            record.approach_plans[j]->save_to_stream( output_stream );
                            // PRX_DEBUG_COLOR("Output Retract plan of size: " << record.retraction_plans[j]->size(), PRX_TEXT_LIGHTGRAY);
                            //Serialize the retract plan
                            record.retraction_plans[j]->save_to_stream( output_stream );
                        }
                    }
                }

                virtual bool checked_deserialize(std::ifstream& input_stream)
                {
                    unsigned utemp_a;
                    unsigned utemp_b;
                    double dtemp_a;
                    double dtemp_b;

                    //Read in the node id
                    input_stream >> node_id;
                    sample_bounds.resize(3);
                    //Read in the bounds
                    input_stream >> dtemp_a >> dtemp_b;
                    sample_bounds[0].set_bounds( dtemp_a, dtemp_b );
                    input_stream >> dtemp_a >> dtemp_b;
                    sample_bounds[1].set_bounds( dtemp_a, dtemp_b );
                    input_stream >> dtemp_a >> dtemp_b;
                    sample_bounds[2].set_bounds( dtemp_a, dtemp_b );
                    // PRX_DEBUG_COLOR("Deserializing Node: " << node_id, PRX_TEXT_BROWN);

                    //Figure out how many arms we have here
                    input_stream >> utemp_a;
                    arms.resize( utemp_a );
                    // PRX_DEBUG_COLOR("With " << arms.size() << " arms.", PRX_TEXT_LIGHTGRAY);
                    //Now, read in all the arms
                    for( unsigned i=0; i<arms.size(); ++i )
                    {
                        //First, figure out how many physical arms are in this set.
                        input_stream >> utemp_a;
                        arms[i].resize( utemp_a );
                        //Then, read in each arm index
                        for( unsigned j=0; j<arms[i].size(); ++j )
                        {
                            input_stream >> arms[i][j];
                        }
                    }

                    //Then, read in how many records are on this node
                    input_stream >> num_records;
                    //Check if the number of records has exceeded our maximum
                    if( num_records > max_records )
                    {
                        max_records = num_records;
                        poses.resize( max_records );
                    }
                    // PRX_DEBUG_COLOR("Node has records: " << num_records << "  and is using object space: " << object_space, PRX_TEXT_MAGENTA);
                    //Then, let's read the records
                    for( unsigned i=0; i<num_records; ++i )
                    {
                        // PRX_DEBUG_COLOR(" = = = Reading in Record [" << i << "]", PRX_TEXT_BLUE);
                        //Grab the record
                        pose_record_t& record = poses[i];
                        //Then read in the object pose associated with this record.
                        record.object_pose = object_space->deserialize_point( input_stream );
                        //As well as the pose's id
                        input_stream >> record.pose_id;

                        //DEBUG: make sure each record has a reasonable object pose
                        // PRX_DEBUG_COLOR("[" << i << "]: " << record.object_pose, PRX_TEXT_LIGHTGRAY);

                        //Then, figure out how many grasps are in this record
                        unsigned nr_grasps;
                        input_stream >> nr_grasps;

                        // PRX_DEBUG_COLOR("Record: " << i << " with " << nr_grasps << " grasps.", PRX_TEXT_BLUE);
                        //Read in each one
                        for( unsigned j=0; j<nr_grasps; ++j )
                        {
                            // PRX_DEBUG_COLOR("Grasp: " << j, PRX_TEXT_CYAN);
                            //First, get the manipulator index, then the grasp id
                            input_stream >> utemp_a >> utemp_b;
                            //Then, read the grasped state
                            util::space_point_t* gr_p = transfer_spaces[utemp_a]->deserialize_point( input_stream );
                            // PRX_DEBUG_COLOR("Grasped: " << transfer_spaces[utemp_a]->print_point( gr_p, 4 ), PRX_TEXT_GREEN );
                            //Then, read the released and then retracted points
                            util::space_point_t* rl_p = move_spaces[utemp_a]->deserialize_point( input_stream );
                            // PRX_DEBUG_COLOR("Release: " << move_spaces[utemp_a]->print_point( rl_p, 4 ), PRX_TEXT_BROWN );
                            util::space_point_t* rt_p = move_spaces[utemp_a]->deserialize_point( input_stream );
                            // PRX_DEBUG_COLOR("Retract: " << move_spaces[utemp_a]->print_point( rt_p, 4 ), PRX_TEXT_LIGHTGRAY );

                            //Allocate the two new plans
                            sim::plan_t* app_p = new sim::plan_t( control_space );
                            sim::plan_t* ret_p = new sim::plan_t( control_space );

                            //Then, deserialize the plans
                            app_p->read_from_stream( input_stream );
                            // PRX_DEBUG_COLOR("Read approach of size: " << app_p->size(), PRX_TEXT_LIGHTGRAY);
                            ret_p->read_from_stream( input_stream );
                            // PRX_DEBUG_COLOR("Read retraction of size: " << ret_p->size(), PRX_TEXT_LIGHTGRAY);

                            if( app_p->size() == 0 || ret_p->size() == 0 )
                                return false;

                            // PRX_DEBUG_COLOR("Approach begin:" << control_space->print_point( app_p->at(0).control, 4 ), PRX_TEXT_CYAN );
                            // PRX_DEBUG_COLOR("Approach end  :" << control_space->print_point( app_p->back().control, 4 ), PRX_TEXT_LIGHTGRAY );
                            // PRX_DEBUG_COLOR("Retract begin:" << control_space->print_point( ret_p->at(0).control, 4 ), PRX_TEXT_RED );
                            // PRX_DEBUG_COLOR("Retract end  :" << control_space->print_point( ret_p->back().control, 4 ) << "\n", PRX_TEXT_LIGHTGRAY );

                            //Now that we have everything we need, we can add all this data to the record
                            add_grasp_to_record( i, utemp_a, utemp_b, gr_p, rl_p, rt_p, ret_p, app_p );
                        }
                    }
                    return true;
                }

                const std::vector< pose_record_t >& get_records()
                {
                    return poses;
                }

                unsigned get_num_records()
                {
                    return num_records;
                }

                void link_spaces( const std::vector< util::space_t* >& movers, const std::vector< util::space_t* >& shakers, util::space_t* the_hero, util::space_t* the_plan = NULL )
                {
                    move_spaces = movers;
                    transfer_spaces = shakers;
                    object_space = the_hero;
                    control_space = the_plan;
                }

                void set_bounds( const std::vector< util::bounds_t >& in_bounds )
                {
                    PRX_ASSERT(in_bounds.size() == 3);
                    sample_bounds = in_bounds;
                }

                const std::vector< util::bounds_t >& get_bounds()
                {
                    return sample_bounds;
                }

                void add_arm_set( const std::vector< unsigned >& new_arms )
                {
                    arms.push_back( new_arms );
                }

                const std::vector< std::vector<unsigned> >& get_arms()
                {
                    return arms;
                }

              protected:
                unsigned num_records; //Number of records put into this node at current
                unsigned max_records; //Maximum number of allowed records in this state

                std::vector< pose_record_t > poses; //List of pose records
                util::space_t* object_space; //The space of the object.
                util::space_t* control_space; //The movement control space we can use for deserialization of the automaton
                std::vector< util::space_t* > move_spaces; //All of the movement spaces in this problem
                std::vector< util::space_t* > transfer_spaces; //All of the transfer spaces in this problem

                std::vector< std::vector< unsigned > > arms; //All arm sets which are involved in this state

                std::vector< util::bounds_t > sample_bounds; //Bounds in which poses should be sampled for this state
            };

            // =======================================================
            // =====........==......======....====........============
            // =====..========..===...==...==...==..==================
            // =====..========..====..==..====..==..==================
            // =====......====..====..==..========......==============
            // =====..========..====..==..==....==..==================
            // =====..========..===...==...==...==..==================
            // =====........==......======....=.==........============
            // =======================================================

            /**
             *
             */
            class automaton_edge_t : public util::undirected_edge_t
            {
              public:
                automaton_edge_t()
                {
                    transition_weight = 0;
                }

                ~automaton_edge_t()
                {
                    clear_records();
                }

                void clear_records()
                {
                    //We have to assume that these nodes own their own memory, so have to free it... o_O
                    for( unsigned i=0; i<poses.size(); ++i )
                    {
                        unsigned manip_index;
                        object_space->free_point(poses[i].object_pose);
                        for( unsigned j=0; j<poses[i].manipulator_indices.size(); ++j )
                        {
                            manip_index = poses[i].manipulator_indices[j];
                            transfer_spaces[manip_index]->free_point( poses[i].states[j] );
                            move_spaces[manip_index]->free_point( poses[i].released[j] );
                            move_spaces[manip_index]->free_point( poses[i].retracted[j] );
                            delete poses[i].retraction_plans[j];
                            delete poses[i].approach_plans[j];
                        }
                    }
                    //And report no poses here.
                    poses.resize(0);
                }

                void clear_records_no_free()
                {
                    poses.resize(0);
                }

                unsigned add_record( util::space_point_t* object_pose, unsigned new_id )
                {
                    //Embiggen the record list
                    poses.resize( poses.size() + 1 );
                    //Finally, need to copy in the pose information we have been given
                    poses.back().object_pose = object_pose;
                    poses.back().pose_id = new_id;
                    poses.back().discovered = false;
                    poses.back().search_node = NULL;
                    //Return the index for this created recrord
                    return poses.size()-1;
                }

                void add_grasp_to_record( unsigned record, unsigned manip_index, unsigned new_id, util::space_point_t* state, util::space_point_t* released, util::space_point_t* retracted, sim::plan_t* retract_plan, sim::plan_t* approach_plan )
                {
                    poses[record].manipulator_indices.push_back( manip_index );
                    poses[record].grasp_ids.push_back( new_id );
                    poses[record].states.push_back( state );
                    poses[record].released.push_back( released );
                    poses[record].retracted.push_back( retracted );
                    poses[record].approach_plans.push_back( approach_plan );
                    poses[record].retraction_plans.push_back( retract_plan );
                }

                virtual void serialize(std::ofstream& output_stream)
                {
                    // PRX_DEBUG_COLOR("Serializing edge: " << source_vertex << " <> " << target_vertex, PRX_TEXT_GREEN);
                    output_stream << source_vertex << " " << target_vertex << " ";
                    if( transition_weight < PRX_ZERO_CHECK )
                        output_stream << 0 << std::endl;
                    else
                        output_stream << transition_weight << std::endl;

                    if(sample_bounds.size() != 0)
                    {
                        output_stream << sample_bounds[0].get_lower_bound() << " " << sample_bounds[0].get_upper_bound() << " " << sample_bounds[1].get_lower_bound() << " " << sample_bounds[1].get_upper_bound() << " " << sample_bounds[2].get_lower_bound() << " " << sample_bounds[2].get_upper_bound() << std::endl;
                    }
                    else
                    {
                        output_stream << "0.0 0.0 0.0 0.0 0.0 0.0" << std::endl;
                    }

                    // PRX_DEBUG_COLOR("Now outputting Records: " << poses.size(), PRX_TEXT_MAGENTA);
                    //Then, output the pose records for this node
                    output_stream << poses.size() << std::endl;
                    for( unsigned i=0; i<poses.size(); ++i )
                    {
                        // PRX_DEBUG_COLOR("Record: " << i, PRX_TEXT_BLUE);
                        const pose_record_t& record = poses[i];
                        //Print out the object grasp point
                        output_stream << object_space->serialize_point( record.object_pose ) << " " << record.pose_id << std::endl;
                        //Now, print out how many grasps are in this thing
                        output_stream << record.manipulator_indices.size() << std::endl;
                        for( unsigned j=0; j<record.manipulator_indices.size(); ++j )
                        {
                            unsigned manip_index = record.manipulator_indices[j];
                            // PRX_DEBUG_COLOR("Record for: " << manip_index, PRX_TEXT_CYAN);
                            //Well crap, we need ALL the spaces it seems...
                            output_stream << manip_index << " " << record.grasp_ids[j] << std::endl;
                            //Serialize the grasped states
                            output_stream << transfer_spaces[manip_index]->serialize_point( record.states[j] ) << std::endl;
                            //Serialize the ungrasped states
                            output_stream << move_spaces[manip_index]->serialize_point( record.released[j] ) << std::endl;
                            output_stream << move_spaces[manip_index]->serialize_point( record.retracted[j] ) << std::endl;
                            // PRX_DEBUG_COLOR("Output Approach plan of size: " << record.approach_plans[j]->size(), PRX_TEXT_LIGHTGRAY);
                            //Serialize the approach plan
                            record.approach_plans[j]->save_to_stream( output_stream );
                            // PRX_DEBUG_COLOR("Output Retract plan of size: " << record.retraction_plans[j]->size(), PRX_TEXT_LIGHTGRAY);
                            //Serialize the retract plan
                            record.retraction_plans[j]->save_to_stream( output_stream );
                        }
                    }

                }

                virtual bool checked_deserialize( std::ifstream& input_stream )
                {
                    unsigned utemp_a;
                    unsigned utemp_b;
                    double dtemp_a;
                    double dtemp_b;
                    // PRX_DEBUG_COLOR("Deserializing an edge: ", PRX_TEXT_RED);

                    //Absolute first thing, read in the weight
                    input_stream >> transition_weight;
                    // PRX_DEBUG_COLOR("Weight: " << transition_weight, PRX_TEXT_LIGHTGRAY);

                    sample_bounds.resize(3);
                    //Read in the bounds
                    input_stream >> dtemp_a >> dtemp_b;
                    sample_bounds[0].set_bounds( dtemp_a, dtemp_b );
                    // PRX_DEBUG_COLOR("Bounds: " << dtemp_a << " " << dtemp_b, PRX_TEXT_LIGHTGRAY);
                    input_stream >> dtemp_a >> dtemp_b;
                    sample_bounds[1].set_bounds( dtemp_a, dtemp_b );
                    // PRX_DEBUG_COLOR("Bounds: " << dtemp_a << " " << dtemp_b, PRX_TEXT_LIGHTGRAY);
                    input_stream >> dtemp_a >> dtemp_b;
                    sample_bounds[2].set_bounds( dtemp_a, dtemp_b );
                    // PRX_DEBUG_COLOR("Bounds: " << dtemp_a << " " << dtemp_b, PRX_TEXT_LIGHTGRAY);

                    //Then, read in how many records are on this node
                    input_stream >> utemp_a;
                    //Check if the number of records has exceeded our maximum
                    poses.resize( utemp_a );
                    // PRX_DEBUG_COLOR("Edge has: " << utemp_a << " records.", PRX_TEXT_RED);
                    //Then, let's read the records
                    for( unsigned i=0; i<poses.size(); ++i )
                    {
                        //Grab the record
                        pose_record_t& record = poses[i];
                        //Then read in the object pose associated with this record.
                        record.object_pose = object_space->deserialize_point( input_stream );
                        //As well as the pose's id
                        input_stream >> record.pose_id;

                        //Then, figure out how many grasps are in this record
                        unsigned nr_grasps;
                        input_stream >> nr_grasps;
                        // PRX_DEBUG_COLOR("[" << i << "]: " << record.object_pose, PRX_TEXT_LIGHTGRAY);
                        //Read in each one
                        for( unsigned j=0; j<nr_grasps; ++j )
                        {
                            // PRX_DEBUG_COLOR("Grasp: " << j, PRX_TEXT_CYAN);
                            //First, get the manipulator index, then the grasp id
                            input_stream >> utemp_a >> utemp_b;
                            //Then, read the grasped state
                            util::space_point_t* gr_p = transfer_spaces[utemp_a]->deserialize_point( input_stream );
                            //Then, read the released and then retracted points
                            util::space_point_t* rl_p = move_spaces[utemp_a]->deserialize_point( input_stream );
                            util::space_point_t* rt_p = move_spaces[utemp_a]->deserialize_point( input_stream );

                            //Allocate the two new plans
                            sim::plan_t* app_p = new sim::plan_t( control_space );
                            sim::plan_t* ret_p = new sim::plan_t( control_space );

                            //Then, deserialize the plans
                            app_p->read_from_stream( input_stream );
                            // PRX_DEBUG_COLOR("Read approach of size: " << app_p->size(), PRX_TEXT_LIGHTGRAY);
                            ret_p->read_from_stream( input_stream );
                            // PRX_DEBUG_COLOR("Read retraction of size: " << ret_p->size(), PRX_TEXT_LIGHTGRAY);

                            if( app_p->size() == 0 || ret_p->size() == 0 )
                                return false;

                            //Now that we have everything we need, we can add all this data to the record
                            add_grasp_to_record( i, utemp_a, utemp_b, gr_p, rl_p, rt_p, ret_p, app_p );
                        }
                    }
                    return true;
                }

                const std::vector< pose_record_t >& get_records()
                {
                    return poses;
                }

                unsigned get_num_records()
                {
                    return poses.size();
                }

                void link_spaces( const std::vector< util::space_t* >& movers, const std::vector< util::space_t* >& shakers, util::space_t* the_hero, util::space_t* the_plan = NULL )
                {
                    move_spaces = movers;
                    transfer_spaces = shakers;
                    object_space = the_hero;
                    control_space = the_plan;
                }

                void set_bounds( const std::vector< util::bounds_t >& in_bounds )
                {
                    PRX_ASSERT(in_bounds.size() == 3);
                    sample_bounds = in_bounds;
                }

                const std::vector< util::bounds_t >& get_bounds()
                {
                    return sample_bounds;
                }

                void set_weight( double weight )
                {
                    transition_weight = weight;
                }

                double get_weight()
                {
                    return transition_weight;
                }

              protected:

                //Members
                std::vector< util::bounds_t > sample_bounds; //Sample bounds for states along this edge.
                std::vector< pose_record_t > poses; //An array of poses associated with this edge

                double transition_weight; //Storing the transition weight

                util::space_t* object_space; //Also need the object space to free stuff in the records
                util::space_t* control_space; //The movement control space we can use for deserialization of the automaton
                std::vector< util::space_t* > move_spaces; //All of the movement spaces in this problem
                std::vector< util::space_t* > transfer_spaces; //All of the transfer spaces in this problem

            };

            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================
            // =======================================================


            class automaton_graph_t : public util::undirected_graph_t
            {
              public:
                automaton_graph_t()
                {

                }

                ~automaton_graph_t()
                {

                }

                template<class node_type, class edge_type>
                bool deserialize(const std::string& filename, std::ifstream& input_stream, const std::vector< util::space_t* >& move_spaces, const std::vector< util::space_t* >& transfer_spaces, util::space_t* object_space, util::space_t* control_space )
                {
                    input_stream.open(filename.c_str());
                    if( input_stream.good() )
                    {
                        int num_edges, num_vertices;
                        input_stream >> num_vertices;
                        // PRX_DEBUG_COLOR("Reading " << num_vertices << " vertices!", PRX_TEXT_GREEN);
                        std::vector< util::undirected_vertex_index_t> node_map(num_vertices);
                        for( int i = 0; i < num_vertices; i++ )
                        {
                            util::undirected_vertex_index_t new_vertex = add_vertex<node_type > ();
                            //Alright, so we actually need this to be of the node type
                            node_type* node = get_vertex_as< node_type >( new_vertex );
                            node->index = new_vertex;
                            node->link_spaces( move_spaces, transfer_spaces, object_space, control_space );
                            if( !node->checked_deserialize(input_stream) )
                                return false;
                            node_map[graph[new_vertex].node->node_id] = new_vertex;
                        }
                        input_stream >> num_edges;
                        // PRX_DEBUG_COLOR("Reading " << num_edges << " edges!", PRX_TEXT_RED);
                        for( int i = 0; i < num_edges; i++ )
                        {
                            int from, to;
                            input_stream >> from >> to;
                            // PRX_DEBUG_COLOR("Create edge: " << from << " - " << to, PRX_TEXT_RED);
                            util::undirected_edge_index_t new_edge = add_edge< edge_type > (node_map[from], node_map[to]);
                            edge_type* edge = get_edge_as< edge_type >( new_edge );
                            edge->link_spaces( move_spaces, transfer_spaces, object_space, control_space );
                            if( !edge->checked_deserialize(input_stream) )
                                return false;
                        }
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            };
        }
    }
}

#endif //PRX_MA_AUTOMATON_HPP


