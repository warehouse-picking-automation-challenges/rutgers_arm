/**
 * @file IK_database.cpp 
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

#include "planning/modules/IK_database.hpp"

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {
        namespace manipulation
        {
            IK_seed_database_t::IK_seed_database_t()
            {
                for( int i = 0; i < 7; ++i )
                    effector_space_memory.push_back(new double());
                effector_space = new space_t("SE3", effector_space_memory);
                effector_point = effector_space->alloc_point();
                effector_vec.resize(effector_space_memory.size());
                left_metric = NULL;
                right_metric = NULL;
            }

            IK_seed_database_t::~IK_seed_database_t() 
            {
                clear();
                
                effector_space->free_point(effector_point);
                delete effector_space;
                if( left_metric != NULL )
                    delete left_metric;
                if( right_metric != NULL )
                    delete right_metric;
            }

            void IK_seed_database_t::clear()
            {
                if( left_metric != NULL )
                    left_metric->clear();
                if( right_metric != NULL )
                    right_metric->clear();
                for( unsigned i=0; i<all_pairs.size(); ++i )
                {
                    delete all_pairs[i];
                }
                for( unsigned i=0; i<left_pairs.size(); ++i )
                {
                    delete left_pairs[i];
                }
                for( unsigned i=0; i<right_pairs.size(); ++i )
                {
                    delete right_pairs[i];
                }
                all_pairs.resize(0);
                left_pairs.resize(0);
                right_pairs.resize(0);
            }

            void IK_seed_database_t::init(const parameter_reader_t* reader)
            {
                PRX_DEBUG_COLOR("IK_data_base init().", PRX_TEXT_CYAN);
                if(reader->has_attribute("distance_metric"))
                {
                    left_metric = reader->initialize_from_loader<distance_metric_t>("distance_metric","prx_utilities");
                    right_metric = reader->initialize_from_loader<distance_metric_t>("distance_metric","prx_utilities");
                }
                else
                {
                    PRX_FATAL_S("Missing distance_metric attribute for IK_data_base!");
                }
                left_metric->link_space(effector_space);
                right_metric->link_space(effector_space);
            }

            void IK_seed_database_t::add_pair(const space_t* manip_space, const config_t& left_effector_config, const config_t& right_effector_config, state_t* state) 
            {
                IK_set_t* ik_set = new IK_set_t();
                ik_set->manip_state = manip_space->clone_point(state);
                ik_set->left_end_effector = left_effector_config;
                ik_set->right_end_effector = right_effector_config;

                //create a node for the left
                IK_metric_pair_t* left_node = new IK_metric_pair_t();
                left_node->manip_state = manip_space->clone_point(state);
                left_node->point = effector_space->alloc_point();
                effector_config_to_state(left_effector_config,left_node->point);
                left_metric->add_point(left_node);

                //create a node for the right
                IK_metric_pair_t* right_node = new IK_metric_pair_t();
                right_node->manip_state = manip_space->clone_point(state);
                right_node->point = effector_space->alloc_point();
                effector_config_to_state(right_effector_config,right_node->point);
                right_metric->add_point(right_node);


                all_pairs.push_back(ik_set);
            }

            void IK_seed_database_t::get_near_neighbors( std::vector< state_t* >& states, const config_t& target_config, unsigned number, bool left )
            {
                effector_config_to_state(target_config, effector_point);
                std::vector< const abstract_node_t* > nodes;

                if(left)
                    nodes = left_metric->multi_query( effector_point, number );
                else
                    nodes = right_metric->multi_query(effector_point,number);

                foreach(const abstract_node_t* node, nodes)
                {
                    states.push_back(dynamic_cast<const IK_metric_pair_t *>(node)->manip_state);
                }
            }

            bool IK_seed_database_t::has_data()
            {
                return left_metric->get_nr_points() > 0 && right_metric->get_nr_points() > 0 ;
            }

            void IK_seed_database_t::serialize(std::ofstream& output_stream, const space_t* manip_space, unsigned precision) 
            {
                output_stream << all_pairs.size() << std::endl;
                foreach(IK_set_t* pair, all_pairs)
                {
                    pair->serialize(output_stream, manip_space, precision);
                }
                
            }

            void IK_seed_database_t::deserialize(std::ifstream& input_stream, const space_t* manip_space)  
            {
                int num;
                input_stream >> num;
                
                
                for(int i = 0; i < num; ++i)
                {
                    IK_set_t* pair = new IK_set_t();
                    pair->deserialize(input_stream, manip_space);

                    //create a node for the left
                    IK_metric_pair_t* left_node = new IK_metric_pair_t();
                    left_node->manip_state = manip_space->clone_point(pair->manip_state);
                    left_node->point = effector_space->alloc_point();
                    effector_config_to_state(pair->left_end_effector,left_node->point);
                    left_metric->add_point(left_node);

                    //create a node for the right
                    IK_metric_pair_t* right_node = new IK_metric_pair_t();
                    right_node->manip_state = manip_space->clone_point(pair->manip_state);
                    right_node->point = effector_space->alloc_point();
                    effector_config_to_state(pair->left_end_effector,right_node->point);
                    right_metric->add_point(right_node);

                    all_pairs.push_back(pair);
                    PRX_STATUS("IK Database Loading ["<< i+1<<"/"<< num << "] points...", PRX_TEXT_GREEN);
                    // PRX_PRINT(pair->print(manip_space),PRX_TEXT_CYAN);
                }
                std::cout << std::endl;

            }

            void IK_seed_database_t::effector_config_to_state(const config_t& config, state_t* state)
            {
                config.get_position(effector_vec[0], effector_vec[1], effector_vec[2]);
                quaternion_t q = config.get_orientation();
                effector_vec[3] = q.get_x();
                effector_vec[4] = q.get_y();
                effector_vec[5] = q.get_z();
                effector_vec[6] = q.get_w();
                effector_space->set_from_vector(effector_vec, state);
            }
        }
    }
}
