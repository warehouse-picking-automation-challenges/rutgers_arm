/**
 * @file IK_database.hpp 
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

#ifndef PRX_IK_DATA_BASE_HPP
#define PRX_IK_DATA_BASE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/graph/abstract_node.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/simulation/state.hpp"

#include <boost/any.hpp>

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
        
            class IK_set_t
            {
              public:
                util::config_t left_end_effector;
                util::config_t right_end_effector;
                sim::state_t* manip_state;

                void serialize(std::ofstream& output_stream, const util::space_t* manip_space, unsigned precision = 8)
                {             
                    output_stream << manip_space->print_point(manip_state, precision) << std::endl;
                    double x,y,z,w;
                    left_end_effector.get_position(x,y,z);
                    output_stream << x << " " << y << " " << z << " ";
                    left_end_effector.get_orientation().get(x,y,z,w);
                    output_stream << x << " " << y << " " << z << " " << w << std::endl;

                    right_end_effector.get_position(x,y,z);
                    output_stream << x << " " << y << " " << z << " ";
                    right_end_effector.get_orientation().get(x,y,z,w);
                    output_stream << x << " " << y << " " << z << " " << w << std::endl;
                }
                
                void deserialize(std::ifstream& input_stream, const util::space_t* manip_space)
                {
                    char trash;
                    int num = manip_space->get_dimension();
                    std::vector<double> tmp_vec(num);
                    for(int i = 0; i < num; ++i)
                    {
                        input_stream >> tmp_vec[i];
                        if( i < num - 1 )
                            input_stream >> trash;
                    }
                    manip_state = manip_space->alloc_point();
                    manip_space->set_from_vector(tmp_vec,manip_state);

                    double x,y,z,w;
                    input_stream >> x >> y >> z;
                    left_end_effector.set_position(x,y,z);
                    input_stream >> x >> y >> z >> w;
                    left_end_effector.set_orientation(x,y,z,w);
                    left_end_effector.normalize_orientation();

                    input_stream >> x >> y >> z;
                    right_end_effector.set_position(x,y,z);
                    input_stream >> x >> y >> z >> w;
                    right_end_effector.set_orientation(x,y,z,w);
                    right_end_effector.normalize_orientation();
                }

                std::string print(const util::space_t* manip_space, int precision = 8)
                {
                    std::stringstream out(std::stringstream::out);
                    out << std::endl << left_end_effector.print() << std::endl << left_end_effector.print() << std::endl << manip_space->print_point(manip_state, precision);
                    return out.str();
                }

            };

            class IK_metric_pair_t : public util::abstract_node_t
            {
              public:
                sim::state_t* manip_state;
            };

            /**
             * @anchor IK_data_base
             *
             * 
             *
             * @brief <b> . </b>
             *
             * @author Athanasios Krontiris
             */
            class IK_seed_database_t
            {
              public:
                IK_seed_database_t();

                virtual ~IK_seed_database_t();
                
                void init(const util::parameter_reader_t* reader);
                
                void clear();
                
                void add_pair(const util::space_t* manip_space, const util::config_t& left_effector_config, const util::config_t& right_effector_config, sim::state_t* state);
                
                void get_near_neighbors( std::vector< sim::state_t* >& states, const util::config_t& target_config, unsigned number, bool left);

                bool has_data();

                void serialize(std::ofstream& output_stream, const util::space_t* manip_space, unsigned precision = 8);
                
                void deserialize(std::ifstream& input_stream, const util::space_t* manip_space);
                
              private: 

                void effector_config_to_state(const util::config_t& config, sim::state_t* state);

                util::space_t* effector_space;
                std::vector<double*> effector_space_memory;
                sim::state_t* effector_point;
                std::vector<double> effector_vec;
                
                util::distance_metric_t* left_metric;
                util::distance_metric_t* right_metric;
                std::vector<IK_set_t*> all_pairs;
                std::vector<IK_metric_pair_t*> left_pairs;
                std::vector<IK_metric_pair_t*> right_pairs;
            };
        }

    }
}

#endif
