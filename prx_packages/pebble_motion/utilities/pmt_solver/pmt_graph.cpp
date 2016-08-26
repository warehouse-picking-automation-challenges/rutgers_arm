/**
 * @file pmt_graph.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2012, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Athanasios Krontiris, Andrew Dobson, Andrew Kimmel, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "utilities/pmt_solver/pmt_graph.hpp"

namespace prx
{
    using namespace util;
    
    namespace packages
    {
        namespace pebble_motion
        {

            pmt_graph_node_t::pmt_graph_node_t()
            {
                bfs_id = 0;
                obstacle_id = 0;
                avoid_id = 0;
                visited_id = 0;
                on_the_path_branch_id = 0;
                is_branch = false;
                index_on_path = 0;

            }

            pmt_graph_node_t::~pmt_graph_node_t() { }

//            void pmt_graph_node_t::serialize(std::ofstream& output_stream, const space_t* point_space)
//            {
//                abstract_node_t::serialize(output_stream,point_space);
//                output_stream << " " << id << std::endl;
//            }
//
//            void pmt_graph_node_t::deserialize(std::ifstream& input_stream, const space_t* point_space)
//            {
//                abstract_node_t::deserialize(input_stream,point_space);
//                input_stream >> id;
//            }
        }
    }
}
