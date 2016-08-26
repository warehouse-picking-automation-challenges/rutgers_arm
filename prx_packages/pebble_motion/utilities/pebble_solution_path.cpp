/**
 * @file pebble_solution_path.cpp
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

#include "utilities/pebble_solution_path.hpp"
#include "pebble_assignment.hpp"

namespace prx
{
    using namespace util;

    namespace packages
    {
        namespace pebble_motion
        {

            pebble_solution_path_t::pebble_solution_path_t(int initial_size)
            {
                solution.resize(initial_size);
                max_size = initial_size;
                sz = 0;
            }

            pebble_solution_path_t::~pebble_solution_path_t() { }

            void pebble_solution_path_t::push_back(int pebble_id, undirected_vertex_index_t v_from, undirected_vertex_index_t v_to)
            {
                solution[sz].set(pebble_id, v_from, v_to);
                ++sz;
                if( sz >= max_size )
                {
                    max_size = max_size * 2;
                    solution.resize(max_size);
                }
            }

            void pebble_solution_path_t::push_back(int pebble_id, std::deque<undirected_vertex_index_t>& path, int path_length)
            {
                for( int i = 0; i < path_length - 1; ++i )
                    push_back(pebble_id, path[i], path[i + 1]);
            }

            void pebble_solution_path_t::push_back(int pebble_id, std::deque<undirected_vertex_index_t>& path, int begin, int end)
            {
                int direction = PRX_SIGN(end - begin);
                int i = begin;
                while( i != end )
                {
                    push_back(pebble_id, path[i], path[i + direction]);
                    i += direction;
                }
            }

            void pebble_solution_path_t::revert(int step_index)
            {
                sz = step_index;
            }

            void pebble_solution_path_t::reverse_path(pebble_assignment_t& assign, int start, int end, int pebble1, int pebble2)
            {
                PRX_INFO_S("Going to reverse : " << start << " - " << end << "  total size: " << sz);
                for( int i = end; i >= start; --i )
                {
                    int id = solution[i].pebble_id;

                    if( id == pebble1 )
                        id = pebble2;
                    else if( id == pebble2 )
                        id = pebble1;

                    push_back(id, solution[i].to(), solution[i].from());
                    PRX_INFO_S("REVERSE Pebble : " << id << "  " << solution[i].to() << " -> " << solution[i].from());
                    assign.change_position(id, solution[i].from());
                }
            }

            int pebble_solution_path_t::size()
            {
                return sz;
            }

            int pebble_solution_path_t::get_last_position()
            {
                if( sz == 0 )
                    return 0;
                return sz - 1;
            }

            std::string pebble_solution_path_t::print(const undirected_graph_t* graph, const space_t* state_space, int prec)
            {
                std::stringstream out(std::stringstream::out);

                int pebble_id;
                int i = 0;
                while( i < sz )
                {
                    pebble_id = solution[i].pebble_id;
                    out << std::endl << pebble_id << " : " << solution[i].from() << " -> ";
                    while( i < sz && pebble_id == solution[i].pebble_id )
                    {
                        out << state_space->print_point(graph->get_vertex_as<undirected_node_t > (solution[i].to())->point, prec) << " -> ";
                        ++i;
                    }
                }

                return out.str();
            }

            void pebble_solution_path_t::compress(std::vector<pebble_step_t>* final_solution, pebble_assignment_t assign)
            {
                int pebble_id;
                int i = 0;
                std::vector<undirected_vertex_index_t> path;

                while( i < sz )
                {
                    //        std::stringstream out(std::stringstream::out);
                    pebble_id = solution[i].pebble_id;
                    path.clear();
                    path.push_back(solution[i].from());
                    //        out << assign.get_position(pebble_id) << " -> ";
                    while( i < sz && pebble_id == solution[i].pebble_id )
                    {
                        path.push_back(solution[i].to());
                        //            out << solution[i].v << " -> ";
                        ++i;
                    }
                    final_solution->push_back(std::make_pair(pebble_id, path));
                    assign.change_position(pebble_id, path.back());
                }
            }
            
            void pebble_solution_path_t::convert(std::vector<pebble_step_t>* final_solution, pebble_assignment_t assign)
            {
                int pebble_id;
                int i = 0;
                std::vector<undirected_vertex_index_t> path(1);

                while( i < sz )
                {
                    pebble_id = solution[i].pebble_id;
                    path[0] = solution[i].to();                    
                    final_solution->push_back(std::make_pair(pebble_id, path));
                    assign.change_position(pebble_id, path[0]);
                    ++i;
                }
            }
        }
    }
}
