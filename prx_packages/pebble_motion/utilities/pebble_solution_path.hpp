/**
 * @file pebble_solution_path.hpp
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
#pragma once

#ifndef PRX_PEBBLE_SOLUTION_PATH_HPP
#define	PRX_PEBBLE_SOLUTION_PATH_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"
#include "pebble_assignment.hpp"
#include <deque>

namespace prx
{
    namespace packages
    {
        namespace pebble_motion
        {

            typedef std::pair<int, std::vector<util::undirected_vertex_index_t> > pebble_step_t;

            struct pebble_solution_step_t
            {

                int pebble_id;
                std::pair<util::undirected_vertex_index_t, util::undirected_vertex_index_t> step;

                void set(int id, util::undirected_vertex_index_t v_start, util::undirected_vertex_index_t v_end)
                {
                    pebble_id = id;
                    step.first = v_start;
                    step.second = v_end;
                }

                util::undirected_vertex_index_t from()
                {
                    return step.first;
                }

                util::undirected_vertex_index_t to()
                {
                    return step.second;
                }
            };

            /**
             * A pre allocated data structure to store efficient the solution path for the pebble problems.
             */
            class pebble_solution_path_t
            {

              public:
                pebble_solution_path_t(int initial_size = 10000);
                virtual ~pebble_solution_path_t();

                void push_back(int pebble_id, util::undirected_vertex_index_t v_from, util::undirected_vertex_index_t v_to);

                /**
                 * Will push back all the vertices from the path up to the index path_length, without adding the 
                 * element from the path that sits on \c path[path_length].
                 * @param pebble_id The id of the pebble that is going to move.
                 * @param path The path that the pebble executed.
                 * @param path_length The end of the this path on the vector path.
                 */
                void push_back(int pebble_id, std::deque<util::undirected_vertex_index_t>& path, int path_length);

                /**
                 * Will push back all the vertices from the path from the start to the end indices. Both elements
                 * that sit on the \c path[begin] and \c path[end] will be added in the solution. Index end
                 * can be before index begin, which means that we need the elemets of the path in the opposite
                 * direction. 
                 * @param pebble_id The id of the pebble that is going to move.
                 * @param path The path that the pebble executed.
                 * @param begin From where on the path we will start to add elements.
                 * @param end Till where from the path elements will be added in the solution. Needs the last position
                 * of the path that has a valid number.
                 */
                void push_back(int pebble_id, std::deque<util::undirected_vertex_index_t>& path, int begin, int end);

                void revert(int step_index);

                void reverse_path(pebble_assignment_t& assign, int start, int end, int pebble1, int pebble2);

                int size();

                int get_last_position();

                std::string print(const util::undirected_graph_t* graph, const util::space_t* state_space, int prec = 2);

                void compress(std::vector<pebble_step_t>* final_solution, pebble_assignment_t assign);
                
                void convert(std::vector<pebble_step_t>* final_solution, pebble_assignment_t assign);

              protected:


              private:
                std::vector<pebble_solution_step_t> solution;

                int max_size;
                int sz;

            };

        }
    }
}

#endif	// PRX_PEBBLE_SOLUTION_PATH_HPP
