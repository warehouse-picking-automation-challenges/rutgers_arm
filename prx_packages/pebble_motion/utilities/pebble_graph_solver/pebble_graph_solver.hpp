/**
 * @file pebble_graph_solver.hpp
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

#ifndef PRX_PEBBLE_GRAPH_SOLVER_HPP
#define	PRX_PEBBLE_GRAPH_SOLVER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "utilities/pebble_tree_solver/pebble_tree_solver.hpp"
#include "utilities/pebble_spanning_tree.hpp"


#include <boost/range/adaptor/map.hpp>

namespace prx
{
    namespace packages
    {
        namespace pebble_motion
        {

            struct biconnected_component_info_t
            {

                util::undirected_vertex_index_t v_trans; // the transshipment node.
                util::undirected_vertex_index_t a_node; // a node from the biconnected component. 
                util::hash_t < util::undirected_vertex_index_t, bool > nodes; //the vertices in the biconnected graph and if they have been visited.
                std::vector<util::undirected_edge_index_t> edges; //The edges belongs to biconnected component.
                int biconnected_class; //The class of the biconnected component.
                int b_comps_size; //The size of the biconnected component ( = edges.size()).
                int vertices_size; //The number of vertices in the biconnected component.

                biconnected_component_info_t()
                {
                    v_trans = NULL;
                    a_node = NULL;
                    biconnected_class = -1;
                    b_comps_size = 0;
                    vertices_size = 0;
                }

                std::string print(util::undirected_graph_t* graph, util::undirected_graph_t* g, const util::space_t* state_space = NULL) const
                {
                    std::stringstream out(std::stringstream::out);
                    out << "class: " << biconnected_class << "   e_size:" << b_comps_size << "  v_size:" << vertices_size << std::endl;
                    if( state_space != NULL )
                    {
                        if( v_trans != NULL )
                            out << "trans : " << state_space->print_point(g->get_vertex_as<util::undirected_node_t > (v_trans)->point, 3) << std::endl;

                        foreach(util::undirected_vertex_index_t v, nodes | boost::adaptors::map_keys)
                        {
                            out << state_space->print_point(graph->get_vertex_as<util::undirected_node_t > (v)->point, 3) << std::endl;
                        }
                    }

                    return out.str();
                }
            };

            /**
             * A brief description of this class (no tag required).
             * 
             * A more detailed description of this class with multiple paragraphs delimited by blank lines. 
             */
            class pebble_graph_solver_t : public pebble_tree_solver_t
            {

              public:
                pebble_graph_solver_t();
                virtual ~pebble_graph_solver_t();

                virtual util::undirected_vertex_index_t add_new_vertex(util::undirected_graph_t* graph);

              protected:

                util::hash_t<util::undirected_vertex_index_t, util::undirected_vertex_index_t> to_tree;
                util::hash_t<util::undirected_vertex_index_t, util::undirected_vertex_index_t> to_graph;
                pebble_assignment_t sim_assignment;
                std::vector<pebble_step_t> tmp_solution;

                // A hash table with 
                // Key   : The transshipment vertex. 
                // Value : The first element is the maximal 2-vertex-connected components. 
                //         The second element is for all the possible cut vertices with their adjacent vertices
                //         of the maximal 2-vertex-connected component.
                util::hash_t<util::undirected_vertex_index_t, std::pair<std::vector<util::undirected_vertex_index_t>, util::hash_t<util::undirected_vertex_index_t, util::undirected_vertex_index_t> > > mbc;
                std::vector<biconnected_component_info_t> biconnected_comps;
                util::hash_t<util::undirected_vertex_index_t, int> transshipments_class;

                /** @copydoc pebble_solver_t::reduce(pebble_assignment_t&, pebble_assignment_t&, pebble_assignment_t&)  */
                virtual bool reduce(std::vector<pebble_step_t>* solution, util::undirected_graph_t* graph, pebble_assignment_t& s_new, pebble_assignment_t& s_assign, pebble_assignment_t& t_assign);
                void update_node_info(util::undirected_graph_t* graph, util::undirected_vertex_index_t v, util::undirected_vertex_index_t u, pebble_assignment_t& assign);

                /** @copydoc pebble_solver_t::find_solution(std::vector<pebble_step_t>*, util::undirected_graph_t*)  */
                virtual bool find_solution(std::vector<pebble_step_t>* solution, util::undirected_graph_t* graph);

                virtual void reduce_to_tree(util::undirected_graph_t* tree, const util::undirected_graph_t* graph);
                virtual void convert_path(std::vector<pebble_step_t>* solution, util::undirected_graph_t* graph, util::undirected_graph_t* tree, std::vector<pebble_step_t>* tmp_solution);

                /**
                 * It will find a free path between the start and the goal, if one exist.
                 * @param path The returned path. It does not contain the start position. 
                 * @param graph The graph that it searches for the free path
                 * @param start The starting point on the graph
                 * @param goal The goal point that we are trying to reach
                 * @return True if the free path exists else false.
                 */
                virtual bool free_path(util::undirected_graph_t* graph, util::undirected_vertex_index_t start, util::undirected_vertex_index_t goal);

                /**
                 * It will return the closest path between two point on the graph. Will not
                 * take into account the rest of the pebbles on the graph.
                 * @param path The returned path. Its a full path connecting start to goal. 
                 * @param graph The graph that it searches for the free path
                 * @param start The starting point on the graph
                 * @param goal The goal point that we are trying to reach
                 * @return True if the free path exists else false.
                 */
                virtual bool find_closest_path_for_mcbc(const util::undirected_graph_t* graph, util::undirected_vertex_index_t start, util::undirected_vertex_index_t goal);
                virtual bool check_second_path(const util::undirected_graph_t* graph, util::undirected_vertex_index_t start, util::undirected_vertex_index_t goal);
                virtual util::undirected_vertex_index_t find_first_empty_on_mbc(const util::undirected_graph_t* graph, pebble_assignment_t assign, util::undirected_vertex_index_t start, util::undirected_vertex_index_t goal);

                virtual util::undirected_vertex_index_t find_closest_vertex_with(const util::undirected_graph_t* graph, bool robot_on, util::undirected_vertex_index_t start, const pebble_assignment_t& assign);

                /**
                 * Will find the closest empty or occupied vertex on the graph. Special function for 
                 * the graph so as not to convert the assignment to graph vertices.
                 * @param graph The graph
                 * @param robot_on If we want a vertex with or without pebble on.
                 * @param start The start vertex to start the search
                 * @param assign The current assignment of the pebbles, with vertices on the tree.
                 * @return The closest vertex with the requested properties. 
                 * @assignment_on_tree Specifies if the assignment is on the tree or on the graph. By default is on the tree.
                 */
                util::undirected_vertex_index_t find_closest_on_graph_with(const util::undirected_graph_t* graph, bool robot_on, util::undirected_vertex_index_t start, const pebble_assignment_t& assign, bool assignment_on_tree = true);
                virtual int find_closest_robot(const util::undirected_graph_t* graph, util::undirected_vertex_index_t v, pebble_assignment_t& assign);
                virtual std::pair<util::undirected_vertex_index_t, bool> find_closest_useful_vertex(const util::undirected_graph_t* graph, const util::undirected_graph_t* tree, pebble_assignment_t& assign, util::undirected_vertex_index_t v, util::undirected_vertex_index_t w, bool with_robot = false);

                /**
                 * 
                 * @param solution
                 * @param graph
                 * @param tree
                 * @param assign
                 * @param v
                 * @param u
                 * @param w
                 * @return The closest vertex on the tree that is adjacent to the transshipment vertex.
                 */
                virtual util::undirected_vertex_index_t find_closest_useful_vertex(std::vector<pebble_step_t>* solution, const util::undirected_graph_t* graph, const util::undirected_graph_t* tree, pebble_assignment_t& assign, util::undirected_vertex_index_t v, util::undirected_vertex_index_t u, int b_class);

                /**
                 * Will return the vertex that is closer to v, but it is not avoid1 or avoid2. All the input except the graph are on the
                 * transformed tree.
                 * @param solution
                 * @param graph
                 * @param tree
                 * @param assign
                 * @param v
                 * @param avoid1
                 * @param avoid2
                 * @return The closest vertex on the tree that is adjacent to the transshipment vertex and close to vertex v.
                 */
                virtual util::undirected_vertex_index_t find_closest_useful_vertex_except(std::vector<pebble_step_t>* solution, const util::undirected_graph_t* graph, const util::undirected_graph_t* tree, pebble_assignment_t& assign, util::undirected_vertex_index_t v, util::undirected_vertex_index_t avoid1, util::undirected_vertex_index_t avoid2);
                virtual util::undirected_vertex_index_t is_tree_root_valid(std::vector<pebble_step_t>* solution, const util::undirected_graph_t* graph, const util::undirected_graph_t* tree, pebble_assignment_t& assign, util::undirected_vertex_index_t v, util::undirected_vertex_index_t w);

                virtual bool push_pebble_once(std::vector<pebble_step_t>* solution, const util::undirected_graph_t* graph, util::undirected_vertex_index_t v, pebble_assignment_t& assign, bool execute_push = true);
                virtual bool push_pebble_once_no_path(const util::undirected_graph_t* graph, util::undirected_vertex_index_t v, pebble_assignment_t& assign, bool execute_push = true);


                /**
                 * Push all the pebbles on the path from vertex v to vertex u. The graph has already the predecessors ready. 
                 * Works only for the initial graph. It does not work on the converted tree. 
                 * @param solution The solution will be stored here.
                 * @param graph The graph that the push will be executed.
                 * @param v Starting vertex.
                 * @param u Final vertex.
                 * @param assign The current assignment of the robots.
                 * @param assignment_on_graph Specifies if the the given assignment is on the graph or on the tree.
                 */
                virtual void push_graph(std::vector<pebble_step_t>* solution, const util::undirected_graph_t* graph, util::undirected_vertex_index_t v, util::undirected_vertex_index_t u, pebble_assignment_t& assign, bool assignment_on_tree);
                virtual void push_tree(std::vector<pebble_step_t>* solution, const util::undirected_graph_t* graph, util::undirected_vertex_index_t v, util::undirected_vertex_index_t u, pebble_assignment_t& assign);
                virtual void push_tree_no_path(const util::undirected_graph_t* graph, util::undirected_vertex_index_t v, util::undirected_vertex_index_t u, pebble_assignment_t& assign);
                virtual bool swap(std::vector<pebble_step_t>* solution, const util::undirected_graph_t* graph, int pebble1, int pebble2, util::undirected_vertex_index_t w, int criterion_no);

                virtual util::undirected_vertex_index_t get_adjacent_cut_vertex(const util::undirected_graph_t* graph, util::undirected_vertex_index_t v);

                /**
                 * Rotates the pebbles on a minimum circle in the biconnected component towards the direction, that is
                 * closer to the goal. The minimum circle in the biconnected component has been computed 
                 * in the global variable mcbc.
                 * @param solution The solution path that will be returned.
                 * @param assign The current assignment for the pebbles.
                 * @param v From the vertex v.
                 * @param u To the vertex u.
                 * @param empty An empty vertex on the maximal biconnected component graph for the rotation to start.
                 * @return The new position for the empty spot, after the rotation.
                 */
                virtual util::undirected_vertex_index_t rotate(std::vector<pebble_step_t>* solution, pebble_assignment_t& assign, util::undirected_vertex_index_t v, util::undirected_vertex_index_t u, util::undirected_vertex_index_t empty);

                virtual void move_once(std::vector<pebble_step_t>* solution, pebble_assignment_t& assign, util::undirected_vertex_index_t v, util::undirected_vertex_index_t u);


                virtual bool pass_through(std::vector<pebble_step_t>* solution, pebble_assignment_t& assign, util::undirected_vertex_index_t v_adj, util::undirected_vertex_index_t v_cut, util::undirected_vertex_index_t u);
                virtual bool biconnected_swap(std::vector<pebble_step_t>* solution, pebble_assignment_t& assign, util::undirected_vertex_index_t v_adj, util::undirected_vertex_index_t v_cut, util::undirected_vertex_index_t v, util::undirected_vertex_index_t u);

              private:

                virtual std::pair<util::undirected_vertex_index_t, util::undirected_vertex_index_t> compute_circle_with_cut(const util::undirected_graph_t* graph, const util::undirected_graph_t* tree, pebble_assignment_t& assign, util::undirected_vertex_index_t start, util::undirected_vertex_index_t goal);
                virtual void free_cycle(std::vector<pebble_step_t>* solution, int pebble_id, const util::undirected_graph_t* graph, pebble_assignment_t& assignment);
                virtual util::undirected_vertex_index_t get_v_cut(util::undirected_vertex_index_t v, const util::undirected_graph_t* graph, const util::undirected_graph_t* tree, pebble_assignment_t& assign);

                std::vector<util::undirected_vertex_index_t>::iterator adjust_iterator(std::vector<util::undirected_vertex_index_t>::iterator it, std::vector<util::undirected_vertex_index_t>& vec);
                std::deque<util::undirected_vertex_index_t>::iterator adjust_iterator(std::deque<util::undirected_vertex_index_t>::iterator it, std::deque<util::undirected_vertex_index_t>& vec, int last_element);

                virtual util::undirected_vertex_index_t get_adjacent_except(const util::undirected_graph_t* graph, util::undirected_vertex_index_t v, util::undirected_vertex_index_t avoid);

                virtual bool path_validation(const util::undirected_graph_t* graph, std::vector<util::undirected_vertex_index_t> path);
                virtual bool solution_validation(const util::undirected_graph_t* tree, std::vector<pebble_step_t>* solution);
                virtual bool all_solution_validation(const util::undirected_graph_t* tree, std::vector<pebble_step_t>* solution, const pebble_assignment_t assignment, bool print_results = true);
                bool mcbc_validation(const util::undirected_graph_t* graph);


                //    virtual void compute_maximal_biconnected_components(const util::undirected_graph_t* graph);
                //    virtual bool maximal_bc(std::vector<util::undirected_vertex_index_t>& mbc_graph, const util::undirected_graph_t* graph, int b_class, util::undirected_vertex_index_t v);
                //    virtual util::undirected_vertex_index_t is_cut_vertex(const util::undirected_graph_t* graph, util::undirected_vertex_index_t v, int b_class);
                //    virtual void block_vertex(util::undirected_vertex_index_t graph_v, util::undirected_graph_t* graph, util::undirected_graph_t* tree);
                //    virtual std::pair<util::undirected_vertex_index_t, util::undirected_vertex_index_t> get_closest_v_cut(const util::undirected_graph_t* tree, pebble_assignment_t& assign, util::undirected_vertex_index_t start, std::vector<util::undirected_vertex_index_t>& mbc_graph, util::hash_t < util::undirected_vertex_index_t, util::undirected_vertex_index_t>& v_cuts);
                //    virtual std::pair<util::undirected_vertex_index_t, util::undirected_vertex_index_t> get_closest_v_cut(const util::undirected_graph_t* tree, pebble_assignment_t& assign, util::undirected_vertex_index_t start, util::undirected_vertex_index_t goal, std::vector<util::undirected_vertex_index_t>& mbc_graph, util::hash_t < util::undirected_vertex_index_t, util::undirected_vertex_index_t>& v_cuts);

                const util::undirected_graph_t* initial_graph;

                std::deque<util::undirected_vertex_index_t> mcbc; //A pre-allocated vector in order to avoid allocating vectors during run time
                int mcbc_index; //The current index of the pre-allocated vector      

                allocated_heap_t<util::undirected_vertex_index_t> mcbc_blockers;

                std::deque<util::undirected_vertex_index_t> connecting_path;
                int connecting_path_index;

                unsigned int on_path_global;
                unsigned int on_mcbc_global;
                int last_solution_step;
                int last_solution_check;

                pebble_assignment_t s_initial; //TODO: it is just for debug you can remove it after you will be done.

            };

        }
    }
}

#endif	// PRX_PEBBLE_GRAPH_SOLVER_HPP