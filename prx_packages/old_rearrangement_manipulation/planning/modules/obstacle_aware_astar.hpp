/**
 * @file obstacle_aware_astar.hpp
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

#ifndef PRX_OBSTACLE_AWARE_ASTAR_HPP
#define	PRX_OBSTACLE_AWARE_ASTAR_HPP


#include "prx/planning/modules/heuristic_search/astar_module.hpp"

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/heuristic_search/astar_open_set.hpp"

#include <set>
#include <boost/range/adaptor/map.hpp>

#define PRX_OA_ASTAR_PENALTY 10000

/**
 * Writing this assuming the following variables are (somehow) available:
 *
 * directed_graph_t graph (With node and edge types from arrange_graph.hpp)
 * system_name_validity_checker_t naming_validity_checker
 * std::vector< directed_vertex_index_t > OA_predecessor_map
 */

namespace prx
{
    namespace packages
    {
        namespace rearrangement_manipulation
        {

            class system_name_validity_checker_t;

            class rearrangement_astar_node_t : public util::astar_node_t
            {

              public:

                rearrangement_astar_node_t(){ }

                rearrangement_astar_node_t(util::undirected_vertex_index_t vertex) : astar_node_t(vertex, 0)
                {
                    this->vertex = vertex;
                }

                rearrangement_astar_node_t(util::undirected_vertex_index_t vertex, double cost, double h) : astar_node_t(vertex, cost + h)
                {
                    this->cost = cost;
                    this->h = h;
                }

                rearrangement_astar_node_t(const rearrangement_astar_node_t & n) : astar_node_t(n.vertex, n.f)
                {
                    constraints = n.constraints;
                    foul_constraints = n.foul_constraints;
                    path = n.path;
                    cost = n.cost;
                    h = n.h;
                }

                virtual ~rearrangement_astar_node_t(){ }

                virtual const rearrangement_astar_node_t& operator=(const rearrangement_astar_node_t & other)
                {
                    vertex = other.vertex;
                    f = other.f;
                    constraints = other.constraints;
                    foul_constraints = other.foul_constraints;
                    path = other.path;
                    cost = other.cost;
                    h = other.h;
                    return (*this);
                }

                virtual bool operator<(const rearrangement_astar_node_t & n) const
                {
                    if( constraints.size() == n.constraints.size() )
                    {
                        if( foul_constraints.size() == n.foul_constraints.size() )
                            return f < n.f;
                        return foul_constraints.size() < n.foul_constraints.size();
                    }
                    return constraints.size() < n.constraints.size();
                }

                virtual bool operator<(const astar_node_t & n) const
                {
                    const rearrangement_astar_node_t* node = dynamic_cast<const rearrangement_astar_node_t*>(&n);
                    if( constraints.size() == node->constraints.size() )
                    {
                        if( foul_constraints.size() == node->foul_constraints.size() )
                            return f < node->f;
                        return foul_constraints.size() < node->foul_constraints.size();
                    }
                    return constraints.size() < node->constraints.size();
                    //                    PRX_DEBUG_COLOR("Yes2 checking: c:" << constraints.size() << "/" << node->constraints.size() << "   f:" << f << "/" << node->f, PRX_TEXT_BROWN);
                    //                    if( constraints.size() == node->constraints.size() )
                    //                    return f < node->f;

                    //                    return constraints.size() < node->constraints.size();
                }

                virtual operator unsigned() const
                {
                    return *(unsigned int*)(&vertex);
                }

                virtual void set_f(double cost, double h)
                {
                    this->cost = cost;
                    this->h = h;
                    f = cost + h;
                }

                virtual void add_a_constraints(unsigned constraint)
                {
                    constraints.insert(constraint);
                }

                virtual void add_constraints(const std::set<unsigned>& new_constraints)
                {
                    constraints.insert(new_constraints.begin(), new_constraints.end());
                }

                virtual void add_fouls(const std::set<unsigned>& new_constraints)
                {
                    foul_constraints.insert(new_constraints.begin(), new_constraints.end());
                }

                virtual bool path_has_vertex(util::undirected_vertex_index_t v)
                {
                    return std::find(path.begin(), path.end(), v) != path.end();
                }

                virtual void merge(const rearrangement_astar_node_t* node)
                {
                    add_constraints(node->constraints);
                    add_fouls(node->foul_constraints);
                    path = node->path;
                    path.push_back(vertex);
                }

                virtual unsigned no_constraints() const
                {
                    return constraints.size();
                }

                virtual unsigned no_fouls() const
                {
                    return foul_constraints.size();
                }

                std::string print_constraints() const
                {
                    std::stringstream output(std::stringstream::out);

                    foreach(unsigned i, constraints)
                    {
                        output << i << " , ";
                    }
                    return output.str();
                }

                std::string print() const
                {
                    std::stringstream output(std::stringstream::out);
                    output << "f:" << f << " cost:" << cost << " path:" << path.size() << std::endl;
                    output << "constraints      : " << print_constraints() << std::endl;
                    output << "foul_constraints : ";

                    foreach(unsigned i, foul_constraints)
                    {
                        output << i << " , ";
                    }
                    return output.str();
                }

                std::string print_few() const
                {
                    std::stringstream output(std::stringstream::out);
                    output << "f:" << f << "\t sum:" << cost + h << "\t cost: " << cost << "\t h:" << h << "\t foul_constraints: " << foul_constraints.size() << "\t constraints: " << print_constraints();
                    return output.str();
                }

                std::set<unsigned> constraints;
                std::set<unsigned> foul_constraints;
                std::deque<util::undirected_vertex_index_t > path;
                double cost;
                double h;
            };

            class obstacle_aware_astar_t : public plan::astar_module_t
            {

              public:
                obstacle_aware_astar_t();

                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                virtual void link_validity_checker(plan::validity_checker_t* checker);
                virtual void link_valid_constraints(const std::set<unsigned>* constraints);
                virtual void link_constraints(const std::set<unsigned>* blocking_constraints, const std::set<unsigned>* collision_constraints, const std::set<unsigned>* avoid_constraints);

                virtual bool solve(util::undirected_vertex_index_t start, const std::vector<util::undirected_vertex_index_t>& goals);
                virtual bool solve(util::undirected_vertex_index_t start, util::undirected_vertex_index_t goal);
                virtual void extract_path(util::undirected_vertex_index_t start, util::undirected_vertex_index_t goal, std::deque<util::undirected_vertex_index_t>& vertices);
                virtual void extract_path_constraints(std::set<unsigned>& constraints);
                virtual bool is_valid_path();
                virtual void get_path_constraints(std::set<unsigned>& constraints);
                virtual void get_foul_constraints(std::set<unsigned>& constraints);
                virtual sim::state_t* get_start_point();
                virtual sim::state_t* get_goal_point();

                virtual bool expand_vertex(util::undirected_vertex_index_t vertex);
                virtual bool examine_vertex(util::undirected_vertex_index_t vertex);
                virtual bool examine_edge(util::undirected_vertex_index_t start_index, util::undirected_edge_index_t e, util::undirected_vertex_index_t end_index);
                virtual double heuristic(util::undirected_vertex_index_t current, util::undirected_vertex_index_t goal);
                virtual double heuristic(util::undirected_vertex_index_t current, const std::vector< util::undirected_vertex_index_t >& goals);


                virtual void set_minimum_conflict(bool flag);
                virtual void set_shortest_path_flag(bool flag);
                virtual void set_exact_flag(bool flag);
                virtual void set_max_length(double length);
                virtual void set_collision_penalty(double penalty);

                virtual double get_path_cost() const;

                std::vector<util::undirected_vertex_index_t> extra_starts;

              protected:
                bool in_the_same_component(const util::undirected_vertex_index_t v, const std::vector<util::undirected_vertex_index_t> vertices);
                std::string print(const std::set<unsigned>& constraints);
                std::string print(const util::undirected_graph_t *graph, const std::deque<util::undirected_vertex_index_t>& vertices);

                util::undirected_vertex_index_t found_start;
                double max_length;
                double collision_penalty;
                double new_constraint_penalty;
                bool minimum_conflict;
                bool shortest_path;
                bool exact;
                std::deque<util::undirected_vertex_index_t> final_path;
                std::set<unsigned> path_constraints;
                /** @brief How many new constraints we are invalidating.*/
                std::set<unsigned> foul_constraints;
                double final_path_cost;
                system_name_validity_checker_t* system_checker;

                /** @brief The constraints that the astar has to respect.*/
                const std::set<unsigned>* obstacle_constraints;
                /** @brief The constraints that it would be nice to avoid. Applies the \c avoid_penalty*/
                const std::set<unsigned>* valid_constraints;
                /** @brief Poses that the astar should avoid if it cans. Applies the \c new_constraint_penalty (\c new_constraint_penalty << \c avoid_penalty)*/
                const std::set<unsigned>* avoid_constraints;

                //helping variable
                rearrangement_astar_node_t* top_node;
                std::set<unsigned> new_constraints;
                bool new_has_constraints;
                double new_dist;
            };
        }
    }
}

#endif
