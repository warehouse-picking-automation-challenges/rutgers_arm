/**
 * @file manipulation_graph.hpp
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
#pragma once

#ifndef PRX_MANIPULATION_GRAPH_HPP
#define	PRX_MANIPULATION_GRAPH_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"
#include "prx/planning/motion_planners/motion_planner_edge.hpp"
#include "prx/planning/motion_planners/prm_star/prm_star_graph.hpp"

#include <algorithm>

namespace prx
{
    namespace packages
    {
        namespace rearrangement_manipulation
        {

            /**
             * @anchor manipulation_node_t
             *
             * Manipulation node is just a PRM* node.
             *
             * @brief <b> Node class used by the manipulation planning structure. </b>
             *
             * @author Athanasios Krontiris
             */
            class manipulation_node_t : public plan::prm_star_node_t
            {

              public:

                virtual ~manipulation_node_t(){ }

                void init()
                {
                    shortest_distance = PRX_INFINITY;
                    has_constraints = false;
                    updated = false;
                    constraints_sets.clear();
                    visited = 0;
                }

                /**
                 * Checks if the node has already a superset of the constraint set. If there is a super set of the given constraint set
                 * it will replace the existed one with the new smaller set. If the given set is a super set of an already existed set then
                 * the function will return false. If none of the above happens then new set will be included in the node and the function
                 * will return true.
                 * 
                 * @param constraints The new set of constraints that we want to check if exist in the node.
                 * @return -1 If the given set is super-set of an existed one, which means the new node will not be added.                 
                 *         an index If the given set is a subset of the expanded set. In this case we have to replace the existing set at place index with the new one.
                 *         size If the given set is a totally new set then we have to add the set at the end of the list. 
                 */
                bool is_set_added(const std::set<unsigned>& constraints, double dist)
                {                    
#ifndef NDEBUG              
                    //PRX_ASSERT(constraints.size() != 0);
                    // PRX_DEBUG_COLOR("Check for new constraints. This node has constraints: " << has_constraints << "  size(" << constraints_sets.size() << ") :", PRX_TEXT_GREEN);
                    // for( unsigned i = 0; i < constraints_sets.size(); ++i )
                    // {
                    //     PRX_DEBUG_COLOR(i << ") dist: " << constraints_sets[i].second << "  C:" << print_constraints(constraints_sets[i].first), PRX_TEXT_BLUE);
                    // }
                    // PRX_DEBUG_COLOR("New constraints:  dist:" << dist << "   C:" << print_constraints(constraints), PRX_TEXT_MAGENTA);
#endif
                    std::vector<unsigned> indices;

                    if( !is_dominated(constraints, dist) )
                    {
                        //Which nodes the new node will dominate
                        for( unsigned i = 0; i < constraints_sets.size(); ++i )
                        {
                            if( std::includes(constraints_sets[i].first.begin(), constraints_sets[i].first.end(), constraints.begin(), constraints.end()) && constraints_sets[i].second - dist >= -PRX_ZERO_CHECK )
                            {
                                indices.push_back(i);
                            }
                        }

                        //                        if(indices.size() > 0)
                        // PRX_DEBUG_COLOR("Node (d:" << dist << "  C:" << print_constraints(constraints) << ") will be added and will dominate: ", PRX_TEXT_GREEN);
                        for( int i = indices.size() - 1; i >= 0; --i )
                        {
                            // PRX_DEBUG_COLOR("  - Dist:" << constraints_sets[indices[i]].second << "   C: " << print_constraints(constraints_sets[indices[i]].first),  PRX_TEXT_LIGHTGRAY);
                            constraints_sets.erase(constraints_sets.begin() + indices[i]);
                        }
                        constraints_sets.push_back(std::make_pair(constraints, dist));
                        return true;
                    }
                    // PRX_DEBUG_COLOR("IT IS DOMINATED", PRX_TEXT_RED);
                    return false;
                }

                bool is_dominated(const std::set<unsigned>& constraints, double dist)
                {
                    for( unsigned i = 0; i < constraints_sets.size(); ++i )
                    {
                        //If the given set is super set of an existed set. 
                        if( std::includes(constraints.begin(), constraints.end(), constraints_sets[i].first.begin(), constraints_sets[i].first.end()) )
                        {
                            // This will take both cases. If the existed constraints have less or equal number of constraints.
                            if( fabs(constraints_sets[i].second - dist) > PRX_ZERO_CHECK && constraints_sets[i].second < dist )
                            {
                                // PRX_DEBUG_COLOR("Is Dominated by :  dist:" << constraints_sets[i].second <<  "   C:" << print_constraints(constraints_sets[i].first) , PRX_TEXT_RED); 
                                return true;
                            }
                            else if( constraints_sets[i].first.size() == constraints.size() )
                            {
                                // PRX_DEBUG_COLOR(" The new node will dominate for sure :  dist:" << constraints_sets[i].second <<  "   C:" << print_constraints(constraints_sets[i].first) , PRX_TEXT_GREEN);
                                //Here the dist of the new node is smaller than the existed one. The new node will replace this node. 
                                return false;
                            }
                        }
                    }
                    // PRX_DEBUG_COLOR("No one could dominate the new node. It is safe!", PRX_TEXT_GREEN);
                    //No one dominates the new node so it will be added in the lists.
                    return false;
                }

                std::string print_constraints(const std::set<unsigned>& constraints) const
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

                    output << "Node( " << node_id << " )  visited: " << visited << " shortest_distance:" << shortest_distance << " has_constraints:" << has_constraints << " |C|:" << constraints_sets.size() << "   C:" << std::endl;
                    for(unsigned i = 0; i < constraints_sets.size(); ++i)
                        output << print_constraints(constraints_sets[i].first) << std::endl;
                    return output.str();
                }


                bool updated;
                double shortest_distance;
                bool has_constraints;
                std::vector<std::pair<std::set<unsigned >, double> > constraints_sets;
                int visited;
            };

            /**
             * @anchor manipulation_edge_t
             *
             * Edges in the manipulation planning structure store everything that the PRM edge
             * will store. Moreover, will store the constraints for the edge.
             *
             * @brief <b> Edge class used in the manipulation planning structure. </b>
             *
             * @author Athanasios Krontiris
             */
            class manipulation_edge_t : public plan::prm_star_edge_t
            {

              public:
                /** @brief The cost associated with this edge. */
                std::set<unsigned> constraints;
                util::undirected_vertex_index_t v_source;
                sim::plan_t other_way_plan;

                virtual ~manipulation_edge_t(){ }

                virtual void link_spaces(const util::space_t* state_space, const util::space_t* control_space)
                {
                    plan.link_control_space(control_space);
                    other_way_plan.link_control_space(control_space);
                    path.link_space(state_space);
                }

                virtual void set_source_vertex(util::undirected_vertex_index_t v)
                {
                    v_source = v;
                }

                virtual void add_constraints(std::set<unsigned>& constraints)
                {
                    this->constraints.insert(constraints.begin(), constraints.end());
                }

                virtual void add_constraint(unsigned pose_id)
                {
                    constraints.insert(pose_id);
                }

                virtual bool is_valid(const std::set<unsigned>* valid_constraints)
                {

                    foreach(unsigned c, constraints)
                    {
                        if( valid_constraints->find(c) != valid_constraints->end() )
                            return false;
                    }
                    return true;
                }

                virtual void get_valid_constraints(std::set<unsigned>& edge_constraints, const std::set<unsigned>* valid_constraints)
                {

                    foreach(unsigned c, constraints)
                    {
                        if( valid_constraints->find(c) != valid_constraints->end() )
                            edge_constraints.insert(c);
                    }
                }

                virtual void get_plan(sim::plan_t& plan, util::undirected_vertex_index_t v)
                {
                    if( v == v_source )
                        plan = this->plan;
                    else
                        plan = other_way_plan;
                        // plan.reverse_plan(this->plan);
                }

                virtual void append_plan(sim::plan_t& plan, util::undirected_vertex_index_t v)
                {
                    if( v == v_source )
                    {
                        PRX_DEBUG_COLOR("use this way", PRX_TEXT_GREEN);
                        plan += this->plan;
                    }
                    else
                    {
                        PRX_DEBUG_COLOR("use the other way", PRX_TEXT_CYAN);
                        plan += other_way_plan;
                        // sim::plan_t reverse;
                        // reverse.reverse_plan(this->plan);
                        // plan += reverse;
                    }
                }

                virtual const sim::plan_t& use_plan(util::undirected_vertex_index_t v)
                {
                    if( v == v_source )
                        return plan;
                    else
                        return other_way_plan;
                }

                virtual void serialize(std::ofstream& output_stream)
                {
                    // prm_star_edge_t::serialize(output_stream);
                    // output_stream << std::endl;
                    // plan.save_to_stream(output_stream, 10);
                    // other_way_plan.save_to_stream(output_stream, 10);
                    // output_stream << source_vertex << std::endl << constraints.size() << std::endl;

                    // foreach(unsigned c, constraints)
                    // {
                    //     output_stream << c << " ";
                    // }

                    prm_star_edge_t::serialize(output_stream);
                    output_stream << std::endl;
                    output_stream << source_vertex << std::endl;

                    plan.save_to_stream(output_stream, 8);
                    other_way_plan.save_to_stream(output_stream, 8);

                    output_stream << constraints.size() << std::endl;
                    foreach(unsigned c, constraints)
                    {
                        output_stream << c << " ";
                    }
                    output_stream << std::endl;
                }

                virtual void deserialize(std::ifstream& input_stream)
                {
                    unsigned size;
                    unsigned constraint;
                    input_stream >> source_vertex;
                    if( plan.is_initialized() )
                        plan.read_from_stream(input_stream);
                    
                    if( other_way_plan.is_initialized() )
                        other_way_plan.read_from_stream(input_stream);
                    
                    input_stream >> size;
                    for( unsigned i = 0; i < size; ++i )
                    {
                        input_stream >> constraint;
                        constraints.insert(constraint);
                    }
                    //PRX_DEBUG_COLOR("Constraints [" << size << "]: " << print_constraints(), PRX_TEXT_LIGHTGRAY);
                }

                virtual std::string print_constraints() const
                {
                    std::stringstream output(std::stringstream::out);

                    foreach(unsigned i, constraints)
                    {
                        output << i << " , ";
                    }
                    return output.str();
                }

            };
        }
    }
}

#endif
