/**
 * @file tree_pebble_graph.hpp
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

#ifndef PRX_TREE_PEBBLE_GRAPH_HPP
#define	PRX_TREE_PEBBLE_GRAPH_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"
#include "utilities/pebble_assignment.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"

#include <boost/range/adaptor/map.hpp>

namespace prx
{
    namespace packages
    {
        namespace pebble_motion
        {

            struct tree_information_t
            {

                tree_information_t()
                {
                    swap_dist = PRX_INFINITY;
                    holes = 0;
                }

                std::string print_info() const
                {
                    std::stringstream out(std::stringstream::out);
                    out << "holes:" << holes << "  v_swap:" << v_swap << "  dist:" << swap_dist << std::endl;

                    foreach(util::undirected_vertex_index_t v, vertices_in_tree)
                    {
                        out << v << "  ";
                    }
                    out << std::endl;

                    return out.str();
                }

                util::undirected_vertex_index_t v_swap;
                double swap_dist;
                std::vector<util::undirected_vertex_index_t> vertices_in_tree; //all the vertices exist in that tree
                unsigned int holes;
                std::vector<util::undirected_vertex_index_t> tree_seen; //the seen(T(F(v),u))

            };

            /**
             * The graph node that keeps the information for the tree_feasibility
             * algorithm for Pebble Motion Problems on Trees.
             */
            class tree_pebble_node_t : public util::undirected_node_t
            {

              public:

                tree_pebble_node_t()
                {
                    equiv_class = -1;
                    checked = false;
                    occupied = false;
                    visited = false;
                    checked_trees = 0;
                    num_trees = -1;
                    num_free_trees = 0;
                    bfs_id = 0;
                    obstacle_id = 0;
                }

                ~tree_pebble_node_t(){ }

                void init_node(const util::space_t* space, const std::vector<double>& vec)
                {
                    state_space = space;
                    point = state_space->alloc_point();
                    state_space->set_from_vector(vec, point);
                    equiv_class = -1;
                    checked = false;
                    occupied = false;
                    visited = false;
                    checked_trees = 0;
                    num_trees = -1;
                    num_free_trees = 0;
                }

                util::undirected_vertex_index_t get_parent2(const util::undirected_graph_t* graph)
                {

                    foreach(util::undirected_vertex_index_t v, boost::adjacent_vertices(index, graph->graph))
                    {
                        //            PRX_DEBUG_S("adj of " << index << " is : " << v << "  (" << graph->get_vertex_as<tree_pebble_node_t>(v)->checked << ")" );
                        if( !graph->get_vertex_as<tree_pebble_node_t > (v)->checked )
                        {
                            v_par = v;
                            return v;
                        }
                    }
                    v_par = index;
                    return index;
                }

                util::undirected_vertex_index_t get_parent(const util::undirected_graph_t* graph, unsigned long obstacle_global_id)
                {

                    foreach(util::undirected_vertex_index_t v, boost::adjacent_vertices(index, graph->graph))
                    {
                        //            PRX_DEBUG_S("adj of " << index << " is : " << v << "  (" << graph->get_vertex_as<tree_pebble_node_t>(v)->checked << ")" );
                        if( graph->get_vertex_as<tree_pebble_node_t > (v)->obstacle_id != obstacle_global_id )
                        {
                            v_par = v;
                            return v;
                        }
                    }
                    v_par = index;
                    return index;
                }

                bool has_children(const util::undirected_graph_t* graph)
                {
                    return (boost::out_degree(index, graph->graph) > 1);
                }

                int num_unvisited_neighboors2(const util::undirected_graph_t* graph)
                {
                    int num = 0;

                    foreach(util::undirected_vertex_index_t v, boost::adjacent_vertices(index, graph->graph))
                    {
                        if( !graph->get_vertex_as<tree_pebble_node_t > (v)->checked )
                        {
                            num++;
                        }
                    }
                    return num;
                }

                int num_unvisited_neighboors(const util::undirected_graph_t* graph, unsigned long obstacle_global_id)
                {
                    int num = 0;

                    foreach(util::undirected_vertex_index_t v, boost::adjacent_vertices(index, graph->graph))
                    {
                        if( graph->get_vertex_as<tree_pebble_node_t > (v)->obstacle_id != obstacle_global_id )
                        {
                            num++;
                        }
                    }
                    //        PRX_ERROR_S("has " << num << " / " << boost::degree(index,graph->graph) << "   free neighbors");

                    return num;
                }

                int get_holes_except(util::undirected_vertex_index_t v, util::undirected_vertex_index_t u)
                {
                    int holes = 0;
                    util::undirected_vertex_index_t t1 = tree_for_vertex[v];
                    util::undirected_vertex_index_t t2 = tree_for_vertex[u];

                    foreach(util::undirected_vertex_index_t t, free_trees)
                    {
                        if( t != t1 && t != t2 )
                        {
                            holes += trees[t].holes;
                        }
                    }
                    return holes;
                }

                util::undirected_vertex_index_t get_empty_tree_except(util::undirected_vertex_index_t t1, util::undirected_vertex_index_t t2 = NULL)
                {

                    foreach(util::undirected_vertex_index_t v, trees | boost::adaptors::map_keys)
                    {
                        //            PRX_DEBUG_S("v:" << v << " has holes: "<< trees[v].holes);
                        if( v != NULL && v != t1 && v != t2 && trees[v].holes > 0 )
                            return v;
                    }

                    return NULL;
                }

                util::undirected_vertex_index_t find_empty_tree_except(const pebble_assignment_t& assign, util::undirected_vertex_index_t t1, util::undirected_vertex_index_t t2 = NULL)
                {

                    foreach(util::undirected_vertex_index_t v, trees | boost::adaptors::map_keys)
                    {
                        if( v != NULL && v != t1 && v != t2 )
                        {

                            foreach(util::undirected_vertex_index_t u, trees[v].vertices_in_tree)
                            {
                                if( !assign.has_robot_on(u) )
                                    return v;
                            }
                        }
                    }
                    return NULL;
                }

                util::undirected_vertex_index_t find_full_tree_except(const pebble_assignment_t& assign, util::undirected_vertex_index_t t1, util::undirected_vertex_index_t t2 = NULL)
                {

                    foreach(util::undirected_vertex_index_t v, trees | boost::adaptors::map_keys)
                    {
                        if( v != NULL && v != t1 && v != t2 )
                        {
                            bool full_tree = true;
                            //                PRX_DEBUG_S("going to check " << v );

                            for( unsigned int i = 0; i < trees[v].vertices_in_tree.size() && full_tree; ++i )
                            {
                                //                    PRX_DEBUG_S("cheching vertex : " << trees[v].vertices_in_tree[i]);
                                if( !assign.has_robot_on(trees[v].vertices_in_tree[i]) )
                                    full_tree = false;
                            }
                            if( full_tree )
                                return v;
                        }
                    }
                    return NULL;
                }

                util::undirected_vertex_index_t get_any_full_tree(util::undirected_vertex_index_t t1 = NULL)
                {

                    //        PRX_ERROR_S("size of trees :  " << trees.size());

                    foreach(util::undirected_vertex_index_t v, trees | boost::adaptors::map_keys)
                    {
                        //            PRX_DEBUG_S("for full v:" << v << " has holes: " << trees[v].holes);
                        if( v != NULL && v != t1 && trees[v].holes == 0 )
                            return v;
                    }

                    return NULL;
                }

                util::undirected_vertex_index_t get_closest_swap_avoid(util::undirected_vertex_index_t v)
                {
                    double dist = PRX_INFINITY;
                    util::undirected_vertex_index_t branch = NULL;

                    foreach(util::undirected_vertex_index_t t, trees | boost::adaptors::map_keys)
                    {
                        if( t != tree_for_vertex[v] && trees[t].swap_dist < dist )
                        {
                            branch = trees[t].v_swap;
                            dist = trees[t].swap_dist;
                        }
                    }
                    return branch;
                }

                std::string print_info() const
                {
                    std::stringstream out(std::stringstream::out);
                    out << "class:" << equiv_class << "  checked_trees: " << checked_trees << "   num_trees:" << num_trees << "  free_trees: " << num_free_trees << std::endl;

                    foreach(util::undirected_vertex_index_t v, trees | boost::adaptors::map_keys)
                    {
                        out << " for the tree : " << v << std::endl;
                        out << trees[v].print_info();
                    }
                    out << std::endl << "seen : ";

                    foreach(util::undirected_vertex_index_t v, seen)
                    {
                        out << v << "  ";
                    }
                    out << std::endl;

                    foreach(util::undirected_vertex_index_t v, tree_for_vertex | boost::adaptors::map_keys)
                    {
                        out << v << "  ";
                    }
                    out << std::endl;
                    out << "tree for vertex size :" << tree_for_vertex.size() << "    seen size: " << seen.size() << std::endl;
                    return out.str();
                }

                virtual std::string print_point(const util::space_t* space, unsigned int prec = 3) const
                {
                    std::stringstream out(std::stringstream::out);
                    out << space->print_point(point, prec);
                    return out.str();
                }

                int equiv_class;
                bool occupied;
                bool checked;
                bool visited;
                int checked_trees;
                int num_trees;
                int num_free_trees;
                unsigned long bfs_id;
                unsigned long obstacle_id;
                std::vector<util::undirected_vertex_index_t> free_trees;
                util::undirected_vertex_index_t v_par;
                std::vector<util::undirected_vertex_index_t> seen; //the seen(v)
                //key: the root of the subtree, value the struct tree_information_t with information for the subtree
                util::hash_t<util::undirected_vertex_index_t, tree_information_t> trees;
                //key : the vertex of the tree, value the root of the subtree
                util::hash_t<util::undirected_vertex_index_t, util::undirected_vertex_index_t> tree_for_vertex;

              protected:
                const util::space_t* state_space;

            };


        }
    }
}

#endif	// PRX_TREE_PEBBLE_GRAPH_HPP

