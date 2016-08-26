/**
 * @file pebble_test.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2012, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file robot_idd LICENSE.
 *
 * @authors Athanasios Krontiris, Andrew Dobson, Andrew Kimmel, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "utilities/kornhauser/kornhauser_tester.hpp"
#include "utilities/pebble_spanning_tree.hpp"

#include <boost/graph/connected_components.hpp>
#include <boost/graph/biconnected_components.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/range/adaptor/map.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::packages::pebble_motion::kornhauser_tester_t, prx::packages::pebble_motion::pebble_tester_t)

namespace prx
{
    using namespace util;
    
    namespace packages
    {
        namespace pebble_motion
        {

            void kornhauser_tester_t::reset()
            {
                connectors.clear();
                same_sub_nodes.clear();
                G_bic.clear();
                G_sub_blanks.clear();

                foreach(pebble_subproblem_t pp, PP_sub)
                {
                    delete pp.graph;
                }
                PP_sub.clear();
            }

            bool kornhauser_tester_t::pebble_test(undirected_graph_t* graph)
            {
                pebble_assignment_t s_new;

                foreach(undirected_vertex_index_t v, boost::vertices(graph->graph))
                PRX_DEBUG_S(v);

                g = graph;

                foreach(undirected_vertex_index_t v, boost::vertices(g->graph))
                PRX_WARN_S(v);
                PRX_INFO_S("... Going to check if there is solution");
                s_assignment.print();
                t_assignment.print();
                PRX_WARN_S("************************************************************************");

                int num = boost::connected_components(g->graph, g->components);
                if( num > 1 )
                {
                    //        PRX_LOG_ERROR("Multiple components! Coming SOON !!! ");
                    if( !s_assignment.same_connected_components(g, t_assignment) )
                    {
                        PRX_ERROR_S("Assignments in different Connected Components");
                        return false;
                    }
                }

                if( !reduce(s_new, s_assignment, t_assignment) )
                {
                    PRX_ERROR_S("Could not occupy same nodes as final assignment");
                    return false;
                }

                connected_components_subproblems(g, s_new, t_assignment, num);


                int m = boost::num_vertices(g->graph) - k;
                PRX_DEBUG_S("vertices: " << boost::num_vertices(g->graph) << "   k:" << k << "    m : " << m);
                if( m <= 1 )
                {
                    PRX_ERROR_S("WILSON");
                    return false; //Because of Wilson
                }

                for( int i = 0; i < num; i++ )
                {
                    reset();
                    m = boost::num_vertices(G_cc[i].graph->graph) - G_cc[i].robot_size();
                    //        PRX_WARN_S("m = " << m << "     k: " << G_cc[i].robot_size());
                    compute_subproblems(G_cc[i], m);

                    if( !start_plank.has_same_robots(end_plank) )
                        return false;

                    foreach(pebble_subproblem_t pp, PP_sub)
                    {
                        if( !pp.has_same_assignments() )
                            return false;
                        if( is_polygon(pp.graph) && !polygon_has_solution(pp) )
                            return false;
                        if( m == 1 )//&& !wilson_solution())
                            return false;
                    }
                }
                return true;
            }

            undirected_vertex_index_t kornhauser_tester_t::add_new_vertex(undirected_graph_t* graph)
            {
                return graph->add_vertex<pebble_node_t > ();
            }

            void kornhauser_tester_t::connected_components_subproblems(undirected_graph_t* graph, pebble_assignment_t& s_assignment, pebble_assignment_t& t_assignment, int num_cc)
            {
                undirected_vertex_index_t v_new;
                undirected_vertex_index_t source, target;
                int index;

                G_cc.resize(num_cc);

                for( int i = 0; i < num_cc; ++i )
                    G_cc[i].graph = new undirected_graph_t();

                foreach(undirected_vertex_index_t v, boost::vertices(graph->graph))
                {
                    index = graph->components[v];
                    v_new = G_cc[index].graph->add_vertex<pebble_node_t > ();
                    G_cc[index].graph->get_vertex_as<pebble_node_t > (v_new)->init_node(v, state_space, graph->get_vertex_as<undirected_node_t > (v)->point);
                    same_cc_nodes[v] = v_new;
                }

                foreach(undirected_edge_index_t e, boost::edges(graph->graph))
                {
                    index = graph->components[boost::source(e, graph->graph)];
                    source = same_cc_nodes[boost::source(e, graph->graph)];
                    target = same_cc_nodes[boost::target(e, graph->graph)];

                    G_cc[index].graph->add_edge<undirected_edge_t > (source, target);
                }

                foreach(undirected_vertex_index_t v, s_assignment.assignments | boost::adaptors::map_keys)
                {
                    index = graph->components[v];
                    G_cc[index].s_assign.add_assignment(same_cc_nodes[v], s_assignment.get_robot(v));
                }

                foreach(undirected_vertex_index_t v, t_assignment.assignments | boost::adaptors::map_keys)
                {
                    index = graph->components[v];
                    G_cc[index].e_assign.add_assignment(same_cc_nodes[v], t_assignment.get_robot(v));
                    //        PRX_INFO_S("robot is : " << s_assignment.get_robot(v));
                }

                for( int i = 0; i < num_cc; ++i )
                    G_cc[i].print();
            }

            bool kornhauser_tester_t::reduce(pebble_assignment_t& s_new, pebble_assignment_t& s_assignment, pebble_assignment_t& t_assignment)
            {
                pebble_spanning_tree_t spanning_tree;
                std::deque<undirected_vertex_index_t> leaves;
                undirected_vertex_index_t l, v, par;
                pebble_assignment_t curr_assign;

                curr_assign = s_assignment;

                spanning_tree.compute_spanning_tree(g);
                spanning_tree.get_leaves(leaves);

                //    PRX_INFO_S("spanning tree has :" << boost::connected_components(spanning_tree.spanning_tree.g, spanning_tree.spanning_tree.components) );

                while( !leaves.empty() )
                {
                    //        PRX_INFO_S("the number of vertices: " << boost::num_vertices(spanning_tree.spanning_tree.g));
                    //        PRX_INFO_S("no of leaves : " << leaves.size());
                    l = leaves[0];
                    par = spanning_tree.get_parent(l);
                    v = spanning_tree.get_graph_index(l);
                    leaves.pop_front();
                    //PRX_INFO_S("BEFORE no of v:" << boost::num_vertices(spanning_tree.g) << "    e:" << boost::num_edges(spanning_tree.g));

                    //        PRX_INFO_S("curr assignment:");
                    //        curr_assign.print();
                    if( t_assignment.is_empty(v) )
                    {
                        if( !curr_assign.is_empty(v) )
                            if( !spanning_tree.push(l, curr_assign) )
                                return false;
                    }
                    else
                    {
                        if( curr_assign.is_empty(v) )
                        {
                            int robot_id = spanning_tree.find_closest_robot(l, curr_assign);

                            if( robot_id == -1 )
                                return false;

                            s_new.add_assignment(v, robot_id);
                        }
                        else
                        {
                            s_new.add_assignment(v, curr_assign.get_robot(v));
                        }
                    }

                    //        PRX_ERROR_S("Delete node : " << l);
                    spanning_tree.remove_vertex(l);

                    if( par != l && !spanning_tree.has_children(par) )
                        if( std::find(leaves.begin(), leaves.end(), par) == leaves.end() )
                            leaves.push_back(par);
                }

                //    PRX_WARN_S("s_new and t_assignment");
                //    s_assignment.print();
                //    s_new.print();
                //    t_assignment.print();
                //    PRX_WARN_S("------------------------------");

                return true;
            }

            void kornhauser_tester_t::compute_equivalent_connectors(undirected_graph_t* graph, int m)
            {
                undirected_graph_t g_equivalent;
                hash_t<undirected_vertex_index_t, undirected_vertex_index_t> same_nodes;

                foreach(undirected_vertex_index_t v, connectors)
                {
                    undirected_vertex_index_t v_new = g_equivalent.add_vertex<info_node_t > ();
                    g_equivalent.get_vertex_as<info_node_t > (v_new)->graph_index = v;
                    same_nodes[v] = v_new;
                }

                int sz = (int)connectors.size();
                for( int i = 0; i < sz - 1; i++ )
                {
                    undirected_vertex_index_t vi = connectors[i];
                    for( int j = i + 1; j < sz; j++ )
                    {
                        undirected_vertex_index_t vj = connectors[j];
                        if( vi != vj )
                        {
                            if( in_same_non_trivial_component(graph, vi, vj) )
                                g_equivalent.add_edge<undirected_edge_t > (same_nodes[vi], same_nodes[vj]);
                            else if( has_unique_path(graph, vi, vj) )
                                if( path_length <= m - 2 )
                                    g_equivalent.add_edge<undirected_edge_t > (same_nodes[vi], same_nodes[vj]);
                        }
                    }
                }
                int num = boost::connected_components(g_equivalent.graph, g_equivalent.components);

                PP_sub.resize(num);
                for( int i = 0; i < num; i++ )
                    PP_sub[i].graph = new undirected_graph_t();

                foreach(undirected_vertex_index_t v, connectors)
                {
                    int index = g_equivalent.components[same_nodes[v]];
                    graph->get_vertex_as<pebble_node_t > (v)->add_graph_class(index);
                    undirected_vertex_index_t v_new = PP_sub[index].graph->add_vertex<pebble_node_t > ();
                    *(PP_sub[index].graph->get_vertex_as<pebble_node_t > (v_new)) = *(graph->get_vertex_as<pebble_node_t > (v));
                    PP_sub[index].graph->get_vertex_as<pebble_node_t > (v_new)->add_graph_class(index);
                    same_sub_nodes[v] = v_new;

                }
                //    PRX_INFO_S("equivalent are : " << num << "    v:" << boost::num_vertices(g_equivalent.graph) << "    e:" << boost::num_edges(g_equivalent.graph));
                //    
                //    foreach(undirected_vertex_index_t v, boost::vertices(g_equivalent.graph))
                //    {
                //        PRX_WARN_S("v:" << v << "  in component : " << g_equivalent.components[v] ); 
                //    }
            }

            void kornhauser_tester_t::compute_subgraphs(undirected_graph_t* graph, int m)
            {
                bool valance_exist = false;

                std::size_t num_comps = boost::biconnected_components(graph->graph, graph->edge_components);
                G_bic.resize(num_comps);
                int index;

                foreach(undirected_vertex_index_t v, boost::vertices(graph->graph))
                {

                    foreach(undirected_edge_index_t e, boost::out_edges(v, graph->graph))
                    {
                        index = graph->edge_components[e];
                        if( graph->get_vertex_as<pebble_node_t > (v)->add_class(graph, index) )
                            G_bic[index]++;
                    }
                    if( graph->get_vertex_as<pebble_node_t > (v)->is_connector() )
                    {
                        valance_exist = true;
                        connectors.push_back(v);
                    }
                }

                ////////////////////////   PRINTS   /////////////////////////////////////////
                //
                //    foreach(undirected_vertex_index_t v, boost::vertices(graph->graph))
                //    {
                //        if( graph->get_vertex_as<pebble_node_t > (v)->is_connector() )
                //            PRX_INFO_S(v);
                //    }
                //
                //    PRX_INFO_S(" there are comp: " << num_comps);
                //    for( size_t i = 0; i < num_comps; ++i )
                //    {
                //        PRX_INFO_S("graph " << i << "   has : " << G_bic[i]);
                //    }
                ///////////////////////////////////////////////////////////////////////////


                if( m == 1 )
                {
                    PRX_LOG_ERROR("Wilson. We are not here yet!");
                }
                else
                {
                    if( num_comps == 1 && !is_graph_trivial(graph) )
                    {
                        pebble_subproblem_t prob(graph);
                        PP_sub.push_back(prob);

                        foreach(undirected_vertex_index_t v, boost::vertices(graph->graph))
                        {
                            PP_sub[0].graph->get_vertex_as<pebble_node_t > (v)->add_graph_class(0);
                        }
                        PRX_INFO_S("graph is not trivial");
                    }
                    else if( valance_exist )
                    {
                        compute_equivalent_connectors(graph, m);

                        foreach(undirected_vertex_index_t v, boost::vertices(graph->graph))
                        {
                            if( !graph->get_vertex_as<pebble_node_t > (v)->is_connector() )
                            {

                                foreach(undirected_vertex_index_t v_con, connectors)
                                {
                                    int index = graph->get_vertex_as<pebble_node_t > (v_con)->get_graph_classes()[0]; //Connectors are only in one graph
                                    if( !graph->get_vertex_as<pebble_node_t > (v)->has_graph_class(index) )
                                    {
                                        if( in_same_non_trivial_component(graph, v, v_con) )
                                        {
                                            undirected_vertex_index_t v_new = PP_sub[index].graph->add_vertex<pebble_node_t > ();
                                            graph->get_vertex_as<pebble_node_t > (v)->add_graph_class(index);
                                            *(PP_sub[index].graph->get_vertex_as<pebble_node_t > (v_new)) = *(graph->get_vertex_as<pebble_node_t > (v));
                                            PP_sub[index].graph->get_vertex_as<pebble_node_t > (v_new)->add_graph_class(index);
                                            same_sub_nodes[v] = v_new;
                                        }
                                        else if( has_unique_path(graph, v, v_con) )
                                        {
                                            if( path_length <= m - 1 )
                                            {
                                                undirected_vertex_index_t v_new = PP_sub[index].graph->add_vertex<pebble_node_t > ();
                                                graph->get_vertex_as<pebble_node_t > (v)->add_graph_class(index);
                                                *(PP_sub[index].graph->get_vertex_as<pebble_node_t > (v_new)) = *(graph->get_vertex_as<pebble_node_t > (v));
                                                PP_sub[index].graph->get_vertex_as<pebble_node_t > (v_new)->add_graph_class(index);
                                                same_sub_nodes[v] = v_new;
                                            }
                                        }
                                    }
                                }
                            }
                        }

                        foreach(undirected_edge_index_t e, boost::edges(graph->graph))
                        {
                            undirected_vertex_index_t s = boost::source(e, graph->graph);
                            undirected_vertex_index_t t = boost::target(e, graph->graph);

                            foreach(int c, graph->get_vertex_as<pebble_node_t > (s)->get_graph_classes())
                            {
                                if( graph->get_vertex_as<pebble_node_t > (t)->has_graph_class(c) )
                                {
                                    PP_sub[c].graph->add_edge<undirected_edge_t > (same_sub_nodes[s], same_sub_nodes[t]);
                                }
                            }
                        }
                    }
                }

                //////////////////////  PRINTS  //////////////////////////////
                //    for( int i = 0; i < (int)G_sub.size(); ++i )
                //    {
                //        PRX_WARN_S("G_sub (" << i << ")   has v:" << boost::num_vertices(G_sub[i]->graph) << "      e: " << boost::num_edges(G_sub[i]->graph));
                //
                //        foreach(undirected_vertex_index_t v, boost::vertices(G_sub[i]->graph))
                //        {
                //            PRX_INFO_S(G_sub[i]->get_vertex_as<pebble_node_t > (v)->print_point());
                //            PRX_INFO_S("has classes : ");
                //            foreach(int c, G_sub[i]->get_vertex_as<pebble_node_t > (v)->get_graph_classes())
                //            std::cout << c << " , ";
                //            std::cout << std::endl;
                //        }
                //    }
                //    
                //    PRX_ERROR_S("GrAPH CLASSES");
                //    foreach(undirected_vertex_index_t v, boost::vertices(graph->graph))
                //    {
                //        PRX_ERROR_S(graph->get_vertex_as<pebble_node_t > (v)->print_point());
                //        PRX_ERROR_S("has classes : ");
                //        foreach(int c, graph->get_vertex_as<pebble_node_t > (v)->get_graph_classes())
                //                std::cout << c << " , ";
                //        std::cout << std::endl;
                //    }
                //////////////////////////////////////////////////////////////////////////////////

            }

            void kornhauser_tester_t::compute_subproblems(pebble_subproblem_t& problem, int m)
            {
                //    PRX_ERROR_S("*********   compute subproblems **********");
                //    problem.print();
                compute_subgraphs(problem.graph, m);

                size_t sz = PP_sub.size();
                //    PRX_WARN_S("the g_sub size is : " << sz);

                G_sub_blanks.resize(sz);
                for( size_t i = 0; i < sz; ++i )
                {
                    G_sub_blanks[i] = 0;
                }

                pebble_distance_heuristic_t* heuristic = new pebble_distance_heuristic_t(problem.graph, state_space);
                pebble_astar_goal_visitor_t* visitor = new pebble_astar_goal_visitor_t();

                foreach(undirected_vertex_index_t v, boost::vertices(problem.graph->graph))
                {

                    foreach(undirected_vertex_index_t v_adj, boost::adjacent_vertices(v, problem.graph->graph))
                    {

                        foreach(int c, problem.graph->get_vertex_as<pebble_node_t > (v)->get_graph_classes())
                        {
                            if( !problem.graph->get_vertex_as<pebble_node_t > (v_adj)->has_graph_class(c) )
                            {
                                undirected_vertex_index_t v_con = find_closest_connector(problem.graph, v, c);
                                if( v_con != v )
                                {

                                    heuristic->set_new_goal(v_con);
                                    visitor->set_new_goal(v_con);
                                    try
                                    {
                                        astar_search<
                                                undirected_graph_type,
                                                undirected_graph_t,
                                                undirected_vertex_index_t,
                                                pebble_distance_heuristic_t,
                                                pebble_astar_goal_visitor_t
                                                > (problem.graph, v, heuristic, visitor);
                                    }
                                    catch( prx_found_goal_t f_goal )
                                    {
                                        for( undirected_vertex_index_t vp = v_con; vp != v; vp = problem.graph->predecessors[vp] )
                                            problem.graph->get_vertex_as<pebble_node_t > (vp)->add_plank_class(c);

                                        problem.graph->get_vertex_as<pebble_node_t > (v)->add_plank_class(c);
                                    }
                                }
                            }
                        }
                    }
                }
                delete heuristic;
                delete visitor;


                //    PRX_ERROR_S("PLANK CLASSES");
                //    foreach(undirected_vertex_index_t v, boost::vertices(problem.graph->graph))
                //    {
                //        PRX_ERROR_S(problem.graph->get_vertex_as<pebble_node_t > (v)->print_point());
                //        PRX_ERROR_S("has plank classes : ");
                //        foreach(int c, problem.graph->get_vertex_as<pebble_node_t > (v)->get_plank_classes())
                //                std::cout << c << " , ";
                //        std::cout << std::endl;
                //        PRX_WARN_S("has graph classes : ");
                //        foreach(int c, problem.graph->get_vertex_as<pebble_node_t > (v)->get_graph_classes())
                //                std::cout << c << " , ";
                //        std::cout << std::endl;
                //    }

                foreach(undirected_vertex_index_t v_sub, boost::vertices(problem.graph->graph))
                {
                    if( !problem.graph->get_vertex_as<pebble_node_t > (v_sub)->on_plank() )
                    {
                        if( !problem.s_assign.has_robot_on(v_sub) )
                        {
                            //                PRX_INFO_S(problem.graph->get_vertex_as<pebble_node_t>(v_sub)->print_point() <<"  has :" << problem.graph->get_vertex_as<pebble_node_t>(v_sub)->get_graph_classes().size() << "  classes");
                            //                PRX_ASSERT(problem.graph->get_vertex_as<pebble_node_t>(v_sub)->get_graph_classes().size() == 1);

                            foreach(int g_class, problem.graph->get_vertex_as<pebble_node_t > (v_sub)->get_graph_classes())
                            {
                                G_sub_blanks[g_class]++;
                            }
                        }
                    }
                }

                //    foreach(int mi, G_sub_blanks)
                //    {
                //        PRX_INFO_S("The graph has blanks : " << mi);
                //    }
                //    
                //    foreach(undirected_vertex_index_t vindex, boost::vertices(G_sub[0]->graph))
                //    {
                //        PRX_INFO_S(vindex << "   node:" << G_sub[0]->get_vertex_as<pebble_node_t>(vindex)->print_point());
                //    }
                //    
                //    foreach(undirected_edge_index_t eindex, boost::edges(G_sub[0]->graph))
                //    {
                //        PRX_INFO_S("edge:" << eindex << ": [" << boost::source(eindex,G_sub[0]->graph) << " , " << boost::target(eindex,G_sub[0]->graph) <<"]");
                //    }
                //    
                //    PRX_ERROR_S("--------------------");
                //    foreach(undirected_vertex_index_t vindex, boost::vertices(G_sub[1]->graph))
                //    {
                //        PRX_INFO_S(vindex << "   node:" << G_sub[1]->get_vertex_as<pebble_node_t>(vindex)->print_point());
                //    }
                //    
                //    foreach(undirected_edge_index_t eindex, boost::edges(G_sub[1]->graph))
                //    {
                //        PRX_INFO_S("edge:" << eindex << ": [" << boost::source(eindex,G_sub[1]->graph) << " , " << boost::target(eindex,G_sub[1]->graph) <<"]");
                //    }

                foreach(undirected_vertex_index_t v, problem.s_assign.assignments | boost::adaptors::map_keys)
                {
                    if( problem.graph->get_vertex_as<pebble_node_t > (v)->on_plank() )
                    {
                        start_plank.add_assignment(v, problem.s_assign.get_robot(v));
                        end_plank.add_assignment(v, problem.e_assign.get_robot(v));
                    }
                    else
                    {
                        if( !problem.graph->get_vertex_as<pebble_node_t > (v)->on_graph_plank() )
                        {
                            PRX_ASSERT(problem.graph->get_vertex_as<pebble_node_t > (v)->get_graph_classes().size() == 1);
                            int c = problem.graph->get_vertex_as<pebble_node_t > (v)->get_graph_classes()[0];
                            PP_sub[c].s_assign.add_assignment(v, problem.s_assign.get_robot(v));
                            PP_sub[c].e_assign.add_assignment(v, problem.e_assign.get_robot(v));
                        }
                        else
                        {
                            int sub_class = assign_pebble_to_graph(problem.graph, v, problem.s_assign);
                            if( sub_class != -1 )
                            {
                                PP_sub[sub_class].s_assign.add_assignment(v, problem.s_assign.get_robot(v));
                                PP_sub[sub_class].e_assign.add_assignment(v, problem.e_assign.get_robot(v));
                                //                    PP_sub[sub_class].s_assign.print();
                            }
                            else
                            {
                                start_plank.add_assignment(v, problem.s_assign.get_robot(v));
                                end_plank.add_assignment(v, problem.e_assign.get_robot(v));
                                //                    start_plank.print();
                                //                    end_plank.print();
                            }
                        }
                    }
                }

                foreach(pebble_subproblem_t p, PP_sub)
                {
                    p.print();
                }
                PRX_WARN_S("------------ PLANKS --------------");
                start_plank.print();
                end_plank.print();
            }

            bool kornhauser_tester_t::is_graph_trivial(undirected_graph_t* graph)
            {
                return (boost::num_vertices(graph->graph) == 2 && boost::num_edges(graph->graph) == 1);
            }

            bool kornhauser_tester_t::in_same_non_trivial_component(undirected_graph_t* graph, undirected_vertex_index_t vi, undirected_vertex_index_t vj)
            {

                foreach(int c, graph->get_vertex_as<pebble_node_t > (vi)->get_classes())
                {
                    if( graph->get_vertex_as<pebble_node_t > (vj)->has_class(c) )
                        if( G_bic[c] > 2 )
                            return true;
                }
                return false;
            }

            bool kornhauser_tester_t::has_unique_path(undirected_graph_t* graph, undirected_vertex_index_t start, undirected_vertex_index_t goal)
            {
                bool not_unique = false;
                path_length = PRX_INFINITY;
                std::vector<undirected_edge_index_t> path;

                pebble_distance_heuristic_t* heuristic = new pebble_distance_heuristic_t(graph, goal, state_space);
                pebble_astar_goal_visitor_t* visitor = new pebble_astar_goal_visitor_t(goal);

                try
                {
                    astar_search<
                            undirected_graph_type,
                            undirected_graph_t,
                            undirected_vertex_index_t,
                            pebble_distance_heuristic_t,
                            pebble_astar_goal_visitor_t
                            > (graph, start, heuristic, visitor);
                }
                catch( prx_found_goal_t f_goal )
                {
                    path_length = 0;
                    for( undirected_vertex_index_t v = goal; v != start; v = graph->predecessors[v] )
                    {
                        path.push_back(boost::edge(v, graph->predecessors[v], graph->graph).first);
                        path_length++;
                    }
                }

                if( path_length > 1 )
                {

                    foreach(undirected_edge_index_t e, path)
                    {
                        undirected_vertex_index_t s = boost::source(e, graph->graph);
                        undirected_vertex_index_t t = boost::target(e, graph->graph);
                        graph->remove_edge(e);
                        try
                        {
                            astar_search<
                                    undirected_graph_type,
                                    undirected_graph_t,
                                    undirected_vertex_index_t,
                                    pebble_distance_heuristic_t,
                                    pebble_astar_goal_visitor_t
                                    > (graph, start, heuristic, visitor);
                        }
                        catch( prx_found_goal_t f_goal )
                        {
                            not_unique = true;
                        }

                        graph->add_edge<undirected_edge_t > (s, t);
                        if( not_unique )
                        {
                            delete heuristic;
                            delete visitor;
                            return false;
                        }
                    }
                }

                delete heuristic;
                delete visitor;
                return true;
            }

            undirected_vertex_index_t kornhauser_tester_t::find_closest_connector(undirected_graph_t* graph, undirected_vertex_index_t v, int graph_class)
            {
                pebble_bfs_visitor_t* visitor = new pebble_bfs_visitor_t();

                foreach(undirected_vertex_index_t v_con, connectors)
                {
                    if( graph->get_vertex_as<pebble_node_t > (v_con)->has_graph_class(graph_class) )
                    {
                        visitor->add_connectors(v_con);
                        //            PRX_WARN_S("visitor add: " <<  graph->get_vertex_as<pebble_node_t>(v_con)->print_point());
                    }
                }


                try
                {
                    breadth_first_search<
                            undirected_graph_type,
                            undirected_graph_t,
                            undirected_vertex_index_t,
                            pebble_bfs_visitor_t
                            > (*graph, v, v, visitor);
                }
                catch( found_closest_t fc )
                {
                    //        PRX_INFO_S("Start : " << graph->get_vertex_as<pebble_node_t>(v)->print_point());
                    //        PRX_ERROR_S("Found v_connector: " << graph->get_vertex_as<pebble_node_t>(fc.index)->print_point());
                    delete visitor;
                    return fc.index;
                }

                delete visitor;
                return v;
            }

            int kornhauser_tester_t::assign_pebble_to_graph(undirected_graph_t* graph, undirected_vertex_index_t v, pebble_assignment_t& assignment)
            {
                pebble_distance_heuristic_t* heuristic = new pebble_distance_heuristic_t(graph, v, state_space);
                pebble_astar_goal_visitor_t* visitor = new pebble_astar_goal_visitor_t(v);

                foreach(int cl, graph->get_vertex_as<pebble_node_t > (v)->get_plank_classes())
                {

                    foreach(undirected_vertex_index_t v_con, connectors)
                    {
                        PRX_ASSERT(graph->get_vertex_as<pebble_node_t > (v_con)->get_graph_classes().size() == 1);
                        if( cl == graph->get_vertex_as<pebble_node_t > (v_con)->get_graph_classes()[0] )
                        {
                            int N = -1;
                            try
                            {
                                //                    PRX_ERROR_S("STart astar with cl : " << cl <<  "  predecessors[" << v << "] : " << G_sub[cl]->predecessors[v]);
                                //                    PRX_INFO_S(" sub graph has: " << boost::num_vertices(G_sub[cl]->graph) << " vertices and " << boost::num_edges(G_sub[cl]->graph) << " edges");
                                //                    PRX_WARN_S(" graph has: " << boost::num_vertices(graph->graph) << " vertices and " << boost::num_edges(graph->graph) << " edges");                                               
                                astar_search<
                                        undirected_graph_type,
                                        undirected_graph_t,
                                        undirected_vertex_index_t,
                                        pebble_distance_heuristic_t,
                                        pebble_astar_goal_visitor_t
                                        > (*graph, v_con, heuristic, visitor);
                            }
                            catch( prx_found_goal_t f_goal )
                            {
                                //                    PRX_INFO_S("FOUND SOLUTION ASTAR");
                                N = 0;
                                for( undirected_vertex_index_t vp = v; vp != v_con; vp = PP_sub[cl].graph->predecessors[vp] )
                                    if( assignment.has_robot_on(vp) )
                                        N++;

                                if( assignment.has_robot_on(v_con) )
                                    N++;

                                //                    PRX_INFO_S("The pebbles are N:" << N << "and the blanks : " << G_sub_blanks[cl] );                    
                                if( G_sub_blanks[cl] >= N )
                                {
                                    delete heuristic;
                                    delete visitor;
                                    //                        PRX_WARN_S("return cl: " << cl);
                                    return cl;
                                }
                            }
                        }
                    }
                }

                delete heuristic;
                delete visitor;
                //    PRX_WARN_S("return -1");
                return -1;
            }

            bool kornhauser_tester_t::is_polygon(undirected_graph_t* graph)
            {

                foreach(undirected_vertex_index_t v, boost::vertices(graph->graph))
                {
                    if( boost::out_degree(v, graph->graph) != 2 )
                        return false;
                }
                return true;
            }

            bool kornhauser_tester_t::polygon_has_solution(pebble_subproblem_t& problem)
            {
                undirected_vertex_index_t vs = problem.s_assign.get_first_position_with_robot();
                undirected_vertex_index_t vt = vs;

                int rs = problem.s_assign.get_robot(vs);
                int rt = problem.e_assign.get_robot(vt);

                undirected_vertex_index_t v_next = *(boost::adjacent_vertices(vt, problem.graph->graph).first);
                undirected_edge_index_t et = boost::edge(vt, v_next, problem.graph->graph).first;
                undirected_edge_index_t es = et;

                while( rs != rt )
                {
                    boost::tie(vt, et) = next_clockwise_vertex(problem.graph, vt, et);
                    rt = problem.e_assign.get_robot(vt);
                }

                int N = problem.s_assign.size();
                while( N != 0 )
                {
                    if( rs != rt )
                        return false;

                    do
                    {
                        boost::tie(vs, es) = next_clockwise_vertex(problem.graph, vs, es);
                        rs = problem.s_assign.get_robot(vs);
                    }
                    while( rs == -1 );

                    do
                    {
                        boost::tie(vt, et) = next_clockwise_vertex(problem.graph, vt, et);
                        rt = problem.e_assign.get_robot(vt);
                    }
                    while( rt == -1 );

                    N--;
                }


                return true;
            }

            std::pair<undirected_vertex_index_t, undirected_edge_index_t> kornhauser_tester_t::next_clockwise_vertex(undirected_graph_t* graph, undirected_vertex_index_t v, undirected_edge_index_t edge)
            {
                undirected_vertex_index_t v_next;

                if( boost::source(edge, graph->graph) == v )
                    v_next = boost::target(edge, graph->graph);
                else
                    v_next = boost::source(edge, graph->graph);

                foreach(undirected_edge_index_t e, boost::out_edges(v_next, graph->graph))
                {
                    if( e != edge )
                    {
                        return std::make_pair(v_next, e);
                    }

                }

                PRX_LOG_ERROR("Wrong in next_clockwise_vertex");
                return std::make_pair(v, edge);
            }

        }
    }
}
