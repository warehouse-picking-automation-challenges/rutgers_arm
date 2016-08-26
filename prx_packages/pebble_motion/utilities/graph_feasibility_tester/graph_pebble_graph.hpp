/**
 * @file graph_pebble_graph.hpp
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

#ifndef PRX_GRAPH_PEBBLE_GRAPH_HPP
#define	PRX_GRAPH_PEBBLE_GRAPH_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "utilities/tree_feasibility_tester/tree_pebble_graph.hpp"


namespace prx 
{
    namespace packages
    {
        namespace pebble_motion
        {



/**
 * The graph node that keeps the information for the graph_feasibility
 * algorithm for Pebble Motion Problems on Graphs. It's almost the same as 
 * the nodes for the tree_feasibility problem, but it also keeps the information
 * if the node is transshipment node or not
 */
class graph_pebble_node_t : public tree_pebble_node_t
{

  public:

    graph_pebble_node_t() : tree_pebble_node_t()
    {
        //tree_pebble_node_t::tree_pebble_node_t();
        transshipment = false;
        on_mcbc = 0;
        on_path = 0;
        on_path2 = 0;
    
    }
    
    ~graph_pebble_node_t(){}

    void init_node(const util::space_t* space, const std::vector<double>& vec)
    {
        tree_pebble_node_t::init_node(space,vec);
        transshipment = false;
    }       
    
    virtual std::string print_point(const util::space_t* space, unsigned int prec = 3) const
    {
        std::stringstream out(std::stringstream::out);
        out << space->print_point(point,prec) << "    transshipment: " << transshipment;
        return out.str();
    }
       
    bool transshipment;   
    unsigned int on_mcbc;
    unsigned int on_path;
    unsigned int on_path2;
    util::undirected_vertex_index_t other_pred;

};

        }
    }
}

#endif	// PRX_GRAPH_PEBBLE_GRAPH_HPP
