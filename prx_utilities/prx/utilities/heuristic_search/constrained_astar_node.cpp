/**
 * @file constrained_astar_node.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/utilities/heuristic_search/constrained_astar_node.hpp"

namespace prx
{
    namespace util
    {
        constrained_astar_node_t::constrained_astar_node_t()
        {
            vertex = NULL;
            f = 0;
            g = 0; 
            h = 0;
            
            constraints = NULL;
        }

        constrained_astar_node_t::constrained_astar_node_t(util::undirected_vertex_index_t vertex) : astar_node_t(vertex, 0, 0)
        {
            this->vertex = vertex;
            
            constraints = NULL;
        }

        constrained_astar_node_t::constrained_astar_node_t(util::undirected_vertex_index_t vertex, double g, double h) : astar_node_t(vertex, g, h)
        {
            this->g = g;
            this->h = h;
            
            constraints = NULL;
        }

        constrained_astar_node_t::constrained_astar_node_t(const constrained_astar_node_t & n) : astar_node_t(n.vertex, n.g, n.h)
        {
            constraints = n.constraints;
            path = n.path;
        }

        constrained_astar_node_t::~constrained_astar_node_t()
        {
        }

        const constrained_astar_node_t& constrained_astar_node_t::operator=(const constrained_astar_node_t& other)
        {
            vertex = other.vertex;
            f = other.f;
            if( constraints != NULL && other.constraints != NULL )
            {
                *constraints = *other.constraints;
            }
            path = other.path;
            g = other.g;
            h = other.h;
            return (*this);
        }

        bool constrained_astar_node_t::operator<(const astar_node_t& n) const
        {
            const constrained_astar_node_t& node = dynamic_cast<const constrained_astar_node_t&>(n);

            //TODO: Test to see if having this code is a speed-up in the average case
            // if( *constraints == *node.constraints )
            //     return f < node.f;

            if (*constraints < *node.constraints)
                return true;
            if (*node.constraints < *constraints)
                return false;
            return f < node.f;
        }


        void constrained_astar_node_t::add_constraints(const constraints_t* new_constraints)
        {
            PRX_ASSERT( constraints != NULL );
            constraints->merge(new_constraints);
        }

        bool constrained_astar_node_t::has_constraints(const constraints_t* valid_constraints)
        {
            if( valid_constraints == NULL || constraints == NULL )
            {
                return false;
            }
            return constraints->has_intersection(valid_constraints);
        }

        bool constrained_astar_node_t::path_has_vertex(util::undirected_vertex_index_t v)
        {
            return std::find(path.begin(), path.end(), v) != path.end();
        }

        void constrained_astar_node_t::merge(const constrained_astar_node_t* node)
        {
            if( constraints != NULL && node->constraints != NULL )
            {
                constraints->merge(node->constraints);
            }
            path = node->path;
        }

        void constrained_astar_node_t::merge(const constrained_astar_node_t* node, double g, double h)
        {
            merge(node);
            set_costs(g, h);
        }

        std::string constrained_astar_node_t::print_constraints() const
        {
            std::stringstream output(std::stringstream::out);
            
            constraints != NULL ? output << constraints->print() : "--";
            
            return output.str();
        }

        std::string constrained_astar_node_t::print() const
        {
            std::stringstream output(std::stringstream::out);
            output << "f:" << f << " g:" << g << " path:" << path.size() << " constraints: " << print_constraints();
            return output.str();
        }
        
    }
}
