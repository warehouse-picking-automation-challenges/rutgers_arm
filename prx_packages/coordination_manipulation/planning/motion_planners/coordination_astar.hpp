/**
 * @file coordination_astar.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_COORDINATION_ASTAR_HPP
#define PRX_COORDINATION_ASTAR_HPP

#include "prx/planning/modules/heuristic_search/astar_module.hpp"

namespace prx
{
    namespace packages
    {
        namespace coordination_manipulation
        {
            /**
             * A* search for generating coordination roadmaps for two manipulators
             * 
             * @author Andrew Kimmel
             * 
             */
            class coordination_astar_t : public plan::astar_module_t
            {

                public:

                    enum coordination_heuristic_type_t
                    {
                        BIAS_RIGHT, BIAS_LEFT, JOINT_BIAS, ANY_BIAS
                    };

                    coordination_astar_t();
                    ~coordination_astar_t();

                    virtual bool examine_edge(util::undirected_vertex_index_t start_index, util::undirected_edge_index_t edge, util::undirected_vertex_index_t end_index);
                    virtual bool examine_vertex(util::undirected_vertex_index_t v);
                    virtual double heuristic(util::undirected_vertex_index_t current, util::undirected_vertex_index_t goal);
                    virtual double heuristic(util::undirected_vertex_index_t current, const std::vector<util::undirected_vertex_index_t>& goals);
                    virtual bool check_if_goal_found(util::undirected_vertex_index_t potential_goal, util::undirected_vertex_index_t actual_goal);
                    virtual bool solve(util::undirected_vertex_index_t start, const std::vector<util::undirected_vertex_index_t>& goals);
                    
                    virtual void set_coordination_problem(double x_axis, double y_axis, coordination_heuristic_type_t bias);

                protected:

                    double x_axis_goal, y_axis_goal;
                    coordination_heuristic_type_t bias_type;
                    bool append_actual_goal;
            };
        }

    }
}

#endif