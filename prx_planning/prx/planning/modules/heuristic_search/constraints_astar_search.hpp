/**
 * @file planning_astar_search.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_CONSTRAINTS_ASTAR_SEARCH_HPP
#define PRX_CONSTRAINTS_ASTAR_SEARCH_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/heuristic_search/astar_search.hpp"
#include "prx/utilities/heuristic_search/constrained_astar_node.hpp"
#include "prx/utilities/heuristic_search/astar_node.hpp"
#include "prx/utilities/heuristic_search/constraints.hpp"

#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/simulation/trajectory.hpp"
#include "prx/simulation/plan.hpp"

#include <set>
#include <boost/range/adaptor/map.hpp>

#include <boost/graph/properties.hpp>
#include <deque>

#include <pluginlib/class_loader.h>


namespace prx
{
    namespace plan
    {
        //TODO: Move this into defs in core utilities?
        /** @brief Determines which mode (in constraints astar search) the astar is running in **/
        enum search_mode_t { STANDARD_SEARCH, LAZY_SEARCH, TRUSTED_MCR, UNTRUSTED_MCR };
        

        /**
         * @anchor constraints_astar_search_t
         * 
         * Flexible implementation of the A* heuristic graph search algorithm. To use, 
         * derive from this class and provide an implementation of the single-goal 
         * heuristic function. The callback functions can also be overridden to give 
         * fine-grained information and control over the search process.
         * 
         * Warning! This is a version that will work ONLY for the undirected graph. 
         * 
         * @brief <b> Flexible implementation of A* search. </b>
         * 
         * @author Athanasios Krontiris
         */
         /*
         * Warning! This is a version that will work ONLY for the undirected graph. 
         * At the time I was implementing this class we had found out that this will work
         * only for the undirected graphs and we were ok with that. 
         */
        class constraints_astar_search_t : public util::astar_search_t         
        {

          public:

            constraints_astar_search_t();
            virtual ~constraints_astar_search_t();
            void clear_memory();

            /** INIT AND SETUPS **/
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);
            virtual void restart();
            virtual void setup_modules(util::undirected_graph_t *g, const util::space_t* state_sp, const util::space_t* control_sp, validity_checker_t* checker, local_planner_t* loc_planner);
            virtual void setup_astar_search(search_mode_t heuristic_search_mode, const util::constraints_t* input_valid_constraints, bool restart_astar = true, int lazy_iters = PRX_INFINITY);
            
            virtual void set_length_multiplier( double new_multiplier );
            virtual void set_constraints_after_search( bool flag = true );

            /** SOLVE **/
            virtual bool solve(const std::vector<util::undirected_vertex_index_t>& starts, const std::vector<util::undirected_vertex_index_t>& goals );
            virtual bool solve(const util::undirected_vertex_index_t& start, const std::vector<util::undirected_vertex_index_t>& goals );
            virtual bool solve(const util::undirected_vertex_index_t& start, util::undirected_vertex_index_t& goal );


            /** GETS **/
            virtual util::undirected_vertex_index_t extract_path(std::deque<util::undirected_vertex_index_t>& vertices);

            virtual void extract_path_constraints(util::constraints_t* constraints);
            
            virtual double get_path_cost() const;
            /**
             * @brief Retrieve the loader for this class instance.
             *
             * @return The pluginlib class loader for creating instances of planners.
             */
            static pluginlib::ClassLoader<constraints_astar_search_t>& get_loader();

          protected:
/** MINCONFLICT ASTAR **/

            virtual util::astar_node_t* generate_new_node(util::undirected_vertex_index_t vertex, util::undirected_vertex_index_t parent, double g, double h);

            virtual void allocate_transient_constraint( util::undirected_edge_t* edge );

            virtual void free_transient_constraints();

            virtual void initialize_vertex(util::undirected_vertex_index_t vertex, util::undirected_vertex_index_t parent, double g, double h);

            virtual bool examine_vertex(util::undirected_vertex_index_t vertex);

            virtual bool discover_vertex(util::undirected_vertex_index_t vertex, util::undirected_vertex_index_t parent, double new_distance, double h);

            virtual bool mcr_node_addition_check( util::undirected_vertex_index_t graph_vertex, util::constraints_t* heap_constraints, double heap_distance );

            virtual bool examine_edge(util::undirected_vertex_index_t vertex, util::undirected_vertex_index_t parent, util::undirected_edge_index_t e, double new_distance);

            virtual void finalize_search(util::undirected_vertex_index_t vertex);

            virtual util::astar_node_t* openset_insert_node(util::undirected_vertex_index_t vertex, util::undirected_vertex_index_t parent, double g, double h);

            virtual double get_node_distance(util::astar_node_t* node);

            /**
             * @brief Recomputes the information on the edge.
             * @details Recomputes the information on the edge. If the graph is not informed
             * about collisions and other information the user could overwrite this function in order to 
             * compute the information that is needed when the mode for the A* is PRX_REPROPAGATE_EDGE_INFO
             * By Default will call the \c reuse_and_validate_edge_info function that will return true if the 
             * edge does not have the updated soft_constraint struct.
             * 
             * @param e The edge that will be examined.
             * @return True if the edge is valid, otherwise false. 
             */
            virtual bool recompute_and_validate_edge_info(util::undirected_vertex_index_t vertex, util::undirected_vertex_index_t parent, util::undirected_edge_index_t e);

            /**
             * @brief Reuses the information on the edge.
             * @details Reuses the information on the edge. A* will just use the information that it is 
             * on the edge. This information is trusted and it will be used when the mode for the 
             * A* is PRX_REUSE_EDGE_INFO
             * 
             * @param e The edge that will be examined.
             * @return True if the edge is valid, otherwise false. 
             */
            virtual bool reuse_edge_info(util::undirected_edge_index_t e, bool transient = false);
            
            
            /** @brief Sets the valid_constraints variable to be the empty_constraints 
             *  @details  Saves the current state of the valid_constraints into the
             *  saved_valid_constraints variable.  Then sets valid_constraints
             *  to be the empty_constraints
             */
            virtual void disable_valid_constraints();
            
            /** @brief Sets the valid_constraints variable to be the saved_valid_constraints 
             *  @details  Restores valid_constraints to whatever has been saved in
             *  saved_valid_constraints (typically after disable_valid_constraints() has been called)
             */
            virtual void reenable_valid_constraints();   
            
            util::constrained_astar_node_t* goal_node;
            search_mode_t _search_mode;

            double length_multiplier;
            
            bool update_constraints;

            util::constraints_t* new_constraints;
            const util::constraints_t* valid_constraints;
            const util::constraints_t* saved_valid_constraints;
            util::constraints_t* empty_constraints;

            /** ASTAR MODULE **/
            const util::space_t* state_space;
            const util::space_t* control_space;
            validity_checker_t* validity_checker;
            local_planner_t* local_planner;

            sim::trajectory_t edge_path;
            sim::plan_t edge_plan;
            util::sys_clock_t _solve_time;
            double total_solve_time;

          private:
            int lazy_iterations;
            util::hash_t< util::abstract_edge_t*, util::constraints_t* > transient_constraints;            
            
            /** @brief The pluginlib loader which is returned by the get_loader() class. */
            static pluginlib::ClassLoader<constraints_astar_search_t> loader;
        };
    }
}

#endif
