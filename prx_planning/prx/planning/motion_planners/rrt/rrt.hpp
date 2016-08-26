/**
 * @file rrt.hpp
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

#ifndef PRX_RRT_HPP
#define	PRX_RRT_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/motion_planners/rrt/rrt_graph.hpp"

namespace prx
{
    namespace plan
    {

        /**
         * @brief <b> Randomly-exploring Random Tree </b>
         * @author Zakary Littlefield
         */
        class rrt_t : public motion_planner_t
        {
          friend class rrtc_t;

          public:
            rrt_t();
            virtual ~rrt_t();

            /**
             * @copydoc motion_planner_t::init(const util::parameter_reader_t*, const util::parameter_reader_t*)
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

            /**
             * @copydoc motion_planner_t::reset()
             */
            virtual void reset();

            /**
             * @copydoc motion_planner_t::setup()
             */
            virtual void setup();

            /**
             * @copydoc motion_planner_t::link_specification(specification_t*)
             */
            virtual void link_specification(specification_t* new_spec);

            /**
             * @copydoc motion_planner_t::step()
             */
            virtual void step();

            /**
             * @copydoc motion_planner_t::execute()
             */
            virtual bool execute();

            /**
             * @copydoc motion_planner_t::link_query()
             */
            virtual void link_query(query_t* new_query);

            /**
             * @copydoc motion_planner_t::resolve_query()
             */
            virtual void resolve_query();

            /**
             * @copydoc motion_planner_t::succeeded() const
             */
            virtual bool succeeded() const;

            /**
             * @copydoc motion_planner_t::serialize()
             */
            virtual bool serialize();

            /**
             * @copydoc motion_planner_t::deserialize()
             */
            virtual bool deserialize();

            /**
             * @copydoc motion_planner_t::get_statistics()
             */
            virtual const util::statistics_t* get_statistics();

          protected:
            std::string file_name;

            void prune_trajectory(sim::plan_t& new_plan, sim::trajectory_t& new_trajectory, sim::state_t* new_end_state=NULL);

            /**
             * @brief Get the \ref rrt_node_t from the tree.
             * @param v The index of the node to return.
             * @return A casted pointer to the node.
             */
            rrt_node_t* get_vertex(util::tree_vertex_index_t v) const;

            /**
             * @brief Get the \ref rrt_edge_t from the tree.
             * @param e The index of the edge to return.
             * @return A casted pointer to the edge.
             */
            rrt_edge_t* get_edge(util::tree_edge_index_t e) const;

            /**
             * Returns the nearest vertex to the given state.
             * @param state The state to find the closest node to.
             * @return The index to the closest node in the tree.
             */
            util::tree_vertex_index_t nearest_vertex(const sim::state_t* state) const;

            /**
             * @brief Samples a point at random or according to goal bias and stores in the variable sample_point
             */
            void random_sample();

            /**
             * @brief Increments the point number and increases the preallocated buffer if required
             */
            void increment_point_number();

            /**
             * Adds a new node in the graph and trying to connect it with the 
             * existing graph. 
             * 
             * @brief Add a new node to the graph.
             *
             * @param n_state The new state that I want to add in the graph.
             * 
             * @return The index for the node in the boost::graph and whether it got added.
             */
             std::pair<bool, util::tree_vertex_index_t> add_node(util::space_point_t* n_state);

             /**
             * @brief Retraces the nodes from the new_v vertex till the root_v or the root of the tree
             *
             * @param plan The plan that is filled up with the plans on the retraced edges.
             *
             * @param new_v The vertex from where we start the retrace
             *
             * @param root_v The vertex till which we retrace.
             */
             void retrace_path(sim::plan_t& plan, util::tree_vertex_index_t new_v, util::tree_vertex_index_t root_v);

             bool check_path_for_collisions_and_constraints(util::constraints_t* constraints, sim::trajectory_t& path_to_evaluate);

            /**
             * @brief The actual tree.
             */
            util::tree_t tree;

            /**
             * @brief An index to the root node.
             */
            util::tree_vertex_index_t start_vertex;

            /**
             * @brief A temporary storage for sampled points.
             */
            sim::state_t* sample_point;

            /**
             * @brief Clock for collecting timing statistics.
             */
            util::sys_clock_t clock;

            /**
             * @brief Keeps the count of iterations.
             */
            int iteration_count;

            /**
             * @copydoc planner_t::update_vis_info() const
             */
            virtual void update_vis_info() const;

            /**
             * @brief Flag to visualize tree.
             */
            bool visualize_tree;
            /**
             * @brief Flag to visualize the solution.
             */
            bool visualize_solution;
            /**
             * @brief Flag that determines if goals are found within a radius.
             */
            bool radius_solution;

            /**
             * @brief Flag that turns on and off collision checking.
             */
            bool collision_checking;

            /**
             * @brief Flag for goal biasing.
             */
            bool goal_biased;

            /**
             * @brief Flag for using the boost random.
             */
            bool use_boost_random;

            /**
             * @brief The goal biasing percentage.
             */
            double goal_bias_rate;

            /**
             * @brief Flag to smooth the RRT solution
             */
            bool smooth_solution;

            /**
             * @brief keeps track of the goal vertex that is satisfied
             */
            std::vector<util::tree_vertex_index_t> satisfied_goal_vertex;

            /**
             * @brief The visualization name for the tree visualization geometries.
             */
            std::string visualization_tree_name;

            /**
             * @brief The visualization name for the solution visualization geometries.
             */
            std::string visualization_solution_name;

            /**
             * @brief The rigid body that determines visualization positions.
             */
            std::string visualization_body;

            /**
             * @brief The color of the solution path.
             */
            std::string solution_color;

            /**
             * @brief The color of the tree.
             */
            std::string graph_color;

            /**
             * @brief A trajectory to store the result of propagations.
             */
            sim::trajectory_t trajectory;

            /**
             * @brief Optimizes memory allocations by storing a large set of allocated points.
             */
            std::vector<sim::state_t*> pre_alloced_points;
            /**
             * @brief The current max number of points.
             */
            unsigned max_points;

            /**
             * @brief In replanning, this list of colors will visualize solution paths differently.
             */
            std::vector<std::string> color_map;

            /**
             * @brief Index into \c color_map
             */
            int solution_number;

            /**
             * @brief Counter for number of points added to the structure.
             */
            unsigned point_number;

            /**
             * @brief Temporary constraint storage
             */
            util::constraints_t* tmp_constraint;

            /**
            * @brief Maximum number of edges sent to visualization. It is updated from const function update_vis_info.
            **/
            mutable int max_edges;
            /**
            * @brief Maximum number of goal points sent to visualization It is updated from const function update_vis_info.
            **/
            mutable int max_goals;
        };

    }
}

#endif
