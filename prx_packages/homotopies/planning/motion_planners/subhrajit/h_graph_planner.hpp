/**
 * @file h_graph_planner.hpp
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

#ifndef PRX_H_GRAPH_PLANNER_HPP
#define	PRX_H_GRAPH_PLANNER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/motion_planners/prm_star/prm_star_statistics.hpp"
#include "planning/motion_planners/subhrajit/h_graph.hpp"
#include "planning/algorithms/homotopies/2d/H_augmented_graph.hpp"
#include "prx/utilities/boost/hash.hpp"

// H-Graph uses complex numbers
#include <complex>

namespace prx
{
    namespace packages
    {
        namespace homotopies
        {


            /**
             * @anchor h_graph_planner_t
             *
             * H-signature graph planner
             * 
             * @brief <b> H-signature graph </b>
             * 
             * @author Andrew Kimmel
             */
            class h_graph_planner_t : public plan::motion_planner_t
            {
              public:
                h_graph_planner_t();
                virtual ~h_graph_planner_t();

                /**
                 * @copydoc motion_planner_t::init(const util::parameter_reader_t*,const util::parameter_reader_t*) 
                 */
                virtual void init(const util::parameter_reader_t* reader,const util::parameter_reader_t* template_reader);

                /**
                 * @copydoc motion_planner_t::reset() 
                 */
                virtual void reset();   

                /** 
                 * @copydoc motion_planner_t::setup() 
                 * 
                 * Will occupy memory for the random_point and the new_control, after 
                 * planning_query has been linked. 
                 */
                virtual void setup();

                /** @copydoc motion_planner_t::step() 
                 * 
                 * The algorithm will add one new node for every call of this function.
                 */     
                virtual void step();

                /** 
                 * @copydoc motion_planner_t::resolve_query() 
                 *
                 * At the end of the resolve_query the algorithm will remove the vertices 
                 * for the start and the goal for this specific query from the graph.
                 */
                virtual void resolve_query();
                
                virtual void approximate_resolve_query(int num_neighbors = 1);

                virtual bool resolve_homotopies(int num_homotopies = 0);
                virtual bool resolve_homotopies(int num_homotopies, std::vector<sim::plan_t>& resolved_plans, std::vector<sim::trajectory_t>& resolved_trajectories);

                
                virtual bool magical_resolve_query(int num_homotopies, const std::vector<sim::state_t*>& invalid_states, double collision_radius, std::vector<sim::plan_t>& resolved_plans, std::vector<sim::trajectory_t>& resolved_trajectories);
                virtual void invalidate_and_resolve_query_mk2(std::vector<sim::state_t*> invalid_states, double collision_radius, double horizon_factor );
                /**
                 * @copydoc motion_planner_t::succeeded() const 
                 */
                virtual bool succeeded() const;     

                /** @brief Returns the set of computed homotopies */
                virtual const std::vector<sim::trajectory_t>& get_homotopic_trajectories();
                /** @brief Returns the corresponding plans for the computed homotopies */
                virtual const std::vector<sim::plan_t>& get_homotopic_plans();
                
                bool serialize();
                bool deserialize();

                void set_connection_properties(int max_n, bool collision_check);
                
                /** Calculate H-signature */
                complex_t calculate_H_signature(sim::trajectory_t* path);
                
                bool is_same_homotopy(complex_t h1, complex_t h2);

              protected:    

                /** Protected functions */
                virtual void update_vis_info() const;

                /**
                 * @brief Generates a collision-free random sample and stores it in random_point.
                 */
                virtual void valid_random_sample();

                /**
                 * Adds a new node in the graph and trying to connect it with the 
                 * existing graph. 
                 * 
                 * @brief Add a new node to the graph.
                 *
                 * @param n_state The new state that I want to add in the graph.
                 * 
                 * @return The index for the node in the boost::graph.
                 */
                virtual util::directed_vertex_index_t add_node(const util::space_point_t* n_state, bool collision_check = true);

                /**
                 * @brief Connects the node v on the existing graph.
                 * 
                 * @param v Its the index of an existing node on the graph.
                 */
                virtual void connect_node( util::directed_vertex_index_t v, bool collision_check = true);

                /**
                 * @brief Connects the node v in a neighbor of radian rad on the existing graph.
                 * 
                 * @param v Its the index of an existing node on the graph.
                 * @param rad The diameter of the neighbor that we need to check in order to connect the new node on the graph.
                 */
                virtual void connect_node( util::directed_vertex_index_t v, double rad , bool collision_check = true);

                void link_node_to_neighbors( util::directed_vertex_index_t v, const std::vector< const util::abstract_node_t* >& neighbors, bool collision_check = true );
                
                void split_edges(double min_length);

                /** ---- Graph discretization things */
                /** @brief Temporary path storage. */
                sim::trajectory_t path1;
                /** @brief Temporary path storage. */
                sim::trajectory_t path2;
                /** @brief Vertex index to refer to the last node added to the graph. */
                util::directed_vertex_index_t v_new;
                /** @brief The planning structure maintained by the PRM. */
                util::directed_graph_t graph;
                /** @brief Temporary storage for random samples. */
                util::space_point_t* random_point;
                /** @brief Temporary plan storage */
                sim::plan_t new_plan;
                /** @brief Temporary plan storage. */
                sim::plan_t new_plan2;
                /** @brief The number of nearest neighbors to attempt connections with. */
                unsigned int k;

                /** @brief The number of edges in the PRM's planning structure. */
                int num_edges;
                /** @brief The number of nodes in the PRM's planning structure. */
                int num_vertices;

                /** @brief The distance heuristic needed for A* searches on the PRM planning structure. */
                h_graph_distance_heuristic* default_heuristic;
                /** @brief The boost visitor funciton used for A* searches. */
                h_graph_astar_visitor* default_visitor;


                /** ---- H-Graph Things ---- */

                std::vector<complex_t> discovered_homotopies;
                double homotopy_threshold;
                int number_explored_homotopies;
                int current_number_homotopies;

                // Allowed homotopy classes (set A)
                std::vector<complex_t> allowed_homotopies;
                // Disallowed homotopy classes (set B)
                std::vector<complex_t> disallowed_homotopies;

                /** Obstacle representative points */
                std::vector<complex_t> representative_points;

                /** Contains all Al values */
                std::vector<complex_t> Al;

                /** Contains computed homotopic classes */
                std::vector<edge_index_list_t> computed_homotopies_list;
                std::vector<sim::trajectory_t> computed_homotopies;
                std::vector<sim::plan_t> computed_plans;

              private:

                /** Obstacle Marker functions */
                complex_t polynomial_analytic_function(complex_t z);

                complex_t calculate_edge_signature(complex_t z1, complex_t z2);

                // Helper function
                void create_trajectories_and_plans();
                void create_trajectories_and_plans(std::vector<sim::plan_t>& resolved_plans, std::vector<sim::trajectory_t>& resolved_trajectories);
                void scale_representative_points();

            //    bool same_homotopy(complex_t h1, complex_t h2);

                void calculate_a_b(); // calculates the powers used in the polynomial marker function
                void calculate_Al(); // calculates Al for each obstacle and fills the vector Al

                complex_t BL, TR; // bottom left corner , top right corner
                int a, b; // the polynomial function powers
                std::vector<int> KL_vals;


                // PRM STUFF
                void update_k( unsigned nr_nodes );

                /** @brief A flag indicating whether PRM should send its computed graph to the visualization node. */
                bool visualize_graph;
                /** @brief A flag indicating whether PRM should send computed solutions to the visualization node. */
                bool visualize_solutions;
                /** @brief A flag indicating whether PRM should send computed solutions to the visualization node. */
                bool visualize_homotopies;
                /** @brief A unique name identifier for the PRM's graph. */
                std::string visualization_graph_name;
                /** @brief A unique name identifier for the PRM's solution path. */
                std::string visualization_solutions_name;
                /** @brief A list of all bodies which the PRM wishes to visualize. */
                std::vector<std::string> visualization_bodies;
                /** @brief A list of colors to use for displaying solution paths. */
                std::vector<std::string> solutions_colors;
                /** @brief The color to visualize the PRM graph in. */
                std::string graph_color;   

                bool BL_set, TR_set;
                
                /** Customization options for graph connections */
                int max_neighbor_links;
                bool collision_check_links;
                
                // Temporary Variables
                sim::plan_t start_plan, end_plan;
                sim::trajectory_t start_connection, end_connection;
                sim::state_t* temp_state;
                
                /** NEW STUFF */
                /** @brief Stores the original cost for the edges in the PRM planning structure. */
                std::vector<double> original_costs_for_edges;
                /** @brief Stores all the edges which have had their edge weights artificially modified. */
                std::vector<util::directed_edge_index_t> altered_edges;
                util::hash_t<int, util::directed_edge_index_t> altered_edge_map;
                double weight_factor;
                double split_edge_length, max_step_length;

            };
        }
    }
}

#endif
