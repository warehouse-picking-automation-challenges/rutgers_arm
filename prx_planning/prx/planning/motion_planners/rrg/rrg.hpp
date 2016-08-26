#pragma once

#ifndef PRX_RRG_HPP
#define PRX_RRG_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/modules/heuristic_search/constraints_astar_search.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/motion_planners/rrg/rrg_graph.hpp"
#include "prx/utilities/boost/hash.hpp"

#include "prx/planning/motion_planners/rrg/rrg_statistics.hpp"

namespace prx
{
    namespace plan
    {
        class pno_criterion_t;

        class rrg_t : public motion_planner_t
        {

          public:
            rrg_t();
            virtual ~rrg_t();

            /**
             * @copydoc motion_planner_t::init(const util::parameter_reader_t*,const util::parameter_reader_t*) 
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

            /**
             * @copydoc motion_planner_t::reset() 
             */
            virtual void reset();

            /**
             * @copydoc motion_planner_t::link_specification(specification_t*) 
             */
            void link_specification(specification_t* new_spec);
            
            /** 
             * @copydoc motion_planner_t::setup() 
             * 
             * Will occupy memory for the random_point and the new_control, after 
             * planning_query has been linked. 
             */
            virtual void setup();

            /**
             * @copydoc motion_planner_t::execute() 
             * If the motion planner has a file to deserialize then it reads the graph from there instead of building a new graph. 
             *              
             * 
             * @return True if the execution was ok, otherwise false. 
             */
            virtual bool execute();

            /** 
             * @copydoc motion_planner_t::step() 
             * 
             * The algorithm will add one new node for every call of this function.
             */
            virtual void step();

            //TODO: Link query to pass down the mode and constraint information?
            /**
             * @copydoc motion_planner_t::link_query()
             *
             * The link query additionally links the search mode and the active constraints.
             */
            virtual void link_query(query_t* new_query);

            /** 
             * @copydoc motion_planner_t::resolve_query() 
             *
             * At the end of the resolve_query the algorithm will remove the vertices 
             * for the start and the goal for this specific query from the graph.
             */
            virtual void resolve_query();


            /**
             * @copydoc motion_planner_t::succeeded() const 
             */
            virtual bool succeeded() const;

            /**
             * @copydoc motion_planner_t::serialize() const 
             */
            virtual bool serialize();

            /**
             * @copydoc motion_planner_t::deserialize() const 
             */
            virtual bool deserialize(std::string file);
            
            virtual int get_number_of_roadmap_vertices()
            {
                return boost::num_vertices(graph.graph);
            }
            
            virtual int get_number_of_roadmap_edges()
            {
                return boost::num_edges(graph.graph);
            }

            /**
             * @copydoc motion_planner_t::get_statistics() const 
             */
            virtual const util::statistics_t* get_statistics();

            friend class pno_criterion_t;
            
          protected:
            /** @brief Temporary path storage. */
            sim::trajectory_t new_path;
            /** @brief Vertex index to refer to the last node added to the graph. */
            util::undirected_vertex_index_t v_new;
            /** @brief The planning structure maintained by the RRG. */
            util::undirected_graph_t graph;
            /** @brief Temporary storage for random samples. */
            util::space_point_t* random_point;
            /** @brief Temporary plan storage */
            sim::plan_t new_plan;
            /** @brief The number of nearest neighbors to attempt connections with. */
            unsigned int k;

            // TODO This will become the constrained_astar
            /** @brief The astar algorithm used for searching the graph*/
            constraints_astar_search_t* astar;

            /** @brief The number of edges in the RRG's planning structure. */
            int num_edges;
            /** @brief The number of nodes in the RRG's planning structure. */
            int num_vertices;
            /** @brief The total number of attempted sample generations. */
            int num_generated;
            /** @brief Whether to be running for Probabilistic Near-Optimality */
            bool pno_mode;
            /** @brief succeeded condition for the RRG Motion Planner*/
            int number_of_nodes;

            util::hash_t<unsigned, util::undirected_vertex_index_t> node_map;
            util::hash_t<unsigned, util::undirected_edge_index_t> edge_map;
            std::vector<int> roadmaps_bounds;
            std::vector<std::string> deserialization_files;
            int miniumum_connections;

            /**
             * @copydoc planner_t::update_vis_info() const
             */
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
            virtual std::pair<bool,util::undirected_vertex_index_t> add_node(util::space_point_t* n_state, bool is_resolving);

            /**
             * @brief Connects the node v on the existing graph.
             * 
             * @param v Its the index of an existing node on the graph.
             */
            virtual void connect_node(util::undirected_vertex_index_t v, bool is_resolving);

            /**
             * @brief Tries to link the node v with the neighbor nodes in the vector neighbors.
             * 
             * @param v Its the index of an existing node on the graph, that we want to connect on the graph.
             * @param neighbors The nodes that we will try to connect with the node v.
             */
            virtual void link_node_to_neighbors(util::undirected_vertex_index_t v, const std::vector< const util::abstract_node_t* >& neighbors);


            /**
             * @brief Updates the k value for the k-prm*.
             * 
             * @param nr_nodes The number of the nodes in the graph.
             */
            virtual void update_k(unsigned nr_nodes);
            
            /**
             * @brief Checks if the trajectory on the edge is valid or not. 
             * 
             * Checks if the trajectory on the edge is valid or not. 
             * 
             * @param path The trajectory that has to be checked.
             */
            bool is_valid_trajectory(const sim::trajectory_t& path);
            
            //TODO: Function to remove the start and goals from the roadmap
            virtual void remove_start_and_goals( util::undirected_vertex_index_t v_start, std::vector< util::undirected_vertex_index_t >& v_goals );
            virtual void remove_vertex( util::undirected_vertex_index_t v );
            virtual void trace_vertices( const std::deque< util::undirected_vertex_index_t >& path_vertices );

            // Stitching and connected components functions
            int print_components();
            void clean_components( unsigned graph_size );
            bool connects_components(util::undirected_vertex_index_t added_vertex);
            bool minimum_connections_satisfied();
            bool discover_candidate_neighbors( util::space_point_t* point, std::vector< const util::abstract_node_t* >& neighbor_set );
            bool validity_check_candidate_edges( util::space_point_t* point, std::vector< const util::abstract_node_t*>& neighbor_set, std::vector< bool >& valid_edges );
            unsigned component( const util::abstract_node_t* node );


            /** @brief A flag indicating whether RRG should send its computed graph to the visualization node. */
            bool visualize_graph;
            /** @brief A flag indicating whether RRG should send computed solutions to the visualization node. */
            bool visualize_solutions;
            /** @brief A unique name identifier for the RRG's graph. */
            std::string visualization_graph_name;
            /** @brief A unique name identifier for the RRG's solution path. */
            std::string visualization_solutions_name;
            /** @brief A list of all bodies which the RRG wishes to visualize. */
            std::vector<std::string> visualization_bodies;
            /** @brief A list of colors to use for displaying solution paths. */
            std::vector<std::string> solutions_colors;
            /** @brief The color to visualize the RRG graph in. */
            std::string graph_color;
            int execute_num;
            bool serialize_plan;
            bool using_rrg_sampler;
            std::vector<int> component_connections;

            /** @brief A temporary flag to check whether we are performing collisions for the query points */
            bool no_collision_query_type;

            /** @brief Threshold for similar nodes.*/
            double similarity_threshold;
            /** @brief Checks if we need to remove the start node after we finish with the query.*/
            bool remove_start;
            /** @brief Checks if we need to remove the goal nodes after we finish with the query.*/
            std::vector<bool> remove_goals;

        };

    }
}

#endif
