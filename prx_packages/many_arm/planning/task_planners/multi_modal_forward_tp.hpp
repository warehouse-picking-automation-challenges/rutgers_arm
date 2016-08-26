/**
 * @file multi_modal_forward_tp.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#pragma once

#ifndef PRX_BFMA_MANIPULATION_TP_HPP
#define PRX_BFMA_MANIPULATION_TP_HPP

#include "planning/graphs/sig_graph.hpp"
#include "planning/graphs/automaton.hpp"
#include "planning/graphs/search_tree.hpp"
#include "planning/modules/tree_open_set.hpp"

#include "prx/utilities/graph/undirected_graph.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"

#include "prx/planning/task_planners/task_planner.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/motion_planners/rrt/rrt_graph.hpp"

#include "../../../baxter/simulation/plants/manipulator.hpp"

namespace prx
{
    namespace util
    {
        class linear_distance_metric_t;
        class astar_open_set_t;
        class astar_node_t;

        unsigned factorial(unsigned n)
        {
            return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
        }

        unsigned combination(unsigned n, unsigned k)
        {
            unsigned denom = factorial(k)*(factorial(n-k));
            return factorial(n)/denom;
        }

    }

    namespace sim
    {
        class manipulation_simulator_t;
    }

    namespace plan
    {
        class motion_planner_t;
    }

    namespace packages
    {
        namespace manipulation
        {
            class manip_sampler_t;
            class movable_body_plant_t;
            class manipulation_specification_t;
        }

        namespace multi_arm
        {
            using namespace baxter;
            using namespace manipulation;

            class hand_off_sampler_t;

            /**
             * Brute force multi-arm manipulation, mostly for testing.
             *
             * @autors Andrew Dobson
             */
            class multi_modal_forward_tp_t : public plan::task_planner_t
            {
            public:
                multi_modal_forward_tp_t();
                ~multi_modal_forward_tp_t();

                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                virtual const util::statistics_t* get_statistics();
                virtual bool succeeded() const;

                virtual void setup();
                virtual bool execute();
                virtual void resolve_query();

            protected:
                //Specific High-Level operations
                void resolve_discrete();
                void resolve_est();

                //Setup functions
                void find_planning_waypoints();
                virtual void find_plants();
                virtual void store_start_state();
                void generate_seeds_from_automaton();
                void setup_grasp_sampler();
                void sample_low_level_states();
                void connect_query_points();
                void add_seeds_to_roadmaps();
                void construct_automaton();
                void post_process_graphs();

                //General Search Functions
                bool plan_top( unsigned record_index, automaton_node_t* source, automaton_node_t* target );

                bool plan_stable_stable( const pose_record_t& source_record, automaton_node_t* source, automaton_node_t* target );
                bool plan_stable_handoff( const pose_record_t& source_record, automaton_node_t* source, automaton_node_t* target, bool to_stable );
                bool plan_handoff_handoff( const pose_record_t& source_record, automaton_node_t* source, automaton_node_t* target );
                const pose_record_t* get_approach_record( util::space_point_t* object_pose, automaton_node_t* target );
                const pose_record_t* get_s_s_record( automaton_node_t* target, const pose_record_t& source_record, bool new_grasp, bool transfer );
                const pose_record_t* get_h_s_record( automaton_node_t* target, const pose_record_t& source_record, unsigned manip_index );
                const pose_record_t* get_h_h_record( automaton_node_t* target, const pose_record_t& source_record, unsigned manip_index, bool same_node );
                bool candidate_record_check( const pose_record_t& source_record, const pose_record_t& test_record );

                bool plan_move(int manip_index, util::space_point_t* start, util::space_point_t* goal, bool ignore_others = false, bool verbose = false);
                bool plan_transfer(int manip_index, unsigned grasp_id, util::space_point_t* start, util::space_point_t* goal, bool ignore_others = false, bool verbose = false);
                bool plan(util::space_point_t* start, util::space_point_t* goal, plan::motion_planning_specification_t* spec, plan::motion_planning_query_t* query, plan::motion_planner_t* planner, bool verbose = false);

                double object_cost_to_go( util::space_point_t* state );
                double object_distance(util::space_point_t* from, util::space_point_t* to);
                void trace_path( util::tree_vertex_index_t target );
                bool state_satisfies_goal( util::space_point_t* state );

                bool extend( const util::tree_vertex_index_t& tree_node );
                bool generate_branch( automaton_node_t* auto_node, search_node_t* search_node, automaton_node_t* adj_node );
                void add_node_to_tree( const util::tree_vertex_index_t& nearest, automaton_node_t* adj_node );

                void branch_discrete( search_node_t* search_node, automaton_node_t* adj_node );
                void reduce_branch_cost(util::tree_vertex_index_t v, double amount);

                //Our discrete search method
                util::tree_vertex_index_t get_min_open_set();

                //Hauser's Forward Search Functions
                util::tree_vertex_index_t select_est_no_heuristic();
                util::tree_vertex_index_t select_est();

                //Heuristics functions
                void compute_heuristics();
                unsigned get_heuristic_index( automaton_node_t* adj_node );

                //I/O Functions
                void serialize_motion_graphs();
                void serialize_automaton();
                void deserialize_automaton();

                //Utility Functions
                void report_statistics();
                void generate_timeout_file();

                virtual void go_to_safe( unsigned manip_index );
                virtual void go_to_start();
                void set_zero_control();

                void add_edge_records_to_nodes();
                void print_automaton();

                void append_plan(int manip_index, unsigned replications = 1);
                void prepend_plan(int manip_index, unsigned replications = 1);
                void append_grasp(int manip_index, unsigned replications, double grasping);
                void prepend_grasp(int manip_index, unsigned replications, double grasping);

                bool is_same_grasp( unsigned manip_index, const pose_record_t& first_record, const pose_record_t& second_record );
                bool is_same_pose( const pose_record_t& first_record, const pose_record_t& second_record );
                bool is_same_pose( const util::space_point_t* first_pose, const util::space_point_t* second_pose, double epsilon = PRX_DISTANCE_CHECK );

                unsigned get_grasp_index( unsigned manip_index, const pose_record_t& record );

                const std::vector< unsigned >& get_common_arms( automaton_node_t* node_a, automaton_node_t* node_b );
                void non_common_arm_set( std::vector< unsigned >& result, automaton_node_t* node_a, automaton_node_t* node_b );
                bool have_common_arms( automaton_node_t* node_a, automaton_node_t* node_b );

                bool index_is_common_manipulator( unsigned index, const std::vector< unsigned >& arms_a, const std::vector< unsigned >& arms_b );
                void arm_union_setminus( std::vector< unsigned >& result, const std::vector< std::vector< unsigned > >& lhs, const std::vector< unsigned >& rhs );

                bool have_interaction( const std::vector< unsigned >& arms, std::vector< util::bounds_t >& bounds, const util::undirected_graph_t& SIG, const std::vector< util::undirected_vertex_index_t >& map );
                bool shared_surface( util::undirected_vertex_index_t a, util::undirected_vertex_index_t b, std::vector< util::bounds_t >& bounds );
                bool shared_space( util::undirected_vertex_index_t a, util::undirected_vertex_index_t b, std::vector< util::bounds_t >& bounds );

                void set_object_pose_from_record( const pose_record_t& record );
                void set_arm_grasping_from_record( unsigned manip_index, const pose_record_t& record );
                void set_arm_released_from_record( unsigned manip_index, const pose_record_t& record );
                void set_arm_retracted_from_record( unsigned manip_index, const pose_record_t& record );

                std::vector< std::vector< unsigned > > generate_combinations(unsigned in_k, const std::vector< unsigned >& max_cc);




                // Members
                automaton_graph_t automaton; //The high-level automaton which contains all of our planning information.  Whoa.
                util::undirected_graph_t SIGs; //Surface commonality spatial interaction graph (SIG)
                util::undirected_graph_t SIGh; //Hand-off spatial interaction graph (SIG)

                // Operational modes
                bool use_automaton_states; //Whether to use the states in the automaton for searches or to re-sample
                bool discrete_search; //True if we use our discrete searching method, false to use Hauser's RRT method
                bool delayed_planning; //True if we are delaying our planning until the very last moment

                bool do_preprocessing; //True if we are doing the pre-processing phase.
                unsigned number_of_samples; //Number of different object poses to try
                unsigned num_grasp_samples; //Number of different grasps to try for each pose

                unsigned approach_attempts; //The number of times to attempt to find approach/depart grasps
                unsigned pause_frames; //The number of frames to have plans pause at the end control

                sim::plan_t saved_plan; //Plan to save things to between calls
                sim::plan_t final_plan; //The plan which holds the result of a "plan_top"


                unsigned reached_record_index; //The index for what record we reached in the planning process
                pose_record_t* reached_record; //The actual record we reached during planning

                std::vector< util::undirected_vertex_index_t > goal_vertices; //All of the high-level nodes which can reach the goal directly.
                std::vector< double > heuristics; //The finally computed heurisitcs for each high-level node

                std::vector< plan::motion_planning_query_t* > planner_queries; //Queries we will use when planning a move
                std::vector< std::vector< plan::motion_planning_query_t* > > transfer_queries; //Queries we wlll use when planning a transfer
                std::vector< plan::motion_planning_specification_t* > move_specifications; //Specifications we will pass to the motion planner (move)
                std::vector< std::vector< plan::motion_planning_specification_t* > > transfer_specifications; //Specification we will pass to the motion planner (transfer)
                manipulation_specification_t* manip_spec; //Stored cast poniter of the input specification

                std::vector< plan::motion_planner_t* > move_planners; //Planners which are used for move planning
                std::vector< std::vector< plan::motion_planner_t* > > transfer_planners; //Planners which are used for transfer planning

                util::space_t* global_state_space; //The full state space
                util::space_t* global_control_space; //The full control space
                std::vector< manipulator_plant_t* > manipulators; //All of the Manipulators
                std::vector< std::string > move_context_names; //Context names for moving the arms without objects
                std::vector< std::string > transfer_context_names; //Context names for each arm manipulating the object
                std::vector< std::string > move_ignoring_context_names; //Context names for moving an arm without the object, while ignoring other arms
                std::vector< std::string > transfer_ignoring_context_names; //Context names for transferring with an arm, while ignoring the other arms
                std::vector< util::space_t* > move_spaces; //A collection of all of the movement spaces in this planning problem
                std::vector< util::space_t* > transfer_spaces; //All of the transfer spaces
                std::vector< movable_body_plant_t* > objects; //List of the objects in the scene
                std::string object_context_name; //Context name which presents only the object

                util::hash_t< std::string, unsigned > manipulator_map; //Mapping from manipulator names to global indices
                std::vector< unsigned > SIG_map; //A mapping from manipulator indices in the manipulators list to the index which should be considered in the SIG
                std::vector< util::undirected_vertex_index_t > SIGs_map; //Map to stable pose SIG vertices
                std::vector< util::undirected_vertex_index_t > SIGh_map; //Map to stable pose SIG vertices

                std::string stable_sig_filename; //Filename where the stable-grasp SIG can be read from
                std::string handoff_sig_filename; //Filename where the handoff SIG can be read from
                std::string automaton_filename;

                double grasp_retraction_distance; //Retraction distance for attempting grasps
                std::vector< double > grasp_zs; //The grasp_z values for all of the manipulators (oh lord how will this work in the general case?)
                util::space_t* object_space; //Space of the object
                unsigned object_k; //The number of manipulators needed to manipulate the object

                bool reached_goal;
                util::tree_vertex_index_t goal_vertex;

                //More heuristic-like stuff?
                bool bias_low_level_samples;

                //EST parameters
                double order_weight;
                double valence_weight;
                double heuristic_weight;
                double selection_weight;
                double arms_weight;
                bool successful_transfer;

                //Discrete Search Parameters
                util::open_set_t< search_node_t > open_set;

                util::space_point_t* global_start_state; //The global start state that is read in from input
                util::space_point_t* object_goal_state; //The goal state for the object
                util::space_point_t* object_state; //A state to use for object positioning/sampling/etc.
                std::vector< util::space_point_t* > safe_states; //The safe configurations for each of the arms

                std::string all_manips_context_name; //Name for the context which has all manipulators plannable

                manipulation_simulator_t* manip_sim; //A cast pointer to the simulator so I can ask for its freakin' mode

                manip_sampler_t* manipulation_sampler; //A pointer to a manipulation sampler to generate manipulation states
                manip_sampler_t* ungrasping_sampler; //A manipulation sampler which will operate in move spaces.
                hand_off_sampler_t* grasp_sampler; //A sampler which can sample grasps of all sorts, including handoffs

                std::vector< util::undirected_vertex_index_t > stable_pose_vis; //All of the vertex indices which correspond to the stable poses
                std::vector< util::undirected_vertex_index_t > handoff_pose_vis; //All of the vertex indices which correspond to the handoff states
                std::vector< util::undirected_vertex_index_t > all_vis; //ALL of the vertex indices
                std::vector< util::undirected_edge_index_t > edge_indices; //All of the edge indices in one easy to access place.

                std::vector< std::vector<sim::state_t*>* > move_seeds;
                std::vector< std::vector< std::vector<sim::state_t*>* > > transfer_seeds;

                //Motion planner things?
                util::tree_t tree;
                util::tree_vertex_index_t start_vertex;
                std::vector< util::tree_vertex_index_t > tree_vis; //All vertex indices for the vertices in the tree.
                sim::state_t* tree_sample_point;
                unsigned point_number;
                unsigned max_points;
                std::vector<sim::state_t*> pre_alloced_points;

                //Statistics
                util::sys_clock_t clock; //Clock for timing when things happen
                unsigned path_transitions; //Number of path transitions in the final plan
                double connect_time; //Time it took to connect the query points to the graph
                double search_time; //Time it took to search and find a solution
                unsigned expansions; //Number of time node was expanded
                double original_object_distance; //The original position distance of the object from the goal position
            };
        }
    }
}

#endif //PRX_BFMA_MANIPULATION_TP_HPP
