// /**
//  * @file manipulation_tp_t.hpp
//  *
//  * @copyright Software License Agreement (BSD License)
//  * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick 
//  * All Rights Reserved.
//  * For a full description see the file named LICENSE.
//  *
//  * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
//  *
//  * Email: pracsys@googlegroups.com
//  */

// #pragma once

// #ifndef PRX_MANIPULATION_TP_HPP
// #define	PRX_MANIPULATION_TP_HPP

// #include "prx/utilities/definitions/defs.hpp"
// #include "prx/utilities/definitions/sys_clock.hpp"

// #include "prx/utilities/boost/hash.hpp"
// #include "prx/utilities/math/configurations/config.hpp"
// #include "prx/simulation/systems/obstacle.hpp"

// #include "prx/planning/task_planners/task_planner.hpp"
// #include "prx/planning/motion_planners/prm_star/prm_star_graph.hpp"
// #include "../../../baxter/simulation/plants/manipulator.hpp"
// #include "../../../manipulation/simulation/plants/movable_body_plant.hpp"

// namespace prx
// {

//     namespace util
//     {
//         class bounds_t;
//         class goal_state_t;
//         class statistics_t;

//     }

//     namespace plan
//     {
//         class motion_planning_specification_t;
//         class motion_planning_query_t;
//     }

//     namespace packages
//     {
//         namespace manipulation
//         {
//             class manip_sampler_t;
//         }
//         namespace cloud_manipulation
//         {
//             using namespace baxter;
            
//             class cloud_manipulation_specification_t;
//             class cloud_manipulation_query_t;

//             struct pose_t
//             {

//                 sim::state_t* state;
//                 //This is the set of the directed_vertex_index_t of the grasped graph 
//                 //that can grasp this pose.
//                 std::vector < sim::state_t* > grasped_set;
//                 std::vector < sim::state_t* > raised_set;
//                 //This is the set of the ungrasped points for this pose. 1-1 connection with 
//                 //the grasped set.
//                 std::vector < sim::state_t* > ungrasped_set;
//                 std::vector < sim::state_t* > retracted_set;
//                 //This is the set of the trajectories for reaching, grasping and lifting the 
//                 //object given the states to grasp this pose;
//                 std::vector< sim::plan_t > reaching_plans;
//                 std::vector< sim::plan_t > retracting_plans;
//                 std::vector< sim::trajectory_t > reaching_paths;
//                 std::vector< sim::trajectory_t > retracting_paths;
//                 //The paths reversed for placing an object at this pose;
//                 std::vector< sim::plan_t > lifting_plans;
//                 std::vector< sim::plan_t > placing_plans;
//                 std::vector< sim::trajectory_t > lifting_paths;
//                 std::vector< sim::trajectory_t > placing_paths;

//                 bool equal(const util::space_t* object_state_space, const pose_t & other) const
//                 {
//                     return object_state_space->equal_points(state, other.state, PRX_DISTANCE_CHECK);
//                 }

//                 bool equal(const util::space_t* object_state_space, const sim::state_t * other) const
//                 {
//                     return object_state_space->equal_points(state, other, PRX_DISTANCE_CHECK);
//                 }

//             };

//             //            struct grasping_motion_info_t
//             //            {
//             //
//             //                unsigned pose;
//             //                util::directed_vertex_index_t v_pose;
//             //                std::vector<bool> is_valid;
//             //                std::vector<OA_astar_query_t*> motions;
//             //
//             //                grasping_motion_info_t()
//             //                {
//             //                    v_pose = NULL;
//             //                    pose = -1; //That will be big number because pose is unsigned                    
//             //                }
//             //
//             //                void init(int in_pose, util::directed_vertex_index_t v)
//             //                {
//             //                    pose = in_pose;
//             //                    v_pose = v;
//             //                }
//             //
//             //                std::string print() const
//             //                {
//             //                    std::stringstream output(std::stringstream::out);
//             //                    output << "Pose:  " << pose << "    v_pose: " << v_pose << std::endl;
//             //                    for( unsigned i = 0; i < motions.size(); ++i )
//             //                    {
//             //                        output << is_valid[i] << ") " << motions[i]->start_index << "->" << motions[i]->end_index << "   path: " << motions[i]->path.size() << "   plan:" << motions[i]->plan.size() << "    constraints: " << motions[i]->pose_constraints.size() << "   : ";
//             //
//             //                        foreach(unsigned index, motions[i]->pose_constraints)
//             //                        {
//             //                            output << index << ",";
//             //                        }
//             //
//             //                        output << std::endl;
//             //                    }
//             //
//             //                    return output.str();
//             //                }
//             //            };

//             /**
//              * Manipulation task planner. 
//              * 
//              * 
//              * @autors Athanasios Krontiris
//              */
//             class cloud_manipulation_tp_t : public plan::task_planner_t
//             {

//               public:

//                 cloud_manipulation_tp_t();
//                 virtual ~cloud_manipulation_tp_t();

//                 virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

//                 /**
//                  * @copydoc motion_planner_t::reset() 
//                  */
//                 virtual void reset();

//                 /**
//                  * @copydoc motion_planner_t::link_world_model()
//                  */
//                 virtual void link_world_model(plan::world_model_t * const model);

//                 /**
//                  * @copydoc motion_planner_t::get_statistics()
//                  */
//                 virtual const util::statistics_t* get_statistics();

//                 /**
//                  * @copydoc planner_t::link_specification(specification_t*)
//                  */
//                 virtual void link_specification(plan::specification_t* new_spec);

//                 /**
//                  * @copydoc motion_planner_t::link_query()
//                  */
//                 virtual void link_query(plan::query_t* new_query);

//                 /** 
//                  * @copydoc motion_planner_t::setup() 
//                  * 
//                  * Will occupy memory for the random_open_point and the new_control, after 
//                  * planning_query has been linked. 
//                  */
//                 virtual void setup();

//                 /**
//                  * @copydoc motion_planner_t::execute()
//                  */
//                 virtual bool execute();
//                 virtual bool execute2();


//                 /**
//                  * @copydoc motion_planner_t::succeeded() const 
//                  */
//                 virtual bool succeeded() const;

//                 /** 
//                  * @copydoc motion_planner_t::resolve_query() 
//                  *
//                  * At the end of the resolve_query the algorithm will remove the vertices 
//                  * for the start and the goal for this specific query from the graph.
//                  */
//                 virtual void resolve_query();

//                 std::pair<const std::vector<pose_t>*, int> get_grasping_info() const;

//                 bool serialize();
//                 bool deserialize();
//                 void manipulate(unsigned start_pose_index, unsigned target_pose_index, cloud_manipulation_query_t* manip_query);
//                 void manipulate(unsigned start_pose_index, unsigned target_pose_index);
//                 void manipulate(std::vector<double> start_pose, std::vector<double> end_pose);
//                 bool compute_grasp(cloud_manipulation_query_t* manip_query);

//                 //                virtual void set_grasped_graph(const std::string graph_name);
//                 //                
//                 //                virtual void set_ungrasped_graph(const std::string graph_name);
//                 //
//                 //              protected:
//                 //                
//                 //                virtual bool execute2();

//               protected:

//                 /**
//                  * @copydoc planner_t::set_param( const std::string&, const boost::any& )
//                  */
//                 virtual void set_param(const std::string& parameter_name, const boost::any& value);


//                 /**
//                  * @copydoc planner_t::update_vis_info() const
//                  */
//                 virtual void update_vis_info() const;

//                 //                virtual bool valid_random_sample(sim::state_t*& object_state);
//                 //
//                 //
//                 //                void serialize_trajectory(std::ofstream& fout, arrange_edge_t* edge);
//                 //                void deserialize_trajectory(std::ifstream& fin, arrange_edge_t* edge);
//                 //                std::vector< double > get_cup_data_from_point(util::space_point_t* point);
//                 //
//                 //
//                 //                virtual util::directed_vertex_index_t add_node(util::directed_graph_t& mode_graph, const util::space_point_t* n_point, util::distance_metric_t* mode_metric);
//                 //                /**
//                 //                 * This function will add a new n_state point but also the related grasping and releasing 
//                 //                 * 
//                 //                 * 
//                 //                 * @param n_point The point that the manipulator has the object within its fingers.
//                 //                 * @param grasped_p The point that the manipulator is grasping the object.
//                 //                 * @param released_p The point that the manipulator will move forward towards grasping the object.
//                 //                 * @return 
//                 //                 */
//                 //                virtual void add_nodes(const util::space_point_t* grasped_point, const util::space_point_t* released_point, const util::space_point_t* raised_point, const util::space_point_t* retracted_point, int index);
//                 //
//                 //                /**
//                 //                 * @brief Tries to link the node v with the neighbor nodes in the vector neighbors.
//                 //                 * 
//                 //                 * @param v Its the index of an existing node on the graph, that we want to connect on the graph.
//                 //                 * @param neighbors The nodes that we will try to connect with the node v.
//                 //                 */
//                 //                virtual void link_node_to_neighbors(util::directed_graph_t& mode_graph, util::directed_vertex_index_t v, const std::vector< const util::abstract_node_t* >& neighbors);


//                 std::string manipulator_path;
//                 manipulator_plant_t* _manipulator;
//                 /** @brief Random manipulation sampling module. Samples both manipulator's state and object's state */
//                 manipulation::manip_sampler_t* manip_sampler;
//                 /** @brief The state space over the manipulator that the manipulation task planner is using. */
//                 const util::space_t* manip_state_space;
//                 /** @brief The control space over the manipulator that the manipulation task planner is using. */
//                 const util::space_t* manip_control_space;
//                 /** @brief The safe state for the manipulator.*/
//                 sim::state_t* safe_state;
//                 /** @brief The control that will bring the manipulator to the safe state. Because the manipulator is a rigid body the safe_control = safe_state.*/
//                 sim::control_t* safe_control;

//                 sim::state_t* tmp_manip_state;
//                 sim::control_t* tmp_manip_ctrl;
//                 std::vector<double> tmp_control_vec;

//                 /** @brief The state space over the objects that the task planner is using*/
//                 const util::space_t* object_state_space;
//                 /** @brief The state space over the manipulator and the object that the task planner is using*/
//                 const util::space_t* mo_space;
//                 /** @brief The state space for the real objects*/
//                 const util::space_t* real_object_space;

//                 std::string ungrasped_graph_file_name;
//                 std::string grasped_graph_file_name;
//                 bool serialize_grasped_graph;
//                 bool serialize_ungrasped_graph;
//                 bool deserialize_grasped_graph;
//                 bool deserialize_ungrasped_graph;
//                 bool graph_builder;
//                 bool tree_based_planning_mode;
//                 bool best_manipulation_combination;
//                 bool tree_set_ungrasped;
//                 bool tree_set_grasped;

//                 std::string ungrasped_name;
//                 std::string grasped_name;

//                 /** @brief The query that will be used for the graphs to get the path 
//                  *  that connects two poses.
//                  */
//                 plan::motion_planning_query_t* graph_query;
//                 plan::motion_planning_query_t* graph_query2;

//                 util::goal_state_t* first_part_goal;
//                 util::goal_state_t* second_part_goal;
//                 util::goal_state_t* third_part_goal;

//                 /** @brief The specification for the motion planners*/
//                 plan::motion_planning_specification_t* graph_specification;

//                 /** @brief The specification for this manipulation problem */
//                 cloud_manipulation_specification_t* specs;

//                 /** @brief The query for this manipulation problem */
//                 cloud_manipulation_query_t* manip_query;

//                 /** @brief Reads from input the name of the planning context that is for the manipulator only*/
//                 std::string pc_name_manipulator_only;
//                 /** @brief Reads from input the name of the planning context that is for the object*/
//                 std::string pc_name_object_only;
//                 /** @brief Reads from input the name of the planning context that is for the manipulator and the object*/
//                 std::string pc_name_manipulator_with_object;
//                 /** @brief Reads from input the name of the planning context that is for the manipulator and the object is in active space*/
//                 std::string pc_name_manipulator_with_active_object;
//                 /** @brief Reads from input the name of the planning context that is for the real world*/
//                 std::string pc_name_real_world;

//                 std::string pc_name_grasp_planning;

//                 /** @brief A flag indicating whether PRM should send its computed graph to the visualization node. */
//                 bool visualize_graph;

//                 /** @brief Temporary storages for random samples. */
//                 sim::state_t* released_point; //manip state space
//                 sim::state_t* retracted_point; //manip state space
//                 sim::state_t* grasped_point; //manip and object state space
//                 sim::state_t* raised_point; //manip and object state space

//                 sim::state_t* initial_pose; //Imaginary object state space
//                 sim::state_t* goal_pose; //Imaginary object state space 

//                 sim::state_t* real_initial_poses; //Real object state space.
//                 sim::state_t* real_object_point;
//                 sim::state_t* real_initial_object_point;

//                 /** @brief Temporary path storage. */
//                 sim::trajectory_t retract_path;
//                 /** @brief Temporary path storage. */
//                 sim::trajectory_t raise_path;
//                 /** @brief Temporary plan storage. */
//                 sim::plan_t retract_plan;
//                 /** @brief Temporary plan storage. */
//                 sim::plan_t raise_plan;

//                 sim::plan_t resolve_query_plan_1;
//                 sim::plan_t resolve_query_plan_2;
//                 sim::plan_t resolve_query_plan_3;
//                 sim::plan_t resolve_query_min_plan;

//                 util::space_t* stable_pose_space;
//                 sim::state_t* stable_pose_point;
//                 std::vector<double*> stable_pose_memory;

//                 util::space_point_t* object_point;
//                 std::vector<double> object_pos;
//                 std::vector<double> real_object_point_vec;

//                 std::vector<pose_t> poses_set;
//                 int poses_set_length;
//                 double min_total_path_found;
//                 std::string cloud_statistics_file;

//                 int augment_iterations;
//                 int graph_generation_mode;
//                 //                ////////////////////////////// OLD VARIABLES /////////////////////
//                 //
//                 //                /** @brief Temporary path storage. */
//                 //                sim::trajectory_t edge_path;
//                 //                /** @brief Temporary path storage. */
//                 //                sim::trajectory_t retract_path;
//                 //                /** @brief Temporary path storage. */
//                 //                sim::trajectory_t raise_path;
//                 //                /** @brief Temporary plan storage */
//                 //                sim::plan_t edge_plan;
//                 //                /** @brief Temporary plan storage. */
//                 //                sim::plan_t retract_plan;
//                 //                /** @brief Temporary plan storage. */
//                 //                sim::plan_t raise_plan;
//                 //                /** @brief Vertex index to refer to the last node added to the graph. */
//                 //                util::directed_vertex_index_t v_new;
//                 //                /** @brief The planning structure maintained by the arrange PRM for the configurations that 
//                 //                 * the manipulator is not grasping an object. */
//                 //                util::directed_graph_t graph;
//                 //                /** @brief The planning structure maintained by the arrange PRM for the configurations that 
//                 //                 * the manipulator is grasping an object. */
//                 //                util::directed_graph_t grasped_graph;
//                 //                /** @brief The number of nearest neighbors to attempt connections with. */
//                 //                unsigned int k;
//                 //
//                 //                /** @brief The number of edges in the PRM's planning structure. */
//                 //                int num_edges;
//                 //                /** @brief The number of nodes in the PRM's planning structure. */
//                 //                int num_vertices;
//                 //
//                 //                int _mode;
//                 //
//                 //
//                 //
//                 //
//                 //                std::vector<grasp_set_t> grasping_set;
//                 //                int grasping_set_length;
//                 //
//                 //                double raise_distance;
//                 //                double retract_distance;
//                 //
//                 //                const util::space_t* full_state_space;
//                 //                util::space_point_t* full_state_point;
//                 //                std::vector<double> full_state_pos;
//                 //                util::space_point_t* first_leg_full_state_point;
//                 //                util::space_point_t* second_leg_full_state_point;
//                 //                util::space_point_t* third_leg_full_state_point;
//                 //
//                 //                std::pair<double, double> near_point;
//                 //
//                 //
//                 //                manipulation_sampler_t* manip_sampler;
//                 //                util::distance_metric_t* grasped_metric;
//                 //                util::distance_metric_t* pose_metric;
//                 //                util::distance_metric_t* grasped_pose_metric;
//                 //
//                 //
//                 //                util::quaternion_t tmp_quat;
//                 //                util::quaternion_t tmp_quat2;
//                 //
//                 //                util::space_t* random_sample_space;
//                 //                util::space_point_t* random_sample_point;
//                 //                double _x;
//                 //                double _y;
//                 //                double _z;
//                 //                double _tx;
//                 //                double _ty;
//                 //                double _tz;
//                 //                std::vector<double*> random_sample_memory;
//                 //
//                 //                util::space_t* pose_space;
//                 //                util::space_point_t* pose_point;
//                 //                double _pose_x;
//                 //                double _pose_y;
//                 //                std::vector<double*> pose_memory;
//                 //                std::vector<unsigned> new_poses;
//                 //                bool keep_new_edges;
//                 //                std::vector<arrange_edge_t*> new_edges;
//                 //                unsigned old_poses;
//                 //
//                 //                int object_dim;
//                 //
//                 //                mutable std::vector<double> manip_pos;
//                 //                std::vector<double> manip_control;
//                 //                std::vector<const util::abstract_node_t*> neighbors_links;
//                 //
//                 //
//                 //                cloud_manipulation_query_t* manip_query;
//                 //                OA_astar_query_t* oa_query;
//                 //                OA_astar_query_t* tmp_oa_query;
//                 //                OA_astar_query_t* first_leg_oa_query;
//                 //                OA_astar_query_t* second_leg_oa_query;
//                 //                OA_astar_query_t* third_leg_oa_query;
//                 //                util::directed_vertex_index_t v_safe;
//                 //                sim::control_t* safe_control;
//                 //
//                 //                //Profiling a bit: have a system clock
//                 util::sys_clock_t _clock;
//                 double statistics_time;
//                 std::vector< std::vector<double> > init_poses,tgt_poses;
//                 std::vector<util::bounds_t*> bounds_grasped,bounds_ungrasped;
                
//                 //                std::vector< arrange_edge_t* > mod_edges;
//                 //                bool astar_shortest_path_mode;
//                 //                bool astar_minimum_conflict_mode;
//                 //                bool ignore_constrained_paths;
//                 //                double penalty_multiplier;
//                 //                double OA_penalty;
//                 //
//                 //                plan::world_model_t* world_model;

                
//               private:
//                 util::config_t tmp_config;


//                 bool valid_move(sim::plan_t& plan, sim::trajectory_t& path, const sim::state_t* manip_start, const sim::state_t* start, util::config_t& goal_config);
//                 bool valid_grasp();
//                 int similar_pose(sim::state_t* pose);
                

//                 //                void expand_graphs();
//                 //
//                 //                util::directed_vertex_index_t get_vertex_at(unsigned index, const util::directed_graph_t& ongraph);
//                 //                double OA_edge_cost(const pr_node_t& source, const arrange_edge_t* edge, const std::set<unsigned>& pebble_poses, const std::set<unsigned>& ignore_constraints);
//                 //                bool in_the_same_connected_component(const util::directed_graph_t& mode_graph, util::directed_vertex_index_t start, const std::set< util::directed_vertex_index_t >& goals);
//                 //                void obstacle_aware_astar(OA_astar_query_t* solution, util::directed_vertex_index_t start, const std::set< util::directed_vertex_index_t >& goals, const util::directed_graph_t& graph, const util::hash_t<std::string, unsigned>& collision_map, const std::set<unsigned>& pebble_poses, const std::set<unsigned>& ignore_constraints);
//                 //                void generate_edge_trajectory(arrange_edge_t* e);
//                 //                void compute_nominal(arrange_edge_t* e);
//             };

//         }
//     }
// }


// #endif	


// //bool manipulation_tp_t::valid_random_sample(state_t*& object_state)
// //{
// //    int failures = -1;
// //    do
// //    {
// //        failures++;
// //        if( manip_sampler->sample(object_state, manip_state_space, grasped_point, released_point) )
// //        {
// //            //                        PRX_DEBUG_COLOR("the grasped point  : " << state_space->serialize_point(grasped_point, 8), PRX_TEXT_GREEN);
// //            //                        PRX_DEBUG_COLOR("the released_point : " << state_space->serialize_point(released_point, 8), PRX_TEXT_GREEN);
// //            if( validity_checker->is_valid(grasped_point) && validity_checker->is_valid(released_point) )
// //            {
// //                double x, y, z;
// //                _manipulator->get_end_effector_offset_configuration(tmp_config, grasped_point);
// //                tmp_config.get_position(x, y, z);
// //                tmp_config.set_position(x, y, z + raise_distance);
// //                raise_path.clear();
// //                raise_plan.clear();
// //                manip_state_space->copy_from_point(grasped_point);
// //                if( valid_move(raise_plan, raise_path, grasped_point, tmp_config) )
// //                {
// //
// //                    if( grasped_point->memory.back() == 1 )
// //                    {
// //                        state_space->copy_from_point(raise_path.at(0));
// //                        full_state_space->copy_to_point(full_state_point);
// //                        if( full_state_point->memory.back() == -1 )
// //                        {
// //                            full_state_point->memory.back() = manip_query->object_state_pos.first;
// //                            full_state_space->copy_from_point(full_state_point);
// //                            state_space->copy_to_point(raise_path.at(0));
// //                        }
// //                        //                                    else
// //                        //                                    {
// //                        //                                        PRX_ASSERT(false);
// //                        //                                    }
// //                    }
// //
// //                    state_space->copy_from_point(raise_path.back());
// //                    full_state_space->copy_to_point(full_state_point);
// //                    full_state_point->memory.back() = -1;
// //                    full_state_space->copy_from_point(full_state_point);
// //                    state_space->copy_to_point(raised_point);
// //
// //                    //                                for(int i = 0; i < raise_path.size(); ++i)
// //                    //                                    PRX_DEBUG_COLOR("The raise_path (" << i << "):" << state_space->serialize_point(raise_path.at(i),5), PRX_TEXT_CYAN);
// //                    //                                PRX_DEBUG_COLOR("the raised_point  : " << state_space->serialize_point(raised_point, 6), PRX_TEXT_GREEN);
// //                    //                                PRX_DEBUG_COLOR("raised is good) plan: " << raise_plan.size() << "   path: " << raise_path.size(), PRX_TEXT_GREEN);
// //                    manip_query->_manipulator->get_end_effector_offset_configuration(tmp_config, released_point, 0, 0, -retract_distance);
// //                    retract_path.clear();
// //                    retract_plan.clear();
// //                    state_space->copy_from_point(grasped_point);
// //                    for( int i = 0; i < state_space->get_dimension(); ++i )
// //                        manip_pos[i] = grasped_point->at(i);
// //                    manip_pos.back() = 0;
// //
// //                    state_space->set_from_vector(manip_pos, retracted_point);
// //                    if( valid_move(retract_plan, retract_path, retracted_point, tmp_config) )
// //                    {
// //                        //                                    for(int i = 0; i < retract_path.size(); ++i)
// //                        //                                        PRX_DEBUG_COLOR("The retract_path (" << i << "):" << state_space->serialize_point(retract_path.at(i),5), PRX_TEXT_CYAN);
// //                        object_pos[2] = -70;
// //                        state_space->copy_from_point(retract_path.back());
// //                        object_space->set_from_vector(object_pos);
// //                        state_space->copy_to_point(retracted_point);
// //                        //state_space->copy_point(retracted_point, retract_path.back());                                    
// //
// //                        //PRX_DEBUG_COLOR("the retracted_point  : " << state_space->serialize_point(retracted_point, 8), PRX_TEXT_GREEN);
// //                        //PRX_DEBUG_COLOR("retract is good) plan: " << retract_plan.size() << "   path: " << retract_path.size(), PRX_TEXT_CYAN);
// //                        return true;
// //                    }
// //                }
// //            }
// //        }
// //    }
// //    while( failures < manip_query->max_random_failures ); // || !validity_checker->is_valid(released_point) || !validity_checker->is_valid(retracted_point) || !validity_checker->is_valid(grasped_point) );
// //
// //    PRX_DEBUG_COLOR("DID NOT GET a good random sample :", PRX_TEXT_GREEN);
// //    return false;
// //}