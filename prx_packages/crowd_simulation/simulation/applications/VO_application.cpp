/**
 * @file vo_application.cpp
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
#include "simulation/applications/VO_application.hpp"

#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"

#include "prx/simulation/systems/obstacle.hpp"
#include "prx/simulation/communication/visualization_comm.hpp"

#include "simulation/graph/nav_node.hpp"

#include "simulation/plants/pedestrian.hpp"
#include "simulation/sensing/PABT_sensing_model.hpp"
#include "simulation/sensing/navigation_graph_sensor.hpp"
#include "simulation/controllers/path_follow_controller.hpp"
#include "simulation/controllers/behavior_controller.hpp"

#include <map>
#include <pluginlib/class_list_macros.h>
#include <boost/range/adaptor/map.hpp>
#include <boost/graph/connected_components.hpp>
#include <yaml-cpp/yaml.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <ros/ros.h>

#define MSEC_2_SEC 0.001

PLUGINLIB_EXPORT_CLASS( prx::packages::crowd::VO_application_t, prx::sim::application_t)

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {

        namespace crowd
        {
            struct nav_queue_node_t
            {
                nav_node_t* node;
                nav_node_t* back_pointer;
                unsigned door;
                
                nav_queue_node_t( nav_node_t* self, nav_node_t* bp, unsigned source_door )
                {
                    node = self;
                    back_pointer = bp;
                    door = source_door;
                }
            };

            VO_application_t::VO_application_t()
            {
                vo_sensing = NULL;
                selected = NULL;
                initialized = false;
                navigation_space = NULL;
                num_of_responses = 1;
                
                state_vec.resize(3);
                state_vec[0] = 500;
                state_vec[1] = 0;
                state_vec[2] = -50;

                frame_id = 0;
                MSEC_2_FRAMES = 0;
                SEC_2_FRAMES = 0;
                msg_index = 0;

                spawn_time = communication_time = propagate_time = publish_state_time = update_node_time = update_node_id = frame_time = saturated_time = 0;
                saturated_frames = 0;

                num_of_agents_in_total = 0;
                total_computation_time = 0;
                total_simulation_time = 0;
                update_once = false;
                evacuation_frame = PRX_INFINITY;
                evacuation_plan = false;

                debug_colors.push_back("blue");
                debug_colors.push_back("red");
                debug_colors.push_back("yellow");
                debug_colors.push_back("green");
                debug_colors.push_back("teal");
                debug_colors.push_back("purple");
                debug_colors.push_back("grey");
                debug_colors.push_back("pink");
                debug_colors.push_back("dark_green");
                debug_colors.push_back("magenta");
                debug_colors.push_back("brown");
                debug_colors.push_back("indigo");
                debug_colors.push_back("orange");
                debug_colors.push_back("black");
            }

            VO_application_t::~VO_application_t()
            {
                //Have to clean out the override points
                foreach( undirected_vertex_index_t v, boost::vertices( navigation_graph.graph ) )
                {
                    nav_node_t* node = navigation_graph.get_vertex_as< nav_node_t >( v );
                    foreach( space_point_t* point, node->grandfather_override | boost::adaptors::map_values )
                    {
                        navigation_space->free_point( point );
                    }
                }

                if(export_data)
                {
                
                    //First, clear out our file streams
                    for( unsigned i=0; i<file_streams.size(); ++i )
                    {
                        file_streams[i]->flush();
                        
                        file_streams[i]->close();
                        file_streams[i]->clear();

                        delete file_streams[i];
                    }

                    if(stats_stream.is_open())
                    {
                        stats_stream.flush();
                        stats_stream.close();
                        stats_stream.clear();
                    }


                    if(waiting_time_stream.is_open())
                    {
                        waiting_time_stream.flush();
                        waiting_time_stream.close();
                        waiting_time_stream.clear();
                    }

                    if(moe_file_stream.is_open())
                    {
                        moe_file_stream.flush();
                        moe_file_stream.close();
                        moe_file_stream.clear();
                    }

                    if(simulation_file_stream.is_open())
                    {
                        simulation_file_stream.flush();
                        simulation_file_stream.close();
                        simulation_file_stream.clear();
                    }
                }

                for( unsigned i=0; i<ramps.size(); ++i )
                {
                    delete ramps[i];
                }

                if( navigation_space != NULL )
                {
                    delete navigation_space;
                }
                
                for( unsigned i=0; i<prox_elements.size(); ++i )
                {
                    delete prox_elements[i];
                }
                
                for( unsigned i=0; i<pq_nodes.size(); ++i )
                {
                    delete pq_nodes[i];
                }             

            }

            void VO_application_t::init(const parameter_reader_t * const reader)
            {
                // ros::NodeHandle node("global");
                // ros::NodeHandle msg_node("~");

                stats_clock.reset();
                double time_stat = 0;
                PRX_DEBUG_COLOR(" === Begin VO application init === \n", PRX_TEXT_CYAN);

                application_t::init(reader);
                PRX_DEBUG_COLOR("  application has been initialized ", PRX_TEXT_GREEN);
                hindered_escalator_cost = reader->get_attribute_as<double>("hindered_escalator_cost", 6);
                hindered_stair_cost = reader->get_attribute_as<double>("hindered_stair_cost", 18);

                MSEC_2_FRAMES = 0.001/sim::simulation::simulation_step;
                SEC_2_FRAMES = 1/sim::simulation::simulation_step;
                PRX_PRINT("simStep: " << sim::simulation::simulation_step << "     MSEC_2_FRAMES:" << MSEC_2_FRAMES, PRX_TEXT_CYAN);
                export_data = reader->get_attribute_as<bool>("export_data", true);

                gather_systems();

                PRX_ASSERT(vo_sensing != NULL);

                world_structure.init( reader );

                if(!reader->has_attribute("folder_path"))
                    PRX_FATAL_S("No global folder path is specified!");

                folder_path = reader->get_attribute("folder_path");

                if( reader->has_attribute("agent_radius") )
                {
                    agent_radius = reader->get_attribute_as<double>("agent_radius");
                    PRX_PRINT("Initializing using an agent radius: " << agent_radius, PRX_TEXT_RED);
                }
                else
                {
                    PRX_FATAL_S("No agent radius specified in the VO application, cannot compute minkowski obstacles!!");
                }
                
                if( reader->has_attribute("obstacle_metric") )
                {
                    obstacle_metric = reader->initialize_from_loader< distance_metric_t >("obstacle_metric", "prx_utilities");
                }
                else
                {
                    PRX_FATAL_S("Missing Obstacle Metric in the VO application!");
                }

                minimum_obstacle_neighbors = reader->get_attribute_as< unsigned >("minimum_obstacle_neighbors", 4);

                // =================================
                //   Setup scenario
                // =================================
                PRX_PRINT("SIMULATION STEP : " << sim::simulation::simulation_step, PRX_TEXT_LIGHTGRAY);
                //Default is 24h = 4320000 frames                
                end_frame_id = reader->get_attribute_as<unsigned>("end_time",86400000) * MSEC_2_FRAMES;// sim::simulation::simulation_step;
                frame_id = reader->get_attribute_as<double>("start_time", 0) * MSEC_2_FRAMES;// sim::simulation::simulation_step; 
                evacuation_frame = reader->get_attribute_as<unsigned>("evacuation_time",PRX_INFINITY) * MSEC_2_FRAMES;
                frame_rate_output = reader->get_attribute_as<int>("frame_rate_output",5);

                // =================================
                //   Data for Parallel version
                // =================================
                node_id = reader->get_attribute_as<int>("node_id", 0);
                num_of_nodes = reader->get_attribute_as<int>("num_of_nodes", 1);
                spawned_agents_per_node.resize(num_of_nodes,0);
                agent_states_pub = node.advertise<prx_simulation::pabt_node_msg> ("node_"+ int_to_str(node_id) +"/agent_states", 2, false); 
                for(int i = 0; i<num_of_nodes; ++i)
                {
                    if (i != node_id)
                    {
                        node_subs.push_back(node.subscribe("node_" + int_to_str(i) +"/agent_states", 2, &VO_application_t::update_sensing_model, this, ros::TransportHints().reliable().tcpNoDelay(true))); 
                    }
                }

                // node_id = reader->get_attribute_as<int>("node_id", 0);
                // num_of_nodes = reader->get_attribute_as<int>("num_of_nodes", 1);
                // spawned_agents_per_node.resize(num_of_nodes,0);
                // agent_states_pub = node.advertise<prx_simulation::pabt_node_msg> ("node_"+ int_to_str(node_id) +"/agent_states", 1, true);
                // frames_from_nodes.resize(num_of_nodes, -1);
                // for(int i = 0; i<num_of_nodes; ++i)
                // {                    
                //     if( i != node_id)
                //     {
                //         ros::CallbackQueue *my_queue = new ros::CallbackQueue();
                //         std::string topic_name = "node_" + int_to_str(i) +"/agent_states";
                //         ros::SubscribeOptions ops = ros::SubscribeOptions::create<prx_simulation::pabt_node_msg>(topic_name, 1, boost::bind(&VO_application_t::update_sensing_model, this, _1), ros::VoidPtr(), my_queue);
                //         node_subs.push_back(node.subscribe(ops));
                //         my_callback_queues.push_back(my_queue);
                //         // node_subs.push_back(node.subscribe("node_" + int_to_str(i) +"/agent_states", 1, &VO_application_t::update_sensing_model, this, ros::TransportHints().reliable().tcpNoDelay(true)))); 
                //     }
                // }
                
                // ==================================
                //   Load the navigation graph
                // ==================================
                PRX_PRINT("Load the navigation graph using an agent radius: " << agent_radius, PRX_TEXT_CYAN);
                time_stat = stats_clock.measure();
                load_nav_graph( reader );
                time_stat = stats_clock.measure() - time_stat;
                PRX_PRINT(":: (" << node_id <<") Load nav graph takes: " << time_stat << " seconds." , PRX_TEXT_LIGHTGRAY);

                // =====================================
                //   Locate structures in the building
                // =====================================
                time_stat = stats_clock.measure();
                multifloor_sim = reader->get_attribute_as< bool >( "multifloor", true );
                if( multifloor_sim )
                {
                    find_structures();
                }
                time_stat = stats_clock.measure() - time_stat;
                PRX_PRINT(":: (" << node_id <<") Compute Ramps takes: " << time_stat << " seconds." , PRX_TEXT_LIGHTGRAY);

                // ==================================================
                //   Clean up the graph and remove small components
                // ==================================================
                clean_up_components( reader );

                // ===================================
                //   Setup Near-Triangle information
                // ===================================
                find_nearby_triangles();

                // ======================================
                //   Compute Minkowski information
                // ======================================
                obstacle_metric->link_space( navigation_space );
                //Compute Minkowski obstacles, and optionally the nearest obstacle annotations
                compute_static_obstacle_vertices();

                // =======================================
                //  Give the controllers the minkowski obstacles now that they have been formed
                // =======================================
                for( unsigned i=0; i<agent_pool.size(); ++i )
                {
                    agent_pool[i]->VO_controller->set_obstacles( minkowski_vertices );                    
                }

                // ==========================================
                //   Assign each agent the world structure
                // ==========================================
                for( unsigned i=0; i<plants.size(); ++i )
                {
                    pedestrian_t* agent = dynamic_cast< pedestrian_t* >(plants[i]);
                    agent->set_world_structure( &world_structure );
                }

                // ================================================
                //   Assign navigation g primitives to the sensor
                // ================================================
                //Get from the vo_sensing the navigation_graph sensor
                nav_sensor = vo_sensing->graph_sensor;
                //Give it all the things
                nav_sensor->link_navigation_primitives( &navigation_graph, metric, all_agents );
                if(num_of_nodes > 1)
                    nav_sensor->set_num_of_triangles(triangle_counter);

                if(export_data)
                    open_file_output();
               // visualize_nav_graph();
                
                // ==================================
                //   Read in semantics information
                // ==================================
                time_stat = stats_clock.measure();
                if(reader->has_attribute("semantics_filename"))
                    load_semantics_from_file(reader->get_attribute("semantics_filename"));
                else
                    load_semantics(reader);
                time_stat = stats_clock.measure() - time_stat;
                PRX_PRINT(":: (" << node_id <<") Load all the semantics in takes: " << time_stat << " seconds." , PRX_TEXT_LIGHTGRAY);

                // =================================
                //   Compute annotations
                // =================================
                time_stat = stats_clock.measure();
                collect_annotations();
                time_stat = stats_clock.measure() - time_stat;
                PRX_PRINT(":: (" << node_id <<") Collects annotations takes: " << time_stat << " seconds." , PRX_TEXT_LIGHTGRAY);

                // =================================
                //   Read in agent OD information
                // =================================
                time_stat = stats_clock.measure();
                load_agent_paths(reader);
                time_stat = stats_clock.measure() - time_stat;
                PRX_PRINT("Load agents' paths takes : " << time_stat, PRX_TEXT_GREEN);

                // ==================================================
                //   Init Queue Manager and read queues information
                // ==================================================

                if( reader->has_attribute("queue_filename"))
                {
                    std::string filename = folder_path + reader->get_attribute("queue_filename");
                    init_queue_manager_from_file(filename);
                    PRX_PRINT("Initializing queue_manager " , PRX_TEXT_GREEN);
                    visualize_queue_points();
                }

                //Use something like that to get the closest triangle on the queue.

                // std::vector< double > start_of_the_queue; you will read that from the csv file
                //new_goal is a space_point_t*/state_t* that you will crate for the start_of_the_queue vector
                // const nav_node_t* start_node = dynamic_cast< const nav_node_t* >( metric->single_query( new_goal ) );                
                // start_node = nav_sensor->find_triangle(start_node,  start_of_the_queue);
                
                simulator_running = reader->get_attribute_as<bool>("autostart", true);

                std::string stats_filename = folder_path + "/stats_node" + int_to_str(node_id) + ".csv";
                stats_stream.open(stats_filename.c_str());
                //stats_stream << "Frame , Active Agents , Behavior C. , Path Follower C. , Collision-Avoid C. , Total per Agent , Sensing , Spawn , Propagate , Publish , Update Node , Waiting , Frame time , Saturated Frame Time \n";
                std::string waiting_stats_filename = folder_path + "/waiting_time_node_" + int_to_str(node_id) + ".csv";
                waiting_time_stream.open(waiting_stats_filename.c_str());
                //waiting_time_stream << "Frame , Starts , |A| , Spins 1st , 1st Node , When 1st , Update 1st , 1st Frame , Spins 2nd , 2nd Node , When 2nd , Update 2nd , 2nd Frame , Total Spins , spins_time , spin_avg , FullWaitingTime , Full , ros::start , ros::publish , ros::wait , ros::done , ros::waiting_time , ros::Full\n";

                PRX_PRINT("Start frame : " << frame_id << "   End Frame: " << end_frame_id << "    evacuation_frame: " << evacuation_frame, PRX_TEXT_LIGHTGRAY);
                if(frame_id != 0)
                {
                    start_sim_with_agents(); 
                }

                if( display_triangles )
                {
                    visualize_triangles( );
                }
                if( display_segments )
                {
                    visualize_segments( );
                }
                // visualize_elevator_associations();
                // world_structure.print();
                PRX_PRINT("Finished initialization!!!!!!", PRX_TEXT_GREEN);
                PRX_PRINT("All init took: " << stats_clock.measure(), PRX_TEXT_LIGHTGRAY);

            }

            // Initializing the queue manager
            void VO_application_t::init_queue_manager_from_file(const std::string filename)
            {
                queue_manager = new queue_managers_t();
                std::ifstream queue_file(filename.c_str());
                std::string line;
                char delimiter=',';
                std::string local_region_name,queue_name, queue_angle, queue_fluctuation, queue_floor, queue_x, queue_y;
                int q_id = 1;
                if(queue_file.is_open())
                {   
                    getline(queue_file,line);
                    while(getline(queue_file,line))
                    {
                        std::vector<std::string> region_properties;
                        std::string each_property;
                        std::string sub_properties;
                        boost::tie(each_property, sub_properties) = split_path(line,delimiter);
                        do
                        {
                            region_properties.push_back(each_property);
                            boost::tie(each_property, sub_properties) = split_path(sub_properties,delimiter);
                        }while(each_property !="");

                        //PRX_PRINT("region_properties.size():"<<region_properties.size(),PRX_TEXT_BLUE);
                        if(region_properties.size()==region_properties_row_length || region_properties.size()==region_properties_row_length - 1 )
                        {
                            //PRX_PRINT("we are looking for q_region from the world structure name:"<<region_properties[region_name_for_queue], PRX_TEXT_MAGENTA);
                            q_region = world_structure.get_region(region_properties[region_name_for_queue]);

                            if(q_region != NULL)
                            {
                                queue = get_queue_from_region_properites(region_properties, q_id);
                                if(queue!=NULL)
                                {
                                    PRX_PRINT("INSERTING QUEUE "<< queue->queue_name,PRX_TEXT_LIGHTGRAY);
                                    queue_manager->insert_queue(queue);
                                    queue->queue_manager = queue_manager;
                                    ++q_id;
                                    for(int i=0;i<10;i++)
                                    {                                
                                        queue_manager->generate_slots(queue,1);
                                        //PRX_PRINT("GENERATED SLOT:"<<i,PRX_TEXT_LIGHTGRAY);
                                    }
                                    q_region->set_queue(queue);
                                    PRX_PRINT("queue:"<<queue->queue_name<<" set for region:"<<q_region->name,PRX_TEXT_CYAN);
                                }
                            }
                       }
                        else
                        {
                            PRX_PRINT("There is a wrong input at line "<<q_id<<"for the queues, given size:"<<region_properties.size()<<" expected size:"<<region_properties_row_length,PRX_TEXT_RED);
                            PRX_PRINT(line<<"each_property:"<<each_property<<"sub_properties:"<<sub_properties,PRX_TEXT_RED);
                        }
                    }
                }
                else
                {
                    PRX_PRINT("File cannot be read "<<filename, PRX_TEXT_RED);                
                }
            }

            // Create a new queue with the given properties
            queue_t* VO_application_t::get_queue_from_region_properites(const std::vector<std::string> &region_properties, int queue_id)
            {
                string queue_name = region_properties[0];
                double angle = boost::lexical_cast<double>(region_properties[1]);
                double fluctuation = boost::lexical_cast<double>(region_properties[2]);
                double first_turn_dist = boost::lexical_cast<double>(region_properties[3]);
                double floor_level = boost::lexical_cast<double>(region_properties[4]);
                double first_turn_direction = boost::lexical_cast<double>(region_properties[7]);
                double spacing = boost::lexical_cast<double>(region_properties[8]);
                points_t q_start_point;
                q_start_point.set(boost::lexical_cast<double>(region_properties[5]),boost::lexical_cast<double>(region_properties[6]),boost::lexical_cast<double>(region_properties[4]));
               
                // initialize queue with necessary details
                queue = new queue_t(queue_id);
                queue->init(q_start_point,queue_name, angle, fluctuation,floor_level, first_turn_dist,first_turn_direction, spacing);
                nav_node = get_nav_node_from_point(q_start_point);
                
                if(nav_node!=NULL)
                {
                    //If the first point has changed because buffer lines.
                    if(queue_manager->update_start_point(q_start_point, nav_node,queue))
                    {
                        // Set queue state from the queue point
                        set_queue_state_from_point(q_start_point);
                        // Find triangle using previous triangle information
                        nav_node = nav_sensor->find_triangle(nav_node,current_queue_state);
                    }
                    // Add start triangle details
                    if(nav_node!=NULL)
                    {                            
                        queue->add_start_triangle(q_start_point,const_cast<nav_node_t*>(nav_node));
                    }
                    PRX_PRINT("Triangle for queue:"<<queue_name<<" and region:"<<region_properties[region_name_for_queue]<<" is:"<<navigation_space->print_point(nav_node->point, 5), PRX_TEXT_BROWN);
                }
                else
                {
                    PRX_PRINT("Cannot Set Triangle for queue"<<queue_name, PRX_TEXT_CYAN);
                }
                return queue;
             }

            // Get the navigation node closest to the given point
            const nav_node_t* VO_application_t::get_nav_node_from_point(points_t point)
            {
                set_queue_state_from_point(point);
                navigation_space->set_from_vector(current_queue_state, queue_query_point);
                nav_node = dynamic_cast<  const nav_node_t* >( metric->single_query( queue_query_point ) ) ;
                return find_queue_triangle_slot(nav_node,  current_queue_state);
            }

            void VO_application_t::set_queue_state_from_point(points_t point)
            {
                current_queue_state[0] = point.x;
                current_queue_state[1] = point.y;
                current_queue_state[2] = point.z;
            }

            // Returns the triangle node for the given nav node
            const nav_node_t* VO_application_t::find_queue_triangle_slot( const nav_node_t* start, const std::vector< double >& current_state )
            {
                //If start node is a triangle then we are done
                if(start->has_triangle())
                {
                    return start;
                }

                //Keep a list of nav nodes we have tested (closed list)
                std::vector<const nav_node_t* > checked_nav_nodes;
                //Also need a list that is a frontier of nav nodes (open list)
                //std::deque< std::pair< const nav_node_t*, unsigned > > frontier;
                std::deque<const nav_node_t* > frontier;
                //frontier.push_back(std::pair< const nav_node_t*, unsigned >( neighbor, 0 ) );
                frontier.push_back(start);
                
                while(!frontier.empty())
                {
                    /*std::pair< const nav_node_t*, unsigned >& front = frontier.front();
                    const nav_node_t* neighbor = front.first;
                    unsigned depth = front.second;
                    */
                    cur_nav_node = frontier.front();
                    frontier.pop_front();
                    checked_nav_nodes.push_back( cur_nav_node );

                    //Check for new neighbors
                    foreach( undirected_vertex_index_t v, boost::adjacent_vertices( cur_nav_node->index, navigation_graph.graph ) )
                    {
                        neighbor = navigation_graph.get_vertex_as< nav_node_t >( v );
                        
                        //If this neighbor is a triangle node and is in the same triangle of start node
                        if( neighbor->has_triangle() && neighbor->point_in_triangle( current_state ) )
                        {
                            return neighbor;
                        }
                    
                        // If we have not checked this nav node yet
                        if( std::find( checked_nav_nodes.begin(), checked_nav_nodes.end(), neighbor ) == checked_nav_nodes.end() )
                        {
                            // Add it to the frontier
                            //frontier.push_back( std::pair< const nav_node_t*, unsigned >( neighbor, depth +1 ) );
                            frontier.push_back(neighbor);
                        }
                    }
                }

                PRX_PRINT("Agent got completely lost, could not find its triangle!",PRX_TEXT_BROWN);
                PRX_PRINT("Agent at ::  " << current_state[0] << ", " << current_state[1] << ", " << current_state[2], PRX_TEXT_LIGHTGRAY );
                PRX_PRINT("Checked Triangles: (" << checked_nav_nodes.size() << ")", PRX_TEXT_CYAN );                
                return NULL;
            }

            bool VO_application_t::ready_to_go()
            {
                return false;
            }

            bool VO_application_t::running()
            {
                return end_frame_id > frame_id;
            }

            void VO_application_t::frame(const ros::TimerEvent& event)
            {
                //waiting_time_stream << ros::Time::now() << ",";
                stats_clock.reset();

                // bool spawned = false;
                // if(frame_id % 10000 == 0)
                // {
                //     PRX_PRINT(" -------------------- ", PRX_TEXT_BLUE);
                //     PRX_PRINT(" ------- " << frame_id << "/" << end_frame_id << " ------- ", PRX_TEXT_RED);
                //     PRX_PRINT(" -------------------- ", PRX_TEXT_BLUE);
                // }
                // ROS_INFO("\n--------------------------------------------------------------------------------\n                                   frame %d in node %d/%d                                    \n--------------------------------------------------------------------------------", frame_id, node_id,num_of_nodes);

                if( simulator_running )
                {
                    if(update_once)
                    {
                        PRX_PRINT("---- First sense ----", PRX_TEXT_MAGENTA);
                        simulation_state_space->copy_to_point(simulation_state);
                        PRX_PRINT("First point: " << simulation_state_space->print_point(simulation_state, 5), PRX_TEXT_CYAN);
                        simulator->push_state(simulation_state);
                        // nav_sensor->update_data();
                        update_once = false; 
                        PRX_PRINT("========= DONE FIRST SENSE ========", PRX_TEXT_MAGENTA);
                    }
                    //Check for spawning agents
                    ++frame_id;

                    if(frame_id > evacuation_frame && !evacuation_plan)
                    {
                        PRX_PRINT("----- EVACUATE frame : " << frame_id << "/" << evacuation_frame <<" \n\n\n\nz" , PRX_TEXT_RED);
                        evacuation_plan = true;
                        foreach(behavior_controller_t* controller, behavior_controllers)
                        {                            
                            if(controller->is_active())
                                controller->evacuate(evacuation_points);
                        }
                    }

                    // spawned=check_for_agent_spawn();
                    check_for_departures();
                    if(!evacuation_plan)
                        check_for_agent_spawn();
                    //Also let the world structure update
                    world_structure.frame();
                    // //PRX_PRINT("=== Frame ===", PRX_TEXT_BLUE);
                }

                // waiting_time_stream << ros::Time::now() << ",";
                
                //Propagate
                simulator->push_state(simulation_state);
                simulator->push_control(simulation_control);
                simulator->propagate_and_respond();
                simulation_state_space->copy_to_point(simulation_state);   

                //waiting_time_stream << ros::Time::now() << ",";

                prx_simulation::pabt_node_msg node_msg;
                nav_sensor->generate_node_msg(node_id, node_msg);
                node_msg.frame_id = frame_id;
                node_msg.stamp = ros::Time::now();

                //waiting_time_stream << ros::Time::now() << ",";

                agent_states_pub.publish(node_msg);
                

                //waiting_time_stream << ros::Time::now() << ",";
                int count = 0;
                ros::Rate loop_rate(2000);
                while ((ros::ok()) && msg_index < (num_of_nodes-1))
                {
                    ros::spinOnce();
                    loop_rate.sleep();
                    count ++;
                }
                msg_index = 0;


                //waiting_time_stream << count << "," << ros::Time::now() << ",";
            
                frame_time += frame_clock.measure();
                if( agent_pool.empty() )
                {
                    saturated_time += frame_clock.measure();
                    ++saturated_frames;
                }

                if( simulator_running )
                {                    
                    total_computation_time += stats_clock.measure();
                    total_simulation_time += sim::simulation::simulation_step;
                    // Take care of selection things
                    if (!selected_path.empty())
                    {
                        if (plant_to_VO.find(selected_path) != plant_to_VO.end())
                        {
                            selected = plant_to_VO[selected_path];
                            selected->visualize_VOs();
                        }
                    }

                    int currently_active_agents = all_agents_size - agent_pool.size();
                    waiting_time_stream << frame_id << " , " << currently_active_agents << " , " << (total_computation_time/frame_id) << " , " << (total_simulation_time/frame_id) << "\n";
                    if(frame_id % 1000 == 0)
                        stats_stream << frame_id << " , " << currently_active_agents << " , " << num_of_agents_in_total << " , " << total_computation_time << " , " << total_simulation_time << "\n";
                    //Then, for each plant, let's record its trajectory

                    if(export_data)
                    {
                        double time_stamp = frame_id * sim::simulation::simulation_step;
                        for( unsigned i=0; i<plants.size(); ++i )
                        {
                             (*(file_streams[i])) << plants[i]->print_state() << "," << all_agents[i].get_queue_id() << "," << behavior_controllers[i]->get_lookup_x() <<"," << behavior_controllers[i]->get_lookup_y() << "\n";

                            if(frame_id % frame_rate_output == 0)
                                if(all_agents[i].is_active())
                                    simulation_file_stream << frame_id << "," << all_agents[i].agent_id << "," << time_stamp << "," <<  all_agents[i].agent_type << "," <<  plants[i]->print_state() << "," << vo_controllers[i]->get_current_velocity() << "," << "," << all_agents[i].get_queue_id() << "," << behavior_controllers[i]->get_lookup_x() <<"," << behavior_controllers[i]->get_lookup_y() << ","<< all_agents[i].has_luggage << "," << all_agents[i].hindered  << "\n";

                            if(frame_id % 50 == 0)
                                if(all_agents[i].is_active())
                                    moe_file_stream << all_agents[i].agent_id << "," << time_stamp << "," <<  all_agents[i].agent_type << "," << all_agents[i].get_queue_id() << "," <<  plants[i]->print_state() << "," << all_agents[i].VO_controller->get_current_velocity() << "\n";
                        }
                    }
                }


                if(visualize && (visualization_counter++%visualization_multiple == 0))
                {
                    tf_broadcasting();
                }

                // double stime = sim::simulation::simulation_time.toSec();
                // if( (stime - std::floor(stime + PRX_ZERO_CHECK)) < PRX_ZERO_CHECK )
                // if(frame_id % 1000 == 0)
                // {
                //     //PRX_PRINT("====================== NODE : " << node_id << " : frame:" << frame_id << " ======================" , PRX_TEXT_LIGHTGRAY);
                //     //PRX_PRINT("Active Agents: " << all_agents.size() - agent_pool.size() , PRX_TEXT_LIGHTGRAY );
                //     //PRX_PRINT("Behavior C. compute control: " << behavior_controller_t::cc_time / (double)behavior_controller_t::cc_calls, PRX_TEXT_GREEN );
                //     //PRX_PRINT("Path Follower C. compute control: " << path_follow_controller_t::cc_time / (double)path_follow_controller_t::cc_calls, PRX_TEXT_CYAN );
                //     //PRX_PRINT("Collision-Avoid C. compute control: " << collision_avoidance_controller_t::cc_time / (double)collision_avoidance_controller_t::cc_calls, PRX_TEXT_BLUE );
                //     //PRX_PRINT(" -> Total Per-Agent Compute Control Time: " << behavior_controller_t::tot_time / (double)behavior_controller_t::cc_calls, PRX_TEXT_LIGHTGRAY );
                //     //PRX_PRINT("Per-Agent Sensing Time: " << navigation_graph_sensor_t::update_time / (double)navigation_graph_sensor_t::update_calls, PRX_TEXT_BROWN );
                //     //PRX_PRINT("Spawn/Structure time: " << spawn_time/(double)frame_id, PRX_TEXT_RED);
                //     //PRX_PRINT("Propagate time: " << propagate_time/(double)frame_id, PRX_TEXT_CYAN);
                //     //PRX_PRINT("Publish state time: " << publish_state_time/(double)frame_id, PRX_TEXT_CYAN);
                //     //PRX_PRINT("Update node time: " << update_node_time/(double)update_node_id, PRX_TEXT_CYAN);
                //     //PRX_PRINT("Waiting time: " << communication_time/(double)frame_id, PRX_TEXT_CYAN);
                //     //PRX_PRINT("Total Frame time: " << frame_time/(double)frame_id, PRX_TEXT_MAGENTA);
                //     //PRX_PRINT("  -> Saturated Frame Time: " << saturated_time/(double)saturated_frames, PRX_TEXT_MAGENTA);

                //     // //stats_stream << frame_id << " , " << all_agents.size() - agent_pool.size()
                //     //                          << " , " << behavior_controller_t::cc_time / (double)behavior_controller_t::cc_calls
                //     //                          << " , " << path_follow_controller_t::cc_time / (double)path_follow_controller_t::cc_calls
                //     //                          << " , " << collision_avoidance_controller_t::cc_time / (double)collision_avoidance_controller_t::cc_calls
                //     //                          << " , " << behavior_controller_t::tot_time / (double)behavior_controller_t::cc_calls
                //     //                          << " , " << navigation_graph_sensor_t::update_time / (double)navigation_graph_sensor_t::update_calls
                //     //                          << " , " << spawn_time/(double)frame_id
                //     //                          << " , " << propagate_time/(double)frame_id
                //     //                          << " , " << publish_state_time/(double)frame_id
                //     //                          << " , " << update_node_time/(double)update_node_id
                //     //                          << " , " << communication_time/(double)frame_id
                //     //                          << " , " << frame_time/(double)frame_id
                //     //                          << " , " << saturated_time/(double)saturated_frames
                //     //                          << std::endl;
                // }
/*
                //stats_stream << frame_id << " , " << all_agents.size() - agent_pool.size()
                                             << " , " << behavior_controller_t::cc_time / (double)behavior_controller_t::cc_calls
                                             << " , " << path_follow_controller_t::cc_time / (double)path_follow_controller_t::cc_calls
                                             << " , " << collision_avoidance_controller_t::cc_time / (double)collision_avoidance_controller_t::cc_calls
                                             << " , " << behavior_controller_t::tot_time / (double)behavior_controller_t::cc_calls
                                             << " , " << navigation_graph_sensor_t::update_time / (double)navigation_graph_sensor_t::update_calls
                                             << " , " << spawn_time/(double)frame_id
                                             << " , " << propagate_time///(double)frame_id
                                             << " , " << publish_state_time/(double)frame_id
                                             << " , " << update_node_time/(double)update_node_id
                                             << " , " << communication_time/(double)frame_id
                                             << " , " << frame_time/(double)frame_id
                                             << " , " << saturated_time/(double)saturated_frames
                                             << std::endl;
                */
                if(end_frame_id <= frame_id && export_data)
                {
                    waiting_time_stream.flush();
                    waiting_time_stream.close();
                    waiting_time_stream.clear();

                    moe_file_stream.flush();
                    moe_file_stream.close();
                    moe_file_stream.clear();

                    stats_stream.flush();                    
                    stats_stream.close();
                    stats_stream.clear();

                    simulation_file_stream.flush();
                    simulation_file_stream.close();
                    simulation_file_stream.clear();
                }
                // if( frame_id % 3 == 0 )
                // {
                //     visualize_wall_data( (frame_id/50) % boost::num_vertices( navigation_graph.graph ) );
                // }

               // ROS_INFO("------------------------------------------------------------------------");
               
               //waiting_time_stream << ros::Time::now() << ",\n";
               

            }

            // void VO_application_t::frame(const ros::TimerEvent& event)
            // {
            //     //PRX_PRINT("=== Node " << node_id << "  ===  Frame " << frame_id << "  ===", PRX_TEXT_BLUE);
            //     frame_clock.reset();
            //     ros::Time starting_sec =ros::Time::now();
            //     if( simulator_running )
            //     {
            //         //Check for spawning agents
            //         ++frame_id;

            //         check_for_agent_spawn();
            //         //Also let the world structure update
            //         world_structure.frame();
            //         // PRX_PRINT("=== Frame ===", PRX_TEXT_BLUE);
            //     }
                
            //     spawn_time += frame_clock.measure();

            //     double t = frame_clock.measure();
            //     //Propagate
            //     simulator->push_state(simulation_state);
            //     simulator->push_control(simulation_control);
            //     simulator->propagate_and_respond();
            //     simulation_state_space->copy_to_point(simulation_state);  
            //     propagate_time += (frame_clock.measure() - t);    

            //     //Publish the new state;
            //     //PRX_PRINT("Node " << node_id << " publishes frame:" << frame_id, PRX_TEXT_GREEN);
            //     t = frame_clock.measure();
            //     prx_simulation::pabt_node_msg node_msg;
            //     nav_sensor->generate_node_msg(node_id, node_msg);
            //     node_msg.frame_id = frame_id;
            //     node_msg.stamp = ros::Time::now();                
            //     ros::Time to_publish_sec = ros::Time::now();
            //     agent_states_pub.publish(node_msg);
            //     publish_state_time += (frame_clock.measure() - t);

            //     //Wait while you read all the states from all the other nodes.
            //     //Starts from 1 because we do not wait for the current node and the num_of_nodes is 
            //     //the total number of simulation nodes.
            //     num_of_responses = 1;
            //     num_spins = 0;
            //     //waiting_time_stream << frame_id  << " , " << t << " , " << (all_agents.size() - agent_pool.size()) << " , ";
            //     ros::Rate r(700);
            //     double spin_avg = 0;
            //     ros::Time to_wait_sec =ros::Time::now();
            //     t = frame_clock.measure();
            //     //PRX_PRINT("Node " << node_id << " waits for the other nodes. Frames form other nodes: " << frames_from_nodes.size(), PRX_TEXT_GREEN+node_id);
            //     frames_from_nodes[node_id] = frame_id;
            //     while(num_of_responses < num_of_nodes)
            //     {
            //         num_spins++;
            //         double spin_t = frame_clock.measure();
            //         for(int n = 0; n < num_of_nodes; ++n)
            //         {
            //             if(frames_from_nodes[n] != frame_id)
            //             {
            //                 //PRX_PRINT("Node " << node_id << " calls the queue from the node " <<  n, PRX_TEXT_GREEN+node_id);
            //                 my_callback_queues[n]->callOne(ros::WallDuration());
            //             }
            //         }
            //         // ros::spinOnce();
            //         spin_avg += (frame_clock.measure() - spin_t);
            //         // ros::Duration(0.0005).sleep();
            //         if(num_of_responses < num_of_nodes)
            //             r.sleep();
            //         //PRX_PRINT("Node " << node_id << " done with the queues and now waits again", PRX_TEXT_GREEN+node_id);
            //     }
            //     //PRX_PRINT("Node " << node_id << " DONE waiting ", PRX_TEXT_BROWN);
 
            //     ros::Time done_waiting_sec =ros::Time::now();
            //     communication_time += (frame_clock.measure() - t);
            //     double full_time = frame_clock.measure() - t;
            //     //waiting_time_stream << num_spins << " , " << spin_avg << " , " << spin_avg/num_spins << " , " << full_time << " , ";
            //     // PRX_PRINT("Node " << node_id << ") num_of_responses " << num_of_responses << "/" << num_of_nodes, PRX_TEXT_CYAN);

            //     frame_time += frame_clock.measure();
            //     if( agent_pool.empty() )
            //     {
            //         saturated_time += frame_clock.measure();
            //         ++saturated_frames;
            //     }

            //     if( simulator_running )
            //     {
            //         // Take care of selection things
            //         if (!selected_path.empty())
            //         {
            //             if (plant_to_VO.find(selected_path) != plant_to_VO.end())
            //             {
            //                 selected = plant_to_VO[selected_path];
            //                 selected->visualize_VOs();
            //             }
            //         }

            //         //Then, for each plant, let's record its trajectory
            //         for( unsigned i=0; i<plants.size(); ++i )
            //         {
            //             // PRX_PRINT("Recording path for : " << i, PRX_TEXT_LIGHTGRAY);
            //             (*(file_streams[i])) << plants[i]->print_state() << "\n";
            //         }                    
            //     }

            //     //waiting_time_stream << frame_clock.measure() << " , " ;
            //     //waiting_time_stream << starting_sec << " , " << to_publish_sec << " , " << to_wait_sec << " , " << done_waiting_sec << " , " << (done_waiting_sec - to_wait_sec) << " , " << (ros::Time::now() - starting_sec)<< std::endl;

            //     if(visualize && (visualization_counter++%visualization_multiple == 0))
            //     {
            //         tf_broadcasting();
            //     }

            //     // double stime = sim::simulation::simulation_time.toSec();
            //     // if( (stime - std::floor(stime + PRX_ZERO_CHECK)) < PRX_ZERO_CHECK )
            //     if(frame_id % 1000 == 0)
            //     {
            //         PRX_PRINT("====================== NODE : " << node_id << " : frame:" << frame_id << " ======================" , PRX_TEXT_LIGHTGRAY);
            //         // PRX_PRINT("Active Agents: " << all_agents.size() - agent_pool.size() , PRX_TEXT_LIGHTGRAY );
            //         // PRX_PRINT("Behavior C. compute control: " << behavior_controller_t::cc_time / (double)behavior_controller_t::cc_calls, PRX_TEXT_GREEN );
            //         // PRX_PRINT("Path Follower C. compute control: " << path_follow_controller_t::cc_time / (double)path_follow_controller_t::cc_calls, PRX_TEXT_CYAN );
            //         // PRX_PRINT("Collision-Avoid C. compute control: " << collision_avoidance_controller_t::cc_time / (double)collision_avoidance_controller_t::cc_calls, PRX_TEXT_BLUE );
            //         // PRX_PRINT(" -> Total Per-Agent Compute Control Time: " << behavior_controller_t::tot_time / (double)behavior_controller_t::cc_calls, PRX_TEXT_LIGHTGRAY );
            //         // PRX_PRINT("Per-Agent Sensing Time: " << navigation_graph_sensor_t::update_time / (double)navigation_graph_sensor_t::update_calls, PRX_TEXT_BROWN );
            //         // PRX_PRINT("Spawn/Structure time: " << spawn_time/(double)frame_id, PRX_TEXT_RED);
            //         // PRX_PRINT("Propagate time: " << propagate_time/(double)frame_id, PRX_TEXT_CYAN);
            //         // PRX_PRINT("Publish state time: " << publish_state_time/(double)frame_id, PRX_TEXT_CYAN);
            //         // PRX_PRINT("Update node time: " << update_node_time/(double)update_node_id, PRX_TEXT_CYAN);
            //         // PRX_PRINT("Waiting time: " << communication_time/(double)frame_id, PRX_TEXT_CYAN);
            //         // PRX_PRINT("Total Frame time: " << frame_time/(double)frame_id, PRX_TEXT_MAGENTA);
            //         // PRX_PRINT("  -> Saturated Frame Time: " << saturated_time/(double)saturated_frames, PRX_TEXT_MAGENTA);

            //         // //stats_stream << frame_id << " , " << all_agents.size() - agent_pool.size()
            //         //                          << " , " << behavior_controller_t::cc_time / (double)behavior_controller_t::cc_calls
            //         //                          << " , " << path_follow_controller_t::cc_time / (double)path_follow_controller_t::cc_calls
            //         //                          << " , " << collision_avoidance_controller_t::cc_time / (double)collision_avoidance_controller_t::cc_calls
            //         //                          << " , " << behavior_controller_t::tot_time / (double)behavior_controller_t::cc_calls
            //         //                          << " , " << navigation_graph_sensor_t::update_time / (double)navigation_graph_sensor_t::update_calls
            //         //                          << " , " << spawn_time/(double)frame_id
            //         //                          << " , " << propagate_time/(double)frame_id
            //         //                          << " , " << publish_state_time/(double)frame_id
            //         //                          << " , " << update_node_time/(double)update_node_id
            //         //                          << " , " << communication_time/(double)frame_id
            //         //                          << " , " << frame_time/(double)frame_id
            //         //                          << " , " << saturated_time/(double)saturated_frames
            //         //                          << std::endl;
            //     }

            //     //stats_stream << frame_id << " , " << all_agents.size() - agent_pool.size()
            //                                  << " , " << behavior_controller_t::cc_time / (double)behavior_controller_t::cc_calls
            //                                  << " , " << path_follow_controller_t::cc_time / (double)path_follow_controller_t::cc_calls
            //                                  << " , " << collision_avoidance_controller_t::cc_time / (double)collision_avoidance_controller_t::cc_calls
            //                                  << " , " << behavior_controller_t::tot_time / (double)behavior_controller_t::cc_calls
            //                                  << " , " << navigation_graph_sensor_t::update_time / (double)navigation_graph_sensor_t::update_calls
            //                                  << " , " << spawn_time/(double)frame_id
            //                                  << " , " << propagate_time/(double)frame_id
            //                                  << " , " << publish_state_time/(double)frame_id
            //                                  << " , " << update_node_time/(double)update_node_id
            //                                  << " , " << communication_time/(double)frame_id
            //                                  << " , " << frame_time/(double)frame_id
            //                                  << " , " << saturated_time/(double)saturated_frames
            //                                  << std::endl;
                
            //     if(end_frame_id <= frame_id)                
            //         //stats_stream.close();
            //     // if( frame_id % 3 == 0 )
            //     // {
            //     //     visualize_wall_data( (frame_id/50) % boost::num_vertices( navigation_graph.graph ) );
            //     // }
            // }            

            void VO_application_t::check_regions_for_depatures()
            {
                //PRX_PRINT("CHECKING FOR REGIONS AT FRAME:"<<frame_id,PRX_TEXT_BROWN);
                foreach( region_t* region, world_structure.get_all_regions() )
                {
                    region->check_departure(frame_id);
                }
            }

            void VO_application_t::info_broadcasting(const ros::TimerEvent& event)
            {
                // std::string full_name = "Application_Debug";
                // ((sim::comm::visualization_comm_t*)sim::comm::vis_comm)->visualization_geom_map[full_name] = debug_vis_geoms;
                // ((sim::comm::visualization_comm_t*)sim::comm::vis_comm)->visualization_configs_map[full_name] = debug_vis_configs;

                tf_broadcasting();
            }

            void VO_application_t::set_systems_to_sensing()
            {
                if(!initialized)
                {
                    initialized = true;
                    vo_sensing = dynamic_cast< PABT_sensing_model_t* >(simulator->get_sensing_model());
                    PRX_ASSERT(vo_sensing != NULL);

                    application_t::initialize_sensing();
                }
            }

            void VO_application_t::handle_key()
            {
                application_t::handle_key();
            }

            void VO_application_t::visualize_nav_graph()
            {
                //Alright, then, if we are visualizing
                unsigned i=0;
                if( visualize && display_graph )
                {
                    //Let's visualize the graph
                    std::vector<geometry_info_t> geoms;
                    std::vector<double> params;
                    std::vector<config_t> configs;
                    config_t use_config;

                    //Alright, let's visualize the edges
                    foreach( undirected_edge_index_t e, boost::edges( navigation_graph.graph ) )
                    {
                        undirected_node_t* source = navigation_graph.get_vertex_as< undirected_node_t >( boost::source( e, navigation_graph.graph ) );
                        undirected_node_t* target = navigation_graph.get_vertex_as< undirected_node_t >( boost::target( e, navigation_graph.graph ) );

                        use_config.set_position( 0, 0, 0 );
                        params.clear();

                        params.push_back( source->point->memory[0] );
                        params.push_back( source->point->memory[1] );
                        params.push_back( source->point->memory[2] );
                        params.push_back( target->point->memory[0] );
                        params.push_back( target->point->memory[1] );
                        params.push_back( target->point->memory[2] );

                        // PRX_DEBUG_COLOR("Line: ( " << params[0] << " , " << params[1] << " , " << params[2] << ") ( " << params[3] << " , " << params[4] << " , " << params[5] << ") ", PRX_TEXT_CYAN );

                        std::string name = ros::this_node::getName() + "/nav_edge_" + int_to_str( i );
                        configs.push_back( use_config );
                        geoms.push_back( geometry_info_t(name, "sim", PRX_LINESTRIP, params, "blue") );

                        ++i;
                    }

                    ((comm::visualization_comm_t*)comm::vis_comm)->visualization_geom_map["vo_application"] = geoms;
                    ((comm::visualization_comm_t*)comm::vis_comm)->visualization_configs_map["vo_application"] = configs;
                    ((comm::visualization_comm_t*)comm::vis_comm)->send_geometries();
                }
            }

            void VO_application_t::visualize_back_pointers( region_t* region, bool hindered )
            {
                //Alright, then, if we are visualizing
                unsigned i=0;
                if( visualize )
                {
                    //Let's visualize the graph
                    std::vector<geometry_info_t> geoms;
                    std::vector<double> params;
                    std::vector<config_t> configs;
                    config_t use_config;

                    foreach( undirected_vertex_index_t v, boost::vertices( navigation_graph.graph ) )
                    {
                        nav_node_t* node = navigation_graph.get_vertex_as< nav_node_t >( v );
                        nav_node_t* source;
                        if( hindered ) 
                        {
                            source = node->hindered_back_pointer[ region ];
                        }
                        else
                        {
                            source = node->back_pointer[ region ];
                        }
                        
                        use_config.set_position( 0, 0, 0 );
                        params.clear();

                        params.push_back( source->point->memory[0] );
                        params.push_back( source->point->memory[1] );
                        params.push_back( source->point->memory[2] );
                        params.push_back( node->point->memory[0] );
                        params.push_back( node->point->memory[1] );
                        params.push_back( node->point->memory[2] );

                        std::string name = ros::this_node::getName() + "/back_edge_" + int_to_str( i );
                        configs.push_back( use_config );
                        geoms.push_back( geometry_info_t(name, "sim", PRX_LINESTRIP, params, (hindered ? "red" : "green") ) );

                        ++i;
                    }

                    ((comm::visualization_comm_t*)comm::vis_comm)->visualization_geom_map["vo_application"] = geoms;
                    ((comm::visualization_comm_t*)comm::vis_comm)->visualization_configs_map["vo_application"] = configs;
                    ((comm::visualization_comm_t*)comm::vis_comm)->send_geometries();
                }
            }

            void VO_application_t::visualize_triangles(  )
            {
                //Alright, then, if we are visualizing
                unsigned i=0;
                if( visualize )
                {
                    //Let's visualize the graph
                    std::vector<geometry_info_t> geoms;
                    std::vector<double> params;
                    std::vector<config_t> configs;
                    config_t use_config;

                    foreach( undirected_vertex_index_t v, boost::vertices( navigation_graph.graph ) )
                    {
                        nav_node_t* node = navigation_graph.get_vertex_as< nav_node_t >( v );
                        if( node->has_triangle() )
                        {
                            use_config.set_position( 0, 0, 0 );
                            params.clear();

                            params.push_back( node->triangle[0][0] );
                            params.push_back( node->triangle[0][1] );
                            params.push_back( node->triangle[0][2] + 0.0001 );
                            params.push_back( node->triangle[1][0] );
                            params.push_back( node->triangle[1][1] );
                            params.push_back( node->triangle[1][2] + 0.0001 );
                            params.push_back( node->triangle[2][0] );
                            params.push_back( node->triangle[2][1] );
                            params.push_back( node->triangle[2][2] + 0.0001 );
                            params.push_back( node->triangle[0][0] );
                            params.push_back( node->triangle[0][1] );
                            params.push_back( node->triangle[0][2] + 0.0001 );

                            std::string name = ros::this_node::getName() + "/triangle_" + int_to_str( i );
                            configs.push_back( use_config );
                            
                            std::string color = "grey";
                            if( node->near_elevator != NULL )
                            {
                                color = "red";
                            }
                            else if( node->near_ramp != NULL )
                            {
                                color = "magenta";
                            }
                            
                            geoms.push_back( geometry_info_t(name, "sim", PRX_LINESTRIP, params, color) );
                            // PRX_PRINT("Triangle: " << configs.back(), PRX_TEXT_CYAN);

                            ++i;
                        }
                    }
                    ((comm::visualization_comm_t*)comm::vis_comm)->visualization_geom_map["vo_application"] = geoms;
                    ((comm::visualization_comm_t*)comm::vis_comm)->visualization_configs_map["vo_application"] = configs;
                    ((comm::visualization_comm_t*)comm::vis_comm)->send_geometries();
                }
            }

            void VO_application_t::visualize_segments(  )
            {
                //Alright, then, if we are visualizing
                unsigned i=0;
                if( visualize )
                {
                    //Let's visualize the graph
                    std::vector<geometry_info_t> geoms;
                    std::vector<double> params;
                    std::vector<config_t> configs;
                    config_t use_config;

                    foreach( undirected_vertex_index_t v, boost::vertices( navigation_graph.graph ) )
                    {
                        nav_node_t* node = navigation_graph.get_vertex_as< nav_node_t >( v );
                        if( node->has_segment() )
                        {
                            use_config.set_position( 0, 0, 0 );
                            params.clear();

                            params.push_back( node->segment[0][0] );
                            params.push_back( node->segment[0][1] );
                            params.push_back( node->segment[0][2] + 0.03 );
                            params.push_back( node->segment[1][0] );
                            params.push_back( node->segment[1][1] );
                            params.push_back( node->segment[1][2] + 0.03 );

                            std::string name = ros::this_node::getName() + "/segment_" + int_to_str( i );
                            configs.push_back( use_config );
                            
                            std::string color = "violet";
                            
                            geoms.push_back( geometry_info_t(name, "sim", PRX_LINESTRIP, params, color) );
                            // PRX_PRINT("Triangle: " << configs.back(), PRX_TEXT_CYAN);

                            ++i;
                        }
                    }
                    ((comm::visualization_comm_t*)comm::vis_comm)->visualization_geom_map["vo_application"] = geoms;
                    ((comm::visualization_comm_t*)comm::vis_comm)->visualization_configs_map["vo_application"] = configs;
                    ((comm::visualization_comm_t*)comm::vis_comm)->send_geometries();
                }
            }

            void VO_application_t::visualize_wall_data( unsigned index )
            {
                //Alright, then, if we are visualizing
                unsigned i=0;
                if( visualize )
                {
                    //Let's visualize the graph
                    std::vector<geometry_info_t> geoms;
                    std::vector<double> params;
                    std::vector<config_t> configs;
                    config_t use_config;

                    foreach( undirected_vertex_index_t v, boost::vertices( navigation_graph.graph ) )
                    {
                        if( i == index )
                        {
                            //Get the node
                            nav_node_t* node = navigation_graph.get_vertex_as< nav_node_t >( v );
                            
                            //Visualize the node position
                            use_config.set_position( node->point->memory[0], node->point->memory[1], node->point->memory[2] );
                            params.clear();
                            params.push_back( 0.15 ); params.push_back( 0.15 ); params.push_back( 0.15 );
                            std::string name = ros::this_node::getName() + "/triangle";
                            configs.push_back( use_config );
                            geoms.push_back( geometry_info_t(name, "sim", PRX_BOX, params, "red") );
                            // PRX_PRINT("Visualize node at: " << use_config.get_position(), PRX_TEXT_RED);
                            
                            //Then, visualize each wall
                            for( unsigned k=0; k<node->obstacle_info->geometries.size(); ++k )
                            {
                                geometry_t* geom = node->obstacle_info->geometries[k];
                                use_config = node->obstacle_info->configs[k];
                                const std::vector<double>* dims = geom->get_info();
                                params[0] = (*dims)[0] + 0.01; params[1] = (*dims)[1] + 0.01; params[2] = (*dims)[2] + 0.01;
                                name = ros::this_node::getName() + "/wall_" + int_to_str( k );
                                
                                // PRX_PRINT("Wall [" + int_to_str(k) + "] at: " << use_config.get_position() << "  length: " << (*dims)[0], PRX_TEXT_LIGHTGRAY);
                                configs.push_back(use_config);
                                geoms.push_back( geometry_info_t(name, "sim", PRX_BOX, params, "green") );
                            }

                            ((comm::visualization_comm_t*)comm::vis_comm)->visualization_geom_map["vo_application"] = geoms;
                            ((comm::visualization_comm_t*)comm::vis_comm)->visualization_configs_map["vo_application"] = configs;
                            ((comm::visualization_comm_t*)comm::vis_comm)->send_geometries();
                            
                            return;
                        }
                        
                        ++i;
                    }
                }
            }

            void VO_application_t::visualize_queue_points(  )
            {

                //Alright, then, if we are visualizing
                unsigned i=0;
                if( visualize)
                {
                    //Let's visualize the graph
                    std::vector<geometry_info_t> geoms;
                    std::vector<double> params;
                    std::vector<config_t> configs;
                    config_t use_config;
                    std::vector<string> colors;
                    colors.push_back("blue");
                    colors.push_back("green");
                    colors.push_back("violet");
                    colors.push_back("orange");

                    int queue_counter = 0;                   

                    foreach(queue_t* q, queue_manager->get_queues())
                    {
                        PRX_PRINT("--------------------------", PRX_TEXT_GREEN);                        
                        foreach(_numnpt point, q->queue_point_slots)
                        {                            
                            use_config.set_position( point.pt.x, point.pt.y, point.pt.z );
                            //PRX_PRINT("x:" << point.pt.x << "  y: " << point.pt.y << "  z:" << point.pt.z, PRX_TEXT_LIGHTGRAY);
                            params.clear();
                            params.push_back(0.1);
                            std::string name = ros::this_node::getName() + "/queue_point" + int_to_str( i );
                            // color = "blue";
                            configs.push_back( use_config );                            
                            geoms.push_back( geometry_info_t(name, "sim", PRX_SPHERE, params, colors[queue_counter%4]) );
                            ++i;
                        }
                        ++queue_counter;
                        PRX_PRINT("Visualized queue:"<<queue_counter<<" of "<<queue_manager->get_queues().size(),PRX_TEXT_BROWN);
                    }

                    ((comm::visualization_comm_t*)comm::vis_comm)->visualization_geom_map["vo_application"] = geoms;
                    ((comm::visualization_comm_t*)comm::vis_comm)->visualization_configs_map["vo_application"] = configs;
                    ((comm::visualization_comm_t*)comm::vis_comm)->send_geometries();
                    
                }
                
            }


            void VO_application_t::visualize_elevator_associations(  )
            {
                //Alright, then, if we are visualizing
                if( visualize )
                {
                    //Let's visualize the graph
                    std::vector<geometry_info_t> geoms;
                    std::vector<double> params;
                    std::vector<config_t> configs;
                    config_t use_config;

                    unsigned vis_index = 0;
                    foreach( undirected_vertex_index_t v, boost::vertices( navigation_graph.graph ) )
                    {
                        //Get the node
                        nav_node_t* node = navigation_graph.get_vertex_as< nav_node_t >( v );
                        if( node->near_elevator != NULL )
                        {
                            //Visualize the node position
                            use_config.set_position( node->point->memory[0], node->point->memory[1], node->point->memory[2] );
                            params.clear();
                            params.push_back( 0.15 ); params.push_back( 0.15 ); params.push_back( 0.15 );
                            std::string name = ros::this_node::getName() + "/assoc_triangle_" + int_to_str( vis_index++ );
                            configs.push_back( use_config );
                            geoms.push_back( geometry_info_t(name, "sim", PRX_BOX, params, debug_colors[node->elevator_index % debug_colors.size() ]) );
                            // PRX_PRINT("Visualize node at: " << use_config.get_position(), PRX_TEXT_RED);

                            ((comm::visualization_comm_t*)comm::vis_comm)->visualization_geom_map["vo_application"] = geoms;
                            ((comm::visualization_comm_t*)comm::vis_comm)->visualization_configs_map["vo_application"] = configs;
                            ((comm::visualization_comm_t*)comm::vis_comm)->send_geometries();
                        }
                    }
                }
            }

            void VO_application_t::load_nav_graph(const parameter_reader_t * const reader)
            {
                //Create the navigation space...
                navigation_space = new space_t("XYZ", {&_Nx,&_Ny,&_Nz});
                queue_query_point = navigation_space->alloc_point();
                current_queue_state.resize(3);

                if( reader->has_attribute("navigation_graph"))
                {
                    std::string graph_file = folder_path + reader->get_attribute_as< std::string >( "navigation_graph" );
                    PRX_PRINT("Attempting to read the nav_graph from file: " << graph_file , PRX_TEXT_MAGENTA);

                    std::ifstream fin;
                    if( navigation_graph.deserialize<nav_node_t, undirected_edge_t>( graph_file, fin, navigation_space ) )
                    {
                        PRX_PRINT("Successfully read in Navigation Graph", PRX_TEXT_GREEN);
                    }
                    else
                    {
                        PRX_FATAL_S("Error reading Navigation Graph");
                    }
                    fin.close();
                }
                else
                {
                    PRX_FATAL_S("No Navigation Graph file provided, aborting.");
                }
                
                //Apply the global offsets to the z-values
                graph_offset = reader->get_attribute_as< double >( "graph_offset", 1.0 );
                foreach( undirected_vertex_index_t v, boost::vertices( navigation_graph.graph ) )
                {
                    //Get the node
                    nav_node_t* node = navigation_graph.get_vertex_as< nav_node_t >( v );
                    //Adjust the nav graph so the points end up above the floor
                    node->point->memory[2] += graph_offset;
                }

                //Displaying the graph
                display_graph = reader->get_attribute_as< bool >( "display_graph", false );
                display_triangles = reader->get_attribute_as< bool >( "display_triangles", true );
                display_segments = reader->get_attribute_as< bool >( "display_segments", true );

                //Then, link the astar up with the graph
                graph_search.link_graph( &navigation_graph );

                //We also kinda need a distance metric to get a path for things perhaps?
                if( reader->has_attribute("distance_metric") )
                {
                    metric = reader->initialize_from_loader< distance_metric_t >("distance_metric", "prx_utilities");
                }
                else
                {
                    PRX_FATAL_S("Missing distance_metric attribute in VO Application!");
                }

                //Then, we need to add all the nodes in the graph to the metric
                metric->link_space( navigation_space );

                foreach( undirected_vertex_index_t v, boost::vertices( navigation_graph.graph ) )
                {
                    //Get the node
                    nav_node_t* node = navigation_graph.get_vertex_as< nav_node_t >( v );
                    metric->add_point( node );
                }
            }
            
            void VO_application_t::clean_up_components( const parameter_reader_t * const reader )
            {
                // Remove all of the smaller components
                std::vector< undirected_vertex_index_t > to_remove;
                std::vector< unsigned > component_size;
                
                unsigned largest_comp = 0;
                unsigned largest_size = 0;
                int nr_components = boost::connected_components(navigation_graph.graph, navigation_graph.components);
                
                PRX_PRINT("Graph starts with: " << boost::num_vertices( navigation_graph.graph ) << " vertices.", PRX_TEXT_RED );
                PRX_PRINT("Graph has " << nr_components << " components.", PRX_TEXT_CYAN);
                
                component_size.resize( nr_components );
                for( unsigned i=0; i<nr_components; ++i )
                {
                    component_size[i] = 0;
                }
                
                foreach( undirected_vertex_index_t v, boost::vertices( navigation_graph.graph ) )
                {
                    ++component_size[ navigation_graph.components[v] ];
                }
                
                for( unsigned i=0; i<nr_components; ++i )
                {
                    if( component_size[i] > largest_size )
                    {
                        largest_size = component_size[i];
                        largest_comp = i;
                    }
                }
                
                PRX_PRINT("Largest component is " << largest_comp << " of size: " << largest_size, PRX_TEXT_MAGENTA);
                
                foreach( undirected_vertex_index_t v, boost::vertices( navigation_graph.graph ) )
                {
                    if( navigation_graph.components[v] != largest_comp )
                    {
                        to_remove.push_back( v );
                    }
                }
                
                PRX_PRINT("We are going to remove " << to_remove.size() << " vertices.", PRX_TEXT_CYAN);
                
                for( unsigned i=0; i<to_remove.size(); ++i )
                {
                    navigation_graph.clear_and_remove_vertex( to_remove[i] );
                }
                
                PRX_PRINT("Graph now has: " << boost::num_vertices( navigation_graph.graph ) << " vertices.", PRX_TEXT_RED );

                //Then, re-initialize the metric to only have the points which are still in the graph.
                metric->clear();
                foreach( undirected_vertex_index_t v, boost::vertices( navigation_graph.graph ) )
                {
                    //Get the node
                    nav_node_t* node = navigation_graph.get_vertex_as< nav_node_t >( v );
                    metric->add_point( node );
                }

                graph_search.link_distance_function(metric->distance_function);
                for( unsigned i=0; i<behavior_controllers.size(); ++i )
                {
                    behavior_controllers[i]->link_search_primitives( &navigation_graph, &graph_search, metric );
                }                
            }
            
            void VO_application_t::save_nav_graph_annotations( std::ofstream& out )
            {
                PRX_PRINT("Saving Annotations!!!", PRX_TEXT_CYAN);
                
                //Output in YAML?
                out << "nav_nodes:\n";
                
                foreach( undirected_vertex_index_t v, boost::vertices( navigation_graph.graph ) )
                {
                    //Get the node
                    nav_node_t* node = navigation_graph.get_vertex_as< nav_node_t >( v );
                    out << "  -\n";

                    //1) PROX_ELEM_T : Geometry information
                    out << "    geometries:\n";
                    for( unsigned i=0; i<node->obstacle_info->geometries.size(); ++i )
                    {
                        out << "      -\n";
                        out << "        name: " << node->obstacle_info->config_names[i] << "\n";
                        out << "        distance: " << node->obstacle_info->distances[i] << "\n";
                    }

                    //2) WALLS:
                    out << "    walls:\n";
                    for( unsigned i=0; i<node->walls.size(); ++i )
                    {
                        out << "      -\n";
                        out << "        point_a: [";
                        out << node->walls[i].first[0] << ", " << node->walls[i].first[1];
                        out << "]\n";
                        out << "        point_b: [";
                        out << node->walls[i].second[0] << ", " << node->walls[i].second[1];
                        out << "]\n";
                        out << "        wall_id: ";
                        out << node->wall_segment_structures[i]->actual_segment->get_hash_key();
                        out << "\n";
                    }

                    //3) Attractor Distances: attractor_info_t
                    out << "    attractor_distances:\n";
                    for( unsigned i=0; i<node->attractor_distances.size(); ++i )
                    {
                        out << "      -\n";
                        out << "        name: " << node->attractor_distances[i]->attractor->name << "\n";
                        out << "        distance: " << node->attractor_distances[i]->dist << "\n";
                        out << "        doorway: " << node->attractor_distances[i]->doorway_index << "\n";
                    }

                    /* This is Andrew's Code - Now we are using region instead of origin*/
                    //4) Origin Distances: hash<origin_t*, double>
                    /*out << "    origin_distances:\n";
                    foreach( origin_t* org, node->origin_distances | boost::adaptors::map_keys )
                    {
                        out << "      -\n";
                        out << "        name: " << org->name << "\n";
                        out << "        distance: " << node->origin_distances[ org ] << "\n";
                    }*/

                    //4) Region Distances: hash<region_t*, double>
                    out << "    region_distances:\n";
                    foreach( region_t* org, node->region_distances | boost::adaptors::map_keys )
                    {
                        out << "      -\n";
                        out << "        name: " << org->name << "\n";
                        out << "        distance: " << node->region_distances[ org ] << "\n";
                    }
                    
                    //5) Region information: bool and name
                    if( node->is_region )
                    {
                        out << "    region: " << node->corresponding_region->name << "\n";
                        for( unsigned i=0; i<node->corresponding_region->nodes.size(); ++i )
                        {
                            if( node == node->corresponding_region->nodes[i] )
                            {
                                out << "    index: " << i << "\n";
                                break;
                            }
                        }
                    }
                    
                    //6) Associated structure information
                    if( node->near_ramp != NULL )
                    {
                        out << "    ramp_index: " << node->ramp_index << "\n";
                    }
                    if( node->near_elevator != NULL )
                    {
                        out << "    elevator_index: " << node->elevator_index << "\n";
                    }
                    
                }
            }
            
            void VO_application_t::load_nav_graph_annotations( const std::string& filename )
            {
                PRX_PRINT("Loading Navigation Graph Annotations!!!", PRX_TEXT_CYAN);
                
                //Index for the readers
                unsigned index = 0;

                foreach(system_ptr_t ob, simulator->get_obstacles() | boost::adaptors::map_values)
                {
                    dynamic_cast< obstacle_t* >(ob.get())->update_phys_configs(obstacle_configs, index);
                    ob->get_sensed_geoms(geom_map);
                    ++index;
                }
                
                index = 0;
                
                //YAML::Node Method
                //Open the file
                double yaml_time  = stats_clock.measure();
                YAML::Node annotations = YAML::LoadFile(filename);
                yaml_time = stats_clock.measure() - yaml_time;
                PRX_PRINT(":::: (" << node_id <<") Load the file to the YAML NODE takes: " << yaml_time << " seconds", PRX_TEXT_CYAN);

                prox_elements.resize( boost::num_vertices( navigation_graph.graph ) );
                
                double avg_annotation_time = 0;
                foreach( undirected_vertex_index_t v, boost::vertices( navigation_graph.graph ) )
                {
                    yaml_time = stats_clock.measure();
                    nav_node_t* node = navigation_graph.get_vertex_as< nav_node_t >( v );

                    prox_elements[index] = new prox_elem_t();
                    node->obstacle_info = prox_elements[index];
                    
                    //Get this particular nav_node entry
                    YAML::Node entry = annotations["nav_nodes"][index];
                    
                    //1) PROX_ELEM_T : Geometry information 
                    for( unsigned i=0; i<entry["geometries"].size(); ++i )
                    {
                        std::string geom_name = entry["geometries"][i]["name"].as< std::string >();
                        double dist = entry["geometries"][i]["distance"].as< double >();
                     
                        geometry_t* geom = &(geom_map[geom_name]); //This is a duder we need to get
                        config_t* config = NULL;
                        for( unsigned i=0; i<obstacle_configs.size() && config == NULL; ++i )
                        {
                            if( obstacle_configs[i].first == geom_name )
                            {
                                config = &(obstacle_configs[i].second);
                            }
                        }
                        if( config == NULL )
                            PRX_FATAL_S("Could not find config: " << geom_name);
                        //Have everything now, assign it in
                        node->obstacle_info->geometries.push_back( geom );
                        node->obstacle_info->configs.resize( node->obstacle_info->configs.size() + 1 );
                        node->obstacle_info->configs.back() = (*config);
                        node->obstacle_info->config_names.push_back( geom_name );
                        node->obstacle_info->distances.push_back( dist );                        
                    }
                    //2) WALLS
                    std::vector<unsigned> walls_ids;
                    // unsigned w_id;
                    for( unsigned i=0; i<entry["walls"].size(); ++i )
                    {
                        node->walls.resize( node->walls.size() + 1 );

                        entry["walls"][i]["point_a"] >> node->walls.back().first;
                        entry["walls"][i]["point_b"] >> node->walls.back().second;
                         // w_id = entry["walls"][i]["wall_id"].as<unsigned>();
                        walls_ids.push_back(entry["walls"][i]["wall_id"].as<unsigned>());
                        
                        //segment_t* segment = new segment_t(entry["walls"][i]["point_a"],entry["walls"][i]["point_b"]);
                        //node->add_new_wall_segment(segment);
                    }
                    //node->update_wall_segment(walls_ids);
                    //3) Attractor Distances: attractor_info_t
                    for( unsigned i=0; i<entry["attractor_distances"].size(); ++i )
                    {
                        std::string attractor_name = entry["attractor_distances"][i]["name"].as< std::string >();
                        double dist = entry["attractor_distances"][i]["distance"].as< double >();
                        unsigned doorway = entry["attractor_distances"][i]["doorway"].as< unsigned >();
                        
                        attractor_t* attractor = world_structure.get_attractor( attractor_name );
                        PRX_ASSERT( attractor != NULL );
                        node->attractor_distances.push_back( new attractor_info_t(attractor, dist, doorway) );
                    }

                    /* This is Andrew's Code - Now we are using region instead of origin*/
                    /*//4) Origin Distances: hash<origin_t*, double>
                    for( unsigned i=0; i<entry["origin_distances"].size(); ++i )
                    {
                        std::string origin_name = entry["origin_distances"][i]["name"].as< std::string >();
                        double dist = entry["origin_distances"][i]["distance"].as< double >();
                        
                        origin_t* origin = world_structure.get_origin( origin_name );
                        PRX_ASSERT( origin != NULL );
                        node->origin_distances[ origin ] = dist;                        
                    }*/
                    //4) Region Distances: hash<origin_t*, double>
                    for( unsigned i=0; i<entry["origin_distances"].size(); ++i )
                    {
                        std::string origin_name = entry["origin_distances"][i]["name"].as< std::string >();
                        double dist = entry["origin_distances"][i]["distance"].as< double >();
                        
                        region_t* origin = world_structure.get_origin( origin_name );
                        PRX_ASSERT( origin != NULL );

                        PRX_PRINT(" load_nav_graph_annotations detect region : " << origin_name <<" dist:"<< dist, PRX_TEXT_BLUE);
                        node->region_distances[ origin ] = dist;                        
                    }

                    //5) Region information: bool and name
                    if( entry["region"] != NULL )
                    {
                        std::string region_name = entry["region"].as< std::string >();
                        unsigned door_index = entry["index"].as< unsigned >();

                        region_t* region = world_structure.get_region( region_name );
                        PRX_ASSERT( region != NULL );
                        node->is_region = true;
                        node->corresponding_region = region;
                        if( region->nodes.size() <= door_index )
                        {
                            region->nodes.resize( door_index + 1 );
                        }
                        region->nodes[door_index] = node;
                    }
                    //6) Associated structure information
                    if( entry["ramp_index"] != NULL )
                    {
                        node->ramp_index = entry["ramp_index"].as< unsigned >();
                        node->near_ramp = ramps[ node->ramp_index ];
                    }
                    if( entry["elevator_index"] != NULL )
                    {
                        node->elevator_index = entry["elevator_index"].as< unsigned >();
                        node->near_elevator = world_structure.get_elevators()[ node->elevator_index ];
                    }
                    
                    yaml_time = stats_clock.measure() - yaml_time;
                    avg_annotation_time += yaml_time;
                    ++index;
                }
                PRX_PRINT(":::: (" << node_id <<") On average to load one node on the graph it takes : " << (avg_annotation_time/boost::num_vertices(navigation_graph.graph)) << " seconds", PRX_TEXT_CYAN);

            }

            void VO_application_t::find_nearby_triangles()
            {
                //For each of the vertices
                foreach( undirected_vertex_index_t v, boost::vertices( navigation_graph.graph ) )
                {
                    //Get the nav node
                    nav_node_t* node = navigation_graph.get_vertex_as< nav_node_t >( v );
                    //If it corresponds to a triangle
                    if( node->has_triangle() )
                    {
                        //Pick up all of its neighbors, which should be non-triangle nodes
                        foreach( undirected_vertex_index_t u, boost::adjacent_vertices( v, navigation_graph.graph ) )
                        {
                            //In an elevator, our neighbor might have triangle information, but we should ignore those
                            nav_node_t* neighbor = navigation_graph.get_vertex_as< nav_node_t >( u );
                            if( !neighbor->has_triangle() )
                            {
                                //For each of our neighbor's neighbors
                                foreach( undirected_vertex_index_t w, boost::adjacent_vertices( u, navigation_graph.graph ) )
                                {
                                    //These guys are our potential triangle neighbors
                                    nav_node_t* triangle_neighbor = navigation_graph.get_vertex_as< nav_node_t >( w );
                                    //If it does have triangle information and is not ourself
                                    if( triangle_neighbor->has_triangle() && triangle_neighbor != node )
                                    {
                                        //Then it must be a valid triangle neighbor
                                        node->neighboring_triangles.push_back( triangle_neighbor );
                                    }
                                }
                            }
                        }
                    }
                }
            }

            
            void VO_application_t::get_longest_line(std::pair< std::vector< double > , std::vector< double > >& line, const std::vector< util::vector_t >& points, double z)
            {
                double first_dist = (points[0][0] - points[1][0])*(points[0][0] - points[1][0]) + (points[0][1] - points[1][1])*(points[0][1] - points[1][1]);
                double second_dist = (points[1][0] - points[2][0])*(points[1][0] - points[2][0]) + (points[1][1] - points[2][1])*(points[1][1] - points[2][1]);
                if( first_dist > second_dist )
                {
                    //Endpoints are averages of 0&3 and 1&2
                    //X coordinate of first point
                    line.first[0] = (points[0][0] + points[3][0])/2.0;
                    //Y coordinate of first point
                    line.first[1] = (points[0][1] + points[3][1])/2.0;
                    //Z coordinate of the first point
                    line.first[2] = z;
                    //X coordinate of second point
                    line.second[0] = (points[1][0] + points[2][0])/2.0;
                    //Y coordinate of second point
                    line.second[1] = (points[1][1] + points[2][1])/2.0;
                    //Z coordinate of the second point
                    line.second[2] = z;
                }
                else
                {
                    //Otherwise, Endpoints are 0&1 and 2&3
                    //X coordinate of first point
                    line.first[0] = (points[0][0] + points[1][0])/2.0;
                    //Y coordinate of first point
                    line.first[1] = (points[0][1] + points[1][1])/2.0;
                    //Z coordinate of the first point
                    line.first[2] = z;
                    //X coordinate of second point
                    line.second[0] = (points[3][0] + points[2][0])/2.0;
                    //Y coordinate of second point
                    line.second[1] = (points[3][1] + points[2][1])/2.0;
                    //Z coordinate of the second point
                    line.second[2] = z;
                }
            }

            void VO_application_t::compute_static_obstacle_vertices()
            {
                //Some local variables
                std::vector< std::string > obstacle_names;
                std::vector< util::geometry_t* > obstacle_geoms;
                std::vector< std::vector< proximity_query_node_t > > object_verts;
                util::config_list_t hold_configs;
                std::pair< std::vector< double > , std::vector< double > > line;
                line.first.resize(3);
                line.second.resize(3);

                vector_t primary_vector(2);
                vector_t secondary_vector(2);
                unsigned next;
                unsigned previous;
                
                pq_nodes.clear();

                PRX_PRINT("Computing static obstacle vertices...", PRX_TEXT_RED);
            
                // ==============================================
                //   Get all of the obstacles in the simulation
                // ==============================================
                unsigned index = 0;
                foreach(system_ptr_t ob, simulator->get_obstacles() | boost::adaptors::map_values)
                {
                    dynamic_cast< obstacle_t* >(ob.get())->update_phys_configs(hold_configs, index);
                    ob->get_sensed_geoms(geom_map);
                    ++index;
                }

                // ========================================
                //   Begin setting up list of names/geoms
                // ========================================
                unsigned hold_index = 0;
                foreach(std::string name, hold_configs | boost::adaptors::map_keys)
                {
                    geometry_t& geo = geom_map[name];

                    //Then, we only really care about boxes for now
                    if( geo.get_type() == PRX_BOX )
                    {
                        obstacle_configs.push_back( hold_configs[hold_index] );
                        obstacle_geoms.push_back( &(geom_map[name]) );
                        obstacle_names.push_back( name );
                        
                        obstacle_neighbors.resize( obstacle_neighbors.size() + 1 );
                        obstacle_neighbors.back().set_name( name );
                        obstacle_neighbors.back().set_reciprocity( false );
                        obstacle_neighbors.back().set_obstacle_marker( true );
                        obstacle_neighbors.back().set_neighbor_radius( obstacle_geoms.back()->get_bounding_radius() );
                        obstacle_neighbors.back().set_neighbor_geotype( PRX_BOX );
                        obstacle_neighbors.back().set_neighbor_center( hold_configs[hold_index].second.get_position()[0], hold_configs[hold_index].second.get_position()[1] );
                        obstacle_neighbors.back().set_neighbor_velocity( 0, 0 );
                        obstacle_neighbors.back().set_valid( true );
                    }
                    ++hold_index;
                }
                
                //Remember how many things we had before elevators
                unsigned nr_non_elevators = obstacle_names.size();

                // ====================================
                //   Adding the elevators to the list
                // ====================================
                const std::vector< elevator_t* >& elevators = world_structure.get_elevators();
                for( unsigned i=0; i<elevators.size(); ++i )
                {
                    obstacle_configs.push_back( std::pair< std::string, config_t >( elevators[i]->name, elevators[i]->configuration ) );
                    obstacle_geoms.push_back( elevators[i]->shaft_geometry );
                    obstacle_names.push_back( elevators[i]->name );
                }
                
                // ========================================================
                //   Going over every BOX in the list to compute vertices
                // ========================================================
                unsigned vindex = 0;
                for( unsigned i=0; i<obstacle_configs.size(); ++i )
                {
                    //Get the geometry we are computing vertices
                    geometry_t& geo = *(obstacle_geoms[i]);

                    //We need to process the vertices of the mesh
                    const std::vector<vector_t>& vertices = geo.get_trimesh()->get_vertices();

                    //We need another set of vertices
                    object_verts.resize( object_verts.size() + 1 );

                    // ==========================================
                    //   Actually find the vertices of this box
                    // ==========================================
                    for( unsigned s=0; s<vertices.size(); ++s )
                    {
                        const vector_t& first = vertices[s];
                        for( unsigned t=s+1; t<vertices.size(); ++t )
                        {
                            const vector_t& second = vertices[t];

                            //If the x and y values for the points are nearly the same, they must be above each other
                            if( fabs(first[0] - second[0]) < PRX_DISTANCE_CHECK && fabs(first[1] - second[1]) < PRX_DISTANCE_CHECK )
                            {
                                //Now, we need to transform the point so it represent the obstacle's pose.
                                transform_config.set_position( first );
                                transform_config.set_orientation( 0, 0, 0, 1 );
                                transform_config.relative_to_global( obstacle_configs[i].second );

                                object_verts[i].resize( object_verts[i].size() + 1 );

                                //Then, put in the transformed points
                                object_verts[i].back().point = navigation_space->alloc_point();
                                object_verts[i].back().point->memory[0] = transform_config.get_position()[0];
                                object_verts[i].back().point->memory[1] = transform_config.get_position()[1];
                                object_verts[i].back().point->memory[2] = (first[2] + second[2])/2.0 + obstacle_configs[i].second.get_position()[2];
                            }
                        }
                    }
                }
                
                // ===========================================================
                //  Go over all of the boxes and compute Minkowski Obstacles
                // ===========================================================
                for( unsigned i=0; i<object_verts.size(); ++i )
                {
                    // ==========================================
                    //   Compute the centroid of these vertices
                    // ==========================================
                    std::vector< double > centroid(2);
                    centroid[0] = centroid[1] = 0;
                    std::vector< util::vector_t > orig_points(object_verts[i].size());
                    for( unsigned t=0; t<object_verts[i].size(); ++t )
                    {
                        centroid[0] += object_verts[i][t].point->memory[0];
                        centroid[1] += object_verts[i][t].point->memory[1];
                        orig_points[t].resize(2);
                        orig_points[t][0] = object_verts[i][t].point->memory[0];
                        orig_points[t][1] = object_verts[i][t].point->memory[1];
                        // PRX_PRINT(i << "," << t << ") " << object_verts[i][t].point->memory[0] << "," << object_verts[i][t].point->memory[1], PRX_TEXT_GREEN);
                    }
                    // PRX_PRINT(i << ") vert size :" << object_verts[i].size(), PRX_TEXT_BLUE);
                    centroid[0] /= ((double)object_verts[i].size());
                    centroid[1] /= ((double)object_verts[i].size());
                    get_longest_line(line, orig_points, obstacle_configs[i].second.get_position()[2]);
                    object_original_vertices[obstacle_names[i]] = line;

                    // ================================================
                    //   Sort the vertices in counter-clockwise order
                    // ================================================
                    for( unsigned t=0; t<object_verts[i].size(); ++t )
                    {
                        object_verts[i][t].sort_value = atan2( object_verts[i][t].point->memory[1] - centroid[1], object_verts[i][t].point->memory[0] - centroid[0] );
                    }
                    std::sort( object_verts[i].begin(), object_verts[i].end() );

                    // ==========================================================
                    //   Retrieving/Making the minkowski body for this obstacle
                    // ==========================================================
                    minkowski_body_t& minky = minkowski_vertices[ obstacle_names[i] ];
                    minky.x_offset = centroid[0];
                    minky.y_offset = centroid[1];
                    
                    unsigned nr_verts = object_verts[i].size();
                    
                    minky.vertices.resize( nr_verts );
                    for( unsigned k=0; k<nr_verts; ++k )
                    {
                        //We only need the 2D points
                        minky.vertices[k].resize(2);
                        //Try to compute this more efficiently without using sin/cos
                        next = (k+1)%nr_verts;
                        previous = (k-1);
                        if( previous > nr_verts )
                        {
                            previous = nr_verts-1;
                        }
                        //Now, compute the vectors corresponding to the sides
                        primary_vector[0] = object_verts[i][k].point->memory[0] - object_verts[i][previous].point->memory[0];
                        primary_vector[1] = object_verts[i][k].point->memory[1] - object_verts[i][previous].point->memory[1];
                        secondary_vector[0] = object_verts[i][k].point->memory[0] - object_verts[i][next].point->memory[0];
                        secondary_vector[1] = object_verts[i][k].point->memory[1] - object_verts[i][next].point->memory[1];
                        
                        //Make them normal
                        primary_vector.normalize();
                        secondary_vector.normalize();
                        
                        //Combine them
                        primary_vector += secondary_vector;
                        
                        //Re-scale to be minkowski sum distance
                        primary_vector *= (1.6 * agent_radius);
                        
                        minky.vertices[k][0] = object_verts[i][k].point->memory[0] + primary_vector[0];
                        minky.vertices[k][1] = object_verts[i][k].point->memory[1] + primary_vector[1];
                        // object_original_vertices[k][0] = object_verts[i][k].point->memory[0];
                        // object_original_vertices[k][1] = object_verts[i][k].point->memory[1];
                    }                    
                }

                // PRX_FATAL_S("stop");
                
                // ====================================================
                //   Go over all the non-elevator Minkowski Obstacles
                // ====================================================
                for( unsigned i=0; i<nr_non_elevators; ++i )
                {
                    // PRX_PRINT("Points for: " << obstacle_names[i], PRX_TEXT_RED);
                    // =========================================================
                    //   Create additional interpolated Proximity Query Points
                    // =========================================================
                    for( unsigned k=0; k<object_verts[i].size(); ++k )
                    {
                        unsigned l = (k+1)%(object_verts[i].size());

                        //Figure out how many interpolation points we want
                        double distance = navigation_space->distance( object_verts[i][k].point, object_verts[i][l].point );
                        double steps = std::floor( distance/2.0 );

                        //Then, for each interpolation point
                        for(double t=0; t<=steps; ++t)
                        {
                            //Get another node
                            pq_nodes.push_back( new proximity_query_node_t() );

                            //Fill up the node's information
                            pq_nodes.back()->obstacle_index = i;
                            pq_nodes.back()->point = navigation_space->alloc_point();
                            pq_nodes.back()->node_id = pq_nodes.size()-1;

                            navigation_space->interpolate( object_verts[i][k].point, object_verts[i][l].point, t/(steps+1.0), pq_nodes.back()->point );
                            // PRX_PRINT( navigation_space->print_point( pq_nodes.back()->point, 3 ), PRX_TEXT_LIGHTGRAY );
                            
                            //Add all points to the distance metric
                            obstacle_metric->add_point( pq_nodes.back() );
                        }
                    }
                }
                
                // ===============================================
                //   We need to do special things here for ramps
                // ===============================================
                
                
                // ====================================================
                //   Computing obstacles for each vertex of the graph
                // ====================================================
                std::vector< std::pair< std::vector< double > , std::vector< double > > > lines;
                std::vector< neighbor_t* > neighbors;
                
                if( obstacle_metric->get_nr_points() > 0 )
                {
                    prox_elements.resize( boost::num_vertices( navigation_graph.graph ) );
                    unsigned prox_index = 0;

                    std::vector< unsigned > found_indices;
                    //ALRIGHTY then, we just need to have appropriate things in the thing
                    foreach( undirected_vertex_index_t v, boost::vertices( navigation_graph.graph ) )
                    {
                        //Get the node
                        nav_node_t* node = navigation_graph.get_vertex_as< nav_node_t >( v );

                        //if the node corresponds to a triangle
                        if( node->has_triangle() )
                        {
                            //Allocate the prox_element
                            prox_elements[prox_index] = new prox_elem_t();

                            //Alright, do the query
                            double radius = 0.8;
                            
                            // PRX_PRINT("=================================================================", PRX_TEXT_BLUE);
                            // PRX_PRINT(" Doing query for node with triangle at: " << navigation_space->print_point(node->point, 3), PRX_TEXT_CYAN );
                            // PRX_PRINT("=================================================================", PRX_TEXT_BLUE);
                            
                            std::vector< const abstract_node_t* > query_result;
                            std::vector< const abstract_node_t* > ramp_results;
                            
                            std::vector< std::pair< double, unsigned > > final_results;
                            
                            space_point_t* query_point = navigation_space->alloc_point();
                            
                            //Clear out any previous information we might have found
                            prox_elements[prox_index]->config_names.clear();
                            prox_elements[prox_index]->configs.clear();
                            prox_elements[prox_index]->distances.clear();
                            prox_elements[prox_index]->geometries.clear();
                            neighbors.clear();
                            lines.clear();
                            
                            ramp_t* query_ramp = NULL;
                            double base_ramp_height = 0;
                            
                            // ===========================================
                            //  Extra checks for finding geometries below
                            // ===========================================
                            std::vector< bool > check_below(3);
                            for( unsigned b=0; b<3; ++b )
                            {
                                check_below[b] = false;
                            }
                            //If we are not associated with a ramp
                            if( node->near_ramp == NULL )
                            {
                                //Check our neighbors
                                foreach( undirected_vertex_index_t v, boost::adjacent_vertices( v, navigation_graph.graph ) )
                                {
                                    nav_node_t* neighbor = navigation_graph.get_vertex_as< nav_node_t >( v );
                                    //If this neighbor is not a triangle and it is associated with a ramp
                                    if( !neighbor->has_triangle() && neighbor->near_ramp != NULL )
                                    {
                                        //Get the height at the base of the ramp
                                        base_ramp_height = neighbor->near_ramp->get_base();
                                        
                                        //If the current triangle we are considering is well above the base of the stairs
                                        if( node->point->memory[2] - base_ramp_height > 2.5 )
                                        {
                                            query_ramp = neighbor->near_ramp;
                                            
                                            //Then its neighboring triangle vertices have to check below
                                            double dx1 = node->triangle[0][0] - neighbor->point->memory[0];
                                            double dy1 = node->triangle[0][1] - neighbor->point->memory[1];
                                            double dist_one = (dx1 * dx1) + (dy1 * dy1);
                                            
                                            double dx2 = node->triangle[1][0] - neighbor->point->memory[0];
                                            double dy2 = node->triangle[1][1] - neighbor->point->memory[1];                                            
                                            double dist_two = (dx2 * dx2) + (dy2 * dy2);
                                            
                                            //If they are equidistant, it must be that these are the guys who have to check below
                                            if( fabs( dist_one - dist_two ) < 0.01 )
                                            {
                                                check_below[0] = check_below[1] = true;
                                            }
                                            //Otherwise, compare the second and third points
                                            else
                                            {
                                                double dx3 = node->triangle[2][0] - neighbor->point->memory[0];
                                                double dy3 = node->triangle[2][1] - neighbor->point->memory[1];                                            
                                                double dist_three = (dx3 * dx3) + (dy3 * dy3);
                                                
                                                //If these two are equidistant, they must check below
                                                if( fabs( dist_two - dist_three ) < 0.01 )
                                                {
                                                    check_below[1] = check_below[2] = true;
                                                }
                                                //Otherwise, it must be the first and third points who straddle this point
                                                else if( fabs( dist_one - dist_three ) < 0.01 )
                                                {
                                                    check_below[0] = check_below[2] = true;
                                                }
                                                //If nothing panned out...
                                                else
                                                {
                                                    PRX_FATAL_S("YOUR LOGIC IS WRONG!  Fixit fixit fixit!");
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                            else
                            {
                                query_ramp = node->near_ramp;
                            }
                            
                            //Perform the query
                            query_result.clear();

                            //For each of its triangle vertices
                            for( unsigned t=0; t<node->triangle.size(); ++t )
                            {
                                //We need a query point to use
                                navigation_space->copy_vector_to_point( node->triangle[t], query_point );
                                query_point->memory[2] += graph_offset;

                                // PRX_PRINT("Computing for triangle vertex: " << navigation_space->print_point( query_point, 3 ), PRX_TEXT_BROWN);
                                query_result = obstacle_metric->radius_query( query_point, radius );
                                
                                for( unsigned d=0; d<query_result.size(); ++d )
                                {
                                    const proximity_query_node_t* result = dynamic_cast< const proximity_query_node_t* >(query_result[d]);

                                    double dif_x = result->point->memory[0] - query_point->memory[0];
                                    double dif_y = result->point->memory[1] - query_point->memory[1];
                                    double dist = sqrt( dif_x * dif_x + dif_y * dif_y );

                                    // PRX_PRINT("[" << d << "] : (" << obstacle_configs[result->obstacle_index].first << ") : " << result->sort_value << "  comp.dist: " << dist, PRX_TEXT_LIGHTGRAY );
                                    
                                    final_results.push_back( std::pair<double, unsigned>(dist, result->obstacle_index) );
                                }
                                
                                //If this node is associated with a ramp or we have been told to check below
                                if( node->near_ramp != NULL || check_below[t] )
                                {
                                    //Get the base height of the ramp if our node is the one associated with the ramp
                                    if( node->near_ramp != NULL )
                                    {
                                        base_ramp_height = node->near_ramp->get_base();                                    
                                    }
                                    
                                    //If we are much higher than this base
                                    if( query_point->memory[2] > base_ramp_height + 1 + graph_offset )
                                    {
                                        // PRX_PRINT("Grabbing results from below: " << query_result.size(), PRX_TEXT_BLUE);
                                        //Then we should probably also have a query around the height of the base of the ramp
                                        double stored_height = query_point->memory[2];
                                        query_point->memory[2] = base_ramp_height + graph_offset;

                                        // PRX_PRINT("Doing ramp stuff for triangle vertex: " << navigation_space->print_point( query_point, 3 ), PRX_TEXT_GREEN);

                                        //Perform the actual query
                                        ramp_results = obstacle_metric->radius_query( query_point, radius );
                                        //And add those results to the list
                                        for( unsigned i=0; i<ramp_results.size(); ++i )
                                        {
                                            const proximity_query_node_t* result = dynamic_cast< const proximity_query_node_t* >(ramp_results[i]);

                                            bool add_result = true;
                                            //If we are checking a point below
                                            if( check_below[t] )
                                            {
                                                const vector_t& pos = obstacle_configs[ result->obstacle_index ].second.get_position();
                                                //We must be careful only to add results which correspond to a side-wall
                                                add_result = query_ramp->near_side_wall( pos[0], pos[1] );
                                            }
                                            if( add_result )
                                            {
                                                double dif_x = result->point->memory[0] - query_point->memory[0];
                                                double dif_y = result->point->memory[1] - query_point->memory[1];
                                                double dist = sqrt( dif_x * dif_x + dif_y * dif_y );
                                                final_results.push_back( std::pair<double, unsigned>(dist, result->obstacle_index) );
                                            }
                                        }
                                        query_point->memory[2] = stored_height;
                                    }
                                }
                        
                            }
                            
                            std::sort( final_results.begin(), final_results.end() );

                            //DEBUG
                            // PRX_PRINT("Finalized Query set: " << query_result.size(), PRX_TEXT_RED);
                            // for( unsigned d=0; d<final_results.size(); ++d )
                            // {
                            //     PRX_PRINT("[" << d << "] : (" << obstacle_configs[final_results[d].second].first << ") : " << final_results[d].first, PRX_TEXT_LIGHTGRAY );
                            // }
                            
                            // PRX_PRINT("Getting the actual obstacles now: ", PRX_TEXT_GREEN);
                            std::vector<unsigned> wall_ids;
                            for( unsigned i=0; i<final_results.size(); ++i )
                            {
                                const std::pair<double, unsigned>& result = final_results[i];
                                unsigned conf_index = result.second;
                                
                                //If this prox_index has not been found in the past
                                if( std::find( found_indices.begin(), found_indices.end(), conf_index ) == found_indices.end() )
                                {
                                    //Mark this obstacle as having been found before
                                    found_indices.push_back( conf_index );

                                    // PRX_PRINT("Found: [" << conf_index << "] : " << obstacle_configs[conf_index].first, PRX_TEXT_LIGHTGRAY);
                                    
                                    //Finally, assign everything in
                                    prox_elements[prox_index]->config_names.push_back( obstacle_configs[conf_index].first );
                                    prox_elements[prox_index]->configs.push_back( obstacle_configs[conf_index].second );
                                    prox_elements[prox_index]->distances.push_back( result.first );
                                    prox_elements[prox_index]->geometries.push_back( obstacle_geoms[conf_index] );
                                    neighbors.push_back( &(obstacle_neighbors[conf_index]) );
                                    
                                    lines.resize( lines.size() + 1 );
                                    wall_ids.push_back(conf_index);
                                    lines.back().first.resize(3);
                                    lines.back().second.resize(3);
                                    get_longest_line(lines.back(), (minkowski_vertices[ obstacle_names[conf_index] ]).vertices, obstacle_configs[conf_index].second.get_position()[2]);
                                    // const std::vector< util::vector_t >& minky_points = (minkowski_vertices[ obstacle_names[conf_index] ]).vertices;
                                    // double first_dist = (minky_points[0][0] - minky_points[1][0])*(minky_points[0][0] - minky_points[1][0]) + (minky_points[0][1] - minky_points[1][1])*(minky_points[0][1] - minky_points[1][1]);
                                    // double second_dist = (minky_points[1][0] - minky_points[2][0])*(minky_points[1][0] - minky_points[2][0]) + (minky_points[1][1] - minky_points[2][1])*(minky_points[1][1] - minky_points[2][1]);
                                    // double z = obstacle_configs[conf_index].second.get_position()[2];
                                    // if( first_dist > second_dist )
                                    // {
                                    //     //Endpoints are averages of 0&3 and 1&2
                                    //     //X coordinate of first point
                                    //     lines.back().first.push_back( (minky_points[0][0] + minky_points[3][0])/2.0 );
                                    //     //Y coordinate of first point
                                    //     lines.back().first.push_back( (minky_points[0][1] + minky_points[3][1])/2.0 );
                                    //     //Z coordinate of the first point
                                    //     lines.back().first.push_back( z );
                                    //     //X coordinate of second point
                                    //     lines.back().second.push_back( (minky_points[1][0] + minky_points[2][0])/2.0 );
                                    //     //Y coordinate of second point
                                    //     lines.back().second.push_back( (minky_points[1][1] + minky_points[2][1])/2.0 );
                                    //     //Z coordinate of the second point
                                    //     lines.back().second.push_back( z );
                                    // }
                                    // else
                                    // {
                                    //     //Otherwise, Endpoints are 0&1 and 2&3
                                    //     //X coordinate of first point
                                    //     lines.back().first.push_back( (minky_points[0][0] + minky_points[1][0])/2.0 );
                                    //     //Y coordinate of first point
                                    //     lines.back().first.push_back( (minky_points[0][1] + minky_points[1][1])/2.0 );
                                    //     //Z coordinate of the first point
                                    //     lines.back().first.push_back( z );
                                    //     //X coordinate of second point
                                    //     lines.back().second.push_back( (minky_points[3][0] + minky_points[2][0])/2.0 );
                                    //     //Y coordinate of second point
                                    //     lines.back().second.push_back( (minky_points[3][1] + minky_points[2][1])/2.0 );
                                    //     //Z coordinate of the second point
                                    //     lines.back().second.push_back( z );
                                    // }
                                
                                    // line = object_original_vertices[obstacle_names[conf_index]];
                                    // PRX_PRINT("Found: [" << conf_index << "] : " << obstacle_configs[conf_index].first, PRX_TEXT_LIGHTGRAY);
                                    // PRX_PRINT("Found: [" << conf_index << "] : " << obstacle_configs[conf_index].second.get_position()[0] << "," <<  obstacle_configs[conf_index].second.get_position()[1] << "," << obstacle_configs[conf_index].second.get_position()[2], PRX_TEXT_LIGHTGRAY);
                                    // PRX_PRINT("Found: [" << conf_index << "] : " << (minkowski_vertices[ obstacle_names[conf_index] ]).x_offset << "," << (minkowski_vertices[ obstacle_names[conf_index] ]).y_offset, PRX_TEXT_MAGENTA);
                                    // PRX_PRINT("LINES1 : " << lines.back().first[0] << "," << lines.back().first[1] << "," << lines.back().first[2], PRX_TEXT_BROWN);
                                    // PRX_PRINT("LINES2 : " << lines.back().second[0] << "," << lines.back().second[1] << "," << lines.back().second[2], PRX_TEXT_BROWN);
                                    // PRX_PRINT("Original center: " << ((lines.back().first[0] + lines.back().second[0])/2.0) << "," << ((lines.back().first[1] + lines.back().second[1])/2.0), PRX_TEXT_MAGENTA);
                                    // PRX_PRINT("ORIGINAL Line: " << line.first[0] << "," << line.first[1] << "," << line.first[2], PRX_TEXT_CYAN);
                                    // PRX_PRINT("ORIGINAL Line: " << line.second[0] << "," << line.second[1] << "," << line.second[2], PRX_TEXT_CYAN);
                                    // PRX_PRINT("Original center: " << ((line.first[0] + line.second[0])/2.0) << "," << ((line.first[1] + line.second[1])/2.0), PRX_TEXT_MAGENTA);
                                    // PRX_PRINT("------------------------------------------------------------------------------", PRX_TEXT_BLUE);

                                    //Fill the node with the correct wall information. Queues need the original points of the
                                    //walls and no the minkowski vertices.
                                    if(wall_segment_structures_map.find(conf_index) != wall_segment_structures_map.end())
                                    {
                                        segment_structure = wall_segment_structures_map[conf_index];
                                    }
                                    else
                                    {
                                        line = object_original_vertices[obstacle_names[conf_index]];
                                        segment_structure = new segment_struct_t(conf_index, conf_index, points_t(line.first[0], line.first[1], line.first[2]), points_t(line.second[0], line.second[1], line.second[2]));
                                        wall_segment_structures_map[conf_index] = segment_structure;
                                    }
                                    node->insert_wall_segment_structure(segment_structure);

                                }

                            }
                            found_indices.clear();
                            navigation_space->free_point( query_point );                        
                        
                            //Set it up the thing
                            node->obstacle_info = prox_elements[prox_index++];
                            node->obstacle_neighbors = neighbors;
                            // for( unsigned i=0; i<wall_ids.size(); ++i )
                            // {
                            //     if(wall_segment_structures_map.find(wall_ids[i]) != wall_segment_structures_map.end())
                            //     {
                            //         segment_structure = wall_segment_structures_map[wall_ids[i]];
                            //     }
                            //     else
                            //     {

                            //         segment_structure = new segment_struct_t(wall_ids[i], wall_ids[i], points_t(lines[i].first[0], lines[i].first[1], lines[i].first[2]), points_t(lines[i].second[0], lines[i].second[1], lines[i].second[2]));
                            //         wall_segment_structures_map[wall_ids[i]] = segment_structure;
                            //     }
                            //     node->insert_wall_segment_structure(segment_structure);
                            // }
                            //node->add_new_wall_segment(lines, wall_ids);
                        }
                    }
                }

                PRX_PRINT("Completed setting up information onto the nav graph!", PRX_TEXT_GREEN );
                // PRX_FATAL_S("stop");
            }

            void VO_application_t::load_semantics(const parameter_reader_t * const reader)
            {
                //Attraction Types
                nr_attractor_types = 0;
                foreach( const parameter_reader_t* r, reader->get_list("AttractionTypes") )
                {
                    // PRX_PRINT("Attraction: " << r->get_attribute_as< std::string >("Name") << " " << index, PRX_TEXT_GREEN );
                    std::string name = r->get_attribute_as< std::string >("Name");
                    
                    if(r->has_attribute("strength_changes_over_time"))
                    {
                        foreach( const parameter_reader_t* inf_reader, r->get_list("strength_changes_over_time") )
                        {
                            attractors_influence_in_time.push_back(attractor_influence_t(nr_attractor_types, inf_reader->get_attribute_as<double>("time") * MSEC_2_FRAMES, inf_reader->get_attribute_as<double>("influence")));
                        }
                    }
                    attractor_index[ name ] = nr_attractor_types++;
                }

                //Agent types
                nr_agent_types = 0;
                foreach( const parameter_reader_t* r, reader->get_list("AgentTypes") )
                {
                    // PRX_PRINT("Agent: " << r->get_attribute_as< std::string >("Name") << " " << index, PRX_TEXT_BLUE );
                    std::string agent_type = r->get_attribute_as< std::string >("Name");
                    agent_index[ agent_type ] = nr_agent_types++;
                    agent_types.push_back( agent_type );
                    // std::vector<std::pair<double, double> > distributions;
                    // distributions.resize(attractor_index.size());
                    // foreach( const parameter_reader_t* q, r->get_list("AttractionStrengths") )
                    // {
                    //     std::string attr_name = q->get_attribute_as< std::string >("Name");
                    //     PRX_ASSERT(attractor_index.find(attr_name) != attractor_index.end());                        
                    //     std::vector< double > distribution = q->get_attribute_as< std::vector< double > >("Distribution");
                    //     PRX_ASSERT(distribution.size() == 2);
                    //     distributions[attractor_index[attr_name]] = std::make_pair(distribution[0], distribution[1]);


                    //     // PRX_PRINT("Attraction Type: " << attr_name, PRX_TEXT_CYAN);
                    //     // for(unsigned i=0; i<distribution.size(); ++i)
                    //     // {
                    //     //     PRX_PRINT("::: " << distribution[i], PRX_TEXT_LIGHTGRAY);
                    //     // }
                    // }
                    // agent_attractor_desires.push_back(distributions);

                    // std::vector< double > speed = r->get_attribute_as< std::vector< double > >("walking_speed_distribution");
                    // PRX_ASSERT(speed.size() == 2);
                    // agent_speed_distribution.push_back(std::make_pair(speed[0], speed[1]));

                }

                //Origin types
                nr_origin_types = 0;
                foreach( const parameter_reader_t* r, reader->get_list("OriginTypes") )
                {
                    // PRX_PRINT("Origin: " << r->get_attribute_as< std::string >("Name") << " " << index, PRX_TEXT_BROWN );
                    origin_index[ r->get_attribute_as< std::string >("Name") ] = nr_origin_types++;
                }
              
                //Local information
                world_structure.init_regions(reader, origin_index, attractor_index, MSEC_2_FRAMES);
            }

            void VO_application_t::load_semantics_from_file(const std::string& filename)
            {
                std::string full_path = folder_path + filename;
                PRX_PRINT("Going to read semantics from file: " << full_path, PRX_TEXT_GREEN);
                YAML::Node doc = YAML::LoadFile( full_path );
                //Attraction Types
                YAML::Node attraction_doc = doc["AttractionTypes"];
                nr_attractor_types = 0;
                for(unsigned i = 0; i < attraction_doc.size(); ++i)
                {
                    attractor_index[ attraction_doc[i]["Name"].as<std::string>() ] = nr_attractor_types;
                    if(attraction_doc[i]["strength_changes_over_time"])
                    {
                        for(unsigned j = 0; j<attraction_doc[i]["strength_changes_over_time"].size(); ++j)
                            attractors_influence_in_time.push_back(attractor_influence_t(nr_attractor_types,  attraction_doc[i]["strength_changes_over_time"][j]["time"].as<double>() * MSEC_2_FRAMES, attraction_doc[i]["strength_changes_over_time"][j]["influence"].as<double>() ));
                    }
                    nr_attractor_types++;
                }


                PRX_PRINT("ERRRR: 3 " << full_path, PRX_TEXT_RED);
                //Agent types
                nr_agent_types = 0;
                for(unsigned i = 0; i < doc["AgentTypes"].size(); ++i)
                {
                    std::string agent_type = doc["AgentTypes"][i]["Name"].as<std::string>();
                    agent_index[ agent_type ] = nr_agent_types++;
                    agent_types.push_back( agent_type );
                }

                PRX_PRINT("ERRRR: 4 " << full_path, PRX_TEXT_RED);
                //Origin types
                nr_origin_types = 0;
                for(unsigned i = 0; i < doc["OriginTypes"].size(); ++i)
                {
                    origin_index[ doc["OriginTypes"][i]["Name"].as<std::string>() ] = nr_origin_types++;
                }
              
                PRX_PRINT("ERRRR: 5 " << full_path, PRX_TEXT_RED);
                //Local information
                world_structure.init_regions_from_file(doc["Local"], origin_index, attractor_index, MSEC_2_FRAMES);
            }

            double VO_application_t::find_closest_door_for_region(const nav_node_t* node, region_t* region)
            {
                if(navigation_graph.components[node->index] == navigation_graph.components[region->nodes[0]->index])
                {
                    std::vector<undirected_vertex_index_t> goals;
                    foreach(const nav_node_t* door_node, region->nodes)
                    {
                        goals.push_back(door_node->index);
                    }

                    if( (goals.size() > 0) && (graph_search.solve( node->index, goals ) ) )
                    {
                        std::deque< util::undirected_vertex_index_t > path_vertices;
                        graph_search.extract_path( path_vertices );
                        double dist = 0;
                        for( size_t i = 0; i < path_vertices.size() - 1; ++i )
                        {
                            dist += metric->distance_function(navigation_graph[path_vertices[i]]->point, navigation_graph[path_vertices[i+1]]->point);
                        }
                        return dist;
                    }
                }                
                return PRX_INFINITY;

            }
            
            void VO_application_t::propagate_region_distances( region_t* region, int search_id, bool hindered )
            {
                //We need to set up each door as a start
                std::multimap< double, nav_queue_node_t > list;
                std::multimap< double, nav_queue_node_t >::iterator it;
                
                for( unsigned i=0; i<region->nodes.size(); ++i )
                {
                    list.insert( std::make_pair( 0, nav_queue_node_t( region->nodes[i], region->nodes[i], i ) ) );
                }
                
                //We need to know what kind of region we are dealing with
                bool is_origin = ( dynamic_cast< origin_t* >( region ) != NULL );

                unsigned expansions = 0;
                int cnt=0;
                //Then, begin the flood
                while( !list.empty() )
                {
                    //Look at the front of the list
                    it = list.begin();

                    //If we have not yet visited this node
                    if( it->second.node->search_id < search_id )
                    {
                        //DEBUG
                        ++expansions;
                        
                        //We reached it with the best cost so far, so update
                        it->second.node->stored_dist = it->first;
                        it->second.node->search_id = search_id;
                        if( hindered )
                        {
                            it->second.node->hindered_back_pointer[ region ] = it->second.back_pointer;
                        }
                        else
                        {
                            it->second.node->back_pointer[ region ] = it->second.back_pointer;
                        }
                        
                        if( !is_origin )
                        {
                            if( hindered )
                            {
                                it->second.node->hindered_attractor_distances.push_back( new attractor_info_t( (attractor_t*)region, it->first, it->second.door ) );
                            }
                            else
                            {
                                it->second.node->attractor_distances.push_back( new attractor_info_t( (attractor_t*)region, it->first, it->second.door ) );
                            }
                        }

                        if(hindered)
                            it->second.node->hindered_region_distances[region] = it->first;
                        else
                            it->second.node->region_distances[region] = it->first;

                        //Then, for each adjacent vertex
                        foreach(undirected_vertex_index_t u, boost::adjacent_vertices(it->second.node->index, navigation_graph.graph))
                        {
                            //Summon up the corresponding node
                            nav_node_t* neighbor = navigation_graph.get_vertex_as< nav_node_t >( u );
                            //If we are allowed to move in this direction (esc checks)
                            double transition_cost = check_transition( neighbor, it->second.node, hindered );
                            if( transition_cost > 0 )
                            {
                                //Then put it into the list
                                list.insert( std::make_pair( it->second.node->stored_dist + transition_cost, nav_queue_node_t( neighbor, it->second.node, it->second.door ) ) );
                            }
                        }
                    }
                    // if(region->name=="NJT-3198959")
                    // {
                    //     if(it->first > 0)
                    //     {
                    //         cnt++;
                    //     }
                    //     //PRX_PRINT("  propagate_region_distances detect region : " << region->name <<" dist:"<< it->first, PRX_TEXT_BLUE);
                    // }
                    
                    list.erase( it );
                }
                // if(region->name=="NJT-3198959")
                // {
                //     PRX_PRINT("  propagate_region_distances detect region : " << region->name <<" cnt:"<<cnt, PRX_TEXT_BLUE);
                // }
                // // DEBUG 
                // size_t nr_verts = boost::num_vertices( navigation_graph.graph );
                // PRX_PRINT("Propagated region: " << region << " where we visited " << expansions << "/" << nr_verts, PRX_TEXT_GREEN + is_origin);
                // if( expansions < nr_verts-1 )
                // {
                //     PRX_PRINT("We have a problem reaching region: " << region->name, PRX_TEXT_RED );
                // }
            }
            
            double VO_application_t::check_transition( const nav_node_t* source, const nav_node_t* destination, bool hindered )
            {
                //First, get what the transition cost of the edge is
                double transition_cost = navigation_graph.weights[ boost::edge( source->index, destination->index, navigation_graph.graph ).first ];
                
                //If both nodes are associated with the same ramp
                if( source->near_ramp != NULL && source->near_ramp == destination->near_ramp )
                {
                    //If that ramp has flow
                    if( source->near_ramp->is_escalator )
                    {
                        //If the escalator is going up
                        if( source->near_ramp->going_up )
                        {
                            //If we are trying to go down
                            if( destination->point->memory[2] < source->point->memory[2] )
                            {
                                //Disallow this
                                transition_cost = -1;
                            }
                        }
                        else //The escalator is going down
                        {
                            //If we are trying to go up
                            if( destination->point->memory[2] > source->point->memory[2] )
                            {
                                //Disallow this
                                transition_cost = -1;
                            }
                        }
                        //If we are hindered, we don't like escalators
                        if( hindered )
                        {
                            transition_cost *= hindered_escalator_cost;
                        }
                    }
                    //This is a normal staircase, check if we are hindered
                    else if( hindered )
                    {
                        //Make an insanely steep cost
                        transition_cost *= hindered_stair_cost;
                    }
                }
                
                return transition_cost;
            }

            void VO_application_t::collect_info_for_vertex(undirected_vertex_index_t v, int search_id)
            {
                std::multimap<double, nav_node_t*> list;
                std::multimap<double, nav_node_t*>::iterator it;
                std::multimap<double, nav_node_t*>::iterator it2;

                nav_node_t* start_node = navigation_graph.get_vertex_as< nav_node_t >( v );
                // start_node->search_id = search_id;
                list.insert( std::make_pair( 0, start_node ));

                nav_node_t* node;
                double s_dist;
                while(!list.empty())
                {
                    it = list.begin();
                    if(it->second->is_region && it->second->search_id != search_id && it->second->corresponding_region != start_node->corresponding_region)
                    {
                        PRX_PRINT(" collect_info_for_vertex detect region : " << it->second->corresponding_region->name <<" dist:"<< it->first, PRX_TEXT_BLUE);
                        start_node->region_distances[it->second->corresponding_region] = it->first;
                        if(dynamic_cast<attractor_t*>(it->second->corresponding_region) != NULL)
                        {
                            start_node->attractor_distances.push_back( new attractor_info_t((attractor_t*)it->second->corresponding_region, it->first, detect_door(it->second->corresponding_region, it->second->index)));                            
                        }
                        /* This is Andrew's Code - Now we are using region instead of origin*/
                        /*
                        if(dynamic_cast<origin_t*>(it->second->corresponding_region) != NULL)
                        {
                            start_node->origin_distances[(origin_t*)it->second->corresponding_region] = it->first;                            
                        }
                        else
                        {
                            start_node->attractor_distances.push_back( new attractor_info_t((attractor_t*)it->second->corresponding_region, it->first, detect_door(it->second->corresponding_region, it->second->index)));
                        }*/
                    }
                    it->second->search_id = search_id;
                    s_dist = it->first;

                    foreach(undirected_vertex_index_t u, boost::adjacent_vertices(it->second->index, navigation_graph.graph))
                    {
                        node = navigation_graph.get_vertex_as< nav_node_t >( u );
                        if(node->search_id != search_id)
                        {             
                            double dist = s_dist + metric->distance_function(it->second->point, node->point);
                            if(node->added_in_the_list == search_id)
                            {
                                if(node->stored_dist > dist)
                                {
                                    //it2 = list.find(node->stored_dist);
                                    list.insert( std::make_pair( dist , node ));
                                    node->stored_dist = dist;
                                    //list.erase(it2);
                                }
                            }
                            else
                            {
                                node->added_in_the_list = search_id;
                                node->stored_dist = dist;
                                list.insert( std::make_pair( dist , node ));
                                
                            }
                        }
                    }
                    list.erase(it);
                }
            }

            int VO_application_t::detect_door(region_t* region, util::undirected_vertex_index_t v)
            {
                for(unsigned i = 0; i < region->nodes.size(); ++i)
                {
                    if(region->nodes[i]->index == v)
                        return i;
                }
                PRX_WARN_S("wrong door");
                return -1;
            }

            void VO_application_t::collect_annotations()
            {
                PRX_PRINT("Computing semantic info...", PRX_TEXT_MAGENTA);
                PRX_PRINT("Origins Size: " << world_structure.get_origins().size() << "    Attractors: " << world_structure.get_attractors().size() ,PRX_TEXT_MAGENTA);
                std::deque< util::undirected_vertex_index_t > path_vertices;

                PRX_PRINT("There are " << boost::num_vertices( navigation_graph.graph ) << " vertices.", PRX_TEXT_CYAN );
                PRX_PRINT("And the metric has " << metric->get_nr_points(), PRX_TEXT_BLUE);

                //Inform all the origins inside the nav_graph
                int counter = 0;
                foreach(origin_t* origin, world_structure.get_origins())
                {
                    counter++;
                    //Inform  all the doorways
                    foreach(space_point_t* point, origin->get_inside_points())
                    {
                        const undirected_node_t* node = (const undirected_node_t*)metric->single_query( point );
                        PRX_ASSERT(node != NULL);
                        if(!navigation_space->equal_points(node->point, point))
                        {
                            PRX_PRINT("Origin " << origin->name << " is not connected in the main navigation graph", PRX_TEXT_RED);
                            PRX_PRINT("Origin point (" << counter << "): " << navigation_space->print_point(point,4), PRX_TEXT_LIGHTGRAY);
                            PRX_PRINT("Found point : " << navigation_space->print_point(node->point,4), PRX_TEXT_LIGHTGRAY);
                            //PRX_FATAL_S("Origin " << origin->name << " is not connected in the main navigation graph");
                        }
                        origin->nodes.push_back(navigation_graph.get_vertex_as<nav_node_t>(node->index));
                        origin->nodes.back()->set_region(origin);
                        
                    }
                }

                //Inform all the attractors inside the nav_graph
                foreach(attractor_t* attractor, world_structure.get_attractors())
                {
                    //Inform  all the doorways
                    foreach(space_point_t* point, attractor->get_inside_points())
                    {
                        const undirected_node_t* node = (const undirected_node_t*)metric->single_query( point );
                        PRX_ASSERT(node != NULL);
                        if(!navigation_space->equal_points(node->point, point))
                        {
                            //PRX_PRINT("Attractor " << attractor->name << " is not connected in the main navigation graph", PRX_TEXT_RED);
                            //PRX_PRINT("Attractor point: " << navigation_space->print_point(point,4),PRX_TEXT_LIGHTGRAY);
                            //PRX_PRINT("Found point    : " << navigation_space->print_point(node->point,4),PRX_TEXT_LIGHTGRAY);
                            //PRX_FATAL_S("Attractor " << attractor->name << " is not connected in the main navigation graph");
                        }
                        attractor->nodes.push_back(navigation_graph.get_vertex_as<nav_node_t>(node->index));
                        attractor->nodes.back()->set_region(attractor);

                    }  
                }

                //Compute all distances 
                double dist; 
                double best_dist;

                unsigned v_num =0;
                unsigned v_size = boost::num_vertices(navigation_graph.graph);

                PRX_PRINT("Finding the attractor/origin distances  ", PRX_TEXT_BROWN);
                
                //New approach: region-based (MUCH FASTER)
                foreach( attractor_t* attractor, world_structure.get_attractors() )
                {
                    ++v_num;
                    propagate_region_distances( attractor, v_num, false );
                    ++v_num;
                    propagate_region_distances( attractor, v_num, true );
                }
                foreach( origin_t* origin, world_structure.get_origins() )
                {
                    ++v_num;
                    propagate_region_distances( origin, v_num, false );
                    ++v_num;
                    propagate_region_distances( origin, v_num, true );
                }

                foreach( undirected_vertex_index_t v, boost::vertices( navigation_graph.graph ) )
                {
                    nav_node_t* node = navigation_graph.get_vertex_as< nav_node_t >( v );
                    //If we didn't end up with back-pointer information due to silliness in the model
                    if( node->back_pointer.empty() )
                    {
                        //Copy over one of his neighbor's information
                        nav_node_t* neighbor = navigation_graph.get_vertex_as< nav_node_t >( *(boost::adjacent_vertices( node->index, navigation_graph.graph ).first) );
                        node->copy_navigation_info( neighbor );
                    }
                }
                
                PRX_PRINT("Going over regions to setup new doorway information.", PRX_TEXT_GREEN);
                PRX_PRINT("evacuation frame: " << evacuation_frame,PRX_TEXT_CYAN);
                
                //Now, we need to go override some back-pointer information
                foreach( region_t* region, world_structure.get_all_regions() )
                {
                    if(evacuation_frame != PRX_INFINITY && region->is_evacuation_point())
                    {
                        evacuation_points.push_back(region);
                        // PRX_PRINT("\n------------\n\n------------\nEvacuation Point "  << region->name << "\n------------\n\n-------------\n",PRX_TEXT_MAGENTA);
                    }

                    //For each of this region's doorways
                    foreach( space_point_t* door_point, region->get_doorways() )
                    {
                        //Retrieve the nav-graph node which represents this door.
                        nav_node_t* door_node = const_cast< nav_node_t* >( dynamic_cast< const nav_node_t* >( metric->single_query( door_point ) ) );

                        //Then, for each possible region back-pointer we could be considering
                        foreach( region_t* query_region, door_node->back_pointer | boost::adaptors::map_keys )
                        {
                            //We will be aggregating neighbors who point to us
                            std::vector< nav_node_t* > neighbors;
                            std::vector< nav_node_t* > hindered_neighbors;
                            
                            //Find all of our neighbors who point to us for this particular back-pointer
                            foreach( undirected_vertex_index_t n, boost::adjacent_vertices( door_node->index, navigation_graph.graph ) )
                            {
                                nav_node_t* neighbor = navigation_graph.get_vertex_as< nav_node_t >( n );
                                //Regular BP
                                if(neighbor->triangle_id != -1)
                                {
                                    if( neighbor->back_pointer[ query_region ] == door_node )
                                    {
                                        neighbors.push_back( neighbor );
                                    }
                                    //Hindered BP
                                    if( neighbor->hindered_back_pointer[ query_region ] == door_node )
                                    {
                                        hindered_neighbors.push_back( neighbor );
                                    }
                                }
                            }


                            //We need to have a point on the other side of the door as the override
                            space_point_t* override_point = navigation_space->alloc_point();
                            navigation_space->copy_point( override_point, door_node->point );
                            
                            //We are going to do some vector math
                            vector_t offset_vector(2);
                            foreach( nav_node_t* neighbor, neighbors )
                            {
                                offset_vector[0] += door_node->point->memory[0] - neighbor->point->memory[0];
                                offset_vector[1] += door_node->point->memory[1] - neighbor->point->memory[1];
                            }
                            offset_vector.normalize();
                            
                            //Then, update the override point with this new vector information
                            override_point->memory[0] += offset_vector[0];
                            override_point->memory[1] += offset_vector[1];
                            
                            //This duder should override anybody who uses him
                            door_node->grandfather_override[ query_region ] = override_point;
                            door_node->hindered_grandfather_override[ query_region ] = override_point;

                            // PRX_PRINT( "Set override point: " << navigation_space->print_point( override_point, 3 ) << "   for door: " << navigation_space->print_point( door_node->point, 3 ) << "  for region: " << query_region->name, PRX_TEXT_BLUE );
                            
                            //For each of those neighbors
                            foreach( nav_node_t* neighbor, neighbors )
                            {
                                //We need to get all of its neighbors
                                foreach( undirected_vertex_index_t n, boost::adjacent_vertices( neighbor->index, navigation_graph.graph ) )
                                {
                                    nav_node_t* candidate_redirection_node = navigation_graph.get_vertex_as< nav_node_t >( n );
                                    //If that dude points to the neighbor
                                    if( candidate_redirection_node->back_pointer[ query_region ] == neighbor )
                                    {
                                        //Redirect him to the door
                                        candidate_redirection_node->back_pointer[ query_region ] = const_cast< nav_node_t* >( door_node );
                                        // PRX_PRINT("Redirecting (" << query_region->name << ") " << navigation_space->print_point( candidate_redirection_node->point, 3 ) << "  to point to -> " << navigation_space->print_point( door_node->point, 3 ), PRX_TEXT_BLUE );
                                    }
                                }
                            }

                            //For each of the hindered neighbors
                            foreach( nav_node_t* neighbor, hindered_neighbors )
                            {
                                //We need to get all of its neighbors
                                foreach( undirected_vertex_index_t n, boost::adjacent_vertices( neighbor->index, navigation_graph.graph ) )
                                {
                                    nav_node_t* candidate_redirection_node = navigation_graph.get_vertex_as< nav_node_t >( n );
                                    //If that dude points to the neighbor
                                    if( candidate_redirection_node->hindered_back_pointer[ query_region ] == neighbor )
                                    {
                                        //Redirect him to the door
                                        candidate_redirection_node->hindered_back_pointer[ query_region ] = const_cast< nav_node_t* >( door_node );
                                        // PRX_PRINT("Redirecting (" << query_region->name << ") " << navigation_space->print_point( candidate_redirection_node->point, 3 ) << "  to point to -> " << navigation_space->print_point( door_node->point, 3 ), PRX_TEXT_LIGHTGRAY );
                                    }
                                }
                            }
                            
                        }
                    }
                }


                // foreach( undirected_vertex_index_t v, boost::vertices( navigation_graph.graph ) )
                // {
                //     nav_node_t* node = navigation_graph.get_vertex_as< nav_node_t >( v );
                //     if(node->is_region)
                //     {
                //         PRX_DEBUG_COLOR("Node : " << node->corresponding_region->name, PRX_TEXT_CYAN);
                //         foreach(origin_t* origin, node->origin_distances | boost::adaptors::map_keys)
                //         {
                //             PRX_DEBUG_COLOR("  -Origin: " << origin->name << "  dist:" << node->origin_distances[origin], PRX_TEXT_GREEN);
                //         }

                //         foreach(attractor_info_t* attr, node->attractor_distances)
                //         {
                //             PRX_DEBUG_COLOR("  -Attractor: " << attr->print(), PRX_TEXT_BROWN);
                //         }

                //     }
                // }

                PRX_PRINT("Completed setting up semantic information onto the nav graph!", PRX_TEXT_GREEN );
            }

            void VO_application_t::load_agent_paths(const parameter_reader_t * const reader)
            {
                agent_path_index = 0;
                char junk_string[255];
                char junk;
                nr_paths = 0;

                if(reader->has_attribute("paths_filename"))
                {
                    std::string paths_file = folder_path + reader->get_attribute_as< std::string >("paths_filename");

                    //CSV file reading version
                    //open the file stream
                    std::ifstream fin;
                    fin.open( paths_file.c_str() );
                    //read in how many paths we have
                    fin >> nr_paths;
                    fin.getline( junk_string, 255, '\n' );
                    //for each of those
                    for( unsigned i=0; i<nr_paths; ++i )
                    {
                        //Push in a new OD info, having it read from the file
                        agent_paths.push_back( OD_info_t( &world_structure, agent_index, attractor_index, MSEC_2_FRAMES, fin ) );
                        // PRX_PRINT("Read agent : " << agent_paths.back().print(), PRX_TEXT_CYAN);
                    }

                    PRX_PRINT("Read " << nr_paths << " agent paths.", PRX_TEXT_CYAN);
                    
                    fin.close();
                }
                else
                {
                    PRX_WARN_S("WARNING!!! There is no paths file specified. No agents will be spawned");
                }

                std::sort(agent_paths.begin(), agent_paths.end());
            }

            void VO_application_t::start_sim_with_agents()
            {
                int curr_frame = 0;
                std::deque< nav_node_t* > path_to_goal;
                const nav_node_t* tmp_node;
                origin_t* exit_origin;
                const space_point_t* start_point;
                // PRX_PRINT("--- Start sim with agents at frame : " << frame_id << "    index: " << agent_path_index, PRX_TEXT_GREEN);
                while( agent_path_index < nr_paths && frame_id >= agent_paths[agent_path_index].arrival_frame)
                {
                    // PRX_PRINT("------ Agent " << agent_path_index << " arrives at : "<< agent_paths[agent_path_index].arrival_frame << "  leaves at: " << agent_paths[agent_path_index].departure_frame, PRX_TEXT_LIGHTGRAY );
                    if(agent_paths[agent_path_index].departure_frame > frame_id && can_spawn())
                    {
                        double t = agent_paths[agent_path_index].departure_frame - agent_paths[agent_path_index].arrival_frame;
                        double percentage = (frame_id-agent_paths[agent_path_index].arrival_frame)/t;                        
                        path_to_goal.clear();
                        exit_origin = agent_paths[agent_path_index].exit_origin;
                        start_point = agent_paths[agent_path_index].entrance->get_spawn_point(frame_id);
                        tmp_node = dynamic_cast< const nav_node_t* >( metric->single_query( start_point ) );
                        // PRX_PRINT(agent_path_index << ")Generate the agent: " << agent_paths[agent_path_index].print(), PRX_TEXT_GREEN);
                        // PRX_PRINT(agent_path_index << ") To compute percentage:  t:" << t << "    curr:" << (frame_id- agent_paths[agent_path_index].arrival_frame) << "     per:" << percentage, PRX_TEXT_GREEN);
                        // PRX_PRINT("Start point is: " << navigation_space->print_point(start_point, 5) << "   back pointers size: " << tmp_node->back_pointer.size(), PRX_TEXT_LIGHTGRAY);
                        while(tmp_node->corresponding_region == NULL || tmp_node->corresponding_region != exit_origin)
                        {
                            path_to_goal.push_back(tmp_node->back_pointer[ exit_origin ]);
                            // PRX_PRINT("node at: " << navigation_space->print_point(tmp_node->point, 5) << "   is triangle: " << tmp_node->triangle_id << "   neighTriangle:" << tmp_node->neighboring_triangles.size(), PRX_TEXT_BLUE);
                            tmp_node = tmp_node->back_pointer[ exit_origin ];
                        }
                        // PRX_PRINT(agent_path_index << ") got a path to its exit :" << exit_origin->name << "  path size: " << path_to_goal.size() << "     percentage: " << percentage, PRX_TEXT_CYAN);
                        int index = path_to_goal.size()*percentage;
                        if(path_to_goal.size() != 0)
                            PRX_PRINT(agent_path_index << ") it will be at the place : " << index << "/" << path_to_goal.size() << "      point:" << navigation_space->print_point(path_to_goal[index]->point, 5), PRX_TEXT_CYAN);

                        bool spawned = false;
                        if(path_to_goal.size() == 0)
                            spawned = spawn_agent(start_point);
                        else if(path_to_goal[index]->triangle_id != -1)
                            spawned = spawn_agent(path_to_goal[index]->point);
                        else 
                            spawned = spawn_agent(get_adjacent_triangle(path_to_goal[index]->index));
                        // PRX_PRINT("----------- Spawned :" << spawned, PRX_TEXT_GREEN);
                    }
                    ++agent_path_index;
                       
                }
                // PRX_PRINT("--- Done with agent path index: " << agent_path_index, PRX_TEXT_BLUE);
                // simulation_state_space->copy_to_point(simulation_state);
                update_once = true;
                //simulator->push_state(simulation_state);
                // PRX_PRINT("------ DONE WITH INIT AGENTS ------",PRX_TEXT_BROWN);
            }

            const space_point_t* VO_application_t::get_adjacent_triangle(undirected_vertex_index_t v)
            {
                foreach(undirected_vertex_index_t adj, boost::adjacent_vertices(v, navigation_graph.graph))
                {
                    if(navigation_graph.get_vertex_as<nav_node_t>(adj)->triangle_id != -1)
                        return navigation_graph[adj]->point;
                }
                PRX_FATAL_S("There is no adjacent triangle on a navigation node! There is problem with the model!");
                return NULL;
            }

            void VO_application_t::open_file_output()
            {
                //Alright, so then, let's open up a stream for each plant
                file_streams.resize( plants.size() );

                std::string trajectories_path = folder_path + "trajectories/";

                boost::filesystem::path output_dir (trajectories_path);
                if (!boost::filesystem::exists(output_dir))
                {
                    boost::filesystem::create_directories( output_dir );
                }

                //Then, for each one
                for( unsigned i=0; i<file_streams.size(); ++i )
                {
                    //Get the correct path
                    std::string odir(trajectories_path.c_str());

                    std::stringstream s2;
                    s2 << "path_" << node_id << "_" << i << ".path";
                    odir += (s2.str());

                    //create the stream
                    file_streams[i] = new std::ofstream();

                    //And open that file
                    file_streams[i]->open( odir.c_str());
                    
                    if( !file_streams[i]->good() )
                    {
                        PRX_WARN_S("Something is wrong with file stream: (" << i << "):" << odir.c_str());
                    }
                }

                std::string moe_filename = folder_path +"/node_" + int_to_str(node_id) + "_moe_stats.txt";
                moe_file_stream.open(moe_filename.c_str());

                std::string simuation_filename = folder_path +"/node_" + int_to_str(node_id) + "_simuation_file.paths";
                simulation_file_stream.open(simuation_filename.c_str());
                simulation_file_stream << plants.size() << "," << MSEC_2_FRAMES << "\n";
            }

            void VO_application_t::gather_systems()
            {
                collision_avoidance_controller_t* cont;
                path_follow_controller_t* follower;
                behavior_controller_t* behavior;

                std::vector<system_graph_t::directed_vertex_t> get_plants;
                sys_graph.get_plant_vertices(get_plants);
                system_ptr_t test_plant, test_system;
                system_graph_t::directed_vertex_t test_vertex;
                foreach(system_graph_t::directed_vertex_t v, get_plants )
                {
                    // PRX_DEBUG_COLOR ("Testing ", PRX_TEXT_MAGENTA);
                    test_plant = sys_graph.system_graph[v].system;
                    test_vertex = v;
                    test_vertex = sys_graph.system_graph[test_vertex].parent_vertex;
                    test_system = sys_graph.system_graph[test_vertex].system;

                    bool vo_plant = false;
                    while (test_system != NULL)
                    {
                        // PRX_WARN_S ("Checking system: " << test_system->get_pathname());
                        follower = NULL;
                        behavior = NULL;
                        cont = dynamic_cast< collision_avoidance_controller_t* >(test_system.get());
                        if (cont != NULL )
                        {
                            vo_plants[test_plant->get_pathname()] = dynamic_cast<plant_t*>(test_plant.get());
                            plant_to_VO[test_plant->get_pathname()] = cont;
                            vo_plant = true;

                            //Alright, let's record some stuff
                            plants.push_back( dynamic_cast<plant_t*>(test_plant.get()) );
                            vo_controllers.push_back( cont );
                            follower = dynamic_cast< path_follow_controller_t* >( sys_graph.system_graph[ sys_graph.system_graph[test_vertex].parent_vertex ].system.get() );
                            PRX_ASSERT( follower != NULL );
                            path_follower_controllers.push_back( follower );
                            behavior = dynamic_cast< behavior_controller_t* >( sys_graph.system_graph[ sys_graph.system_graph[ sys_graph.system_graph[test_vertex].parent_vertex ].parent_vertex ].system.get() );
                            PRX_ASSERT( behavior != NULL );
                            behavior_controllers.push_back( behavior );

                            //Also make sure the plant knows its corresponding path controller
                            (dynamic_cast< pedestrian_t* >(test_plant.get()))->link_path_follower( follower );

                            // PRX_DEBUG_COLOR ("Found VO controller :: plant: " << test_plant->get_pathname() << "  vo_controller: " << vo_controllers.back()->get_pathname() << "  follower: " << path_follower_controllers.back()->get_pathname(), PRX_TEXT_GREEN);

                            break;
                        }
                        test_vertex = sys_graph.system_graph[test_vertex].parent_vertex;
                        test_system = sys_graph.system_graph[test_vertex].system;
                    }

                    if (!vo_plant)
                    {
                        other_plants[test_plant->get_pathname()] = dynamic_cast<plant_t*>(test_plant.get());
                    }
                }
                
                //Now that we have all the cast things, make the agent pool
                all_agents.resize( behavior_controllers.size() );
                all_agents_size = all_agents.size();
                agent_pool.resize( all_agents_size );

                PRX_PRINT("all_agents size:"<<all_agents.size(),PRX_TEXT_GREEN);
                for( unsigned i=0; i<all_agents_size; ++i )
                {
                    all_agents[i].agent_id = i;
                    all_agents[i].plant = (pedestrian_t*)plants[i];
                    all_agents[i].behavior_controller = behavior_controllers[i];
                    all_agents[i].VO_controller = vo_controllers[i];
                    all_agents[i].VO_controller->set_id( i );
                    all_agents[i].path_controller = path_follower_controllers[i];
                    all_agents[i].path_controller->set_id( i );
                    
                    all_agents[i].set_active( false );
                    agent_pool[i] = &(all_agents[i]);
                }
                
                set_systems_to_sensing();
            }


            void VO_application_t::find_structures()
            {
                //Create a parameter reader which will read in geom information from the obstacles
                parameter_reader_t* reader = new parameter_reader_t( ros::this_node::getName(), global_storage );
                
                parameter_reader_t::reader_map_t obstacle_map;
                if(ros::param::has("prx/obstacles"))
                {
                    parameter_reader_t* obstacle_reader = new parameter_reader_t("prx/obstacles", global_storage);
                    obstacle_map = obstacle_reader->get_map("");                
                    delete obstacle_reader;
                }
                else
                    obstacle_map = parameters::get_map( simulator->get_pathname() + "/obstacles", reader, NULL);

                // std::string sim_path_name = ros::this_node::getName() + "/" + simulator->get_pathname();
                // parameter_reader_t::reader_map_t obstacle_map = parameters::get_map( sim_path_name + "/obstacles", reader, NULL);
                delete reader;
                
                space_point_t* query_point = navigation_space->alloc_point();

                // PRX_PRINT("Map has " << obstacle_map.size() << " elements:", PRX_TEXT_LIGHTGRAY);

                triangle_counter = 0;
                foreach(const parameter_reader_t::reader_map_t::value_type key_value, obstacle_map)
                {
                    const parameter_reader_t* obstacle_template_reader = NULL;

                    if( key_value.second->has_attribute("template") )
                    {
                        std::string template_name = key_value.second->get_attribute("template");
                        // TODO: Find a way to load templates under namespaces more cleanly.
                        obstacle_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + template_name, global_storage);
                    }

                    foreach(const parameter_reader_t* r, parameters::get_list("geometries", key_value.second, obstacle_template_reader))
                    {
                        std::string geom_type = r->get_attribute_as< std::string > ("collision_geometry/type");
                        // PRX_PRINT("Type: " << geom_type, PRX_TEXT_LIGHTGRAY);
                        if( geom_type == "polygon" )
                        {
                            // = = = = = = = = = = = =
                            // Checking for Stairs
                            // = = = = = = = = = = = =
                            std::vector< double > triangles = r->get_attribute_as< std::vector<double> >("collision_geometry/triangles");
                            PRX_ASSERT( triangles.size() >= 3 );

                            double min_height, max_height;
                            min_height = max_height = triangles[2];
                            for( unsigned i=0; i<triangles.size(); i+=3 )
                            {
                                double z_val = triangles[i+2];
                                if( z_val < min_height )
                                {
                                    min_height = z_val;
                                }
                                if( z_val > max_height )
                                {
                                    max_height = z_val;
                                }
                            }

                            double height = max_height - min_height;

                            bool is_escalator = r->has_attribute("escalator");
                            bool direction = false;
                            if( is_escalator )
                            {
                                direction = (r->get_attribute_as< std::string >("escalator") == "up");
                            }

                            if( height > 0.001 )
                            {
                                //We need to construct a representation for the slope
                                register_ramp( triangles, height, min_height, is_escalator, direction );
                            }
                            
                            // = = = = = = = = = = = = = = = = = = = = = = = =
                            // Need to give triangle information to each node
                            // = = = = = = = = = = = = = = = = = = = = = = = =

                            for( unsigned i=0; i<triangles.size(); i += 9 )
                            {
                                //Construct a query point for the triangle
                                query_point->memory[0] = (triangles[i] + triangles[i+3] + triangles[i+6])/3.0;
                                query_point->memory[1] = (triangles[i+1] + triangles[i+4] + triangles[i+7])/3.0;
                                query_point->memory[2] = (triangles[i+2] + triangles[i+5] + triangles[i+8])/3.0 + graph_offset;
                                //Get the corresponding closest node
                                undirected_node_t* close_node = (undirected_node_t*)metric->single_query( query_point );
                                //As long as it is reasonably close to where we think it should be
                                if( navigation_space->distance( close_node->point, query_point ) < 0.5 )
                                {
                                    //Add this triangle to the nav node
                                    nav_node_t* nav_node = dynamic_cast< nav_node_t* >( close_node );
                                    if( nav_node != NULL )
                                    {
                                        //This is assuming that we have the points in the right order... which they need to be for visualization...
                                        nav_node->set_triangle(triangle_counter, triangles.begin() + i );
                                        triangle_counter++;
                                    }
                                    else
                                    {
                                        PRX_FATAL_S("Something went wrong: we have a node that's not a nav node?");
                                    }
                                }
                            }
                            
                            // = = = = = = = = = = = =
                            // Checking for Elevators
                            // = = = = = = = = = = = =
                            if( r->get_attribute_as<bool>("isElevator") )
                            {
                                //We need to figure out the bounds of the polygon.
                                double min_x = PRX_INFINITY, min_y = PRX_INFINITY;
                                double max_x = -PRX_INFINITY, max_y = -PRX_INFINITY;
                                for( unsigned i=0; i<triangles.size(); i+=3 )
                                {
                                    if( triangles[i] < min_x )
                                    {
                                        min_x = triangles[i];
                                    }
                                    
                                    if( triangles[i] > max_x )
                                    {
                                        max_x = triangles[i];
                                    }

                                    if( triangles[i+1] < min_y )
                                    {
                                        min_y = triangles[i+1];
                                    }
                                    
                                    if( triangles[i+1] > max_y )
                                    {
                                        max_y = triangles[i+1];
                                    }
                                }

                                //Make our best guess of what is the central point in the shaft
                                double a_x = (min_x + max_x) / 2.0;
                                double a_y = (min_y + max_y) / 2.0;
                                query_point->memory[0] = a_x;
                                query_point->memory[1] = a_y;
                                query_point->memory[2] = triangles[2] + graph_offset;
                                //Get the corresponding closest node
                                const nav_node_t* close_node = dynamic_cast< const nav_node_t* >( metric->single_query( query_point ) );

                                //If the close node is within the elevator bounds
                                if( close_node->point->memory[0] > min_x && close_node->point->memory[0] < max_x  &&  close_node->point->memory[1] > min_y && close_node->point->memory[1] < max_y )
                                {
                                    //We might not have actually ended up with a triangle here, so we need to find one
                                    if( !close_node->has_triangle() )
                                    {
                                        const nav_node_t* found_node = NULL;
                                        //For each of its neighbors
                                        foreach( undirected_vertex_index_t v, boost::adjacent_vertices( close_node->index, navigation_graph.graph ) )
                                        {
                                            //Get the actual node
                                            const nav_node_t* neighbor = navigation_graph.get_vertex_as< nav_node_t >( v );
                                            //If it has triangle information and is not on a separate floor
                                            if( neighbor->has_triangle() && fabs( close_node->point->memory[2] - neighbor->point->memory[2] ) < 0.1 )
                                            {
                                                //This is a node we have found
                                                found_node = neighbor;
                                            }
                                        }
                                        
                                        if( found_node != NULL )
                                        {
                                            close_node = found_node;
                                        }
                                        else
                                        {
                                            PRX_FATAL_S("Could not find a suitable triangle node for the elevator!");
                                        }
                                    }
                                    //Then try to add this part of the elevator to the world structure
                                    world_structure.add_elevator( min_x, max_x, min_y, max_y, std::pair<double, const nav_node_t*>( triangles[2], close_node ) );
                                }
                                else
                                {
                                    PRX_WARN_S( "Unable to detect suitable connections for elevator stop: " << r->get_attribute_as<std::string>("name") );
                                }
                            }
                        }
                    }
                    
                    if(obstacle_template_reader != NULL)
                        delete obstacle_template_reader;
                }

                // Now that we have all of the ramps, make sure the structure knows them
                for( unsigned i=0; i<ramps.size(); ++i )
                {
                    world_structure.add_ramp( ramps[i] );
                }
                
                // We also need to update the graph to incorporate all of the elevator stops
                const std::vector< elevator_t* >& elevators = world_structure.get_elevators();
                
                //For each elevator
                for( unsigned i=0; i<elevators.size(); ++i )
                {
                    //Get the pointer
                    elevator_t* e = elevators[i];
                    
                    //We know they are done at this point, so go ahead and compute their shafts
                    e->name = "elevator_";
                    e->name += int_to_str(i);
                    e->name += "/shaft";
                    e->compute_shaft_geometry();
                    
                    //collect a list of all the vertex indices at the stops
                    std::vector< undirected_vertex_index_t > elevator_vertices;
                    //for each stop
                    for( unsigned j=0; j<e->stops.size(); ++j )
                    {
                        //Make our best guess of what is the central point in the shaft
                        query_point->memory[0] = (e->bounds[0].get_lower_bound() + e->bounds[0].get_upper_bound()) / 2.0;
                        query_point->memory[1] = (e->bounds[1].get_lower_bound() + e->bounds[1].get_upper_bound()) / 2.0;
                        query_point->memory[2] = e->stops[j].first + graph_offset;
                        //Get the corresponding closest node
                        const undirected_node_t* close_node = (const undirected_node_t*)metric->single_query( query_point );
                        //As long as it is reasonably close to where we think it should be
                        if( navigation_space->distance( close_node->point, query_point ) < 1.0 )
                        {
                            //Add the nearest point to the list
                            elevator_vertices.push_back( close_node->index );
                        }
                    }
                    //For each found vertex
                    for( unsigned j=0; j<elevator_vertices.size(); ++j )
                    {
                        //For each other found vertex
                        for( unsigned k=j+1; k<elevator_vertices.size(); ++k )
                        {
                            //Add an edge between these vertices
                            navigation_graph.add_edge<undirected_edge_t>(elevator_vertices[j], elevator_vertices[k], 25 + 10*e->stops.size()); //TODO: how are we dealing with this in general?
                        }
                    }
                }
                
                // PRX_PRINT("======================================", PRX_TEXT_MAGENTA);
                // PRX_PRINT("  Associating nodes with elevators: ", PRX_TEXT_BLUE );
                // PRX_PRINT("======================================", PRX_TEXT_MAGENTA);
                
                //Do an initial pass over all of the vertices to see if that vertex lies entirely inside an elevator, if so, always have this association
                unsigned index = 0;
                foreach( undirected_vertex_index_t v, boost::vertices( navigation_graph.graph ) )
                {
                    //Get the node
                    nav_node_t* node = navigation_graph.get_vertex_as< nav_node_t >( v );

                    //Figuring out if we are at an elevator
                    bool found_structure = false;
                    for( unsigned i=0; i<elevators.size() && !found_structure; ++i )
                    {
                        //Check if the node itself is a doorway of the elevator
                        if( elevators[i]->in_elevator( node->point->memory[0], node->point->memory[1], node->point->memory[2] ) )
                        {
                            // PRX_PRINT("Node : " << navigation_space->print_point( node->point, 3 ) << "  =>  " << i, PRX_TEXT_LIGHTGRAY);
                            node->near_elevator = elevators[i];
                            node->elevator_index = i;
                            found_structure = true;
                        }
                    }
                }
                    
                // PRX_PRINT("======================================", PRX_TEXT_MAGENTA);
                foreach( undirected_vertex_index_t v, boost::vertices( navigation_graph.graph ) )
                {
                    //Get the node
                    nav_node_t* node = navigation_graph.get_vertex_as< nav_node_t >( v );

                    //Figuring out if we are at an elevator
                    bool found_structure = false;
                    //Do a second pass to see if we are adjacent to an elevator    
                    for( unsigned i=0; i<elevators.size(); ++i )
                    {
                        //Also want to associate nodes which have triangle points inside the elevator's effective region
                        if( node->has_triangle() )
                        {
                            //If the elevator even stops at this floor
                            if( elevators[i]->has_stop_at_height( node->point->memory[2] ) )
                            {
                                //Go over all of these points
                                for( unsigned k=0; k<node->triangle.size(); ++k )
                                {
                                    if( elevators[i]->approximately_in_elevator( node->triangle[k][0], node->triangle[k][1], node->triangle[k][2] ) )
                                    {
                                        //If this node is already associated with an elevator
                                        if( node->near_elevator != NULL )
                                        {
                                            //Need to see if we are closer to this one?
                                            double dist_new = elevators[i]->distance( node->point->memory[0], node->point->memory[1] );
                                            double dist_old = node->near_elevator->distance( node->point->memory[0], node->point->memory[1] );
                                            if( dist_new < dist_old )
                                            {
                                                // PRX_PRINT("Node : " << navigation_space->print_point( node->point, 3 ) << "  =>  " << i, PRX_TEXT_BROWN);
                                                node->near_elevator = elevators[i];
                                                node->elevator_index = i;
                                            }
                                        }
                                        //Otherwise, simply associate it
                                        else
                                        {
                                            // PRX_PRINT("Node : " << navigation_space->print_point( node->point, 3 ) << "  =>  " << i, PRX_TEXT_LIGHTGRAY);
                                            node->near_elevator = elevators[i];
                                            node->elevator_index = i;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    
                    //Now, to figure out if we are on stairs
                    found_structure = false;
                    for( unsigned i=0; i<ramps.size() && !found_structure; ++i )
                    {
                        //If we're possibly on this ramp
                        if( ramps[i]->on_ramp( node->point->memory[0], node->point->memory[1], -1.5 ) )
                        {
                            //And the height of this point is approximately where we expect
                            if( fabs( ramps[i]->height_on_ramp( node->point->memory[0], node->point->memory[1] ) - node->point->memory[2] ) < 1.0 )
                            {
                                //Then this must be our ramp
                                node->near_ramp = ramps[i];
                                node->ramp_index = i;
                                found_structure = true;
                            }
                        }
                    }
                    ++index;
                }
                
                //We're going to need some points to do checks and stuff
                space_point_t* point_a = navigation_space->alloc_point();
                space_point_t* point_b = navigation_space->alloc_point();
                std::vector< double > segment(6);
                
                //Alright, now finally, we have to determine where the segments between triangles are
                foreach( undirected_vertex_index_t v, boost::vertices( navigation_graph.graph ) )
                {
                    //Get the node
                    nav_node_t* node = navigation_graph.get_vertex_as< nav_node_t >( v );
                    nav_node_t* neighbor_triangle = NULL;
                    
                    //If the node is not a triangle, it must be connected to triangles somewhere
                    if( !node->has_triangle() )
                    {
                        //Debug stuff: check that each of these has exactly two triangle neighbors
                        unsigned count = 0;
                        
                        //Go over each of its neighbors
                        foreach( undirected_vertex_index_t u, boost::adjacent_vertices( v, navigation_graph.graph ) )
                        {
                            nav_node_t* neighbor = navigation_graph.get_vertex_as< nav_node_t >( u );
                            if( neighbor->has_triangle() && fabs( node->point->memory[2] - neighbor->point->memory[2] ) < 2.0 )
                            {
                                neighbor_triangle = neighbor;
                                ++count;
                            }   
                        }
                        
                        //DEBUG
                        if( count != 2 )
                        {
                            PRX_PRINT(":: " << navigation_space->print_point(node->point, 3), PRX_TEXT_MAGENTA);
                            PRX_FATAL_S("Non-triangle node does not have 2 triangle neighbors, has " << count);
                        }
                        
                        //We need to find which triangle side corresponds to this point
                        int side_index = -1;
                        for( unsigned i=0; i<3 && side_index < 0; ++i )
                        {
                            //Set the query point information to be the averages here
                            query_point->memory[0] = (neighbor_triangle->triangle[i][0] + neighbor_triangle->triangle[(i+1)%3][0]) / 2;
                            query_point->memory[1] = (neighbor_triangle->triangle[i][1] + neighbor_triangle->triangle[(i+1)%3][1]) / 2;
                            query_point->memory[2] = ((neighbor_triangle->triangle[i][2] + neighbor_triangle->triangle[(i+1)%3][2]) / 2) + graph_offset;
                            
                            //If it is really close to the point we expect to be at
                            if( navigation_space->distance( query_point, node->point ) < 0.1 )
                            {
                                //This must be the side we use
                                side_index = i;
                            }

                        }
                        
                        //DEBUG
                        if( side_index < 0 )
                        {
                            PRX_PRINT(":: " << navigation_space->print_point(node->point, 3), PRX_TEXT_MAGENTA);
                            PRX_FATAL_S("Node could not find a corresponding segment in neighboring triangles");
                        }
                        
                        //Now we know what points we have to get, so get them
                        point_a->memory.assign( neighbor_triangle->triangle[side_index].begin(), neighbor_triangle->triangle[side_index].begin()+3 );
                        point_b->memory.assign( neighbor_triangle->triangle[(side_index +1)%3].begin(), neighbor_triangle->triangle[(side_index +1)%3].begin()+3 );
                        
                        //Alright, now we have to figure out how much has to get chopped off from this segment
                        double length = navigation_space->distance( point_a, point_b );
                        double buffer_distance = PRX_MAXIMUM( 1.2 * agent_radius, 0.1 * length );
                        
                        if( buffer_distance/length > 0.5 )
                        {
                            PRX_PRINT("TOO NARROW! :: " << navigation_space->print_point(node->point, 3), PRX_TEXT_MAGENTA);
                            buffer_distance = length*0.5;
                        }
                        else if( buffer_distance/length > 0.25 )
                        {
                            PRX_PRINT("Very narrow area: " << navigation_space->print_point( node->point, 3 ), PRX_TEXT_BROWN );
                        }
                        
                        //Then, find the new endpoints
                        navigation_space->interpolate( point_a, point_b, buffer_distance / length, query_point );
                        segment[0] = query_point->memory[0];
                        segment[1] = query_point->memory[1];
                        segment[2] = query_point->memory[2];
                        navigation_space->interpolate( point_a, point_b, (length - buffer_distance) / length, query_point );
                        segment[3] = query_point->memory[0];
                        segment[4] = query_point->memory[1];
                        segment[5] = query_point->memory[2];
                        
                        node->set_segment( segment.begin() );
                    }
                }               
                
                navigation_space->free_point( query_point );
                navigation_space->free_point( point_a );
                navigation_space->free_point( point_b );
                
                // world_structure.print();
            }

            void VO_application_t::register_ramp( const std::vector< double >& points, double height, double base, bool is_escalator, bool goes_up )
            {
                //So, we need to find the points which are high, and the points which are low.
                std::vector< std::vector< double > > high_points;
                std::vector< std::vector< double > > low_points;
                double length = PRX_INFINITY;
                double difx, dify;

                ramps.push_back( new ramp_t() );
                unsigned ramp_index = ramps.size() - 1;

                //Go over the points
                for( unsigned i=0; i<points.size(); i+=3 )
                {
                    std::vector< double > point( points.begin()+i, points.begin()+(i+3) );

                    //If it is a high point
                    if( point[2] >= base + height - 0.001 )
                    {
                        //If we have not seen this point
                        if( std::find( high_points.begin(), high_points.end(), point ) == high_points.end() )
                        {
                            //Put it in there
                            high_points.push_back( point );
                        }
                    }
                    else if( point[2] < base + 0.001 )
                    {
                        //If we have not seen this point
                        if( std::find( low_points.begin(), low_points.end(), point ) == low_points.end() )
                        {
                            //Put it in there
                            low_points.push_back( point );
                        }
                    }
                }

                //Alright, let's compute the slope
                std::vector< std::vector< unsigned > > sides;

                //Then, between all pairs of high and low points, see if we can find the length
                for( unsigned i=0; i<high_points.size(); ++i )
                {
                    for( unsigned j=0; j<low_points.size(); ++j )
                    {
                        difx = high_points[i][0] - low_points[j][0];
                        dify = high_points[i][1] - low_points[j][1];
                        double sq_dist = ((difx) * (difx)) + ((dify) * (dify));
                        if( sq_dist <= length )
                        {
                            length = sq_dist;
                        }
                    }
                }

                //Then, using that length, figure out which two are the sides
                for( unsigned i=0; i<high_points.size(); ++i )
                {
                    for( unsigned j=0; j<low_points.size(); ++j )
                    {
                        difx = high_points[i][0] - low_points[j][0];
                        dify = high_points[i][1] - low_points[j][1];
                        double sq_dist = ((difx) * (difx)) + ((dify) * (dify));
                        if( sq_dist <= length + PRX_DISTANCE_CHECK )
                        {
                            sides.resize( sides.size() + 1 );
                            sides[sides.size()-1].push_back( j );
                            sides[sides.size()-1].push_back( i );
                        }
                    }
                }
                length = sqrt( length );

                if( sides.size() < 2 )
                {
                    PRX_PRINT("Unable to create ramp: improperly formed!", PRX_TEXT_RED);
                    PRX_PRINT("We created: " << sides.size() << " sides.", PRX_TEXT_LIGHTGRAY);
                    PRX_PRINT("From High Points: " << high_points.size(), PRX_TEXT_CYAN);
                    for( unsigned i=0; i<high_points.size(); ++i )
                    {
                        PRX_PRINT( "::" << high_points[i][0] << ", " << high_points[i][1] << ", " << high_points[i][2], PRX_TEXT_LIGHTGRAY );
                    }
                    PRX_PRINT("And Low Points: " << low_points.size(), PRX_TEXT_BROWN);
                    for( unsigned i=0; i<low_points.size(); ++i )
                    {
                        PRX_PRINT( "::" << low_points[i][0] << ", " << low_points[i][1] << ", " << low_points[i][2], PRX_TEXT_LIGHTGRAY );
                    }
                    PRX_PRINT("That's a height of: " << height, PRX_TEXT_GREEN);
                    PRX_PRINT("Using " << (points.size())/3.0 << " points:", PRX_TEXT_MAGENTA);
                    for( unsigned i=0; i<points.size(); i+=3 )
                    {
                        PRX_PRINT( "::" << points[i] << ", " << points[i+1] << ", " << points[i+2], PRX_TEXT_LIGHTGRAY );
                    }
                    // while(1);
                    // PRX_FATAL_S("FIX THIS NOW!");
                    
                    ramps.resize( ramps.size()-1 );
                    return;
                }

                PRX_ASSERT( low_points.size() >= 2 );
                PRX_ASSERT( high_points.size() >= 2 );

                // if( low_points.size() > 2 )
                // {
                //     PRX_WARN_S("A ramp has 3 or more low points...");
                // }
                // if( high_points.size() > 2 )
                // {
                //     PRX_WARN_S("A ramp has 3 or more high points...");
                // }

                //Alright, we have the ramp's slope
                ramps[ramp_index]->set_slope( height / length );
                ramps[ramp_index]->set_length( length );
                ramps[ramp_index]->set_base( base );

                //We need to get the points in the correct order
                //Again, need a centroid.
                vector_t centroid (2);
                for( unsigned i=0; i<2; ++i )
                {
                    centroid[0] += low_points[i][0];
                    centroid[1] += low_points[i][1];
                    centroid[0] += high_points[i][0];
                    centroid[1] += high_points[i][1];
                }
                centroid[0] /= 4.0;
                centroid[1] /= 4.0;

                //Alright, we need to figure out which side is the right side and which is the left?
                vector_t direction(2);
                direction[0] = high_points[ sides[0][1] ][ 0 ] - low_points[ sides[0][0] ][ 0 ];
                direction[1] = high_points[ sides[0][1] ][ 1 ] - low_points[ sides[0][0] ][ 1 ];

                line_t center_line( centroid, centroid + direction );
                vector_t test_point(2);
                test_point[0] = high_points[ sides[0][1] ][0];
                test_point[1] = high_points[ sides[0][1] ][1];

                unsigned left_side;
                unsigned right_side;

                //If the point is on the right side of the center line
                if( center_line.side( test_point ) < 0 )
                {
                    //Then, side 0 is the right side and side 1 is the left side
                    right_side = 0;
                    left_side = 1;
                }
                //Otherwise, we know it is the left side
                else
                {
                    right_side = 1;
                    left_side = 0;
                }

                //Now that we know which side is which, set the points
                ramps[ramp_index]->set_points( low_points[ sides[left_side][0] ], low_points[ sides[right_side][0] ], high_points[ sides[right_side][1] ], high_points[ sides[left_side][1] ] );

                //Then, figure out how much to offset the effective area
                direction /= length;
                
                //Now, if this is an escalator, set the flow
                if( is_escalator )
                {
                    ramps[ramp_index]->going_up = goes_up;
                    ramps[ramp_index]->set_flow( direction, goes_up );                    
                }
                
                //TODO: can we get the radius of the cylinders?
                direction *= -0.35;

                ramps[ramp_index]->offset_effective_area( direction );
            }

            void VO_application_t::check_for_departures()
            {
                double time_stamp = frame_id * sim::simulation::simulation_step;
                bool update_state = false;
                for( unsigned i=0; i<behavior_controllers.size(); ++i )
                {
                    //If it is at the end of its path and it was going to its final destination
                    if( all_agents[i].is_active() && behavior_controllers[i]->is_agent_leaving() && path_follower_controllers[i]->at_goal() )
                    {
                        // Adding a new constraint - If the agent is in normal mode
                        if(path_follower_controllers[i]->is_agent_mode_normal())
                        {
                            // PRX_PRINT( "Node " << node_id << ") " << behavior_controllers[i]->get_pathname() << " leaves at frame : " << frame_id << " and has frame : " << behavior_controllers[i]->get_frames_to_leave(), PRX_TEXT_RED);
                            update_state = true;
                            //Move the plant back into la-la land
                            state_vec[1] = i;
                            plants[i]->get_state_space()->set_from_vector( state_vec );
                            //Deactivate the agent
                            all_agents[i].set_active( false );

                            simulation_file_stream << frame_id << "," << all_agents[i].agent_id << "," << time_stamp << "," <<  all_agents[i].agent_type << "," <<  plants[i]->print_state() << "," << vo_controllers[i]->get_current_velocity() << ","  << all_agents[i].get_queue_id() << "," << behavior_controllers[i]->get_lookup_x() <<"," << behavior_controllers[i]->get_lookup_y() << "," << all_agents[i].has_luggage << "," << all_agents[i].hindered   << "\n";
                            
                            // Removing the agent from the current nav node
                            const_cast<nav_node_t*>(path_follower_controllers[i]->get_current_triangle_node())->remove_agent(all_agents[i].agent_id);

                            //add the agent back to the pool
                            agent_pool.push_back( &(all_agents[i]) );
                            spawned_agents_per_node[node_id]--;
                        }
                    }
                }

                if( update_state )
                {
                    simulation_state_space->copy_to_point(simulation_state);
                }

                if(frame_id%1000==0)
                {
                    check_regions_for_depatures();
                }
            }
            
            bool VO_application_t::check_for_agent_spawn()
            {
                bool update_state = false;
                bool spawned=false;
                //For each of the agents
                

                while(agent_path_index < nr_paths && frame_id >= agent_paths[agent_path_index].arrival_frame)
                {
                    if(can_spawn())
                    {
                        spawned = spawn_agent(agent_paths[agent_path_index].entrance->get_spawn_point(frame_id));  
                        if(spawned)
                            update_state = true;                      
                    }
                    // else
                    //     spawn_in_other_node();
                    ++agent_path_index;
                } 
                
                if( update_state )
                {
                    simulation_state_space->copy_to_point(simulation_state);
                } 
                return spawned;   
            }

            bool VO_application_t::spawn_agent(const space_point_t* spawn_position)
            {
                if(spawn_position != NULL && !agent_pool.empty())
                {
                    ++num_of_agents_in_total;
                    //We will have to update the state the application uses for updating
                    //Pull an agent out of the pool
                    agent_data_t* agent = agent_pool.front();
                    agent_pool.pop_front();
                    //ROS_INFO("frame %d : NOde %d is going to spwan an agent (%d):  ", frame_id, node_id, agent_path_index);                   
                    //PRX_PRINT("Node " << node_id << ") Going to add agent (" << agent_path_index << "): " << agent->plant->get_pathname() << " (enter frame: " << agent_paths[agent_path_index].arrival_frame << ") at the origin: " << agent_paths[agent_path_index].entrance->name << "\nThe agent (" << agent_path_index << ") will exit from : " << agent_paths[agent_path_index].exit_origin->name << "  at frame: " << agent_paths[agent_path_index].departure_frame,PRX_TEXT_MAGENTA);
                    
                    // //Set the behavior controller to be active
                    // agent->behavior_controller->set_active(true, "");                            
                    
                    agent->agent_type = agent_paths[agent_path_index].agent_type;
                    agent->hindered = agent_paths[agent_path_index].has_luggage || agent_paths[agent_path_index].has_disability;
                    agent->has_luggage = agent_paths[agent_path_index].has_luggage;
                    
                    //Output the new agent type information in the path file
                    (*(file_streams[agent->agent_id])) << "# " << agent_types[ agent->agent_type ] << "\n";
                    
                    agent->VO_controller->update_max_velocity(agent_paths[agent_path_index].max_speed);
                    agent->behavior_controller->setup(agent_paths[agent_path_index], MSEC_2_FRAMES);
                    agent->path_controller->set_bias( uniform_random( 0.30, 0.85 ) );
                    // agent->path_controller->set_bias( PRX_MAXIMUM( PRX_MINIMUM( ( 0.5*gaussian_random() )+0.1, 1.0 ), 0 ) );
                    
                    agent->set_active(true);
                    agent->just_spawned = true;
                    
                    //DEBUG
                    //PRX_PRINT("Spawned: " << agent->plant->get_pathname() << "  at " << navigation_space->print_point( spawn_position, 3 ), PRX_TEXT_GREEN );
                    
                    // agent->behavior_controller->setup(desires, uniform_random(0,1), max_speed/2.0, agent_paths[agent_path_index].duration_frame, agent_paths[agent_path_index].exit_origin, MSEC_2_FRAMES);
                    // PRX_DEBUG_COLOR("The new agent will exit from : " << agent_paths[agent_path_index].exit_origin->name << "  at frame: " << agent_paths[agent_path_index].departure_frame, PRX_TEXT_MAGENTA);
                    //agent->behavior_controller->setup(desires, uniform_random(0,1), 10, uniform_random(120,400), agent_paths[agent_path_index].exit_origin);

                    //Set up the initial triangle for the new agent
                    const nav_node_t* near_node = dynamic_cast< const nav_node_t* >( metric->single_query( spawn_position ) );
                    agent->path_controller->update_node_information( near_node, NULL );

                    //Set the Plant's initial state to the spawn point
                    agent->plant->get_state_space()->copy_from_point( spawn_position );
                    // spawned_agents_per_node[node_id]++;
                    return true;
                }
                return false;
            }

            // bool VO_application_t::can_spawn()
            // {
            //     int spawned_agents = spawned_agents_per_node[node_id];

            //     for(unsigned i = 0; i < num_of_nodes; ++i)
            //     {
            //         PRX_PRINT("Node " << node_id << ") has agents: " << spawned_agents << "  the node " << i << " has agents: " << spawned_agents_per_node[i], PRX_TEXT_GREEN);
            //         if(i != node_id && spawned_agents_per_node[i] < spawned_agents)
            //         {
            //             spawned_agents_per_node[i]++;
            //             PRX_PRINT("FALSE 1", PRX_TEXT_RED);
            //             return false;
            //         }

            //         if(spawned_agents_per_node[i] == spawned_agents && i < node_id)
            //         {
            //             spawned_agents_per_node[i]++;
            //             PRX_PRINT("FALSE 2", PRX_TEXT_RED);
            //             return false;
            //         }
            //     }
            //     PRX_PRINT("Node " << node_id << ") TRUE ",PRX_TEXT_GREEN);
            //     return true;
            // }

            // void VO_application_t::spawn_in_other_node()
            // {
            //     int val = spawned_agents_per_node[0];
            //     int index = 0;
            //     for(unsigned i = 1; i < num_of_nodes; ++i)
            //     {
            //         if(spawned_agents_per_node[i] < val)
            //         {
            //             index = i;
            //             val = spawned_agents_per_node[i];
            //         }
            //     }
            //     spawned_agents_per_node[index]++;
            // }

            bool VO_application_t::can_spawn()
            {
                int val = PRX_INFINITY;
                int index = -1;
                for(unsigned i = 0; i < num_of_nodes; ++i)
                {
                    if(spawned_agents_per_node[i] < val && spawned_agents_per_node[i] < all_agents_size)
                    {
                        index = i;
                        val = spawned_agents_per_node[i];
                    }
                }
                if(index != -1)
                    spawned_agents_per_node[index]++;

                // std::string believe = "Node " + int_to_str(node_id) + " frame: " + int_to_str(frame_id) + ") ";
                // for(unsigned i = 0; i < num_of_nodes; ++i)
                //     believe += int_to_str(spawned_agents_per_node[i]) + " , ";
                // PRX_PRINT(believe,PRX_TEXT_LIGHTGRAY);

                return index == node_id;
            }

            void VO_application_t::update_sensing_model(const prx_simulation::pabt_node_msg& msg)
            {
                comm_time_stream << ros::Time::now() << ",";
                nav_sensor->update_node_info(node_id, msg);
                spawned_agents_per_node[msg.node_id] = msg.agents.size();
                //PRX_PRINT(goal_region->name<<"CALLING OPEN SLOT with mode "<<agent_mode<<" agent index "<<node_info->agent_index<<" path_vertices_to_queue_counter "<<path_vertices_to_queue_counter<<" path_vertices_to_queue.size() - "<<path_vertices_to_queue.size()<<" triangle_node - "<<output_control_space->print_point(triangle_node->point, 5)<<" way point node - "<<output_control_space->print_point(waypoint_node->point, 5)<<" current_state "<<output_control_space->print_point(current_state, 5) , PRX_TEXT_BLUE);ROS_INFO("frame: %d -> update sensing node %d from node %d size of agents %d", frame_id,node_id, msg.node_id, msg.agents.size());
                comm_time_stream << ros::Time::now() << ",";
                msg_index ++;
                if (msg_index == (num_of_nodes -1))
                {
                    comm_time_stream << "\n";
                }
            }

            // void VO_application_t::update_sensing_model(const prx_simulation::pabt_node_msg& msg)
            // {
            //     // ros::Duration diff = ros::Time::now() - msg.stamp;
            //     PRX_PRINT("Node " << node_id << "  frame_id: " << frame_id,  PRX_TEXT_CYAN);
            //     PRX_PRINT(" Read the message from node: " << msg->node_id,  PRX_TEXT_CYAN);
                
            //     double t = frame_clock.measure();
            //     update_node_id++;
            //     nav_sensor->update_node_info(node_id, msg);
            //     spawned_agents_per_node[msg.node_id] = msg.agents.size();
            //     num_of_responses++;
            //     frames_from_nodes[msg.node_id] = msg.frame_id;
            //     update_node_time += (frame_clock.measure() - t);                
            //     //waiting_time_stream << num_spins << " , " << msg.node_id << " , " << t << " , " << (frame_clock.measure() - t) << " , " << msg.frame_id << " , ";
                
            //     // PRX_PRINT("Node " << node_id << ")  frame: " << frame_id << " responses: " << num_of_responses << " is going to update data from node: " << msg.node_id, PRX_TEXT_BROWN);
            // }
        }
    }
}
