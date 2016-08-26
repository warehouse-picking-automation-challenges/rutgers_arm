/*
 * file queue.cpp
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

#include "simulation/structures/queue.hpp"
#include "simulation/structures/queue_manager.hpp"

namespace prx
{
    using namespace util;

    namespace packages
    {
        namespace crowd
        {
           
           int queue_t::global_neighbor_search_id = 0;

            queue_t::queue_t()
            {
            	//global_queue_info_search_id = 0;
				floor_level=1;
			//	queue_refersh=false;
				fluctuation_angle=-1;
			//	if_occupy=false; //*** change
				if_trapped=false;
				reach_first_turn=false;//*** change
				available_slot_index=-1;
				following_seg_hash_key=-1;
				following_a_wall=false;
				//queue_manager = NULL;
				spacing = 0.75;
				cur_queue_state.resize(3);
				queue_id = -1;
            }

			queue_t::queue_t(int queue_id)
            {
            	//global_queue_info_search_id = 0;
				floor_level=1;
			//	queue_refersh=false;
				fluctuation_angle=-1;
			//	if_occupy=false; //*** change
				if_trapped=false;
				reach_first_turn=false;//*** change
				available_slot_index=-1;
				following_seg_hash_key=-1;
				following_a_wall=false;
				//queue_manager = NULL;
				cur_queue_state.resize(3);
				spacing = 0.75;
				this->queue_id = queue_id;
            }

            queue_t::~queue_t()
            {
            	//global_queue_info_search_id = 0;
            }

			void queue_t::init( points_t ini_pt, string& name, double  angle, double fluctuation_angle, double floor_level, double ft_dist,double ft_angle, double spacing) //in interface
			{
				// Points to the start of the queue ( Since this would be intially empty)
				this->available_slot_index = 0;
				this->fluctuation_angle = fluctuation_angle;
				this->queue_name = name;
				this->angle = angle/180*PI;
				this->floor_level = floor_level;
				this->spacing = spacing;
				points_t tmp;
				vector<double> tmp_v;
				tmp_v.resize(3);
				tmp_v=pol2cart(angle/180*PI,500);
				tmp.x=tmp_v[0]+ini_pt.x;
				tmp.y=tmp_v[1]+ini_pt.y;
				tmp.z=ini_pt.z;
				q_start_point=ini_pt;
				direction=tmp;

				_numnpt dl;
				dl.pt = direction;
				dl.vals = 1;
				direction_list.push_back(dl.pt);

				_numnpt bp;
				bp.vals=0;
				bp.pt=q_start_point;
				add_breakpts(bp);
				queue_segment_structures.clear();
				//**** -- DEPRECATED -- ****//
				// queue_segments.clear();
				// queue_segment_buffers_left.clear();
				// queue_segment_buffers_right.clear();
				if (ft_dist < 1e-6)
				{
					reach_first_turn = true;
					first_turn = direction;
					first_turn_direction = direction;
					first_turn_dist = -1;
					first_turn_angle = ft_angle;
				}
				else
				{
					first_turn = direction;
					first_turn_direction = direction;
					first_turn_dist = ft_dist;
					first_turn_angle = ft_angle;
				}		
			     
				return;
			}
			
			// TODO : Need to check this function	
	  //       void queue_t::pop_first_agent() 
			// {
			// 	behavior_controller_t* agent;
			// 	agent = agents_in_queue.front();

			// 	agents_in_queue.pop_front();
			// 	agent->leave_queue();
				
			// 	PRX_PRINT("agent:"<<agent->get_id()<< " leaving queue:"<<queue_name<<" with frames_to_leave:"<<agent->get_frames_to_leave(),PRX_TEXT_LIGHTGRAY);
			// 	for(unsigned i = 0; i < available_slot_index-1; ++i)
			// 	{
			// 		point2vector(cur_queue_state, queue_point_slots[i].pt);
			// 		agents_in_queue[i]->move_in_queue(i, cur_queue_state);
			// 		PRX_PRINT("Moving "<<i<<"th agent:"<<agents_in_queue[i]->get_id()<< " to:"<<cur_queue_state[0]<<","<<cur_queue_state[1]<<","<<cur_queue_state[2]<<" in queue:"<<queue_name<<" with frames_to_leave:"<<agents_in_queue[i]->get_frames_to_leave(),PRX_TEXT_BROWN );
			// 	}
			// 	if(available_slot_index==0)
			// 	{
			// 		PRX_PRINT("WHAT THE HELL ARE YOU DOING DUMBO?"<<" HOW CAN available_slot_index be negative!!",PRX_TEXT_RED);
			// 	}
			// 	--available_slot_index;

			//     //return agent;
			// }

			void queue_t::pop_first_agent() 
			{
				behavior_controller_t* agent;
				agent = agents_in_queue.front();

				agents_in_queue.pop_front();
				agent->leave_queue();
				//PRX_PRINT("agent:"<<agent->get_id()<< " leaving queue:"<<queue_name<<" with frames_to_leave:"<<agent->get_frames_to_leave(),PRX_TEXT_LIGHTGRAY);
				for(unsigned i = 0; i < available_slot_index-1; ++i)
				{
					point2vector(cur_queue_state, queue_point_slots[i].pt);
					if(i==0)
					{
						agents_in_queue[i]->set_region_lookup();
					}
					else
					{
						agents_in_queue[i]->set_lookup(queue_point_slots[i-1].pt.x,queue_point_slots[i-1].pt.y);	
					}
					
					agents_in_queue[i]->move_in_queue(i, cur_queue_state);
					//PRX_PRINT("Moving "<<i<<"th agent:"<<agents_in_queue[i]->get_id()<< " to:"<<cur_queue_state[0]<<","<<cur_queue_state[1]<<","<<cur_queue_state[2]<<" in queue:"<<queue_name<<" with frames_to_leave:"<<agents_in_queue[i]->get_frames_to_leave(),PRX_TEXT_BROWN );
				}
				if(available_slot_index==0)
				{
					PRX_PRINT("WHAT THE HELL ARE YOU DOING DUMBO?"<<" HOW CAN available_slot_index be negative!!",PRX_TEXT_RED);
				}
				--available_slot_index;

			    //return agent;
			}

			// Region forces the agents in the queue to leave - Since it is time for a departure
			void queue_t::depart_agents(int amount)
			{
				if(amount > available_slot_index)
				{
					amount = available_slot_index;
				}
				//PRX_PRINT("DEPARTING "<<amount<<" agents from queue"<<queue_name,PRX_TEXT_BROWN);
				for(int i=0;i<amount;++i)
				{
					agents_in_queue[i]->set_time_for_departure();
				}
			}

            void queue_t::update_direction(points_t refpt,points_t new_direc,double times)
            {

                direction_list.push_back(direction);
				direction=extend_direction(refpt,new_direc,times);
                return;
            }

            

            points_t queue_t::get_last_position_pt()  //get last occupied slot (last agent in queue)
            {
                points_t rt;
                rt.x=-9999;rt.y=-9999;rt.z=-9999; //set a maximum impossible point that can indicates this is just an initial data
                //if(agents_loc.empty())
                if(available_slot_index<1)
                {
                    return q_start_point;
                }
				else if (available_slot_index>queue_point_slots.size()-1)
                {
                	return queue_point_slots.back().pt;
                }

				rt=queue_point_slots[available_slot_index-1].pt;
                //rt=agents_loc.back().pt;
                return rt;
            }

            vector<double> queue_t::get_last_position_vector()
            {
                point2vector(cur_queue_state, q_start_point);
                //if(agents_loc.empty())
                if(available_slot_index<1)
                {
                    return cur_queue_state;
                }
                
                else if (available_slot_index>queue_point_slots.size()-1)
                {
                	point2vector(cur_queue_state,queue_point_slots.back().pt);
                	return cur_queue_state;
                }

                point2vector(cur_queue_state,queue_point_slots[available_slot_index-1].pt);
                
                return cur_queue_state;
            }

			points_t queue_t::get_last_slot_pt() //last slot, no matter it is available or not
			{
				points_t rt;
				rt.x=-9999;rt.y=-9999;rt.z=-9999;
				if(queue_point_slots.empty())
				{
					return q_start_point;
				}
				rt=queue_point_slots.back().pt;
				return rt;
			}

            points_t queue_t::get_direction()
            {
                return direction;
            }

            double queue_t::queue_size()
            {
                return available_slot_index+1;
            }            

			_queue_info queue_t::get_queue_info()
			{
				_queue_info queue_info;
				queue_info.queue_id = queue_id;
				queue_info.queue_name = queue_name;
				queue_info.floor_level = floor_level;

				if (available_slot_index <= 0)
				{
					queue_info.last_position = q_start_point;
				}
				else
				{
					queue_info.last_position=queue_point_slots[available_slot_index].pt;
				}

				if (queue_point_slots.size() == 0)
				{
					queue_info.last_slot = q_start_point;
				}
				else
				{
					queue_info.last_slot = queue_point_slots.back().pt;
				}

				queue_info.latest_direction = direction;
				queue_info.start_point = q_start_point;
				queue_info.queue_length = 0;
				for (int i = 0;i < ((int)breakpts.size() - 1);++i)
				{
					queue_info.queue_length = queue_info.queue_length + sqrt((breakpts[i + 1].pt.x - breakpts[i].pt.x)*(breakpts[i + 1].pt.x - breakpts[i].pt.x) + (breakpts[i + 1].pt.y - breakpts[i].pt.y)*(breakpts[i + 1].pt.y - breakpts[i].pt.y));
				}

				points_t lastpt;
				lastpt = get_last_slot_pt();

				if(breakpts.size()!=0)
				{
					queue_info.queue_length = queue_info.queue_length + sqrt((breakpts.back().pt.x - lastpt.x)*(breakpts.back().pt.x - lastpt.x) + (breakpts.back().pt.y - lastpt.y)*(breakpts.back().pt.y - lastpt.y));
				}
				
				return queue_info;
			}

			// Add navigation node corresponding to the traingle.
			void queue_t::add_triangle_slot(const nav_node_t* nav_node)
			{
				triangle_slots.push_back(nav_node);
			}

			deque<const nav_node_t*>& queue_t::get_triangle_slots()
			{
				return triangle_slots;
			}

			void queue_t::add_queue_point_slot(_numnpt destination_point)
			{
				queue_point_slots.push_back(destination_point);
			}

			void queue_t::add_breakpts(_numnpt destination_point)
			{
				breakpts.push_back(destination_point);
			}

			unsigned queue_t::get_available_slot_index()
			{
				return available_slot_index;
			}
			
			void queue_t::reset_available_slot_index()
			{
				available_slot_index=0;
			}

			const nav_node_t* queue_t::get_open_slot(behavior_controller_t* agent)
			{   
				cur_nav_node = triangle_slots[available_slot_index];
				return cur_nav_node;
     	    }

     	    // Move agents in queue according to their frames_to_leave
     	    unsigned queue_t::put_agent_in_queue(behavior_controller_t* agent, int frames_to_leave)
     	    {

     	    	int latest_frames_to_leave = PRX_INFINITY; // Agent with frames higher than the current agent
     	    	unsigned slot_to_go = available_slot_index;
				do
     	    	{
     	    		--slot_to_go;
     	    		latest_frames_to_leave = agents_in_queue[slot_to_go]->get_frames_to_leave();
     	    	}while(frames_to_leave < latest_frames_to_leave && slot_to_go > 0);

     	    	if(slot_to_go+1 == available_slot_index && frames_to_leave >= latest_frames_to_leave)
     	    	{
     	    		slot_to_go = available_slot_index;
     	    		agents_in_queue.push_back(agent);
     	    	}
     	    	else
     	    	{
     	    	    agents_in_queue.push_back(agents_in_queue[available_slot_index-1]);
     	    	    point2vector(cur_queue_state, queue_point_slots[available_slot_index].pt);
					agents_in_queue[available_slot_index-1]->move_in_queue(available_slot_index, cur_queue_state);
			   		for(unsigned i=available_slot_index-1;i>slot_to_go;--i)
	     	    	{
	     	    	    point2vector(cur_queue_state, queue_point_slots[i].pt);
						agents_in_queue[i-1]->move_in_queue(i, cur_queue_state);
	     	    		agents_in_queue[i] = agents_in_queue[i-1];
	     	    	}
	     	    	agents_in_queue[slot_to_go] = agent;
     	    	}
     	    	return slot_to_go;
     	    }

			const nav_node_t* queue_t::reserve_slot(behavior_controller_t* agent, std::vector<double>& queue_points, int frames_to_leave, double &orientation)
			{            
				unsigned slot_to_go = available_slot_index;

				if(available_slot_index!=0 && frames_to_leave!= PRX_INFINITY)
				{
					slot_to_go = put_agent_in_queue(agent,frames_to_leave);
				}
				else
				{
					agents_in_queue.push_back(agent);
					
				}

				// Set the orientation of the agent
				if(slot_to_go==0)
				{
					orientation = this->angle;
					agent->set_region_lookup();
				}
				else
				{
					orientation = calculate_orientation(queue_point_slots[slot_to_go-1].pt,queue_point_slots[slot_to_go].pt);
					agent->set_lookup(queue_point_slots[slot_to_go-1].pt.x, queue_point_slots[slot_to_go-1].pt.y);
				}
				agent->in_queue(this, slot_to_go);

            	cur_nav_node = triangle_slots[slot_to_go];
            	point2vector(queue_points, queue_point_slots[slot_to_go].pt);
				
				++available_slot_index;
				
            	queue_manager->create_new_slots(this); // Create a new open slot if there aren't enough
            	return cur_nav_node;
     	    }

			// Fill the segment structure information for the queue
     	    void queue_t::fill_segment_structure_info(points_t start_point, points_t end_point, segment_struct_t*& segment_structure)
     	    {
     	    	if(queue_segment_structures.size() == breakpts.size()) // Update the segment
     	    	{
     	    		segment_structure = queue_segment_structures.back();
     	    		segment_structure->update(start_point, end_point);
     	    	}
     	    	else //Create the segmnent
     	    	{
     	    		segment_structure = new segment_struct_t(queue_id, breakpts.size(), start_point, end_point);
     	    		queue_segment_structures.push_back(segment_structure);
     	    	}
     	    }

			// Adds Start Triangle details to the queue
			void queue_t::add_start_triangle(points_t start_point,nav_node_t* nav_node)
			{
				segment_struct_t* segment_struct;
				fill_segment_structure_info(start_point,start_point,segment_struct);

				nav_node->insert_queue_segment_structure(segment_struct);
				add_triangle_slot(nav_node);
			}

     	    // Adds both queue point as well as nav nodes
		    void queue_t::add_new_slot(_numnpt destination_point)
			{
				
				point2vector(cur_queue_state, destination_point.pt);
				
				points_t prev_break_pt = breakpts.back().pt;
				
				segment_struct_t* segment_structure;
				fill_segment_structure_info(prev_break_pt, destination_point.pt, segment_structure);
				
				prev_nav_node = triangle_slots.back();
				
				nav_node_t* current_nav_node = find_triangle_from_previous_slot(prev_nav_node,cur_queue_state);
				
				// filling info regarding segments for nav node here
				if(current_nav_node!=NULL)
				{
					if(prev_nav_node != current_nav_node) // insert segment into the current_nav_node
					{
						current_nav_node->insert_queue_segment_structure(segment_structure);
					}
					add_queue_point_slot(destination_point);
					add_triangle_slot(current_nav_node);
				}
				else
				{
					PRX_PRINT("Cannot find a triangle for the slot generated by the Queue Mananger for queue:"+queue_name,PRX_TEXT_RED);
				}
			}

			nav_node_t* queue_t::find_triangle_from_previous_slot(const nav_node_t* prev_nav_node, const std::vector< double >& current_state )
            {
            	++global_neighbor_search_id;
            	
            	nav_node_t* node = const_cast<nav_node_t*>(prev_nav_node);
                node->neighbor_search_id = global_neighbor_search_id;
                
                //Also need a list that is a frontier of nodes (open list)
                std::deque< nav_node_t* > frontier;
                frontier.push_back(node);
                
                // checked.push_back(node);
                while(!frontier.empty())
                {
                	
                    node = frontier.front();
                    if( node->point_in_triangle( current_state ) )
                    {
                    	
                    	//cout<<frontier.size()<<endl;
                        return node;
                    }
                    
                    foreach(nav_node_t* nd, node->neighboring_triangles)
                    {	
                    	
                    	if( fabs( nd->point->memory[2] - current_state[2] ) < 2.0 )
                        {
	                        if(nd->neighbor_search_id != global_neighbor_search_id)
	                        {
	                        	
	                            nd->neighbor_search_id = global_neighbor_search_id;
	                            frontier.push_back(nd);
	                            
	                        }
	                    }
                    }
                    
                    frontier.pop_front();
                }
                
                return NULL;
            }

            void queue_t::nearby_queue_info(std::deque<segment_struct_t*>& arg_wall_segment_structures, util::hash_t<int, std::vector<segment_struct_t*> >& arg_queue_segment_structures, int search_depth)
            {
            	nearby_queue_info_at_node(const_cast<nav_node_t*>(triangle_slots.back()),arg_wall_segment_structures,arg_queue_segment_structures, search_depth);
            }

            void queue_t::nearby_queue_info_at_node(nav_node_t* nav_node, std::deque<segment_struct_t*>& arg_wall_segment_structures,util::hash_t<int, std::vector<segment_struct_t*> >& arg_queue_segment_structures, int search_depth)
            {
            	++global_neighbor_search_id;
            	//global_queue_info_search_id++;

                nav_node->neighbor_search_id = global_neighbor_search_id;
                
                std::deque< std::pair< nav_node_t*, unsigned > > frontier;
                frontier.push_back(std::make_pair(nav_node,0));
                while(!frontier.empty())
                {
                    nav_node = frontier.front().first;
                    unsigned depth = frontier.front().second;

                    nav_node->fill_queue_info(arg_wall_segment_structures, arg_queue_segment_structures, global_neighbor_search_id);
                    if(depth < search_depth)
                    {
                        foreach(nav_node_t* nd, nav_node->neighboring_triangles)
                        {
                            if(nd->neighbor_search_id != global_neighbor_search_id)
                            {
                                nd->neighbor_search_id = global_neighbor_search_id;
                                frontier.push_back(std::make_pair(nd,depth+1));
                            }
                        }
                    }
                    frontier.pop_front();
                }
            }

            bool queue_t::is_first_agent(std::vector<double>& current_position, behavior_controller_t* agent)
            {
            	if(available_slot_index==0 || agent != agents_in_queue[0])
            	{
            		return false;
            	}

            	point2vector(cur_queue_state, queue_point_slots[0].pt);

            	// Check height
            	if( fabs(cur_queue_state[2] - current_position[2]) > Z_DISTANCE_CHECK)
            		return false;

          		// Check distance(norm-1) in xy plane
            	if((fabs(cur_queue_state[0] - current_position[0]) + fabs(cur_queue_state[1] - current_position[1])) < XY_DISTANCE_CHECK)
            		return true;
            	
            	return false;		
            }
        }         
    }
}
