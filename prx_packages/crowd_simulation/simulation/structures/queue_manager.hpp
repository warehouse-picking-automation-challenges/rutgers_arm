/**
 * @file queue_manager.hpp
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

#ifndef PRX_AGENT_QUEUE_MANAGER_HPP
#define PRX_AGENT_QUEUE_MANAGER_HPP

#include "simulation/structures/queue.hpp"

#include "prx/utilities/definitions/defs.hpp"

namespace prx
{
    namespace packages
    {
        namespace crowd
        {
            
			class queue_managers_t 
			//In this class, we only need to change sunit and scale. sunit is the distance between two agents in queue(oset),scale is ratio of coordinate and real distance,default is 1:1
			//In operation, we only need one function:require_queue_destination(rq_agent,rq_queue,require_time)
			//rq_agent is the pointer of the agent who wants to join the queue
			//rq_queue is the pointer of the queue that the agent wants to join
			//require_time can be set as 1 or 2. when it equals 1, the agent will not occupy the returned destination point, when it equals 2, the agent will occupy it no matter if he arrives that point
			////IMPORTANT: I dont write code for movement of the agent when he requires a destination.
			{
			public:
				int queueing_type; //1-Lshape_angle//
				//now, only queueing_type=1 works. more queueing_type will be added. it controlls how to form the queue's shape
				double dlineangle; //agents randomly stand with this maximum angle  to the straight queue line.
				//queue_manager will read the "fluctuation angle" in queue object for dlineangle. if it reads -1, set 20 as a default value
				double sunit; //agents' acceptable radius to others and walls. (oset) - i.e distance between agent and the walls(buffers).
				string qm_name;
				int qm_id;
				double scale;// scale*sunit = distance between two agents

				queue_managers_t();
				//void set_queue_manager(int argc,double vals);
				//change parameters of the queue_manager.1-queueing_type, 2-angle,3-sunit(offset distance) 4-scale

				void update_buffer(void);//update buffer lines of nearby walls and queues. qeue) val2:conflict_sequence_in_container pt:conflict point
				//void update_buffer_offline(void);
				_2numnpt check_conflict(segment_t* seg); //val1:conflict flag(0-none 1-wall,2-queue) val2:conflict_sequence_in_container pt:conflict point
				//void calculate_buffer(segment_t& seg,segment_t& seg1,segment_t& seg2);



				//require time 1: get the next point to go. 2. reserve the position for you and the agents will get the next available point. 
				// To generate slots
				int generate_slots(queue_t* rq_queue,int slot_number); // For now we are not using this variable(slot_number)

				// To create new(open) slots if they are not enough
				void create_new_slots(queue_t* rq_queue);
				
				// Adds the given queue to the list of queues
				void insert_queue(queue_t* queue);

				// Get a pointer to the list of queues
				deque<queue_t*>& get_queues(); 

				//update the start point of a queue in order to let it locate out of buffer area
				bool update_start_point(points_t& ini_pt, const nav_node_t* first_nav_node,queue_t* rq_queue);

			private:
				// deque<segment_t*> bufferwalls; //the buffer lines of the nearby walls of the target queue
				// deque<segment_t*> bufferqueues;// the buffer lines of all queues
				// deque<pair<int,int> > bufferwallid;
				// deque<pair<int,int> > bufferqueueid;
				// deque<segment_t*> wall_container_temp; //the nearby walls of the target queue
				// deque<segment_t*> queue_segs_obtained; //queue segments obtained from triangle area

				deque<segment_t*> wall_conflict_container; //the conflict walls 
				deque<segment_t*> queue_conflict_container; //the conflict queues
				deque<pair<int,int> > wall_conflict_id; //pair<wall seq(i),0 or1>
				deque<pair<int,int> > queue_conflict_id; //queue seq(i),break pt seq(j)


				deque<segment_struct_t*> wall_segment_structures_container;
	            util::hash_t<int, std::vector<segment_struct_t*> > queue_segment_structures_container;
				//util::hash_t<int, std::vector<segment_struct_t*> >::iterator queue_seg_it;

				void get_environment_from_triangle_structure(int index,queue_t* rq_queue,int dist);
                void get_environment_from_triangle(int index,queue_t* rq_queue,int dist);
                void get_environment_for_update_start_point(const nav_node_t* navnode,queue_t* rq_queue,int dist);

				points_t get_aligned(points_t refpt, points_t reftopt, segment_t* seg, bool acute_angle);
				deque<queue_t*> queues_list;
				queue_t* current_operate_queue;

				nav_node_t* environment_tri;
			};




			extern deque<queue_managers_t*> queue_manager_container; //the pointer of any created queue_manager will be added into this container automatically

			//Name Angle Fluctuation Floor x y

			struct _q_ini_info{char name[200],floor[200],des[200];double angle,fluc,x,y,first_dist,first_angle;};
			extern deque<_q_ini_info> q_ini_list;
			void read_initial_queue(char* fn); //fn is the full directory of input csv file


        }
    }
}

#endif //PRX_AGENT_QUEUE_MANAGER_HPP

