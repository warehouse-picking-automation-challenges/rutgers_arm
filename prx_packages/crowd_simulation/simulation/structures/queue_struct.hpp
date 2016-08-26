#pragma once

#ifndef PRX_AGENT_QUEUE_STRUCT_HPP
#define PRX_AGENT_QUEUE_STRUCT_HPP

#include <vector>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <map>
#include <stdlib.h> 
#include <time.h>  
#include <queue>
#include <algorithm>
#include <string>
#include <cmath>
#include "prx/utilities/boost/hash.hpp"


//#include "simulation/structures/agent_data.hpp"

#include "prx/utilities/definitions/defs.hpp"
using namespace std;
#define PI 3.1415926

namespace prx
{
    namespace packages
    {
        namespace crowd
        {

			class points_t
			{
			  public:
			  	points_t();
			  	points_t(double a,double b,double c);
				double x;
				double y;
				double z;
				int object_id;
				void set(double a,double b,double c);
				void set(const std::vector<double>& pt);
				bool equal(points_t p);
			};

			class segment_t
			{
			  public:
			  	segment_t();
			  	segment_t(int object_id, points_t start_point, points_t end_point);
			  	int object_id; // We will not use this from now on. Remove this Remove this
			  	points_t startpt;int seg_key;
				points_t endpt;				
				double a();
				double b();
				double c();
				double k();
				double p();
				vector<double> nor(points_t nor_pt);
				double paraa,parab,parac,parak,parap;
				void cal_para();
				void set(points_t p1,points_t p2);
				int get_hash_key();
				double get_length();
				bool operator== (const segment_t& seg) const;
			};



			class segment_struct_t
			{
			public:
				int search_id;
				int object_id; // queue_id or wall_id
				int segment_id; // breakpt.size() or wall_id
				segment_t* actual_segment;
				segment_t* buffer_segment_left;
				segment_t* buffer_segment_right;
				segment_struct_t();
				segment_struct_t(int object_id, int segment_id, points_t start_point, points_t end_point);
				void update(points_t start_point, points_t end_point);
				int get_hash_key();
			};


	
			struct _2numnpt{double val1;double val2;points_t pt;};
			struct _numnpt{double vals;points_t pt;};


			_numnpt check_seg_intersec(segment_t* s1,segment_t* s2);  ////return val:0-not inter 1-inter 2-overlap.check the intersection point of two line(not segment.thus the intersection point may not between the two end points)
    		double pt_pt_dist(points_t pt1,points_t pt2);//distance between two points

			vector<double> cart2pol(double x,double y,double z); //convert coordinate to polar coordinate,return:theta,rho,z theta range:0-2pi
			vector<double> pol2cart(double theta,double r); //convert polar coordinate to x,y,z
			vector<double> regressline(vector<points_t> pts); //get the regress line of a set of points
			double pt_seg_dist(points_t pt,segment_t* seg);// the perpendicular distance from a point to a line(not segment)
			points_t pt_seg_intersec_pt(points_t pt,segment_t* seg);// the perpendicular intersection point of a point and a line
			int judge_bt_points(points_t jbpx, points_t jbp1, points_t jbp2); //judge whether a point is in a range of other two points.0-not 1-in rectangle area but not on diagonal seg 2-in area and on seg

			_numnpt offsetptrand(points_t refpt,points_t reftopt,double oset,double dlineangle); //vals-rho pt-randomized pt
			points_t offsetpt(points_t refpt,points_t reftopt,double oset);
			//to look for the next point based on an existing point(refpt),a direction point(reftopt), distance(oset), and a swing angle(dlineangle) 
			//oset and dlineangle can be changed in queue_manager instance.

			points_t extend_direction(points_t refpt,points_t reftopt,double times);
			//based on an existing point(refpt), given a direction point (reftopt), get the new end point of an extended line(from refpt to reftopt by xxx times)

			void vector2point(points_t& pt, vector<double>& vec); //convert an vector<double>[x,y,z] to a point class object
			void point2vector(vector<double>& vec, points_t& pt);


			//new:
			void update_buffer_wall(deque<segment_t> original_wall_segs,boost::unordered_map<int,segment_t> buffer_walls);
			void update_buffer_queue(deque<segment_t> original_queue_segs,deque<int> queue_seg_id,boost::unordered_map<int,segment_t> buffer_queues);


			/**
			 * @brief [brief description]
			 * @details [long description]
			 * 
			 * @param seg [description]
			 * @param seg1 [description]
			 * @param seg2 [description]
			 * @param sunit This is the distance between buffer lines and segment. 
			 */
			void calculate_buffer(segment_t* seg,segment_t* seg1,segment_t* seg2, double sunit = 1);
			double calculate_orientation(const points_t &prev, const points_t &cur);

		}

	}

}



#endif