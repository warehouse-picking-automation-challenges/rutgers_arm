/**
 * @file region.cpp
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

#include "simulation/structures/region.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "simulation/structures/queue.hpp"
#include "simulation/controllers/behavior_controller.hpp"

/*
Itruns - thanks

*/

namespace prx
{
    using namespace util;

    namespace packages
    {
        namespace crowd
        {
            void operator >> (const YAML::Node& node, std::vector<double>& vec) 
            {
                vec.resize( node.size() );
                
                for( unsigned i=0; i<vec.size(); ++i )
                {
                    vec[i] = node[i].as< double >();
                }
            }

            bool operator<(const departure_t &departure_1, const departure_t &departure_2)
            {
                return departure_1.frames < departure_2.frames;
            }

            region_t::region_t( util::space_t* input_space, unsigned in_type, unsigned id)
            {
                nav_space = input_space;
                region_id = id;
                type = in_type;
                queue = NULL;

                influence = 1;
                occupancy = 0;
                max_capacity = 0;
                duration_distribution = std::make_pair(0,0); 
                evacuation_point = false;
            }

            region_t::~region_t()
            {
                clear();
            }

            void region_t::init(const util::parameter_reader_t * const reader, double MSEC_2_FRAMES)
            {
                std::vector<double> next_point(3);

                name = reader->get_attribute("Name");
                if(!reader->has_attribute("Doorways"))
                    PRX_FATAL_S("Region " << name << " does not have doors. Please fix the model!");
                std::vector< double > in_doorways = reader->get_attribute_as< std::vector< double > >("Doorways");
                PRX_ASSERT(in_doorways.size()%3 == 0);                
                doorways.clear();

                nr_doorways = in_doorways.size()/3;
                doorways.resize(nr_doorways);
                for(unsigned i=0; i<nr_doorways; ++i)
                {
                    next_point[0] = in_doorways[3*i];
                    next_point[1] = in_doorways[3*i+1];
                    next_point[2] = in_doorways[3*i+2] + 1.0;
                    doorways[i] = nav_space->alloc_point();
                    nav_space->set_from_vector(next_point, doorways[i]);
                }
                --nr_doorways;

                std::vector< double > triangles = reader->get_attribute_as< std::vector< double > >("DefiningTriangles");
                PRX_ASSERT(triangles.size()%3 == 0);
                points.clear();

                nr_inside_points = triangles.size()/3;
                points.resize(nr_inside_points);
                for(unsigned i=0; i<nr_inside_points; ++i)
                {
                    next_point[0] = triangles[3*i];
                    next_point[1] = triangles[3*i+1];
                    next_point[2] = triangles[3*i+2] + 1.0;
                    points[i] = nav_space->alloc_point();
                    nav_space->set_from_vector(next_point, points[i]);
                }
                --nr_inside_points;

                evacuation_point = reader->get_attribute_as<bool>("isEvacutionPoint", false);

                nr_departures = 0;
                if(reader->has_attribute("DepartureInformation"))
                {
                    unsigned frames;
                    double capacity;
                    PRX_PRINT("DepartureInformation FOR REGION:"<<name,PRX_TEXT_CYAN);
                    departures.push_back(departure_t(6000,5));
                    ++nr_departures;
                    departures.push_back(departure_t(9000,5));
                    ++nr_departures;
                    foreach( const util::parameter_reader_t* r, reader->get_list("DepartureInformation") )
                    {
                        // PRX_PRINT("Departure"<<nr_departures<<" frames: " << r->get_attribute_as< unsigned >("Time")*MSEC_2_FRAMES << " Amount:" << r->get_attribute_as< double >("Capacity"), PRX_TEXT_GREEN );
                        frames = r->get_attribute_as< unsigned >("Time") * MSEC_2_FRAMES;
                        capacity = r->get_attribute_as< double >("Capacity");
                        departures.push_back(departure_t(frames,capacity));
                        ++nr_departures;
                    }
                    std::sort( departures.begin(), departures.end() );
                }

            }

            void region_t::init_from_file( const YAML::Node& node, double MSEC_2_FRAMES)
            {
                std::vector<double> next_point(3);
                occupancy = 0;
                name = node["Name"].as<std::string>();
                if(node["Doorways"])
                {
                    std::vector< double > in_doorways;
                    node["Doorways"] >> in_doorways;
                    PRX_ASSERT(in_doorways.size()%3 == 0);                
                    doorways.clear();

                    nr_doorways = in_doorways.size()/3;
                    doorways.resize(nr_doorways);
                    for(unsigned i=0; i<nr_doorways; ++i)
                    {
                        next_point[0] = in_doorways[3*i];
                        next_point[1] = in_doorways[3*i+1];
                        next_point[2] = in_doorways[3*i+2] + 1.0;
                        doorways[i] = nav_space->alloc_point();
                        nav_space->set_from_vector(next_point, doorways[i]);
                    }
                    --nr_doorways;

                    std::vector< double > triangles;
                    node["DefiningTriangles"] >> triangles;
                    PRX_ASSERT(triangles.size()%3 == 0);
                    points.clear();

                    nr_inside_points = triangles.size()/3;
                    points.resize(nr_inside_points);
                    for(unsigned i=0; i<nr_inside_points; ++i)
                    {
                        next_point[0] = triangles[3*i];
                        next_point[1] = triangles[3*i+1];
                        next_point[2] = triangles[3*i+2] + 1.0;
                        points[i] = nav_space->alloc_point();
                        nav_space->set_from_vector(next_point, points[i]);
                    }
                    --nr_inside_points;

                    evacuation_point = node["isEvacutionPoint"];//!=false? false);

                    nr_departures = 0;

                    if(node["DepartureInformation"])
                    {
                        unsigned frames,capacity;
                        departures.push_back(departure_t(6000,5));
                        ++nr_departures;
                        departures.push_back(departure_t(9000,5));
                        ++nr_departures;
                        for(unsigned i = 0; i < node["DepartureInformation"].size(); ++i)
                        {
                            frames = node["DepartureInformation"][i]["Time"].as<unsigned>() * MSEC_2_FRAMES;
                            capacity =  node["DepartureInformation"][i]["Capacity"].as<double>();
                            // PRX_PRINT("Departure"<<nr_departures<<" Time: " <<frames << " Amount:" << capacity, PRX_TEXT_GREEN );
                            departures.push_back(departure_t(frames,capacity));
                            ++nr_departures;
                        }
                        std::sort( departures.begin(), departures.end() );
                    }
                    return;
                }

                PRX_FATAL_S("Region " << name << " does not have doors. Please fix the model!");                               
            }

            bool region_t::is_queue_present()
            {
                return queue!=NULL?true:false;
            }

            std::string region_t::get_queue_name()
            {
                return queue!=NULL?queue->queue_name:"NO QUEUE";
            }

            void region_t::clear()
            {
                foreach(space_point_t* point, doorways)
                    nav_space->free_point(point);
                doorways.clear();

                foreach(space_point_t* point, points)
                    nav_space->free_point(point);
                points.clear();
            }

            const std::vector< util::space_point_t* >& region_t::get_doorways()
            {
                return doorways;
            }

            const std::vector< util::space_point_t* >& region_t::get_inside_points()
            {
                return points;
            }

            void region_t::set_queue(queue_t* queue)
            {
                this->queue = queue;
            }

            const nav_node_t*  region_t::get_open_slot(behavior_controller_t* agent)
            {    
                if(queue!=NULL)
                {
                    return queue->get_open_slot(agent);
                }
                return NULL;
            }

            const nav_node_t*  region_t::reserve_slot(behavior_controller_t* agent, std::vector<double>& queue_points, int frames_to_leave, double &orientation)
            {
                if(queue!=NULL)
                {
                    return queue->reserve_slot(agent, queue_points, frames_to_leave, orientation);
                }
                return NULL;
            } 

            bool region_t::need_to_check_for_queue(util::space_point_t* current_state)
            {
                return nav_space->distance(current_state,doorways[uniform_int_random(0,nr_doorways)]) < QUEUE_EPSILON;
            }


            void region_t::check_departure(int current_frame)
            {
                // if(!departures.empty())
                // {
                //     PRX_PRINT("REGION:"<<name<<" DEPARTURE FRAME:"<<departures.front().frames<<" CURRENT FRAME:"<<current_frame,PRX_TEXT_BROWN);   
                // }

                if(queue!=NULL && !departures.empty() && departures.front().frames <= current_frame)
                {
                    // PRX_PRINT("REGION:"<<name<<" DEPARTURE Amount:"<<departures.front().amount<<" CURRENT FRAME:"<<current_frame,PRX_TEXT_CYAN);
                    queue->depart_agents(departures.front().amount);
                    departures.pop_front();
                }
            }

            // returns position in z-axis
            double region_t::get_height() const
            {
                return nodes[0]->get_height();
            }

            // Returns a random point in the attractor
            void region_t::get_point_to_go(std::vector<double>& attractor_point)
            {
                attractor_point = doorways[uniform_int_random(0,nr_doorways)]->memory;
            }

            bool region_t::is_evacuation_point()
            {
                return evacuation_point;
            }

            void region_t::decrease_occupancy(){}

            void region_t::increase_occupancy(){}

            bool region_t::has_available_space()
            {
                return false;
            }
        }
    }
}

