/**
 * @file naive_octree.cpp 
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

#include "utilities/naive_octree.hpp"
#include <pluginlib/class_list_macros.h>
#include "prx/utilities/parameters/parameter_reader.hpp"

PLUGINLIB_EXPORT_CLASS( prx::packages::conformant::naive_octree_t, prx::util::distance_metric_t)

namespace prx
{
    using namespace util;
    namespace packages
    {       
        namespace conformant
        {
            naive_octree_t::naive_octree_t() { }

            naive_octree_t::~naive_octree_t()
            {
                clear();
            }
            void naive_octree_t::init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader)
            {
                distance_metric_t::init(reader,template_reader);
                threshold = parameters::get_attribute_as<double>("threshold",reader,template_reader);
            }
            void naive_octree_t::link_space(const space_t* inspace)
            {
                particle_space = dynamic_cast<const particle_space_t*>(inspace);
                if(particle_space!=NULL)
                    distance_metric_t::link_space(inspace);
            }

            double naive_octree_t::kostas_distance(const util::space_point_t* p1, const util::space_point_t* p2) const
            {
                const particle_point_t* p1p = (const particle_point_t*)p1;
                const particle_point_t* p2p = (const particle_point_t*)p2;
                std::vector<double> point1;
                std::vector<double> point2;
                // PRX_INFO_S(state_size);
                unsigned state_size = particle_space->get_point_space()->get_dimension();
                point1.resize(state_size);
                point2.resize(state_size);
                for(unsigned long j=0;j<state_size;j++)
                {
                    point1[j] = 0;
                    point2[j] = 0;
                }
                unsigned number_of_states = particle_space->get_num_particles();
                for(unsigned long i=0;i<number_of_states;i++)
                {
                    for(unsigned long j=0;j<state_size;j++)
                    {
                        point1[j] += p1p->links[i]->memory[j];
                        point2[j] += p2p->links[i]->memory[j];
                    }
                }
                double val = 0;
                for(unsigned long j=0;j<state_size;j++)
                {
                    point1[j] /= number_of_states;
                    point2[j] /= number_of_states;
                    // PRX_INFO_S(point1[j]<<" "<<point2[j]);
                    val+=(point1[j]-point2[j])*(point1[j]-point2[j]);
                }
                val = sqrt(val);
                double temp_dist = 0;
                double dist1 = 0;
                double dist2 = 0;
                for(unsigned long i=0;i<number_of_states;i++)
                {
                    temp_dist = 0;
                    for(unsigned long j=0;j<state_size;j++)
                    {
                        temp_dist+=(point1[j]-p1p->links[i]->memory[j])*(point1[j]-p1p->links[i]->memory[j]);
                    }
                    if(dist1<temp_dist)
                    {
                        dist1 = temp_dist;
                    }
                }
                for(unsigned long i=0;i<number_of_states;i++)
                {
                    temp_dist = 0;
                    for(unsigned long j=0;j<state_size;j++)
                    {
                        temp_dist+=(point2[j]-p2p->links[i]->memory[j])*(point2[j]-p2p->links[i]->memory[j]);
                    }
                    if(dist2<temp_dist)
                    {
                        dist2 = temp_dist;
                    }
                }
                // PRX_INFO_S(dist1<<" "<<dist2);
                dist1 = sqrt(dist1);
                dist2 = sqrt(dist2);
                // PRX_INFO_S(dist1<<" "<<dist2);
                // PRX_INFO_S(val);
                if(val-dist1-dist2 < threshold)
                    return distance_function(p1,p2);
                else
                    return val;
            }

            unsigned naive_octree_t::add_point(const abstract_node_t* embed)
            {
                unsigned k;
                if( nr_points > 14 )
                    k = 4.25 * log(nr_points);
                else
                    k = nr_points;

                points.push_back(embed);
                ++nr_points;
                return nr_points;
            }

            unsigned naive_octree_t::add_points(const std::vector< const abstract_node_t* >& embeds)
            {
                for( unsigned i = 0; i < embeds.size(); ++i )
                    add_point(embeds[i]);
                return nr_points;
            }

            void naive_octree_t::remove_point(const abstract_node_t* embed)
            {
                unsigned counter = 0;
                bool found = false;

                while( !found && counter < nr_points )
                {
                    if( embed == points[counter] )
                        found = true;
                    else
                        ++counter;
                }
                if( found )
                {
                    --nr_points;
                    points[counter] = points[nr_points];
                    points.pop_back();
                }
            }

            const std::vector< const abstract_node_t* > naive_octree_t::multi_query(const space_point_t* query_point, unsigned ink) const
            {
                //Make sure we are performing a query for less or as many points as we know
                if( ink > nr_points )
                    ink = nr_points;
                //Set up the vector for the return
                std::vector< const abstract_node_t* > ret;
                ret.resize(ink);
                double dists[ink];
                double tmp_dist;
                double worst_dist = 0.0;
                unsigned worst_index = 0;
                //First make an initial population to return
                for( unsigned i = 0; i < ink; i++ )
                {
                    ret[i] = points[i];
                    dists[i] = kostas_distance(query_point, points[i]->point);
                    if( dists[i] > worst_dist )
                    {
                        worst_index = i;
                        worst_dist = dists[i];
                    }
                }

                //Then, search through the remaining population to get the closest neighbors
                for( unsigned i = ink; i < nr_points; i++ )
                {
                    //Get the linear distance
                    tmp_dist = kostas_distance(query_point, points[i]->point);
                    //If this distance is better than our worst distance
                    if( tmp_dist < worst_dist )
                    {
                        //Replace the dude who is worst
                        ret[worst_index] = points[i];
                        dists[worst_index] = tmp_dist;
                        //Now find the new worst guy ever
                        worst_dist = 0;
                        for( unsigned k = 0; k < ink; k++ )
                        {
                            if( dists[k] > worst_dist )
                            {
                                worst_dist = dists[k];
                                worst_index = k;
                            }
                        }
                    }
                }

                // std::sort(ret.begin(), ret.end(), compare_node_t(kostas_distance, query_point));
                return ret;
            }

            const std::vector< const abstract_node_t* > naive_octree_t::radius_query(const space_point_t* query_point, double rad) const
            {
                //Set up the vector for the return
                std::vector< const abstract_node_t* > ret;

                //Then, search through the remaining population to get the closest neighbors
                for( unsigned i = 0; i < nr_points; i++ )
                {
                    //Get the linear distance
                    double tmp_dist = kostas_distance(query_point, points[i]->point);
                    if( tmp_dist < rad )
                        ret.push_back(points[i]);
                }

                // std::sort(ret.begin(), ret.end(), compare_node_t(kostas_distance, query_point));

                return ret;
            }

            unsigned naive_octree_t::radius_query(const space_point_t* query_point, double rad, std::vector< const abstract_node_t* >& ret) const
            {
                unsigned counter = 0;
                std::vector<const abstract_node_t*>::iterator iter;
                std::vector<const abstract_node_t*>::const_iterator node_iter;
                iter = ret.begin();
                for( node_iter = points.begin(); node_iter != points.end(); node_iter++ )
                {
                    const abstract_node_t* node = *node_iter;
                    double distance = kostas_distance(node->point, query_point);
                    if( distance < rad )
                    {
                        *iter = node;
                        iter++;
                        counter++;
                    }
                }
                return counter;
            }

            const std::vector< const abstract_node_t* > naive_octree_t::radius_and_closest_query(const space_point_t* query_point, double rad, const abstract_node_t*& closest)const
            {
                std::vector< const abstract_node_t* > ret;
                for( unsigned i = 0; i < nr_points; i++ )
                {
                    double distance = kostas_distance(points[i]->point, query_point);
                    if( distance < rad )
                    {
                        ret.push_back(points[i]);
                    }
                }
                if( ret.size() == 0 )
                {
                    double min_distance = std::numeric_limits<double>::max();
                    int min_index = -1;
                    for( unsigned i = 0; i < nr_points; i++ )
                    {
                        double distance = kostas_distance(points[i]->point, query_point);
                        if( distance < min_distance )
                        {
                            min_index = i;
                            min_distance = distance;
                        }
                    }
                    ret.push_back(points[ min_index ]);
                }
                // std::sort(ret.begin(), ret.end(), compare_node_t(kostas_distance, query_point));
                return ret;
            }

            unsigned naive_octree_t::radius_and_closest_query(const space_point_t* query_point, double rad, std::vector<const abstract_node_t*>& closest) const
            {
                unsigned counter = 0;
                std::vector<const abstract_node_t*>::iterator iter;
                std::vector<const abstract_node_t*>::const_iterator node_iter;
                iter = closest.begin();
                for( node_iter = points.begin(); node_iter != points.end(); node_iter++ )
                {
                    const abstract_node_t* node = *node_iter;
                    double distance = kostas_distance(node->point, query_point);
                    if( distance < rad )
                    {
                        *iter = node;
                        iter++;
                        counter++;
                    }
                }
                if( counter == 0 )
                {
                    double min_distance = std::numeric_limits<double>::max();
                    for( node_iter = points.begin(); node_iter != points.end(); node_iter++ )
                    {
                        const abstract_node_t* node = *node_iter;
                        double distance = kostas_distance(node->point, query_point);
                        if( distance < min_distance )
                        {
                            *iter = node;
                            min_distance = distance;
                        }
                    }
                    iter++;
                    counter++;
                }

                // std::sort(closest.begin(), iter, compare_node_t(kostas_distance, query_point));
                return counter;
            }

            const abstract_node_t* naive_octree_t::single_query(const space_point_t* query_point, double* dist) const
            {
                double min_distance = std::numeric_limits<double>::max();
                int min_index = -1;
                for( unsigned i = 0; i < nr_points; i++ )
                {
                    double distance = kostas_distance(points[i]->point, query_point);
                    if( distance < min_distance )
                    {
                        min_index = i;
                        min_distance = distance;
                    }
                }
                if(dist!=NULL)
                    *dist = min_distance;
                if( min_index != -1 )
                    return points[ min_index ];
                else
                    return NULL;
            }

            void naive_octree_t::clear()
            {
                points.resize(0);
                nr_points = 0;
            }

            void naive_octree_t::rebuild_data_structure() {
                // The Array will take care of itself as it is being updated
                // This function thusly does nothing.
            }
        }
    }
}



