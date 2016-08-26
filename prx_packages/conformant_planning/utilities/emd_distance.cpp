/**
 * @file emd_distance.cpp 
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
#include "utilities/emd_distance.hpp"
#include "prx/utilities/spaces/space.hpp"
#include <pluginlib/class_list_macros.h>
#include <boost/range/adaptor/map.hpp> //adaptors
// #include "wemd.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


PLUGINLIB_EXPORT_CLASS( prx::packages::conformant::emd_distance_t, prx::util::distance_function_t)

namespace prx
{
    using namespace util;
    using namespace cv;
    namespace packages
    {       
        namespace conformant
        {
            // typedef struct
            // {
            //   int n;                
            //   feature_t *Features; 
            //   double *Weights;       
            // } signature_t;

            double emd_distance_t::distance(const space_point_t* p1, const space_point_t* p2)
            {
                counts.clear();
                counts2.clear();
                valid_indices.clear();
                valid_indices2.clear();

                const particle_point_t* p1p = (const particle_point_t*)p1;
                const particle_point_t* p2p = (const particle_point_t*)p2;

                for(unsigned long i=0;i<number_of_states;i++)
                {
                    populate_point(p1p,i,0);
                    populate_point(p2p,i,1);
                }
                unsigned size1 = counts.size();
                unsigned size2 = counts2.size();
                double *w1 = new double[size1];
                double *w2 = new double[size2];

                unsigned counter = 0;
                foreach(insert& elem, counts | boost::adaptors::map_values )
                {
                    f1[counter].size = state_size;
                    f1[counter].isAlloc = false;
                    w1[counter] = elem.count;
                    f1[counter].x = elem.real_world;
                    counter++;
                }
                counter = 0;
                foreach(insert& elem, counts2 | boost::adaptors::map_values )
                {
                    f2[counter].size = state_size;
                    f2[counter].isAlloc = false;
                    w2[counter] = elem.count;
                    f2[counter].x = elem.real_world;
                    counter++;
                }

                signature_t s1,s2;
                s1.n = size1;
                s2.n = size2;
                s1.Features = f1;
                s2.Features = f2;
                s1.Weights = w1;
                s2.Weights = w2;

                double dist = emd(&s1,&s2,dist_p,NULL,NULL);
                delete w1;
                delete w2;
                foreach(insert& elem, counts | boost::adaptors::map_values )
                {
                    delete elem.real_world;
                }
                foreach(insert& elem, counts2 | boost::adaptors::map_values )
                {
                    delete elem.real_world;
                }
                return dist;
            }

            // double emd_distance_t::distance(const space_point_t* p1, const space_point_t* p2)
            // {
            //     clock.reset();
            //     counts.clear();
            //     counts2.clear();

            //     const double* p1p = &p1->memory.front();
            //     const double* p2p = &p2->memory.front();

            //     for(unsigned long i=0;i<number_of_states;i++)
            //     {
            //         // unsigned long index = get_index(p1p,i);
            //         // valid_indices.insert(index);
            //         populate_point(p1p,i,0);
            //         // index = get_index(p2p,i);
            //         // valid_indices2.insert(index);
            //         populate_point(p2p,i,1);
            //     }
            //     unsigned size1 = counts.size();//valid_indices.size();
            //     unsigned size2 = counts2.size();//valid_indices2.size();

            //     Mat sig1(size1, state_size+1, CV_32FC1);
            //     Mat sig2(size2, state_size+1, CV_32FC1);

            //     unsigned i=0;
            //     foreach(insert& elem, counts | boost::adaptors::map_values )
            //     {
            //         sig1.at<float>(i, 0) = (elem.count*1.0/number_of_states);
            //         for(unsigned j=0; j<state_size ; j++)
            //         {
            //             sig1.at<float>(i, j+1) = elem.real_world[j];
            //         }
            //         i++;
            //     }
            //     i=0;
            //     foreach(insert& elem, counts2 | boost::adaptors::map_values )
            //     {
            //         sig2.at<float>(i, 0) = (elem.count*1.0/number_of_states);
            //         for(unsigned j=0; j<state_size ; j++)
            //         {
            //             sig2.at<float>(i, j+1) = elem.real_world[j];
            //         }
            //         i++;
            //     }
            //     float val = EMD(sig1,sig2,CV_DIST_L2);

            //     total_time+=clock.measure();
            //     count_of_calls++;
            //     if(count_of_calls==10000)
            //     {
            //         PRX_INFO_S(total_time/count_of_calls);
            //     }
            //     return val;
            // }

            void emd_distance_t::link_space(const space_t* space)
            {
                particle_space = dynamic_cast<const particle_space_t*>(space);
                if(particle_space!=NULL)
                {
                    state_size = particle_space->get_point_space()->get_dimension();
                    number_of_states = particle_space->get_num_particles();
                    distance_function_t::link_space(particle_space->get_point_space());

                    bin_size = .25;
                    bin_size = 1.0/bin_size;
                    for(unsigned i=0;i<state_size;++i)
                    {
                        index_offsets.push_back(1);
                    }
                    for(unsigned i=state_size-1;i>0;--i)
                    {
                        unsigned long val = (unsigned long)(ref_space->get_bounds()[i]->get_upper_bound()
                                                                -ref_space->get_bounds()[i]->get_lower_bound())*bin_size;
                        for(unsigned j=0;j<i;j++)
                        {
                            index_offsets[j]*=val;
                        }
                    }
                    f1 = new feature_t[number_of_states];
                    f2 = new feature_t[number_of_states];
                }
            }

            unsigned long emd_distance_t::get_index(const particle_point_t* p, unsigned long index)
            {
                unsigned long pos = 0;
                space_point_t* link = p->links[index];
                for(unsigned long i=0;i<state_size;i++)
                {
                    unsigned long val = (link->memory[i]-ref_space->get_bounds()[i]->get_lower_bound())*bin_size;
                    pos += index_offsets[i]*val;
                }
                return pos;
            }
            void emd_distance_t::populate_point(const particle_point_t* p, unsigned long index, int choice)
            {
                unsigned long val = get_index(p,index);
                space_point_t* link = p->links[index];
                if(choice == 0)
                {
                    insert& elem = counts[val];
                    elem.count++;
                    if(elem.count==1)
                    {
                        elem.real_world = new double[state_size];
                        for(unsigned i=0;i<state_size;++i)
                        {
                            elem.real_world[i] = (floor(link->memory[i]*bin_size))/bin_size;
                        }
                        // valid_indices.insert(val);
                    }
                }
                else if(choice == 1)
                {
                    insert& elem = counts2[val];
                    elem.count++;

                    if(elem.count==1)
                    {
                        elem.real_world = new double[state_size];
                        for(unsigned i=0;i<state_size;++i)
                        {
                            elem.real_world[i] = (floor(link->memory[i]*bin_size))/bin_size;
                        }
                        // valid_indices2.insert(val);
                    }
                }
            }
        }
    }
}
