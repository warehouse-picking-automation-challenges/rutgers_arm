/**
 * @file hausdorff_distance.cpp 
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
#include "utilities/hausdorff_distance.hpp"
#include "prx/utilities/spaces/space.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::packages::conformant::hausdorff_distance_t, prx::util::distance_function_t)

namespace prx
{
    using namespace util;
    namespace packages
    {       
        namespace conformant
        {
            double hausdorff_distance_t::distance(const space_point_t* p1, const space_point_t* p2)
            {
                counts.clear();
                counts2.clear();
                valid_indices.clear();
                valid_indices2.clear();
                double h = 0;
                //code for Hausdorff

                const double* p1p = &p1->memory.front();
                const double* p2p = &p2->memory.front();

                for(unsigned long i=0;i<number_of_states;i++)
                {
                    populate_point(p1p,i,0);
                    populate_point(p2p,i,1);
                }
                foreach(unsigned long index, valid_indices)
                {
                    double min_dist = PRX_INFINITY;
                    for(unsigned i=0;i<state_size;i++)
                        subpoint1->at(i) = counts[index].real_world[i];
                    foreach(unsigned long index2, valid_indices2)
                    {
                        for(unsigned i=0;i<state_size;i++)
                            subpoint2->at(i) = counts2[index2].real_world[i];
                        double val = subspace->distance(subpoint1,subpoint2);
                        if(val < min_dist )
                        {
                            min_dist = val;
                            // best_index = j;
                        }

                    }
                    delete counts[index].real_world;
                    h = PRX_MAXIMUM(h,min_dist);
                }
                foreach(unsigned long index2, valid_indices2)
                {
                    delete counts2[index2].real_world;
                }
                return h;


            }

            void hausdorff_distance_t::link_space(const space_t* space)
            {
                distance_function_t::link_space(space);
                std::stringstream ss(ref_space->get_space_name());
                std::string item;
                std::getline(ss, item, '|');
                parameter_reader_t reader("prx/spaces",global_storage);
                std::string topo;
                if( reader.has_attribute(item) )
                    topo = reader.get_attribute(item);
                state_size = topo.size();
                number_of_states = ref_space->get_dimension()/state_size;
                memory.resize(topo.size());
                for(unsigned i=0;i<topo.size();i++)
                    memory[i] = new double;
                subspace = new space_t(item,memory);
                for(unsigned i=0;i<state_size;i++)
                    *subspace->get_scales()[i] = *ref_space->get_scales()[i];
                subpoint1 = subspace->alloc_point();
                subpoint2 = subspace->alloc_point();
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
            }
            unsigned long hausdorff_distance_t::get_index(const double* p, unsigned long index)
            {
                unsigned long pos = 0;
                for(unsigned long i=0;i<state_size;i++)
                {
                    unsigned long val = (p[index*state_size+i]-ref_space->get_bounds()[i]->get_lower_bound())*bin_size;
                    pos += index_offsets[i]*val;
                }
                return pos;
            }
            void hausdorff_distance_t::populate_point(const double* p, unsigned long index, int choice)
            {
                unsigned long val = get_index(p,index);
                if(choice == 0)
                {
                    insert& elem = counts[val];
                    elem.count++;
                    if(elem.count==1)
                    {
                        elem.real_world = new double[subspace->get_dimension()];
                        for(unsigned i=0;i<state_size;++i)
                        {
                            elem.real_world[i] = (floor(p[index*state_size+i]*bin_size))/bin_size;
                        }
                        valid_indices.insert(val);
                    }
                }
                else if(choice == 1)
                {
                    insert& elem = counts2[val];
                    elem.count++;
                    if(elem.count==1)
                    {
                        elem.real_world = new double[subspace->get_dimension()];
                        for(unsigned i=0;i<state_size;++i)
                        {
                            elem.real_world[i] = (floor(p[index*state_size+i]*bin_size))/bin_size;
                        }
                        valid_indices2.insert(val);
                    }
                }
            }
        }
    }
}
