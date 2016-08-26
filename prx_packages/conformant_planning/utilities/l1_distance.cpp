/**
 * @file l1.cpp 
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
#include "utilities/l1_distance.hpp"
#include "prx/utilities/spaces/space.hpp"
#include <pluginlib/class_list_macros.h>
#include <limits>

PLUGINLIB_EXPORT_CLASS( prx::packages::conformant::l1_distance_t, prx::util::distance_function_t)

namespace prx
{
    using namespace util;
    namespace packages
    {       
        namespace conformant
        {
            double l1_distance_t::distance(const space_point_t* p1, const space_point_t* p2)
            {
                counts.clear();
                unsigned long total = 0;
                //code for EMD
                const double* p1p = &p1->memory.front();
                const double* p2p = &p2->memory.front();
                for(unsigned long i=0;i<number_of_states;i++)
                {
                    unsigned long index = get_index(p1p,i);
                    counts[index]++;
                    total++;
                }
                for(unsigned long i=0;i<number_of_states;i++)
                {
                    unsigned long index = get_index(p2p,i);
                    if(counts[index]!=0)
                    {
                        total--;
                        counts[index]--;
                    }
                    else
                    {
                        total++;
                    }
                }

                // PRX_WARN_S("Number of States: "<<number_of_states<<" State_size: "<<state_size<<" --- "<<total);
                return total;
            }

            void l1_distance_t::link_space(const space_t* space)
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
            unsigned long l1_distance_t::get_index(const double* p, unsigned long index)
            {
                unsigned long pos = 0;
                unsigned long offset = index*state_size;
                for(unsigned long i=0;i<state_size;i++)
                {
                    unsigned long val = (p[offset+i]-ref_space->get_bounds()[i]->get_lower_bound())*bin_size;
                    pos += index_offsets[i]*val;
                }
                return pos;
            }
        }
    }
}
