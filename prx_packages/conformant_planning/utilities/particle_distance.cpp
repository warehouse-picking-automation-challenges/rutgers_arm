/**
 * @file particle_distance.cpp 
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
#include "utilities/particle_distance.hpp"
#include "prx/utilities/spaces/space.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::packages::conformant::particle_distance_t, prx::util::distance_function_t)

namespace prx
{
    using namespace util;
    namespace packages
    {       
        namespace conformant
        {
            double particle_distance_t::distance(const space_point_t* p1, const space_point_t* p2)
            {
                double total = 0;
                //code for EMD
                std::fill(used.begin(), used.end(), false);
                const double* p1p = &p1->memory.front();
                const double* p2p = &p2->memory.front();
                for(unsigned i=0;i<number_of_states;i++)
                {
                    double min_dist = PRX_INFINITY;
                    unsigned best_index = 0;
                    for(unsigned index=0;index<state_size;index++)
                        subpoint1->memory[index] = p1p[i*state_size+index];
                    std::vector<bool>::const_iterator iter = used.begin();
                    for(unsigned j=0;j<number_of_states;j++,iter++)
                    {
                        if(!(*iter))
                        {
                            for(unsigned index=0;index<state_size;index++)
                                subpoint2->memory[index] = p2p[j*state_size+index];
                            double val = subspace->distance(subpoint1,subpoint2);
                            // sqrt((subpoint1->memory[0] - subpoint2->memory[0])*(subpoint1->memory[0] - subpoint2->memory[0]) + 
                            //             (subpoint1->memory[1] - subpoint2->memory[1])*(subpoint1->memory[1] - subpoint2->memory[1]));
                            if(val < min_dist )
                            {
                                min_dist = val;
                                best_index = j;
                            }

                        }
                    }
                    // PRX_INFO_S(best_index);
                    used[best_index] = true;
                    total+=min_dist;
                }
                return total;

            }

            void particle_distance_t::link_space(const space_t* space)
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
                used.resize(number_of_states);
            }
        }
    }
}
