/**
 * @file kl_distance.cpp 
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
#include "utilities/kl_closed_distance.hpp"
#include "prx/utilities/spaces/space.hpp"
#include <pluginlib/class_list_macros.h>
 #include <iomanip>
 #include <limits>
 #include <Eigen/Dense>
 #include <Eigen/LU>

PLUGINLIB_EXPORT_CLASS( prx::packages::conformant::kl_closed_distance_t, prx::util::distance_function_t)

using Eigen::MatrixXd;

namespace prx
{
    using namespace util;
    namespace packages
    {       
        namespace conformant
        {
            double kl_closed_distance_t::distance(const space_point_t* p1, const space_point_t* p2)
            {
                MatrixXd cov1(state_size,state_size);
                cov1 = MatrixXd::Zero(state_size,state_size);
                MatrixXd cov2(state_size,state_size);
                cov2 = MatrixXd::Zero(state_size,state_size);
                MatrixXd mean1(state_size,1);
                mean1 = MatrixXd::Zero(state_size,1);
                MatrixXd mean2(state_size,1);
                mean2 = MatrixXd::Zero(state_size,1);

                const double* p1p = &p1->memory.front();
                const double* p2p = &p2->memory.front();
                for(unsigned long i=0;i<number_of_states;i++)
                {
                    for(unsigned j=0;j<state_size;j++)
                    {
                        mean1(j,0) += p1p[state_size*i+j];
                        mean2(j,0) += p2p[state_size*i+j];
                    }
                    // PRX_INFO_S("\n"<<mean1<<"\n"<<mean2);
                }
                for(unsigned j=0;j<state_size;j++)
                {
                    mean1(j,0) /= number_of_states;
                    mean2(j,0) /= number_of_states;
                }
                for(unsigned long i=0;i<number_of_states;i++)
                {
                    std::vector<double> diff(state_size,0);
                    std::vector<double> diff2(state_size,0);
                    for(unsigned j=0;j<state_size;j++)
                    {
                        diff[j] += p1p[state_size*i+j]-mean1(j,0);
                        diff2[j] += p2p[state_size*i+j]-mean2(j,0);
                    }
                    for(unsigned j=0;j<state_size;j++)
                    {
                        for(unsigned k=0;k<state_size;k++)
                        {
                            cov1(j,k) += diff[j]*diff[k];
                            cov2(j,k) += diff2[j]*diff2[k];
                            if(j==k)
                            {
                                cov1(j,k) += .00001;
                                cov2(j,k) += .00001;
                            }
                        }
                    }
                }
                for(unsigned j=0;j<state_size;j++)
                {
                    for(unsigned k=0;k<state_size;k++)
                    {
                        cov1(j,k) /= number_of_states;
                        cov2(j,k) /= number_of_states;
                    }
                }

                MatrixXd temp_result(state_size,1);
                temp_result = mean2-mean1;
                MatrixXd temp_resultTrans = temp_result.transpose();
                MatrixXd cov2inv = cov2.inverse();

                MatrixXd temp_cov_mult = cov2inv*cov1;

                double result = temp_cov_mult.trace();
                MatrixXd interResult = (temp_resultTrans*cov2inv*temp_result);
                result+=interResult(0,0);
                result-=state_size;
                result+=log(cov2.determinant()/cov1.determinant());
                result*=.5;
                if(std::isfinite(result))
                {
                    return result;
                }
                return PRX_INFINITY;


            }

            void kl_closed_distance_t::link_space(const space_t* space)
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
            unsigned long kl_closed_distance_t::get_index(const double* p, unsigned long index)
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
