// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include "prx/utilities/definitions/sys_clock.hpp"

#include "simulation/prx_chainiksolvervel_pinv_nso.hpp"

#include <math.h>
#include <kdl/utilities/svd_eigen_HH.hpp>
#include <kdl/frames_io.hpp>

namespace prx
{
    namespace sim
    {
        namespace simulation
        {
            extern double simulation_step;            
        }
    }
    
    namespace packages
    {
        using namespace util;
        using namespace sim;
        using namespace KDL;

        namespace manipulation
        {
            prx_chainiksolvervel_pinv_nso::prx_chainiksolvervel_pinv_nso(const Chain& _chain, JntArray _opt_pos, JntArray _weights, double _eps, int _maxiter, double _alpha):
                chain(_chain),
                jnt2jac(chain),
                nj(chain.getNrOfJoints()),
                critical_H_rank( chain.getNrOfJoints()/2 + chain.getNrOfJoints()%2 ),
                jac(nj),
                U(MatrixXd::Zero(6,nj)),
                U_nj(MatrixXd::Zero(nj,nj)),
                H(VectorXd::Zero(nj)),
                I(MatrixXd::Identity(nj,nj)),
                S(VectorXd::Zero(nj)),
                S_nj(VectorXd::Zero(nj)),
                Sinv(VectorXd::Zero(nj)),
                SinvL(VectorXd::Zero(nj)),
                J(MatrixXd::Zero(6,nj)),
                J_inv(MatrixXd::Zero(nj,6)),
                J_Q_cross(MatrixXd::Zero(nj,6)),
                qmin(chain.getNrOfJoints()),
                qmax(chain.getNrOfJoints()),
                V(MatrixXd::Zero(nj,nj)),
                Uhat( MatrixXd::Zero(nj,nj) ),
                Shat( VectorXd::Zero(nj) ),
                Vhat( MatrixXd::Zero(6,6) ),
                e_dot(VectorXd::Zero(6)),
                tmp(VectorXd::Zero(nj)),
                tmp2(VectorXd::Zero(nj)),
                projected_q(VectorXd::Zero(nj)),
                eps(_eps),
                maxiter(_maxiter),
                alpha(_alpha),
                weights(_weights),
                opt_pos(_opt_pos),
                elapsed(0)
            {
                compute_powerset();
            }

            prx_chainiksolvervel_pinv_nso::prx_chainiksolvervel_pinv_nso(const Chain& _chain, double _eps, int _maxiter, double _alpha):
                chain(_chain),
                jnt2jac(chain),
                nj(chain.getNrOfJoints()),
                critical_H_rank( chain.getNrOfJoints()/2 + chain.getNrOfJoints()%2 ),
                jac(nj),
                U(MatrixXd::Zero(6,nj)),
                U_nj(MatrixXd::Zero(nj,nj)),
                H(VectorXd::Zero(nj)),
                I(MatrixXd::Identity(nj,nj)),
                S(VectorXd::Zero(nj)),
                S_nj(VectorXd::Zero(nj)),
                Sinv(VectorXd::Zero(nj)),
                SinvL(VectorXd::Zero(nj)),
                J(MatrixXd::Zero(6,nj)),
                J_inv(MatrixXd::Zero(nj,6)),
                J_Q_cross(MatrixXd::Zero(nj,6)),
                ones(VectorXd::Ones(nj)),
                qmin(chain.getNrOfJoints()),
                qmax(chain.getNrOfJoints()),
                V(MatrixXd::Zero(nj,nj)),
                Uhat( MatrixXd::Zero(nj,nj) ),
                Shat( VectorXd::Zero(nj) ),
                Vhat( MatrixXd::Zero(6,6) ),
                e_dot(VectorXd::Zero(6)),
                tmp(VectorXd::Zero(nj)),
                tmp2(VectorXd::Zero(nj)),
                projected_q(VectorXd::Zero(nj)),
                eps(_eps),
                maxiter(_maxiter),
                alpha(_alpha),
                elapsed(0)
            {
            }

            prx_chainiksolvervel_pinv_nso::~prx_chainiksolvervel_pinv_nso()
            {
            }  


            int prx_chainiksolvervel_pinv_nso::CartToJntChiaverini(const KDL::JntArray& q_in, const KDL::Twist& v_in, KDL::JntArray& qdot_out)
            {
                jnt2jac.JntToJac(q_in,jac);

                int svdResult = svd_eigen_HH(jac.data,U,S,V,tmp,maxiter);
                if (0 != svdResult)
                {
                    qdot_out.data.setZero() ;
                    return svdResult;
                }

                if( S(5) < PRX_ZERO_CHECK )
                {
                    PRX_FATAL_S("SVD decomposition gave something not full-rank!");
                }

                nj = q_in.data.size();
                unsigned int i;
                double lambda = 1e-3;

                for (i = 0; i < nj; ++i) 
                {
                    SinvL(i) = S(i)/(S(i)*S(i)+lambda*lambda);
                }
                for (i = 0; i < 6; ++i) 
                {
                    tmp(i) = v_in(i);
                }
                MatrixXd temp_mat = MatrixXd::Zero(nj,1);
                VectorXd ones = VectorXd::Ones(nj);
                MatrixXd null_space_movement = MatrixXd::Zero(nj,1);
                mt_v_m(temp_mat,SinvL,U,tmp.head(6),V,0,5);
                qdot_out.data = temp_mat;

                double min_diff=9999;
                int index_best = -1;
                alpha = 0;
                for (i = 0; i < nj; ++i) 
                {
                    double range = qmax(i)-qmin(i);

                    double s_i;
                    double q_max_val = qmax(i)-range/10.0;
                    double q_min_val = qmin(i)+range/10.0;
                    if(q_in(i) > q_max_val)
                        s_i = (q_in(i) - q_max_val)/range;
                    else if(q_in(i) < q_min_val)
                        s_i = (q_in(i) - q_min_val)/range;
                    else
                        s_i = 0;

                    if(fabs(q_in(i)+.0002*qdot_out(i)-qmax(i)) < min_diff )
                    {
                        min_diff = fabs(q_in(i)+.0002*qdot_out(i)-qmax(i));
                        index_best = i;
                    }
                    if(fabs(q_in(i)+.0002*qdot_out(i)-qmin(i)) < min_diff )
                    {
                        min_diff = fabs(q_in(i)+.0002*qdot_out(i)-qmin(i));
                        index_best = i;
                    }

                    tmp(i) = s_i;
                }

                MatrixXd tmp_mat2 = MatrixXd::Zero(nj,1);
                mt_v_m(tmp_mat2,ones,V,tmp,V,6,8);
                // if(index_best!=0)
                //     std::cout<<"index"<<index_best<<std::endl;
                if(tmp(index_best)==0)
                {
                    alpha = 0;

                    ones*=-alpha;

                    mt_v_m(null_space_movement,ones,V,tmp_mat2,V,6,8);

                    qdot_out.data += null_space_movement;
                }
                else
                {   
                    // PRX_WARN_S("aCTUALLY APPLYING");
                    alpha = qdot_out(index_best)/(tmp_mat2(index_best,0));

                    ones*=-alpha;

                    // PRX_INFO_S("\n"<<qdot_out.data);
                    mt_v_m(null_space_movement,ones,V,tmp_mat2,V,6,8);

                    // PRX_INFO_S("\n"<<null_space_movement);
                    qdot_out.data += null_space_movement;
                    // PRX_INFO_S("\n"<<qdot_out.data);
                }

                return svdResult;

            }

            double prx_chainiksolvervel_pinv_nso::measure_jac()
            {
                double ret = jaco_time;
                jaco_time = 0;
                return ret;
            }

            double prx_chainiksolvervel_pinv_nso::measure_svd()
            {
                double ret = svd_time;
                svd_time = 0;
                return ret;
            }

            int prx_chainiksolvervel_pinv_nso::CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out)
            {
                sys_clock_t pclock;

                //Parameter: damping constant
                double lambda = 1e-3;
                Eigen::VectorXd& q = tmp2;

                //Let's begin by copying over our current state information
                for( unsigned i=0; i<nj; ++i )
                {
                    q(i) = q_in(i);
                }
                //Copy over our e^dot into an eigen structure
                for( unsigned i = 0; i < 6; ++i )
                {
                    e_dot(i) = v_in(i);
                }

                //Then, if the error is within tolerance
                double jac_error = (q - projected_q).norm();
                // PRX_PRINT("Error: [" << jac_error << "]", PRX_TEXT_LIGHTGRAY);
                if( jac_error < 0.0087 )
                {
                    // PRX_PRINT("SAFE!", PRX_TEXT_GREEN);
                    //Compute our control law based on our previous Jacobian computations
                    qdot_out.data = J_Q_cross * e_dot;
                    //Let's take note of our projected state
                    projected_q = q + (simulation::simulation_step * qdot_out.data);
                    return 0;
                }
                // PRX_PRINT("UNSAFE!", PRX_TEXT_BLUE);

                pclock.reset();
                //Compute the end-effector task jacobian
                jnt2jac.JntToJac(q_in,jac);
                jaco_time += pclock.measure();

                //Copy over into an Eigen Matrix
                J = jac.data;

                //Compute the Jacobian pseudoinverse
                pclock.reset();
                int svdResult = compute_post_pseudoinverse( J_Q_cross, J, VectorXd::Ones(nj), lambda );
                svd_time += pclock.measure();
                //If the SVD didn't work out, abort mission
                if( svdResult != 0 )
                {
                    qdot_out.data.setZero();
                    return svdResult;
                }
                
                //Compute Q^dot based on the very simple control law
                qdot_out.data = J_Q_cross * e_dot;

                //Let's take note of our projected state
                projected_q = q + (simulation::simulation_step * qdot_out.data);

                //And report the svd result
                return svdResult;

                // double min_diff=9999;
                // int index_best = -1;
                // alpha = 0;
                // for (i = 0; i < nj; ++i) 
                // {
                //     double range = qmax(i)-qmin(i);

                //     double s_i;
                //     double q_max_val = qmax(i)-range/10.0;
                //     double q_min_val = qmin(i)+range/10.0;
                //     if(q_in(i) > q_max_val)
                //         s_i = (q_in(i) - q_max_val)/range;
                //     else if(q_in(i) < q_min_val)
                //         s_i = (q_in(i) - q_min_val)/range;
                //     else
                //         s_i = 0;

                //     if(fabs(q_in(i)+.0002*qdot_out(i)-qmax(i)) < min_diff )
                //     {
                //         min_diff = fabs(q_in(i)+.0002*qdot_out(i)-qmax(i));
                //         index_best = i;
                //     }
                //     if(fabs(q_in(i)+.0002*qdot_out(i)-qmin(i)) < min_diff )
                //     {
                //         min_diff = fabs(q_in(i)+.0002*qdot_out(i)-qmin(i));
                //         index_best = i;
                //     }

                //     tmp2(i) = s_i;
                // }

                // MatrixXd temp_mat = MatrixXd::Zero(nj,1);
                // mt_v_m(temp_mat,SinvL,U,tmp.head(6),V,0,5);

                // qdot_out.data = temp_mat;
                // qdot_out.data = V * SinvL.asDiagonal() * U.transpose() * tmp.head(6);

                // return svdResult;

                /////////////////// basic bounds avoidance
                // double min_diff=9999;
                // int index_best = -1;
                // alpha = 0;
                // for (i = 0; i < nj; ++i) 
                // {
                //     double range = qmax(i)-qmin(i);

                //     double s_i;
                //     double q_max_val = qmax(i)-range/10.0;
                //     double q_min_val = qmin(i)+range/10.0;
                //     if(q_in(i) > q_max_val)
                //         s_i = (q_in(i) - q_max_val)/range;
                //     else if(q_in(i) < q_min_val)
                //         s_i = (q_in(i) - q_min_val)/range;
                //     else
                //         s_i = 0;

                //     if(fabs(q_in(i)+.0002*qdot_out(i)-qmax(i)) < min_diff )
                //     {
                //         min_diff = fabs(q_in(i)+.0002*qdot_out(i)-qmax(i));
                //         index_best = i;
                //     }
                //     if(fabs(q_in(i)+.0002*qdot_out(i)-qmin(i)) < min_diff )
                //     {
                //         min_diff = fabs(q_in(i)+.0002*qdot_out(i)-qmin(i));
                //         index_best = i;
                //     }

                //   tmp(i) = s_i;
                // }
                // MatrixXd temp_mat1 = MatrixXd::Zero(nj,nj);
                // vector_cross_product(temp_mat1,V,V,0,5);
                // MatrixXd proj = I - temp_mat1;

                // tmp2 = (proj3*tmp);
                // // if(index_best!=0)
                // //     std::cout<<"index"<<index_best<<std::endl;
                // if(tmp(index_best)==0)
                //     alpha = 0;
                // else
                //     alpha = qdot_out(index_best)/(tmp2(index_best));

                // tmp2 = -alpha*(proj3*tmp);
                // for (i = 0; i < nj; ++i) 
                // {
                //     qdot_out(i) += tmp2(i);
                // }
                // //return the return value of the svd decomposition
                // return svdResult;
                ///////////////////////


                //////////////////////////////////old existing code
                // double g = 0; // g(q)
                // double A = 0; // normalizing term
                // for (i = 0; i < nj; ++i) {
                //     double qd = q_in(i) - opt_pos(i);
                //     g += 0.5 * qd*qd * weights(i);
                //     A += qd*qd * weights(i)*weights(i);
                // }

                // if (A > 1e-9) {
                //   // Calculate inverse Jacobian Jc^-1
                //   for (i = 0; i < nj; ++i) {
                //       tmp(i) = weights(i)*(q_in(i) - opt_pos(i)) / A;
                //   }

                //   // Calcualte J^-1 * J * Jc^-1 = V*S^-1*U' * U*S*V' * tmp
                //   tmp2 = V * Sinv.asDiagonal() * U.transpose() * U * S.asDiagonal() * V.transpose() * tmp;

                //   for (i = 0; i < nj; ++i) {
                //       //std::cerr << i <<": "<< qdot_out(i) <<", "<< -2*alpha*g * (tmp(i) - tmp2(i)) << std::endl;
                //       qdot_out(i) += -2*alpha*g * (tmp(i) - tmp2(i));
                //   }
                // }
                // //return the return value of the svd decomposition
                // return svdResult;
            }


            int prx_chainiksolvervel_pinv_nso::CartToJntSDLS(const KDL::JntArray& q_in, const KDL::Twist& v_in, KDL::JntArray& qdot_out)
            {
                // PRX_PRINT("================ START Cart to Joint (SDLS) =====================", PRX_TEXT_BLUE);
                //Compute again Mr. Jacobean                
                jnt2jac.JntToJac(q_in,jac);
                
                //Copy over the Jacobian
                J = jac.data;

                //Compute the pseudoinverse (with no damping)
                int svdResult = compute_post_pseudoinverse( J_Q_cross, J, VectorXd::Ones(9), 0 );
                //If the SVD didn't work out, abort
                if (0 != svdResult)
                {
                    qdot_out.data.setZero() ;
                    return svdResult;
                }
                //Copy over our end-effector error
                for( unsigned i = 0; i < 6; ++i)
                {
                    tmp(i) = v_in(i);
                }

                //If we are close to our goal, just turn off damping and return what we normally would
                if( v_in.vel.Norm() < 0.01 )
                {
                    qdot_out.data = J_Q_cross * tmp.head(6);
                    return svdResult;
                }
                
                //Otherwise, compute the SDLS damping values
                double gamma_max = 0.78; //Approx pi/12
                Eigen::VectorXd gamma( VectorXd::Zero(6) );
                compute_SDLS_damping( gamma, gamma_max );
                
                //Next, we clamp based on the individual damping factors
                Eigen::MatrixXd phi_i (MatrixXd::Zero( nj, 1 ));
                Eigen::VectorXd phi_i_vec (VectorXd::Zero( nj ));
                // PRX_PRINT("Checking for vector clampage...", PRX_TEXT_MAGENTA);
                for( unsigned i=0; i<6; ++i )
                {
                    // PRX_STATUS("[" << i << "]", PRX_TEXT_LIGHTGRAY);
                    mt_v_m( phi_i, Sinv, U, tmp.head(6), V, 0, 5 );
                    //Convert the matrix result into a vector
                    for( unsigned k=0; k<nj; ++k )
                    {
                        phi_i_vec(k) = phi_i.col(0)(k);
                    }
                    clamp_max_abs( phi_i_vec, gamma(i) );
                    //Go ahead and add this to the resulting q^dot
                    qdot_out.data += phi_i_vec;
                }
                
                // PRX_PRINT("Checking for GLOBAL clampage...", PRX_TEXT_MAGENTA);
                //Then, the computed q^dot is the clamping over the sum of the PHI vectors
                clamp_max_abs( qdot_out.data, gamma_max );

                // PRX_PRINT("Qdot Damping: " << qdot_out.data.norm(), PRX_TEXT_BLUE);
                // PRX_PRINT("qdot: \n" << qdot_out.data, PRX_TEXT_LIGHTGRAY);
                // qdot_out.data.normalize();
                // PRX_PRINT("qdot_norm: \n" << qdot_out.data, PRX_TEXT_CYAN);

                //Return the SVD results
                return svdResult;
            }


            int prx_chainiksolvervel_pinv_nso::CartToJntSDLS_SVF(const KDL::JntArray& q_in, const KDL::Twist& v_in, KDL::JntArray& qdot_out)
            {
                // PRX_PRINT("================ START Cart to Joint (SDLS + SVF) =====================", PRX_TEXT_BLUE);
                // PRX_PRINT("Twist: " << v_in, PRX_TEXT_LIGHTGRAY);
                
                jnt2jac.JntToJac(q_in,jac);

                double curve = 10;
                double sigma_0 = 0.005;

                int svdResult = svd_eigen_HH(jac.data,U,S,V,tmp,maxiter);
                if (0 != svdResult)
                {
                    qdot_out.data.setZero() ;
                    return svdResult;
                }

                if( S(5) < PRX_ZERO_CHECK )
                {
                    PRX_FATAL_S("SVD decomposition gave something not full-rank!");
                }

                unsigned int i;

                //Get our e vector copied over
                for (i = 0; i < 6; ++i) 
                {
                    Sinv(i) = 1.0 / ( ( (S(i)*S(i)*S(i)) + (curve*S(i)*S(i)) + (2*S(i)) + (2*sigma_0) ) /
                                      ( (S(i)*S(i)) + (curve*S(i)) + 2 ) );
                }
                for (i = 0; i < 6; ++i)
                {
                    tmp(i) = v_in(i);
                }

                //If we are close to our goal, just turn off damping?
                if( v_in.vel.Norm() < 0.01 )
                {
                    qdot_out.data = V * Sinv.asDiagonal() * U.transpose() * tmp.head(6);
                    // PRX_PRINT("Qdot Normal: " << qdot_out.data.norm(), PRX_TEXT_GREEN);
                    return svdResult;
                }
                
                //First, compute the SDLS damping values
                double gamma_max = 0.78; //0.262;//Approx pi/12
                Eigen::VectorXd gamma( VectorXd::Zero(6) );
                compute_SDLS_damping( gamma, gamma_max );
                
                // PRX_PRINT("Damping values:", PRX_TEXT_BROWN);
                // PRX_PRINT("gamma: \n" << gamma, PRX_TEXT_LIGHTGRAY);
                
                //Next, we clamp based on the individual damping factors
                Eigen::MatrixXd phi_i (MatrixXd::Zero( nj, 1 ));
                Eigen::VectorXd phi_i_vec (VectorXd::Zero( nj ));
                // PRX_PRINT("Checking for vector clampage...", PRX_TEXT_MAGENTA);
                for( unsigned i=0; i<6; ++i )
                {
                    // PRX_STATUS("[" << i << "]", PRX_TEXT_LIGHTGRAY);
                    mt_v_m( phi_i, Sinv, U, tmp.head(6), V, 0, 5 );
                    //Convert the matrix result into a vector
                    for( unsigned k=0; k<nj; ++k )
                    {
                        phi_i_vec(k) = phi_i.col(0)(k);
                    }
                    clamp_max_abs( phi_i_vec, gamma(i) );
                    //Go ahead and add this to the resulting q^dot
                    qdot_out.data += phi_i_vec;
                }
                
                // PRX_PRINT("Checking for GLOBAL clampage...", PRX_TEXT_MAGENTA);
                //Then, the computed q^dot is the clamping over the sum of the PHI vectors
                clamp_max_abs( qdot_out.data, gamma_max );

                // PRX_PRINT("Qdot Damping: " << qdot_out.data.norm(), PRX_TEXT_BLUE);
                // PRX_PRINT("qdot: \n" << qdot_out.data, PRX_TEXT_LIGHTGRAY);
                // qdot_out.data.normalize();
                // PRX_PRINT("qdot_norm: \n" << qdot_out.data, PRX_TEXT_CYAN);

                //This pre-empts any NULL-space stuff
                return svdResult;
            }

            void prx_chainiksolvervel_pinv_nso::compute_SDLS_damping( Eigen::VectorXd& gamma, double gamma_max )
            {
                Eigen::VectorXd M(VectorXd::Zero(6));
                Eigen::VectorXd N(VectorXd::Zero(6));
                
                //First, compute N_i
                for(int i = 0; i < 6; ++i)
                {
                    //In our case, we have a single end-effector, so k=1
                    N(i) = U.col(i).norm();
                }
                
                //Then, compute M_i
                Eigen::VectorXd col_j( Eigen::VectorXd::Zero(6) );
                for( int i=0; i<6; ++i )
                {
                    for( int j=0; j<nj; ++j )
                    {
                        for( unsigned t=0; t<6; ++t )
                        {
                            col_j( t ) = jac.getColumn(j)(t);
                        }
                        M(i) += fabs(V.row(j)(i)) * col_j.norm();
                    }
                    //Note : S(i) should never be so small that this returns zero!
                    M(i) *= (fabs(S(i)) < eps ? 0 : 1.0/S(i));
                }
                
                //Finally, get the final gamma values (damping) for each DoF
                for( int i=0; i<6; ++i )
                {
                    gamma(i) = PRX_MINIMUM( 1, N(i)/M(i) ) * gamma_max;
                }
            }

            void prx_chainiksolvervel_pinv_nso::compute_continuous_damping( Eigen::VectorXd& gamma, double gamma_max )
            {
                Eigen::VectorXd M(VectorXd::Zero(6));
                
                //Then, compute M_i
                Eigen::VectorXd col_j( Eigen::VectorXd::Zero(6) );
                for( int i=0; i<6; ++i )
                {
                    for( int j=0; j<nj; ++j )
                    {
                        M(i) += fabs( V(j,i) ) * (J.col(j)).norm();
                    }
                    M(i) *= (1.0/Shat(i));
                }
                
                //Finally, get the final gamma values (damping) for each DoF
                for( int i=0; i<6; ++i )
                {
                    gamma(i) = PRX_MINIMUM( 1, 1.0/M(i) ) * gamma_max;
                }
            }
            
            int prx_chainiksolvervel_pinv_nso::CartToJntClamping(const KDL::JntArray& q_in, const std::vector< bounds_t* >& q_bounds, const KDL::Twist& v_in, KDL::JntArray& qdot_out)
            {
                unsigned int i;
                double lambda = 1e-3;

                //Compute Jacobian
                jnt2jac.JntToJac(q_in,jac);

                //Initially, none of the joints are clamped
                Eigen::VectorXd proj_j0( VectorXd::Ones(nj) );
                
                //Start off in a "clamping" state to make sure the loop starts
                bool clamping_detected = true;
                int svdResult;
                
                while( clamping_detected && proj_j0.count() >= 6 )
                {
                    //Compute the SVD of the Jacobian
                    svdResult = svd_eigen_HH(jac.data,U,S,V,tmp,maxiter);
                    
                    //If the SVD does not work out
                    if (0 != svdResult)
                    {
                        //Report the failure
                        qdot_out.data.setZero() ;
                        return svdResult;
                    }

                    //Compute the Damped Pseudoinverse
                    for (i = 0; i < nj; ++i) 
                    {
                        SinvL(i) = S(i)/(S(i)*S(i)+lambda*lambda);
                    }
                    //Copy over the input velocity (unfortunately have to do this because SVD trashes it?)
                    for (i = 0; i < 6; ++i) 
                    {
                        tmp(i) = v_in(i);
                    }

                    //Compute q^dot 
                    qdot_out.data = V * SinvL.asDiagonal() * U.transpose() * tmp.head(6);                    

                    //Now, we assume no clamping has happened
                    clamping_detected = false;

                    //Then, for each DoF
                    for( i=0; i<nj; ++i )
                    {
                        //If that DoF is not yet clamped
                        if( proj_j0(i) != 0 )
                        {
                            //Compute the theta update for this DoF (Quick, crummy approximation)
                            double theta_up = q_in(i) + simulation::simulation_step * qdot_out(i);
                            double DTCi = 0;
                            //If the update is not within bounds
                            if( !q_bounds[i]->is_valid( theta_up ) )
                            {
                                // PRX_PRINT("[" << i << "] has clampage!", PRX_TEXT_BROWN);
                                
                                //Need to get Delta Theta_Ci (The clamping variation)
                                if( q_bounds[i]->get_lower_bound() > theta_up )
                                {
                                    DTCi = theta_up - q_bounds[i]->get_lower_bound();
                                }
                                else if( q_bounds[i]->get_upper_bound() < theta_up )
                                {
                                    DTCi = theta_up - q_bounds[i]->get_upper_bound();
                                }
                                
                                //We have detected a clamping
                                clamping_detected = true;
                                //Need to extract J_i as a vector
                                Eigen::VectorXd J_i_vec (VectorXd::Zero(6));
                                KDL::Twist J_i = jac.getColumn(i);
                                for( unsigned t=0; t<6; ++t )
                                {
                                    J_i_vec( t ) = J_i( t );
                                }
                                //Then, tmp -= J_i DEL Theta_Ci
                                tmp -= J_i_vec * DTCi;
                                //Set theta_i to the breached limit  (I'm not sure I do have to explicitly do this)
                                
                                //Zero out J_i
                                jac.setColumn( i, KDL::Twist::Zero() );
                                //Zero out the Jacobian_naught projection at i
                                proj_j0(i) = 0;
                            }
                        }
                    }
                }
                return svdResult;
            }

            //I don't even remember exactly what this one was
            int prx_chainiksolvervel_pinv_nso::CartToJnt_IDF(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out)
            {
                //Go get Mr. Jacobean
                jnt2jac.JntToJac(q_in,jac);

                //Perform SVD
                int svdResult = svd_eigen_HH(jac.data,U,S,V,tmp,maxiter);
                if (0 != svdResult)
                {
                    //Report out if SVD decomp does not work
                    qdot_out.data.setZero() ;
                    return svdResult;
                }
                //Always be checkin' for full rankin'
                if( S(5) < PRX_ZERO_CHECK )
                {
                    PRX_FATAL_S("SVD decomposition gave something not full-rank!");
                }

                // PRX_PRINT("Decomposition complete:", PRX_TEXT_BLUE);
                // PRX_PRINT("S: \n" << S, PRX_TEXT_LIGHTGRAY);

                //Some parameters
                nj = q_in.data.size();
                unsigned int i;
                double lambda = 1e-3;
                
                // PRX_PRINT("Distance: " << v_in.vel.Norm(), PRX_TEXT_LIGHTGRAY);
                
                //We need to compute a substitute for lamda^2 (If sigma_m is sufficiently large, we will use no lambda damping)
                double lambda_sq = 0;
                if( S(5) < eps )
                {
                    double s = ( S(5)/eps );
                    lambda_sq = ( lambda * lambda ) * ( 1.0 - ( s * s ) );
                }
                double beta_sq = lambda_sq * 1e-4;
                
                //For the first m-1 singular values
                for (i = 0; i < 5; ++i)
                {
                    //We perform damping with beta squared instead of lambda
                    SinvL(i) = S(i)/(S(i)*S(i)+beta_sq);
                }
                //For the mth singular value, we use beta squared and lambda squared
                SinvL(5) = S(5)/( S(5)*S(5) + beta_sq + lambda_sq );
                //All of the singular values at 7 (index 6) and above should be 0 already...
                for( i=6; i<nj; ++i )
                {
                    SinvL(i) = 0;
                }
                
                // PRX_PRINT("Computed Pseudoinverse:", PRX_TEXT_MAGENTA);
                // PRX_PRINT("J^o: \n" << SinvL, PRX_TEXT_LIGHTGRAY);
                
                //Copy over x^dot
                for (i = 0; i < 6; ++i) 
                {
                    tmp(i) = v_in(i);
                }

                //This is the equivalent computation using the vector cross products method
                // MatrixXd VSiUt(MatrixXd::Zero(nj,6));
                // vector_cross_product( VSiUt, SinvL, V, U, 0, 5 );
                // qdot_out.data = VSiUt * tmp.head( 6 );
                
                qdot_out.data = V * SinvL.asDiagonal() * U.transpose() * tmp.head(6);
                
                // PRX_PRINT("Results in a q^dot", PRX_TEXT_GREEN);
                // PRX_PRINT("q^dot:\n" << qdot_out.data, PRX_TEXT_LIGHTGRAY);

                //This pre-empts any NULL-space stuff
                return svdResult;
            }

            //Right now, this corresponds to Eq. 34 of the Colome paper
            int prx_chainiksolvervel_pinv_nso::CartToJnt_continuous(const JntArray& q_in, const std::vector< bounds_t* >& q_bounds, const Twist& v_in, JntArray& qdot_out)
            {
                //Some parameters and such
                double lambda_jl = 0.2; //Joint limit weighting factor
                Eigen::VectorXd& q = tmp2;
                
                //Alright, first thing, we still need to compute Mr. Jacobean around this state
                jnt2jac.JntToJac( q_in, jac );
                J = jac.data;

                //We need to compute H based on how close everything is to the joint limits
                int H_rank = compute_continuous_activation_matrix( q_in, q_bounds );

                // PRX_STATUS(v_in << " d[" << v_in.vel.Norm() << "] H(" << H_rank << ")   ", PRX_TEXT_LIGHTGRAY);
                
                //Now, we need the monster J_2^{(I_m - H)\oplus}
                compute_continuous_post_pseudoinverse_colome( J_inv, J, VectorXd::Ones(nj) - H );
                // compute_continuous_post_pseudoinverse_colome( J_inv, J, VectorXd::Ones(nj) );

                //We also need to convert current position from KDL::JointArray to an Eigen Vector
                for(unsigned i = 0; i < 6; ++i) 
                {
                    e_dot(i) = v_in(i);
                }
                //Also interestingly need to convert current state into an Eigen Vector
                for( unsigned i=0; i<nj; ++i )
                {
                    q(i) = q_in(i);
                }
                
                //DEBUG: What happens when we try the absolutely most naive thing here?
                // qdot_out.data = J_inv * e_dot;
                
                //Then, compute our actual delta theta
                qdot_out.data = -lambda_jl * H.asDiagonal() * q;
                
                // PRX_PRINT("Current Angles: \n" << q, PRX_TEXT_LIGHTGRAY);
                // PRX_PRINT("Activations: \n" << H, PRX_TEXT_BLUE);
                // PRX_PRINT("J_inv: \n" << J_inv, PRX_TEXT_RED);
                // PRX_PRINT("Joint correction: \n" << qdot_out.data, PRX_TEXT_CYAN);
                // PRX_PRINT("RHS: \n" << J_inv * ( e_dot - J * qdot_out.data ), PRX_TEXT_BROWN);

                qdot_out.data += J_inv * ( e_dot - J * qdot_out.data );

                // PRX_PRINT("Final q^dot: \n" << qdot_out.data, PRX_TEXT_LIGHTGRAY);
                // PRX_FATAL_S("Check?");
                
                //If the rank of H has become sufficiently large, it is unlikely we will be able to get out of this, so report it
                if( H_rank >= critical_H_rank )
                {
                    return SOLVER_ERR_LARGE_H_RANK;
                }
                return 0;
            }

            int prx_chainiksolvervel_pinv_nso::CartToJnt_continuous_SD_SVF(const JntArray& q_in, const std::vector< bounds_t* >& q_bounds, const Twist& v_in, JntArray& qdot_out)
            {
                //Some parameters and such
                double lambda = 0.1;
                double lambda_jl = 0.2; //Joint limit weighting factor
                Eigen::VectorXd& q = tmp2;
                
                //Alright, first thing, we still need to compute Mr. Jacobean around this state
                jnt2jac.JntToJac( q_in, jac );
                J = jac.data;

                //We need to compute H based on how close everything is to the joint limits
                int H_rank = compute_continuous_activation_matrix( q_in, q_bounds );

                PRX_STATUS(v_in << " d[" << v_in.vel.Norm() << "] H(" << H_rank << ")   ", PRX_TEXT_LIGHTGRAY);
                
                //Now, we need the monster J_2^{(I_m - H)\oplus}
                compute_continuous_post_pseudoinverse_colome( J_inv, J, VectorXd::Ones(nj) - H );

                //We also need to convert current position from KDL::JointArray to an Eigen Vector
                for(unsigned i = 0; i < 6; ++i) 
                {
                    e_dot(i) = v_in(i);
                }
                //Also interestingly need to convert current state into an Eigen Vector
                for( unsigned i=0; i<nj; ++i )
                {
                    q(i) = q_in(i);
                }
                
                //We need to perform the SVD of the continuous pseudoinverse
                int svdResult = svd_eigen_HH( J_inv, Uhat, Shat, Vhat, tmp, maxiter);

                //We need to compute the selective damping parameters for this setup
                double gamma_max = 0.78; //0.262; //Approx pi/12
                Eigen::VectorXd gamma( VectorXd::Zero(6) );
                compute_continuous_damping( gamma, gamma_max );
                
                qdot_out.data.setZero();
                //Now, for the computation
                for( unsigned i=0; i<6; ++i )
                {
                    //Compute w_s
                    tmp2 = (1.0/Shat(i)) * Vhat.col(i) * (Uhat.col(i).transpose() * e_dot);
                    //Clamp it
                    clamp_max_abs( tmp2, gamma(i) );
                    
                    //Accumulate this result (45)
                    qdot_out.data += tmp2;
                }
                
                //Then, add the front end before the summation (45)
                qdot_out.data += (I - J_inv * J) * H.asDiagonal() * (-lambda * q);
                
                //Finally, clamp over this result
                clamp_max_abs( qdot_out.data, gamma_max );
                
                //If the rank of H has become sufficiently large, it is unlikely we will be able to get out of this, so report it
                if( H_rank >= critical_H_rank )
                {
                    return SOLVER_ERR_LARGE_H_RANK;
                }
                return 0;//svdResult;
            }

            
            //This corresponds to Equation 52 in the Khatib/Mansard paper
            int prx_chainiksolvervel_pinv_nso::CartToJnt_Khatib_52(const JntArray& q_in, const std::vector< bounds_t* >& q_bounds, const Twist& v_in, JntArray& qdot_out)
            {
                //I have no idead if these are at all accurate
                double lambda = 0.1;
                double lambda_jl = 0.05;
                Eigen::VectorXd& q = tmp2;
                
                // PRX_PRINT("=================== Begin CartToJnt (Khatib 52) ======================", PRX_TEXT_GREEN);
                
                //Alright, first thing, we still need to, as always, compute Mr. Jacobean around this state
                jnt2jac.JntToJac( q_in, jac );
                //We will need to convert the Jacobian into an Eigen Matrix
                J = jac.data;
                
                //Also, let us get the appropriate activation matrix
                int H_rank = compute_continuous_activation_matrix( q_in, q_bounds );
                
                // PRX_STATUS(v_in << " d[" << v_in.vel.Norm() << "] H(" << H_rank << ")   ", PRX_TEXT_LIGHTGRAY);
                
                //Next, we need Delta_JL
                Eigen::VectorXd delta_jl (VectorXd::Zero(nj));
                //fill in the ranges
                for( unsigned i=0; i<nj; ++i )
                {
                    delta_jl(i) = q_bounds[i]->get_upper_bound() - q_bounds[i]->get_lower_bound();
                }
                
                //We also need to convert current position from KDL::JointArray to an Eigen Vector
                for(unsigned i = 0; i < 6; ++i) 
                {
                    e_dot(i) = v_in(i);
                }
                //Also interestingly need to convert current state into an Eigen Vector
                for( unsigned i=0; i<nj; ++i )
                {
                    q(i) = q_in(i);
                }

                //Compute the post-psuedoinverse of our jacobian
                int svdResult = compute_post_pseudoinverse( J_Q_cross, J, VectorXd::Ones(nj) - lambda_jl * H );
                //Then, we simply follow Eq.52 to implement the control law
                qdot_out.data = delta_jl.asDiagonal() * q;
                qdot_out.data = (-lambda_jl * H.asDiagonal() * qdot_out.data);
                // PRX_PRINT("Current Angles: \n" << q, PRX_TEXT_LIGHTGRAY);
                // PRX_PRINT("Activations: \n" << H, PRX_TEXT_BLUE);
                // PRX_PRINT("Joint correction: \n" << qdot_out.data, PRX_TEXT_CYAN);
                // PRX_PRINT("RHS: \n" << (J_Q_cross * ( lambda * e_dot - J * qdot_out.data ) ), PRX_TEXT_BROWN);
                qdot_out.data += (J_Q_cross * ( lambda * e_dot - J * qdot_out.data ) );
                // PRX_PRINT("Final q^dot: \n" << qdot_out.data, PRX_TEXT_LIGHTGRAY);
                // PRX_FATAL_S("Check?");
                
                return svdResult;
            }

            //This corresponds to Equation 57 in the Khatib/Mansard paper
            int prx_chainiksolvervel_pinv_nso::CartToJnt_Khatib_57(const JntArray& q_in, const std::vector< bounds_t* >& q_bounds, const Twist& v_in, JntArray& qdot_out)
            {
                //I have no idead if these are at all accurate
                double lambda = 0.1;
                double lambda_jl = 0.05;
                Eigen::VectorXd& q = tmp2;
                
                //Alright, first thing, we still need to, as always, compute Mr. Jacobean around this state
                jnt2jac.JntToJac( q_in, jac );
                //We will need to convert the Jacobian into an Eigen Matrix
                J = jac.data;
                
                //Also, let us get the appropriate activation matrix
                int H_rank = compute_continuous_activation_matrix( q_in, q_bounds );
                
                PRX_STATUS(v_in << " d[" << v_in.vel.Norm() << "] H(" << H_rank << ")   ", PRX_TEXT_LIGHTGRAY);
                
                //We also need to convert current position from KDL::JointArray to an Eigen Vector
                for(unsigned i = 0; i < 6; ++i) 
                {
                    e_dot(i) = v_in(i);
                }
                //Also interestingly need to convert current state into an Eigen Vector
                for( unsigned i=0; i<nj; ++i )
                {
                    q(i) = q_in(i);
                }

                //We need to compute J_jl^thingy H
                Eigen::MatrixXd J_jl_H (MatrixXd::Zero(nj,nj));
                compute_continuous_pre_pseudoinverse_colome( J_jl_H, I, H );
                
                //Alright, then, just need to compute this monstrosity (This is probably not the right thing to do?)
                Eigen::MatrixXd J_r_P_th_jl_th (MatrixXd::Zero(nj,6));
                compute_continuous_post_pseudoinverse_colome( J_r_P_th_jl_th, J, VectorXd::Ones(nj) - H );
                
                //And after that, we just follow Eq. 57
                qdot_out.data = -lambda_jl * J_jl_H * q;

                // PRX_PRINT("Current Angles: \n" << q, PRX_TEXT_LIGHTGRAY);
                // PRX_PRINT("Activations: \n" << H, PRX_TEXT_BLUE);
                // PRX_PRINT("J_jl_H: \n" << J_jl_H, PRX_TEXT_RED);
                // PRX_PRINT("Joint correction: \n" << qdot_out.data, PRX_TEXT_CYAN);
                // PRX_PRINT("RHS: \n" << J_r_P_th_jl_th * ( lambda * e_dot - J * qdot_out.data ), PRX_TEXT_BROWN);

                qdot_out.data += J_r_P_th_jl_th * ( lambda * e_dot - J * qdot_out.data );

                // PRX_PRINT("Final q^dot: \n" << qdot_out.data, PRX_TEXT_LIGHTGRAY);
                // PRX_FATAL_S("Check?");
                
                return 0;
            }

            double prx_chainiksolvervel_pinv_nso::measure()
            {
                double ret = elapsed;
                elapsed = 0;
                return ret;
            }

            void prx_chainiksolvervel_pinv_nso::compute_continuous_post_pseudoinverse_colome( Eigen::MatrixXd& result, const Eigen::MatrixXd& J_in, const Eigen::MatrixXd& H_in )
            {
                sys_clock_t pclock;
                pclock.reset();
                
                result.setZero();
                J_Q_cross.setZero();
                double product = 1;
                
                //For each set in the power set
                for( unsigned set=0; set<P.size(); ++set )
                {
                    //Compute the product term
                    product = 1;
                    //Grab the current H naught
                    const Eigen::VectorXd & H_0 = P[set];
                    
                    for( unsigned i=0; i<nj && product != 0; ++i )
                    {
                        if( H_0(i) != 0 )
                        {
                            product *= H_in(i);
                        }
                        else
                        {
                            product *= 1 - H_in(i);
                        }
                    }
                    //If we have a nonzero product
                    if( product != 0 )
                    {
                        //We need to compute J_Q's pseudoinverse, where J_Q = J H_0; H_0 = P[set].diag
                        int inner_inverse_result = compute_post_pseudoinverse( J_Q_cross, J_in, H_0 );
                        
                        //If the pseudoinverse computed correctly, add it in (TODO: What if it is not computed correctly?)
                        if( inner_inverse_result == 0 )
                        {
                            //Add in the product times the result of the pseudoinverse
                            result += product * J_Q_cross;
                        }
                    }
                }
                elapsed += pclock.measure();
                // PRX_PRINT("[" << pclock.measure() << "]", PRX_TEXT_CYAN);
            }

            void prx_chainiksolvervel_pinv_nso::compute_continuous_pre_pseudoinverse_colome( Eigen::MatrixXd& result, const Eigen::MatrixXd& J_in, const Eigen::MatrixXd& H_in )
            {
                result.setZero();
                J_Q_cross.setZero();
                double product = 1;
                
                Eigen::VectorXd gamma( VectorXd::Zero(6) );
                
                //For each set in the power set
                for( unsigned set=0; set<P.size(); ++set )
                {
                    //Compute the product term
                    product = 1;
                    //Grab the current H naught
                    const Eigen::VectorXd & H_0 = P[set];
                    
                    for( unsigned i=0; i<nj && product != 0; ++i )
                    {
                        if( H_0(i) != 0 )
                        {
                            product *= H_in(i);
                        }
                        else
                        {
                            product *= 1 - H_in(i);
                        }
                    }
                    //If we have a nonzero product
                    if( product != 0 )
                    {
                        //We need to compute J_Q's pseudoinverse, where J_Q = J H_0; H_0 = P[set].diag
                        int inner_inverse_result = compute_pre_pseudoinverse( J_Q_cross, J_in, H_0 );
                        
                        //If the pseudoinverse computed correctly, add it in (TODO: What if it is not computed correctly?)
                        if( inner_inverse_result == 0 )
                        {
                            //Add in the product times the result of the pseudoinverse
                            result += product * J_Q_cross;
                        }
                    }
                }
            }
            
            void prx_chainiksolvervel_pinv_nso::compute_continuous_post_pseudoinverse_mansard( Eigen::MatrixXd& result, const Eigen::MatrixXd& J_in, const Eigen::MatrixXd& H_in )
            {
                result.setZero();
                J_Q_cross.setZero();
                double product = 1;
                
                //Clear out all the previous coupling matrices
                XP.resize(0);
                
                //For each set in the power set
                for( unsigned set=0; set<P.size(); ++set )
                {
                    //Compute the product term
                    product = 1;
                    //Grab the current H naught
                    const Eigen::VectorXd & H_0 = P[set];

                    //Need to get the product for everything which is active here.                    
                    for( unsigned i=0; i<nj && product != 0; ++i )
                    {
                        if( H_0(i) != 0 )
                        {
                            product *= H_in(i);
                        }
                    }
                    //Compute the coupling matrix X_P
                    compute_post_coupling_matrix( J_in, H_0, set );
                    //If we have a nonzero product
                    if( product != 0 )
                    {
                        //And do the multiplication
                        result += product * XP[set];
                    }
                }
            }
            
            void prx_chainiksolvervel_pinv_nso::compute_continuous_pre_pseudoinverse_mansard( Eigen::MatrixXd& result, const Eigen::MatrixXd& J_in, const Eigen::MatrixXd& H_in )
            {
                result.setZero();
                J_Q_cross.setZero();
                double product = 1;
                
                //Clear out all the previous coupling matrices
                XP.resize(0);
                
                //For each set in the power set
                for( unsigned set=0; set<P.size(); ++set )
                {
                    //Compute the product term
                    product = 1;
                    //Grab the current H naught
                    const Eigen::VectorXd & H_0 = P[set];

                    //Need to get the product for everything which is active here.                    
                    for( unsigned i=0; i<nj && product != 0; ++i )
                    {
                        if( H_0(i) != 0 )
                        {
                            product *= H_in(i);
                        }
                    }
                    //Compute the coupling matrix X_P
                    compute_pre_coupling_matrix( J_in, H_0, set );
                    //If we have a nonzero product
                    if( product != 0 )
                    {
                        //And do the multiplication
                        result += product * XP[set];
                    }
                }
            }
            
            int prx_chainiksolvervel_pinv_nso::compute_post_pseudoinverse( Eigen::MatrixXd & result, const Eigen::MatrixXd & in_jac, const Eigen::VectorXd & H_0, double lambda )
            {
                //we will sometimes have to perform SVD here, so we'll need something to return
                int svdResult = 0;
                
                //Need to create a copy of J
                Eigen::MatrixXd J_Q = in_jac;
                //Zero out our result
                result.setZero();
                //Then, for each element in H_0 which is zero
                for( unsigned i=0; i<nj; ++i )
                {
                    if( H_0(i) == 0 )
                    {
                        //We Need to zero out that column of the Jacobian
                        J_Q.col(i).setZero();
                    }
                    else if( H_0(i) < 1 )
                    {
                        J_Q.col(i) *= H_0(i);
                    }
                }
                
                //Perform SVD decomposition of J_Q
                svdResult = svd_eigen_HH( J_Q, U, S, V, tmp, maxiter);
                if( svdResult != 0 )
                {
                    return svdResult;
                }

                //Setup S inverse
                for(unsigned i = 0; i < 6; ++i)
                {
                    Sinv(i) = S(i) / ( S(i)*S(i) + lambda*lambda );
                }
                
                result = V * Sinv.asDiagonal() * U.transpose();
                
                return svdResult;
            }

            int prx_chainiksolvervel_pinv_nso::compute_pre_pseudoinverse( Eigen::MatrixXd & result, const Eigen::MatrixXd & in_jac, const Eigen::VectorXd & H_0, double lambda )
            {
                //we will sometimes have to perform SVD here, so we'll need something to return
                int svdResult = 0;
                
                //Need to create a copy of J
                Eigen::MatrixXd J_Q = in_jac;
                //Zero out our result
                result.setZero();
                //Then, for each element in H_0 which is zero
                for( unsigned i=0; i<nj; ++i )
                {
                    if( H_0(i) == 0 )
                    {
                        //We Need to zero out that row of the Jacobian
                        J_Q.row(i).setZero();
                    }
                    else if( H_0(i) < 1 )
                    {
                        J_Q.row(i) *= H_0(i);
                    }
                }
                
                //Perform SVD decomposition of J_Q
                svdResult = svd_eigen_HH( J_Q, U_nj, S_nj, V, tmp, maxiter);
                if( svdResult != 0 )
                {
                    return svdResult;
                }

                //Setup S inverse
                for(unsigned i = 0; i<nj; ++i)
                {
                    Sinv(i) = S_nj(i) / ( S_nj(i)*S_nj(i) + lambda*lambda );
                }
                
                result = V * Sinv.asDiagonal() * U_nj.transpose();
                
                return svdResult;                
            }
            
            int prx_chainiksolvervel_pinv_nso::compute_post_pseudoinverse_SVF( Eigen::MatrixXd& result, const Eigen::MatrixXd & in_jac, const Eigen::VectorXd & H_0, double lambda )
            {
                //Some parameters
                double curve = 10;
                double sigma_0 = 0.005;

                //Need to create a copy of J
                Eigen::MatrixXd J_Q = in_jac;
                //Zero out our result
                result.setZero();
                //Then, for each element in H_0 which is zero
                for( unsigned i=0; i<nj; ++i )
                {
                    if( H_0(i) == 0 )
                    {
                        //We Need to zero out that column of the Jacobian
                        J_Q.col(i).setZero();
                    }
                    else if( H_0(i) < 1 )
                    {
                        J_Q.col(i) *= H_0(i);
                    }
                }
                
                //Perform SVD decomposition of J_Q
                int svdResult = svd_eigen_HH( J_Q, U, S, V, tmp, maxiter);
                if( svdResult != 0 )
                {
                    return svdResult;
                }

                //Setup S inverse
                for(unsigned i = 0; i < 6; ++i)
                {
                    Sinv(i) = 1.0 / ( ( (S(i)*S(i)*S(i)) + (curve*S(i)*S(i)) + (2*S(i)) + (2*sigma_0) ) /
                                      ( (S(i)*S(i)) + (curve*S(i)) + 2 ) );
                }
                
                //TODO: HOW IN THE WORLD CAN WE DO SDLS IN THIS CASE?
                // for( unsigned i=0; i<6; ++i )
                // {
                //     Eigen::MatrixXd phi_i( MatrixXd::Zero(nj,6) );
                //     phi_i = Sinv(i) * V.col(i) * (U.col(i)).transpose();

                //     clamp_max_abs( result.col(i), gamma(i) );
                //     //Go ahead and add this to the resulting q^dot
                //     result += phi_i_vec;
                // }
                
                result = V * Sinv.asDiagonal() * U.transpose();
                
                return svdResult;
            }
            
            int prx_chainiksolvervel_pinv_nso::compute_pre_pseudoinverse_SVF( Eigen::MatrixXd& result, const Eigen::MatrixXd & in_jac, const Eigen::VectorXd & H_0, double lambda )
            {
                //Some parameters
                double curve = 10;
                double sigma_0 = 0.005;
                
                //Need to create a copy of J
                Eigen::MatrixXd J_Q = in_jac;
                //Zero out our result
                result.setZero();
                //Then, for each element in H_0 which is zero
                for( unsigned i=0; i<nj; ++i )
                {
                    if( H_0(i) == 0 )
                    {
                        //We Need to zero out that row of the Jacobian
                        J_Q.row(i).setZero();
                    }
                    else if( H_0(i) < 1 )
                    {
                        J_Q.row(i) *= H_0(i);
                    }
                }
                
                //Perform SVD decomposition of J_Q
                int svdResult = svd_eigen_HH( J_Q, U_nj, S_nj, V, tmp, maxiter);
                if( svdResult != 0 )
                {
                    return svdResult;
                }

                //Setup S inverse
                for(unsigned i = 0; i<nj; ++i)
                {
                    Sinv(i) = 1.0 / ( ( (S(i)*S(i)*S(i)) + (curve*S(i)*S(i)) + (2*S(i)) + (2*sigma_0) ) /
                                      ( (S(i)*S(i)) + (curve*S(i)) + 2 ) );
                }
                
                result = V * Sinv.asDiagonal() * U_nj.transpose();
                
                return svdResult;                
            }
            
            int prx_chainiksolvervel_pinv_nso::compute_post_coupling_matrix( const Eigen::MatrixXd & in_jac, const Eigen::VectorXd & H_0, unsigned index )
            {
                //We're going to need a new matrix
                XP.push_back( MatrixXd::Zero(nj, 6) );
                //Let's just have an alias for this thing to use
                Eigen::MatrixXd& X = XP.back();
                
                //Begin by computing (JH)+
                int svdResult = compute_post_pseudoinverse( X, in_jac, H_0 );
                
                //Now, subtract away all of the subset results
                for( unsigned i=1; i<index; ++i )
                {
                    if( is_subset( i, index ) )
                    {
                        X -= XP[i];
                    }
                }
                return svdResult;
            }

            //In this case, we actually don't need to do pseudoinverses, because our Jacobian is sqare
            int prx_chainiksolvervel_pinv_nso::compute_pre_coupling_matrix( const Eigen::MatrixXd & in_jac, const Eigen::VectorXd & H_0, unsigned index )
            {
                // PRX_PRINT("Computing Coupling Matrix [" << index << "]", PRX_TEXT_GREEN);
                //We're going to need a new matrix
                XP.push_back( MatrixXd::Zero(nj, nj) );
                //Let's just have an alias for this thing to use
                Eigen::MatrixXd& X = XP.back();
                
                //Begin by computing (JH)-1
                for( unsigned i=0; i<nj; ++i )
                {
                    X(i, i) = H_0(i) * in_jac(i, i);
                    if( X(i, i) < 0 )
                    {
                        X(i, i) = 1 / X(i, i);
                    }
                }
                
                // int svdResult = compute_pre_pseudoinverse( X, in_jac, H_0 );
                // PRX_PRINT("Computed Initial XP [" << index << "] \n" << X, PRX_TEXT_LIGHTGRAY);
                //Now, subtract away all of the subset results
                for( unsigned i=1; i<index; ++i )
                {
                    if( is_subset( i, index ) )
                    {
                        X -= XP[i];
                    }
                }
                
                // PRX_PRINT("Computed Coupling Matrix [" << index << "] \n" << X, PRX_TEXT_LIGHTGRAY);
                
                return 0;
            }

            int prx_chainiksolvervel_pinv_nso::compute_continuous_activation_matrix( const JntArray& q_in, const std::vector< bounds_t* >& q_bounds )
            {
                //Threshold parameter for the activation
                double beta = 0.1;
                //Begin by assuming none of the joints are near their limits
                int rank = 0;
                
                //Clear out our prior activation values
                H.setZero();
                
                //Then, for each joint DoF
                for( unsigned i=0; i<nj; ++i )
                {
                    //Compute the range of the DoF
                    double range = q_bounds[i]->get_upper_bound() - q_bounds[i]->get_lower_bound();
                    //Compute the threshold value
                    double thresh = range*beta;
                    
                    //If we are out of bounds
                    if( !q_bounds[i]->is_valid( q_in(i) ) )
                    {
                        //We have a full activation here
                        H(i) = 1;
                        ++rank;
                    }
                    //Now, if we are close to our upper bound
                    else if( q_in(i) > q_bounds[i]->get_upper_bound() - thresh )
                    {
                        //Then, we know we have some activation here
                        double x = ( q_in(i) - q_bounds[i]->get_upper_bound() + thresh ) / thresh;
                        H(i) = transition_function( x );
                        ++rank;
                    }
                    //Then, if we are close to the lower bound
                    else if( q_in(i) < q_bounds[i]->get_lower_bound() + thresh )
                    {
                        //We have another activation here
                        double x = (q_bounds[i]->get_lower_bound() + thresh - q_in(i)) / thresh;
                        H(i) = transition_function( x );
                        ++rank;
                    }
                }
                
                return rank;
            }
            
            int prx_chainiksolvervel_pinv_nso::compute_discrete_activation_matrix( const JntArray& q_in, const std::vector< bounds_t* >& q_bounds )
            {
                //Begin by assuming none of the joints are at the limit
                int rank = 0;
                
                //Make sure H is zeroed out
                H.setZero();
                
                //For each DoF
                for( unsigned i=0; i<nj; ++i )
                {
                    //If we're basically at the upper bound
                    if( q_in(i) > q_bounds[i]->get_upper_bound() - 1e-7 )
                    {
                        //This joint is getting activated
                        H(i) = 1;
                        ++rank;
                    }
                    //Also check if we're at the lower bound
                    else if( q_in(i) < q_bounds[i]->get_lower_bound() + 1e-7 )
                    {
                        H(i) = 1;
                        ++rank;
                    }
                }
                
                return rank;
            }
            
            double prx_chainiksolvervel_pinv_nso::transition_function( double x )
            {
                double a = 1.0/(1.0 - x);
                a -= (1/x);
                return 0.5 * ( 1 + tanh(a) );
            }

            void prx_chainiksolvervel_pinv_nso::vector_cross_product( Eigen::MatrixXd & result, const Eigen::VectorXd & constants, const Eigen::MatrixXd & left, const Eigen::MatrixXd & right, unsigned min_range, unsigned max_range )
            {
                result.setZero();
                for( unsigned i=min_range; i<max_range+1; ++i )
                {
                    result += constants(i) * left.col(i) * right.col(i).transpose();
                }
            }

            void prx_chainiksolvervel_pinv_nso::mt_v_m( Eigen::MatrixXd & result, const Eigen::VectorXd & constants, const Eigen::MatrixXd & left, const Eigen::VectorXd & vec, const Eigen::MatrixXd & right, unsigned min_range, unsigned max_range )
            {
                result.setZero();
                for( unsigned i=min_range; i<max_range+1; ++i )
                {
                    Eigen::VectorXd val(constants(i) * (left.col(i).transpose() * vec));
                    result +=  (val(0) * right.col(i));
                }
            }            

            bool prx_chainiksolvervel_pinv_nso::is_subset( unsigned lh_index, unsigned rh_index )
            {
                return (lh_index & rh_index) == lh_index;
            }

            void prx_chainiksolvervel_pinv_nso::clamp_max_abs( Eigen::VectorXd & w, double d )
            {
                double norm = 0;// w.lpNorm<1>();
                for( unsigned i=0; i<w.size(); ++i )
                {
                    norm = PRX_MAXIMUM( norm, fabs(w(i)) );
                }
                if( norm > d )
                {
                    // PRX_PRINT("Clampage executing over: \n" << w, PRX_TEXT_RED);
                    w *= d;
                    w /= norm;
                    // PRX_PRINT("Created:\n" << w, PRX_TEXT_LIGHTGRAY);
                }
            }

            int prx_chainiksolvervel_pinv_nso::setWeights(const JntArray & _weights)
            {
              weights = _weights;
              return 0;
            }

            int prx_chainiksolvervel_pinv_nso::setOptPos(const JntArray & _opt_pos)
            {
              opt_pos = _opt_pos;
              return 0;
            }

            int prx_chainiksolvervel_pinv_nso::setAlpha(const double _alpha)
            {
              alpha = _alpha;
              return 0;
            }
            
            void prx_chainiksolvervel_pinv_nso::compute_powerset()
            {
                // We have 2^n sets to compute
                unsigned sz = pow( 2, nj );
                
                //Compute each of them
                Eigen::VectorXd v (VectorXd::Zero(nj));
                for( unsigned i=0; i<sz; ++i )
                {
                    //Begin with it at zero
                    v.setZero();
                    
                    unsigned val = i;
                    //We will be dividing up these numbers
                    for( unsigned k=0; k<nj; ++k )
                    {
                        v(k) = val%2;
                        val /= 2;
                    }
                    
                    //Add it to the set
                    P.push_back( v );
                }
            }

        }
    }
}
