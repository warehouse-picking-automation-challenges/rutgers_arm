// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/KDL

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

#ifndef PRX_KDL_CHAIN_IKSOLVERVEL_PINV_NSO_HPP
#define PRX_KDL_CHAIN_IKSOLVERVEL_PINV_NSO_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/bounds.hpp"

#include <kdl/chainiksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <Eigen/Core>

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
            enum solver_error_code_t
            {
                SOLVER_ERR_LARGE_H_RANK = 1
            };
            
            /**
             * Implementation of a inverse velocity kinematics algorithm based
             * on the generalize pseudo inverse to calculate the velocity
             * transformation from Cartesian to joint space of a general
             * KDL::Chain. It uses a svd-calculation based on householders
             * rotations.
             *
             * In case of a redundant robot this solver optimizes the the following criterium:
             * g=0.5*sum(weight*(Desired_joint_positions - actual_joint_positions))^2 as described in 
             *  A. Liegeois. Automatic supervisory control of the configuration and 
             * behavior of multibody mechnisms. IEEE Transactions on Systems, Man, and 
             * Cybernetics, 7(12):868–871, 1977
             *
             * @ingroup KinematicFamily
             */
            class prx_chainiksolvervel_pinv_nso : public KDL::ChainIkSolverVel
            {
            public:
                /**
                 * Constructor of the solver
                 *
                 * @param chain the chain to calculate the inverse velocity
                 * kinematics for
                 * @param opt_pos the desired positions of the chain used by to resolve the redundancy
                 * @param weights the weights applied in the joint space
                 * @param eps if a singular value is below this value, its
                 * inverse is set to zero, default: 0.00001
                 * @param maxiter maximum iterations for the svd calculation,
                 * default: 150
                 * @param alpha the null-space velocity gain
                 *
                 */
                prx_chainiksolvervel_pinv_nso(const KDL::Chain& chain, KDL::JntArray opt_pos, KDL::JntArray weights, double eps=0.00001,int maxiter=150, double alpha = 0.25);
                explicit prx_chainiksolvervel_pinv_nso(const KDL::Chain& chain, double eps=0.00001,int maxiter=150, double alpha = 0.25);
                ~prx_chainiksolvervel_pinv_nso();

                virtual int CartToJnt(const KDL::JntArray& q_in, const KDL::Twist& v_in, KDL::JntArray& qdot_out);

                virtual int CartToJntSDLS(const KDL::JntArray& q_in, const KDL::Twist& v_in, KDL::JntArray& qdot_out);

                virtual int CartToJntSDLS_SVF(const KDL::JntArray& q_in, const KDL::Twist& v_in, KDL::JntArray& qdot_out);

                virtual int CartToJntClamping(const KDL::JntArray& q_in, const std::vector< util::bounds_t* >& q_bounds, const KDL::Twist& v_in, KDL::JntArray& qdot_out);

                virtual int CartToJnt_IDF(const KDL::JntArray& q_in, const KDL::Twist& v_in, KDL::JntArray& qdot_out);

                virtual int CartToJntChiaverini(const KDL::JntArray& q_in, const KDL::Twist& v_in, KDL::JntArray& qdot_out);

                //Methods from Colome and Torres
                virtual int CartToJnt_continuous(const KDL::JntArray& q_in, const std::vector< util::bounds_t* >& q_bounds, const KDL::Twist& v_in, KDL::JntArray& qdot_out);
                virtual int CartToJnt_continuous_SD_SVF(const KDL::JntArray& q_in, const std::vector< util::bounds_t* >& q_bounds, const KDL::Twist& v_in, KDL::JntArray& qdot_out);

                //Methods from Mansard, Khatib, & Kheddar
                virtual int CartToJnt_Khatib_52(const KDL::JntArray& q_in, const std::vector< util::bounds_t* >& q_bounds, const KDL::Twist& v_in, KDL::JntArray& qdot_out);
                virtual int CartToJnt_Khatib_57(const KDL::JntArray& q_in, const std::vector< util::bounds_t* >& q_bounds, const KDL::Twist& v_in, KDL::JntArray& qdot_out);

                /**
                 * not (yet) implemented.
                 *
                 */
                virtual int CartToJnt(const KDL::JntArray& q_init, const KDL::FrameVel& v_in, KDL::JntArrayVel& q_out){return -1;};

                /**
                 *Set joint weights for optimization criterion
                 *
                 *@param weights the joint weights
                 *
                 */
                virtual int setWeights(const KDL::JntArray &weights);

                /**
                 *Set optimal joint positions
                 *
                 *@param opt_pos optimal joint positions
                 *
                 */
                virtual int setOptPos(const KDL::JntArray &opt_pos);

                /**
                 *Set null psace velocity gain
                 *
                 *@param alpha NUllspace velocity cgain
                 *
                 */
                virtual int setAlpha(const double alpha);

                void setBounds(const KDL::JntArray& q_min,const KDL::JntArray& q_max)
                {
                    qmin = q_min;
                    qmax = q_max;
                }

                /**
                 * Debugging timing function
                 */
                double measure();
                double measure_jac();
                double measure_svd();

            protected:
                void compute_powerset();

                virtual int compute_post_pseudoinverse( Eigen::MatrixXd& result, const Eigen::MatrixXd & J, const Eigen::VectorXd & H_0, double lambda = 1e-3 );
                virtual int compute_pre_pseudoinverse( Eigen::MatrixXd& result, const Eigen::MatrixXd & J, const Eigen::VectorXd & H_0, double lambda = 1e-3 );
                virtual int compute_post_pseudoinverse_SVF( Eigen::MatrixXd& result, const Eigen::MatrixXd & J, const Eigen::VectorXd & H_0, double lambda = 1e-3 );
                virtual int compute_pre_pseudoinverse_SVF( Eigen::MatrixXd& result, const Eigen::MatrixXd & J, const Eigen::VectorXd & H_0, double lambda = 1e-3 );
                
                virtual int compute_post_coupling_matrix( const Eigen::MatrixXd & J, const Eigen::VectorXd & H_0, unsigned index );
                virtual int compute_pre_coupling_matrix( const Eigen::MatrixXd & J, const Eigen::VectorXd & H_0, unsigned index );

                virtual int compute_continuous_activation_matrix( const KDL::JntArray& q_in, const std::vector< util::bounds_t* >& q_bounds );
                virtual int compute_discrete_activation_matrix( const KDL::JntArray& q_in, const std::vector< util::bounds_t* >& q_bounds );
                virtual double transition_function( double x );

                virtual void compute_continuous_post_pseudoinverse_colome( Eigen::MatrixXd& result, const Eigen::MatrixXd& J_in, const Eigen::MatrixXd& H_in );
                virtual void compute_continuous_pre_pseudoinverse_colome( Eigen::MatrixXd& result, const Eigen::MatrixXd& J_in, const Eigen::MatrixXd& H_in );
                virtual void compute_continuous_post_pseudoinverse_mansard( Eigen::MatrixXd& result, const Eigen::MatrixXd& J_in, const Eigen::MatrixXd& H_in );
                virtual void compute_continuous_pre_pseudoinverse_mansard( Eigen::MatrixXd& result, const Eigen::MatrixXd& J_in, const Eigen::MatrixXd& H_in );

                void vector_cross_product( Eigen::MatrixXd & result, const Eigen::VectorXd & constants, const Eigen::MatrixXd & left, const Eigen::MatrixXd & right, unsigned min_range, unsigned max_range );
                void mt_v_m( Eigen::MatrixXd & result, const Eigen::VectorXd & constants, const Eigen::MatrixXd & left, const Eigen::VectorXd & vec, const Eigen::MatrixXd & right, unsigned min_range, unsigned max_range );

                bool is_subset( unsigned lh_index, unsigned rh_index );
                void compute_SDLS_damping( Eigen::VectorXd& gamma, double gamma_max );
                void compute_continuous_damping( Eigen::VectorXd& gamma, double gamma_max );
                void clamp_max_abs( Eigen::VectorXd & w, double d );

            private:
                const KDL::Chain chain;
                KDL::ChainJntToJacSolver jnt2jac;
                unsigned int nj;
                unsigned int critical_H_rank;
                KDL::Jacobian jac;
                Eigen::MatrixXd U;
                Eigen::MatrixXd U_nj;
                Eigen::VectorXd H;
                Eigen::VectorXd S;
                Eigen::VectorXd S_nj;
                Eigen::VectorXd Sinv;
                Eigen::VectorXd SinvL;
                Eigen::MatrixXd V;
                Eigen::MatrixXd Uhat;
                Eigen::VectorXd Shat;
                Eigen::MatrixXd Vhat;
                Eigen::MatrixXd I;
                Eigen::MatrixXd J;
                Eigen::MatrixXd J_inv;
                Eigen::MatrixXd J_Q_cross;
                Eigen::VectorXd ones;
                Eigen::VectorXd e_dot;
                Eigen::VectorXd tmp;
                Eigen::VectorXd tmp2;
                Eigen::VectorXd projected_q;
                std::vector< Eigen::VectorXd > P;
                std::vector< Eigen::MatrixXd > XP;
                double eps;
                int maxiter;

                double alpha;
                KDL::JntArray weights;
                KDL::JntArray opt_pos;
                KDL::JntArray qmin;
                KDL::JntArray qmax;
                
                double elapsed;
                double jaco_time;
                double svd_time;
            };
        }
    }
}
#endif
