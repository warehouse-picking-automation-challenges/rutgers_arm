/**
 * @file pno_criterion.hpp 
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
#pragma once

#ifndef PRX_PNO_CRITERION_HPP
#define PRX_PNO_CRITERION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/planning/modules/stopping_criteria/element/criterion.hpp"
#include "prx/planning/motion_planners/prm_star/prm_star.hpp"
//#include "prx/utilities/definitions/sys_clock.hpp"

namespace prx
{
    namespace util
    {
        class statistics_t;
    }
    
    namespace plan
    {
        /**
         * @anchor pno_criterion_t
         *
         * This simple criterion checks an iteration limit.  If the iteration count has
         * surpassed some maximum threshold, then the criterion is satisfied.
         *
         * @brief <b> A criterion which checks an iteration counter. </b>
         *
         * @author Andrew Kimmel
         */
        class pno_criterion_t : public criterion_t
        {

          public:
            pno_criterion_t();
            ~pno_criterion_t();

            /**
             * @copydoc criterion_t::init()
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);

            /**
             * @copydoc criterion_t::criterion_check()
             *
             * @note Checks if iteration_counter >= max_iterations.
             */
            virtual bool criterion_check();
            
            /**
             * @copydoc criterion_t::link_motion_planner( motion_planner_t* )
             */
            virtual void link_motion_planner(motion_planner_t* mp);

            /**
             * 
             */
            void compute_limit();

            /**
             * 
             */
            unsigned compute_n( double rad );

            /**
             *
             */
            void compute_M( double path );
            
            /**
             *
             */
            double compute_delta( double in_lambda = 0.5 );

            /**
             *
             */
            double compute_pcover( double in_lambda = 0.5 );

            void dump_stats();
            
            /**
             * @copydoc criterion_t::print_statistics()
             */
            virtual std::string print_statistics();
            
            /**
             * @copydoc criterion_t::reset()
             */
            virtual void reset();

          protected:

          private:

            /** @brief Pointer to the motion planner, cast as a prm_star */
            prm_star_t* prm_motion_planner;
            /** @brief Pointer to the state space the linked motion planner is operating over */
            const util::space_t* linked_space;
            
            /** @brief Current Estimate of the length of the optimal path. */
            double Iopt;
            double Ialg;
            /** @brief Computed clearance for the clearance-robust optimal path. */
            double epsilon;
            /** @brief Input: set bound on epsilon which must be reached. */
            double eps_bound;
            /** @brief Input: Desired sub-optimality bound */
            double delta;
            /** @brief Input: Desired percentage of having a bad path */
            double alpha;
            
            /** @brief The currently computed iteration limit for the stopping criterion */
            double limit;
            /** @brief The total volume of the bounded configuraiton space. */
            double total_volume;
            /** @brief The current estimate of the volume of the free space. */
            double free_volume;
            /** @brief The current number of covering balls required to cover Iopt. */
            double M;
            /** @brief The dimensionality of the space */
            unsigned d;
            
            /** @brief Number of iterations since last update */
            unsigned iters;
            /** @brief Number of samples in the space */
            unsigned samples;
            /** @brief Total number of sample attempts */
            unsigned attempts;
            
            /** @brief Inner radius of the interior balls, computed by the criterion. */
            double beta;
            /** @brief achieved suboptimality */
            double achieved_bound;
            
            /** @brief output file for gathering statistics. */
            std::ofstream stat_out;
        };

    }
}

#endif