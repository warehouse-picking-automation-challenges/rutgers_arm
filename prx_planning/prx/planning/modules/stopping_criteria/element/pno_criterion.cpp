/**
 * @file pno_criterion.cpp 
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

#include "prx/planning/modules/stopping_criteria/element/pno_criterion.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

#include <math.h>
#include <time.h>

#include <pluginlib/class_list_macros.h>

namespace prx 
{ 
    using namespace util;
    namespace plan 
    {
        PLUGINLIB_EXPORT_CLASS( prx::plan::pno_criterion_t, prx::plan::criterion_t)
        
        pno_criterion_t::pno_criterion_t()
        {
            iters = 0;
            beta = 0.5;
            epsilon = PRX_INFINITY+1;
            samples = attempts = 0;
            achieved_bound = eps_bound = Ialg = PRX_INFINITY;
        }
        
        pno_criterion_t::~pno_criterion_t()
        {
            stat_out.close();
            stat_out.clear();
        }
        
        void pno_criterion_t::init(const parameter_reader_t* reader,const parameter_reader_t* template_reader )
        {
            Iopt = parameters::get_attribute_as<double>("path_estimate", reader, template_reader );
            delta = parameters::get_attribute_as<double>("optimality_bound", reader, template_reader, 0.25 );
            alpha = parameters::get_attribute_as<double>("allowed_failure_chance", reader, template_reader, 0.1 );
            eps_bound = parameters::get_attribute_as<double>("epsilon_bound", reader, template_reader, PRX_INFINITY );
            
            Ialg = Iopt;
            
            compute_M( Iopt );
        }
        
        bool pno_criterion_t::criterion_check()
        {
            ++iters;
            bool dumped = false;
            
            samples = prm_motion_planner->num_vertices;
            //Now, every once in a while, we need to update the things
            if( iters % 10 == 0 )
            {
                //Get attempts information
                attempts = prm_motion_planner->num_generated;
                if( iters >= 50 )
                {
                    iters = 0;
                    //Retrieve the planner's best computed path thus far
                    prm_motion_planner->resolve_query();
                }
                //Update free space estimate
                free_volume = total_volume * (((double)samples)/((double)attempts));
                
                //Now, since this is PRM*, must compute e_n and b_n
                double power = ((1.0)/((double)d));
                double pa = (1.0 + power);
                double pb = (log( (double)samples ))/((double)samples);
                double pc = ((double)free_volume)/(linked_space->n_ball_measure(1));
                epsilon = 2*pow( pa * pb * pc, power );
                
                //TODO: Do we care to maintain PNO criterion?  This seems really annoying to maintain just for this...
                // double path_length = prm_motion_planner->last_solution_length;
                double path_length = 0;
                if( path_length < Ialg )
                    Ialg = path_length;
                
                compute_M( Ialg );
                compute_limit();
                
                double tmp = path_length/(1.0 + achieved_bound);
                if( achieved_bound < 1 )
                {
                    Iopt = tmp;
                    compute_M( Iopt );
                    compute_limit();
                }
            }
            
            if( iters == 0 ) //Do some statistics gathering.
            {
                dump_stats();
                dumped = true;
                PRX_PRINT("E[eps]: " << achieved_bound << "   :d: " << epsilon << "   :B: " << beta << "   :Ialg: " << Ialg, PRX_TEXT_LIGHTGRAY );
                // PRX_PRINT("Ialg: " << Ialg << "   Iopt: " << Iopt, PRX_TEXT_LIGHTGRAY );
            }
            
            if( eps_bound > epsilon && delta > achieved_bound )
            //if( eps_bound > epsilon ) // HACK HACK HACK
            {
                PRX_PRINT("Final Expected Degradation: " << achieved_bound, PRX_TEXT_BROWN );
                if( !dumped )
                    dump_stats();
                stat_out.close();
                stat_out.clear();
                return true;
            }
            return false;
        }
        
        void pno_criterion_t::link_motion_planner(motion_planner_t* mp)
        {
            PRX_DEBUG_COLOR("Linking the motion planner...", PRX_TEXT_CYAN );
            criterion_t::link_motion_planner( mp );
            //I think we need to store a pointer explicitly to a prm* planner
            prm_motion_planner = dynamic_cast<prm_star_t*>( mp );
            if( prm_motion_planner == NULL )
                PRX_FATAL_S("PNO Criterion requires the linked motion planner be a PRM*");
            linked_space = mp->state_space;
            total_volume = linked_space->space_size();
            d = linked_space->get_dimension();
            PRX_PRINT("Working in a " << d << " dimensional space, of size: " << total_volume, PRX_TEXT_LIGHTGRAY );
            free_volume = total_volume;
            //compute_limit();
            
            //Open up an appropriate file for output...
            char* w = std::getenv("PRACSYS_PATH");
            std::string fname(w);
            fname += ("/prx_output/pno_data/");
            //fname += "dim_" + int_to_str( d ) + "_" + int_to_str( time(NULL) ) + ".txt";
            fname += "dim_" + int_to_str( d ) + "_" + ros::this_node::getName().substr(1,ros::this_node::getName().length()) + ".txt";
            
            stat_out.open( fname.c_str() );
        }

        void pno_criterion_t::compute_limit()
        {
            //PRX_DEBUG_COLOR("Computing the limit...", PRX_TEXT_BLUE );
            //Need to seed \beta with something small.
            double lambda = 0.5;
            double step = 0.05;
            
            //Then, we need to compute the limit for this beta
            double last_deg = compute_delta( lambda );
            
            //Now perform hill climbing until resolution is small
            while( step > PRX_ZERO_CHECK )
            {
                //Get some idea about the neighborhood
                double low = compute_delta( lambda - step );
                double high = compute_delta( lambda + step );
                //PRX_PRINT(": " << low << " " << last_deg << " " << high, PRX_TEXT_LIGHTGRAY );
                //Check if we appear to be at a minimum
                if( last_deg < low && last_deg < high )
                {
                    //Then, we need to decrease the step size
                    step /= 1.5;
                }
                else if( low < high ) //We need to move to smaller beta
                {
                    lambda -= step;
                    last_deg = low;
                }
                else if( high < low ) //Need to move to larger beta
                {
                    lambda += step;
                    last_deg = high;
                }
                else //We've essentially bottomed out at this point
                {
                    step = 0;
                }
                //Also slightly degrade the step size a bit so as to ensure it stops.
                step -= 0.0001;
                if( lambda > 0.5 )
                {
                    lambda = 0.5;
                    step = 0;
                    last_deg = compute_delta( lambda );
                }
            }
            
            //Now, set the limit to the minimum n we found.
            beta = lambda*epsilon;
            achieved_bound = last_deg;
        }
        
        unsigned pno_criterion_t::compute_n( double rad )
        {
//            PRX_DEBUG_COLOR("Computing n for the following parameters:", PRX_TEXT_GREEN );
//            PRX_DEBUG_COLOR("d: " << d, PRX_TEXT_LIGHTGRAY );
//            PRX_DEBUG_COLOR("|Cfree|: " << free_volume, PRX_TEXT_LIGHTGRAY );
//            PRX_DEBUG_COLOR("delta: " << delta, PRX_TEXT_LIGHTGRAY );
//            PRX_DEBUG_COLOR("Iopt: " << Iopt, PRX_TEXT_LIGHTGRAY );
//            PRX_DEBUG_COLOR("epsilon: " << epsilon, PRX_TEXT_LIGHTGRAY );
//            PRX_DEBUG_COLOR("beta: " << rad, PRX_TEXT_LIGHTGRAY );
//            PRX_DEBUG_COLOR("alpha: " << alpha, PRX_TEXT_LIGHTGRAY );
//            PRX_DEBUG_COLOR("M: " << M, PRX_TEXT_LIGHTGRAY );
            
            //First, compute eta
            double a = (4.0 * pow(rad,2.0));
            double b = (d+2.0)*pow(M,2.0)*pow(epsilon,2.0)*pow( ( delta + ((d-1.0)/(d+2.0))*(pow(rad,2.0)/pow(epsilon,2.0)) ), 2.0 );
            
            double eta = a/b;
            
            //PRX_DEBUG_COLOR("eta: " << eta, PRX_TEXT_CYAN );
            
            //Now, get the result which will be raised to (1/(M+1))
            a = (alpha - 1.0)/(eta - 1.0);
            
            //If this is a negative value, this is an invalid radius, return as many iterations as possible.
            if( a < 0 )
                return -1; // Computation was bad...
            
            //If that value was okay, compute the numerator log value.
            a = 1 - pow( a, (1.0/(M+1.0)) );
            
            //If this is a negative value, this is an invalid radius
            if( a < 0 )
                return -1; // Computation was bad...
            
            //Now get the denominator log value.
            b = 1 - ( linked_space->n_ball_measure( rad ) / free_volume );
            
            //If this is a negative value, then either our space is messed up, or too huge beta
            if( b < 0 )
                return -1; // Computation was bad
            
            unsigned n = ceil( log( a ) / log ( b ) );
            
            //PRX_DEBUG_COLOR("Rad> " << rad << " >> " << n, PRX_TEXT_LIGHTGRAY );
            
            return n;
        }
        
        void pno_criterion_t::compute_M( double path )
        {
            M = path / epsilon;
        }
        
        double pno_criterion_t::compute_delta( double in_lambda )
        {
            // in_lambda = 0.5;
            
            double p_cover = compute_pcover( in_lambda );
            double inroot = (1.0)/((d+2)*(1 - ( (1-alpha)/(p_cover) )));
            
            if( inroot < 0 )
                return PRX_INFINITY;
            double mult = ( 2*in_lambda )/((double)M);
            double root = sqrt( inroot );
            double tail = in_lambda*in_lambda*((d-1.0)/(d+2.0));
            
//            PRX_PRINT("============================================", PRX_TEXT_GREEN );
//            PRX_PRINT("=    Computation of Delta, root defined    =", PRX_TEXT_CYAN );
//            PRX_PRINT("============================================", PRX_TEXT_GREEN );
//            PRX_PRINT("Delta Params:", PRX_TEXT_BLUE );
//            PRX_PRINT("Mult: " << mult , PRX_TEXT_LIGHTGRAY );
//            PRX_PRINT("Root: " << root , PRX_TEXT_LIGHTGRAY );
//            PRX_PRINT("Tail: " << tail , PRX_TEXT_LIGHTGRAY );
//            PRX_PRINT("Other params:", PRX_TEXT_MAGENTA );
//            PRX_PRINT("Pcov: " << p_cover , PRX_TEXT_LIGHTGRAY );
//            PRX_PRINT("inrt: " << inroot , PRX_TEXT_LIGHTGRAY );
//            PRX_PRINT("M:    " << M , PRX_TEXT_LIGHTGRAY );
//            PRX_PRINT("d:    " << d , PRX_TEXT_LIGHTGRAY );

            return mult*root + tail;
        }
        
        double pno_criterion_t::compute_pcover( double in_lambda )
        {
            double ballratio = linked_space->n_ball_measure( in_lambda*epsilon )/free_volume;
            //if( ballratio > 1 ) //If the balls are still larger than the free space,
            //    return 0.0000000000001;
            double inner = pow( 1.0 - ballratio, samples );
            double outer = pow( 1.0 - inner, M+1 );
            
            //PRX_DEBUG_COLOR("Things: " << in_beta << " (" << linked_space->n_ball_measure( in_beta ) << ") == " << M << " == " << samples, PRX_TEXT_LIGHTGRAY ); 
            if( fabs(outer) > 1 )
                outer = 1;
            return outer;
        }
        
        void pno_criterion_t::reset()
        {
            //Is there some logical way I can reset this thing?  I don't think so.
        }
        
        void pno_criterion_t::dump_stats()
        {
            // stat_out << samples << "\t" << attempts << "\t" << beta << "\t" << achieved_bound << "\t" << M << "\t" << Iopt << "\t" << free_volume << "\t" << compute_pcover() << "\t" << prm_motion_planner->last_solution_length << "\n";
            stat_out.flush();
        }
        
        std::string pno_criterion_t::print_statistics()
        {
            return std::string("Iteration Limit: " + int_to_str(limit) );
        }
        
        
    }
}