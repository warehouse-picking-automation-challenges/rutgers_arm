#pragma once
/**
 * @file LTV_LQR.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2012, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Zakary Littlefield, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, James Marble, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#ifndef PRX_LTV_LQR_HPP
#define	PRX_LTV_LQR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/trajectory.hpp"
#include "prx/simulation/plan.hpp"
#include "prx/utilities/spaces/simple_space.hpp"
#include "prx/simulation/dynamics/systems/controllers/stateful_controller.hpp"

#ifdef OCTAVE_FOUND
#include "prx/utilities/octave_interface/octave_caller.hpp"
#endif

PRX_START

/**
 * A controller that implements the linear time variant version of LQR.
 * 
 */
class LTV_LQR_t : public stateful_controller_t
{

  public:
    LTV_LQR_t();
    virtual ~LTV_LQR_t();
    void init(const parameter_reader_t* reader);
    void propagate( const double simulation_step = 0);  
    void push_control(const control_t * const source);

    /** @copydoc system_t::set_param(const std::string&, const boost::any&) */
    void set_param(const std::string& parameter_name, const boost::any& value);

  protected:
#ifdef OCTAVE_FOUND
    std::vector<Matrix> A;
    std::vector<Matrix> B;
    std::vector<Matrix> K;
    std::vector<Matrix> S;
    Matrix Q,R;
#endif
  private:
    trajectory_t trajectory;    
    plan_t plan;

};

PRX_FINISH

#endif	// PRX_LTV_LQR_HPP

