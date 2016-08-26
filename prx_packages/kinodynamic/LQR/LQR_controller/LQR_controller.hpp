#pragma once
/**
 * @file LQR_controller.hpp
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

#ifndef PRX_LQR_CONTROLLER_HPP
#define	PRX_LQR_CONTROLLER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/simulation/dynamics/systems/controllers/simple_controller.hpp"

#ifdef OCTAVE_FOUND
#include "prx/utilities/octave_interface/octave_caller.hpp"
#endif

PRX_START

/**
 * A controller that implements the linear quadratic regulator (LQR).
 */
class LQR_controller_t : public simple_controller_t
{

  public:
    LQR_controller_t();
    virtual ~LQR_controller_t();
    void init(const parameter_reader_t* reader);

    void push_control(const control_t * const source);

    /** @copydoc system_t::set_param(const std::string&, const boost::any&) */
    void set_param(const std::string& parameter_name, const boost::any& value);

  protected:    
#ifdef OCTAVE_FOUND
    octave_value_list return_val;
#endif
  private:
    vector_t goal_state;

};

PRX_FINISH

#endif	// PRX_LQR_CONTROLLER_HPP

