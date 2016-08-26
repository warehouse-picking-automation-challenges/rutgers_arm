#pragma once
/**
 * @file LQR_tree.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2011, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Zakary Littlefield, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, James Marble, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#ifndef PRX_LQR_TREE_HPP
#define	PRX_LQR_TREE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/simulation/dynamics/systems/controllers/simple_controller.hpp"

#ifdef OCTAVE_FOUND
#include "prx/utilities/octave_interface/octave_caller.hpp"
#endif

PRX_START

/**
 * The controller to execute a path given from the planning LQR-tree planner. 
 */
class LQR_tree_t : public simple_controller_t
{

  public:
    LQR_tree_t();
    virtual ~LQR_tree_t();
    void init(const parameter_reader_t* reader);
    
    void set_internal_gains(std::vector<double>& radii, std::vector<vector_t>& centers, std::vector<vector_t>& controls, std::vector<vector_t>& gains,std::vector<vector_t>& costs);

    void push_control(const control_t * const source);

    /** @copydoc system_t::set_param(const std::string&, const boost::any&) */
    void set_param(const std::string& parameter_name, const boost::any& value);

  protected:
#ifdef OCTAVE_FOUND
    std::vector<Matrix> gains;
    std::vector<Matrix> costs;
#endif
    std::vector<double> radii;
    std::vector<state_t*> centers;
    std::vector<control_t*> controls;

  private:
    vector_t goal_state;
    bool planning;

};

PRX_FINISH

#endif	// PRX_LQR_TREE_HPP

