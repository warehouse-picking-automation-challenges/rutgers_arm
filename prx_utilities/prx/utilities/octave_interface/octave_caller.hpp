// /**
//  * @file octave_caller.hpp
//  * 
//  * @copyright Software License Agreement (BSD License)
//  * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
//  * All Rights Reserved.
//  * For a full description see the file named LICENSE.
//  * 
//  * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
//  * 
//  * Email: pracsys@googlegroups.com
//  */

// #pragma once

// #ifndef PRX_OCTAVE_CALLER_HPP
// #define	PRX_OCTAVE_CALLER_HPP

// #ifdef OCTAVE_FOUND
// #include "prx/utilities/definitions/defs.hpp"

// #include <octave/oct.h>
// #include <octave/octave.h>
// #include <octave/parse.h>
// #include <octave/toplev.h>
// #include <octave/quit.h>

// namespace prx 
//  { 
//  namespace util 
//  {

// /**
//  * @brief <b> A wrapper class to call Octave functions.</b>
//  * 
//  * @author Zakary Littlefield 
//  */
// class octave_caller_t
// {
//     public:
//         octave_caller_t();
//         virtual ~octave_caller_t();

//         /**
//          * @brief Calls an Octave function file.
//          */
//         octave_value_list call_function(std::string function_name, const octave_value_list& args);

//     private:

//         /**
//          * An argument for initialization specifying the directory for function files.
//          */
//         octave_value_list f_arg;

// };

// } 
//  }

// #endif
// #endif	

