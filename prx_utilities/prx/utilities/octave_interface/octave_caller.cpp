// /**
//  * @file octave_caller.cpp
//  * 
//  * @copyright Software License Agreement (BSD License)
//  * Copyright (c) 2011, Board of Regents, NSHE, obo UNR
//  * All Rights Reserved.
//  * For a full description see the file named LICENSE.
//  *
//  * @authors Zakary Littlefield, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, James Marble, Kostas Bekris 
//  * 
//  * Email: pracsys@googlegroups.com
//  */
// #ifdef OCTAVE_FOUND
// #include "prx/utilities/octave_interface/octave_caller.hpp"
// #include <wordexp.h>

// namespace prx 
// { 
//     namespace util 
//     {

//         octave_caller_t::octave_caller_t()
//         {
//             wordexp_t p;
//             std::string test_path = "$PRACSYS_PATH/prx_utilities/prx/utilities/octave_interface/functions/";
//             wordexp(test_path.c_str(), &p, 0);
//             std::string dir(p.we_wordv[0]);
//             f_arg(0) = dir;
//             const char * argvv[] = {"", "-q"};
//             octave_main(2, (char**)argvv, true);
//             feval("cd", f_arg);
//         }

//         octave_value_list octave_caller_t::call_function(std::string function_name,const octave_value_list& args)
//         {
//             octave_value_list result = feval(function_name.c_str(), args);
//             return result;
//         }

//         octave_caller_t::~octave_caller_t()
//         {
//         }
//     } 
//  }

// #endif