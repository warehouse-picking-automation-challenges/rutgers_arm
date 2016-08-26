/**
 * @file string_manip.hpp 
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

#ifndef PRACSYS_STRING_MANIP_HPP
#define PRACSYS_STRING_MANIP_HPP

#include "prx/utilities/definitions/defs.hpp"

namespace prx 
 { 
 namespace util 
 {

/**
 * Converts an integer into a std::string
 * 
 * @brief Converts an integer into a std::string
 * @param num The integer to be converted into a string
 * @return The integer converted into a string
 */
std::string int_to_str(int num);

/**
 * Splits a /-deliminated path into the remainder and the beginning
 * 
 * For example: 
 * 
 * path = "a/b/c/d"
 * split = split_path(path);
 * split.first = "b/c/d"
 * split.second = "a"
 * 
 * 
 * @brief Splits a pathname based on slashes "/"
 * @param path The path to be split
 * @return A pair containing the split path and the remainder
 */
std::pair<const std::string, const std::string> split_path(const std::string& path, char delimiter = '/');

/**
 * Splits a /-deliminated path into the first part and thet ail
 * 
 * For example: 
 * 
 * path = "a/b/c/d"
 * split = reverse_split_path(path);
 * split.first = "a/b/c"
 * split.second = "d"
 * 
 * 
 * @brief Splits a pathname based on slashes "/"
 * @param path The path to be split
 * @return A pair containing the split path and the remainder
 */
std::pair<const std::string, const std::string> reverse_split_path(const std::string& path, char delimiter = '/');

/**
 *  Performs string comparison between two strings. The difference 
 *  is that comparison is done from the end of the string to the beginning.
 *  This can be especially helpful for system trees, where only the last few characters differ.
 */
bool reverse_string_compare(const std::string& str1,const std::string& str2);

/**
 * A string replacement helper function.
 */
std::string replace_first(std::string &input, const std::string& to_replace, const std::string& replace_with);


} 
 }

#endif //PRACSYS_STRING_MANIP_HPP
