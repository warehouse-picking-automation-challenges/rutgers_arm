/**
 * @file string_manip.cpp 
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

#include "prx/utilities/definitions/string_manip.hpp"

#include <boost/lexical_cast.hpp>

namespace prx
{
    namespace util
    {

        std::string int_to_str(int num)
        {
            return boost::lexical_cast<std::string > (num);
        }

        std::pair<const std::string, const std::string> split_path(const std::string& path, char delimiter)
        {
            const size_t first_slash = path.find(delimiter);

            if( first_slash == path.npos )
                return std::make_pair(path, "");
            else
                return std::make_pair(
                                      path.substr(0, first_slash),
                                      path.substr(first_slash + 1, path.npos));
        }

        std::pair<const std::string, const std::string> reverse_split_path(const std::string& path, char delimiter)
        {
            const size_t last_slash = path.rfind(delimiter);

            if( last_slash == path.npos )
                return std::make_pair(path, "");
            else
                return std::make_pair(
                                      path.substr(0, last_slash),
                                      path.substr(last_slash + 1, path.npos));
        }

        bool reverse_string_compare(const std::string& str1,const std::string& str2)
        {
            return str1.size() == str2.size() && std::equal(str1.rbegin(), str1.rend(), str2.rbegin());
        }

        std::string replace_first(std::string &input, const std::string& to_replace, const std::string& replace_with)
        {
            return(input.replace(input.find(to_replace), to_replace.length(), replace_with));
        }

    }
}
