/**
 * @file superball_center.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_SUPERBALL_CENTER_HPP
#define PRX_SUPERBALL_CENTER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/mapping_functions/mapping_function.hpp"
// #include "prx/utilities/math/configurations/quaternion.hpp"

namespace prx 
 { 
 namespace packages 
 {
namespace mapping_functions
{
/**
 * @brief <b> Embeds a superball into its center location </b>
 * @author Zakary Littlefield
 */
class superball_center_t : public util::mapping_function_t
{
public:
    superball_center_t() 
    {
        domain = 78+24;
        range = 3;
        image_space = NULL;
        subspace = NULL;
        preimage_space = NULL;
        image_interval = std::make_pair<unsigned,unsigned>(0,0);
        preimage_interval = std::make_pair<unsigned,unsigned>(0,0);
        output_space_name = "XYZ";
        mapping_name = "superball_center";
    }
    virtual ~superball_center_t() {}    
    
    /**
     * @copydoc mapping_function_t::init_spaces()
     */
    void init_spaces();
    /**
     * @copydoc mapping_function_t::embed() const
     */
    virtual void embed() const;
    /**
     * @copydoc mapping_function_t::invert() const
     */
    virtual void invert() const;
    
protected:
       
};
}    
} 
 }


#endif