/**
 * @file bullet_manip_to_se2.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_BULLET_MANIP_TO_XYTHETA_HPP
#define	PRX_BULLET_MANIP_TO_XYTHETA_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/mapping_functions/mapping_function.hpp"
#include "prx/utilities/math/configurations/quaternion.hpp"

namespace prx 
 { 
 namespace packages 
 {
    namespace mapping_functions
    {

/**
 * @brief <b> Embeds an bullet car space into SE2 </b>
 * @author Zakary Littlefield
 */
class bullet_manip_to_se2_t : public util::mapping_function_t
{
public:
    bullet_manip_to_se2_t() 
    {
        domain = 39;
        range = 4;
        image_space = NULL;
        subspace = NULL;
        preimage_space = NULL;
        image_interval = std::make_pair<unsigned,unsigned>(0,0);
        preimage_interval = std::make_pair<unsigned,unsigned>(0,0);
        output_space_name = "SE2|X";
        mapping_name = "bullet_manip_to_se2";
        v.resize(3);
    }
    virtual ~bullet_manip_to_se2_t() {}    
    
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
    mutable util::quaternion_t q;
    mutable util::vector_t v;
       
};
}
} 
 }


#endif