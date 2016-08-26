/**
 * @file rally_car_mapping.hpp 
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

#ifndef PRX_RALLY_CAR_MAPPING_HPP
#define	PRX_RALLY_CAR_MAPPING_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/mapping_functions/mapping_function.hpp"

namespace prx 
{ 
    namespace packages 
    {
        namespace mapping_functions
        {

            /**
            * @brief <b> Embeds an Rally Car into SE2 </b>
            * @author Zakary Littlefield
            */
            class rally_car_mapping_t : public util::mapping_function_t
            {
                public:
                    rally_car_mapping_t() 
                    {
                        domain = 13;
                        range = 3;
                        image_space = NULL;
                        subspace = NULL;
                        preimage_space = NULL;
                        image_interval = std::make_pair<unsigned,unsigned>(0,0);
                        preimage_interval = std::make_pair<unsigned,unsigned>(0,0);
                        output_space_name = "SE2";
                        mapping_name = "rally_car_mapping";
                    }
                    virtual ~rally_car_mapping_t() {}    

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

            };   
        } 
    } 
}


#endif