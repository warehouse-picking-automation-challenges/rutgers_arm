/**
 * @file subset_mapping.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_SUBSET_MAPPING_HPP
#define	PRX_SUBSET_MAPPING_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/mapping_functions/mapping_function.hpp"

namespace prx 
{ 
    namespace util 
    {
        
        /**
         * @brief <b> A mapping that takes a subset of values from the input space. </b>
         * @author Zakary Littlefield
         */
        class subset_mapping_t : public mapping_function_t
        {
        public:
            subset_mapping_t() 
            {
                domain = 0;
                range = 0;
                mapped_indices.clear();
                image_space = NULL;
                subspace = NULL;
                preimage_space = NULL;
                output_space_name = "";
                mapping_name = "subset_mapping";
            }
            virtual ~subset_mapping_t() {}
            
            /**
             * @copydoc mapping_function_t::embed() const
             */
            virtual void embed() const;
            
            /**
             * @copydoc mapping_function_t::invert() const
             */
            virtual void invert() const;


            virtual void init_spaces();
            
        protected:

            std::vector<unsigned> mapped_indices;
            
        };
        
    } 
}


#endif