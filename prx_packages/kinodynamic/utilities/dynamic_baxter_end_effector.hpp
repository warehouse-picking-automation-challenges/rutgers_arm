/**
 * @file dynamic_baxter_end_effector.hpp 
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

#ifndef PRX_DYNAMIC_BAXTER_END_EFFECTOR_HPP
#define	PRX_DYNAMIC_BAXTER_END_EFFECTOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/mapping_functions/mapping_function.hpp"


#ifdef RBDL_FOUND
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#endif

namespace prx 
{ 
    namespace packages 
    {
        namespace mapping_functions
        {
            /**
             * @brief <b> Embeds a dynamic Baxter arm into its end effector position </b>
             * @author Zakary Littlefield
             */
            class dynamic_baxter_end_effector_t : public util::mapping_function_t
            {
            public:
                dynamic_baxter_end_effector_t() 
                {
                    domain = 30;
                    range = 7;
                    image_space = NULL;
                    subspace = NULL;
                    preimage_space = NULL;
                    image_interval = std::make_pair<unsigned,unsigned>(0,0);
                    preimage_interval = std::make_pair<unsigned,unsigned>(0,0);
                    output_space_name = "BAXTER_EE_POSE";
                    mapping_name = "dynamic_baxter_end_effector";
                }
                virtual ~dynamic_baxter_end_effector_t() {}    

                /**
                 * Read in parameters to initialize variables.
                 * @param reader The primary reader. Any parameters found here will be read.
                 * @param template_reader The secondary reader. Any parameters not in the primary reader will be found here.
                 */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader=NULL);
                
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
                unsigned int id;
    #ifdef RBDL_FOUND
                mutable RigidBodyDynamics::Model model;
                mutable RigidBodyDynamics::Math::VectorNd Q;
                mutable RigidBodyDynamics::Math::VectorNd QDot;
                RigidBodyDynamics::Math::VectorNd QDDot;
    #endif
                
            };
        }
    } 
}


#endif