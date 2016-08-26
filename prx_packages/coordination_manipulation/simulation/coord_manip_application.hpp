/**
 * @file coord_manip_application.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_COORDINATION_MANIPULATION_APPLICATION_HPP
#define	PRX_COORDINATION_MANIPULATION_APPLICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/applications/application.hpp"
#include "prx/simulation/systems/controllers/consumer/consumer_controller.hpp"
#include "../../manipulation/simulation/systems/plants/manipulator.hpp"

namespace prx
{
    namespace packages
    {

        namespace manipulation
        {
            class movable_body_plant_t;
            class manipulator_t;
        }

        namespace coordination_manipulation
        {
            using namespace manipulation;

            /**
             * @authors Andrew Kimmel
             */
            class coord_manip_application_t : public sim::application_t
            {

                public:
                    coord_manip_application_t();
                    virtual ~coord_manip_application_t();

                    virtual void init(const util::parameter_reader_t * const reader);
                    virtual void frame(const ros::TimerEvent& event);
                    virtual void shutdown();
                    

                    /** @copydoc application_t::set_selected_path(const std::string&)*/
                    virtual void set_selected_path(const std::string& path);
                
                protected:
                    sim::state_t* left_drop, *right_drop;
                    std::vector< manipulation::movable_body_plant_t* > objects; //List of the objects in the scene
                    std::vector< manipulation::movable_body_plant_t* > removed_objects;
                    manipulator_t* manipulator;
                    const util::space_t* object_space;
                    sim::state_t* object_state;
                    sim::state_t* dropped_object_state;
                    
                    unsigned number_objects;
                    unsigned dropped_objects;
                    bool report_collisions;
                    
//                    sim::consumer_controller_t* consumer_controller;

            };
        }

    }
}

#endif

