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

#ifndef PRX_PLACE_OBJECTS_APPLICATION_HPP
#define	PRX_PLACE_OBJECTS_APPLICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/simulation/applications/application.hpp"

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
            class movable_body_plant_t;
        }
        
        namespace coordination_manipulation
        {
            /**
             * @authors Andrew Kimmel
             */
            
            struct object_placement
            {
                manipulation::movable_body_plant_t* object;
                std::vector<double> pose;
                bool valid;
                int object_index;
                std::string object_type;
            };
            
            class place_objects_application_t : public sim::application_t
            {

            public:
                place_objects_application_t();
                virtual ~place_objects_application_t();

                virtual void init(const util::parameter_reader_t * const reader);
                virtual void frame(const ros::TimerEvent& event);
                virtual void shutdown();


                /** @copydoc application_t::set_selected_path(const std::string&)*/
                virtual void set_selected_path(const std::string& path);
                
            protected:
                
                virtual bool create_object_placement(bool reset_all= false);
                virtual void place_objects_state();
                virtual bool set_object_validity();
                virtual void save_placements_to_file();
                
                std::vector< manipulation::movable_body_plant_t* > objects; //List of the objects in the scene
                std::vector< manipulation::movable_body_plant_t* > removed_objects;

                const util::space_t* object_space;
                sim::state_t* object_state;
                std::vector<std::string> object_types;
                
                unsigned failed_placement_counter, max_tries;
                unsigned valid_placement_counter;
                std::string pose_directory, results_file;
                util::hash_t< std::string, std::vector< std::vector<double> > > left_valid_poses, right_valid_poses;
                util::hash_t< std::string, int> max_pose_limit;
                
                std::vector<object_placement*> current_left_assignments, current_right_assignments;
                
                util::hash_t< std::string, int > left_object_assignments, right_object_assignments;
                bool left_assigned, right_assigned;
                    

            };
        }

    }
}

#endif

