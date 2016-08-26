/**
 * @file VO_sensing_info.hpp
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

#ifndef VO_SENSING_INFO_HPP
#define	VO_SENSING_INFO_HPP

#include "simulation/controllers/VO_structure/neighbor.hpp"
#include "simulation/controllers/path_follow_controller.hpp"

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/simulation/sensing/infos/2d_proximity_sensing_info.hpp"

namespace prx
{
    namespace sim
    {
        class config_sensor_t;
        class velocity_sensor_t;
        class geometry_sensor_t;
        class plant_t;
    }

    namespace packages
    {
        namespace crowd
        {

            class VO_sensing_info_t : public sim::twoD_proximity_sensing_info_t
            {
            protected:
                //System storing/Neighbor parameters
                /** @brief Contains the memory for all neighbors (cannot be cleared, must be deleted)*/
                std::vector< neighbor_t* > all_neighbors;
                /** @brief Contains pointers to vo neighbors (can be cleared) */
                std::vector< neighbor_t* > vo_neighbors;
                /** @brief Contains pointers to non-vo neighbors (can be cleared) */
                std::vector< neighbor_t* > obstacle_neighbors;
                /** @brief Contains pointers to obstacle neighbors (can be cleared) */
                std::vector< neighbor_t* > other_neighbors;
                /** @brief Contains pointers to the current neighbors (cleared everytime update info is called) */
                std::vector< neighbor_t* > current_neighbors;
                /** Sensor Pointers */
                sim::velocity_sensor_t* vel_sensor;
                util::hash_t<std::string, util::vector_t> velocities;

                /** @brief Maps a pathname to a vo-controlled plant */
                util::hash_t<std::string, sim::plant_t*> vo_plants_map;
                /** @brief Maps a pathname to a non-vo-controlled plant */
                util::hash_t<std::string, sim::plant_t*> other_plants_map;
                util::hash_t<std::string, std::string> confpath_to_syspath_map;

                std::vector< sim::plant_t* > all_plants;

                path_follow_controller_t* path_controller;

            public:
                VO_sensing_info_t() ;
                virtual ~VO_sensing_info_t();

                /**
                 * Initializes from the given parameters.
                 *
                 * @brief Initializes from the given parameters.
                 *
                 * @param reader A \ref util::parameter_reader_t with a dictionary of parameters for this system.
                 * @param template_reader A template \ref util::parameter_reader_t with a dictionary of parameters.
                 * Default value for the template reader is NULL. The priority goes to primary reader \c reader, if something
                 * is not specified in the \c reader then it will be read from the \c template_reader
                 */
                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

                void set_follow_controller( void* controller );

                virtual void set_sensors(const std::vector<sim::sensor_t*>& sensors);

                // The conversion function to turn sensing data into the controller's info
                virtual void update_info();
                // Used to access the info?
                virtual void get_info();

                void set_system_path(const std::string& path);

                void set_all_plants( const std::vector< sim::plant_t* >& plants );
                void set_conf_to_sys_path_map(util::hash_t<std::string, std::string>& conf_sys_map);
                void set_sensed_VO_plants(util::hash_t<std::string, sim::plant_t*>& plants);
                void set_sensed_other_plants(util::hash_t<std::string, sim::plant_t*>& plants);

                std::vector< neighbor_t*>& get_current_neighbors();
                std::string get_system_path();

                util::sys_clock_t stats_clock;

                static double update_time;
                static unsigned update_calls;

            };
        }
    }
}

#endif

