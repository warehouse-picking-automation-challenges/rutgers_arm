/**
 * @file VO_sensing_model.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_VO_SENSING_MODEL_HPP
#define PRX_VO_SENSING_MODEL_HPP

#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/distance_metrics/linear_metric/linear_distance_metric.hpp"
#include "prx/utilities/graph/directed_node.hpp"

#include "simulation/controllers/VO_structure/neighbor.hpp"

#include "prx/simulation/sensing/models/proximity_sensing_model.hpp"
#include "prx/simulation/systems/system.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"

#include <boost/assign/list_of.hpp>

namespace prx
{
    namespace util
    {
        class ramp_t;
    }

    namespace sim
    {
        class plant_t;
        class config_sensor_t;
        class velocity_sensor_t;
        class geometry_sensor_t;
        class simulator_t;
    }

    namespace packages
    {
        namespace crowd
        {

            /**
             * The VO sensing model is responsible for generating all the information necessary
             * for a VO_sensing_info class to create VOs from.  It reasons over vo-plants, other plants,
             * and obstacles in the simulation.  Using a config_sensor, velocity_sensor,
             * and geometry_sensor, this sensing model populates the neighborhood
             * information for each VO_sensing_info.
             *
             * @brief The sensing model used in VO-related applications
             *
             * @author Andrew Kimmel
             */

             class VO_sensing_model_t : public sim::proximity_sensing_model_t
             {
              protected:
                /** @brief Maps a pathname to a vo-controlled plant */
                util::hash_t<std::string, sim::plant_t*> vo_plants_map;
                /** @brief Flag for checking if vo-plants map has been set*/
                bool vo_plants_set;
                /** @brief Maps a pathname to a non-vo-controlled plant */
                util::hash_t<std::string, sim::plant_t*> other_plants_map;
                /** @brief Flag for checking if non-vo-plants map has been set*/
                bool other_plants_set;

                std::vector< sim::plant_t* > all_plants;

                util::hash_t<std::string, std::string> confpath_to_syspath_map;

                /** Sensor Pointers */
                sim::velocity_sensor_t* vel_sensor;
                
                /** Holding onto the simulator so we can setup plant information for the infos to check */
                sim::simulator_t* simulator;

                virtual void create_conf_to_sys_path_map(sim::simulator_t* sim);

              public:
                VO_sensing_model_t();
                virtual ~VO_sensing_model_t();

                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);
                virtual void initialize_sensors(sim::simulator_t* sim);

                virtual void update_sensed_state( const sim::state_t* new_state);

                virtual void sense();

                virtual void set_sensing_info( const std::vector<sim::sensing_info_t*>& sensing_info);
                virtual void set_all_plants( const std::vector< sim::plant_t* >& plants );
                virtual void set_sensed_VO_plants(util::hash_t<std::string, sim::plant_t*>& plants);
                virtual void set_sensed_other_plants(util::hash_t<std::string, sim::plant_t*>& plants);
            };
        }
    }
}

#endif


