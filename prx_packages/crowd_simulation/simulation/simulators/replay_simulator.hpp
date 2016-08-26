/**
 * @file replay_simulator.hpp
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

#ifndef PRX_REPLAY_SIMULATOR_HPP
#define	PRX_REPLAY_SIMULATOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/simulation/simulators/default_simulator.hpp"

#include "simulation/plants/pedestrian.hpp"

#include <fstream>
#include <iostream>


namespace prx
{
    namespace packages
    {
        namespace crowd
        {
            class replay_simulator_t : public sim::default_simulator_t
            {

              public:
                replay_simulator_t();
                virtual ~replay_simulator_t();
                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader);
                virtual void propagate_and_respond();

                virtual void add_obstacle(const std::string& name, sim::system_ptr_t obstacle);

              private:
                const util::parameter_reader_t* pedestrian_initializer;

                unsigned num_pedestrians;
              	std::vector<pedestrian_t*> pedestrians;
                std::vector<const util::space_t*> pedestrian_spaces;
                std::vector< std::vector< std::vector<double> > > trajectories;
                
                std::vector<double> state_pos;
                int frame_id;
                int max_frame_id;

                void string_to_pos(std::string str, std::vector<double>& pos);
            };
        }

    }
}

#endif
