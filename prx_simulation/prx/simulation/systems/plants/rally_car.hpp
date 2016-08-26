#pragma once
/**
 * @file rally_car.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2011, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors  Zakary Littlefield, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#ifndef PRX_RALLY_CAR_HPP
#define PRX_RALLY_CAR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/plants/integration_plant.hpp"

namespace prx
{
    namespace sim
    {
        class rally_car_t : public sim::integration_plant_t
        {
            public:

                rally_car_t();

                virtual ~rally_car_t();

                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

                void propagate(const double simulation_step);

                virtual void update_phys_configs(util::config_list_t& configs,unsigned& index) const;

            protected:

                virtual void update_derivative (sim::state_t* const result);

                /** Indexer for state variable : X */
                const static unsigned STATE_X;

                /** Indexer for state variable : Y */
                const static unsigned STATE_Y;

                /** Indexer for state variable : VX */
                const static unsigned STATE_VX;

                /** Indexer for state variable : VY */
                const static unsigned STATE_VY;

                /** Indexer for state variable : THETA */
                const static unsigned STATE_THETA;

                /** Indexer for state variable : THETADOT */
                const static unsigned STATE_THETADOT;

                /** Indexer for state variable : W */
                const static unsigned STATE_WF;

                /** Indexer for state variable : W */
                const static unsigned STATE_WR;

                /** Indexer for control variable STA */
                const static unsigned CONTROL_STA;

                /** Indexer for control variable TF */
                const static unsigned CONTROL_TF;

                /** Indexer for control variable TR */
                const static unsigned CONTROL_TR;

                /** Internal state & control memory */
                double _x;
                double _y;
                double _vx;
                double _vy;
                double _theta;
                double _thetadot;
                double _wf;
                double _wr;

                double _sta;
                double _tf;
                double _tr;

                /** Height value of the car. */
                double _z;
        };
    }
}

#endif //PRX_RALLY_CAR_HPP