/**
 * @file forward_car.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2016, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Colin Rennie, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef LEARNED_FORWARD_CAR_HPP
#define LEARNED_FORWARD_CAR_HPP

#include "prx/utilities/definitions/defs.hpp"
// #include "prx/simulation/systems/plants/plant.hpp"

 
template <class T> 
class network_t;

namespace prx
{
    namespace learn
    {
    	class forward_car_model_t
    	{
    	public:
    		forward_car_model_t();
    		~forward_car_model_t();

    		void forward_simulate(std::vector<double>& input, std::vector<double>& output);
    	protected:
    		network_t<std::vector<double> >* toy_car_mlp;
    	};

    }
}

#endif