/**
 * @file forward_car.cpp 
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


#include <ros/ros.h>
#include <ros/package.h>

#include "prx/learn/forward_car.hpp"
#include "prx/network_t.hpp"

namespace prx
{
	namespace learn
	{
		forward_car_model_t::forward_car_model_t()
		{
			std::string model_path;
			std::string weights_path;
			std::string mean_file;
			std::string min_file;
			std::string max_file;
            const std::string ROOT_SAMPLE = "/home/colin/Repos/prx_ws/src/prx_learn/external/caffe/examples/toy_car/PhysicsLearner/";
            model_path = ROOT_SAMPLE + "toycar_2fc_deploy.prototxt";
            weights_path = ROOT_SAMPLE + "2fc_iter_20001.caffemodel";
            mean_file = ROOT_SAMPLE + "toycar_mean.binaryproto";
            min_file = ROOT_SAMPLE + "toycar_min.binaryproto";
            max_file = ROOT_SAMPLE + "toycar_max.binaryproto";
            toy_car_mlp = new network_t<std::vector<double> >(model_path, weights_path, mean_file, min_file, max_file);
		}
		forward_car_model_t::~forward_car_model_t()
		{
			delete toy_car_mlp;
		}
    	void forward_car_model_t::forward_simulate(std::vector<double>& input, std::vector<double>& output)
    	{
    		output = toy_car_mlp->forward_pass(input);
    	}
	}
}