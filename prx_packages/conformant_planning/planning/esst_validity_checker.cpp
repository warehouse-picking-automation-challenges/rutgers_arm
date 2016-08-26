/**
 * @file esst_validity_checker.cpp 
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

#include "planning/esst_validity_checker.hpp"

#include <numeric>
#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::packages::conformant::esst_validity_checker_t, prx::plan::validity_checker_t)

namespace prx
{
	using namespace util;
	using namespace sim;
	using namespace plan;
	namespace packages
	{
		namespace conformant
		{
			void esst_validity_checker_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader) 
			{
				validity_checker_t::init(reader,template_reader);
				collision_threshold = parameters::get_attribute_as<double>("collision_threshold",reader,template_reader,.5);
				boundary_check = parameters::get_attribute_as<bool>("boundary_check",reader,template_reader,true);
			}

			bool esst_validity_checker_t::is_valid(const trajectory_t& input)
			{
				double prob = probability_of_validity(input);
				return (1-prob)<collision_threshold;
			}

			double esst_validity_checker_t::probability_of_validity(const trajectory_t& input)
			{
				bool crappy = false;


				if(world_model->is_phys_engine())
				{
                	const particle_trajectory_t& particle_traj = dynamic_cast<const particle_trajectory_t&>(input);

					for(unsigned i=0;i<num_states;i++)
					{
						if(!collided[i])
						{
							collided[i] = particle_traj.collisions[i];
						}
					}
				}

		    	if(boundary_check)
		    	{
					for (trajectory_t::const_iterator iter = input.begin(); iter != input.end(); ++iter)
					{
						for(unsigned i=0;i<num_states;i++)
						{
							state_t* particle = particle_space->get_particle(i,*iter);
							if(!collided[i])
							{
								collided[i] = !(world_model->get_state_space()->satisfies_bounds(particle,false));
							}
							if(!collided[i])
							{
								double x,y,z;
								x = particle->at(0);
								y = particle->at(1);
								z = particle->at(2);
								if( (10<x && x<20) && (-10<y && y<10) )
								{
									collided[i] = true;
								}
							}
						}
						double sum = std::accumulate(collided.begin(),collided.end(),0);
						if(sum/num_states > collision_threshold)
						{
							crappy = true;
							break;
						}
					}
				}
				

				if(!crappy && !world_model->is_phys_engine())
				{
					for (trajectory_t::const_iterator iter = input.begin(); iter != input.end(); ++iter)
					{
						for(unsigned i=0;i<num_states;i++)
						{
							state_t* particle = particle_space->get_particle(i,*iter);
							if(!collided[i])
							{
								collided[i] = !is_valid(particle);
							}
						}
						double sum = std::accumulate(collided.begin(),collided.end(),0);
						if(sum/num_states > collision_threshold)
						{
							crappy = true;
							break;
						}
					}
				}
			    double sum = std::accumulate(collided.begin(),collided.end(),0);
			    if(crappy)
			    	return 0;
				return 1.0 - sum/num_states;
			}
	        void esst_validity_checker_t::link_model(world_model_t* model)
	        {
	            world_model = model;
	        }

			bool esst_validity_checker_t::is_valid(const state_t* point)
			{
				return world_model->valid_state(point);
			}
		}
	}
}
