/**
 * @file default_simulator.cpp
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

#include "prx/simulation/simulators/default_simulator.hpp"
#include <pluginlib/class_list_macros.h>
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/simulation/collision_checking/collision_list.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <boost/tuple/tuple.hpp>
#include <boost/range/adaptor/map.hpp> // boost::tie

PLUGINLIB_EXPORT_CLASS(prx::sim::default_simulator_t, prx::sim::simulator_t)

namespace prx
{
    using namespace util;

    namespace sim
    {

        default_simulator_t::default_simulator_t()
        {
            prev_state = NULL;
            state = NULL;
            collision_response = true;
            collision_detection = true;
        }

        default_simulator_t::~default_simulator_t()
        {
            if( prev_state != NULL )
                state_space->free_point(prev_state);
            if( state != NULL )
                state_space->free_point(state);
        }

        void default_simulator_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {

            simulator_t::init(reader, template_reader);
            
            /** Determines if collision detection and response is dne by the simulator */
            collision_response = parameters::get_attribute_as<bool>("collision_response", reader, template_reader);
            collision_detection = parameters::get_attribute_as<bool>("collision_detection", reader, template_reader);
            
            /** If response has been set, detection is automatically set*/
            if (collision_response)
                collision_detection = true;
            
            prev_state = state_space->alloc_point();
            state = state_space->alloc_point();
            
        }

        void default_simulator_t::propagate_and_respond()
        {
            if (collision_response && collision_detection)
            {
                state_space->copy_to_point(prev_state);

                //propagate all the systems in the simulator.
                simulator_t::propagate(simulation::simulation_step);
                collision_list_t* in_collision_list = simulator_t::get_colliding_bodies();
                bool collided = false;

                if( in_collision_list->size() > 0 )
                {
                    collided = true;
                    state_space->copy_to_point(state);
                }

                foreach(collision_pair_t p, in_collision_list->get_body_pairs())
                {
                    set_previous_state(p.first);
                    set_previous_state(p.second);
                    PRX_DEBUG_COLOR("in collision : " << p.first << "  -> " << p.second, PRX_TEXT_MAGENTA);
                }

                if( collided )
                    push_state(state);
                
            }
            else if (collision_detection)
            {
                //propagate all the systems in the simulator.
                simulator_t::propagate(simulation::simulation_step);

                collision_checker->in_collision();
            }
            else
            {
                simulator_t::propagate(simulation::simulation_step);
            }
            
        }

        void default_simulator_t::set_previous_state(const std::string& path)
        {
            std::string name_to_check;
            std::string name;
            std::string subpath;


            // remove the /geometry_name from the end of the path;
            boost::tie(name_to_check, subpath) = reverse_split_path(path);
            if( plants.find(name_to_check) != plants.end() )
            {
                std::pair<unsigned, unsigned> interval = system_intervals[(system_t*)plants[name_to_check]];
                for( unsigned i = interval.first; i < interval.second; i++ )
                {
                    state->at(i) = prev_state->at(i);
                }
            }
        }
        
        bool default_simulator_t::in_collision()
        {
            if (!collision_detection)
            {
                PRX_WARN_S("No Collision Simulator cannot call in_collision!");
                return false;
            }
            else
            {
                return simulator_t::in_collision();
            }
        }

        void default_simulator_t::link_collision_list(collision_list_t* collision_list)
        {
            if (!collision_detection)
            {
                PRX_WARN_S("No Collision Simulator does not need a collision list!");
            }
            else
            {
                simulator_t::link_collision_list(collision_list);
            }
        }

        collision_list_t* default_simulator_t::get_colliding_bodies()
        {
            if (!collision_detection)
            {
                PRX_WARN_S("No Collision Simulator does not have collision list for colliding_bodies!");
                return NULL;
            }
            else
            {
                return simulator_t::get_colliding_bodies();
            }
        }
        
        void default_simulator_t::set_collision_detection(bool detects)
        {
            collision_detection  = detects;
        }
        
        void default_simulator_t::set_collision_response(bool responds)
        {
            collision_response = responds;
        }

        bool default_simulator_t::simulator_collision_detects()
        {
            return collision_detection;
        }
        
        bool default_simulator_t::simulator_collision_responds()
        {
            return collision_response;
        }
    }
}

