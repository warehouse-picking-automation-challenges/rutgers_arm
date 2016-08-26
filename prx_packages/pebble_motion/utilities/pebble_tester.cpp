/**
 * @file pebble_tester.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2012, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Athanasios Krontiris, Andrew Dobson, Andrew Kimmel, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "utilities/pebble_tester.hpp"
#include <boost/range/adaptor/map.hpp>


namespace prx
{
    using namespace util;
    
    namespace packages
    {
        namespace pebble_motion
        {

            pluginlib::ClassLoader<pebble_tester_t> pebble_tester_t::loader("prx_utilities", "prx::packages::pebble_motion::pebble_tester_t");

            pluginlib::ClassLoader<pebble_tester_t>& pebble_tester_t::get_loader()
            {
                return loader;
            }

            pebble_tester_t::~pebble_tester_t() { }

            void pebble_tester_t::setup(pebble_assignment_t& start_assign, pebble_assignment_t& target_assign, int num_robots, const space_t* space, distance_metric_t* distance_metric)
            {
                PRX_INFO_S("Setup pebble_tester");
                state_space = space;
                s_assignment = start_assign;
                t_assignment = target_assign;
                metric = distance_metric;
                k = num_robots;

            }

            void pebble_tester_t::reset()
            {
                PRX_WARN_S("Reset function has not being implemented in pebble tester");
            }

            void pebble_tester_t::print_assignment(const pebble_assignment_t& assign) const
            {
                PRX_INFO_S("assign size : " << assign.size());

                foreach(undirected_vertex_index_t v, assign.assignments | boost::adaptors::map_keys)
                {
                    PRX_DEBUG_S("place(" << v << "): " << state_space->print_point(g->get_vertex_as<undirected_node_t > (v)->point, 2) << "   -   robot: " << assign.get_robot(v));
                }
            }

        }
    }
}
