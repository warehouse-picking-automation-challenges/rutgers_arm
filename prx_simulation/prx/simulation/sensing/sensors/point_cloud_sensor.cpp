/**
 * @file point_cloud_sensor.cpp 
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

#include "prx/simulation/sensing/sensors/point_cloud_sensor.hpp"
#include "prx/simulation/systems/obstacle.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>
#include <boost/range/adaptor/map.hpp> //adaptors

PLUGINLIB_EXPORT_CLASS(prx::sim::point_cloud_sensor_t, prx::sim::sensor_t);

namespace prx
{
    using namespace util;
    
    namespace sim
    {
        
        point_cloud_sensor_t::point_cloud_sensor_t()
        {
            updated = false;
        }

        point_cloud_sensor_t::~point_cloud_sensor_t()
        {
        }
        
        void point_cloud_sensor_t::init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader)
        {
            sensor_t::init(reader, template_reader);
            obstacle_geometry_name = parameters::get_attribute_as<std::string>("obstacle_geometry", reader, template_reader);  
            topic_name = parameters::get_attribute_as<std::string>("topic_name", reader, template_reader);
        }
        void point_cloud_sensor_t::initialize_sensor(simulator_t* sim)
        {
            std::string nm;
            boost::tie(nm, geometry) = reverse_split_path(obstacle_geometry_name);

            hash_t<std::string, system_ptr_t>& obstacles = sim->get_obstacles();

            foreach(system_ptr_t ptr, obstacles | boost::adaptors::map_values)
            {
                std::vector<std::string>* names = static_cast<obstacle_t*>(ptr.get())->get_geometries_names();
                foreach(std::string& name, *names)
                {
                    PRX_INFO_S(name);
                    if(name.compare(obstacle_geometry_name)==0)
                    {
                        obstacle = ptr;
                    }
                }
            }

            collision_checker = dynamic_cast<fcl_collision_checker_t*>(sim->get_collision_checker());
            if(collision_checker==NULL)
            {
                PRX_FATAL_S("Point Cloud collision checks are only available with FCL collision checking.");
            }

#ifdef FCL_FOUND
            sub = n.subscribe(topic_name,1,&point_cloud_sensor_t::point_cloud_callback,this);
#endif
            obstacle->update_phys_geoms(geom_map);
        }
        
        void point_cloud_sensor_t::update_data()
        {   

#ifdef FCL_FOUND
            if(updated)
            {
                collision_checker->update_model(obstacle_geometry_name,cloud_in);
                //get root config for point cloud
                // obstacle->update_root_configuration();
                updated = false;
            }
#endif
        }

#ifdef FCL_FOUND
        void point_cloud_sensor_t::point_cloud_callback(const sensor_msgs::PointCloud2& in_msg)
        {
            cloud_in = in_msg;
            updated = true;
        }
#endif
        
    }
}